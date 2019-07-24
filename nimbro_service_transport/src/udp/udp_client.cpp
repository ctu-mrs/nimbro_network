// UDP service client
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_client.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <netdb.h>

#include <sstream>

#include <ros/names.h>
#include <ros/package.h>

#include <nimbro_service_transport/ServiceStatus.h>

#include "protocol.h"
#include "../common.h"

namespace nimbro_service_transport
{

namespace
{

class CallbackHelper : public ros::ServiceCallbackHelper {
public:
  CallbackHelper(const std::string& name, UDPClient* client) : m_name(name), m_client(client) {
  }

  virtual bool call(ros::ServiceCallbackHelperCallParams& params) {
    ROS_INFO("[%s]: Calling.", m_name.c_str());
    ros::Time     call_time_start = ros::Time::now();
    bool          call_result     = m_client->call(m_name, params);
    ros::Duration call_time       = ros::Time::now() - call_time_start;
    if (call_result) {
      ROS_INFO("[%s]: Call was successful (dt = %.4f sec).", m_name.c_str(), call_time.toSec());
    } else {
      ROS_ERROR("[%s]: Call error (dt = %.4f sec).", m_name.c_str(), call_time.toSec());
    }
    return call_result;
  }

private:
  std::string m_name;
  UDPClient*  m_client;
};

}  // namespace


UDPClient::UDPClient(const std::string& robot_hostname, const std::string& robot_addr, std::map<std::string, std::vector<std::string>>& services,
                     const double& response_timeout, const double& call_timeout, const int& call_repeats, const int& remote_port)
    : m_nh("~"),
      m_fd(-1),
      m_counter(0),
      m_remote_hostname(robot_hostname),
      m_remote(robot_addr),
      m_response_timeout(response_timeout),
      m_call_timeout(call_timeout),
      m_call_repeats(call_repeats),
      m_remotePort(remote_port) {

  std::string portString = boost::lexical_cast<std::string>(m_remotePort);

  // Get local host name for visualization messages
  char hostnameBuf[256];
  gethostname(hostnameBuf, sizeof(hostnameBuf));
  hostnameBuf[sizeof(hostnameBuf) - 1] = 0;

  m_host = hostnameBuf;

  // Resolve remote address
  addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_socktype = SOCK_DGRAM;

  addrinfo* info;

  if (getaddrinfo(m_remote.c_str(), portString.c_str(), &hints, &info) != 0 || !info) {
    std::stringstream ss;
    ss << "getaddrinfo() failed for host '" << m_remote << "': " << strerror(errno);
    throw std::runtime_error(ss.str());
  }

  m_fd = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
  if (m_fd < 0) {
    std::stringstream ss;
    ss << "Could not create socket: " << strerror(errno);
    throw std::runtime_error(ss.str());
  }

  if (connect(m_fd, info->ai_addr, info->ai_addrlen) != 0) {
    std::stringstream ss;
    ss << "Could not connect socket: " << strerror(errno);
    throw std::runtime_error(ss.str());
  }

  freeaddrinfo(info);

  // Initialize & advertise the list of services
  for (auto const& serv_type : services) {
    for (const std::string& topic_name : services[serv_type.first]) {
      ROS_INFO("Advertising service [%s] with type [%s].", topic_name.c_str(), serv_type.first.c_str());
      const std::string md5sum = getServiceMD5(serv_type.first);
      if (md5sum.empty()) {
        break;
      }

      ros::AdvertiseServiceOptions ops;
      ops.callback_queue = 0;
      ops.datatype       = serv_type.first;
      ops.md5sum         = md5sum;
      ops.helper         = boost::make_shared<CallbackHelper>(topic_name, this);
      ops.req_datatype   = ops.datatype + "Request";
      ops.res_datatype   = ops.datatype + "Response";
      ops.service        = topic_name;

      ros::ServiceServer srv = m_nh.advertiseService(ops);
      m_servers.push_back(srv);
    }
  }

  std::string topic_prefix;
  m_nh.param("topic_prefix", topic_prefix, std::string(""));

  std::stringstream topic_name;
  if (!topic_prefix.empty()) {
    topic_name << "/" << topic_prefix << "/network/service_status";
  } else {
    topic_name << "/network/service_status";
  }
  m_pub_status = m_nh.advertise<ServiceStatus>(topic_name.str(), 10);

  m_thread = std::make_unique<boost::thread>(boost::bind(&UDPClient::run, this));
  ROS_INFO("Service client initialized.");
}

UDPClient::~UDPClient() {
  close(m_fd);
}

uint8_t UDPClient::acquireCounterValue() {
  boost::unique_lock<boost::mutex> lock(m_mutex);
  return m_counter++;
}

bool UDPClient::call(const std::string& name, ros::ServiceCallbackHelperCallParams& params) {

  std::vector<uint8_t> buffer(sizeof(ServiceCallRequest) + name.length() + 4 + params.request.num_bytes);

  ServiceCallRequest* header = reinterpret_cast<ServiceCallRequest*>(buffer.data());
  header->timestamp          = ros::Time::now().toNSec();
  header->counter            = acquireCounterValue();
  header->name_length        = name.length();
  header->request_length     = params.request.num_bytes + 4;

  memcpy(buffer.data() + sizeof(ServiceCallRequest), name.c_str(), name.length());
  memcpy(buffer.data() + sizeof(ServiceCallRequest) + name.length(), &params.request.num_bytes, 4);
  memcpy(buffer.data() + sizeof(ServiceCallRequest) + name.length() + 4, params.request.buf.get(), params.request.num_bytes);

  RequestRecord record;
  record.timestamp          = header->timestamp();
  record.counter            = header->counter;
  record.response.num_bytes = 0;

  boost::unique_lock<boost::mutex> lock(m_mutex);

  publishStatus(name, header->counter, ServiceStatus::STATUS_IN_PROGRESS);

  auto it = m_requests.insert(m_requests.end(), &record);

  bool gotAck = false;
  for (int i = 0; i < m_call_repeats; i++) {
    if (send(m_fd, buffer.data(), buffer.size(), 0) != (int)buffer.size()) {
      ROS_ERROR("[%s] Could not send UDP data: %s", m_remote_hostname.c_str(), strerror(errno));
      publishStatus(name, header->counter, ServiceStatus::STATUS_CONNECTION_ERROR);
      return false;
    }
    boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(m_call_timeout * 1000);
    gotAck                           = record.cond_msg_acknowledgement_received.timed_wait(lock, timeout);
    if (gotAck) {
      break;
    }
  }

  if (!gotAck) {
    ROS_ERROR("[%s]: Have not received Ack in timeout!", name.c_str());
    return false;
  }


  boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(m_call_timeout * 1000);
  bool                     gotMsg  = record.cond_response_received.timed_wait(lock, timeout, [&]() { return record.response.num_bytes != 0; });
  m_requests.erase(it);

  if (gotMsg) {
    params.response = record.response;
    publishStatus(name, header->counter, ServiceStatus::STATUS_FINISHED_SUCCESS);
    return true;
  } else {
    ROS_ERROR("[%s]: Have not received the response in timeout!", name.c_str());
    publishStatus(name, header->counter, ServiceStatus::STATUS_TIMEOUT);
    return false;
  }
}


void UDPClient::step() {
  timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 500 * 1000;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  int ret = select(m_fd + 1, &fds, 0, 0, &timeout);
  if (ret < 0) {
    // Silently ignore EINTR, EAGAIN
    if (errno == EINTR || errno == EAGAIN)
      return;

    std::stringstream ss;
    ss << "Could not select(): " << strerror(errno);
    throw std::runtime_error(ss.str());
  }

  if (ret > 0) {
    handlePacket();
  }
}

void UDPClient::handlePacket() {
  int packetSize;

  while (1) {
    if (ioctl(m_fd, FIONREAD, &packetSize) != 0) {
      if (errno == EAGAIN || errno == EINTR)
        continue;
      else {
        perror("FIONREAD");
        break;
      }
    }

    m_recvBuf.resize(packetSize);
    break;
  }

  int bytes = recv(m_fd, m_recvBuf.data(), m_recvBuf.size(), 0);
  if (bytes < 0) {
    if (errno == ECONNREFUSED) {
      ROS_ERROR("[%s], Got negative ICMP reply. Is the UDP server running?", m_remote_hostname.c_str());
      return;
    }

    std::stringstream ss;
    ss << "Could not recv(): " << strerror(errno);
    throw std::runtime_error(ss.str());
  }

  if (bytes < (int)sizeof(ServiceCallAcknowledgement)) {
    ROS_ERROR("[%s] Short response packet, ignoring...", m_remote_hostname.c_str());
    return;
  }

  // parse acknowledgement msg
  if (bytes == (int)sizeof(ServiceCallAcknowledgement)) {

    const auto* resp = reinterpret_cast<const ServiceCallAcknowledgement*>(m_recvBuf.data());

    boost::unique_lock<boost::mutex> lock(m_mutex);
    for (auto it = m_requests.begin(); it != m_requests.end(); ++it) {
      if (resp->timestamp() == (*it)->timestamp && resp->counter == (*it)->counter) {
        (*it)->cond_msg_acknowledgement_received.notify_one();
        return;
      }
    }
    ROS_WARN("[%s] Received unexpected UDP service packet answer, ignoring", m_remote_hostname.c_str());
    return;
  }

  if (bytes < (int)sizeof(ServiceCallResponse)) {
    ROS_ERROR("[%s] Short response packet, ignoring...", m_remote_hostname.c_str());
    return;
  }
  // now it can be only ServiceCallResponse
  const auto* resp = reinterpret_cast<const ServiceCallResponse*>(m_recvBuf.data());

  if (resp->response_length() + sizeof(ServiceCallResponse) > (size_t)bytes) {
    ROS_ERROR("[%s] Response is longer than packet...", m_remote_hostname.c_str());
    return;
  }


  // send acknowledgement for response
  std::vector<uint8_t> buffer(sizeof(ServiceCallAcknowledgement));

  ServiceCallAcknowledgement* header = reinterpret_cast<ServiceCallAcknowledgement*>(buffer.data());

  header->timestamp = resp->timestamp;
  header->counter   = resp->counter;

  boost::unique_lock<boost::mutex> lock(m_mutex);

  // send acknowledgement
  if (send(m_fd, buffer.data(), buffer.size(), 0) != (int)buffer.size()) {
    ROS_ERROR("[%s] Could not send ack UDP data: %s", m_remote_hostname.c_str(), strerror(errno));
    return;
  }

  // parse response
  for (auto it = m_requests.begin(); it != m_requests.end(); ++it) {
    if (resp->timestamp() == (*it)->timestamp && resp->counter == (*it)->counter) {
      boost::shared_array<uint8_t> data(new uint8_t[resp->response_length()]);
      memcpy(data.get(), m_recvBuf.data() + sizeof(ServiceCallResponse), resp->response_length());
      (*it)->response = ros::SerializedMessage(data, resp->response_length());

      (*it)->cond_msg_acknowledgement_received.notify_one();
      (*it)->cond_response_received.notify_one();
      return;
    }
  }

  ROS_WARN("[%s] Received unexpected UDP service packet response, ignoring", m_remote_hostname.c_str());
}  // namespace nimbro_service_transport

void UDPClient::run() {
  while (ros::ok()) {
    step();
  }
}

void UDPClient::publishStatus(const std::string& service, uint32_t call, uint8_t status) {
  ServiceStatus msg;
  msg.host        = m_host;
  msg.remote      = m_remote;
  msg.remote_port = m_remotePort;

  msg.call_id = call;
  msg.service = service;

  msg.status = status;

  m_pub_status.publish(msg);
}

}  // namespace nimbro_service_transport

int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_client");
  ros::NodeHandle nh("~");

  ROS_INFO("Loading parameters: ");

  // Get local hostname
  char hostname_buf[256];
  gethostname(hostname_buf, sizeof(hostname_buf));
  /* hostname_buf[sizeof(hostname_buf) - 1] = 0; */
  std::string hostname = hostname_buf;

  std::vector<std::string> robot_names;
  nh.getParam("network/robot_names", robot_names);

  int destination_port;
  nh.param("destination_port", destination_port, 5000);

  double call_timeout;
  nh.param("call_timeout", call_timeout, double(0.1));

  int call_repeats;
  nh.param("call_repeats", call_repeats, int(3));

  double response_timeout;
  nh.param("response_timeout", response_timeout, double(3));

  // Initialize & advertise the list of services
  XmlRpc::XmlRpcValue service_list;
  nh.getParam("services", service_list);
  ROS_ASSERT(service_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // exclude this robot from the list
  robot_names.erase(std::remove(robot_names.begin(), robot_names.end(), hostname), robot_names.end());

  // printing destination_port parameter
  std::cout << "    parameter 'destination_port': " << destination_port << std::endl;
  // printing call_timeout parameter
  std::cout << "    parameter 'call_timeout': " << call_timeout << std::endl;
  // printing call_repeats parameter
  std::cout << "    parameter 'call_repeats': " << call_repeats << std::endl;
  // printing response_timeout parameter
  std::cout << "    parameter 'response_timeout': " << response_timeout << std::endl;

  // printing robot_names list
  std::string tmp_print_list;
  for (unsigned long i = 0; i < robot_names.size() - 1; i++) {
    tmp_print_list.append(robot_names.at(i) + std::string(", "));
  }
  tmp_print_list.append(robot_names.back());
  std::cout << "  parameter 'network/robot_names': " << tmp_print_list.c_str() << std::endl;

  // printing service list
  std::cout << "    parameter 'services': ";
  for (int32_t i = 0; i < service_list.size(); ++i) {
    XmlRpc::XmlRpcValue& entry = service_list[i];
    ROS_ASSERT(entry.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(entry.hasMember("name"));
    ROS_ASSERT(entry.hasMember("type"));

    std::string name        = (std::string)entry["name"];
    std::string type        = (std::string)entry["type"];
    std::string indentation = "                          ";
    if (i != 0) {
      std::cout << indentation;
    }
    std::cout << "-- name: " << name << std::endl;
    std::cout << indentation << " - type: " << type << std::endl;
  }
  std::cout << std::endl;

  // sanity check
  if (robot_names.empty()) {
    ROS_ERROR("Robot names list is empty or contains only the hostname of this device.!");
    ros::shutdown();
    return -1;
  }

  if (service_list.size() == 0) {
    ROS_ERROR("Service list is empty!");
    ros::shutdown();
    return -1;
  }


  // resolve hostnames
  std::map<std::string, std::string> robot_name_map;  // map(key=robot_name, value=robot_ip_addr)

  struct hostent* host_entry;
  bool            tmp_valid = true;
  for (const std::string& tmp_name : robot_names) {
    // To retrieve host information
    host_entry = gethostbyname(tmp_name.c_str());
    if (host_entry == NULL) {
      ROS_ERROR("Cannot resolve hostname '%s'. It is not specified in '/etc/hosts'.", tmp_name.c_str());
      tmp_valid = false;
      continue;
    }
    // To convert an Internet network
    // address into ASCII string
    const std::string resolved_ip = inet_ntoa(*((struct in_addr*)host_entry->h_addr_list[0]));
    robot_name_map[tmp_name]      = resolved_ip;
  }
  if (!tmp_valid) {
    ros::shutdown();
    return -1;
  }

  // printing resolved robot list
  ROS_INFO("Resolved 'robot_names':");
  for (auto const& [name, ip] : robot_name_map) {
    std::cout << "                 " << name << "\t\t ---> \t\t" << ip << std::endl;
  }

  std::map<std::string, std::map<std::string, std::vector<std::string>>> robot_services;  // map(key=robot_name, value=map(key=topic_type, value=topic_name))

  // parse service list
  for (int32_t i = 0; i < service_list.size(); ++i) {
    XmlRpc::XmlRpcValue& entry = service_list[i];

    const std::string service_name = (std::string)entry["name"];
    const std::string type         = (std::string)entry["type"];

    // Find service from all robots - must start with '/*/'
    if (service_name.compare(0, 3, "/*/") == 0) {
      for (int robot_idx = 0; robot_idx < robot_names.size(); robot_idx++) {
        std::string       service_name_copy = service_name;
        const std::string robot_name        = robot_names.at(robot_idx);
        service_name_copy.replace(service_name_copy.begin() + 1, service_name_copy.begin() + 2, robot_name);
        robot_services[robot_name_map[robot_name]][type].push_back(service_name_copy);
      }
    }
    // Catch further regex expressions
    else if (service_name.find("*") != std::string::npos) {
      ROS_ERROR("Regex expressions [%s] are not allowed, with the exception of regex robot names (i.e. solely '/*/topic_name' is allowed).",
                service_name.c_str());
    }
    // Write down services from specified robots
    else {
      bool specific = false;
      for (int robot_idx = 0; robot_idx < robot_names.size(); robot_idx++) {
        const std::string robot_name = robot_names.at(robot_idx);
        if (service_name.find(robot_name) != std::string::npos) {
          robot_services[robot_name_map[robot_name]][type].push_back(service_name);
          specific = true;
          break;
        }
      }
      // If service is not namespaced, we need address specifications
      if (!specific) {
        // Catch unknown robot name
        if (service_name.find("robot") != std::string::npos) {
          ROS_ERROR("Given robot of service [%s] is unknown.", service_name.c_str());
        }
        // Catch non-namespace elements without 'robot' tag
        else if (!entry.hasMember("robot")) {
          ROS_ERROR("Service [%s] has not unique namespace and its target server is unspecified. Specify its server by adding 'robot' tag.",
                    service_name.c_str());
        } else {
          const std::string robot_name = (std::string)entry["robot"];
          // Catch unknown robot name given by tag 'robot'
          if (robot_name_map.count(robot_name) == 0) {
            ROS_ERROR("Unknown robot name %s. Ommiting.", robot_name.c_str());
          } else {
            robot_services[robot_name_map[robot_name]][type].push_back(service_name);
          }
        }
      }
    }
  }

  std::vector<std::unique_ptr<nimbro_service_transport::UDPClient>> clients;
  for (auto const& robot_name : robot_name_map) {
    ROS_INFO("Initializing connection to server: %s", robot_name.second.c_str());
    clients.push_back(std::make_unique<nimbro_service_transport::UDPClient>(robot_name.first, robot_name.second, robot_services[robot_name.second],
                                                                            response_timeout, call_timeout, call_repeats, destination_port));
  }

  ros::MultiThreadedSpinner spinner(robot_names.size() + 1);
  spinner.spin();

  return 0;
}
