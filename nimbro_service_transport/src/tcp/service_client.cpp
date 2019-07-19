// Client side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "service_client.h"
#include "protocol.h"
#include "../common.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>

#define DATA_DEBUG 0
#define STRING_EQUAL 0

#include <linux/version.h>

#include <ros/names.h>
#include <ros/package.h>

#include <nimbro_service_transport/ServiceStatus.h>

namespace nimbro_service_transport
{

class IOException : public std::runtime_error {
public:
  explicit IOException() : std::runtime_error("IO exception") {
  }
};


static int sureRead(int fd, void* dest, size_t size) {
  size_t readBytes = 0;
  while (size != 0) {
    int ret = read(fd, dest, size);
    if (ret <= 0) {
      perror("Could not read()");
      throw IOException();
    }

    size -= ret;
    dest = ((uint8_t*)dest) + ret;
    readBytes += ret;
  }

  return readBytes;
}

static void sureWrite(int fd, const void* src, size_t size) {
  if (write(fd, src, size) != (int)size) {
    perror("Could not write()");
    throw IOException();
  }
}

class CallbackHelper : public ros::ServiceCallbackHelper {
public:
  CallbackHelper(const std::string& name, ServiceClient* client) : m_name(name), m_client(client) {
  }

  virtual bool call(ros::ServiceCallbackHelperCallParams& params) {
    return m_client->call(m_name, params);
  }

private:
  std::string    m_name;
  ServiceClient* m_client;
};

ServiceClient::ServiceClient(const std::string& robot_addr, std::map<std::string, std::vector<std::string>>& services)
    : m_nh("~"), m_currentCallID(0), m_fd(-1), m_remote(robot_addr) {
  m_nh.param("port", m_remotePort, 6050);

  std::string portString = boost::lexical_cast<std::string>(m_remotePort);

  // Get local host name for visualization messages
  char hostnameBuf[256];
  gethostname(hostnameBuf, sizeof(hostnameBuf));
  hostnameBuf[sizeof(hostnameBuf) - 1] = 0;

  m_host = hostnameBuf;

  // Resolve remote address
  addrinfo hint;
  memset(&hint, 0, sizeof(hint));
  hint.ai_socktype = SOCK_STREAM;

  addrinfo* info = 0;

  if (getaddrinfo(m_remote.c_str(), portString.c_str(), &hint, &info) != 0 || !info) {
    std::stringstream ss;
    ss << "Could not resolve server: " << strerror(errno);
    throw std::runtime_error(ss.str());
  }

  if (info->ai_addrlen > sizeof(m_addr))
    throw std::runtime_error("Invalid address length");

  memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
  m_addrLen = info->ai_addrlen;

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

  m_pub_status = m_nh.advertise<ServiceStatus>("/network/service_status", 10);

  ROS_INFO("Service client initialized.");
}

ServiceClient::~ServiceClient() {
}

bool ServiceClient::call(const std::string& name, ros::ServiceCallbackHelperCallParams& params) {
  m_currentCallID++;

  protocol::ServiceCallRequest req;
  req.name_length    = name.length();
  req.request_length = params.request.num_bytes + 4;

#if DATA_DEBUG
  ROS_INFO("Sending service call request with %d bytes of data", params.request.num_bytes);
  for (int i = 0; i < params.request.num_bytes; ++i)
    ROS_INFO(" %d: 0x%02X (%c)", i, params.request.buf.get()[i], params.request.buf.get()[i]);
#endif

  publishStatus(name, ServiceStatus::STATUS_IN_PROGRESS);

  uint8_t failure_reason = 0;

  for (int tries = 0; tries < 10; ++tries) {
    if (m_fd < 0) {
      m_fd = socket(AF_INET, SOCK_STREAM, 0);
      if (m_fd < 0) {
        perror("Could not create socket");
        throw std::runtime_error("socket error");
      }

      if (connect(m_fd, (const sockaddr*)&m_addr, m_addrLen) != 0) {
        ROS_WARN("ServiceClient could not connect to server: %s", strerror(errno));
        sleep(1);

        close(m_fd);
        m_fd           = -1;
        failure_reason = ServiceStatus::STATUS_CONNECTION_ERROR;
        continue;
      }

#ifdef TCP_USER_TIMEOUT
      int timeout = 8000;
      if (setsockopt(m_fd, SOL_TCP, TCP_USER_TIMEOUT, &timeout, sizeof(timeout)) != 0) {
        ROS_ERROR("Could not set TCP_USER_TIMEOUT: %s", strerror(errno));
        return false;
      }
#else
      ROS_WARN("Not setting TCP_USER_TIMEOUT");
#endif
    }

    try {
      sureWrite(m_fd, &req, sizeof(req));
      sureWrite(m_fd, name.c_str(), name.length());
      sureWrite(m_fd, &params.request.num_bytes, 4);  // FIXME: Not portable
      sureWrite(m_fd, params.request.buf.get(), params.request.num_bytes);

      protocol::ServiceCallResponse resp;
      sureRead(m_fd, &resp, sizeof(resp));

      boost::shared_array<uint8_t> data(new uint8_t[resp.response_length()]);
      sureRead(m_fd, data.get(), resp.response_length());

      params.response = ros::SerializedMessage(data, resp.response_length());

      publishStatus(name, ServiceStatus::STATUS_FINISHED_SUCCESS);

      return true;
    }
    catch (IOException&) {
      failure_reason = ServiceStatus::STATUS_TIMEOUT;
      close(m_fd);
      m_fd = -1;
    }
  }

  publishStatus(name, failure_reason);
  return false;
}

void ServiceClient::publishStatus(const std::string& service, uint8_t status) {
  ServiceStatus msg;
  msg.host        = m_host;
  msg.remote      = m_remote;
  msg.remote_port = m_remotePort;

  msg.call_id = m_currentCallID;
  msg.service = service;

  msg.status = status;

  m_pub_status.publish(msg);
}

}  // namespace nimbro_service_transport

int main(int argc, char** argv) {
  ros::init(argc, argv, "service_client");
  ros::NodeHandle nh("~");

  ROS_INFO("Loading parameters: ");

  // Get local hostname
  char hostname_buf[256];
  gethostname(hostname_buf, sizeof(hostname_buf));
  /* hostname_buf[sizeof(hostname_buf) - 1] = 0; */
  std::string hostname = hostname_buf;

  std::vector<std::string> robot_names;
  nh.getParam("network/robot_names", robot_names);

  // Initialize & advertise the list of services
  XmlRpc::XmlRpcValue service_list;
  nh.getParam("services", service_list);
  ROS_ASSERT(service_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // exclude this robot from the list
  robot_names.erase(std::remove(robot_names.begin(), robot_names.end(), hostname), robot_names.end());

  // printing robot_name list
  std::string tmp_print_list;
  for (unsigned long i = 0; i < robot_names.size() - 1; i++) {
    tmp_print_list.append(robot_names.at(i) + std::string(", "));
  }
  tmp_print_list.append(robot_names.back());
  std::cout << "    parameter 'network/robot_names': " << tmp_print_list.c_str() << std::endl;

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
    ROS_ERROR("robot names list is empty or contains only the hostname of this device.!");
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
    robot_name_map[tmp_name]        = resolved_ip;
  }
  if (!tmp_valid) {
    ros::shutdown();
    return -1;
  }

  // printing resolved robot list
  ROS_INFO("Resolved 'robot_names':");
  for (auto const& [name, ip] : robot_name_map) {
    std::cout << "                 " << name << "\t\t---> \t\t" << ip << std::endl;
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
        std::string service_name_copy = service_name;
        const std::string robot_name = robot_names.at(robot_idx);
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

  std::vector<std::unique_ptr<nimbro_service_transport::ServiceClient>> clients;
  for (auto const& robot_addr : robot_services) {
    ROS_INFO("Initializing connection to server: %s", robot_addr.first.c_str());
    clients.push_back(std::make_unique<nimbro_service_transport::ServiceClient>(robot_addr.first, robot_services[robot_addr.first]));
  }

  ros::MultiThreadedSpinner spinner(robot_names.size()+1);
  spinner.spin();
  return 0;
}
