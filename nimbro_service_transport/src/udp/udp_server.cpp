// UDP service server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_server.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <netinet/in.h>
#include <netinet/udp.h>

#include <arpa/inet.h>

#include <stdexcept>

#include <topic_tools/shape_shifter.h>

#include "protocol.h"

#include <mrs_lib/ParamLoader.h>

static void errnoError(const std::string& msg) {
  std::stringstream ss;
  ss << msg << ": " << strerror(errno);
  throw std::runtime_error(ss.str());
}

namespace nimbro_service_transport
{

UDPServer::UDPServer() : m_nh("~"), m_buffer(1024) {

  // | ------------------- loading parameters ------------------- |
  mrs_lib::ParamLoader param_loader(m_nh, "SERVICE_SERVER");
  ROS_INFO("[SERVICE_SERVER]: Loading parameters: ");

  int port;
  param_loader.load_param("port", port, 5000);
  param_loader.load_param("call_timeout", m_call_timeout, double(0.1));
  param_loader.load_param("call_repeats", m_call_repeats, int(3));

  // | ----------------------- finish loading ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[SERVICE_SERVER]: Could not load all parameters!");
    ros::shutdown();
    return;
  }

  m_fd = socket(AF_INET6, SOCK_DGRAM, 0);
  if (m_fd < 0)
    errnoError("Could not create socket");

  int on = 1;
  if (setsockopt(m_fd, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on)) != 0)
    errnoError("Could not set IP_PKTINFO flag");

  if (setsockopt(m_fd, IPPROTO_IP, IPV6_RECVPKTINFO, &on, sizeof(on)) != 0)
    errnoError("Could not set IPV6_RECVPKTINFO flag");

  sockaddr_in6 addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin6_family = AF_INET6;
  addr.sin6_addr   = IN6ADDR_ANY_INIT;
  addr.sin6_port   = htons(port);

  if (bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
    close(m_fd);
    errnoError("Could not bind socket");
  }

  ROS_WARN("[SERVICE_SERVER]: UDP Service server initialized.");
}

UDPServer::~UDPServer() {
  ROS_INFO("[SERVICE_SERVER]: UDP Service server terminated.");
  close(m_fd);
}

void UDPServer::step() {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 500 * 1000;

  int ret = select(m_fd + 1, &fds, 0, 0, &timeout);
  if (ret < 0) {
    // Silently ignore EINTR, EAGAIN
    if (errno == EINTR || errno == EAGAIN)
      return;

    errnoError("Could not select()");
  }

  if (ret > 0) {
    handlePacket();
  }

  // Cleanup old requestHandlers
  ros::Time now = ros::Time::now();

  auto it = m_requestList.begin();
  while (it != m_requestList.end()) {
    auto& reqHandler = *it;

    if (now - reqHandler->receptionTime < ros::Duration(20.0))
      break;

    {
      boost::unique_lock<boost::mutex> lock(reqHandler->mutex);

      if (reqHandler->calling) {
        // This one is still active, keep it alive
        it++;
        continue;
      }

      reqHandler->serviceThread.join();
    }

    it = m_requestList.erase(it);
  }
}

void UDPServer::handlePacket() {
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

    m_buffer.resize(packetSize);
    break;
  }

  sockaddr_storage addr;
  memset(&addr, 0, sizeof(addr));

  msghdr msg;
  iovec  iov;

  memset(&iov, 0, sizeof(iov));
  iov.iov_base = m_buffer.data();
  iov.iov_len  = m_buffer.size();

  memset(&msg, 0, sizeof(msg));
  msg.msg_iov    = &iov;
  msg.msg_iovlen = 1;

  msg.msg_control    = m_ctrlBuf;
  msg.msg_controllen = sizeof(m_ctrlBuf);

  msg.msg_name    = &addr;
  msg.msg_namelen = sizeof(addr);

  int ret = recvmsg(m_fd, &msg, 0);
  if (ret < 0) {
    ROS_ERROR("[SERVICE_SERVER]: Could not recvmsg(): %s", strerror(errno));
    return;
  }

  m_buffer.resize(ret);


  if (m_buffer.size() < (int)sizeof(ServiceCallAcknowledgement)) {
    ROS_ERROR("[SERVICE_SERVER]: Received short packet of size %lu", m_buffer.size());
    return;
  }

  // parse acknowledgement msg
  if (m_buffer.size() == (int)sizeof(ServiceCallAcknowledgement)) {

    const auto* ack = reinterpret_cast<const ServiceCallAcknowledgement*>(m_buffer.data());

    auto cmp = boost::make_shared<RequestHandler>(ack->timestamp(), ack->counter, "", ros::SerializedMessage(), 0, 0);

    auto it = std::lower_bound(m_requestList.begin(), m_requestList.end(), cmp,
                               [&](const boost::shared_ptr<RequestHandler>& a, const boost::shared_ptr<RequestHandler>& b) {
                                 if (a->timestamp < b->timestamp)
                                   return true;
                                 else if (a->timestamp > b->timestamp)
                                   return false;

                                 return a->counter < b->counter;
                               });

    boost::shared_ptr<RequestHandler> reqHandler;
    if (it != m_requestList.end()) {
      reqHandler = *it;
    }

    if (reqHandler && reqHandler->timestamp == ack->timestamp() && reqHandler->counter == ack->counter) {

      boost::unique_lock<boost::mutex> lock(reqHandler->mutex);
      reqHandler->cond_msg_acknowledgement_received.notify_one();
      return;
    } else {
      ROS_WARN("[SERVICE_SERVER]: Received unexpected UDP service packet answer, ignoring");
      return;
    }
  }

  if (m_buffer.size() < sizeof(ServiceCallRequest)) {
    ROS_ERROR("[SERVICE_SERVER]: Received short packet of size %lu", m_buffer.size());
    return;
  }

  const ServiceCallRequest* req = reinterpret_cast<ServiceCallRequest*>(m_buffer.data());

  if (m_buffer.size() < sizeof(ServiceCallRequest) + req->name_length) {
    ROS_ERROR("[SERVICE_SERVER]: request header references name which is out-of-buffer");
    return;
  }

  if (m_buffer.size() < sizeof(ServiceCallRequest) + req->name_length() + req->request_length()) {
    ROS_ERROR("[SERVICE_SERVER]: request header references data which is out-of-buffer");
    return;
  }

  auto cmp = boost::make_shared<RequestHandler>(req->timestamp(), req->counter, "", ros::SerializedMessage(), 0, 0);

  auto it = std::lower_bound(m_requestList.begin(), m_requestList.end(), cmp,
                             [&](const boost::shared_ptr<RequestHandler>& a, const boost::shared_ptr<RequestHandler>& b) {
                               if (a->timestamp < b->timestamp)
                                 return true;
                               else if (a->timestamp > b->timestamp)
                                 return false;

                               return a->counter < b->counter;
                             });

  boost::shared_ptr<RequestHandler> reqHandler;
  if (it != m_requestList.end()) {
    reqHandler = *it;
  }

  if (!reqHandler || reqHandler->timestamp != req->timestamp() || reqHandler->counter != req->counter) {
    std::string name(reinterpret_cast<char*>(m_buffer.data() + sizeof(ServiceCallRequest)), req->name_length());

    boost::shared_array<uint8_t> array(new uint8_t[req->request_length()]);
    memcpy(array.get(), m_buffer.data() + sizeof(ServiceCallRequest) + req->name_length(), req->request_length());

    ros::SerializedMessage msg_request(array, req->request_length());

    reqHandler     = boost::make_shared<RequestHandler>(req->timestamp(), req->counter, name, msg_request, m_call_timeout, m_call_repeats);
    reqHandler->fd = m_fd;
    memcpy(&reqHandler->addr, &addr, sizeof(addr));
    reqHandler->addrLen = msg.msg_namelen;

    // Read control msg to get the address we received the msg on
    reqHandler->have_in_pktinfo  = false;
    reqHandler->have_in6_pktinfo = false;
    {
      struct cmsghdr* cmsg;

      for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != 0; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
        if (cmsg->cmsg_level == IPPROTO_IP && cmsg->cmsg_type == IP_PKTINFO) {
          reqHandler->in_info         = *(struct in_pktinfo*)CMSG_DATA(cmsg);
          reqHandler->have_in_pktinfo = true;
        }
        if (cmsg->cmsg_level == IPPROTO_IPV6 && cmsg->cmsg_type == IPV6_PKTINFO) {
          reqHandler->in6_info         = *(struct in6_pktinfo*)CMSG_DATA(cmsg);
          reqHandler->have_in6_pktinfo = true;
        }
      }
    }
    reqHandler->sendAcknowledgement();

    reqHandler->calling       = true;
    reqHandler->serviceThread = boost::thread(boost::bind(&RequestHandler::call, reqHandler));

    reqHandler->receptionTime = ros::Time::now();

    m_requestList.push_back(reqHandler);
  } else {
    boost::unique_lock<boost::mutex> lock(reqHandler->mutex);

    if (reqHandler->calling) {
      ROS_WARN("[SERVICE_SERVER]: Received additional request for in-progress service call");
      reqHandler->sendAcknowledgement();
    } else {
      reqHandler->sendResponse();
    }
  }
}

void UDPServer::RequestHandler::sendResponse() {
  iovec  iov;
  msghdr msg;

  memset(&iov, 0, sizeof(iov));
  iov.iov_base = response.data();
  iov.iov_len  = response.size();

  memset(&msg, 0, sizeof(msg));
  msg.msg_name    = &addr;
  msg.msg_namelen = addrLen;

  msg.msg_iov    = &iov;
  msg.msg_iovlen = 1;

  uint8_t controlBuf[512];
  msg.msg_control    = controlBuf;
  msg.msg_controllen = sizeof(controlBuf);

  size_t controlSpace = 0;

  cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  if (have_in6_pktinfo) {
    cmsg->cmsg_level               = IPPROTO_IPV6;
    cmsg->cmsg_type                = IPV6_PKTINFO;
    cmsg->cmsg_len                 = CMSG_LEN(sizeof(in6_pktinfo));
    *(in6_pktinfo*)CMSG_DATA(cmsg) = in6_info;
    controlSpace += CMSG_SPACE(sizeof(in6_pktinfo));
  } else if (have_in_pktinfo) {
    cmsg->cmsg_level              = IPPROTO_IP;
    cmsg->cmsg_type               = IP_PKTINFO;
    cmsg->cmsg_len                = CMSG_LEN(sizeof(in_pktinfo));
    *(in_pktinfo*)CMSG_DATA(cmsg) = in_info;
    controlSpace += CMSG_SPACE(sizeof(in_pktinfo));
  }

  msg.msg_controllen = controlSpace;

  int ret = sendmsg(fd, &msg, 0);
  if (ret < 0) {
    ROS_ERROR("[SERVICE_SERVER]: Could not sendmsg(): %s", strerror(errno));
    return;
  }
}

void UDPServer::RequestHandler::sendAcknowledgement() {
  std::vector<uint8_t> buffer(sizeof(ServiceCallAcknowledgement));

  ServiceCallAcknowledgement* header = reinterpret_cast<ServiceCallAcknowledgement*>(buffer.data());

  header->timestamp = timestamp;
  header->counter   = counter;

  iovec  iov;
  msghdr msg;

  memset(&iov, 0, sizeof(iov));
  iov.iov_base = buffer.data();
  iov.iov_len  = buffer.size();

  memset(&msg, 0, sizeof(msg));
  msg.msg_name    = &addr;
  msg.msg_namelen = addrLen;

  msg.msg_iov    = &iov;
  msg.msg_iovlen = 1;

  uint8_t controlBuf[512];
  msg.msg_control    = controlBuf;
  msg.msg_controllen = sizeof(controlBuf);

  size_t controlSpace = 0;

  cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  if (have_in6_pktinfo) {
    cmsg->cmsg_level               = IPPROTO_IPV6;
    cmsg->cmsg_type                = IPV6_PKTINFO;
    cmsg->cmsg_len                 = CMSG_LEN(sizeof(in6_pktinfo));
    *(in6_pktinfo*)CMSG_DATA(cmsg) = in6_info;
    controlSpace += CMSG_SPACE(sizeof(in6_pktinfo));
  } else if (have_in_pktinfo) {
    cmsg->cmsg_level              = IPPROTO_IP;
    cmsg->cmsg_type               = IP_PKTINFO;
    cmsg->cmsg_len                = CMSG_LEN(sizeof(in_pktinfo));
    *(in_pktinfo*)CMSG_DATA(cmsg) = in_info;
    controlSpace += CMSG_SPACE(sizeof(in_pktinfo));
  }

  msg.msg_controllen = controlSpace;

  int ret = sendmsg(fd, &msg, 0);
  if (ret < 0) {
    ROS_ERROR("[SERVICE_SERVER]: Could not sendmsg() with acknowledgement: %s", strerror(errno));
    return;
  }
}

void UDPServer::RequestHandler::call() {
  ros::ServiceClientOptions ops(service, "*", false, ros::M_string());
  ros::ServiceClient        client = ros::NodeHandle().serviceClient(ops);

  ros::SerializedMessage msg_response;


  ROS_INFO("[SERVICE_SERVER]: [%s]: Calling.", ops.service.c_str());
  bool call_result = client.call(request, msg_response, std::string("*"));
  if (call_result) {
    ROS_INFO("[SERVICE_SERVER]: [%s]: Call was successful.", ops.service.c_str());
  } else {
    ROS_ERROR("[SERVICE_SERVER]: [%s]: Call error.", ops.service.c_str());
  }

  topic_tools::ShapeShifter deserializedResponse;
  ros::serialization::deserializeMessage(msg_response, deserializedResponse);

  ros::SerializedMessage msg_service_response;

  msg_service_response = ros::serialization::serializeServiceResponse(call_result, deserializedResponse);

  boost::unique_lock<boost::mutex> lock(mutex);

  response.resize(sizeof(ServiceCallResponse) + msg_service_response.num_bytes);

  ServiceCallResponse* resp = reinterpret_cast<ServiceCallResponse*>(response.data());
  resp->response_length     = msg_service_response.num_bytes;
  resp->timestamp           = timestamp;
  resp->counter             = counter;

  memcpy(response.data() + sizeof(ServiceCallResponse), msg_service_response.buf.get(), msg_service_response.num_bytes);

  calling = false;

  bool gotAck = false;
  for (int i = 0; i < call_repeats; i++) {
    sendResponse();
    boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(call_timeout * 1000);
    gotAck                           = cond_msg_acknowledgement_received.timed_wait(lock, timeout);
    if (gotAck) {
      break;
    }
  }

  if (!gotAck) {
    ROS_ERROR("[SERVICE_SERVER]: [%s]: Have not received Ack in timeout!", service.c_str());
  }
}

}  // namespace nimbro_service_transport


int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_server");

  nimbro_service_transport::UDPServer server;

  while (ros::ok()) {
    server.step();
    ros::spinOnce();
  }

  return 0;
}
