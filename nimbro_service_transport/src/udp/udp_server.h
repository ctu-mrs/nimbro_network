// UDP service server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <ros/node_handle.h>

#include <boost/thread.hpp>

#include <sys/socket.h>

#include <netinet/ip.h>

namespace nimbro_service_transport
{

class UDPServer {
public:
  UDPServer();
  ~UDPServer();

  void step();

private:
  void handlePacket();

  ros::NodeHandle m_nh;

  int m_fd;
  double m_call_timeout;
  int m_call_repeats;

  std::vector<uint8_t> m_buffer;
  uint8_t              m_ctrlBuf[1024];

  struct RequestHandler
  {
    RequestHandler(uint64_t timestamp, uint8_t counter, const std::string& service, const ros::SerializedMessage& request, const double& call_timeout,
                   const int& call_repeats)
        : timestamp(timestamp), counter(counter), service(service), request(request), call_timeout(call_timeout), call_repeats(call_repeats), calling(false) {
    }

    void call();

    void sendResponse();
    void sendAcknowledgement();

    bool operator<(const RequestHandler& other) const;

    int              fd;
    sockaddr_storage addr;
    socklen_t        addrLen;

    uint64_t               timestamp;
    uint8_t                counter;
    std::string            service;
    ros::SerializedMessage request;

    double call_timeout;
    int    call_repeats;

    bool          calling;
    boost::thread serviceThread;

    boost::mutex         mutex;
    std::vector<uint8_t> response;

    ros::Time receptionTime;

    in_pktinfo  in_info;
    in6_pktinfo in6_info;

    bool have_in_pktinfo;
    bool have_in6_pktinfo;

    boost::condition_variable cond_msg_acknowledgement_received;
  };

  std::list<boost::shared_ptr<RequestHandler>> m_requestList;
};

}  // namespace nimbro_service_transport

#endif
