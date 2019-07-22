// UDP service client
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <ros/node_handle.h>

#include <ros/service_callback_helper.h>

#include <boost/thread.hpp>

namespace nimbro_service_transport
{

class UDPClient {
public:
  UDPClient(const std::string& robot_hostname, const std::string& robot_addr, std::map<std::string, std::vector<std::string>>& services,
            const double& response_timeout, const double& call_timeout, const int& call_repeats);
  virtual ~UDPClient();

  bool call(const std::string& name, ros::ServiceCallbackHelperCallParams& params);
  void step();

private:
  uint8_t acquireCounterValue();
  void    handlePacket();
  void    run();
  void    publishStatus(const std::string& service, uint32_t call, uint8_t status);

  ros::NodeHandle m_nh;
  int             m_fd;

  std::vector<ros::ServiceServer> m_servers;

  boost::mutex m_mutex;
  uint8_t      m_counter;

  std::vector<uint8_t> m_recvBuf;

  struct RequestRecord
  {
    uint64_t timestamp;
    uint8_t  counter;

    ros::SerializedMessage    response;
    boost::condition_variable cond_response_received;
    boost::condition_variable cond_msg_acknowledgement_received;
  };

  std::list<RequestRecord*> m_requests;

  double m_call_timeout;
  int    m_call_repeats;
  double m_response_timeout;

  std::string m_remote;
  std::string m_remote_hostname;
  int         m_remotePort;
  std::string m_host;

  ros::Publisher m_pub_status;

  std::unique_ptr<boost::thread> m_thread;
};

}  // namespace nimbro_service_transport

#endif
