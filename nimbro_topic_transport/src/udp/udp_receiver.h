// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <map>
#include <vector>
#include <string>
#include <list>

#include <sys/socket.h>

#include <ros/publisher.h>
#include <ros/wall_timer.h>

#include <boost/thread.hpp>

#include <nimbro_topic_transport/ReceiverStats.h>
#include <nimbro_topic_transport/CompressedMsg.h>

#include "udp_packet.h"
#include "topic_receiver.h"

namespace nimbro_topic_transport
{


class UDPReceiver {
public:
  UDPReceiver();
  ~UDPReceiver();

  void run();

private:
  typedef std::map<std::string, boost::shared_ptr<TopicReceiver>> TopicMap;
  typedef std::list<Message>                                      MessageBuffer;

  void handleMessagePacket(MessageBuffer::iterator it, std::vector<uint8_t>* buf, std::size_t size);

  template <class HeaderType>
  void handleFinishedMessage(Message* msg, HeaderType* header);

  void pruneMessages();

  void updateStats();

  int m_fd;

  std::string      m_hostname;
  sockaddr_storage m_hostAddr;
  socklen_t        m_hostAddrLen;

  MessageBuffer m_incompleteMessages;
  TopicMap      m_topics;

  bool m_dropRepeatedMessages;
  bool m_warnDropIncomplete;
  bool m_keepCompressed;

  ros::NodeHandle m_nh;
  ros::Publisher  m_pub_heartbeat;
  ros::Time       m_lastHeartbeatTime;

  ros::Publisher m_pub_plot;

  Message m_decompressedMessage;

  bool m_fec;

  nimbro_topic_transport::ReceiverStats m_stats;
  uint64_t                              m_receivedBytesInStatsInterval;
  uint64_t                              m_expectedPacketsInStatsInterval;
  uint64_t                              m_missingPacketsInStatsInterval;
  ros::Publisher                        m_pub_stats;
  ros::WallDuration                     m_statsInterval;
  ros::WallTimer                        m_statsTimer;

  std::vector<sockaddr_storage> m_remoteAddr;
  std::vector<socklen_t>        m_remoteAddrLen;
};

}  // namespace nimbro_topic_transport

#endif
