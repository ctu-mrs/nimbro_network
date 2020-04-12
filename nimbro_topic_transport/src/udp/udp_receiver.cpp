// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_receiver.h"
#include "udp_packet.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <topic_tools/shape_shifter.h>

#include <std_msgs/Time.h>

#include <stdio.h>

#include <ros/names.h>

#include "../topic_info.h"

#include <nimbro_topic_transport/CompressedMsg.h>
#include <mrs_lib/param_loader.h>

#if WITH_PLOTTING
#include <plot_msgs/Plot.h>
#endif

#include <fcntl.h>

namespace nimbro_topic_transport
{

UDPReceiver::UDPReceiver() : m_receivedBytesInStatsInterval(0), m_expectedPacketsInStatsInterval(0), m_missingPacketsInStatsInterval(0) {
  ros::NodeHandle private_nh("~");

  m_pub_heartbeat = private_nh.advertise<std_msgs::Time>("heartbeat", 1);

#if WITH_PLOTTING
  m_pub_plot = private_nh.advertise<plot_msgs::Plot>("plot", 1000);
#endif

  m_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_fd < 0) {
    ROS_FATAL("[TOPIC_RECEIVER]: Could not create socket: %s", strerror(errno));
    throw std::runtime_error(strerror(errno));
  }

  // | ------------------- loading parameters ------------------- |
  mrs_lib::ParamLoader param_loader(private_nh, "TOPIC_RECEIVER");
  ROS_INFO("[TOPIC_RECEIVER]: Loading parameters: ");

  char hostnameBuf[256];
  gethostname(hostnameBuf, sizeof(hostnameBuf));
  hostnameBuf[sizeof(hostnameBuf) - 1] = 0;

  m_hostname = std::string(hostnameBuf);

  int port;
  param_loader.loadParam("port", port, 5050);

  sockaddr_in addr;
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port        = htons(port);

  if (bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
    ROS_FATAL("[TOPIC_RECEIVER]: Could not bind socket: %s", strerror(errno));
    throw std::runtime_error(strerror(errno));
  }

  int on = 1;
  if (setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0) {
    ROS_FATAL("[TOPIC_RECEIVER]: Could not set broadcast flag: %s", strerror(errno));
    throw std::runtime_error(strerror(errno));
  }

  param_loader.loadParam("drop_repeated_msgs", m_dropRepeatedMessages, true);
  param_loader.loadParam("warn_drop_incomplete", m_warnDropIncomplete, true);
  param_loader.loadParam("keep_compressed", m_keepCompressed, false);

  param_loader.loadParam("fec", m_fec, false);

#if !(WITH_OPENFEC)
  if (m_fec)
    throw std::runtime_error("Please compile with FEC support to enable FEC");
#endif


  m_stats.node       = ros::this_node::getName();
  m_stats.protocol   = "UDP";
  m_stats.host       = m_hostname.c_str();
  m_stats.local_port = port;
  m_stats.fec        = m_fec;

  param_loader.loadParam("label", m_stats.label, std::string());

  std::string topic_prefix;
  param_loader.loadParam("topic_prefix", topic_prefix, std::string());

  // | ----------------------- finish loading ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[TOPIC_RECEIVER]: Could not load all parameters!");
    ros::shutdown();
    return;
  }

  std::stringstream topic_name;
  if (!topic_prefix.empty()) {
    topic_name << "/" << topic_prefix << "/network/receiver_stats";
  } else {
    topic_name << "/network/receiver_stats";
  }
  m_pub_stats     = private_nh.advertise<ReceiverStats>(topic_name.str(), 1);
  m_statsInterval = ros::WallDuration(2.0);
  m_statsTimer    = private_nh.createWallTimer(m_statsInterval, boost::bind(&UDPReceiver::updateStats, this));
}

UDPReceiver::~UDPReceiver() {
}

template <class HeaderType>
void UDPReceiver::handleFinishedMessage(Message* msg, HeaderType* header) {
  if (msg->complete)
    return;

  // Packet complete
  msg->complete = true;

  // Enforce termination
  header->topic_type[sizeof(header->topic_type) - 1] = 0;
  header->topic_name[sizeof(header->topic_name) - 1] = 0;

  ROS_DEBUG("[TOPIC_RECEIVER]: Got a packet of type %s, topic %s, (msg id %d), size %d", header->topic_type, header->topic_name, msg->id,
            (int)msg->payload.size());

  // Find topic
  TopicMap::iterator topic_it = m_topics.find(header->topic_name);

  boost::shared_ptr<TopicReceiver> topic;
  if (topic_it == m_topics.end()) {
    m_topics.insert(std::pair<std::string, boost::shared_ptr<TopicReceiver>>(header->topic_name, boost::make_shared<TopicReceiver>()));
    topic = m_topics[header->topic_name];

    topic->last_message_counter = -1;
  } else
    topic = topic_it->second;

  // Send heartbeat message
  ros::Time now = ros::Time::now();
  if (now - m_lastHeartbeatTime > ros::Duration(0.2)) {
    std_msgs::Time time;
    time.data = now;
    m_pub_heartbeat.publish(time);

    m_lastHeartbeatTime = now;
  }

#if WITH_PLOTTING
  // Plot packet size
  plot_msgs::Plot plot;
  plot.header.stamp = ros::Time::now();

  plot_msgs::PlotPoint point;

  std::string safe_topic = header->topic_name;
  std::replace(safe_topic.begin(), safe_topic.end(), '/', '_');

  point.name  = "/udp_receiver/mbyte/" + safe_topic;
  point.value = ((double)msg->getLength()) / 1024 / 1024;

  plot.points.push_back(point);
  m_pub_plot.publish(plot);
#endif

  if (m_dropRepeatedMessages && header->topic_msg_counter() == topic->last_message_counter) {
    // This is the same message, probably sent in relay mode
    return;
  }

  bool compressed = (header->flags & UDP_FLAG_COMPRESSED) || (header->flags & UDP_FLAG_ZSTD);

  // Compare md5
  if (topic->last_message_counter == -1 || memcmp(topic->md5, header->topic_md5, sizeof(topic->md5)) != 0 ||
      (m_keepCompressed && topic->compressed != compressed)) {
    topic->msg_def = topic_info::getMsgDef(header->topic_type);
    topic->md5_str = topic_info::getMd5Sum(header->topic_type);

    if (topic->msg_def.empty() || topic->md5_str.size() != 8 * 4) {
      ROS_ERROR("[TOPIC_RECEIVER]: Could not find msg type '%s', please make sure msg definitions are up to date", header->topic_type);
      return;
    }

    ROS_INFO("[TOPIC_RECEIVER]: Received first message on topic '%s'", header->topic_name);
    for (int i = 0; i < 4; ++i) {
      std::string md5_part = topic->md5_str.substr(8 * i, 8);
      uint32_t    md5_num  = strtoll(md5_part.c_str(), 0, 16);
      topic->md5[i]        = md5_num;
    }

    if (memcmp(topic->md5, header->topic_md5, sizeof(topic->md5)) != 0) {
      ROS_ERROR("[TOPIC_RECEIVER]: Invalid md5 sum for topic type '%s', please make sure msg definitions are up to date", header->topic_type);
      return;
    }

    std::string t = header->topic_name;

    if (m_keepCompressed && compressed) {
      // If we are requested to keep the messages compressed, we advertise our compressed msg type
      topic->publisher = m_nh.advertise<CompressedMsg>(t, 1, boost::bind(&TopicReceiver::handleSubscriber, topic));
    } else {
      // ... otherwise, we advertise the native type
      ros::AdvertiseOptions options(t, 1, topic->md5_str, header->topic_type, topic->msg_def, boost::bind(&TopicReceiver::handleSubscriber, topic));

      // Latching is often unexpected. Better to lose the first msg.
      // options.latch = 1;
      topic->publisher = m_nh.advertise(options);
    }

    topic->compressed = compressed;
  }

  if (compressed && m_keepCompressed) {
    ROS_DEBUG("[TOPIC_RECEIVER]: publishing compressed message as-is");
    CompressedMsgPtr compressed(new CompressedMsg);
    compressed->type = header->topic_type;
    memcpy(compressed->md5.data(), topic->md5, sizeof(topic->md5));

    compressed->data.swap(msg->payload);

    topic->publishCompressed(compressed);
  } else if (compressed) {
    ROS_DEBUG("[TOPIC_RECEIVER]: decompressing, flags: %d, msg->header.flags: %d", (int)header->flags, (int)msg->header.flags);
    auto msgPtr = boost::make_shared<Message>(*msg);
    ROS_DEBUG("[TOPIC_RECEIVER]: msgPtr->header.flags: %d", (int)msgPtr->header.flags);
    topic->takeForDecompression(msgPtr);
  } else {
    ROS_DEBUG("[TOPIC_RECEIVER]: publishing non-compressed message directly");
    boost::shared_ptr<topic_tools::ShapeShifter> shapeShifter(new topic_tools::ShapeShifter);
    shapeShifter->morph(topic->md5_str, header->topic_type, topic->msg_def, "");

    shapeShifter->read(*msg);

    topic->publish(shapeShifter);
  }

  topic->last_message_counter = header->topic_msg_counter();
}

void UDPReceiver::run() {
  std::vector<uint8_t> buf;

  ROS_WARN("[TOPIC_RECEIVER]: UDP receiver is ready");
  while (1) {
    ros::spinOnce();

    // handleMessagePacket() below might swap() our buffer out, so make
    // sure that we have enough room.
    buf.resize(PACKET_SIZE);

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_fd, &fds);

    timeval timeout;
    timeout.tv_usec = 50 * 1000;
    timeout.tv_sec  = 0;

    int ret = select(m_fd + 1, &fds, 0, 0, &timeout);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN)
        continue;

      ROS_FATAL("[TOPIC_RECEIVER]: Could not select(): %s", strerror(errno));
      throw std::runtime_error(strerror(errno));
    }
    if (ret == 0)
      continue;

    sockaddr_storage addr;
    socklen_t        addrlen = sizeof(addr);

    ssize_t size = recvfrom(m_fd, buf.data(), buf.size(), 0, (sockaddr*)&addr, &addrlen);

    if (size < 0) {
      ROS_FATAL("[TOPIC_RECEIVER]: Could not recv(): %s", strerror(errno));
      throw std::runtime_error(strerror(errno));
    }

    bool is_msg_from_new_remote = true;
    for (unsigned long it = 0; it < m_remoteAddr.size(); it++) {
      if (addrlen == m_remoteAddrLen.at(it) && memcmp(&addr, &m_remoteAddr.at(it), addrlen) == 0) {
        is_msg_from_new_remote = false;
        break;
      }
    }

    if (addrlen == m_hostAddrLen && memcmp(&addr, &m_hostAddr, addrlen) == 0) {
      m_receivedBytesInStatsInterval += size;
      continue;
    }


    if (is_msg_from_new_remote) {
      // Perform reverse lookup
      char nameBuf[256];
      char serviceBuf[256];

      ros::WallTime startLookup = ros::WallTime::now();
      if (getnameinfo((sockaddr*)&addr, addrlen, nameBuf, sizeof(nameBuf), serviceBuf, sizeof(serviceBuf), NI_NUMERICSERV) == 0) {
        m_stats.remote      = nameBuf;
        m_stats.remote_port = atoi(serviceBuf);

        // received msg from this host
        if (m_hostname.compare(nameBuf) == 0) {

          m_hostAddr    = addr;
          m_hostAddrLen = addrlen;

          m_receivedBytesInStatsInterval += size;
          continue;
        }

        ROS_INFO("[TOPIC_RECEIVER]: Receive msg from new remote: %s:%s", nameBuf, serviceBuf);
      } else {
        char host[NI_MAXHOST];
        if (getnameinfo((sockaddr*)&addr, addrlen, host, sizeof(host), NULL, 0, NI_NUMERICHOST) == 0) {
          ROS_ERROR("[TOPIC_RECEIVER]: Could not resolve remote address '%s' to name", host);
        } else
          ROS_ERROR("[TOPIC_RECEIVER]: Could not resolve remote address to name");
        m_stats.remote      = "unknown";
        m_stats.remote_port = -1;
      }
      ros::WallTime endLookup = ros::WallTime::now();

      // Warn if lookup takes up time (otherwise the user does not know
      // what is going on)
      if (endLookup - startLookup > ros::WallDuration(1.0)) {
        ROS_WARN(
            "[TOPIC_RECEIVER]: Reverse address lookup took more than a second. "
            "Consider adding '%s' to /etc/hosts",
            m_stats.remote.c_str());
      }


      m_remoteAddr.push_back(addr);
      m_remoteAddrLen.push_back(addrlen);
    }

    // 		ROS_DEBUG("packet of size %lu", size);
    m_receivedBytesInStatsInterval += size;

    uint16_t msg_id;

    // Obtain the message ID from the packet
    if (m_fec) {
      FECPacket::Header* header = (FECPacket::Header*)buf.data();
      msg_id                    = header->msg_id();
    } else {
      UDPGenericPacket* generic = (UDPGenericPacket*)buf.data();
      msg_id                    = generic->msg_id();
    }

    // Look up the message ID in our list of incomplete messages
    MessageBuffer::iterator it = std::find_if(m_incompleteMessages.begin(), m_incompleteMessages.end(), [=](const Message& msg) { return msg.id == msg_id; });

    if (it == m_incompleteMessages.end()) {
      // Insert a new message
      m_incompleteMessages.push_front(Message(msg_id));
      it = m_incompleteMessages.begin();

      pruneMessages();
    }

    handleMessagePacket(it, &buf, size);
  }
}

void UDPReceiver::updateStats() {
  m_stats.header.stamp = ros::Time::now();
  m_stats.bandwidth    = m_receivedBytesInStatsInterval / m_statsInterval.toSec();

  m_stats.drop_rate = ((double)m_missingPacketsInStatsInterval) / m_expectedPacketsInStatsInterval;

  m_pub_stats.publish(m_stats);

  // Reset all stats counters
  m_receivedBytesInStatsInterval   = 0;
  m_missingPacketsInStatsInterval  = 0;
  m_expectedPacketsInStatsInterval = 0;
}

void UDPReceiver::pruneMessages() {
  // Erase messages that are too old (after index 31)
  MessageBuffer::iterator itr    = m_incompleteMessages.begin();
  MessageBuffer::iterator it_end = m_incompleteMessages.end();
  for (int i = 0; i < 31; ++i) {
    itr++;
    if (itr == it_end)
      break;
  }

  // Collect statistics on packets which will be deleted
  for (MessageBuffer::iterator itd = itr; itd != it_end; ++itd) {
    const Message& msg = *itd;

#if WITH_OPENFEC
    if (m_fec) {
      if (msg.params) {
        int total = msg.params->nb_source_symbols + msg.params->nb_repair_symbols;
        m_expectedPacketsInStatsInterval += total;
        m_missingPacketsInStatsInterval += total - msg.received_symbols;
      }
    } else
#endif
    {
      int num_fragments = msg.msgs.size();
      int received      = 0;
      for (unsigned int i = 0; i < msg.msgs.size(); ++i) {
        if (msg.msgs[i])
          received++;
      }

      m_expectedPacketsInStatsInterval += num_fragments;
      m_missingPacketsInStatsInterval += num_fragments - received;
    }
  }

  // If enabled, warn each time we drop an incomplete message
  if (m_warnDropIncomplete) {
    for (MessageBuffer::iterator itd = itr; itd != it_end; ++itd) {
      const Message& msg = *itd;

      if (msg.complete)
        continue;

      int num_fragments = msg.msgs.size();
      int received      = 0;
      for (unsigned int i = 0; i < msg.msgs.size(); ++i) {
        if (msg.msgs[i])
          received++;
      }

#if WITH_OPENFEC
      if (msg.decoder) {
        ROS_WARN("[TOPIC_RECEIVER]: Dropping FEC message %d (%u/%u symbols)", msg.id, msg.received_symbols, msg.params->nb_source_symbols);
      } else
#endif
      {
        ROS_WARN("[TOPIC_RECEIVER]: Dropping message %d, %.2f%% of fragments received (%d/%d)", msg.id, 100.0 * received / num_fragments, received,
                 num_fragments);
      }
    }
  }

  // Finally, delete all messages after index 31
  m_incompleteMessages.erase(itr, it_end);
}

void UDPReceiver::handleMessagePacket(MessageBuffer::iterator it, std::vector<uint8_t>* buf, std::size_t size) {
  Message* msg = &*it;

  // If the message is already completed (happens especially with FEC),
  // drop this packet.
  if (msg->complete) {
#if WITH_OPENFEC
    // ... but still include it in the statistics!
    msg->received_symbols++;
#endif
    return;
  }

  if (m_fec) {
#if WITH_OPENFEC
    // Save the received packet (OpenFEC expects all symbols to stay
    // available until end of decoding)
    boost::shared_ptr<std::vector<uint8_t>> fecBuffer(new std::vector<uint8_t>);
    fecBuffer->swap(*buf);

    msg->fecPackets.push_back(fecBuffer);

    FECPacket* packet = (FECPacket*)fecBuffer->data();

    if (!msg->decoder) {
      of_session_t*    ses    = 0;
      of_parameters_t* params = 0;

      if (packet->header.source_symbols() >= MIN_PACKETS_LDPC) {
        ROS_DEBUG("[TOPIC_RECEIVER]: LDPC");
        if (of_create_codec_instance(&ses, OF_CODEC_LDPC_STAIRCASE_STABLE, OF_DECODER, 1) != OF_STATUS_OK) {
          ROS_ERROR("[TOPIC_RECEIVER]: Could not create LDPC decoder");
          return;
        }

        of_ldpc_parameters_t* ldpc_params   = (of_ldpc_parameters_t*)malloc(sizeof(of_ldpc_parameters_t));
        ldpc_params->nb_source_symbols      = packet->header.source_symbols();
        ldpc_params->nb_repair_symbols      = packet->header.repair_symbols();
        ldpc_params->encoding_symbol_length = packet->header.symbol_length();
        ldpc_params->prng_seed              = packet->header.prng_seed();
        ldpc_params->N1                     = 7;

        ROS_DEBUG("[TOPIC_RECEIVER]: LDPC parameters: %d, %d, %d, 0x%X, %d", ldpc_params->nb_source_symbols, ldpc_params->nb_repair_symbols,
                  ldpc_params->encoding_symbol_length, ldpc_params->prng_seed, ldpc_params->N1);

        params = (of_parameters*)ldpc_params;
      } else {
        ROS_DEBUG("[TOPIC_RECEIVER]: REED");
        if (of_create_codec_instance(&ses, OF_CODEC_REED_SOLOMON_GF_2_M_STABLE, OF_DECODER, 1) != OF_STATUS_OK) {
          ROS_ERROR("[TOPIC_RECEIVER]: Could not create REED_SOLOMON decoder");
          return;
        }

        of_rs_2_m_parameters* rs_params   = (of_rs_2_m_parameters_t*)malloc(sizeof(of_rs_2_m_parameters_t));
        rs_params->nb_source_symbols      = packet->header.source_symbols();
        rs_params->nb_repair_symbols      = packet->header.repair_symbols();
        rs_params->encoding_symbol_length = packet->header.symbol_length();
        rs_params->m                      = 8;

        ROS_DEBUG("[TOPIC_RECEIVER]: REED params: %d, %d, %d", rs_params->nb_source_symbols, rs_params->nb_repair_symbols, rs_params->encoding_symbol_length);

        params = (of_parameters_t*)rs_params;
      }

      if (of_set_fec_parameters(ses, params) != OF_STATUS_OK) {
        ROS_ERROR("[TOPIC_RECEIVER]: Could not set FEC parameters");
        of_release_codec_instance(ses);
        return;
      }

      msg->decoder.reset(ses, of_release_codec_instance);
      msg->params.reset(params, free);

      msg->received_symbols = 0;
    }

    msg->received_symbols++;

    ROS_DEBUG("[TOPIC_RECEIVER]: msg: %10d, symbol: %10d/%10d", msg->id, packet->header.symbol_id(), packet->header.source_symbols());

    uint8_t* symbol_begin = packet->data;

    if (size - sizeof(FECPacket::Header) != msg->params->encoding_symbol_length) {
      ROS_ERROR("[TOPIC_RECEIVER]: Symbol size mismatch: got %d, expected %d", (int)(size - sizeof(FECPacket::Header)),
                (int)(msg->params->encoding_symbol_length));
      return;
    }

    // FEC iterative decoding
    if (of_decode_with_new_symbol(msg->decoder.get(), symbol_begin, packet->header.symbol_id()) != OF_STATUS_OK) {
      ROS_ERROR_THROTTLE(5.0, "[TOPIC_RECEIVER]: Could not decode symbol");
      return;
    }

    bool done = false;

    if (msg->received_symbols >= msg->params->nb_source_symbols) {
      // Are we finished using the iterative decoding already?
      done = of_is_decoding_complete(msg->decoder.get());

      // As it is implemented in OpenFEC right now, we can only
      // try the ML decoding (gaussian elimination) *once*.
      // After that the internal state is screwed up and the message
      // has to be discarded.
      // So we have to be sure that it's worth it ;-)
      if (!done && msg->received_symbols >= msg->params->nb_source_symbols + msg->params->nb_repair_symbols / 2) {
        of_status_t ret = of_finish_decoding(msg->decoder.get());
        if (ret == OF_STATUS_OK)
          done = true;
        else {
          ROS_ERROR("[TOPIC_RECEIVER]: ML decoding failed, dropping message...");
          msg->complete = true;
          return;
        }
      }
    }

    if (done) {
      ROS_DEBUG("[TOPIC_RECEIVER]: FEC: Decoding done!");

      std::vector<void*> symbols(msg->params->nb_source_symbols, 0);

      if (of_get_source_symbols_tab(msg->decoder.get(), symbols.data()) != OF_STATUS_OK) {
        ROS_ERROR("[TOPIC_RECEIVER]: Could not get decoded symbols");
        return;
      }

      uint64_t payloadLength = msg->params->nb_source_symbols * msg->params->encoding_symbol_length;
      if (msg->params->encoding_symbol_length < sizeof(FECHeader) || payloadLength < sizeof(FECHeader)) {
        ROS_ERROR("[TOPIC_RECEIVER]: Invalid short payload");
        m_incompleteMessages.erase(it);
        return;
      }

      FECHeader msgHeader;
      memcpy(&msgHeader, symbols[0], sizeof(FECHeader));
      payloadLength -= sizeof(FECHeader);

      msg->payload.resize(payloadLength);

      uint8_t* writePtr = msg->payload.data();
      memcpy(msg->payload.data(), ((uint8_t*)symbols[0]) + sizeof(FECHeader), msg->params->encoding_symbol_length - sizeof(FECHeader));
      writePtr += msg->params->encoding_symbol_length - sizeof(FECHeader);

      for (unsigned int symbol = 1; symbol < msg->params->nb_source_symbols; ++symbol) {
        memcpy(writePtr, symbols[symbol], msg->params->encoding_symbol_length);
        writePtr += msg->params->encoding_symbol_length;
      }

      ROS_DEBUG("[TOPIC_RECEIVER]: Received a message with padding %d", (int)msgHeader.padding);
      payloadLength -= msgHeader.padding;

      msg->payload.resize(payloadLength);
      msg->size = payloadLength;

      // Fill in the header fields for the non-FEC message as good as we can
      strncpy(msg->header.topic_name, msgHeader.topic_name, sizeof(msg->header.topic_name));
      strncpy(msg->header.topic_type, msgHeader.topic_type, sizeof(msg->header.topic_type));
      msg->header.flags             = msgHeader.flags;
      msg->header.topic_msg_counter = msgHeader.topic_msg_counter;

      handleFinishedMessage(msg, &msgHeader);

      // keep completed messages in the buffer so that we know that
      // we have to ignore any additional symbols of that message.
    }
#endif
  } else {
    UDPGenericPacket* generic = (UDPGenericPacket*)buf->data();
    if (generic->frag_id == 0) {
      UDPFirstPacket* first = (UDPFirstPacket*)buf->data();

      msg->header = first->header;

      // We can calculate an approximate size now
      uint32_t my_size = size - sizeof(UDPFirstPacket);
      if (msg->payload.size() < my_size)
        msg->payload.resize(my_size);
      memcpy(msg->payload.data(), first->data, my_size);

      if (msg->size < my_size)
        msg->size = my_size;

      if (((uint16_t)msg->msgs.size()) < msg->header.remaining_packets() + 1)
        msg->msgs.resize(msg->header.remaining_packets() + 1, false);
    } else {
      UDPDataPacket* data = (UDPDataPacket*)buf->data();

      uint32_t offset        = UDPFirstPacket::MaxDataSize + (data->header.frag_id - 1) * UDPDataPacket::MaxDataSize;
      uint32_t required_size = offset + size - sizeof(UDPDataPacket);
      if (msg->payload.size() < required_size)
        msg->payload.resize(required_size);
      memcpy(msg->payload.data() + offset, data->data, size - sizeof(UDPDataPacket));

      if (msg->size < required_size)
        msg->size = required_size;
    }

    if (generic->frag_id >= msg->msgs.size())
      msg->msgs.resize(generic->frag_id + 1, false);

    msg->msgs[generic->frag_id] = true;

    if (std::all_of(msg->msgs.begin(), msg->msgs.end(), [](bool x) { return x; })) {
      handleFinishedMessage(msg, &msg->header);

      // as we delete the message from the buffer here, immediately
      // add it to the statistics
      m_expectedPacketsInStatsInterval += msg->msgs.size();

      m_incompleteMessages.erase(it);
    }
  }
}

}  // namespace nimbro_topic_transport

int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_receiver", ros::init_options::NoSigintHandler);

  nimbro_topic_transport::UDPReceiver recv;

  recv.run();

  return 0;
}
