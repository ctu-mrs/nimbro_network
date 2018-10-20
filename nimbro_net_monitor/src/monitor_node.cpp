// ROS component
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>
#include <ros/master.h>
#include <ros/network.h>

#include <stdexcept>

#include "ros/xmlrpc_manager.h"
#include "xmlrpcpp/XmlRpc.h"

#include <nimbro_net_monitor/NetworkStats.h>

using XMLRPCManager = ros::XMLRPCManager;

static bool slaveCall(const std::string& host, uint32_t port, const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response)
{
	XmlRpc::XmlRpcClient *c = XMLRPCManager::instance()->getXMLRPCClient(host, port, "/");

	if (!c->execute(method.c_str(), request, response))
	{
		XMLRPCManager::instance()->releaseXMLRPCClient(c);
		return false;
	}

	XMLRPCManager::instance()->releaseXMLRPCClient(c);
	return true;
}

struct ConnectionInfo
{
	enum class Direction
	{
		In, Out, Both
	};

	int id = -1;
	std::string destination;
	Direction direction = Direction::Both;
	std::string transport;
	std::string topic;

	bool bytesValid = false;
	int32_t bytes = 0;
	uint64_t bitsPerSecond = 0;

	bool active = true;

	static ConnectionInfo fromXmlRpc(XmlRpc::XmlRpcValue& in)
	{
		ConnectionInfo ret;
		ret.id = in[0];
		ret.destination = static_cast<std::string>(in[1]);

		std::string d = in[2];
		if(d == "i")
			ret.direction = Direction::In;
		else if(d == "o")
			ret.direction = Direction::Out;
		else if(d == "b")
			ret.direction = Direction::Both;
		else
			throw std::invalid_argument("Unknown direction string");

		ret.transport = static_cast<std::string>(in[3]);
		ret.topic = static_cast<std::string>(in[4]);

		return ret;
	}

	void updateBytes(int32_t newBytes, const ros::WallDuration& dt)
	{
		if(bytesValid)
		{
			int32_t diff = newBytes - bytes;
			bitsPerSecond = static_cast<uint64_t>(diff) * 8 / dt.toSec();
		}

		bytes = newBytes;
		bytesValid = true;
	}
};

struct NodeInfo
{
	std::string name;
	std::string uri;
	std::string host;
	uint32_t port;

	std::map<int, ConnectionInfo> connections;
	bool active = false;

	class LookupException : std::runtime_error
	{
	public:
		LookupException(const char* what) : std::runtime_error(what) {}
	};

	static NodeInfo fromName(const std::string& name)
	{
		NodeInfo ret;

		XmlRpc::XmlRpcValue n_response;
		XmlRpc::XmlRpcValue n_payload;

		XmlRpc::XmlRpcValue args;
		args[0] = "net_monitor";
		args[1] = name;

		if(!ros::master::execute("lookupNode", args, n_response, n_payload, false))
		{
			ROS_WARN("Could not lookup node '%s'", name.c_str());
			throw LookupException("Could not find the node");
		}

		ret.name = name;
		ret.uri = static_cast<std::string>(n_payload);

		if(!ros::network::splitURI(ret.uri, ret.host, ret.port))
		{
			ROS_WARN("Could not parse node URI: '%s'", ret.uri.c_str());
			throw LookupException("Could not parse node URI");
		}

		return ret;
	}
};

struct Peer
{
	std::map<std::string, NodeInfo> nodes;
	bool active = true;
};

class SystemMonitor
{
public:
	typedef std::map<std::string, Peer> PeerMap;

	void updateNodes(const ros::WallDuration& dt)
	{
		XmlRpc::XmlRpcValue response;
		XmlRpc::XmlRpcValue payload;
		if(!ros::master::execute("getSystemState", "net_monitor", response, payload, false))
		{
			ROS_ERROR("Could not query system state from master");
			return;
		}

		std::set<std::string> nodeNames;

		for(int i = 0; i < payload.size(); ++i)
		{
			auto advertisement = payload[i];

			for(int j = 0; j < advertisement.size(); ++j)
			{
				auto item = advertisement[j];

				auto pubs = item[1];

				for(int k = 0; k < pubs.size(); ++k)
					nodeNames.insert(pubs[k]);
			}
		}

		for(auto& name : nodeNames)
		{
			try
			{
				m_nodes[name] = NodeInfo::fromName(name);
			}
			catch(NodeInfo::LookupException)
			{
				continue;
			}
		}
	}

	void updateNodeStats(const NodeInfo& nodeInfo, const ros::WallDuration& dt)
	{
		std::string name = nodeInfo.name;

		// Let's call the node for information!
		XmlRpc::XmlRpcValue stats_response;

		if(!slaveCall(nodeInfo.host, nodeInfo.port, "getBusStats", "net_monitor", stats_response))
		{
			ROS_WARN("Could not call node '%s'", name.c_str());
			return;
		}

		// python nodes report (success code, status, response)
		// C++ nodes report response directly. go figure.
		XmlRpc::XmlRpcValue publish_stats;
		XmlRpc::XmlRpcValue subscribe_stats;
		if(stats_response[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
		{
			publish_stats = stats_response[2][0];
			subscribe_stats = subscribe_stats[2][1];
		}
		else
		{
			publish_stats = stats_response[0];
			subscribe_stats = stats_response[1];
		}

		XmlRpc::XmlRpcValue info_response;
		if(!slaveCall(nodeInfo.host, nodeInfo.port, "getBusInfo", "net_monitor", info_response))
		{
			ROS_WARN("Could not call node '%s'", name.c_str());
			return;
		}

		auto info_array = info_response[2];

		std::map<int, ConnectionInfo> connections;
		for(int i = 0; i < info_array.size(); ++i)
		{
			auto c = ConnectionInfo::fromXmlRpc(info_array[i]);
			connections[c.id] = c;
		}

		for(int i = 0; i < publish_stats.size(); ++i)
		{
			auto publication = publish_stats[i];

			// NOTE: Again, python nodes have (topic, msg num, links),
			// C++ nodes have (topic, links)...
			auto links = publication[publication.size()-1];

			for(int j = 0; j < links.size(); ++j)
			{
				auto link = links[j];
				int conId = link[0];

				auto it = connections.find(conId);
				if(it == connections.end())
				{
					ROS_WARN("Could not find connection");
					continue;
				}

				auto& connection = it->second;

				bool local = false;
				auto nodeIt = m_nodes.find(connection.destination);
				if(nodeIt != m_nodes.end())
				{
					local = (nodeIt->second.host == ros::network::getHost());
				}

				if(local)
					continue;

				Peer& peer = m_peers[nodeIt->second.host];
				peer.active = true;

				auto& nodeStats = peer.nodes[nodeIt->second.name];
				nodeStats.active = true;

				auto conIt = nodeStats.connections.find(connection.id);
				if(conIt != nodeStats.connections.end())
				{
					auto& cacheCon = conIt->second;
					if(cacheCon.destination != connection.destination
						|| cacheCon.topic != connection.topic
						|| cacheCon.transport != connection.transport)
					{
						ROS_WARN("connection does not match cached connection, strange.");
						nodeStats.connections.erase(conIt);
						conIt = nodeStats.connections.end();
					}
				}

				if(conIt == nodeStats.connections.end())
				{
					conIt = nodeStats.connections.insert(
						std::make_pair(connection.id, connection)
					).first;
				}

				auto& cacheCon = conIt->second;

				if(cacheCon.active)
				{
					ROS_WARN("I got two stat responses for one connection, ignoring...");
					continue;
				}

				cacheCon.updateBytes(static_cast<int>(link[1]), dt);
				cacheCon.active = true;
			}
		}
	}

	void updateStats(const ros::WallDuration& dt)
	{
		for(auto& peer : m_peers)
		{
			peer.second.active = false;
			for(auto& node : peer.second.nodes)
			{
				node.second.active = false;
			}
		}

		for(auto& node : m_nodes)
		{
			const auto& name = node.first;
			const auto& nodeInfo = node.second;

			if(nodeInfo.host != ros::network::getHost())
				continue;

			ROS_INFO(" - %s at %s", name.c_str(), nodeInfo.uri.c_str());

			updateNodeStats(nodeInfo, dt);
		}
	}
private:
	std::map<std::string, NodeInfo> m_nodes;
	std::map<std::string, Peer> m_peers;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "net_monitor");

	XmlRpc::XmlRpcValue response;
	XmlRpc::XmlRpcValue payload;
	if(!ros::master::execute("getSystemState", "net_monitor", response, payload, false))
	{
		ROS_ERROR("Could not query system state from master");
		return 1;
	}

	std::set<std::string> nodeNames;

	for(int i = 0; i < payload.size(); ++i)
	{
		auto advertisement = payload[i];

		for(int j = 0; j < advertisement.size(); ++j)
		{
			auto item = advertisement[j];

			auto pubs = item[1];

			for(int k = 0; k < pubs.size(); ++k)
				nodeNames.insert(pubs[k]);
		}
	}

	std::map<std::string, NodeInfo> nodes;
	for(auto& name : nodeNames)
	{
		try
		{
			nodes[name] = NodeInfo::fromName(name);
		}
		catch(NodeInfo::LookupException)
		{
			continue;
		}
	}



	return 0;
}
