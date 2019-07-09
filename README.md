nimbro_network - ROS transport for high-latency, low-quality networks
=====================================================================

`nimbro_network` is a set of [ROS][1] packages for transporting ROS topics
and services over network. It was developed for the
[DLR SpaceBotCup competition][2] and performed very well during the competition.
Our team was one of the few teams which did not have communication problems.

The SpaceBotCup network had a few special aspects which forced us to design
our own network solution, in particular a two-second delay in each direction.

The system was also used extensively in the [DARPA Robotics Challenge][3], in
which our team NimbRo Rescue achieved the fourth place.

[1]: http://www.ros.org
[2]: http://www.dlr.de/rd/desktopdefault.aspx/tabid-8101/13875_read-35268/
[3]: http://www.theroboticschallenge.org/

Why?
----

ROS has a network transparency layer. But it has issues, namely:

* For subscription, a lengthy TCP handshake is required, even if you want to
  use the UDP transport. If you lose the connection, you have to re-do the
  handshake, possibly taking a long time
* No compression
* ROS service calls need several handshakes for each call
* Messages are transmitted at the rate at which they are published

Our network stack offers the same functions as the ROS network transparency,
but addresses each of the above issues.

Alternatives
------------

`nimbro_network` offers robust transport of ROS topics and services over
unreliable networks. For high-level features like auto-discovery, job scheduling
etc. take a look at alternatives like [rocon][rocon] or
[multimaster_fkie][multimaster_fkie].

[rocon]: http://www.robotconcert.org
[multimaster_fkie]: https://fkie.github.io/multimaster_fkie/

Features
--------

* Topic transport:
    * TCP protocol for transmission guarantee
      (still with communication timeouts!)
    * UDP protocol for streaming data (data which has no meaning if it
      arrives late)
    * Optional transparent BZip2 compression using libbz2
    * Automatic topic discovery on the receiver side. The transmitter defines
      which topics get transferred
    * Optional rate-limiting for each topic
    * Experimental Forward Error Correction (FEC) for the UDP transport
* Service transport:
    * TCP protocol with minimal latency (support for TCP Fast-Open is included)
    * UDP protocol
* Additional nodes:
    * Special nimbro_log_transport node for transporting the ROS log over a
      lossy connection
    * Special tf_throttle node for creating & transferring TF snapshots at pre-
      defined intervals.
    * Special nimbro_cam_transport package for encoding/decoding camera images
      to/from H.264

Setting machines
---------------
#### Enable multicast
For all computers, first check if the multicast feature is enabled using the following command.
```bash
cat /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts
```

If this command returns 0, the multicast feature is enabled and you can go ahead to the next section. 
To temporary enable the multicast feature, execute the following command, however, when the computer restarts, this configuration will be lost.
```bash
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"
```

To permanently enable the multicast feature, edit the /etc/sysctl.conf file and add the following line, or uncomment it, if it already exists, and change its default value.
```bash
net.ipv4.icmp_echo_ignore_broadcasts=0
```

In order for the changes to take effect, execute the following command:
```bash
sudo service procps restart
```

To check which multicast groups are already defined in a computer, execute the following command.
```bash
netstat -g
```

This command will report all the IP addresses enabled for multicast for each of the network interfaces available, both for IPv4 and IPv6. The standard IP address for multicast is 224.0.0.1, that should appear on the list provided by the last command, and it is the one we will use.

At this point, to check whether the multicast feature is working or not, execute the following command, at any computer.
```bash
ping 224.0.0.1
```

If everything is configured properly, you should get a reply from each computer in the common network at each iteration.

#### Host name and IP address binding
For all computers on a local ROS network, and for all ROS networks, modify the /etc/hosts file using your favorite text editor. The name of each host should be the same as its machine name.




Getting started
---------------

See nimbro_topic_transport/README.md and nimbro_service_transport/README.md
for documentation on the topic and service transport.

State
-----

`nimbro_network` is mature in the sense that it has been used extensively in
the preparation and during the competition of the SpaceBotCup and in the
DARPA Robotics Challenge.

In the DRC, the software was used for the high-bandwidth link. Communication
over the low-bandwidth link was handled by custom, highly specific code, which
is not released at this point.

License
-------

`nimbro_network` is licensed under the BSD 3-clause license.
This repository includes the [QCustomPlot][4] library, which is licensed under
the GPLv3 license.

[4]: http://www.qcustomplot.com

Authors & Contact
-----------------

```
Max Schwarz <max.schwarz@uni-bonn.de>
Institute of Computer Science VI
Rheinische Friedrich-Wilhelms-Universität Bonn
Friedrich Ebert-Allee 144
53113 Bonn
```

