# Nimbro Network - ROS transport for high-latency, low-quality networks

| Build status | [![Build Status](https://github.com/ctu-mrs/nimbro_network/workflows/Melodic/badge.svg)](https://github.com/ctu-mrs/nimbro_network/actions) | [![Build Status](https://github.com/ctu-mrs/nimbro_network/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/nimbro_network/actions) |
|--------------|---------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|

`nimbro_network` is a set of ROS packages for transporting ROS topics and services over the network. 

## Automatic installation

Allow multicast on each device by running `install/install.sh`.

## Manual installation

### Dependencies

```bash
sudo apt install ros-melodic-catch-ros libx264-dev libzstd-dev libqcustomplot-dev
```

### Enable multicast
For all computers, first, check if the multicast feature is enabled using the following command.
```bash
cat /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts
```

If this command returns 0, the multicast feature is enabled, and you can go ahead to the next section. 
To temporarily enable the multicast feature, execute the following command, however, when the computer restarts, this configuration will be lost.
```bash
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"
```

To permanently enable the multicast feature, edit the `etc/sysctl.conf` file and add the following line, or uncomment it, if it already exists, and change its default value.
```bash
net.ipv4.icmp_echo_ignore_broadcasts=0
```

For the changes to take effect, execute the following command:
```bash
sudo service procps restart
```

At this point, to check whether the multicast feature is working, execute the following command at any computer.
```bash
ping 192.168.0.255
```

If everything is configured correctly, you should get a reply from each computer in the shared network at each iteration.

### Hostname and IP address binding
For all computers on a local ROS network and all ROS networks, modify the `/etc/hosts` file using your favorite text editor. The name of each host should be the same as its machine name.

### Setting the broadcast address in the launch file

* Find out the broadcast ip address of the network using ifconfig
* Set the broadcast address in `mrs_general/launch/nimbro.launch` to: 
  ```xml
  <arg name="broadcast_addr" default="192.168.69.255" />
  ```

## Why?

ROS has a network transparency layer. But it has issues, namely:

* For subscription, a lengthy TCP handshake is required, even when using UDP transport. If you lose the connection, you have to re-do the
  handshake, possibly taking a long time
* No compression
* ROS service calls need several handshakes for each call
* Messages are transmitted at the rate at which they are published

Our network stack offers the same functions as the ROS network transparency,
but addresses each of the above issues.

## Features

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
* Additional ROS nodes:
    * Special nimbro_log_transport node for transporting the ROS log over a
      lossy connection
    * Special tf_throttle node for creating & transferring TF snapshots at pre-
      defined intervals.
    * Special nimbro_cam_transport package for encoding/decoding camera images
      to/from H.264
