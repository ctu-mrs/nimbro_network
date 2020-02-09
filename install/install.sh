#!/bin/bash

SYSCTL_FILE="/etc/sysctl.conf"

# Install dependencies
echo "Installing dependencies"
sudo apt -y install ros-melodic-catch-ros libx264-dev libzstd-dev libqcustomplot-dev

# Check whether multicast is enabled
multicast_enabled=`cat /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts`
if [[ "$multicast_enabled" == "1" ]]; then

  echo "Enabling multicast temporarily"
  sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"

  echo "Enabling multicast permanently"
  sudo sh -c "echo net.ipv4.icmp_echo_ignore_broadcasts=0>>$SYSCTL_FILE"
  
  echo "Restarting procps service"
  sudo service procps restart
else 
  echo "Multicast is already enabled"
fi

# Set broadcast IP address
~/git/uav_core/ros_packages/mrs_general/scripts/set_broadcast_rc_variable.sh

# Populate /etc/hosts
~/git/uav_core/miscellaneous/dotssh/generate_ssh_config.sh

echo "Done"

