#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

unattended=0
for param in "$@"
do
  echo $param
  if [ $param="--unattended" ]; then
    echo "installing in unattended mode"
    unattended=1
    subinstall_params="--unattended"
  fi
done

SYSCTL_FILE="/etc/sysctl.conf"

echo "$0: Installing nimbro_network package"

# Install dependencies
echo "Installing dependencies"
sudo apt -y install ros-melodic-catch-ros libx264-dev libzstd-dev libqcustomplot-dev

default=n
while true; do
  if [[ "$unattended" == "1" ]]
  then
    resp=y
  else
    [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mEnable multicast permanently? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  fi
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then

    # Check whether multicast is enabled
    multicast_enabled=`cat /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts`
    if [[ "$multicast_enabled" == "1" ]]; then

      echo "Enabling multicast permanently"
      sudo sh -c "echo net.ipv4.icmp_echo_ignore_broadcasts=0>>$SYSCTL_FILE"
      
      echo "Restarting procps service"
      sudo service procps restart
    else 
      echo "Multicast is already enabled"
    fi

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    while true; do
      [[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mEnable multicast temporarily? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
      response=`echo $resp | sed -r 's/(.*)$/\1=/'`

      if [[ $response =~ ^(y|Y)=$ ]]
      then
        echo "Enabling multicast temporarily"
        sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"

        echo "Restarting procps service"
        sudo service procps restart

        break
      elif [[ $response =~ ^(n|N)=$ ]]
      then
        break
      else
        echo " What? \"$resp\" is not a correct answer. Try y+Enter."
      fi
    done
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi

done

echo "$0: Done"

