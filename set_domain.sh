#!/bin/bash

read -p "ROS_DOMAIN_ID [0]: " domain_id
domain_id=${domain_id:-0}

case $domain_id in
  28)
    ip=192.168.50.75
    ;;
  58)
    ip=192.168.50.8
    ;;
  10)
    ip=192.168.50.65
    ;;
  46)
    ip=192.168.50.220
    ;;
  77)
    ip=192.168.50.135
    ;;
  21)
    ip=192.168.50.101
    ;;
  53)
    ip=192.168.50.211
    ;;
  50)
    ip=192.168.50.156
    ;;
  34)
    ip=192.168.50.98
    ;;
  * )
    echo "No matching domain"
    exit 0
    ;;
esac
discovery_ip=$ip
discovery_port=11811

#:<<"END"
#read -p "RPi4 IP address: " ip
#read -p "Discovery Server IP [$ip]: " discovery_ip
#discovery_ip=${discovery_ip:-$ip}
#read -p "Discovery Server Port [11811]: " discovery_port
# discovery_port=${discovery_port:-11811}

echo "Configuring:"
echo " ip route add 192.168.186.0/24 via $ip";
echo " ROS_DISCOVERY_SERVER=$discovery_ip:$discovery_port"
echo " ROS_DOMAIN_ID=$domain_id"

# Delete existing route if applicable
if [ -f "/usr/local/sbin/ip_route.sh" ]; then
   ip_add=$(grep -o 'ip [^"]*' /usr/local/sbin/ip_route.sh)
   ip_del=$(echo $ip_add | sed "s/add/del/g")
   echo $(sudo $ip_del)
fi

# Make directory to hold configs
sudo mkdir -p /etc/turtlebot4_discovery/

# Clone turtlebot4_setup and install files
sudo apt install git -y
git clone -b humble https://github.com/turtlebot/turtlebot4_setup.git /tmp/turtlebot4_setup/ &> /dev/null
sudo mv /tmp/turtlebot4_setup/turtlebot4_discovery/ip_route.sh /usr/local/sbin/
sudo mv /tmp/turtlebot4_setup/turtlebot4_discovery/ip_route.service /etc/systemd/system/
sudo mv /tmp/turtlebot4_setup/turtlebot4_discovery/fastdds_discovery_super_client.xml /etc/turtlebot4_discovery/
sudo mv /tmp/turtlebot4_setup/turtlebot4_discovery/setup.bash /etc/turtlebot4_discovery/
rm /tmp/turtlebot4_setup/ -rf

# Modify IP address
sudo sed -i "s/10.42.0.1/$ip/g" /usr/local/sbin/ip_route.sh
sudo sed -i "s/10.42.0.1/$discovery_ip/g" /etc/turtlebot4_discovery/fastdds_discovery_super_client.xml
sudo sed -i "s/11811/$discovery_port/g" /etc/turtlebot4_discovery/fastdds_discovery_super_client.xml
sudo sed -i "s/10.42.0.1/$discovery_ip/g" /etc/turtlebot4_discovery/setup.bash
sudo sed -i "s/11811/$discovery_port/g" /etc/turtlebot4_discovery/setup.bash
sudo sed -i "s/ROS_DOMAIN_ID=0/ROS_DOMAIN_ID=$domain_id/g" /etc/turtlebot4_discovery/setup.bash

# Source setup.bash in .bashrc
if ! grep -Fq "source /etc/turtlebot4_discovery/setup.bash" ~/.bashrc
then
    echo 'source /etc/turtlebot4_discovery/setup.bash' >> ~/.bashrc
fi

# Enable and start IP route service
sudo systemctl enable ip_route.service
sudo systemctl restart ip_route.service

echo "Source your ~/.bashrc file and restart ros2 daemon to apply changes"
echo "'source ~/.bashrc && ros2 daemon stop && ros2 daemon start'"
#END
