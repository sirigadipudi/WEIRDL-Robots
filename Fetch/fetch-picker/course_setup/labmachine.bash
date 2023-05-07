# Source this file at the end of your .bashrc on the lab machines
source /opt/ros/noetic/setup.bash

# If you're on a laptop, change eth0 to wlan0
function my_ip() {
    MY_IP=$(/sbin/ifconfig wlo1 | awk '/inet/ { print $2 } ' | sed -e s/addr://)
    echo ${MY_IP:-"Not connected"}
}

export ROS_HOSTNAME=localhost
export ROS_MASTER_HOST=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROBOT=sim
export ROSCONSOLE_FORMAT='${node} ${function}:${line}: ${message}'

PS1='\[\e[1;35m\][\h \w ($ROS_MASTER_HOST)]$ \[\e[m\]'

function setrobot() {
  unset ROBOT;
  unset ROS_HOSTNAME;
  export ROS_MASTER_HOST=$1;
  export ROS_MASTER_URI=http://$1.cs.washington.edu:11311;
  export ROS_IP=`my_ip`;
}
