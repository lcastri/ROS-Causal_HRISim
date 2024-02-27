# Run as `source connect_tiago.sh TIAGO_NUM ROBOT_IFACE`
# where:
#   - TIAGO_NUM: is the ID number of the robot you are connected
#   - ETH: just pass 1 to connect via ethernet or 0 via TIAGo_WIFI


TIAGO_NUM=$1
ETH=$2
#echo $TIAGO_NUM

# Change ROS Master
if [ "$ETH" -eq 0 ]; then
    echo "Connecting through TIAGo_WIFI"
    export ROS_MASTER=192.168.1.${TIAGO_NUM}
else
    echo "Connecting through Ethernet"
    export ROS_MASTER=10.68.0.1
fi
export ROS_MASTER_URI=http://${ROS_MASTER}:11311
rostopic list &>/dev/null
RETVAL=$?
if [ $RETVAL -ne 0 ]; then
    echo "[ERROR] connection with ROS MASTER not enstablished"
else
    echo "[OK] Connected to ROS MASTER"
fi

# Add forwarding address to DNS
#echo $ROBOT_IFACE
#THIS_IP=`ifconfig ${ROBOT_IFACE} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
THIS_IP=`ip route get ${ROS_MASTER} | grep "src" | sed 's/.*src \([0-9\.]*\).*/\1/'`
export ROS_HOSTNAME=${THIS_IP}
export ROS_IP=${THIS_IP}

sshpass -p "palroot" ssh root@${ROS_MASTER} "addLocalDns -u \"${HOSTNAME}\" -i \"${THIS_IP}\""
