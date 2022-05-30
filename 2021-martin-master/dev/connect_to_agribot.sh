# Connect to agribot
COMPUTER_IP=$(ifconfig | grep "inet " | grep -v "127.0.0.1" | tail -1 | awk '{print $2}')
export COMPUTER_IP=$COMPUTER_IP
export AGRIBOT_IP=$(getent ahosts 'agribot' | grep "192\.\|10\.\|172\." | head -1 | awk '{print $1}')
USED_IP=$AGRIBOT_IP
export ROS_MASTER_URI=http://$USED_IP:11311/
ssh ubuntu@$AGRIBOT_IP
