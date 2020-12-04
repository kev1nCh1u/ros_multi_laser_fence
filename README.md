# multi laser

## chmod
    sudo chmod 777 /dev/ttyACM0
    sudo chmod 777 /dev/ttyACM1

## start
    rosrun hokuyo_node hokuyo_node
    rosrun hokuyo_node hokuyo_node_2
    rosrun ira_laser_tools laserscan_multi_merger
    rosrun rviz rviz -d /home/user/ros/multi_laser/src/20201204.rviz
    rosrun rqt_reconfigure rqt_reconfigure

## launch start
    roslaunch launch_start start.launch

## rosparam
    rosparam set hokuyo_port_1 /dev/ttyACM0
    rosparam set hokuyo_port_1 /dev/ttyACM1