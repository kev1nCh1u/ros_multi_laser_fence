# multi laser

## chmod
    sudo chmod 777 /dev/ttyACM0
    sudo chmod 777 /dev/ttyACM1

## all start
    roslaunch launch_start start.launch

## start hokuyo_node
    rosrun hokuyo_node hokuyo_node
    rosrun hokuyo_node hokuyo_node_2
    rosrun ira_laser_tools laserscan_multi_merger
    rosrun rviz rviz -d /home/user/ros/multi_laser/src/20201204.rviz
    rosrun rqt_reconfigure rqt_reconfigure

## start urg_node
    roslaunch urg_node multi_node.launch
    rosrun ira_laser_tools laserscan_multi_merger
    rosrun rviz rviz -d /home/user/ros/multi_laser/src/20201207.rviz
    rosrun rqt_reconfigure rqt_reconfigure

## rosparam
    rosparam set hokuyo_port_1 /dev/ttyACM0
    rosparam set hokuyo_port_1 /dev/ttyACM1

## sound debug
    rostopic pub /robotsound sound_play/SoundRequest "{sound: -2, command: 2, volume: 1.0, arg: '/home/user/ros/multi_laser/src/audio_common/sound_play/sounds/excuse_me_ryotsu.wav', arg2: ''}"

## debug
    rosrun rqt_tf_tree rqt_tf_tree