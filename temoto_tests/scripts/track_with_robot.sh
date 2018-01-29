rostopic pub --once /veikumoto/human_chatter std_msgs/String "data: 'add the right hand'"
rostopic pub --once /veikumoto/human_chatter std_msgs/String "data: 'track the right hand and show it in rviz'"
rostopic pub --once /veikumoto/human_chatter std_msgs/String "data: 'start the robot'"
sleep 3
rostopic pub --once /veikumoto/human_chatter std_msgs/String "data: 'show the manipulation'"
rostopic pub --once /veikumoto/human_chatter std_msgs/String "data: 'set the target'"

