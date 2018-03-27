rostopic pub --once /operator/human_chatter std_msgs/String "data: 'start the camera and show it in rviz'"
rostopic pub --once /operator/human_chatter std_msgs/String "data: 'add the cylinder'"
rostopic pub --once /operator/human_chatter std_msgs/String "data: 'track the cylinder and show it in rviz'"
rostopic pub --once /operator/human_chatter std_msgs/String "data: 'start the robot'"
sleep 8
rostopic pub --once /operator/human_chatter std_msgs/String "data: 'show the manipulation'"
rostopic pub --once /operator/human_chatter std_msgs/String "data: 'set the target'"

