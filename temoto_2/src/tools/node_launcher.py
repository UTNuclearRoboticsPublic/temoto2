#!/usr/bin/env python
# license removed for brevity

import rospy
import roslaunch
import time
import psutil
import subprocess
from temoto_2.srv import *

# a dictionary of spawned nodes
running_nodes = {}
running_processes = {}

def spawn_kill_cb(action, package, name):
	if action == "run":
		print "[node_manager] Received a run request"

		# add a new entry to the dictionary
		node = roslaunch.core.Node(package, name)
		
		print "[node_manager] debug 1"
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()
		
		print "[node_manager] debug 2"
		running_nodes[name] = launch.launch(node)
		#process = launch.launch(node)

		print "[node_manager] debug 3"
		print running_nodes[name].is_alive()

		print "[node_manager] Started:" + name

	elif action == "kill":
		print "[node_manager] Received a kill request"

		running_nodes[name].stop()
		del running_nodes[name]
	
	elif action == "test":
		cmd = ["roslaunch", "temoto_2", "test_1.launch"]
		running_processes["test"] = subprocess.Popen(cmd, stdout=subprocess.PIPE)
		print "PID = ", running_processes["test"].pid
		#time.sleep(2.0)
		#p.kill()
		#p.wait()
		
	elif action == "ktest":
		running_processes["test"].kill()
		running_processes["test"].wait()
		del running_processes["test"]


if __name__ == '__main__':
  try:
		rospy.init_node('node_manager')
		#srv = rospy.Service('spawn_kill', node_spawn_kill, spawn_kill_cb)
		print "[node_manager] node_spawn_kill service up an running."
		
		while True:
			cmd = input('Enter start or kill: ')

			if cmd == "print":
				for k in running_processes.keys():
					print k
			
			else:
				spawn_kill_cb(cmd, "temoto_2", "language_input")
		
		#rospy.spin()

  except rospy.ROSInterruptException:
    pass

