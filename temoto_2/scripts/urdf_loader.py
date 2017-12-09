#!/usr/bin/env python

import rospy
import xacro
import sys
from cStringIO import StringIO

def myhook():
      print("Removing robot_description from parameter server")
      rospy.delete_param("robot_description")

rospy.on_shutdown(myhook)

def loader():
    rospy.init_node('urdf_loader', anonymous=True)

    print("Parsing xacro")
    sys.stdout = xacro_out = StringIO()
    xacro.main()

    
    sys.stdout = sys.__stdout__
#    print(xacro_out.getvalue())
    rospy.set_param('robot_description', xacro_out.getvalue())

    rospy.spin()

if __name__ == '__main__':
    try:
        loader()
    except rospy.ROSInterruptException:
        pass 
