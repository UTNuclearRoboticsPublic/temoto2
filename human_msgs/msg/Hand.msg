# For position, Leap Motion Controller uses '_optical' axis orientation, i.e.:
#    z forward
#    x right
#    y down

# TRUE if this hand is a valid hand (left or right), FALSE otherwise.
bool is_present					

# Contains the center position of the palm in meters from the Leap 
#Motion Controller origin and the quaternion orientation of the palm.
geometry_msgs/PoseStamped palm_pose		

# The rate of change of the palm position in meters per second. 
geometry_msgs/Vector3Stamped palm_velocity	

# The radius of a sphere fit to the curvature of this hand. This sphere
# is placed roughly as if the hand were holding a ball. Thus the size 
# of the sphere decreases as the fingers are curled into a fist.
float32 sphere_radius				

# The strength is zero for an open hand, and blends to 1.0 when a 
# pinching hand pose is recognized.
float32 pinch_strength			

# TRUE if KEY_TAP gesture has been detected on this hand, FALSE otherwise.
bool key_tap
