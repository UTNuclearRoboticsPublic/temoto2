#########################
#   STATIC PROPERTIES   #
#########################
# Static properties define the object

# REQUIRED - Name of the object
string name

# REQUIRED - How this object can be detected
uint8[] detection_methods

# Supported types for object detection are: 
uint8 ARTAG = 1     # Requires the "tag_id" field to be filled
uint8 SEGMENT = 2   # Requires the "primitive" field to be filled
uint8 TEMPLATE = 3  # Requires the "mesh" field to be filled

# REQUIRED IF this object can be detected by a tag
int16 tag_id

# The pose of the object relative to the AR tag - If this is
# unspecified then it is (probably falsely) assumed, that the
# pose of detected tag is same as the pose of the object
geometry_msgs/Pose obj_relative_pose

# Describes the shape primitive (e.g., box, cylinder, cone, sphere)
shape_msgs/SolidPrimitive primitive
shape_msgs/Mesh mesh

##########################
#   DYNAMIC PROPERTIES   #
##########################
# Dynamic properties indicate the state of the object

geometry_msgs/PoseStamped pose

