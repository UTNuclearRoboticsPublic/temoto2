# Specifies whether the sensor should be added, updated or removed.
string sync_action

# Temoto namespace where the Sensor manager belongs to.
string temoto_namespace

# Sensor name
string sensor_name

# The topic on which the sensor publishes its data. 
string topic

# Sensor type (camera, text, speech, ...)
string sensor_type

# Temoto namespace where the Sensor manager belongs to.
string package_name

# Name of the rosrun or roslaunch file.
string executable

# A brief description about this sensor.
string description

# Reliability value
float32 reliability
