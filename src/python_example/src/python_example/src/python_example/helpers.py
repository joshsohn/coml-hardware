import numpy as np
from geometry_msgs.msg import Point, Quaternion, Vector3

# convert from ROS message to array
def point_msg_to_array(point_msg):
    return np.array([point_msg.x, point_msg.y, point_msg.z])

def vector_msg_to_array(vector_msg):
    return np.array([vector_msg.x, vector_msg.y, vector_msg.z])

def quaternion_msg_to_quaternion(quaternion_msg):
    return np.array([quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w])

# convert from array to ROS message
def point_array_to_msg(point_array):
    return Point(x=point_array[0], y=point_array[1], z=point_array[2])

def vector_array_to_msg(vector_array):
    return Vector3(x=vector_array[0], y=vector_array[1], z=vector_array[2])

def quaternion_array_to_msg(quaternion_array):
    return Quaternion(x=quaternion_array[0], y=quaternion_array[1], z=quaternion_array[2], w=quaternion_array[3])