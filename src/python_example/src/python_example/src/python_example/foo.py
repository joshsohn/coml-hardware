print("  importing bar_fn, baz_fn from simon_pythonsubpackage1")
from .simon_pythonsubpackage1 import bar_fn, baz_fn
print("  imported bar_fn, baz_fn from simon_pythonsubpackage1")
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received: %s", data.data)

def main():
    # Initialize the ROS node with a default name
    rospy.init_node('my_node_name')

    # Node logic
    rospy.Subscriber('state', String, callback)

    # Spin to keep the node alive and process callbacks
    rospy.spin()

if __name__ == '__main__':
    print(" executing main() in foo.py")
    bar_fn()
    baz_fn()
    try:
        main()
    except rospy.ROSInterruptException:
        pass

