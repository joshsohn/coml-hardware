print("  importing bar_fn, baz_fn from simon_pythonsubpackage1")
from .simon_pythonsubpackage1 import bar_fn, baz_fn
print("  imported bar_fn, baz_fn from simon_pythonsubpackage1")
import rospy
from snapstack_msgs.msg import State

def callback(data):
    rospy.loginfo("Received: %s", data)

def main():
    # Initialize the ROS node with a default name
    rospy.init_node('my_node_name')

    # Node logic
    rospy.Subscriber('state', State, callback)

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

