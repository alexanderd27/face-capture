import rospy
from hr_msgs.msg import pau
def main():
    rospy.init_node('test')
    rospy.Subscriber("/hr/actuators/current_state", pau, callback)
    rospy.spin()

def callback(msg):
    print(dir(msg))

if __name__ == '__main__':
    main()
