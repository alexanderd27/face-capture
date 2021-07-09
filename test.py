import rospy
from hr_msgs.msg import pau
def test_main():
    global counter
    counter = 0
    global start_time
    rospy.init_node('test')
    start_time = rospy.get_time()
    print(start_time)
    rospy.Subscriber("/hr/actuators/current_state", pau, test_callback)
    rospy.spin()

def test_callback(msg):
    global counter
    counter += 1
    global start_time
    if counter % 50 == 0:
        print(msg.m_angles)
        time = rospy.get_time() - start_time
        print(time)

if __name__ == '__test__':
    test_main()
