import rospy
import dynamic_reconfigure.client

_joint_list = ['Shoulder_R:0','Shoulder_R:2','Arm_Twist_R:1','Elbow_R:0','Forearm_Twist_R:1','Wrist_R:2','Wrist_R:0','Index_Fing_Base_R:0','Mid_Base_R:0',
'Ring_Base_R:0','Pinky_Base_R:0','Thumb_Base_R:0','Thumb_Pivot_R:1','Thumb_Pivot_R:0','Shoulder_L:0','Shoulder_L:2','Arm_Twist_L:1','Elbow_L:0','Forearm_Twist_L:1',
'Wrist_L:2','Wrist_L:0','Index_Fing_Base_L:0','Mid_Base_L:0','Ring_Base_L:0','Pinky_Base_L:0','Thumb_Base_L:0','Thumb_Pivot_L:1','Thumb_Pivot_L:0', 'Body:0', 'Body:2']

def main():
    rospy.init_node("dynamic_client")
    client = dynamic_reconfigure.client.Client('/hr/control/arms')
    print(client.get_configuration()['arms_mode'])

if __name__ == '__main__':
    main()