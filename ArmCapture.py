import bpy
import math
from bpy.props import *
from bpy_extras.io_utils import ImportHelper
from bpy.props import (
        StringProperty,
        CollectionProperty,
        BoolProperty
        )
from bpy.types import (
        Operator,
        OperatorFileListElement,
        )
import os
# from the reference files:
last_msg = None
def msg_cb(msg):
    print('ok')
    global last_msg
    last_msg = msg

ARM_ROTATIONS = {
    'R_Shoulder_Pitch'  : 'Shoulder_R:0',
    'R_Shoulder_Roll'   : 'Shoulder_R:2',
    'R_Shoulder_Yaw'    : 'Arm_Twist_R:1',
    'R_Elbow'           : 'Elbow_R:0',
    'R_Wrist_Yaw'       : 'Forearm_Twist_R:1',

    'R_Wrist_Roll'      : 'Wrist_R:2',
 #   'R_Wrist_Pitch'     : 'Wrist_R:0',
    'R_Index_Finger'    : 'Index_Fing_Base_R:0',
    'R_Middle_Finger'   : 'Mid_Base_R:0',
    'R_Ring_Finger'     : 'Ring_Base_R:0',
    'R_Pinky_Finger'    : 'Pinky_Base_R:0',
    'R_Thumb_Finger'    : 'Thumb_Base_R:0',
    'R_Thumb_Roll'      : 'Thumb_Pivot_R:1',
    'R_Spreading'       : 'Thumb_Pivot_R:0',

    'L_Shoulder_Pitch'  : 'Shoulder_L:0',
    'L_Shoulder_Roll'   : 'Shoulder_L:2',
    'L_Shoulder_Yaw'    : 'Arm_Twist_L:1',
    'L_Elbow'           : 'Elbow_L:0',
    'L_Wrist_Yaw'       : 'Forearm_Twist_L:1',

    'L_Wrist_Roll'      : 'Wrist_L:2',
#    'L_Wrist_Pitch'     : 'Wrist_L:0',
    'L_Index_Finger'    : 'Index_Fing_Base_L:0',
    'L_Middle_Finger'   : 'Mid_Base_L:0',
    'L_Ring_Finger'     : 'Ring_Base_L:0',
    'L_Pinky_Finger'    : 'Pinky_Base_L:0',
    'L_Thumb_Finger'    : 'Thumb_Base_L:0',
    'L_Thumb_Roll'      : 'Thumb_Pivot_L:1',
    'L_Spreading'       : 'Thumb_Pivot_L:0',
    'Body_Pitch'        : 'Body:0',
    'Body_Roll'        : 'Body:2',
}
_joint_list = ['R_Shoulder_Pitch','R_Shoulder_Roll','R_Shoulder_Yaw','R_Elbow','R_Wrist_Yaw','R_Wrist_Roll','R_Wrist_Pitch','R_Index_Finger','R_Middle_Finger',
'R_Ring_Finger','R_Pinky_Finger','R_Thumb_Finger','R_Thumb_Roll','R_Spreading','L_Shoulder_Pitch','L_Shoulder_Roll','L_Shoulder_Yaw','L_Elbow','L_Wrist_Yaw',
'L_Wrist_Roll','L_Wrist_Pitch','L_Index_Finger','L_Middle_Finger','L_Ring_Finger','L_Pinky_Finger','L_Thumb_Finger','L_Thumb_Roll','L_Spreading', 'Body_Pitch', 'Body_Roll']

class StartNodetButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_node"
    bl_label = "Start Node"

    def execute(self, context):
        try: 
            import rospy
            from hr_msgs.msg import pau
            rospy.init_node("arms_capture")
            print('start')
        except Exception as e:
            self.report({"WARNING"}, "Cant start the node with exception {}".format(e))
        return {'FINISHED'}

    def msg_cb(self, msg):
        print('OK')
        global last_msg
        last_msg = msg

class CapturePoseButtonOperator(bpy.types.Operator):
    bl_idname = "scene.capture_pose"
    bl_label = "Capture Pose"

    def execute(self, context):
        import rospy
        from hr_msgs.msg import pau
        msg = rospy.wait_for_message("/hr/actuators/current_state", pau)
        pose = dict(zip(_joint_list, msg.m_angles))
        for i,a in pose.items():
            try:
                b=ARM_ROTATIONS[i]
                self.set_angle(b, a)
            except:
        
                pass
        return {'FINISHED'}

    def set_angle(self, bone, angle):
        (b,a) = bone.split(':')
        angle = math.radians(angle)
        a = int(a)
        try:
            bb = bpy.data.objects['AA'].pose.bones[b].rotation_euler
            print("{} - {} - {} - {}".format(b,a,angle, bb))
            if a == 0:
                bpy.data.objects['AA'].pose.bones[b].rotation_euler.x = angle
            if a == 1:
                bpy.data.objects['AA'].pose.bones[b].rotation_euler.y = angle
            if a == 2:
                bpy.data.objects['AA'].pose.bones[b].rotation_euler.z = angle
        except Exception as e:
            print(e)




class ArmCapturePanel(bpy.types.Panel):
    """Creates a Panel in the Object UI window"""
    bl_label = "Convertion"
    bl_idname = "scene.convert_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'


    @classmethod
    def poll(self, context):
        return True


    def draw(self, context):
        layout = self.layout
        layout.operator(StartNodetButtonOperator.bl_idname)
        layout.operator(CapturePoseButtonOperator.bl_idname)

bl_info = {"name": "Arm Capture", "category": "User"}

def register():
    bpy.utils.register_class(StartNodetButtonOperator)
    bpy.utils.register_class(CapturePoseButtonOperator)
    bpy.utils.register_class(ArmCapturePanel)


def unregister():
    bpy.utils.unregister_class(StartNodetButtonOperator)
    bpy.utils.unregister_class(CapturePoseButtonOperator)
    bpy.utils.unregister_class(ArmCapturePanel)

if __name__ == "__main__":
    register()
