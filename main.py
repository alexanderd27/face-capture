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


_joint_list = ['Shoulder_R:0','Shoulder_R:2','Arm_Twist_R:1','Elbow_R:0','Forearm_Twist_R:1','Wrist_R:2','Wrist_R:0','Index_Fing_Base_R:0','Mid_Base_R:0',
'Ring_Base_R:0','Pinky_Base_R:0','Thumb_Base_R:0','Thumb_Pivot_R:1','Thumb_Pivot_R:0','Shoulder_L:0','Shoulder_L:2','Arm_Twist_L:1','Elbow_L:0','Forearm_Twist_L:1',
'Wrist_L:2','Wrist_L:0','Index_Fing_Base_L:0','Mid_Base_L:0','Ring_Base_L:0','Pinky_Base_L:0','Thumb_Base_L:0','Thumb_Pivot_L:1','Thumb_Pivot_L:0', 'Body:0', 'Body:2']


class RecordPoseButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_record"
    bl_label = "Record Pose"

    def execute(self, context):
        import rospy
        from hr_msgs.msg import pau
        rospy.init_node("arms_capture")
        global start_time
        start_time = rospy.get_time()
        recording = True
        while recording:
            rospy.Subscriber("/hr/actuators/current_state", pau, self.callback)
            rospy.spin()
        return {'FINISHED'}
    
    def callback(self, msg):
        pose = dict(zip(_joint_list, msg.m_angles))
        global start_time
        frame = (rospy.get_time() - start_time) * bpy.context.scene.render.fps
        for bone,angle in pose.items():
            try:
                self.set_angle(frame, bone, angle)
            except:
                pass

    def set_angle(self, frame, bone, angle):
        (path,index) = bone.split(':')
        angle = math.radians(angle)
        index = int(index)
        fc = bpy.context.object.animation_data.action.fcurves.find("\'pose.bones[\"" + path + "\"].rotation_euler\'", index)
        fc.keyframe_points.insert(frame, angle)


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
        layout.operator(RecordPoseButtonOperator.bl_idname)

bl_info = {"name": "Arm Capture", "category": "User"}

def register():
    bpy.utils.register_class(RecordPoseButtonOperator)
    bpy.utils.register_class(ArmCapturePanel)


def unregister():
    bpy.utils.unregister_class(RecordPoseButtonOperator)
    bpy.utils.unregister_class(ArmCapturePanel)

if __name__ == "__main__":
    register()
