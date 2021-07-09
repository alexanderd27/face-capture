import bpy
import math
import rospy
from hr_msgs.msg import pau


_joint_list = ['Shoulder_R:0','Shoulder_R:2','Arm_Twist_R:1','Elbow_R:0','Forearm_Twist_R:1','Wrist_R:2','Wrist_R:0','Index_Fing_Base_R:0','Mid_Base_R:0',
'Ring_Base_R:0','Pinky_Base_R:0','Thumb_Base_R:0','Thumb_Pivot_R:1','Thumb_Pivot_R:0','Shoulder_L:0','Shoulder_L:2','Arm_Twist_L:1','Elbow_L:0','Forearm_Twist_L:1',
'Wrist_L:2','Wrist_L:0','Index_Fing_Base_L:0','Mid_Base_L:0','Ring_Base_L:0','Pinky_Base_L:0','Thumb_Base_L:0','Thumb_Pivot_L:1','Thumb_Pivot_L:0', 'Body:0', 'Body:2']


class StartNodeButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_node"
    bl_label = "Start Node"
    def execute(self, context):
        try: 
            rospy.init_node("arms_capture")
            print('Node Started')
        except Exception as e:
            self.report({"WARNING"}, "Can\'t start the node with exception {}".format(e))
        return {'FINISHED'}


class RecordPoseButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_record"
    bl_label = "Start Recording"

    #Set fps of recording
    fps = 1

    msgs = []
    recording = False
    counter = 0

    def execute(self, context):
        self.recording = True
        print('Recording Started')
        rospy.Subscriber("/hr/actuators/current_state", pau, self.callback())
        rospy.spin()
        return {'FINISHED'}
    
    def callback(self, msg):
        if self.recording and self.counter % 50/self.fps == 0:
            self.msgs.append(msg)
        self.counter += 1


class StopRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.stop_record"
    bl_label = "Stop Recording"

    def execute(self, context):
        RecordPoseButtonOperator.recording = False
        for i in range(len(RecordPoseButtonOperator.msgs)):
            msg = RecordPoseButtonOperator.msgs[i]
            pose = dict(zip(_joint_list, msg))
            frame = i * 48/RecordPoseButtonOperator.fps
            for bone, angle in pose.items():
                try:
                    self.set_angle(frame, bone, angle)
                except:
                    pass
        RecordPoseButtonOperator.msgs.clear()
        RecordPoseButtonOperator.counter = 0
        print('Recording Stopped')
        return {'FINISHED'}

    def set_angle(frame, bone, angle):
        (path,index) = bone.split(':')
        angle = math.radians(angle)
        index = int(index)
        data_path = "poses.bones[%s].rotation_euler" %path
        action = bpy.context.object.animation_data.action
        try:
            if data_path not in [fc.data_path for fc in bpy.context.object.animation_data.action.fcurves]:
                fc = action.fcurves.new(data_path, index)
                fc.keyframe_points.insert(frame, angle)
                print("inserted {} - {} - {} - {}".format(frame, data_path, index, angle))
            else:
                fc = action.fcurves.find(data_path, index)
                fc.keyframe_points.insert(frame, angle)
                print("inserted {} - {} - {} - {}".format(frame, data_path, index, angle))
        except Exception as e:
            print(e)


class ArmCapturePanel(bpy.types.Panel):
    bl_label = "Convertion"
    bl_idname = "scene.convert_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(self, context):
        return True

    def draw(self, context):
        layout = self.layout
        layout.operator(StartNodeButtonOperator.bl_idname)
        layout.operator(RecordPoseButtonOperator.bl_idname)
        layout.operator(StopRecordButtonOperator.bl_idname)


bl_info = {"name": "Arm Capture", "category": "User"}

def register():
    bpy.utils.register_class(StartNodeButtonOperator)
    bpy.utils.register_class(RecordPoseButtonOperator)
    bpy.utils.register_class(StopRecordButtonOperator)
    bpy.utils.register_class(ArmCapturePanel)

def unregister():
    bpy.utils.unregister_class(StartNodeButtonOperator)
    bpy.utils.unregister_class(RecordPoseButtonOperator)
    bpy.utils.unregister_class(StopRecordButtonOperator)
    bpy.utils.unregister_class(ArmCapturePanel)


if __name__ == "__main__":
    register()
