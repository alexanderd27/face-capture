import bpy
import math
import rospy
from hr_msgs.msg import pau
from bpy.props import (
    BoolProperty
)


_joint_list = ['Shoulder_R:0','Shoulder_R:2','Arm_Twist_R:1','Elbow_R:0','Forearm_Twist_R:1','Wrist_R:2','Wrist_R:0','Index_Fing_Base_R:0','Mid_Base_R:0',
'Ring_Base_R:0','Pinky_Base_R:0','Thumb_Base_R:0','Thumb_Pivot_R:1','Thumb_Pivot_R:0','Shoulder_L:0','Shoulder_L:2','Arm_Twist_L:1','Elbow_L:0','Forearm_Twist_L:1',
'Wrist_L:2','Wrist_L:0','Index_Fing_Base_L:0','Mid_Base_L:0','Ring_Base_L:0','Pinky_Base_L:0','Thumb_Base_L:0','Thumb_Pivot_L:1','Thumb_Pivot_L:0', 'Body:0', 'Body:2']


class UIProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.starting_pose = bpy.props.EnumProperty(
        name = "Starting Pose",
        default = 'none',
        items = [('stand', 'Stand', ''),
                 ('sit', 'Sit', ''),
                 ('none', 'None', '')]
    )
    bpy.types.Scene.recorded_arm = bpy.props.EnumProperty(
        name = "Choose Arm",
        default = 'both',
        items = [('right', 'Record Right Arm only', ''),
                 ('left', 'Record Left Arm Only', ''),
                 ('both', 'Record Both Arms', '')]
    )
    bpy.types.Scene.FPS = bpy.props.IntProperty(
        name = "FPS",
        default = 1,
        soft_min = 1,
        soft_max = 50
    )
    bpy.types.Scene.recording_speed = bpy.props.FloatProperty(
        name = "Recording Speed",
        default = 1,
        soft_min = 0.1,
        soft_max = 1,
        step = 10,
        precision = 1
    )


class StartNodeButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_node"
    bl_label = "Start Node"
    def execute(self, context):
        try: 
            rospy.init_node("arms_capture")
            print('Node Started')
            rospy.Subscriber("/hr/actuators/current_state", pau, self.callback)
            print('Subscribed')   
            bpy.ops.wm.global_timer()
            bpy.ops.wm.live_update_pose()
        except Exception as e:
            self.report({"WARNING"}, "Can\'t start the node with exception {}".format(e))
        return {'FINISHED'}

    def callback(self, msg):
        if RecordPoseButtonOperator.recording:
            if RecordPoseButtonOperator.counter == round(50/RecordPoseButtonOperator.fps):
                RecordPoseButtonOperator.msg_angles.append(msg.m_angles)
                RecordPoseButtonOperator.counter = 0
            RecordPoseButtonOperator.counter += 1
        RecordPoseButtonOperator.last_msg = msg

class RecordPoseButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_record"
    bl_label = "Start Recording"

    recording = False
    fps = 1
    counter = 0
    msg_angles = []
    last_msg = None

    def execute(self, context):
        RecordPoseButtonOperator.msg_angles = []
        RecordPoseButtonOperator.counter = 0
        RecordPoseButtonOperator.fps = context.scene.FPS
        RecordPoseButtonOperator.recording = True
        print('Recording Started')
        return {'FINISHED'}
    

class StopRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.stop_record"
    bl_label = "Stop Recording"

    def execute(self, context):
        RecordPoseButtonOperator.recording = False
        action = bpy.data.actions.new("ARM-MAIN-SIT_Recorded")
        for i in range(len(RecordPoseButtonOperator.msg_angles)):
            frame = i * 48/RecordPoseButtonOperator.fps/context.scene.recording_speed
            angles = RecordPoseButtonOperator.msg_angles[i]

            pose = dict(zip(_joint_list, angles))
            del pose['Body:0']
            del pose['Body:2']
            if context.scene.recorded_arm == 'left':
                pose = {b:a for (b,a) in pose.items() if '_L' in b}
            elif context.scene.recorded_arm == 'right':
                pose = {b:a for (b,a) in pose.items() if '_R' in b}

            for bone, angle in pose.items():
                (path,index) = bone.split(':')
                path = "pose.bones[\"%s\"].rotation_euler" %path
                index = int(index)
                angle = math.radians(angle)

                fc = action.fcurves.find(path, index)
                if not fc:
                    fc = action.fcurves.new(path, index)
                if path == 'Wrist_L':
                    fc.keyframe_points.insert(frame, -angle)
                else:
                    fc.keyframe_points.insert(frame, angle)
        
        if context.scene.starting_pose != 'none':
            for fc in action.fcurves:
                data_path = fc.data_path
                array_index = fc.array_index
                if context.scene.starting_pose == 'stand':
                    fc_new = bpy.data.actions["ARM-MAIN-1"].fcurves.find(data_path, array_index)
                elif context.scene.starting_pose == 'sit':
                    fc_new = bpy.data.actions["ARM-MAIN-2"].fcurves.find(data_path, array_index)
                try:
                    fc.keyframe_points.remove(fc.keyframe_points[0])
                    fc.keyframe_points.insert(0, fc_new.keyframe_points[0].co.y)
                    fc.keyframe_points.insert(fc.keyframe_points[-1].co.x + 48/RecordPoseButtonOperator.fps, fc_new.keyframe_points[1].co.y)
                except:
                    pass


        print('Recording Finished')
        return {'FINISHED'}


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
        layout.prop(bpy.context.scene, "FPS")
        layout.prop(bpy.context.scene, "recording_speed")
        layout.prop(bpy.context.scene, "starting_pose")
        layout.prop(bpy.context.scene, "recorded_arm")
        layout.operator(StartNodeButtonOperator.bl_idname)
        layout.operator(RecordPoseButtonOperator.bl_idname)
        layout.operator(StopRecordButtonOperator.bl_idname)
        # layout.operator(BLGlobalTimer.bl_idname)
        # layout.operator(BLUpdatePose.bl_idname)
        


class BLGlobalTimer(bpy.types.Operator):
    """Timer  Control"""
    bl_label = "Global Timer"
    bl_idname = 'wm.global_timer'
    # New property for global timer
    bpy.types.Scene.globalTimBLUpdatePoseerStarted = bpy.props.BoolProperty(name = "globalTimerStarted", default=False)
    #bpy.context.scene['globalTimerStarted'] = False
    # Property to contextget thedefaultTimerHz = 50
    bpy.types.Scene.maxFPS = bpy.props.IntProperty(name = "maxFPS", soft_min = 10, soft_max = 100, default=50)
    #bpy.context.scene['maxFPS'] = 50
    _timer = None
    _maxFPS = 50

    def execute(self, context):
        print('Starting Timer')
        wm = context.window_manager
        self._timer = wm.event_timer_add(1/self._maxFPS, window=context.window)
        bpy.context.scene['globalTimerStarted'] = True
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)
        if event.type == 'TIMER':
            if self._maxFPS != bpy.context.scene['maxFPS']:
                #Add new timer
                wm = context.window_manager
                wm.event_timer_remove(self._timer)
                self._maxFPS = bpy.context.scene['maxFPS']
                self._timer = wm.event_timer_add(1/self._maxFPS, window=context.window)

        return {'PASS_THROUGH'}

    def cancel(self,context):
        print('Stopping Timer')
        if self._timer:
            wm = context.window_manager
            wm.event_timer_remove(self._timer)
        bpy.context.scene['globalTimerStarted'] = False

    @classmethod
    def poll(cls, context):
        return True



class BLUpdatePose(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Update"
    bl_idname = 'wm.live_update_pose'
    bpy.types.Scene.liveUpdatePose = bpy.props.BoolProperty( name = "liveUpdatePose", default=False)
    # bpy.context.scene['liveUpdatePose'] = FalsedefaultTimerHz = 50
    # bpy.context.scene['keepAlive'] = True


    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)

        if event.type == 'TIMER':
            # compute fps
            print('a')

            if RecordPoseButtonOperator.last_msg is not None:
                pose = dict(zip(_joint_list, RecordPoseButtonOperator.last_msg.m_angles))
                for i,a in pose.items():
                    try:
                        self.set_angle(i, a)
                    except:
                
                        pass
        return {'PASS_THROUGH'}

    def set_angle(self, bone, angle):
        (b,a) = bone.split(':')
        angle = math.radians(angle)
        a = int(a)
        try:
            bb = bpy.data.objects['AA'].pose.bones[b].rotation_euler
            print("{} - {} - {} - {}".format(b,a,angle, bb))
            if a == 0:
                if b == 'Wrist_L':
                    bpy.data.objects['AA'].pose.bones[b].rotation_euler.x = -angle
                else:
                    bpy.data.objects['AA'].pose.bones[b].rotation_euler.x = angle
            if a == 1:
                bpy.data.objects['AA'].pose.bones[b].rotation_euler.y = angle
            if a == 2:
                bpy.data.objects['AA'].pose.bones[b].rotation_euler.z = angle
        except Exception as e:
            print(e)

    def execute(self, context):
        print('Starting Update pose')
        wm = context.window_manager
        wm.modal_handler_add(self)
        bpy.context.scene['liveUpdatePose'] = True
        if not bpy.context.scene['globalTimerStarted']:
            bpy.ops.wm.global_timer()
        return {'RUNNING_MODAL'}


    def cancel(self, context):
        print('Stopping Update Pose')
        bpy.context.scene['liveUpdatePose'] = False
        return {'CANCELLED'}


    @classmethod
    def poll(cls, context):
        return True


bl_info = {"name": "Arm Capture", "category": "User"}

def register():
    bpy.utils.register_class(StartNodeButtonOperator)
    bpy.utils.register_class(RecordPoseButtonOperator)
    bpy.utils.register_class(StopRecordButtonOperator)
    bpy.utils.register_class(BLGlobalTimer)
    bpy.utils.register_class(BLUpdatePose)
    bpy.utils.register_class(ArmCapturePanel)
    bpy.utils.register_class(UIProperties)

def unregister():
    bpy.utils.unregister_class(StartNodeButtonOperator)
    bpy.utils.unregister_class(RecordPoseButtonOperator)
    bpy.utils.unregister_class(StopRecordButtonOperator)
    bpy.utils.unregister_class(ArmCapturePanel)
    bpy.utils.unregister_class(UIProperties)
    bpy.utils.unregister_class(BLGlobalTimer)
    bpy.utils.unregister_class(BLUpdatePose)

if __name__ == "__main__":
    register()
