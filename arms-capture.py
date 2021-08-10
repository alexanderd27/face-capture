import bpy
import math
import rospy
from hr_msgs.msg import pau
import dynamic_reconfigure.client


_joint_list = ['Shoulder_R:0','Shoulder_R:2','Arm_Twist_R:1','Elbow_R:0','Forearm_Twist_R:1','Wrist_R:2','Wrist_R:0','Index_Fing_Base_R:0','Mid_Base_R:0',
'Ring_Base_R:0','Pinky_Base_R:0','Thumb_Base_R:0','Thumb_Pivot_R:1','Thumb_Pivot_R:0','Shoulder_L:0','Shoulder_L:2','Arm_Twist_L:1','Elbow_L:0','Forearm_Twist_L:1',
'Wrist_L:2','Wrist_L:0','Index_Fing_Base_L:0','Mid_Base_L:0','Ring_Base_L:0','Pinky_Base_L:0','Thumb_Base_L:0','Thumb_Pivot_L:1','Thumb_Pivot_L:0', 'Body:0', 'Body:2']


class UIProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.armStartingPose = bpy.props.EnumProperty(
        name = "Starting Pose",
        default = 'none',
        items = [('stand', 'Stand', ''),
                 ('sit', 'Sit', ''),
                 ('none', 'None', '')]
    )
    bpy.types.Scene.armRecorded = bpy.props.EnumProperty(
        name = "Choose Arm",
        default = 'both',
        items = [('right', 'Record Right Arm only', ''),
                 ('left', 'Record Left Arm Only', ''),
                 ('both', 'Record Both Arms', '')]
    )
    bpy.types.Scene.armRecordingHz = bpy.props.IntProperty(name = "Recording Hz", default = 1, soft_min = 1, soft_max = 50)
    bpy.types.Scene.armRecordingSpeed = bpy.props.FloatProperty(name = "Recording Speed", default = 1, soft_min = 0.1, soft_max = 2, step = 10, precision = 2)
    bpy.types.Scene.armTorque = bpy.props.BoolProperty(name = "Torque", default = True)
    bpy.types.Scene.armRecording = bpy.props.BoolProperty(name = "Recording", default = False)
    bpy.types.Scene.armGlobalTimerStarted = bpy.props.BoolProperty(name = "globalTimerStarted", default = False)
    bpy.types.Scene.armLiveUpdatePose = bpy.props.BoolProperty(name = "liveUpdatePose", default = False)


class ArmStartNodeButtonOperator(bpy.types.Operator):
    bl_idname = "scene.arm_start_node"
    bl_label = "Start ROS Node"

    def execute(self, context):
        try:
            rospy.init_node("arms_capture")
            client = dynamic_reconfigure.client.Client('/hr/control/arms')
            if client.get_configuration()['arms_mode'] == 'torque_off':
                context.scene.armTorque = False
            rospy.Subscriber("/hr/actuators/current_state", pau, self.callback)
        except Exception as e:
            self.report({"WARNING"}, "Can\'t start the node with exception {}".format(e))

        return {'FINISHED'}

    def callback(self, msg):
        if ArmStartRecordButtonOperator.recording:
            if ArmStartRecordButtonOperator.counter == round(50/ArmStartRecordButtonOperator.hz):
                ArmStartRecordButtonOperator.msg_angles.append(msg.m_angles)
                ArmStartRecordButtonOperator.counter = 0
            ArmStartRecordButtonOperator.counter += 1
        ArmStartRecordButtonOperator.last_msg = msg


class ArmStartRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.arm_start_record"
    bl_label = "Start Blender Recording"

    hz = 0
    counter = 0
    msg_angles = []
    recording = False
    last_msg = None

    def execute(self, context):
        ArmStartRecordButtonOperator.hz = context.scene.armRecordingHz
        ArmStartRecordButtonOperator.counter = 0
        ArmStartRecordButtonOperator.msg_angles = []
        context.scene.armRecording = True
        ArmStartRecordButtonOperator.recording = True

        return {'FINISHED'}
    

class ArmStopRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.arm_stop_record"
    bl_label = "Stop Recording"

    def execute(self, context):
        context.scene.armRecording = False
        ArmStartRecordButtonOperator.recording = False

        if context.scene.armStartingPose == 'none':
            s1 = "ARM-MAIN_"
        elif context.scene.armStartingPose == 'sit':
            s1 = "ARM-MAIN-SIT_"
        elif context.scene.armStartingPose == 'stand':
            s1 = "ARM-MAIN-STAND_"
        if context.scene.armRecorded == 'both':
            s2 = "1-B_"
        elif context.scene.armRecorded == 'left':
            s2 = "1-L_"
        elif context.scene.armRecorded == 'right':
            s2 = "1-R_"
        action = bpy.data.actions.new(s1 + s2 + "MyRecording")

        for i in range(len(ArmStartRecordButtonOperator.msg_angles)):
            if context.scene.armStartingPose == 'none':
                frame = i * (48/context.scene.armRecordingHz)/context.scene.armRecordingSpeed
            else:
                frame = (i+1) * (48/context.scene.armRecordingHz)/context.scene.armRecordingSpeed + 48

            angles = ArmStartRecordButtonOperator.msg_angles[i]
            pose = dict(zip(_joint_list, angles))
            pose['Wrist_L:0'] = -pose['Wrist_L:0']
            del pose['Body:0']
            del pose['Body:2']

            if context.scene.armRecorded == 'left':
                pose = {b:a for (b,a) in pose.items() if '_L' in b}
            elif context.scene.armRecorded == 'right':
                pose = {b:a for (b,a) in pose.items() if '_R' in b}

            for b, a in pose.items():
                self.set_angle(action, frame, b, a)

        if context.scene.armStartingPose != 'none':
            self.add_starting_pose(context, action)

        context.object.animation_data.action = action

        return {'FINISHED'}
    
    @staticmethod
    def set_angle(action, frame, bone, angle):
        (path,index) = bone.split(':')
        path = "pose.bones[\"%s\"].rotation_euler" %path
        index = int(index)
        angle = math.radians(angle)

        fc = action.fcurves.find(path, index)
        if not fc:
            fc = action.fcurves.new(path, index)
        fc.keyframe_points.insert(frame, angle)
    
    @staticmethod
    def add_starting_pose(context, action):
        if context.scene.armStartingPose == 'stand':
            action_new = bpy.data.actions["ARM-MAIN-1"]
        elif context.scene.armStartingPose == 'sit':
            action_new = bpy.data.actions["ARM-MAIN-2"]

        for fc in action.fcurves:
            if (fc.data_path == 'pose.bones["Wrist_R"].rotation_euler' and fc.array_index == 0):
                y = -0.03307855874300003
            elif (fc.data_path == 'pose.bones["Wrist_L"].rotation_euler' and fc.array_index == 0):
                y = -0.001886842423118651
            else:
                fc_new = action_new.fcurves.find(fc.data_path, fc.array_index)
                y = fc_new.keyframe_points[0].co.y
            fc.keyframe_points.insert(0, y)
            fc.keyframe_points.insert(fc.keyframe_points[0].co.x + 48, y)
            fc.keyframe_points.insert(fc.keyframe_points[-1].co.x + 48, y)
            fc.keyframe_points.insert(fc.keyframe_points[-1].co.x + 48, y)
            

class ArmToggleTorqueButtonOperator(bpy.types.Operator):
    bl_idname = "scene.arm_toggle_torque"
    bl_label = "Toggle Torque"

    def execute(self, context):
        client = dynamic_reconfigure.client.Client('/hr/control/arms')
        if context.scene.armTorque:
            client.update_configuration({'arms_mode':'torque_off'})
            context.scene.armTorque = False
        else:
            client.update_configuration({'arms_mode':'animations'})
            context.scene.armTorque = True

        return{'FINISHED'}


class ArmToggleUpdateButtonOperator(bpy.types.Operator):
    bl_idname = "scene.arm_toggle_update"
    bl_label = "Toggle Live Update"

    def execute(self, context):
        if not bpy.context.scene.armLiveUpdatePose:
            bpy.context.scene.armGlobalTimerStarted = True
            bpy.context.scene.armLiveUpdatePose = True
            bpy.ops.wm.arm_global_timer()
            bpy.ops.wm.arm_live_update_pose()
        else:
            bpy.context.scene.armGlobalTimerStarted = False
            bpy.context.scene.armLiveUpdatePose = False

        return {'FINISHED'}


class ArmBLGlobalTimer(bpy.types.Operator):
    """Timer  Control"""
    bl_label = "Global Timer"
    bl_idname = 'wm.arm_global_timer'

    _timer = None
    _maxFPS = 50

    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(1/self._maxFPS, window=context.window)
        wm.modal_handler_add(self)

        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if not context.scene.armGlobalTimerStarted:
            return self.cancel(context)

        return {'PASS_THROUGH'}

    def cancel(self,context):
        if self._timer:
            wm = context.window_manager
            wm.event_timer_remove(self._timer)

        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class ArmBLUpdatePose(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Update"
    bl_idname = 'wm.arm_live_update_pose'

    def modal(self, context, event):
        if not context.scene.armLiveUpdatePose:
            return self.cancel(context)

        if event.type == 'TIMER':
            if ArmStartRecordButtonOperator.last_msg is not None:
                pose = dict(zip(_joint_list, ArmStartRecordButtonOperator.last_msg.m_angles))
                pose['Wrist_L:0'] = -pose['Wrist_L:0']
                del pose['Body:0']
                del pose['Body:2']
                for i,a in pose.items():
                    self.set_angle(i, a)

        return {'PASS_THROUGH'}

    def set_angle(self, bone, angle):
        (b,a) = bone.split(':')
        angle = math.radians(angle)
        a = int(a)
        if a == 0:
            bpy.data.objects['AA'].pose.bones[b].rotation_euler.x = angle
        if a == 1:
            bpy.data.objects['AA'].pose.bones[b].rotation_euler.y = angle
        if a == 2:
            bpy.data.objects['AA'].pose.bones[b].rotation_euler.z = angle

    def execute(self, context):
        wm = context.window_manager
        wm.modal_handler_add(self)
        if not context.scene.armGlobalTimerStarted:
            bpy.ops.wm.arm_global_timer()

        return {'RUNNING_MODAL'}

    def cancel(self, context):
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class ArmCapturePanel(bpy.types.Panel):
    bl_label = "Record Arms"
    bl_idname = "scene.arm_record_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(self, context):    
        return True

    def draw(self, context):
        layout = self.layout
        layout.operator(ArmStartNodeButtonOperator.bl_idname)
        if not context.scene.armLiveUpdatePose:
            self.layout.operator(ArmToggleUpdateButtonOperator.bl_idname, text = "Start Live Update")
        else:
            self.layout.operator(ArmToggleUpdateButtonOperator.bl_idname, text = "Stop Live Update")
        if context.scene.armTorque:
            layout.operator(ArmToggleTorqueButtonOperator.bl_idname, text = "Torque Off")
        else:
            layout.operator(ArmToggleTorqueButtonOperator.bl_idname, text = "Torque On")
        layout.prop(bpy.context.scene, "armRecordingHz")
        layout.prop(bpy.context.scene, "armRecordingSpeed")
        layout.prop(bpy.context.scene, "armStartingPose")
        layout.prop(bpy.context.scene, "armRecorded")
        if context.scene.armRecording:
            layout.operator(ArmStopRecordButtonOperator.bl_idname)
        else:
            layout.operator(ArmStartRecordButtonOperator.bl_idname)


bl_info = {"name": "Arm Capture", "category": "User"}

def register():
    bpy.utils.register_class(ArmStartNodeButtonOperator)
    bpy.utils.register_class(ArmStartRecordButtonOperator)
    bpy.utils.register_class(ArmStopRecordButtonOperator)
    bpy.utils.register_class(ArmBLGlobalTimer)
    bpy.utils.register_class(ArmBLUpdatePose)
    bpy.utils.register_class(ArmToggleTorqueButtonOperator)
    bpy.utils.register_class(ArmToggleUpdateButtonOperator)
    bpy.utils.register_class(ArmCapturePanel)
    bpy.utils.register_class(UIProperties)

def unregister():
    bpy.utils.unregister_class(ArmStartNodeButtonOperator)
    bpy.utils.unregister_class(ArmStartRecordButtonOperator)
    bpy.utils.unregister_class(ArmStopRecordButtonOperator)
    bpy.utils.unregister_class(ArmBLGlobalTimer)
    bpy.utils.unregister_class(ArmBLUpdatePose)
    bpy.utils.unregister_class(ArmToggleTorqueButtonOperator)
    bpy.utils.unregister_class(ArmToggleUpdateButtonOperator)
    bpy.utils.unregister_class(ArmCapturePanel)
    bpy.utils.unregister_class(UIProperties)

if __name__ == "__main__":
    register()
