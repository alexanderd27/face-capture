import bpy
import time
from threading import Thread
from llv.buchse import Buchse
from llv.gesicht import FaceFrame
from functools import partial
import math

def convert(b, w, frame):
    ret = 0
    for i in range(len(b)):
        ret += frame[b[i]] * w[i]
    return ret

def convert_head_x(frame):                                                                             
    return math.tan(-frame['HeadYaw']*math.pi/2) * 4

def convert_head_roll(frame):
    return min(frame['HeadRoll']*math.pi/2, 0.366519)

def convert_head_z(frame):
    return math.tan(frame['HeadPitch']*math.pi/2) * 4

def convert_eyes_x(frame):
    yaw = frame['HeadYaw'] - (frame['LeftEyeYaw']+frame['RightEyeYaw'])/2
    return math.tan(yaw*math.pi/2) * 4

def convert_eyes_z(frame):
    pitch = frame['HeadPitch'] - (frame['LeftEyePitch']+frame['RightEyePitch'])/2
    return math.tan(pitch*math.pi/2) * 4

Functions = {
 'eye_offset:0' : convert_eyes_x,
 'eye_offset:2' : convert_eyes_z,
 'head:0' : convert_head_x,
 'head:1' : convert_head_roll,
 'head:2' : convert_head_z,
 'chin:2' : partial(convert, ['JawOpen', 'MouthClose'], [0.12, -0.08]),
 'nostril_L:1' : partial(convert, ['NoseSneerLeft'], [0.04]),
 'nostril_R:1' : partial(convert, ['NoseSneerRight'], [0.04]),
 'cheek_L:1' : partial(convert, ['CheekSquintLeft'], [0.04]),
 'cheek_R:1' : partial(convert, ['CheekSquintRight'], [0.04]),
 'lip_D_L:1' : partial(convert, ['MouthLowerDownLeft', 'MouthPressLeft', 'MouthPucker'], [0.04, -0.04, -0.04]),
 'lip_D_R:1' : partial(convert, ['MouthLowerDownRight', 'MouthPressRight', 'MouthPucker'], [0.04, -0.04, -0.04]),
 'lip_D:1' : partial(convert, ['MouthPucker', 'MouthFunnel'], [-0.04, 0.04]),
 'lip_U_L:1' : partial(convert, ['MouthUpperUpLeft', 'MouthPressLeft', 'MouthPucker'], [0.04, 0.04, -0.04]),
 'lip_U_R:1' : partial(convert, ['MouthUpperUpRight', 'MouthPressRight', 'MouthPucker'], [0.04, 0.04, -0.04]),
 'lip_U:1' : partial(convert, ['MouthPucker', 'MouthFunnel'], [-0.04, 0.04]),
 'mouth_D_L:1' : partial(convert, ['MouthFrownLeft'], [0.08]),
 'mouth_D_R:1' : partial(convert, ['MouthFrownRight'], [0.08]),
 'mouth_C_L:1' : partial(convert, ['MouthStretchLeft', 'MouthLeft', 'MouthRight', 'MouthPucker', 'MouthFunnel'], [0.12, 0.04, -0.04, -0.04, -0.04]),
 'mouth_C_R:1' : partial(convert, ['MouthStretchRight', 'MouthRight', 'MouthLeft', 'MouthPucker', 'MouthFunnel'], [0.12, 0.04, -0.04, -0.04, -0.04]),
 'mouth_U_L:1' : partial(convert, ['MouthSmileLeft'], [0.08]),
 'mouth_U_R:1' : partial(convert, ['MouthSmileLeft'], [0.08]),
 'brow.inner.L:1' : partial(convert, ['BrowInnerUp', 'BrowDownLeft'], [0.05, -0.06]),
 'brow.inner.R:1' : partial(convert, ['BrowInnerUp', 'BrowDownRight'], [0.05, -0.06]),
 'brow.C:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.L:1' : partial(convert, ['BrowOuterUpLeft', 'BrowDownLeft'], [0.05, -0.06]),
 'brow.R:1' : partial(convert, ['BrowOuterUpRight', 'BrowDownRight'], [0.05, -0.06]),
 'eyelid_LO_L:1' : partial(convert, ['EyeSquintLeft'], [-0.04]),
 'eyelid_LO_R:1' : partial(convert, ['EyeSquintRight'], [-0.04]),
 'eyelid_UP_L:1' : partial(convert, ['EyeBlinkLeft', 'EyeWideLeft'], [-0.04, 0.04]), 
 'eyelid_UP_R:1' : partial(convert, ['EyeBlinkRight', 'EyeWideRight'], [-0.04, 0.04])
}

MirroredFunctions = {
 'eye_offset:0' : lambda frame : -convert_eyes_x(frame),
 'eye_offset:2' : convert_eyes_z,
 'head:0' : lambda frame : -convert_head_x(frame),
 'head:1' : lambda frame: -convert_head_roll(frame),
 'head:2' : convert_head_z,
 'chin:2' : Functions['chin:2'],
 'nostril_L:1' : Functions['nostril_R:1'],
 'nostril_R:1' : Functions['nostril_L:1'],
 'cheek_L:1' : Functions['cheek_R:1'],
 'cheek_R:1' : Functions['cheek_L:1'],
 'lip_D_L:1' : Functions['lip_D_R:1'],
 'lip_D_R:1' : Functions['lip_D_L:1'],
 'lip_D:1' : Functions['lip_D:1'],
 'lip_U_L:1' : Functions['lip_U_R:1'],
 'lip_U_R:1' : Functions['lip_U_L:1'],
 'lip_U:1' : Functions['lip_U:1'],
 'mouth_D_L:1' : Functions['mouth_D_R:1'],
 'mouth_D_R:1' : Functions['mouth_D_L:1'],
 'mouth_C_L:1' : Functions['mouth_C_R:1'],
 'mouth_C_R:1' : Functions['mouth_C_L:1'],
 'mouth_U_L:1' : Functions['mouth_U_R:1'],
 'mouth_U_R:1' : Functions['mouth_U_L:1'],
 'brow.inner.L:1' : Functions['brow.inner.R:1'],
 'brow.inner.R:1' : Functions['brow.inner.L:1'],
 'brow.C:1' : Functions['brow.C:1'],
 'brow.L:1' : Functions['brow.R:1'],
 'brow.R:1' : Functions['brow.L:1'],
 'eyelid_LO_L:1' : Functions['eyelid_LO_R:1'],
 'eyelid_LO_R:1' : Functions['eyelid_LO_L:1'],
 'eyelid_UP_L:1' : Functions['eyelid_UP_R:1'],
 'eyelid_UP_R:1' : Functions['eyelid_UP_L:1']
}

StreamingFunctions = {
 'EyesYaw': partial(convert, ['LeftEyeYaw', 'RightEyeYaw'], [-math.pi/4, -math.pi/4]),
 'EyesPitch': partial(convert, ['LeftEyePitch', 'RightEyePitch'], [-math.pi/4, -math.pi/4]),
 'HeadYaw' : partial(convert, ['HeadYaw'], [-math.pi/2]),
 'HeadRoll' : partial(convert, ['HeadRoll'], [math.pi/2]),
 'HeadPitch' : partial(convert, ['HeadPitch'], [math.pi/2]),
 'Jaw' : lambda frame: max(min(partial(convert, ['JawOpen', 'MouthClose'], [1, -0.7])(frame), 1), 0),
 'SneerLeft' : lambda frame : max(min(25 * Functions['nostril_L:1'](frame), 1), 0),
 'SneerRight' : lambda frame : max(min(25 * Functions['nostril_R:1'](frame), 1), 0),
 'CheekSquintLeft' : lambda frame : max(min(25 * Functions['cheek_L:1'](frame), 1), 0),
 'CheekSquintRight' : lambda frame : max(min(25 * Functions['cheek_R:1'](frame), 1), 0),
 'LowerLipLeft' : lambda frame : max(min(25 * Functions['lip_D_L:1'](frame), 1), -1),
 'LowerLipRight' : lambda frame : max(min(25 * Functions['lip_D_R:1'](frame), 1), -1),
 'LowerLipCenter' : lambda frame : max(min(25 * Functions['lip_D:1'](frame), 1), -1),
 'UpperLipLeft' : lambda frame : max(min(25 * Functions['lip_U_L:1'](frame), 1), -1),
 'UpperLipRight' : lambda frame : max(min(25 * Functions['lip_U_R:1'](frame), 1), -1),
 'UpperLipCenter' : lambda frame : max(min(25 * Functions['lip_U:1'](frame), 1), -1),
 'FrownLeft' : lambda frame : max(min(25 * Functions['mouth_D_L:1'](frame), 1), 0),
 'FrownRight' : lambda frame : max(min(25 * Functions['mouth_D_R:1'](frame), 1), 0),
 'LipStretchLeft' : lambda frame : max(min(25 * Functions['mouth_C_L:1'](frame), 1), -1),
 'LipStretchRight' : lambda frame : max(min(25 * Functions['mouth_C_R:1'](frame), 1), -1),
 'SmileLeft' : lambda frame : max(min(25 * Functions['mouth_U_L:1'](frame), 1), 0),
 'SmileRight' : lambda frame : max(min(25 * Functions['mouth_U_R:1'](frame), 1), 0),
 'BrowInnerLeft' : lambda frame : max(min(25 * Functions['brow.inner.L:1'](frame), 1), -1),
 'BrowInnerRight' : lambda frame : max(min(25 * Functions['brow.inner.R:1'](frame), 1), -1),
 'BrowCenter' :  lambda frame : max(min(25 * Functions['brow.C:1'](frame), 1), -1),
 'BrowOuterLeft' : lambda frame : max(min(25 * Functions['brow.L:1'](frame), 1), -1),
 'BrowOuterRight' : lambda frame : max(min(25 * Functions['brow.R:1'](frame), 1), -1),
 'LowerLidLeft' : lambda frame : max(min(25 * Functions['eyelid_LO_L:1'](frame), 1), -1),
 'LowerLidRight' : lambda frame : max(min(25 * Functions['eyelid_LO_R:1'](frame), 1), -1),
 'UpperLidLeft' : lambda frame : max(min(25 * Functions['eyelid_UP_L:1'](frame), 1), -1),
 'UpperLidRight' :lambda frame : max(min(25 * Functions['eyelid_UP_R:1'](frame), 1), -1),
}

class UIProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.recording = bpy.props.BoolProperty(name = "Recording", default = False)
    bpy.types.Scene.globalTimerStarted = bpy.props.BoolProperty(name = "globalTimerStarted", default = False)
    bpy.types.Scene.liveUpdatePose = bpy.props.BoolProperty(name = "liveUpdatePose", default = False)
    bpy.types.Scene.streaming = bpy.props.BoolProperty(name = "Streaming", default = False)
    bpy.types.Scene.hz = bpy.props.IntProperty(name = "Recording Hz", default = 1, soft_min = 1, soft_max = 20)
    bpy.types.Scene.updateMirrored = bpy.props.BoolProperty(name = "Update Mirrored", default = False)
    bpy.types.Scene.recordingMirrored = bpy.props.BoolProperty(name = "Recording Mirrored", default = False)
    bpy.types.Scene.recordHead = bpy.props.BoolProperty(name = "Head Movement", default = True)
    bpy.types.Scene.recordEyeMovement = bpy.props.BoolProperty(name = "Eye Movement", default = True)
    bpy.types.Scene.recordEyeLeft = bpy.props.BoolProperty(name = "Left Eye", default = True)
    bpy.types.Scene.recordEyeRight = bpy.props.BoolProperty(name = "Right Eye", default = True)
    bpy.types.Scene.recordMouth = bpy.props.BoolProperty(name = "Mouth", default = True)
    bpy.types.Scene.recordNose = bpy.props.BoolProperty(name = "Nose Sneer", default = True)
    bpy.types.Scene.recordCheek = bpy.props.BoolProperty(name = "Cheek Sneer", default = True)


class StartPortButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_port"
    bl_label = "Start Port"
    
    last_msg = None

    def execute(self, context):
        t1 = Thread(target=self.record, daemon=True)
        t1.start()

        return {'FINISHED'}

    def record(self, host='0.0.0.0', port=11111):
        buchse = Buchse(host, port, as_server=True)
        start_time = time.time()
        imported  = False
        while True:
            data, size = buchse.horch(FaceFrame.PACKET_MAX_SIZE)
            if not data or 0 == size:
                print(f'Received empty frame, skipping ...')
                continue
            frame = FaceFrame.from_raw(data, size)
            StartPortButtonOperator.last_msg = frame.blendshapes
            msg = StartPortButtonOperator.last_msg
            if SetNeutralButtonOperator.neutral_pose:
                msg = {b : (v - SetNeutralButtonOperator.neutral_pose[b]) for (b, v) in msg.items()}
            if StartStreamButtonOperator.face_publisher:
                if not imported:
                    import rospy
                    from hr_msgs.msg import TargetPosture
                    imported = True
                tp = TargetPosture()
                for key in StreamingFunctions.keys():
                    value = StreamingFunctions[key](msg)
                    tp.values.append(value)
                tp.names = list(StreamingFunctions.keys())
                StartStreamButtonOperator.face_publisher.publish(tp)
            if StartRecordButtonOperator.recording and 1/StartRecordButtonOperator.hz <= (time.time() - start_time):
                StartRecordButtonOperator.msgs.append(msg)
                start_time = time.time()


class StartRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_record"
    bl_label = "Start Recording"

    msgs = []
    hz = 0
    recording = False

    def execute(self, context):
        StartRecordButtonOperator.msgs = []
        StartRecordButtonOperator.hz = context.scene.hz
        StartRecordButtonOperator.recording = True
        context.scene.recording = True

        return {'FINISHED'}


class StopRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_stop_record"
    bl_label = "Stop Recording"

    def execute(self, context):
        context.scene.recording = False
        StartRecordButtonOperator.recording = False
        action = bpy.data.actions.new("GST-Recorded")
        for i in range(len(StartRecordButtonOperator.msgs)):
            frame = StartRecordButtonOperator.msgs[i]
            frame_num = i * 48/StartRecordButtonOperator.hz
            if context.scene.recordingMirrored:
                f = MirroredFunctions
            else:
                f = Functions
            for key in f.keys():
                (name, index) = key.split(':')
                index = int(index)
                if key == 'head:1':
                    path = 'pose.bones[\"{bname}\"].rotation_euler'.format(bname = name)
                else:
                    path = 'pose.bones[\"{bname}\"].location'.format(bname = name)
                fc = action.fcurves.find(path, index)
                if not fc:
                    fc = action.fcurves.new(path, index)
                value = f[key](frame)
                fc.keyframe_points.insert(frame_num, value)
        context.object.animation_data.action = action

        return {'FINISHED'}


class SetNeutralButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_set_neutral_pose"
    bl_label = "Set Neutral"

    neutral_pose = None

    def execute(self, context):
        if StartPortButtonOperator.last_msg:
            SetNeutralButtonOperator.neutral_pose = StartPortButtonOperator.last_msg

        return {'FINISHED'}


class ToggleUpdateButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_toggle_update"
    bl_label = "Toggle Live Update"

    def execute(self, context):
        if not bpy.context.scene.liveUpdatePose:
            bpy.context.scene.globalTimerStarted = True
            bpy.context.scene.liveUpdatePose = True
            bpy.ops.wm.face_global_timer()
            bpy.ops.wm.face_live_update_pose()
        else:
            bpy.context.scene.globalTimerStarted = False
            bpy.context.scene.liveUpdatePose = False

        return {'FINISHED'}


class StartNodeButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_node"
    bl_label = "Start Node"


    def execute(self, context):
        try: 
            import rospy
            from hr_msgs.msg import TargetPosture
            rospy.init_node("face_capture")
            print("Node Started")
        except Exception as e:
            self.report({"WARNING"}, "Cant start the node with exception {}".format(e))
            return

        return {'FINISHED'}


class StartStreamButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_stream"
    bl_label = "Start Streaming"

    face_publisher = None

    def execute(self, context):
        try:
            import rospy
            from hr_msgs.msg import TargetPosture
        except:
            return

        StartStreamButtonOperator.face_publisher = rospy.Publisher('/hr/animation/set_state', TargetPosture, queue_size=4)
        context.scene.streaming = True

        return {'FINISHED'}


class StopStreamButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_stop_stream"
    bl_label = "Stop Streaming"

    def execute(self, context):
        StartStreamButtonOperator.face_publisher = None
        context.scene.streaming = False
    
        return{'FINISHED'}


class BLGlobalTimer(bpy.types.Operator):
    """Timer  Control"""
    bl_label = "Global Timer"
    bl_idname = 'wm.face_global_timer'

    _timer = None
    maxFPS = 30

    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(1/self.maxFPS, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if not context.scene.globalTimerStarted:
            return self.cancel(context)
        return {'PASS_THROUGH'}

    def cancel(self, context):
        if self._timer:
            wm = context.window_manager
            wm.event_timer_remove(self._timer)
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class BLUpdatePose(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Update"
    bl_idname = 'wm.face_live_update_pose'

    def modal(self, context, event):
        if not context.scene.liveUpdatePose:
            return self.cancel(context)
        if event.type == 'TIMER':
            if StartPortButtonOperator.last_msg:
                if context.scene.updateMirrored:
                    f = MirroredFunctions
                else:
                    f = Functions
                msg = StartPortButtonOperator.last_msg
                if SetNeutralButtonOperator.neutral_pose:
                    msg = {b : (v - SetNeutralButtonOperator.neutral_pose[b]) for (b, v) in msg.items()}
                for key in f.keys():
                    (name, index) = key.split(':')
                    index = int(index)
                    value = f[key](msg)
                    if key == 'head:1':
                        bpy.data.objects['deform'].pose.bones[name].rotation_euler.y = value
                    else:
                        if index == 0:
                            bpy.data.objects['deform'].pose.bones[name].location.x = value
                        if index == 1:
                            bpy.data.objects['deform'].pose.bones[name].location.y = value
                        if index == 2:
                            bpy.data.objects['deform'].pose.bones[name].location.z = value
        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        wm.modal_handler_add(self)
        if not context.scene.globalTimerStarted:
            bpy.ops.wm.face_global_timer()
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class view3DPanel:
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(self, context):
        return True


class FaceUpdatePanel(view3DPanel, bpy.types.Panel):
    bl_label = "Face Update"
    bl_idname = "scene.face_update_panel"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator(StartPortButtonOperator.bl_idname)
        if not context.scene.liveUpdatePose:
            row.operator(ToggleUpdateButtonOperator.bl_idname, text = "Start Live Update")
        else:
            row.operator(ToggleUpdateButtonOperator.bl_idname, text = "Stop Live Update")
        row = layout.row()
        row.operator(SetNeutralButtonOperator.bl_idname)
        row.prop(bpy.context.scene, "updateMirrored")


class FaceRecordPanel(view3DPanel, bpy.types.Panel):
    bl_parent_id = "scene.face_update_panel"
    bl_idname = "scene.face_record_panel"
    bl_label = "Face Record"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(bpy.context.scene, "hz")
        row.prop(bpy.context.scene, "recordingMirrored")
        if not context.scene.recording:
            self.layout.operator(StartRecordButtonOperator.bl_idname)
        else:
            self.layout.operator(StopRecordButtonOperator.bl_idname)


class FaceStreamPanel(view3DPanel, bpy.types.Panel):
    bl_parent_id = "scene.face_update_panel"
    bl_idname = "scene.face_stream_panel"
    bl_label = "Face Stream"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator(StartNodeButtonOperator.bl_idname)
        if not context.scene.streaming:
            row.operator(StartStreamButtonOperator.bl_idname)
        else:
            row.operator(StopStreamButtonOperator.bl_idname)


bl_info = {"name": "Face Capture", "category": "User"}

def register():
    bpy.utils.register_class(StartRecordButtonOperator)
    bpy.utils.register_class(StopRecordButtonOperator)
    bpy.utils.register_class(StartPortButtonOperator)
    bpy.utils.register_class(ToggleUpdateButtonOperator)
    bpy.utils.register_class(SetNeutralButtonOperator)
    bpy.utils.register_class(StartNodeButtonOperator)
    bpy.utils.register_class(StartStreamButtonOperator)
    bpy.utils.register_class(StopStreamButtonOperator)
    bpy.utils.register_class(BLGlobalTimer)
    bpy.utils.register_class(BLUpdatePose)
    bpy.utils.register_class(FaceUpdatePanel)
    bpy.utils.register_class(FaceRecordPanel)
    bpy.utils.register_class(FaceStreamPanel)

def unregister():
    bpy.utils.unregister_class(StartRecordButtonOperator)
    bpy.utils.unregister_class(StopRecordButtonOperator)
    bpy.utils.unregister_class(StartPortButtonOperator)
    bpy.utils.unregister_class(ToggleUpdateButtonOperator)
    bpy.utils.unregister_class(SetNeutralButtonOperator)
    bpy.utils.unregister_class(StartNodeButtonOperator)
    bpy.utils.unregister_class(StartStreamButtonOperator)
    bpy.utils.unregister_class(StopStreamButtonOperator)
    bpy.utils.unregister_class(BLGlobalTimer)
    bpy.utils.unregister_class(BLUpdatePose)
    bpy.utils.unregister_class(FaceUpdatePanel)
    bpy.utils.unregister_class(FaceRecordPanel)
    bpy.utils.unregister_class(FaceStreamPanel)

if __name__ == "__main__":
    register()
