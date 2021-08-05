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
 'mouth_U_L:1' : partial(convert, ['MouthSmileLeft'], [0.05]),
 'mouth_U_R:1' : partial(convert, ['MouthSmileLeft'], [0.05]),
 'brow.inner.L:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.inner.R:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.C:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.L:1' : partial(convert, ['BrowOuterUpLeft', 'BrowDownLeft'], [0.06, -0.06]),
 'brow.R:1' : partial(convert, ['BrowOuterUpRight', 'BrowDownRight'], [0.06, -0.06]),
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
 'chin:2' : partial(convert, ['JawOpen', 'MouthClose'], [0.12, -0.08]),
 'nostril_L:1' : partial(convert, ['NoseSneerRight'], [0.04]),
 'nostril_R:1' : partial(convert, ['NoseSneerLeft'], [0.04]),
 'cheek_L:1' : partial(convert, ['CheekSquintRight'], [0.04]),
 'cheek_R:1' : partial(convert, ['CheekSquintLeft'], [0.04]),
 'lip_D_L:1' : partial(convert, ['MouthLowerDownRight', 'MouthPressRight', 'MouthPucker'], [0.04, -0.04, -0.04]),
 'lip_D_R:1' : partial(convert, ['MouthLowerDownLeft', 'MouthPressLeft', 'MouthPucker'], [0.04, -0.04, -0.04]),
 'lip_D:1' : partial(convert, ['MouthPucker', 'MouthFunnel'], [-0.04, 0.04]),
 'lip_U_L:1' : partial(convert, ['MouthUpperUpRight', 'MouthPressRight', 'MouthPucker'], [0.04, 0.04, -0.04]),
 'lip_U_R:1' : partial(convert, ['MouthUpperUpLeft', 'MouthPressLeft', 'MouthPucker'], [0.04, 0.04, -0.04]),
 'lip_U:1' : partial(convert, ['MouthPucker', 'MouthFunnel'], [-0.04, 0.04]),
 'mouth_D_L:1' : partial(convert, ['MouthFrownRight'], [0.08]),
 'mouth_D_R:1' : partial(convert, ['MouthFrownLeft'], [0.08]),
 'mouth_C_L:1' : partial(convert, ['MouthStretchRight', 'MouthRight', 'MouthLeft', 'MouthPucker', 'MouthFunnel'], [0.12, 0.04, -0.04, -0.04, -0.04]),
 'mouth_C_R:1' : partial(convert, ['MouthStretchLeft', 'MouthLeft', 'MouthRight', 'MouthPucker', 'MouthFunnel'], [0.12, 0.04, -0.04, -0.04, -0.04]),
 'mouth_U_L:1' : partial(convert, ['MouthSmileRight'], [0.05]),
 'mouth_U_R:1' : partial(convert, ['MouthSmileRight'], [0.05]),
 'brow.inner.L:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.inner.R:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.C:1' : partial(convert, ['BrowInnerUp'], [0.05]),
 'brow.L:1' : partial(convert, ['BrowOuterUpRight', 'BrowDownRight'], [0.06, -0.06]),
 'brow.R:1' : partial(convert, ['BrowOuterUpLeft', 'BrowDownLeft'], [0.06, -0.06]),
 'eyelid_LO_L:1' : partial(convert, ['EyeSquintRight'], [-0.04]),
 'eyelid_LO_R:1' : partial(convert, ['EyeSquintLeft'], [-0.04]),
 'eyelid_UP_L:1' : partial(convert, ['EyeBlinkRight', 'EyeWideRight'], [-0.04, 0.04]), 
 'eyelid_UP_R:1' : partial(convert, ['EyeBlinkLeft', 'EyeWideLeft'], [-0.04, 0.04])
}

class UIProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.recording = bpy.props.BoolProperty(name = "Recording", default = False)
    bpy.types.Scene.globalTimerStarted = bpy.props.BoolProperty(name = "globalTimerStarted", default = False)
    bpy.types.Scene.liveUpdatePose = bpy.props.BoolProperty(name = "liveUpdatePose", default = False)
    bpy.types.Scene.hz = bpy.props.IntProperty(name = "Recording Hz", default = 1, soft_min = 1, soft_max = 20)
    bpy.types.Scene.mirrored = bpy.props.BoolProperty(name = "Mirrored", default = False)
    bpy.types.Scene.mouthSensitivity = bpy.props.FloatProperty(name = "Mouth Sensitivity", default = 1.0, min = 0.1, max = 5.0, step = 10)
    bpy.types.Scene.lipSensitivity = bpy.props.FloatProperty(name = "Lip Sensitivity", default = 1.0, min = 0.1, max = 5.0, step = 10)
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

    def execute(self, context):
        t = Thread(target=self.record, daemon=True)
        t.start()

        return {'FINISHED'}

    def record(self, host='0.0.0.0', port=11111):
        buchse = Buchse(host, port, as_server=True)
        start_time = time.time()
        while True:
            data, size = buchse.horch(FaceFrame.PACKET_MAX_SIZE)
            if not data or 0 == size:
                print(f'Received empty frame, skipping ...')
                continue
            frame = FaceFrame.from_raw(data, size)
            StartRecordButtonOperator.last_msg = frame.blendshapes
            if StartRecordButtonOperator.recording and 1/StartRecordButtonOperator.hz <= (time.time() - start_time):
                StartRecordButtonOperator.msgs.append(frame.blendshapes)
                start_time = time.time()


class StartRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_record"
    bl_label = "Start Recording"

    msgs = []
    last_msg = None
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
        action = bpy.data.actions.new("GST_Recorded")
        for i in range(len(StartRecordButtonOperator.msgs)):
            frame = StartRecordButtonOperator.msgs[i]
            frame_num = i * 48/StartRecordButtonOperator.hz
            if SetNeutralButtonOperator.neutral_pose:
                for k in frame.keys():
                    frame[k] = frame[k] - SetNeutralButtonOperator.neutral_pose[k]
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
                if "mouth" in name:
                    value = value * context.scene.mouthSensitivity
                if "lip" in name:
                    value = value * context.scene.lipSensitivity
                fc.keyframe_points.insert(frame_num, value)
        context.object.animation_data.action = action

        return {'FINISHED'}


class SetNeutralButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_set_neutral_pose"
    bl_label = "Set Neutral"

    neutral_pose = None

    def execute(self, context):
        if StartRecordButtonOperator.last_msg:
            SetNeutralButtonOperator.neutral_pose = StartRecordButtonOperator.last_msg

        return {'FINISHED'}

class ResetButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_reset_pose"
    bl_label = "Reset Pose"

    def execute(self, context):
        for key in Functions.keys():
            (name, index) = key.split(':')
            index = int(index)
            if key == 'head:1':
                bpy.data.objects['deform'].pose.bones[name].rotation_euler.y = 0
            else:
                if index == 0:
                    bpy.data.objects['deform'].pose.bones[name].location.x = 0
                if index == 1:
                    bpy.data.objects['deform'].pose.bones[name].location.y = 0
                if index == 2:
                    bpy.data.objects['deform'].pose.bones[name].location.z = 0

        return {'FINISHED'}


class ToggleUpdateButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_toggle_update"
    bl_label = "Toggle Live Update"

    def execute(self, context):
        if not bpy.context.scene.liveUpdatePose:
            bpy.context.scene.globalTimerStarted = True
            bpy.context.scene.liveUpdatePose = True
            bpy.ops.wm.global_timer()
            bpy.ops.wm.live_update_pose()
        else:
            bpy.context.scene.globalTimerStarted = False
            bpy.context.scene.liveUpdatePose = False

        return {'FINISHED'}


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
            if StartRecordButtonOperator.last_msg:
                if context.scene.mirrored:
                    f = MirroredFunctions
                else:
                    f = Functions
                msg = StartRecordButtonOperator.last_msg.copy()
                if SetNeutralButtonOperator.neutral_pose:
                    for k in msg.keys():
                        msg[k] -= SetNeutralButtonOperator.neutral_pose[k]
                print(msg['MouthShrugLower'], msg['MouthRollLower'], msg['MouthShrugUpper'], msg['MouthRollUpper'])
                for key in f.keys():
                    (name, index) = key.split(':')
                    index = int(index)
                    value = f[key](msg)
                    if "mouth" in name:
                        value = value * context.scene.mouthSensitivity
                    if "lip" in name:
                        value = value * context.scene.lipSensitivity
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
            bpy.ops.wm.global_timer()
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


class FaceCapturePanel(view3DPanel, bpy.types.Panel):
    bl_label = "Face Capture"
    bl_idname = "scene.face_capture_panel"

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
        row.operator(ResetButtonOperator.bl_idname)
        row = layout.row()
        row.prop(bpy.context.scene, "mouthSensitivity")
        row.prop(bpy.context.scene, "lipSensitivity")

class FaceRecordPanel(view3DPanel, bpy.types.Panel):
    bl_parent_id = "scene.face_capture_panel"
    bl_idname = "scene.face_record_panel"
    bl_label = "Face Record"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(bpy.context.scene, "hz")
        row.prop(bpy.context.scene, "mirrored")
        if not context.scene.recording:
            self.layout.operator(StartRecordButtonOperator.bl_idname)
        else:
            self.layout.operator(StopRecordButtonOperator.bl_idname)


bl_info = {"name": "Face Capture", "category": "User"}

def register():
    bpy.utils.register_class(StartRecordButtonOperator)
    bpy.utils.register_class(StopRecordButtonOperator)
    bpy.utils.register_class(StartPortButtonOperator)
    bpy.utils.register_class(ToggleUpdateButtonOperator)
    bpy.utils.register_class(SetNeutralButtonOperator)
    bpy.utils.register_class(ResetButtonOperator)
    bpy.utils.register_class(BLGlobalTimer)
    bpy.utils.register_class(BLUpdatePose)
    bpy.utils.register_class(FaceCapturePanel)
    bpy.utils.register_class(FaceRecordPanel)

def unregister():
    bpy.utils.unregister_class(StartRecordButtonOperator)
    bpy.utils.unregister_class(StopRecordButtonOperator)
    bpy.utils.unregister_class(StartPortButtonOperator)
    bpy.utils.unregister_class(ToggleUpdateButtonOperator)
    bpy.utils.unregister_class(SetNeutralButtonOperator)
    bpy.utils.unregister_class(ResetButtonOperator)
    bpy.utils.unregister_class(BLGlobalTimer)
    bpy.utils.unregister_class(BLUpdatePose)
    bpy.utils.unregister_class(FaceCapturePanel)
    bpy.utils.unregister_class(FaceRecordPanel)

if __name__ == "__main__":
    register()
