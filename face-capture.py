import bpy
import time
from threading import Thread
from llv.buchse import Buchse
from llv.gesicht import FaceFrame
from functools import partial
import math

def convert1(b1, w1, frame):
    return frame[b1] * w1

def convert2(b1, w1, b2, w2, frame):
    return frame[b1]*w1 + frame[b2]*w2

def convert_head(frame):
    head_roll = min(frame['HeadRoll']*math.pi/2, 0.366519)                                                                              
    head_x = math.tan(-frame['HeadYaw']*math.pi/2) * 4
    head_z = math.tan(frame['HeadPitch']*math.pi/2) * 4
    return [head_x, head_roll, head_z]

def convert_eyes(frame):
    yaw = frame['HeadYaw'] - (frame['LeftEyeYaw']+frame['RightEyeYaw'])/2
    pitch = frame['HeadPitch'] - (frame['LeftEyePitch']+frame['RightEyePitch'])/2
    eye_x = math.tan(yaw*math.pi/2) * 4
    eye_z = math.tan(pitch*math.pi/2) * 4
    return [eye_x, 0, eye_z]

Functions = {
 'eye_offset:0' : convert_eyes,
 'eye_offset:2' : convert_eyes,
 'head:0' : convert_head,
 'head:1' : convert_head,
 'head:2' : convert_head,
 'chin:2' : partial(convert1, 'JawOpen', 0.16),
 'nostril_L:1' : partial(convert1, 'NoseSneerLeft', 0.04),
 'nostril_R:1' : partial(convert1, 'NoseSneerRight', 0.04),
 'cheek_L:1' : partial(convert1, 'CheekSquintLeft', 0.04),
 'cheek_R:1' : partial(convert1, 'CheekSquintRight', 0.04),
 'lip_D_L:1' : partial(convert1, 'MouthLowerDownLeft', 0.04),
 'lip_D_R:1' : partial(convert1, 'MouthLowerDownRight', 0.04),
 'lip_D:1' : partial(convert2, 'MouthShrugLower', 0.04, 'MouthRollLower', -0.04),
 'lip_U_L:1' : partial(convert1, 'MouthUpperUpLeft', 0.04),
 'lip_U_R:1' : partial(convert1, 'MouthUpperUpRight', 0.04),
 'lip_U:1' : partial(convert2, 'MouthShrugUpper', 0.04, 'MouthRollUpper', -0.04),
 'mouth_D_L:1' : partial(convert1, 'MouthFrownLeft', 0.04),
 'mouth_D_R:1' : partial(convert1, 'MouthFrownRight', 0.04),
 'mouth_C_L:1' : partial(convert1, 'MouthStretchLeft', 0.04),
 'mouth_C_R:1' : partial(convert1, 'MouthStretchRight', 0.04),
 'mouth_U_L:1' : partial(convert1, 'MouthSmileLeft', 0.04),
 'mouth_U_R:1' : partial(convert1, 'MouthSmileLeft', 0.04),
 'brow.inner.L:1' : partial(convert1, 'BrowInnterUp', 0.04),
 'brow.inner.R:1' : partial(convert1, 'BrowInnterUp', 0.04),
 'brow.C:1' : partial(convert1, 'BrowInnterUp', 0.04),
 'brow.L:1' : partial(convert2, 'BrowOuterUpLeft', 0.04, 'BrowDownLeft', -0.04),
 'brow.R:1' : partial(convert2, 'BrowOuterUpRight', 0.04, 'BrowDownRight', -0.04),
 'eyelid_LO_L:1' : partial(convert1, 'EyeSqintLeft', -0.04),
 'eyelid_LO_R:1' : partial(convert1, 'EyeSqintRight', -0.04),
 'eyelid_UP_L:1' : partial(convert2, 'EyeBlinkLeft', -0.04, 'EyeWideLeft', 0.04), 
 'eyelid_UP_R:1' : partial(convert2, 'EyeBlinkRight', -0.04, 'EyeWideRight', 0.04)
#  'eyelid_U_L' : partial,
#  'eyelid_U_R' : partial,
#  'eyelid_blink_LO_L', 
#  'eyelid_blink_LO_R', 
#  'eyelid_blink_UP_L', 
#  'eyelid_blink_UP_R'
}

class UIProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.hz = bpy.props.IntProperty(
        name = "Recording Hz",
        default = 1,
        soft_min = 1,
        soft_max = 20
    )
    bpy.types.Scene.recording = bpy.props.BoolProperty(
        name = "Recording",
        default = False
    )
    bpy.types.Scene.globalTimerStarted = bpy.props.BoolProperty(
        name = "globalTimerStarted",
        default=False
    )
    bpy.types.Scene.liveUpdateFace = bpy.props.BoolProperty(
        name = "liveUpdateFace",
        default=False
    )

class StartRecordFaceButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_face_record"
    bl_label = "Start Recording"

    msgs = []
    last_msg = None
    hz = 0
    recording = False

    def execute(self, context):
        StartRecordFaceButtonOperator.msgs = []
        StartRecordFaceButtonOperator.hz = context.scene.hz
        StartRecordFaceButtonOperator.recording = True
        context.scene.recording = True
        t = Thread(target=self.record, daemon=True)
        t.start()
        print('Recording Started')

        return {'FINISHED'}

    def record(self, host='0.0.0.0', port=11111):
        buchse = Buchse(host, port, as_server=True)
        start_time = time.time()
        while StartRecordFaceButtonOperator.recording:
            data, size = buchse.horch(FaceFrame.PACKET_MAX_SIZE)
            if not data or 0 == size:
                print(f'Received empty frame, skipping ...')
                continue
            frame = FaceFrame.from_raw(data, size)
            if 1/StartRecordFaceButtonOperator.hz <= time.time() - start_time:
                StartRecordFaceButtonOperator.msgs.append(frame.blendshapes)
                start_time = time.time()
            StartRecordFaceButtonOperator.last_msg = frame.blendshapes


class StopRecordFaceButtonOperator(bpy.types.Operator):
    bl_idname = "scene.stop_face_record"
    bl_label = "Stop Recording"

    def execute(self, context):
        context.scene.recording = False
        StartRecordFaceButtonOperator.recording = False
        print('Recording Stopped')
        action = bpy.data.actions.new("GST_Recorded")
        for i in range(len(StartRecordFaceButtonOperator.msgs)):
            frame = StartRecordFaceButtonOperator.msgs[i]
            frame_num = i * 48/StartRecordFaceButtonOperator.hz
            for key in Functions.keys():
                (name, index) = key.split(':')
                index = int(index)
                if key == 'head:1':
                    path = 'pose.bones[\"{bname}\"].rotation_euler'.format(bname = name)
                else:
                    path = 'pose.bones[\"{bname}\"].location'.format(bname = name)
                fc = action.fcurves.find(path, index)
                if not fc:
                    action.fcurves.new(path, index)
                try:
                    if name == 'head' or name == 'eye_offset':
                        value = Functions[key](frame)[index]
                    else:
                        value = Functions[key](frame)
                    fc.keyframe_points.insert(frame_num, value)
                except:
                    print(path, index)
                    pass
        context.object.animation_data.action = action

        return {'FINISHED'}

class StartUpdateFaceButtonOperator(bpy.types.Operator):
    bl_idname = "scene.start_update"
    bl_label = "Start Live Update"

    def execute(self, context):
        if not bpy.context.scene.liveUpdateFace:
            bpy.ops.wm.global_timer()
            bpy.ops.wm.live_update_face()
        return {'FINISHED'}

class BLGlobalTimer(bpy.types.Operator):
    """Timer  Control"""
    bl_label = "Global Timer"
    bl_idname = 'wm.global_timer'

    _timer = None
    maxFPS = 30

    def execute(self, context):
        print('Starting Timer')
        wm = context.window_manager
        self._timer = wm.event_timer_add(1/self.maxFPS, window=context.window)
        bpy.context.scene.globalTimerStarted = True
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)
        return {'PASS_THROUGH'}

    def cancel(self, context):
        print('Stopping Timer')
        if self._timer:
            wm = context.window_manager
            wm.event_timer_remove(self._timer)
        bpy.context.scene.globalTimerStarted = False
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class BLUpdateFace(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Update"
    bl_idname = 'wm.live_update_face'

    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)

        if event.type == 'TIMER':
            if StartRecordFaceButtonOperator.last_msg is not None:
                print(StartRecordFaceButtonOperator.last_msg)
                for key in Functions.keys():
                    (name, index) = key.split(':')
                    index = int(index)
                    if name == 'head' or name == 'eye_offset':
                        value = Functions[key](StartRecordFaceButtonOperator.last_msg)[index]
                    else:
                        value = Functions[key](StartRecordFaceButtonOperator.last_msg)
                    print(value)
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
        print('Starting Update Face')
        wm = context.window_manager
        wm.modal_handler_add(self)
        bpy.context.scene.liveUpdateFace = True
        if not bpy.context.scene.globalTimerStarted:
            bpy.ops.wm.global_timer()
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        print('Stopping Update Face')
        bpy.context.scene.liveUpdateFace = False
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class FaceCapturePanel(bpy.types.Panel):
    bl_label = "Record Face"
    bl_idname = "scene.record_face_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(self, context):
        return True

    def draw(self, context):
        if not context.scene.liveUpdateFace:
            self.layout.operator(StartUpdateFaceButtonOperator.bl_idname)
        self.layout.prop(bpy.context.scene, "hz")
        self.layout.operator(StartRecordFaceButtonOperator.bl_idname)
        self.layout.operator(StopRecordFaceButtonOperator.bl_idname)


bl_info = {"name": "Face Capture", "category": "User"}


def register():
    bpy.utils.register_class(StartRecordFaceButtonOperator)
    bpy.utils.register_class(StopRecordFaceButtonOperator)
    bpy.utils.register_class(StartUpdateFaceButtonOperator)
    bpy.utils.register_class(BLGlobalTimer)
    bpy.utils.register_class(BLUpdateFace)
    bpy.utils.register_class(FaceCapturePanel)


def unregister():
    bpy.utils.unregister_class(StartRecordFaceButtonOperator)
    bpy.utils.unregister_class(StopRecordFaceButtonOperator)
    bpy.utils.unregister_class(StartUpdateFaceButtonOperator)
    bpy.utils.unregister_class(BLGlobalTimer)
    bpy.utils.unregister_class(BLUpdateFace)
    bpy.utils.unregister_class(FaceCapturePanel)


if __name__ == "__main__":
    register()
