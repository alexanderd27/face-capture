import bpy
import time
from threading import Thread
from llv.buchse import Buchse
from llv.gesicht import FaceFrame
from functools import partial
import math

def convert(b, w, upper, lower, frame):
    ret = 0
    for i in range(len(b)):
        ret += frame[b[i]] * w[i]
    return max(min(ret,upper), lower)

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
 'chin:2' : partial(convert, ['JawOpen', 'MouthClose', 'MouthFunnel', 'MouthPucker', 'MouthShrugUpper', 'MouthShrugLower'], [0.16, -0.16, 0.08, -0.08, -0.06, -0.06], 0.16, 0),
 'nostril_L:1' : partial(convert, ['NoseSneerLeft'], [0.08], 0.04, 0),
 'nostril_R:1' : partial(convert, ['NoseSneerRight'], [0.08], 0.04, 0),
 'cheek_L:1' : partial(convert, ['CheekSquintLeft', 'EyeSquintLeft'], [0.06, 0.04], 0.04, 0),
 'cheek_R:1' : partial(convert, ['CheekSquintRight', 'EyeSquintRight'], [0.06, 0.04], 0.04, 0),
 'lip_D_L:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthSmileLeft', 'MouthFrownLeft', 'MouthRollLower', 'MouthShrugLower', 'MouthPressLeft', 'MouthLowerDownLeft'],
    [-0.02, -0.03, -0.00, -0.04, -0.04, 0.04, -0.02, 0.01], 0.04, -0.04),
 'lip_D_R:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthSmileRight', 'MouthFrownRight', 'MouthRollLower', 'MouthShrugLower', 'MouthPressLeft', 'MouthLowerDownRight'], 
    [-0.02, -0.03, -0.00, -0.04, -0.04, 0.04, -0.02, 0.01], 0.04, -0.04),
 'lip_D:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthSmileLeft', 'MouthSmileRight', 'MouthFrownLeft', 'MouthFrownRight', 'MouthRollLower', 'MouthShrugLower'], 
    [-0.02, -0.02, -0.000, -0.000, -0.02, -0.02, -0.04, 0.04], 0.04, -0.04),
 'lip_U_L:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthSmileLeft', 'MouthFrownLeft', 'MouthRollUpper', 'MouthShrugUpper', 'MouthPressLeft', 'MouthUpperUpLeft'],
    [-0.02, -0.03, 0.01, 0.01, -0.04, 0.01, 0.01, 0.01], 0.04, -0.04),
 'lip_U_R:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthSmileRight', 'MouthFrownRight', 'MouthRollUpper', 'MouthShrugUpper', 'MouthPressRight', 'MouthUpperUpRight'], 
    [-0.02, -0.03, 0.01, 0.01, -0.04, 0.01, 0.01, 0.01], 0.04, -0.04),
 'lip_U:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthSmileLeft', 'MouthSmileRight', 'MouthFrownLeft', 'MouthFrownRight', 'MouthRollUpper', 'MouthShrugUpper'], 
    [-0.02, -0.02, 0.005, 0.005, 0.005, 0.005, -0.04, 0.01], 0.04, -0.04),
 'mouth_C_L:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthLeft', 'MouthRight', 'MouthSmileLeft', 'MouthStretchLeft'],
    [-0.06, -0.06, 0.04, -0.04, 0.08, 0.08], 0.04, -0.04),
 'mouth_C_R:1' : partial(convert, 
    ['MouthFunnel', 'MouthPucker', 'MouthLeft', 'MouthRight', 'MouthSmileRight', 'MouthStretchRight'],
    [-0.06, -0.06, -0.04, 0.04, 0.08, 0.08], 0.04, -0.04),
 'mouth_D_L:1' : partial(convert, ['JawOpen', 'MouthFrownLeft', 'MouthStretchLeft'], [-0.05, 0.08, 0.04], 0.04, 0),
 'mouth_D_R:1' : partial(convert, ['JawOpen', 'MouthFrownRight', 'MouthStretchRight'], [-0.05, 0.08, 0.04], 0.04, 0),
 'mouth_U_L:1' : partial(convert, ['MouthSmileLeft'], [0.08], 0.04, 0),
 'mouth_U_R:1' : partial(convert, ['MouthSmileRight'], [0.08], 0.04, 0),
 'brow.inner.L:1' : partial(convert, ['BrowInnerUp', 'BrowDownLeft'], [0.06, -0.06], 0.04, -0.04),
 'brow.inner.R:1' : partial(convert, ['BrowInnerUp', 'BrowDownRight'], [0.06, -0.06], 0.04, -0.04),
 'brow.C:1' : partial(convert, ['BrowInnerUp', 'BrowDownLeft', 'BrowDownRight'], [0.06, -0.03, -0.03], 0.04, -0.04),
 'brow.L:1' : partial(convert, ['BrowOuterUpLeft', 'BrowDownLeft'], [0.06, -0.06], 0.04, -0.04),
 'brow.R:1' : partial(convert, ['BrowOuterUpRight', 'BrowDownRight'], [0.06, -0.06], 0.04, -0.04),
 'eyelid_LO_L:1' : partial(convert, ['EyeSquintLeft'], [-0.04], 0.04, -0.04),
 'eyelid_LO_R:1' : partial(convert, ['EyeSquintRight'], [-0.04], 0.04, -0.04),
 'eyelid_UP_L:1' : partial(convert, ['EyeBlinkLeft', 'EyeWideLeft'], [-0.04, 0.04], 0.04, -0.04), 
 'eyelid_UP_R:1' : partial(convert, ['EyeBlinkRight', 'EyeWideRight'], [-0.04, 0.04], 0.04, -0.04)
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
 'EyesYaw': partial(convert, ['LeftEyeYaw', 'RightEyeYaw'], [-math.pi/4, -math.pi/4], 2, -2),
 'EyesPitch': partial(convert, ['LeftEyePitch', 'RightEyePitch'], [-math.pi/4, -math.pi/4], 2, -2),
 'HeadYaw' : partial(convert, ['HeadYaw'], [-math.pi/2], 2, -2),
 'HeadRoll' : partial(convert, ['HeadRoll'], [math.pi/2], 2, -2),
 'HeadPitch' : partial(convert, ['HeadPitch'], [math.pi/2], 2, -2),
 'Jaw' : partial(convert, ['JawOpen', 'MouthClose'], [1, -1], 1, 0),
 'SneerLeft' : lambda x : 25 * Functions['nostril_L:1'](x),
 'SneerRight' : lambda x : 25 * Functions['nostril_R:1'](x),
 'CheekSquintLeft' : lambda x : 25 * Functions['cheek_L:1'](x),
 'CheekSquintRight' : lambda x : 25 * Functions['cheek_R:1'](x),
 'LowerLipLeft' : lambda x : 25 * Functions['lip_D_L:1'](x),
 'LowerLipRight' : lambda x : 25 * Functions['lip_D_R:1'](x),
 'LowerLipCenter' : lambda x : 25 * Functions['lip_D:1'](x),
 'UpperLipLeft' : lambda x : 25 * Functions['lip_U_L:1'](x),
 'UpperLipRight' : lambda x : 25 * Functions['lip_U_R:1'](x),
 'UpperLipCenter' : lambda x : 25 * Functions['lip_U:1'](x),
 'FrownLeft' : lambda x : 25 * Functions['mouth_D_L:1'](x),
 'FrownRight' : lambda x : 25 * Functions['mouth_D_R:1'](x),
 'LipStretchLeft' : lambda x : 25 * Functions['mouth_C_L:1'](x),
 'LipStretchRight' : lambda x : 25 * Functions['mouth_C_R:1'](x),
 'SmileLeft' : lambda x : 25 * Functions['mouth_U_L:1'](x),
 'SmileRight' : lambda x : 25 * Functions['mouth_U_R:1'](x),
 'BrowInnerLeft' : lambda x : 25 * Functions['brow.inner.L:1'](x),
 'BrowInnerRight' : lambda x : 25 * Functions['brow.inner.R:1'](x),
 'BrowCenter' :  lambda x : 25 * Functions['brow.C:1'](x),
 'BrowOuterLeft' : lambda x : 25 * Functions['brow.L:1'](x),
 'BrowOuterRight' : lambda x : 25 * Functions['brow.R:1'](x),
 'LowerLidLeft' : lambda x : 25 * Functions['eyelid_LO_L:1'](x),
 'LowerLidRight' : lambda x : 25 * Functions['eyelid_LO_R:1'](x),
 'UpperLidLeft' : lambda x : 25 * Functions['eyelid_UP_L:1'](x),
 'UpperLidRight' :lambda x : 25 * Functions['eyelid_UP_R:1'](x),
}

class UIProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.faceRecording = bpy.props.BoolProperty(name = "Recording", default = False)
    bpy.types.Scene.faceGlobalTimerStarted = bpy.props.BoolProperty(name = "globalTimerStarted", default = False)
    bpy.types.Scene.faceLiveUpdatePose = bpy.props.BoolProperty(name = "liveUpdatePose", default = False)
    bpy.types.Scene.faceStreaming = bpy.props.BoolProperty(name = "Streaming", default = False)
    bpy.types.Scene.faceRecordingHz = bpy.props.IntProperty(name = "Recording Hz", default = 1, soft_min = 1, soft_max = 20)
    bpy.types.Scene.mirrorUpdate = bpy.props.BoolProperty(name = "Mirror Update", default = False)
    bpy.types.Scene.mirrorRecord = bpy.props.BoolProperty(name = "Mirror Recording", default = False)
    bpy.types.Scene.recordHeadMovement = bpy.props.BoolProperty(name = "Head Movement", default = True)
    bpy.types.Scene.recordEyeMovement = bpy.props.BoolProperty(name = "Eye Movement", default = True)
    bpy.types.Scene.recordEyeLeft = bpy.props.BoolProperty(name = "Left Eye", default = True)
    bpy.types.Scene.recordEyeRight = bpy.props.BoolProperty(name = "Right Eye", default = True)
    bpy.types.Scene.recordMouth = bpy.props.BoolProperty(name = "Mouth", default = True)
    bpy.types.Scene.recordNostril = bpy.props.BoolProperty(name = "Nose Sneer", default = True)
    bpy.types.Scene.recordCheek = bpy.props.BoolProperty(name = "Cheek Sneer", default = True)


class FaceStartPortButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_port"
    bl_label = "Connect to Phone"
    
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
            try:
                frame = FaceFrame.from_raw(data, size)
            except:
                continue
            FaceStartPortButtonOperator.last_msg = frame.blendshapes

            msg = FaceStartPortButtonOperator.last_msg
            if FaceSetNeutralButtonOperator.neutral_pose:
                msg = {b : (v - FaceSetNeutralButtonOperator.neutral_pose[b]) for (b, v) in msg.items()}

            if FaceStartStreamButtonOperator.face_publisher:
                if not imported:
                    import rospy
                    from hr_msgs.msg import TargetPosture
                    imported = True

                tp = TargetPosture()
                for key in StreamingFunctions.keys():
                    value = StreamingFunctions[key](msg)
                    tp.values.append(value)
                tp.names = list(StreamingFunctions.keys())
                FaceStartStreamButtonOperator.face_publisher.publish(tp)

            if FaceStartRecordButtonOperator.recording and 1/FaceStartRecordButtonOperator.hz <= (time.time() - start_time):
                FaceStartRecordButtonOperator.msgs.append(msg)
                start_time = time.time()


class FaceStartRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_record"
    bl_label = "Start Recording on Blender"

    msgs = []
    hz = 0
    recording = False

    def execute(self, context):
        FaceStartRecordButtonOperator.msgs = []
        FaceStartRecordButtonOperator.hz = context.scene.faceRecordingHz
        FaceStartRecordButtonOperator.recording = True
        context.scene.faceRecording = True

        return {'FINISHED'}


class FaceStopRecordButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_stop_record"
    bl_label = "Stop Recording"

    def execute(self, context):
        context.scene.faceRecording = False
        FaceStartRecordButtonOperator.recording = False
        action = bpy.data.actions.new("GST-Recorded")

        for i in range(len(FaceStartRecordButtonOperator.msgs)):
            frame = FaceStartRecordButtonOperator.msgs[i]
            frame_num = i * 48/FaceStartRecordButtonOperator.hz

            if context.scene.mirrorRecord:
                f = MirroredFunctions
            else:
                f = Functions
            
            to_record = list(f.keys())
            if not context.scene.recordHeadMovement:
                to_record = [key for key in to_record if not "head" in key]
            if not context.scene.recordEyeMovement:
                to_record = [key for key in to_record if not "eye_offset" in key]
            if not context.scene.recordEyeLeft:
                to_record = [key for key in to_record if not key in ['brow.inner.L:1', 'brow.L:1', 'brow.C:1', 'eyelid_LO_L:1', 'eyelid_UP_L:1']]
            if not context.scene.recordEyeRight:
                to_record = [key for key in to_record if not key in ['brow.inner.R:1', 'brow.R:1', 'brow.C:1', 'eyelid_LO_R:1', 'eyelid_UP_R:1']]
            if not context.scene.recordMouth:
                to_record = [key for key in to_record if not "mouth" in key and not "lip" in key and not "chin" in key]
            if not context.scene.recordNostril:
                to_record = [key for key in to_record if not "nostril" in key]
            if not context.scene.recordCheek:
                to_record = [key for key in to_record if not "cheek" in key]

            for key in to_record:
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


class FaceSetNeutralButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_set_neutral_pose"
    bl_label = "Set Neutral Pose"

    neutral_pose = None

    def execute(self, context):
        if FaceStartPortButtonOperator.last_msg:
            FaceSetNeutralButtonOperator.neutral_pose = FaceStartPortButtonOperator.last_msg

        return {'FINISHED'}


class FaceToggleUpdateButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_toggle_update"
    bl_label = "Toggle Live Update"

    def execute(self, context):
        if not bpy.context.scene.faceLiveUpdatePose:
            bpy.context.scene.faceGlobalTimerStarted = True
            bpy.context.scene.faceLiveUpdatePose = True
            bpy.ops.wm.face_global_timer()
            bpy.ops.wm.face_live_update_pose()
        else:
            bpy.context.scene.faceGlobalTimerStarted = False
            bpy.context.scene.faceLiveUpdatePose = False

        return {'FINISHED'}


class FaceStartNodeButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_node"
    bl_label = "Start ROS Node"


    def execute(self, context):
        try: 
            import rospy
            from hr_msgs.msg import TargetPosture
            rospy.init_node("face_capture")
            print("Node Started")
        except Exception as e:
            self.report({"WARNING"}, "Cant start the node with exception {}".format(e))

        return {'FINISHED'}


class FaceStartStreamButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_start_stream"
    bl_label = "Start Streaming to Robot"

    face_publisher = None

    def execute(self, context):
        try:
            import rospy
            from hr_msgs.msg import TargetPosture
        except:
            return

        FaceStartStreamButtonOperator.face_publisher = rospy.Publisher('/hr/animation/set_state', TargetPosture, queue_size=4)
        context.scene.faceStreaming = True

        return {'FINISHED'}


class FaceStopStreamButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_stop_stream"
    bl_label = "Stop Streaming"

    def execute(self, context):
        FaceStartStreamButtonOperator.face_publisher = None
        context.scene.faceStreaming = False
    
        return{'FINISHED'}


class FaceResetPoseButtonOperator(bpy.types.Operator):
    bl_idname = "scene.face_reset_pose"
    bl_label = "Reset Robot Pose"

    def execute(self, context):
        try:
            import rospy
            from hr_msgs.msg import TargetPosture
        except:
            return

        reset_publisher = rospy.Publisher('/hr/animation/set_state', TargetPosture, queue_size=4)
        tp = TargetPosture()
        for key in StreamingFunctions.keys():
            tp.names.append(key)
            tp.values.append(0)
        reset_publisher.publish(tp)

        return{'FINISHED'}


class FaceBLGlobalTimer(bpy.types.Operator):
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
        if not context.scene.faceGlobalTimerStarted:
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


class FaceBLUpdatePose(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Update"
    bl_idname = 'wm.face_live_update_pose'

    def modal(self, context, event):
        if not context.scene.faceLiveUpdatePose:
            return self.cancel(context)

        if event.type == 'TIMER':
            if FaceStartPortButtonOperator.last_msg:
                if context.scene.mirrorUpdate:
                    f = MirroredFunctions
                else:
                    f = Functions

                msg = FaceStartPortButtonOperator.last_msg
                if FaceSetNeutralButtonOperator.neutral_pose:
                    msg = {b : (v - FaceSetNeutralButtonOperator.neutral_pose[b]) for (b, v) in msg.items()}

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
        if not context.scene.faceGlobalTimerStarted:
            bpy.ops.wm.face_global_timer()

        return {'RUNNING_MODAL'}

    def cancel(self, context):
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return True


class FaceView3DPanel:
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(self, context):
        return True


class FaceUpdatePanel(FaceView3DPanel, bpy.types.Panel):
    bl_label = "Face Update"
    bl_idname = "scene.face_update_panel"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator(FaceStartPortButtonOperator.bl_idname)
        if not context.scene.faceLiveUpdatePose:
            row.operator(FaceToggleUpdateButtonOperator.bl_idname, text = "Start Live Update")
        else:
            row.operator(FaceToggleUpdateButtonOperator.bl_idname, text = "Stop Live Update")

        row = layout.row()
        row.operator(FaceSetNeutralButtonOperator.bl_idname)
        row.prop(bpy.context.scene, "mirrorUpdate")


class FaceRecordPanel(FaceView3DPanel, bpy.types.Panel):
    bl_idname = "scene.face_record_panel"
    bl_label = "Face Record"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(bpy.context.scene, "faceRecordingHz")
        row.prop(bpy.context.scene, "mirrorRecord")
        if not context.scene.faceRecording:
            self.layout.operator(FaceStartRecordButtonOperator.bl_idname)
        else:
            self.layout.operator(FaceStopRecordButtonOperator.bl_idname)
        
        row = layout.row()
        row.prop(bpy.context.scene, "recordHeadMovement")
        row.prop(bpy.context.scene, "recordEyeMovement")
        row = layout.row()
        row.prop(bpy.context.scene, "recordEyeLeft")
        row.prop(bpy.context.scene, "recordEyeRight")
        row = layout.row()
        row.prop(bpy.context.scene, "recordMouth")
        row.prop(bpy.context.scene, "recordNostril")
        row = layout.row()
        row.prop(bpy.context.scene, "recordCheek")


class FaceStreamPanel(FaceView3DPanel, bpy.types.Panel):
    bl_idname = "scene.face_stream_panel"
    bl_label = "Face Stream"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator(FaceStartNodeButtonOperator.bl_idname)
        row.operator(FaceResetPoseButtonOperator.bl_idname)

        row = layout.row()
        if not context.scene.faceStreaming:
            row.operator(FaceStartStreamButtonOperator.bl_idname)
        else:
            row.operator(FaceStopStreamButtonOperator.bl_idname)


bl_info = {"name": "Face Capture", "category": "User"}

def register():
    bpy.utils.register_class(FaceStartRecordButtonOperator)
    bpy.utils.register_class(FaceStopRecordButtonOperator)
    bpy.utils.register_class(FaceStartPortButtonOperator)
    bpy.utils.register_class(FaceToggleUpdateButtonOperator)
    bpy.utils.register_class(FaceSetNeutralButtonOperator)
    bpy.utils.register_class(FaceStartNodeButtonOperator)
    bpy.utils.register_class(FaceStartStreamButtonOperator)
    bpy.utils.register_class(FaceStopStreamButtonOperator)
    bpy.utils.register_class(FaceResetPoseButtonOperator)
    bpy.utils.register_class(FaceBLGlobalTimer)
    bpy.utils.register_class(FaceBLUpdatePose)
    bpy.utils.register_class(FaceUpdatePanel)
    bpy.utils.register_class(FaceRecordPanel)
    bpy.utils.register_class(FaceStreamPanel)

def unregister():
    bpy.utils.unregister_class(FaceStartRecordButtonOperator)
    bpy.utils.unregister_class(FaceStopRecordButtonOperator)
    bpy.utils.unregister_class(FaceStartPortButtonOperator)
    bpy.utils.unregister_class(FaceToggleUpdateButtonOperator)
    bpy.utils.unregister_class(FaceSetNeutralButtonOperator)
    bpy.utils.unregister_class(FaceStartNodeButtonOperator)
    bpy.utils.unregister_class(FaceStartStreamButtonOperator)
    bpy.utils.unregister_class(FaceStopStreamButtonOperator)
    bpy.utils.unregister_class(FaceResetPoseButtonOperator)
    bpy.utils.unregister_class(FaceBLGlobalTimer)
    bpy.utils.unregister_class(FaceBLUpdatePose)
    bpy.utils.unregister_class(FaceUpdatePanel)
    bpy.utils.unregister_class(FaceRecordPanel)
    bpy.utils.unregister_class(FaceStreamPanel)

if __name__ == "__main__":
    register()
