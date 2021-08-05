import time
from threading import Thread
from llv.buchse import Buchse
from llv.gesicht import FaceFrame


FACE_BLENDSHAPE_NAMES = [
    "EyeBlinkLeft",
    "EyeLookDownLeft",
    "EyeLookInLeft",
    "EyeLookOutLeft",
    "EyeLookUpLeft",
    "EyeSquintLeft",
    "EyeWideLeft",
    "EyeBlinkRight",
    "EyeLookDownRight",
    "EyeLookInRight",
    "EyeLookOutRight",
    "EyeLookUpRight",
    "EyeSquintRight",
    "EyeWideRight",
    "JawForward",
    "JawLeft",
    "JawRight",
    "JawOpen",
    "MouthClose",
    "MouthFunnel",
    "MouthPucker",
    "MouthLeft",
    "MouthRight",
    "MouthSmileLeft",
    "MouthSmileRight",
    "MouthFrownLeft",
    "MouthFrownRight",
    "MouthDimpleLeft",
    "MouthDimpleRight",
    "MouthStretchLeft",
    "MouthStretchRight",
    "MouthRollLower",
    "MouthRollUpper",
    "MouthShrugLower",
    "MouthShrugUpper",
    "MouthPressLeft",
    "MouthPressRight",
    "MouthLowerDownLeft",
    "MouthLowerDownRight",
    "MouthUpperUpLeft",
    "MouthUpperUpRight",
    "BrowDownLeft",
    "BrowDownRight",
    "BrowInnerUp",
    "BrowOuterUpLeft",
    "BrowOuterUpRight",
    "CheekPuff",
    "CheekSquintLeft",
    "CheekSquintRight",
    "NoseSneerLeft",
    "NoseSneerRight",
    "TongueOut",
    "HeadYaw",
    "HeadPitch",
    "HeadRoll",
    "LeftEyeYaw",
    "LeftEyePitch",
    "LeftEyeRoll",
    "RightEyeYaw",
    "RightEyePitch",
    "RightEyeRoll",
]

def record(host='0.0.0.0', port=11111):
    buchse = Buchse(host, port, as_server = True)
    current_data_frame = 0
    while current_data_frame < 300:
        data, size = buchse.horch(FaceFrame.PACKET_MAX_SIZE)
        if not data or 0 == size:
            print(f'Received empty frame, skipping ...')
            continue
        try:
            frame = FaceFrame.from_raw(data, size)
            print(frame.blendshapes['LeftEyeYaw'])
            print(frame.blendshapes['LeftEyeYaw']/2)
            print(float(frame.blendshapes['LeftEyeYaw']))
        except Exception as e:
            print(f'Encountered: {e}')
            print(f'Skipping frame ...')
            current_data_frame += 1
            continue
        current_data_frame += 1


t=Thread(target=record)
t.daemon = True
t.start()