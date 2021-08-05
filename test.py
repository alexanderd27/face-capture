from .cli import read_frames
from .gesicht import FaceFrame

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

def main():
    d = dict.fromkeys(FACE_BLENDSHAPE_NAMES)
    for frame_json in read_frames("dao.klare-gesichter", is_binary=False, loop=False):

        frame = FaceFrame.from_json(frame_json)
        print(frame.encode())
        print('ahahahaha')

if '__name__' == '__main__':
    main()