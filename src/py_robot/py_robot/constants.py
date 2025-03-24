DRIVE_WHEEL_DIAMETER = 3.14961
DRIVE_WHEEL_OFFTANGENT = 24.73
DRIVE_TICKS_PER_REV = 1200
DRIVE_SPEED = 0.8
DRIVE_WHEEL_SPACING = 12.48917

DRIVE_P = 1/60
DRIVE_I = 0
DRIVE_D = 0.1
DRIVE_FF = 0.1

START = (31.25, 4.625, -90.0)

APRILTAG_POSE = {  # inches, 0,0 top left corner, 0 degrees is east, world space
    0: ([0.0, 22.5, 2.0], 0.0),
    1: ([0.0, 22.5, 2.0], 0.0),
    2: ([0.0, 22.5, 2.0], 0.0),
    3: ([0.0, 22.5, 2.0], 0.0),
    4: ([0.0, 22.5, 2.0], 0.0),
    5: ([32.0, 45.0, 2.0], 270.0),
    6: ([44.0, 0.0, 2.0], 90.0),
    7: ([93.0, 22.5, 2.0], 180.0)
}

TAG_WIDTH = 3.125
TAG_POINTS = [  # object points of the april tag
    [0.0, -TAG_WIDTH / 2, -TAG_WIDTH / 2,],
    [0.0,  TAG_WIDTH / 2, -TAG_WIDTH / 2,],
    [0.0,  TAG_WIDTH / 2,  TAG_WIDTH / 2,],
    [0.0, -TAG_WIDTH / 2,  TAG_WIDTH / 2,]
]

CAMERA_CALIBRATION_FILE = "camera_calibration_data.npz"

CAMERA_CONFIGURATION = {
    "transform": {
        "hflip": True,
        "vflip": True,
    }
}
