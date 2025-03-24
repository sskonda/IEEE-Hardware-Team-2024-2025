import time

from py_robot.py_robot.encoder import Encoder
from py_robot.py_robot.motor import DutyMotor
from robot.components.tank_drive import TankDrive


class Action:
    def __init__(self, start=None, is_finished=None, repeated=None, end=None):
        self.start = start
        self.end = end
        self.repeated = repeated
        self.is_finished = is_finished

        self.done = False

    def __call__(self):
        if not self.done:
            if self.start:
                self.start()
                self.start = None
            if self.repeated:
                self.repeated()
            if self.is_finished:
                self.done = self.is_finished()
            if self.done and self.end:
                self.end()
        return self.done




def DriveTimeout(DRIVE: TankDrive, direction, duration, **kwargs):
    s = {}

    _start = kwargs.pop("start", None)
    _is_finished = kwargs.pop("is_finished", None)
    _repeated = kwargs.pop("repeated", None)
    _end = kwargs.pop("end", None)

    if _start is not None:

        def start():
            _start()
            s["start_time"] = time.time()

    else:

        def start():
            s["start_time"] = time.time()

    if _repeated is not None:

        def repeated():
            _repeated()
            DRIVE.drive_to_position(DRIVE.current_position + direction)

    else:

        def repeated():
            DRIVE.drive_to_position(DRIVE.current_position + direction)

    if _is_finished is not None:

        def is_finished():
            return _is_finished() and (time.time() - s["start_time"] > duration)

    else:

        def is_finished():
            return time.time() - s["start_time"] > duration
        
    if _end is not None:
        
        def end():
            _end()
            DRIVE.stop()
    
    else:
        
        def end():
            DRIVE.stop()

    return Action(start=start, is_finished=is_finished, repeated=repeated, end=end, **kwargs)


def DriveToPosition(DRIVE: TankDrive, position, tolerance=1.0, **kwargs):
    _start = kwargs.pop("start", None)
    _is_finished = kwargs.pop("is_finished", None)
    _end = kwargs.pop("end", None)

    if _start is not None:

        def start():
            _start()
            DRIVE.drive_to_position(position, tolerance)

    else:

        def start():
            DRIVE.drive_to_position(position, tolerance)

    if _is_finished is not None:
        is_finished = lambda: _is_finished() and DRIVE.at_target()
    else:
        is_finished = DRIVE.at_target
        
    if _end is not None:
        
        def end():
            _end()
            DRIVE.stop()
        
    else:
        
        def end():
            DRIVE.stop()

    return Action(start=start, is_finished=is_finished, end=end, **kwargs)


def DriveToHeading(DRIVE: TankDrive, heading, tolerance=1.0, **kwargs):
    _start = kwargs.pop("start", None)
    _is_finished = kwargs.pop("is_finished", None)
    _end = kwargs.pop("end", None)

    if _start is not None:

        def start():
            _start()
            DRIVE.drive_to_heading(heading, tolerance)

    else:

        def start():
            DRIVE.drive_to_heading(heading, tolerance)

    if _is_finished is not None:
        is_finished = lambda: _is_finished() and DRIVE.at_target()
    else:
        is_finished = DRIVE.at_target
    
    if _end is not None:
        
        def end():
            _end()
            DRIVE.stop()
    
    else:
        
        def end():
            DRIVE.stop()

    return Action(start=start, is_finished=is_finished, end=end, **kwargs)


def WithTimeout(seconds, **kwargs):
    _start = kwargs.pop("start", None)
    _is_finished = kwargs.pop("is_finished", None)

    s = {}
    if _start is not None:

        def start():
            _start()
            s["start_time"] = time.time()

    else:

        def start():
            s["start_time"] = time.time()

    if _is_finished is not None:

        def is_finished():
            return time.time() - s["start_time"] > seconds or _is_finished()

    else:

        def is_finished():
            return time.time() - s["start_time"] > seconds

    return Action(start=start, is_finished=is_finished)


def ClampTighten(CLAMP_MOTOR: DutyMotor, CLAMP_ENCODER: Encoder, **kwargs):
    def start():
        CLAMP_MOTOR.set_duty(-1.0)
    
    def stop():
        CLAMP_MOTOR.set_duty(0.0)
    
    # detect when the motor is no longer moving
    s = {"max_speed": 0.0}
    def is_finished():
        speed = abs(CLAMP_ENCODER.get_speed())
        s["max_speed"] = max(s["max_speed"], speed)
        return speed < s["max_speed"] and speed < 100.0

    return Action(start=start, is_finished=is_finished, end=stop, **kwargs)


def ClampRelease(CLAMP_MOTOR: DutyMotor, CLAMP_ENCODER: Encoder, **kwargs):
    def start():
        CLAMP_MOTOR.set_duty(1.0)
    
    def stop():
        CLAMP_MOTOR.set_duty(0.0)
    
    # detect when the motor is no longer moving
    s = {"max_speed": 0.0}
    def is_finished():
        speed = abs(CLAMP_ENCODER.get_speed())
        s["max_speed"] = max(s["max_speed"], speed)
        return speed < s["max_speed"] and speed < 100.0

    return Action(start=start, is_finished=is_finished, end=stop, **kwargs)