from . import Component

import pigpio


class Encoder(Component):
    def init(self, pi):
        """Initializes the encoder

        Raises
        ------
        NotImplementedError
            If the method is not implemented
        """
        raise NotImplementedError()
    
    def get_speed(self) -> float:
        """Returns the current speed of the encoder in RPM

        Returns
        -------
        float
            The current speed of the encoder in RPM

        Raises
        ------
        NotImplementedError
            If the method is not implemented
        """
        raise NotImplementedError()

    def get_angle(self) -> float:
        """Returns the current angle of the encoder in degrees

        Returns
        -------
        float
            The current angle of the encoder in degrees

        Raises
        ------
        NotImplementedError
            If the method is not implemented
        """
        raise NotImplementedError()


class HallEncoder(Encoder):
    def __init__(self, sin_pin, cos_pin, resolution=1):
        super().__init__()
        self.resolution = resolution

        self._watchdog = 200 # Milliseconds.

        self._high_tick = None
        self._low_tick = None
        self._period = None

        self._sin_pin = sin_pin
        self._cos_pin = cos_pin

        self._cos = None
        self.direction = 1

        self._ticks = 0
    
    def init(self, pi: pigpio.pi) -> bool:
        self.pi = pi
        self.pi.set_mode(self._sin_pin, pigpio.INPUT)
        self.pi.set_mode(self._cos_pin, pigpio.INPUT)

        self._cos_cb = self.pi.callback(self._cos_pin, pigpio.EITHER_EDGE, self._cos_cbf)
        self._sin_cb = self.pi.callback(self._sin_pin, pigpio.EITHER_EDGE, self._sin_cbf)
        self.pi.set_watchdog(self._sin_pin, self._watchdog)

        self._ticks = 0
        return True
        
        
    def _cos_cbf(self, _, level, tick):
        self._cos = (tick, level)
        self._ticks += self.direction

        
    def _sin_cbf(self, _, level, tick):
        if level == 0:
            self._ticks += self.direction
            self._low_tick = tick
        if level == 1: # Rising edge.
            self._ticks += self.direction
            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)
                self._period = t
                if self._cos is not None and self._low_tick is not None and self._low_tick < self._cos[0] < tick:
                    self.direction = 1 if self._cos[1] == 1 else -1
            self._high_tick = tick
        elif level == 2: # Watchdog timeout.
            if self._period is not None:
                self._period += (self._watchdog * 1000)

    def get_speed(self):
        """
        Returns the RPM.
        """
        RPM = 0.0
        if self._period is not None:
            RPM = 1 / self._period / (self.resolution / 4) * 60 * 1_000_000

        return self.direction * RPM
    
    def get_ticks(self):
        """
        Returns the number of ticks.
        """
        return self._ticks
    
    def get_angle(self):
        """
        Returns the angle in degrees.
        """
        return self._ticks * 360 / self.resolution

    def reset(self):
        self._ticks = 0
        pass

    def release(self):
        """
        Cancels the reader and releases resources.
        """
        self.pi.set_watchdog(self._sin_pin, 0) # cancel watchdog
        self._sin_cb.cancel()
        self._cos_cb.cancel()