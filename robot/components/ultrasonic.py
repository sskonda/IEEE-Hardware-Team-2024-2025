from . import Component

import pigpio

SPEED_OF_SOUND = 74.0525 * 2  # us / in (two-way)

# TODO: Might need an echo lock to prevent sensors from interfering, check with hardware

class Ultrasonic(Component):
    def __init__(self, trigger_pin, echo_pin):
        super().__init__()
        self._trigger_pin = trigger_pin
        self._echo_pin = echo_pin

        self._high_tick = None
        self._period = None

    def init(self, pi):
        self.pi = pi
        self.pi.set_mode(self._trigger_pin, pigpio.OUTPUT)
        self.pi.set_mode(self._echo_pin, pigpio.INPUT)

        self.pi.callback(self._echo_pin, pigpio.EITHER_EDGE, self._echo_cbf)
    
    def ping(self):
        self.pi.gpio_trigger(self._trigger_pin, 10, pigpio.HIGH)

    def _echo_cbf(self, gpio, level, tick):
        if level == pigpio.LOW:  # Falling edge
            if self._high_tick is not None:
                self._period = pigpio.tickDiff(self._high_tick, tick)
                self._high_tick = None
        if level == pigpio.HIGH:  # Rising edge.
            self._high_tick = tick

    def get_range(self):
        if self._period is None:
            return 0
        return self._period / SPEED_OF_SOUND