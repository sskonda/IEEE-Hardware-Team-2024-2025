import pigpio

PI = pigpio.pi()
PI.wave_clear()
PI.wave_tx_stop()

pulses = [
    pigpio.pulse(1 << 22, 1 << 23, 8),
    pigpio.pulse(1 << 23, 1 << 22, 8)
]

PI.wave_add_new()
PI.wave_add_generic(pulses)
wave_id = PI.wave_create()

PI.wave_chain([
    255, 0,
            255, 0, 
            wave_id,
            255, 1, 10, 0,
        255, 2, 100, 0,
    255, 3
])

input("Press Enter to Continue...")

PI.wave_send_repeat(wave_id)

input("Press Enter to Continue...")

PI.wave_tx_stop()
PI.wave_clear()