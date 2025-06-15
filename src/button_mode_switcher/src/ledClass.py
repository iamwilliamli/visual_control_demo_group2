import time
import Jetson.GPIO as GPIO


class ButtonLED:
    def __init__(self, gpio_pin):
        if not 1 <= gpio_pin <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
        self._gpio_pin = gpio_pin
        self._is_shutdown = False
        # configure GPIO pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._gpio_pin, GPIO.OUT)

    def on(self):
        if self._is_shutdown:
            return
        GPIO.output(self._gpio_pin, GPIO.HIGH)

    def off(self):
        if self._is_shutdown:
            return
        GPIO.output(self._gpio_pin, GPIO.LOW)

    def set(self, value):
        if self._is_shutdown:
            return
        if value not in [0, 1]:
            raise ValueError("LED value can only be on of [0, 1]")
        GPIO.output(self._gpio_pin, value)

    def confirm_shutdown(self):
        # top button LED blinks for 3 seconds
        self.blink_led(secs_to_blink=3, blink_freq_hz=2)

    def blink_led(self, secs_to_blink, blink_freq_hz):
        # duration of an off-on or on-off transition
        transition_duration_sec = 1.0 / (2 * blink_freq_hz)
        for _ in range(secs_to_blink):
            for _ in range(blink_freq_hz):
                self.off()
                time.sleep(transition_duration_sec)
                self.on()
                time.sleep(transition_duration_sec)

    def shutdown(self):
        self._is_shutdown = True
        self.off()
