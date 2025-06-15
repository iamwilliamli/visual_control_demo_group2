# #!/usr/bin/env python2

# import time
# from buttonClass import ButtonEvent, ButtonDriver

# LED_GPIO = 37
# SIGNAL_GPIO = 40


# class button:
#     def __init__(self):
#         self.driver = ButtonDriver(LED_GPIO, SIGNAL_GPIO, self.event_cb)
#         self.ledState = 1
#         self.driver.led.set(self.ledState)

#     def event_cb(self, event):
#         if event == ButtonEvent.PRESS:
#             print("Press event")
#             return
#         if event == ButtonEvent.RELEASE:
#             print("Release event")
#             self.ledState ^= 1
#             self.driver.led.set(self.ledState)
#             return


# button = button()

# while 1:
#     time.sleep(0.1)
