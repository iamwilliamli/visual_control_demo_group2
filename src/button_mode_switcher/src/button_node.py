#!/usr/bin/env python2

import rospy
from coordinator.msg import NextMode
from buttonClass import ButtonEvent, ButtonDriver

LED_GPIO = 37
SIGNAL_GPIO = 40

NODE_ID = 4

class ButtonNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing Button node...")
        rospy.init_node(self.node_name, anonymous=True)

        self.driver = ButtonDriver(LED_GPIO, SIGNAL_GPIO, self.event_cb)
        self.ledState = 0
        self.driver.led.set(self.ledState)

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/co/next",
            NextMode,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("Node initialized!")
        

    def event_cb(self, event):
        if event == ButtonEvent.PRESS:
            rospy.loginfo("Button press")
            self.ledState ^= 1
            self.driver.led.set(self.ledState)
            self.command_next_mode()
        # if event == ButtonEvent.RELEASE:
        #     return

    def command_next_mode(self):
        msg = NextMode()
        msg.me = NODE_ID
        self.publisher.publish(msg)

if __name__ == "__main__":
    try:
        camera_node = ButtonNode(node_name = "button_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
