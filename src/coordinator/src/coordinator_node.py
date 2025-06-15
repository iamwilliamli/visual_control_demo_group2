#!/usr/bin/env python2

import traceback
import rospy
from coordinator.msg import CurrentMode, GoToMode, NextMode

MODE_NAMES = {
    0: "Lane_Follow",
    1: "Count_People",
    2: "Motion_Detect",
    3: "QR_mode", # Always disabled, only here so the QR detector has an ID
    4: "Button_node",
}

ENABLED_MODES = [
    0, 1, 2
]

class CoordinatorNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing Coordinator node...")
        rospy.init_node(self.node_name, anonymous=False, xmlrpc_port=45100, tcpros_port=45101)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/co/goto",
            GoToMode,
            self.go_to_mode_cb,
            #Change buff size and queue size accordingly
            buff_size=1000,
            queue_size=10,
        )

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/co/next",
            NextMode,
            self.next_mode_cb,
            #Change buff size and queue size accordingly
            buff_size=1000,
            queue_size=10,
        )

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/co/current",
            CurrentMode,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.current_mode = ENABLED_MODES[0]
        self.prev_mode = None

        self.initialized = True
        rospy.loginfo("Node initialized!")

    def ensure_valid_mode(self):
        while True:
            if self.current_mode > max(ENABLED_MODES):
                self.current_mode = 0
            if self.current_mode in ENABLED_MODES:
                break
            self.current_mode += 1

    def go_to_mode_cb(self, msg):
        try:
            if not self.initialized:
                return
            
            self.current_mode = msg.go_to_this_mode
            self.ensure_valid_mode()
            rospy.loginfo("Command from ({}): goto ({}), current ({}) ({})".format(msg.me, msg.go_to_this_mode, self.current_mode, MODE_NAMES[self.current_mode]))
            self.update_mode()
        except Exception as e:
            rospy.logerr("Error detecting motion: %s\n%s\n", e, traceback.format_exc())

    def next_mode_cb(self, msg):
        try:
            if not self.initialized:
                return
            self.current_mode += 1
            self.ensure_valid_mode()
            rospy.loginfo("Command from ({}): next, current ({}) ({})".format(msg.me, self.current_mode, MODE_NAMES[self.current_mode]))
            self.update_mode()
        except Exception as e:
            rospy.logerr("Error in next_mode_cb: %s\n%s\n", e, traceback.format_exc())

    def publish_mode(self):
        msg = CurrentMode()
        msg.current = self.current_mode
        msg.name = MODE_NAMES[self.current_mode]
        self.publisher.publish(msg)
        rospy.loginfo("Sent current mode ({}) ({})".format(self.current_mode, MODE_NAMES[self.current_mode]))

        
    def update_mode(self):
        if self.prev_mode is None or self.prev_mode != self.current_mode:
            self.prev_mode = self.current_mode
            self.publish_mode()

if __name__ == "__main__":
    try:
        camera_node = CoordinatorNode(node_name = "coordinator_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
