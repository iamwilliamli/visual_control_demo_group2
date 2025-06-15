#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS Display Node for showing stats and OCR text on OLED display
Modified from Adafruit SSD1306 example code
"""

import time
import rospy
import threading
from std_msgs.msg import String, Int32MultiArray
from coordinator.msg import CurrentMode

import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess

class DisplayStatsNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('display_stats_node', anonymous=True)
        
        # Data storage with thread locks
        self.ocr_text = "No OCR text"
        self.current_mode = "Unknown"
        self.encoder_ticks = [0, 0]
        self.data_lock = threading.Lock()
        
        # Initialize display first
        self.init_display()
        
        # ROS subscribers
        self.ocr_sub = rospy.Subscriber('/ocr/text', String, self.ocr_callback, queue_size=10)
        self.mode_sub = rospy.Subscriber('/co/current', CurrentMode, self.mode_callback, queue_size=10)
        
        rospy.loginfo("Display Stats Node initialized")
        rospy.loginfo("Waiting for OCR messages on /ocr/text and coordinator messages on /co/current...")
        
        # Start display update timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.display_update_callback)
    
    def init_display(self):
        """Initialize the OLED display"""
        # 128x32 display with hardware I2C:
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, i2c_address=0x3C)
        
        # Initialize library.
        self.disp.begin()
        
        # Clear display.
        self.disp.clear()
        self.disp.display()
        
        # Create blank image for drawing.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        
        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        self.padding = -2
        self.top = self.padding
        self.x = 0
        
        # Load default font.
        self.font = ImageFont.load_default()
    
    def ocr_callback(self, msg):
        """Callback for OCR text messages"""
        with self.data_lock:
            self.ocr_text = msg.data
            rospy.loginfo("Received OCR text: {}".format(self.ocr_text))
    
    def mode_callback(self, msg):
        """Callback for coordinator mode messages"""
        with self.data_lock:
            self.current_mode = msg.name
            rospy.loginfo("Received mode: {}".format(self.current_mode))
    
    def display_update_callback(self, event):
        """Timer callback to update display"""
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        # Get system stats
        try:
            cmd = "hostname -I | cut -d\' \' -f1"
            IP = subprocess.check_output(cmd, shell=True).decode().strip()
            cmd = "top -bn1 | grep load | awk '{printf \"CPU: %.1f\", $(NF-2)}'"
            CPU = subprocess.check_output(cmd, shell=True).decode().strip()
            cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB\", $3,$2 }'"
            MemUsage = subprocess.check_output(cmd, shell=True).decode().strip()
        except:
            IP = "Unknown"
            CPU = "CPU: N/A"
            MemUsage = "Mem: N/A"

        # Get ROS data
        ocr_text, mode_text = self.get_display_data()
        rospy.loginfo(ocr_text)
        rospy.loginfo(mode_text)       
        # Truncate texts if too long
        if len(ocr_text) > 18:
            ocr_text = ocr_text[:15] + "..."
        if len(mode_text) > 18:
            mode_text = mode_text[:15] + "..."

        # Write system stats and ROS data
        self.draw.text((self.x, self.top),       "IP: {}".format(IP), font=self.font, fill=255)
        self.draw.text((self.x, self.top+8),     CPU, font=self.font, fill=255)
        self.draw.text((self.x, self.top+16),    MemUsage, font=self.font, fill=255)
        self.draw.text((self.x, self.top+24),    "OCR: {}".format(ocr_text), font=self.font, fill=255)
        self.draw.text((self.x, self.top+32),    "Mode: {}".format(mode_text), font=self.font, fill=255)

        # Display image.
        self.disp.image(self.image)
        self.disp.display()

    def get_display_data(self):
        """Thread-safe getter for display data"""
        with self.data_lock:
            return self.ocr_text, self.current_mode

if __name__ == '__main__':
    try:
        # Create and run the display node
        display_node = DisplayStatsNode()
        rospy.loginfo("Display node started. Press Ctrl+C to exit.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Display node shutting down")
    except KeyboardInterrupt:
        rospy.loginfo("Display node interrupted by user")
    except Exception as e:
        rospy.logerr("Display node error: {}".format(e))
