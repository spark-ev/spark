#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, String

class CarController:
    def __init__(self):
        rospy.init_node('car_controller', anonymous=True)
        self.speed_publisher = rospy.Publisher('/car/speed', Float32, queue_size=10)
        self.light_publisher = rospy.Publisher('/car/lights', String, queue_size=10)
        self.current_speed = 0.0

    def set_speed(self, speed):
        self.current_speed = speed
        self.speed_publisher.publish(self.current_speed)
        rospy.loginfo(f"Speed set to: {self.current_speed}")

    def set_light(self, command):
        self.light_publisher.publish(command)
        rospy.loginfo(f"Light command: {command}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = CarController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
