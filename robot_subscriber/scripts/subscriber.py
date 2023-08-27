#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class RobotSubscriber(Node):
    def __init__(self):
        super().__init__("robot_sub_node")
        # Robot Subscriber Node
        self.sub = self.create_subscription(Twist, "/cmd_vel_joy", self.robot_response, 10)

        self.pwm_pin_esc = 33  # Change to your ESC GPIO pin
        self.pwm_pin_servo = 32  # Change to your servo GPIO pin
        # Initialize GPIO library
        GPIO.setmode(GPIO.BOARD)
        # Set up PWM channels
        GPIO.setup(self.pwm_pin_esc, GPIO.OUT)
        GPIO.setup(self.pwm_pin_servo, GPIO.OUT)
        self.pwm_esc = GPIO.PWM(self.pwm_pin_esc, 50)  # 50 Hz frequency
        self.pwm_servo = GPIO.PWM(self.pwm_pin_servo, 50)  # 50 Hz frequency
        self.pwm_esc.start(7.5)
        self.pwm_servo.start(5)
    
    def map_to_esc_pwm(self, linear_velocity):
        max_speed = 1.0  # Maximum linear velocity from joystick
        min_speed = -1.0  # Minimum linear velocity from joystick
        max_pwm = 8.25  # Maximum PWM duty cycle for ESC
        min_pwm = 6.75    # Minimum PWM duty cycle for ESC
        # Map linear velocity to PWM duty cycle
        esc_pwm = ((linear_velocity - min_speed) / (max_speed - min_speed)) * (max_pwm - min_pwm) + min_pwm
        return esc_pwm

    def map_to_servo_pwm(self, angular_velocity):
        max_angle = 1.0    # Maximum angular velocity from joystick
        min_angle = -1.0   # Minimum angular velocity from joystick
        max_pwm = 8.25     # Maximum PWM duty cycle for servo (adjust as needed)
        min_pwm = 3.25      # Minimum PWM duty cycle for servo (adjust as needed)
        # Map angular velocity to PWM duty cycle
        servo_pwm = ((angular_velocity - min_angle) / (max_angle - min_angle)) * (max_pwm - min_pwm) + min_pwm
        return servo_pwm

    # The feed back response from the robot
    def robot_response(self, msg):
        cmd_velocity = self.map_to_esc_pwm(msg.linear.x)
        cmd_angvel = self.map_to_servo_pwm(msg.angular.z)
        print("Robot has received the message")
        print("Linear Velocity:"+str(cmd_velocity))
        print("Angular Velocity:"+str(cmd_angvel))

def main(arg = None):
    rclpy.init()
    robot_sub = RobotSubscriber()
    print("Waiting for data to be published from the joystick")
    

    try:
        rclpy.spin(robot_sub)
    except KeyboardInterrupt:
        print("Terminating Node ...")
        robot_sub.destroy_node()

if __name__ == '__main__':
    main()