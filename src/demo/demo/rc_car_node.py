import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class RCCarNode(Node):
    def __init__(self):
        super().__init__('rc_car_node')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, QoSProfile(depth=10))

        # Initialize GPIO and other hardware settings
        self.servo_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 50)
        self.servo_pwm.start(10)  # Start the servo PWM signal at 1000 PWM

        self.dc_motor_pin = 17
        GPIO.setup(self.dc_motor_pin, GPIO.OUT)
        self.motor_pwm = GPIO.PWM(self.dc_motor_pin, 1000)
        self.motor_pwm.start(15)  # Start the motor PWM signal at 1500 PWM

        self.servo_position = 90.0
        self.motor_speed = 0.0

    def joy_callback(self, msg):
        # Process joystick inputs to set servo_position and motor_speed
        # Example:
        self.servo_position = 90.0 + msg.axes[2] * 45.0  # Adjust sensitivity as needed
        self.motor_speed = -msg.axes[1] * 100.0  # Adjust scaling and direction as needed

        # Calculate servo PWM value based on the specified range
        servo_pwm_value = 650 + (self.servo_position / 180) * (1650 - 650)
        self.servo_pwm.ChangeDutyCycle(servo_pwm_value / 20)  # Convert to percentage

        # Calculate motor PWM value based on the specified range
        motor_pwm_value = 1350 + (self.motor_speed / 100) * (1650 - 1350)
        self.motor_pwm.ChangeDutyCycle(motor_pwm_value / 20)  # Convert to percentage

def main(args=None):
    rclpy.init(args=args)
    rc_car = RCCarNode()
    try:
        rclpy.spin(rc_car)
    finally:
        rc_car.servo_pwm.stop()
        rc_car.motor_pwm.stop()
        GPIO.cleanup()
        rc_car.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()