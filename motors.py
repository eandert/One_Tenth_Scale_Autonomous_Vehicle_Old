from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

class Motors:
    def __init__(self):
        self.steering_angle_max = 30.0
        self.servo_center = 90.0
        self.servoPIN = 0
        self.motorPIN = 1

        print ( " Initializing motors " )

        # Startup the i2c interface
        i2c = busio.I2C(SCL, SDA)

        # Create a simple PCA9685 class instance
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        # Start the PWM for the motor at 0
        self.motor = self.pca.channels[self.motorPIN]
        self.motor.duty_cycle = 0

        # Setup the steering servo and set to center
        self.steering = servo.Servo(self.pca.channels[self.servoPIN])
        self.steering.angle = self.servo_center

        print ( " Motors initialized " )

    def setControlMotors(self, steeringAcceleration, motorAcceleration):
        if seeringAngle == 0.0:
            servo = servo_center
            motor = 0
        else:
            servo = 1000 + (float(steeringAcceleration) + math.radians(steering_angle_max)) / (2*math.radians(steering_angle_max)/1000) 
        
        if motorAcceleration != 0.0:
            motor = .05
        else:
            motor = 0

        #self.motor.duty_cycle = 65535*motor
        self.motor.duty_cycle = 0
        self.steering.angle = self.servo_center

        print ( "Steering PID:" , servo, " Motor PID:", motor )

    def emergencyStop(self):
        # emergency stop set the motor to 0
        self.motor.duty_cycle = 0

    # Calling destructor 
    def __del__(self):
        self.pca.deinit()
        



