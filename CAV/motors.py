

class Motors:
    def __init__(self):
        self.steering_angle_max = 30
        self.piservo_center = 1500

        self.servoPIN = 17
        #pi = pigpio.pi()
        #pi.set_servo_pulsewidth(servoPIN, piservo_center)

        self.motorPIN = 27
        #pi = pigpio.pi()
        #pi.set_PWM_dutycycle(motorPIN, 0)

    def setControlMotors(self, steeringAcceleration, motorAcceleration):
        if seeringAngle == 0.0:
            servo = piservo_center
            motor = 0
        else:
            servo = 1000 + (float(steeringAcceleration) + math.radians(steering_angle_max)) / (2*math.radians(steering_angle_max)/1000) 
        
        if motorAcceleration != 0.0:
            motor = .05
        else:
            motor = 0

        #pi.set_PWM_dutycycle(self.motorPIN, 255*motor) 
        #pi.set_servo_pulsewidth(self.servoPIN, servo)

        print ( "Steering PID:" , servo, " Motor PID:", motor )

    def emergencyStop(self):
        # close the camera instance
        #pi.set_PWM_dutycycle(self.motorPIN, 0) 



