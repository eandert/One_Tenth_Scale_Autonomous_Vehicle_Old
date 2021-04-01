import math
import numpy as np
from simple_pid import PID
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


''' Helper function to calculate the difference between 2 angles in radians'''
def angleDifference( angle1, angle2 ):
    diff = ( angle2 - angle1 + math.pi ) % (2*math.pi) - math.pi;
    if diff < -math.pi:
        return diff + (2*math.pi)
    else:
        return diff


''' This class contains the parameters of a RC car platform like wheelbase, etc. as
well as a rudimentary physics estimation based on the ackerman steering model. We can feed the class
a route which consists of a series of x,y coordinates and group speed target integers. The planner
class can then be classed with the current position from the localizer and it will use
pure pursuit to select a target point and decide a PPM value for the steering servo. There is also
a built in PID controller that will select the PID control for the motor based on the target velocity
of the target point velocity group. This velocity group accounts for traffic lights. '''
class Planner:
    def __init__(self):
        # Static Vehicle Params
        self.width = .3
        self.length = .57
        self.wheelbaseLengthFromRear = .1
        self.wheelbaseLength = .4
        self.wheelbaseWidth = .1
        self.steeringAngleMax = 30.0
        self.velocityMax = 1.0
        
        self.maxTurningRadius = self.wheelbaseLength / math.tan(self.steeringAngleMax)
        
        self.k = 0.3  # look forward gain
        self.Lfc = 0.5  # look-ahead distance
        
        # Updatable Params
        self.velocity = 0
        self.theta = 0
        self.seeringAngle = 0
        self.positionX_sim = 0
        self.positionY_sim = 0
        self.localizationPositionX = 0
        self.localizationPositionY = 0
        self.lastPointIndex = 0
        self.targetIndexX = 0
        self.targetIndexY = 0
        self.lookAheadIndex = 0
        self.targetVelocity = 0
        self.steeringAcceleration = 0
        self.motorAcceleration = 0
        self.pursuit_index = 0
        self.centerPointX = 0
        self.centerPointY = 0
        self.lastTargetWithinTL = 0
        self.distance_pid_control_en = False
        self.distance_pid_control_overide = False
        self.followDistance = self.length + self.Lfc
        self.targetFollowDistance = 1

        self.id = None
        self.simVehicle = False
        
    def initialVehicleAtPosition(self, x_offset, y_offset, theta_offset, xCoordinates, yCoordinates, vCoordinates, id_in, simVehicle):
        self.targetVelocityGeneral = 0
        self.id = id_in
        self.simVehicle = simVehicle
        self.seeringAngle = 0
        # This holds the actual position of the vehicle
        self.positionX_offset = x_offset
        self.positionY_offset = y_offset
        self.theta_offset = math.radians(theta_offset)
        # This is the known localization position
        if simVehicle:
            self.localizationPositionX = self.positionX_offset
            self.localizationPositionY = self.positionY_offset
            self.positionX_sim = self.localizationPositionX
            self.positionY_sim = self.localizationPositionY
            self.velocity = 0
            self.theta = self.theta_offset
        else:
            # Since this is a real world test we will start the vehicle somewhere random until it connects
            self.localizationPositionX = 5 + self.positionX_offset
            self.localizationPositionY = 5 + self.positionY_offset
            self.theta = self.theta_offset
            self.velocity = 0
        self.lastPointIndex = None
        self.xCoordinates = xCoordinates
        self.yCoordinates = yCoordinates
        self.vCoordinates = vCoordinates
        
        # Initialize the controllers\
        if self.simVehicle:
            self.v_pid = PID(3, 0.00, 0.0, setpoint=self.targetVelocity)
            self.d_pid = PID(2, 0.00, 0.0, setpoint=self.Lfc)
        else:
            self.v_pid = PID(1.5, 0.00, 0.0, setpoint=self.targetVelocity)
            self.d_pid = PID(2, 0.00, 0.0, setpoint=self.Lfc)

    def updatePosition(self, timestep):
        self.positionX_sim += self.velocity * math.cos(self.theta) * timestep
        self.positionY_sim += self.velocity * math.sin(self.theta) * timestep
        self.theta += ( self.velocity / self.wheelbaseLength ) * math.tan(self.steeringAcceleration) * timestep
        self.velocity += self.motorAcceleration * timestep

    def update_localization(self, localization = None):
        if self.simVehicle:
            # Update the localization, we could inject errors here if we want
            self.localizationPositionX = self.positionX_sim
            self.localizationPositionY = self.positionY_sim
        else:
            # Update the localization from real data
            # Calculate velocity before we update, the localization positions are from last frame
            self.velocity = self.calc_velocity(localization[0], localization[1], self.localizationPositionX, self.localizationPositionY, localization[2])
            self.localizationPositionX = ((localization[0] * math.cos(self.theta_offset)) - (localization[1] * math.sin(self.theta_offset))) + self.positionX_offset
            self.localizationPositionY = ((localization[1] * math.cos(self.theta_offset)) + (localization[0] * math.sin(self.theta_offset))) + self.positionY_offset
            self.theta = localization[2] + self.theta_offset

    def calc_velocity(self, x1, y1, x2, y2, theta):
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards according to the LIDAR so adjust the velocity accordingly
            velocity = -velocity
        return velocity

    def pure_pursuit_control(self):

        ind = self.search_target_index()

        tx = self.xCoordinates[ind]
        ty = self.yCoordinates[ind]

        alpha = math.atan2(ty - self.localizationPositionY, tx - self.localizationPositionX) - self.theta

        Lf = self.k * self.velocity + self.Lfc

        delta = math.atan2(2.0 * self.wheelbaseLength * math.sin(alpha) / Lf, 1.0)

        if delta > math.radians(self.steeringAngleMax):
            delta = math.radians(self.steeringAngleMax)
        elif delta < -math.radians(self.steeringAngleMax):
            delta = -math.radians(self.steeringAngleMax)

        # Account for the fact that in reverse we should be turning the other way
        #if self.velocity < 0:
        #    delta = -delta
        self.steeringAcceleration = delta

        self.targetIndexX = tx
        self.targetIndexY = ty
        self.distance_pid_control_overide = False
        if self.vCoordinates[ind] == 0:
            self.targetVelocity = self.targetVelocityGeneral
            self.lastTargetWithinTL = 0
        else:
            if self.coordinateGroupVelocities[self.vCoordinates[ind]] == 1:
                if self.lastTargetWithinTL == 1:
                    if self.targetVelocity == 0:
                        # We are already stopping so keep stopping
                        self.targetVelocity = 0
                        self.distance_pid_control_overide = True
                    else:
                        # We have already entered the light so keep going
                        self.targetVelocity = self.targetVelocityGeneral
                else:
                    # This is the first point we have seen of this TFL, should have enought time to stop
                    self.targetVelocity = 0
                    self.distance_pid_control_overide = True
                self.lastTargetWithinTL = 1
            elif self.coordinateGroupVelocities[self.vCoordinates[ind]] == 2:
                self.targetVelocity = self.targetVelocityGeneral
                self.lastTargetWithinTL = 1
            else:
                self.targetVelocity = 0
                self.lastTargetWithinTL = 1
                self.distance_pid_control_overide = True

        if alpha != 0:
            turningRadius = self.wheelbaseLength / math.tan(alpha)
            if turningRadius > 10:
                turningRadius = 10
            if turningRadius < -10:
                turningRadius = -10
        else:
            turningRadius = 10
        self.centerPointX = self.localizationPositionX + turningRadius * math.cos(self.theta + math.radians(90))
        self.centerPointY = self.localizationPositionY + turningRadius * math.sin(self.theta + math.radians(90))

    def update_pid(self):
        if self.distance_pid_control_en and not self.distance_pid_control_overide:
            print("TD", self.targetFollowDistance, "FD", self.followDistance)
            self.d_pid.setpoint = self.targetFollowDistance
            self.motorAcceleration = self.d_pid(self.followDistance)
        else:
            # Default to velocity PID cotnrol
            print("TV", self.targetVelocity)
            self.v_pid.setpoint = self.targetVelocity
            self.motorAcceleration = self.v_pid(self.velocity)
        # Check for pause and we have no reverse
        if self.targetVelocityGeneral == 0.0 or self.targetVelocity <= 0.0:
            self.motorAcceleration = 0.0
        #if self.simVehicle == False:
            #commands[self.id] = [-self.steeringAcceleration, self.motorAcceleration]

    def calc_distance(self, point_x, point_y):
        dx = self.localizationPositionX - point_x
        dy = self.localizationPositionY - point_y
        return math.hypot(dx, dy)

    def recieve_coordinate_group_commands(self, commands):
        self.coordinateGroupVelocities = commands

    def get_location(self):
        return [self.localizationPositionX, self.localizationPositionY, self.theta, self.targetVelocity, .2]

    def get_route(self):
        messageString = ""
        for xzip, yzip in zip(self.xCoordinates, self.yCoordinates):
            if len(messageString) == 0:
                messageString = str(xzip) + ',' + str(yzip)
            else:
                messageString = messageString + ':' + str(xzip) + ',' + str(yzip)
        return messageString

    def check_if_point_in_rectangle(self, x1, y1, x2, y2, x, y):
        if x1 > x2:
            xtemp = x2
            x2 = x1
            x1 = xtemp
        if y1 > y2:
            ytemp = y2
            y2 = y1
            y1 = ytemp
        if (x > x1) and (x < x2):
            if (y > y1) and (y < y2):
                return True
        return False

    def check_positions_of_other_vehicles_adjust_velocity(self, positions, myIndex):
        self.followDistance = 99
        self.distance_pid_control_en = False
        for idx, each in enumerate(positions):
            if myIndex != idx:
                # Create a bounding box for each vehicle that is length + 2*buffer long and width + 2*buffer wide
                x1 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]+math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.cos(each[2]-math.radians(180))))
                y1 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]+math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.sin(each[2]-math.radians(180))))
                x2 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]-math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.cos(each[2]-math.radians(180))))
                y2 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]-math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.sin(each[2]-math.radians(180))))
                x3 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]-math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.cos(each[2])))
                y3 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]-math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.sin(each[2])))
                x4 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]+math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.cos(each[2])))
                y4 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]+math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.sin(each[2])))
                point = Point(self.targetIndexX, self.targetIndexY)
                polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
                #print(polygon.contains(point))
                #if self.check_if_point_in_rectangle(x1, y1, x2, y2, self.targetIndexX, self.targetIndexY):
                if polygon.contains(point):
                    #print ( "True" )
                    # Within the safety buffer, adjust speed to be that of the front vehicle
                    #if self.targetVelocity > each[3]:
                    #    self.targetVelocity = each[3]
                    # For distance control
                    targetFollowDistance = self.length - self.wheelbaseLengthFromRear + self.Lfc
                    followDistance = math.hypot(each[0]+((2*self.length - 2*self.wheelbaseLengthFromRear + each[4])*math.cos(each[2]))-self.localizationPositionX,each[1]+((2*self.length - 2*self.wheelbaseLengthFromRear + each[4])*math.sin(each[2]))-self.localizationPositionY)
                    if self.followDistance > followDistance:
                        self.followDistance = followDistance
                        self.targetFollowDistance = targetFollowDistance
                        self.distance_pid_control_en = True

    def check_steering_angle_possible(self, x, y):
        # This equation is a quick check for if it is possible to get to the current point based on geometry
        # Essentually 2 circles that we cant go to
        dx = self.localizationPositionX + self.maxTurningRadius*math.cos(angleDifference(self.theta + math.radians(90),x))
        dy = self.localizationPositionY + self.maxTurningRadius*math.cos(angleDifference(self.theta + math.radians(90),y))
        d = math.hypot(dx, dy)

        dx2 = self.localizationPositionX + self.maxTurningRadius*math.cos(angleDifference(self.theta - math.radians(90), x))
        dy2 = self.localizationPositionY + self.maxTurningRadius*math.cos(angleDifference(self.theta - math.radians(90), y))
        d2 = math.hypot(dx2, dy2)

        a = angleDifference(math.atan2(self.localizationPositionY - y, self.localizationPositionX - x), self.theta)
        
        # Check if the target point is within either no go circle
        if d < self.maxTurningRadius or d2 < self.maxTurningRadius and a > math.radians(-90) and a < math.radians(90):
            return False
        else:
            return True

    def search_target_index(self):
        if self.lastPointIndex is None:
            # Search for the initial point, not reverse
            dx = [self.localizationPositionX - icx for icx in self.xCoordinates]
            dy = [self.localizationPositionY - icy for icy in self.yCoordinates]
            d = np.hypot(dx, dy)
            for index in range(len(dx)):
                if not self.check_steering_angle_possible(dx[index],dy[index]):
                    d[index] = 1000.0
            ind = np.argmin(d)
            self.lastPointIndex = ind
        else:
            ind = self.lastPointIndex
            distance_this_index = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])
            while True:
                checkind = ind + 1
                if checkind >= len(self.xCoordinates):
                    checkind = 0
                distance_next_index = self.calc_distance(self.xCoordinates[checkind], self.yCoordinates[checkind])
                if distance_this_index < distance_next_index:
                    break
                distance_this_index = distance_next_index
                ind = checkind
            self.lastPointIndex = ind

        L = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])

        Lf = self.k * self.velocity + self.Lfc

        # search look ahead target point index
        while Lf > L:
            L = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])
            ind += 1
            if ind >= len(self.xCoordinates):
                ind = 0

        return ind

    def return_command_package(self):
        return -self.steeringAcceleration, self.motorAcceleration

