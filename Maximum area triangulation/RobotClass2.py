from math import *
import random


# landmarks which can be sensed by the robot (in meters)
# [x,y]
##landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
##             [50.0, 20.0], [50.0, 80.0], [80.0, 80.0],
##             [80.0, 20.0], [80.0, 50.0], [85.0, 20.0],
##             [20.0, 15.0]]

# size of one dimension (in meters)
world_size = 100.0

# triangulation states
INITIALIZATION_STATE = 0
EXPANSION_STATE = 1
EXPANDED_STATE = 2

# tranmission signals
transmission_signals = [0, 1, 2, 3, 4, 5, 6, 7]
# let 0 be the transmitter on the front of the robot, and subsequent values increment CCW from 0.

class RobotClass2:
    """ Class for the robot model used in this demo """
    _init_already = False

    def __init__(self):
        self.x = random.random() * world_size           # robot's x coordinate
        self.y = random.random() * world_size           # robot's y coordinate
        self.orientation = random.random() * 2.0 * pi   # robot's orientation
        self.path = [] # the path the robot has actually traveled

        self.perceived_x = 0.0 # robots x coordinate based on odometry
        self.perceived_y = 0.0 # robots y coordinate based on odometry
        self.perceived_path = [] # the path the robot thinks has traveled

        self.forward_noise = 0.0   # noise of the forward movement
        self.turn_noise = 0.0      # noise of the turn
        self.sense_noise = 0.0     # noise of the sensing

        # triangulation parameters
        self.id = 0
        self.state = INITIALIZATION_STATE
        self.is_frontier = False
        self.frontier_edge_passed = False
        self.left_robot = 0        # robot ID
        self.right_robot = 0        # robot ID


    def set(self, new_x, new_y, new_orientation, iid = -1):
        """ Set robot's initial position and orientation
        :param new_x: new x coordinate
        :param new_y: new y coordinate
        :param new_orientation: new orientation
        """

        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
        if (iid != -1): self.id = iid


    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        """ Set the noise parameters, changing them is often useful in particle filters
        :param new_forward_noise: new noise value for the forward movement
        :param new_turn_noise:    new noise value for the turn
        :param new_sense_noise:  new noise value for the sensing
        """

        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)


    def tanFcn(self, x_vector, y_vector):
        angle = 0.0
        if (x_vector == 0) & (y_vector > 0): angle = pi/2.0
        elif (x_vector == 0) & (y_vector < 0): angle = 3*pi/2.0
        elif (x_vector > 0) & (y_vector >= 0): angle = atan(y_vector/x_vector)
        elif (x_vector < 0) & (y_vector >= 0): angle = pi + atan(y_vector/x_vector)
        elif (x_vector < 0) & (y_vector < 0): angle = pi + atan(y_vector/x_vector)
        elif (x_vector > 0) & (y_vector < 0): angle = 2*pi + atan(y_vector/x_vector)
        return angle
    

    def sense(self, r1, r2):
        """ Sense the environment: calculate distances to landmarks
        :return measured distances to the known landmarks
        """

        """
        New implementation: sense any obstacles.
        if an obstacle is detected within 0.2 units return the relative angle between the
        orientation of the bot and the obstacle
        if deltaAngle < 0, obstacle is to the left 
        if deltaAngle > 0, obstacle is to the right 
        if deltaAngle == 0, obstacle is directly in front
        if deltaAngle == -1, there is not obstacle within 2 units
        default values for diff and direction are -1
        """

        robots = [r1, r2]

##        distLand = 0.0
##        deltaAngleLand = -1.0

        distRobots = 0.0
        deltaAngleRobots = -1.0

        minDist = 0.0
        minProximity = 3.0
        
##        for i in range(len(landmarks)):
##            distLand = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
####            distLand += random.gauss(0.0, self.sense_noise) 
##            if (distLand < minProximity):
##                delta_y = (self.y-landmarks[i][1])
##                delta_x = (self.x-landmarks[i][0])
##                angle = self.tanFcn(delta_x, delta_y)
##                deltaAngleLand = self.orientation - angle
##                if (deltaAngleLand < (pi*-1)): deltaAngleLand += 2*pi
##                elif (deltaAngleLand > pi): deltaAngleLand -= 2*pi

        for i in range(0,2):
            distRobots = sqrt((self.x - robots[i].x) ** 2 + (self.y - robots[i].y) ** 2)
##            distRobots += random.gauss(0.0, self.sense_noise)
            if (distRobots < minProximity):
                delta_y = (robots[i].y - self.y)
                delta_x = (robots[i].x - self.x)
                angle = self.tanFcn(delta_x, delta_y)
                deltaAngleRobots = self.orientation - angle                
                if (deltaAngleRobots < (pi*-1)): deltaAngleRobots += 2*pi
                elif (deltaAngleRobots > pi): deltaAngleRobots -= 2*pi

##        minDist = min(distLand, distRobots)        
##        if (minDist == distLand): return deltaAngleLand
##        else: return deltaAngleRobots

        return deltaAngleRobots


    def get_signal(self, robot):

        # uses the relative angle between the two bots, as well as the orientation of the bot sending the signal,
        # to identify which transmitter is sending the signal
        
        x_vector = self.x - robot.x
        y_vector = self.y - robot.y
        angle = self.tanFcn(x_vector, y_vector) - robot.orientation

##        print "self_id: ", self.id, "\trobot id: ", robot.id, "\tdist = ", sqrt(x_vector**2 + y_vector**2)
        
        # ensure angle is within 0 to 2*pi
        if (angle < 0): angle += 2*pi        
        
        if (angle >= 0) & (angle < pi/4.0): signal = 0
        elif (angle >= pi/4.0) & (angle < 2*pi/4.0): signal = 1
        elif (angle >= 2*pi/4.0) & (angle < 3*pi/4.0): signal = 2
        elif (angle >= 3*pi/4.0) & (angle < 4*pi/4.0): signal = 3
        elif (angle >= 4*pi/4.0) & (angle < 5*pi/4.0): signal = 4
        elif (angle >= 5*pi/4.0) & (angle < 6*pi/4.0): signal = 5
        elif (angle >= 6*pi/4.0) & (angle < 7*pi/4.0): signal = 6
        elif (angle >= 7*pi/4.0) & (angle < 8*pi/4.0): signal = 7

##        print "ang = ", angle, "\tsig = ", signal
##        print "---------------------------------"
        
        return signal


    def get_receiver(self, robot):

        # note, 0 is the heading, then proceeds in the CCW direction with 1,2,3,...,7
        
        x_vector = robot.x - self.x
        y_vector = robot.y - self.y
        angle = self.tanFcn(x_vector, y_vector) - self.orientation
        signal = -1
##        correctionFactor = 0.017        # 1 deg

        # ensure angle is within 0 to 2*pi
        if (angle < 0): angle += 2*pi
        elif (angle >= 2*pi): angle -= 2*pi

        # correction to ensure that when a robot is close to the frontier edge, it shows perpendicular sensors
##        if (abs(angle - 0) < correctionFactor) | (abs(angle - 2*pi) < correctionFactor): angle = 0
##        elif (abs(angle - pi/2) < correctionFactor): angle = pi/2
##        elif (abs(angle - 2*pi/2) < correctionFactor): angle = pi
##        elif (abs(angle - 3*pi/2) < correctionFactor): angle = 3*pi/2
                    
        if (angle >= 0) & (angle < pi/4.0): signal = 0
        elif (angle >= pi/4.0) & (angle < 2*pi/4.0): signal = 1
        elif (angle >= 2*pi/4.0) & (angle < 3*pi/4.0): signal = 2
        elif (angle >= 3*pi/4.0) & (angle < 4*pi/4.0): signal = 3
        elif (angle >= 4*pi/4.0) & (angle < 5*pi/4.0): signal = 4
        elif (angle >= 5*pi/4.0) & (angle < 6*pi/4.0): signal = 5
        elif (angle >= 6*pi/4.0) & (angle < 7*pi/4.0): signal = 6
        elif (angle >= 7*pi/4.0) & (angle < 8*pi/4.0): signal = 7
        
        return signal


    def sense_robot_signals(self, r1, r2):
        # sense transmit messages from surrounding bots that are in range
        # assumes each robot has eight transmitters and receivers evenly distributed around
        # it's circumference. each transmitter sends out a unique message, allowing
        # the receiving robot to sense the orientation the other robots
        maxRange = 10
        distR1 = sqrt((r1.x - self.x)**2 + (r1.y - self.y)**2)
        distR2 = sqrt((r2.x - self.x)**2 + (r2.y - self.y)**2)
        signals = [-1, -1]
        if (distR1 < maxRange):
            signals[0] = self.get_signal(r1)        
        if (distR2 < maxRange):
            signals[1] = self.get_signal(r2)
        return signals


    def get_angle_from_signal(self, received_sig):
        # returns the absolute angle between 0 deg and the vector to a surrounding bot (from the perspective of the self bot)
        # based on the receiver signal and the self bot's orientation
        ang = 0        
        if (received_sig == 0): ang = 0
        elif (received_sig == 1): ang = pi/4.0
        elif (received_sig == 2): ang = 2*pi/4.0
        elif (received_sig == 3): ang = 3*pi/4.0
        elif (received_sig == 4): ang = 4*pi/4.0
        elif (received_sig == 5): ang = 5*pi/4.0
        elif (received_sig == 6): ang = 6*pi/4.0
        elif (received_sig == 7): ang = 7*pi/4.0
        # this angle just gives you the transmitted angle of the receiver.
        # to get the absolute angle to the vector, add this to the robots orientation.
        ang += self.orientation
        if (ang >= 2*pi): ang -= 2*pi
        return ang
    
    
    def get_angle_from_surrounding_bots(self, r1, r2):
        # calculates the angle between the vectors that extend from frontier bot to each of the non-frontier bots.
        # uses the signals from the transmitters of surrounding robot to get the angle between orientations,
        # then subtracts the the angles
        received_signals = [0, 0]
        received_signals[0] = self.get_receiver(r1)
        received_signals[1] = self.get_receiver(r2)
        ang1 = self.get_angle_from_signal(received_signals[0])
        ang2 = self.get_angle_from_signal(received_signals[1])
        diff = abs(ang2 - ang1)
        if (diff > pi): diff = 2*pi - diff
##        print "self_id: ", self.id, "\trob", r1.id, "rec sig 1 = ", received_signals[0], "\tang = ", ang1 * 180.0/pi, \
##              "\trob", r2.id, "rec sig 2 = ", received_signals[1], "\tang = ", ang2 * 180.0/pi, "\tdiff = ", diff * 180.0/pi
        return diff
            

    def move(self, turn, forward, return_new_state=True):
        """ Perform robot's turn and move
        :param turn:    turn command (how much more you'd like the robot to turn, NOT the absolute angular position)
                        theta = 0 means the robot is facing to the right; theta increases as you move CCW
        :param forward: forward command (how much more you'd like the robot to move fwd, NOT the absolute position)
        :return robot's state after the move (optional)
        If return_new_state is set to False, it will move the current
        particle and will not create a new one
        """

        if forward < 0:
            raise ValueError('Robot cannot move backwards')

        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn)
        #+ random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward)
        #+ random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)

        #The path the robot has traveled in reallity
        self.path.append([x,y])


        #For the perceived location, I will not add any noise
        #this is the value you would tell the robot to travel,
        #We cannot measure the noise
        dist = float(forward)
        self.perceived_x = self.perceived_x + (cos(orientation) * dist)
        self.perceived_y = self.perceived_y + (sin(orientation) * dist)

        #Also append the new point to the path
        self.perceived_path.append([self.perceived_x,self.perceived_y])


        # cyclic truncate
        x %= world_size
        y %= world_size

        if return_new_state:
            # set particle
            res = RobotClass2()
            res.set(x, y, orientation, self.id)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
            res.setLeftRightRobots(self.left_robot, self.right_robot)
            res.is_frontier = self.is_frontier
            res.frontier_edge_passed = self.frontier_edge_passed

            return res
        else:
            self.x = x
            self.y = y
            self.orientation = orientation



    @staticmethod
    def gaussian(mu, sigma, x):
        """ calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        :param mu:    distance to the landmark
        :param sigma: standard deviation
        :param x:     distance to the landmark measured by the robot
        :return gaussian value
        """

        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


    def measurement_prob(self, measurement):
        """ Calculate the measurement probability: how likely a measurement should be
        :param measurement: current measurement
        :return probability
        """

        prob = 1.0

        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        return prob


    def setLeftRightRobots(self, idLeft, idRight):
        if (idLeft >= 0) & (idLeft <= 3): self.left_robot = idLeft     
        if (idRight >= 0) & (idRight <= 3): self.right_robot = idRight


    def setState(self, state):
        if (state >= INITIALIZATION_STATE) & (state <= EXPANDED_STATE):
            self.state = state


    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
