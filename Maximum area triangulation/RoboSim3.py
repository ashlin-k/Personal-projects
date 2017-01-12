#
# This module contains some basics about the Particle filter (based on the Udacity class by Sebastian Thrun)
#
# The current example uses the class 'robot', this robot lives in the 2D world
# with size of 100 m x 100 m and can see four landmarks with given coordinates
#


from math import *
import random
import matplotlib.pyplot as plt
import numpy as np

from RobotClass2 import RobotClass2

r1 = RobotClass2()
r2 = RobotClass2()
r3 = RobotClass2()

# landmarks which can be sensed by the robot (in meters)
##landmarks = [[20.0, 20.0], [20.0, 80.0], [20.0, 50.0],
##             [50.0, 20.0], [50.0, 80.0], [80.0, 80.0],
##             [80.0, 20.0], [80.0, 50.0], [85.0, 20.0],
##             [20.0, 15.0]]

# size of one dimension (in meters)
world_size = 100.0



def evaluation(r, p):
    """ Calculate the mean error of the system
    :param r: current robot object
    :param p: particle set
    :return mean error of the system
    """

    sum = 0.0

    for i in range(len(p)):

        # the second part is because of world's cyclicity
        dx = (p[i].x - r.x + (world_size/2.0)) % \
             world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % \
             world_size - (world_size/2.0)
        err = sqrt(dx**2 + dy**2)
        sum += err

    return sum / float(len(p))



def visualization(r1, r2, r3, unit, step):
#def visualization(unit, step):
    """ Visualization
    :param r1:      robot1 object
    :param r2:      robot2 object
    :param r3:      robot3 object
    :param step:    the current step
    """

##    print r1.id
##    print r2.id
##    print r3.id
    
    plt.figure("Robot in the world", figsize=(15., 15.))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])    

    # fixed landmarks of known locations
##    for lm in landmarks:
##        circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
##        plt.gca().add_patch(circle)

    # robot locations
    circle1 = plt.Circle((r1.x, r1.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle1)
    circle2 = plt.Circle((r2.x, r2.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle2)
    circle3 = plt.Circle((r3.x, r3.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle3)

    # robot orientations
    arrow1 = plt.Arrow(r1.x, r1.y, 2*cos(r1.orientation), 2*sin(r1.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow1)
    arrow2 = plt.Arrow(r2.x, r2.y, 2*cos(r2.orientation), 2*sin(r2.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow2)
    arrow3 = plt.Arrow(r3.x, r3.y, 2*cos(r3.orientation), 2*sin(r3.orientation), alpha=0.5, facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow3)

    plt.savefig("output/figure_" + str(unit) + "_" + str(step) + ".png")
    plt.close()


def get_robot(iid):
    
    global r1, r2, r3
    if (iid == 1): return r1
    elif (iid == 2): return r2
    elif (iid == 3): return r3
    else: return None


def get_dist(rob1, rob2):

    return sqrt((rob1.x - rob2.x)**2 + (rob1.y - rob2.y)**2)


def setNewFrontierBot(idFrontier, idLeft, idRight):

    global r1, r2, r3
    
    if (idFrontier == 1):
        r1.is_frontier = True
        r2.is_frontier =  False
        r3.is_frontier = False
        r1.setLeftRightRobots(idLeft, idRight)
    elif (idFrontier == 2):
        r2.is_frontier = True
        r1.is_frontier =  False
        r3.is_frontier = False
        r2.setLeftRightRobots(idLeft, idRight)
    elif (idFrontier == 3):
        r3.is_frontier = True
        r2.is_frontier =  False
        r1.is_frontier = False
        r3.setLeftRightRobots(idLeft, idRight)
    

def startNewFrontier():

    global r1, r2, r3

    turn = pi/4.0
    frontBot = 0
    
    rand = random.random()
    
    if (r1.is_frontier):
        
        # make r1->right_bot the new frontier bot
        if (rand < 0.5):
            # find plane between frontier bot and r1->left_bot
##            currPlaneAng = r1.tanFcn(get_robot(r1.left_robot).x - r1.x, get_robot(r1.left_robot).y - r1.y)
            if (r1.right_robot == 2):
                setNewFrontierBot(r2.id, r3.id, r1.id)
                frontBot = 2
            else:
                setNewFrontierBot(r3.id, r2.id, r1.id)
                frontBot = 3
            
        # make r1->left_bot the new frontier bot
        else:
            # find plane between frontier bot and r1->right_bot
##            currPlaneAng = r1.tanFcn(get_robot(r1.right_robot).x - r1.x, get_robot(r1.right_robot).y - r1.y)
            if (r1.left_robot == 2):
                setNewFrontierBot(r2.id, r1.id, r3.id)
                frontBot = 2
            else:
                setNewFrontierBot(r3.id, r1.id, r2.id)
                frontBot = 3
    
    elif (r2.is_frontier):

        # make r2->right_bot the new frontier bot
        if (rand < 0.5):
            # find plane between frontier bot and r2->left_bot
##            currPlaneAng = r2.tanFcn(get_robot(r2.left_robot).x - r2.x, r2.y - get_robot(r2.left_robot).y - r2.y)
            if (r2.right_robot == 1):
                setNewFrontierBot(r1.id, r3.id, r2.id)
                frontBot = 1
            else:
                setNewFrontierBot(r3.id, r1.id, r2.id)
                frontBot = 3
            
        # make r2->left_bot the new frontier bot
        else:
            # find plane between frontier bot and r2->right_bot
##            currPlaneAng = r2.tanFcn(get_robot(r2.right_robot).x - r2.x, get_robot(r2.right_robot).y - r2.y)            
            if (r1.left_robot == 1):
                setNewFrontierBot(r1.id, r2.id, r3.id)
                frontBot = 1
            else:
                setNewFrontierBot(r3.id, r2.id, r1.id)
                frontBot = 3

    else:
        
        # make r3->right_bot the new frontier bot
        if (rand < 0.5):
            # find plane between frontier bot and r3->left_bot
##            currPlaneAng = r3.tanFcn(get_robot(r3.left_robot).x - r3.x, get_robot(r3.left_robot).y - r3.y) 
            if (r3.right_robot == 1):
                setNewFrontierBot(r1.id, r2.id, r3.id)
                frontBot = 1
            else:
                setNewFrontierBot(r2.id, r1.id, r3.id)
                frontBot = 2
            
        # make r3->left_bot the new frontier bot
        else:
            # find plane between frontier bot and r3->right_bot
##            currPlaneAng = r3.tanFcn(get_robot(r3.right_robot).x - r3.x, get_robot(r3.right_robot).y - r3.y) 
            if (r3.left_robot == 1):
                setNewFrontierBot(r1.id, r3.id, r2.id)
                frontBot = 1
            else:
                setNewFrontierBot(r2.id, r3.id, r1.id)
                frontBot = 2

    # ensure robot always travels in the forward direction
##    if (currPlaneAng < pi/2) | (currPlaneAng > 3*pi/2):
##        normal = currPlaneAng + pi/2.0 
##    else:
##        normal = currPlaneAng - pi/2.0
##    if (normal > 2*pi): normal -= 2*pi
##    elif (normal < 0): normal += 2*pi
##    print "Curr plane ang = ", currPlaneAng * 180/pi, "\tNorm = ", normal * 180/pi
    r1 = r1.move(pi/2 - r1.orientation, 0.0)
    r2 = r2.move(pi/2 - r2.orientation, 0.0)
    r3 = r3.move(pi/2 - r3.orientation, 0.0)

    print "*********** New frontier bot is robot #", frontBot, "***********"


def reach_frontier(frontierRobot, unit):

    step = 0
    angle_err = pi/4.0
    turning_increment = pi/16.0
    maxRange = 8
    
    while (frontierRobot.is_frontier):

        print "Unit: ", unit, "\tStep: ", step

        deltaAngle = frontierRobot.sense(get_robot(frontierRobot.left_robot), get_robot(frontierRobot.right_robot))
        sign = 1
        
##        recall
##        if deltaAngle < 0, obstacle is to the left 
##        if deltaAngle > 0, obstacle is to the right 
##        if deltaAngle == 0, obstacle is directly in front
##        if deltaAngle == -1, there is not obstacle within 2 units
        
        # OBSTACLE DETECTION (in front of robot)
        if (deltaAngle != -1) & (abs(deltaAngle) < pi/3.0):
            # turn frontier robot towards the other two bots
            sig1 = frontierRobot.get_receiver(get_robot(frontierRobot.left_robot))
            sig2 = frontierRobot.get_receiver(get_robot(frontierRobot.right_robot))
            # if sign is neg, will turn right. else will turn left
            # both bots are to the right, turn right
            if (sig1 > 4) & (sig2 > 4):
                sign = -1
            # if both robots are to the left, turn left
            elif (sig1 < 4) & (sig2 < 4):
                sign = 1
            # if one robot is behind, and one is to the right, turn right
            elif ((sig1 == 4) & (sig2 > 4)) | ((sig1 > 4) & (sig2 == 4)):
                sign = -1
            # if one robot is behind, and one is to the left, turn left
            elif ((sig1 == 4) & (sig2 < 4)) | ((sig1 < 4) & (sig2 == 4)):
                sign = 1
                            
            # if we've already passed the frontier edge, reverse the above rules
            if (frontierRobot.frontier_edge_passed):
                sign *= -1

            # if one robot on either side, just turn away from the obstacle
            if ((sig1 >= 4) & (sig2 <= 4)) | ((sig2 >= 4) & (sig1 <= 4)):
                if (deltaAngle != 0): sign =  deltaAngle / abs(deltaAngle)

            frontierRobot = frontierRobot.move((pi/2.0) * sign, 0.0)
            # if no obstacle ahead in new fwd direction, move a step fwd
            if (frontierRobot.sense(get_robot(frontierRobot.left_robot), get_robot(frontierRobot.right_robot)) == -1):
                frontierRobot = frontierRobot.move(0.0, 0.5)
                print "stepping past obstacle"

            print "DETECTED OBSTACLE"


        # get angles of triangle
        leftSig = frontierRobot.get_receiver(get_robot(frontierRobot.left_robot))
        rightSig = frontierRobot.get_receiver(get_robot(frontierRobot.right_robot))
        frontierAngle = frontierRobot.get_angle_from_surrounding_bots(get_robot(frontierRobot.left_robot), get_robot(frontierRobot.right_robot))
        leftAngle = get_robot(frontierRobot.left_robot).get_angle_from_surrounding_bots(frontierRobot, get_robot(frontierRobot.right_robot))
        rightAngle = get_robot(frontierRobot.right_robot).get_angle_from_surrounding_bots(frontierRobot, get_robot(frontierRobot.left_robot))

        print "frontier passed: ", frontierRobot.frontier_edge_passed, "\t\tfrontier ang: ", frontierAngle * 180.0/pi
##        print "Front ang = ", frontierAngle * 180 / pi, "\tLeft ang = ", leftAngle * 180 / pi, "\tRight ang = ", rightAngle * 180 / pi

        # if reached frontier edge
        if (not (frontierRobot.frontier_edge_passed)) & (frontierAngle >= (175 * pi/180.0)):
            print "REACHED FRONTIER EDGE"
            frontierRobot.frontier_edge_passed = True
            turn = 0.0
            fwd = 0.0
        # if passed frontier edge and made an equilateral triangle, or out of range, or approaching a boundary, stop
        elif (frontierRobot.frontier_edge_passed) & (frontierAngle < (60 * pi/180.0)) \
             | (1 >= frontierRobot.x >= 99) | (1 >= frontierRobot.y >= 99):
            print "STOP!!!"
            turn = 0.0
            fwd = 0.5       # correction to prevent triangles from getting progressively smaller
            frontierRobot = frontierRobot.move(turn, fwd)
            frontierRobot.frontier_edge_passed = False
            break
        
        # GUIDANCE
        print "Left sig = ", leftSig, "\tRight sig = ", rightSig, "\tLeft ang = ", leftAngle * 180.0/pi, "\tRight ang = ", rightAngle * 180.0/pi
        # if left of center, turn to the right
        #if ((leftAngle - rightAngle) > angle_err) & ((leftSig > 4) & (rightSig > 4)):
        if ((leftSig > 4) & (rightSig > 4)):
            print "turn right"
            turn =  turning_increment * -1.0
            fwd = 0.0
        # if right of center, turn to the left
        #elif ((rightAngle - leftAngle) > angle_err) & ((leftSig < 4) & (rightSig < 4)):
        elif ((leftSig < 4) & (rightSig < 4)):
            print "turn left"
            turn =  turning_increment
            fwd = 0.0
        # if robots are behind frontier bot
        elif (3 <= leftSig <= 5) & (3 <= rightSig <= 5) & (not frontierRobot.frontier_edge_passed):
            print "turn 180"
            turn = turning_increment * 8
            fwd = 0.0
        # if centered, move forward
        else:
            print "move fwd"
            turn = 0.0
            fwd = 0.5
        
        frontierRobot = frontierRobot.move(turn, fwd)
        
        visualization(frontierRobot, get_robot(frontierRobot.left_robot), get_robot(frontierRobot.right_robot), unit, step)
        step += 1
        
        print "-------------------------------"

    return frontierRobot
    

def main():

    # create 3 robots for maximum-area triangulation simulation
    global r1, r2, r3
    r1.set(50.0, 0.0, 0.5*pi, 1)
    r2.set(45.0, 5.0, 0.5*pi, 2)
    r3.set(55.0, 5.0, 0.5*pi, 3)
    
    # set r1 as the first frontier robot
    setNewFrontierBot(r1.id, r2.id, r3.id)
    
    steps = 10  # # steps
    turn = 0.0
    fwd = 0.0
    
    for step in range(steps):
        
        if (r1.is_frontier): r1 = reach_frontier(r1, step)            
        elif (r2.is_frontier): r2 = reach_frontier(r2, step)
        elif (r3.is_frontier): r3 = reach_frontier(r3, step)

        startNewFrontier() 



if __name__ == "__main__":
    MINIMUM_PARTICLES=10
    PARTICLE_NUM_CHANGE_DEPENDENT = False
    PARTICLE_NUM_STEP_DEPENDENT = False
    PARTICLE_NUM_SPLIT = False
    PARTICLE_NUM_GMAPPING=False


    main()
