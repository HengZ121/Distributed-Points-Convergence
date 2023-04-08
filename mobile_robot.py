import math
import smallestenclosingcircle
import numpy as np

class Robot:

    '''
    Initialization: Cast a robot instance at position (x_coordinate, y_coordinate) with specified id

    id : integer
    x_coordinate, y_coordinate: floats
    '''
    def __init__(self, id, x_coordinate, y_coordinate, v, s):
        self.id = id
        self.x = x_coordinate
        self.y = y_coordinate
        self.v = v
        self.sigma = s
        # record the track of this robot to visualize it on graph
        self.track_x = [self.x]
        self.track_y = [self.y]
        self.terminated = False

    '''
    Find set of all visible robots from all robots S
    Visible robots should be within the Euclidean distance of V (itself is also visible)

    robots : list of all robots
    V is vision distance of a robot: integer
    return S
    '''
    def getVisibleRobots(self, robots):
        s = []
        for robot in robots:
            if robot.id != self.id:
                # calculate the Euclidean distance and check if it is less than V
                if math.dist([self.x, self.y], [robot.x, robot.y]) <= self.v:
                    s.append(robot)
            else:
                s.append(robot)
        return s

    '''
    Calculate the new coordinates of this Robot i, and move to it.
    Paper Section III. A POINT CONVERGENCE ALGORITHM
        * If this robot does not see any robot other than itself, it does not move
        * Otherwise, find the center of the smallest enclosing circle of the set of all visible robots c(t)
            1) moving to c(t) in a distance smaller than sigma
            2) For every visible robot j, this robot's position lies in the disc D(j) whose center 
        is the midpoint of this robot and robot j and whose radius is V/2. (Details see [1] page 4 (821))

    robots : list of all robots
    robot_s : set of all visible robots S
    Sigma is the small constant that a robot's moving distance cannot exceed at any time t.
    V is vision distance of a robot: integer
    '''
    def move(self, robots):
        robots_s = self.getVisibleRobots(robots)
        
        # Check if all robots are gathered together, termination can be applied if so
        if len(robots_s) == len(robots):
            allGathered = True
            for robot in robots_s:
                if math.dist([self.x, self.y], [robot.x, robot.y]) > 1:
                    # A robot is found at different position of current robot
                    allGathered = False
                    break
            if allGathered:
                self.terminated = True
                return
            

        # If this robot does not see any robot other than itself
        if len(robots_s) == 1:
            # it does not move
            return
        # Calculate the center of the smallest enclosing circle of the set S c(t) using openCV
        c_x,c_y, _ = smallestenclosingcircle.make_circle([(robot.x, robot.y) for robot in robots_s if robot.id != self.id])

        max_disc_Dj = []
        for robot in robots_s:
            if robot.id != self.id:
                cos, sin = self.trigonometrics([c_x,c_y], [robot.x, robot.y])
                # Calculate The maximum distance that this robot can move toward c(t) without leaving D(j):
                # (dist(i, j)/2) * cos(theta(c,i,j)) + sqrt( (V/2)^2 - (dist(i, j)/2) * sin(theta(c,i,j))^2 )
                max_disc_Dj.append( (dist:=math.dist([self.x, self.y], [robot.x, robot.y]))/2 * cos + 
                    math.sqrt( (self.v/2)**2  - (dist/2) * sin**2 ) )
        # the maximum distance this robot can move without the loss of vision of all visible robots
        limit = min(max_disc_Dj)

        # the distance from this robot to c
        goal = math.dist([self.x, self.y], [c_x,c_y])

        move_distance = min([limit, goal, self.sigma])

        # new position should be: (((1−t)x+t*c_x),((1−t)y+t*c_y)) where t is the ratio of move_distance / dist(i, c)
        if goal == 0:
            # it does not move
            return
        new_x = ((1 - (ratio:=move_distance/goal)) * self.x +ratio*c_x)
        new_y = ((1 - ratio) * self.y + ratio * c_y)
        self.x = new_x
        self.y = new_y
        self.track_x = np.append(self.track_x, new_x)
        self.track_y = np.append(self.track_y, new_y)

    '''
    helper function, get trigonometrics (sine & cosine) on theta of ∠c,i,j
    '''
    def trigonometrics(self, c, j):
        a = np.array(c)
        b = np.array([self.x,self.y])
        c = np.array(j)
        ba = a - b
        bc = c - b
        if (ba_value := np.linalg.norm(ba)) == 0 or (bc_value := np.linalg.norm(bc)) == 0: # Special Case: same x/y coordinates, theta = 0
            return 1, 0
        cosine = np.dot(ba, bc) / (ba_value * bc_value)
        # Special Case, Python's limited computing capacity on float, cos(theta) slightly greater than 1 (e.g., 1.00000001), then let it be 1
        if cosine > 1: 
            cosine = 1
        sine = math.sqrt(1 - cosine**2)
        return cosine, sine


'''
Reference:
    1. H. Ando, Y. Oasa, I. Suzuki and M. Yamashita, "Distributed memoryless point convergence algorithm 
    for mobile robots with limited visibility," in IEEE Transactions on Robotics and Automation, vol. 15, 
    no. 5, pp. 818-828, Oct. 1999, doi: 10.1109/70.795787.
'''