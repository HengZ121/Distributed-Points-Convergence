import time
import random
import argparse
import numpy as np
import matplotlib.pyplot as plt

from mobile_robot import *
from threading import Thread

def robot_is_moving(robot, ls_of_robots):
    #while(not robot.terminated):
    for x in range(10):
        start = time.time()
        robot.move(ls_of_robots)
        while (time.time() - start < 1):
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', type=int, default=10, help='The number of robots')
    parser.add_argument('-v', type=float, default=100, help='The radius of vision range of the robots')
    parser.add_argument('-s', type=float, default=10, help='The maximum moving distance of the robots')
    parser.add_argument('-x', type=int, default=100, help='The horizontal length of the space')
    parser.add_argument('-y', type=int, default=100, help='The vertical length of the space')
    opt = parser.parse_args()

    # initialize robots
    ls_of_robots = []
    for id in range(opt.n):
        ls_of_robots.append(Robot(id, int(random.random()*opt.x), int(random.random()*opt.y), opt.v, opt.s))

    plt.ion()

    for robot in ls_of_robots:
        t = Thread(target=robot_is_moving, args=(robot, ls_of_robots))
        t.start()

if __name__ == "__main__":
    main()
    
