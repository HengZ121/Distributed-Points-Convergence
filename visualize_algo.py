import time
import random
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from mobile_robot import *
from threading import Thread
from functools import partial 



def robot_is_moving(robot, ls_of_robots):
    print("Robot " + str(robot.id) + " is running")
    while(not robot.terminated):
        start = time.time()
        robot.move(ls_of_robots)
        while (time.time() - start < 1):
            pass
    print("Robot " + str(robot.id) + " has terminated")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', type=int, default=20, help='The number of robots')
    parser.add_argument('-v', type=float, default=40, help='The radius of vision range of the robots')
    parser.add_argument('-s', type=float, default=10, help='The maximum moving distance of the robots')
    parser.add_argument('-x', type=int, default=100, help='The horizontal length of the space')
    parser.add_argument('-y', type=int, default=100, help='The vertical length of the space')
    opt = parser.parse_args()

    fig, ax = plt.subplots()
    ax.set_xlim([0, opt.x])
    ax.set_ylim([0, opt.y])

    ls_of_scatter = []
    ls_of_robots = []
    ls_of_threads = []

    # initialize robots
    for id in range(opt.n):
        ls_of_robots.append(Robot(id, (x := int(random.random()*opt.x)), (y := int(random.random()*opt.y)), opt.v, opt.s))
        ls_of_scatter.append(ax.scatter(x, y))
        print("Robot " + str(id) + " has been initialized")
    print("All robots have been initialized")

    for robot in ls_of_robots:
        t = Thread(target=robot_is_moving, args=(robot, ls_of_robots))
        ls_of_threads.append(t)
    
    for t in ls_of_threads:
        t.start()
    
    for t in ls_of_threads:
        t.join()

    ani = animation.FuncAnimation(fig, partial(animate, ls_of_scatter = ls_of_scatter,ls_of_robots = ls_of_robots), repeat=True,
                                    frames=len(ls_of_robots[0].track_x) - 1, interval=500)
    plt.show()

def animate(frame, ls_of_scatter, ls_of_robots):
    track_lens = []
    for robot in ls_of_robots:
        track_lens.append( len(robot.track_x))
    min_track_len = min(track_lens)

    for idx, scatter in enumerate(ls_of_scatter):
        x = ls_of_robots[idx].track_x[frame] if (lx := len(ls_of_robots[idx].track_x)) > frame else ls_of_robots[idx].track_x[lx-1]
        y = ls_of_robots[idx].track_y[frame] if (ly := len(ls_of_robots[idx].track_y)) > frame else ls_of_robots[idx].track_y[ly-1]
        scatter.set_offsets((x, y))
    return scatter,

if __name__ == "__main__":
    main()
    
