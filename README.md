This Python repository implements a 2D convergence/gathering algorithm for mobile robots proposed by Hideki Ando et al. [1].

# Demo 
![](https://github.com/HengZ121/Distributed-Points-Convergence/blob/main/demo.gif)

# Overview

The robots are simulated using OOP (file mobile_robot.py), each robot has the following parameters:
1. distinct id
2. coordinates on a 2D plane
3. vision range, denoted by 'v'
4. upper bond moving speed, denoted by 'sigma'

The robots' spontaneousness is achieved using concurrent programming. However, since starting threads that mimic mobile robots is processed within a time interval whose length is positively correlated to the number of robots and the processor of the device, the threads (robots) may not be able to start executing the algorithm at the same time. This can be further optimized by letting all threads sleep until a specific time spot in future after being started, but it is unnecessary since the robots are not likely to behaviour simultaneously in the real-world case. 

The implementation will first simulate the convergence algorithm and will record the tracks of all robots, and once all robots terminate (the algorithm is locally terminated), the code will then make an animation to display the motions of mobile robots. A robot terminates if after moving, it finds out that all other robots are gathered at a point. The calculation of coordinates, which are floats, is not absolutely precise due to the limitation of the programming language where decimal fractions cannot be represented exactly as binary (base 2) fractions, so all the robots may eventually converge in a small area instead of an exact point constrained by a pair of coordinates; the deviation can be specified in line 64 in file mobile_robot.py. That being said, considering that the rally point of robots in the real world is also a small range of space due to the physical volumes of robots, we may even want to increase this deviation in the real world.

To test this algorithm's validity, the locations of mobile robots are generated randomly every time; therefore, the time for the algorithm to terminate should fall in a normal distribution. The robots calculate the moving direction and distance based on other robots who are within the visible range of themselves with the equations devised in [1]. Therefore, the deadlock is possible if we choose a deficient vision range for robots, or a very small amount of robots over a relatively large 2D space because the robots may not see anyone within their vision range, so they don't move at all. In the default setting, 20 robots with a vision range of a radius of 40 on a 100 x 100 plane should complete the convergence within 1 minute, so if the algorithm doesn't terminate after a long time (they may never terminate with the current settings) please manually interrupt the script and try to run the script again or try another set of parameters.

# Assumptions

1. Every mobile robot knows how many mobile robots are there in total
2. Finite vision range of some constant V > 0 for mobile robots

# Calculation of the Moving Destination

[1] Section III. A POINT CONVERGENCE ALGORITHM

If this robot i does not see any robot other than itself, it does not move; otherwise, find the center c of the smallest enclosing circle of the set of all visible robots 
    
1. moving to c in a distance smaller than sigma
        
2. For every visible robot j, this robot's position lies in the disc D(j) whose center is the midpoint of this robot and robot j and whose radius is V/2. (Details see [1] page 4 (821))


***GOAL = dist(c, i)***  (1)

$LIMIT = \displaystyle \min_{\forall j} \left( (dist(i,j)/2)*cos\theta_j + \sqrt{(V/2)^2 - ((dist(i,j)/2)*sin\theta_j)^2}  \right)$  where $\theta_j$ is ∠c,i,j  (2)

***min( GOAL, sigma, LIMIT )***  (1 & 2)


# Required Packages:
    numpy
    argparse
    matplotlib
    smallestenclosingcircle

# How to run:

    python3 visualize_algo.py

        or user may want to customize the parameters of mobile robots

    python3 visualize_algo.py -h to check how to set parameters

References:

    1. H. Ando, Y. Oasa, I. Suzuki and M. Yamashita, "Distributed memoryless point convergence algorithm for mobile robots with limited visibility," in IEEE Transactions on Robotics and Automation, vol. 15, no. 5, pp. 818-828, Oct. 1999, doi: 10.1109/70.795787.
