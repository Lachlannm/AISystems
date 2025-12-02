Lachlann Macuisdin
V00943411

CSC 486A Assignment 3

1) The boids are initialized with random directions and are uniformly spaced within the 
initialization radius

2) The boid forces are reset at the start of the simulation loop

3) The neighbour list is calculated for each boid using simulated vision as described in lecture

4) The separation rule is calculated as described in class

5) The alignment rule is calculated as described in class

6) The cohesion rule is calculated as described in class

7) The wander rule is calculated and applied only if there are no neighbours

8) The obstacle rule is calculated as described in the assignment description

9) The world boundary rule is applied within the obstacle rule calculations

10) The total forces are summed and used to determine the new positions and velocities
of the boids

11) Set goal calculates the path from boid zero to the goal

12) boid zero correctly follows the path by sequentially moving towards corners until
it reaches the end of the path

13) The boid objects are updated based on the information from the simulation

14) Symplectic Euler is used to calculate the simulation step values

15) The simulator loop is implemented as shown in class and assignment 2

16) The original values of the test case are used.
