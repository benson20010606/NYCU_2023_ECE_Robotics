# NYCU_2023_ECE_Robotics
##  Robotics Project: Part 1 
For a PUMA 560 robot manipulator with the following kinematic table,

| Joint | d (m) | a (m) | α   | θ   |
|-------|-------|-------|-----|-----|
| 1     | 0     | 0     | -90°| 0°  |
| 2     | 0     | 0.432 | 0°  | 0°  |
| 3     | 0.149 | -0.02 | 90° | 0°  |
| 4     | 0.433 | 0     | -90°| 0°  |
| 5     | 0     | 0     | 90° | 0°  |
| 6     | 0     | 0     | 0°  | 0°  |
  
$-160^{\circ} \leq \theta_1  \leq 160^{\circ} , -125^{\circ} \leq \theta_2  \leq 125^{\circ}$  
$-135^{\circ} \leq \theta_3  \leq 135^{\circ} , -140^{\circ} \leq \theta_4  \leq 140^{\circ}$  
$-100^{\circ} \leq \theta_5  \leq 100^{\circ} , -260^{\circ} \leq \theta_6  \leq 260^{\circ}$  

please write a program for the following two transformations:
- input: Cartesian point (n, o, a, p), output: the corresponding joint variables.
- input: joint variables, output: Cartesian point (n, o, a, p) and (x, y, z, $\phi,\theta,\psi$)


##  Robotics Project: Part 2 
For the PUMA 560 robot manipulator, please write a program to plan a path for both (a) Joint move and (b) Cartesian move. The starting, via, and end points for the path are  

$$
A = \begin{bmatrix}
0.64 & 0.77 & 0 & 5 \\
0.77 & -0.64 & 0 & -55 \\
0 & 0 & -1 & -60 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
B = \begin{bmatrix}
0.87 & -0.1 & 0.48 & 50 \\
0.77 & -0.64 & 0 & -55 \\
0 & 0 & -1 & -60 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
C = \begin{bmatrix}
0.64 & 0.77 & 0 & 5 \\
0.77 & -0.64 & 0 & -55 \\
0 & 0 & -1 & -60 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$


respectively. The length unit above is cm. For each move, you need to do the planning for both (a) straight line portion and (b) transition portion. The transition portion has to meet the requirement of position, velocity, and acceleration continuities with the straight line portion. Please use the method introduced in the class. For joint move, choose one configuration and derive its corresponding Cartesian path. For Cartesian move, find one feasible corresponding joint path. Please demonstrate the results by geometrical curves instead of listings of numbers. The time to move from point A to B is 0.5 sec., and from point B to C 0.5 sec. The tacc for the transition portion is 0.2 sec. and the sampling time 0.002 sec.
