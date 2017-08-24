## Project: Kinematics Pick & Place
---

[//]: # (Image References)

[image1]: ./misc_images/frames.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Analyzing the axes pose and the kr210.urdf.xacro file, I was able to determine the coordinate frames for each joint as seen in the figure.

![alt text][image1]


From here, I could derive the DH parameter table where most of the parameters came directly from .xacro file, but some of them were calculated:

d1 = 0.33 + 0.42 = 0.75

d4 = 0.96 + 0.54 = 1.5

d7 = dG = 0.193 + 0.11 = 0.303

therefore, the DH parameter table is the following:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | - 0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 2. Transformation matrices

I wrote the transformation matrix from _i-1_ to _i_ independently in the code and then substitute the parameter value.

```python
    T0_1 = Matrix([[                     cos(q1),                        -sin(q1),                   0,                        a0],
                               [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                               [ sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0), cos(alpha0),  cos(alpha0)*d1],
                               [                                0,                                  0,                    0,                           1]])
    T0_1 = T0_1.subs(s)
```

then I made the composition of homogeneous transformation matrices in order to obtain the complete homegeneous transformation matrix from frame 0 to the EE

```python
    # Composition of Homogeneous Transformation matrix
    T0_2 = T0_1 * T1_2  # base_link to link_2
    T0_3 = T0_2 * T2_3  # base_link to link_3
    T0_4 = T0_3 * T3_4  # base_link to link_4
    T0_5 = T0_4 * T4_5  # base_link to link_5
    T0_6 = T0_5 * T5_6  # base_link to link_6
    T0_G = T0_6 * T6_G  # base_link to gripper
```
but the orientation of the gripper frame in the DH convention is different from what we extract from the .xacro file, so we need to make a couple of rotations of our frame in order to obtain the same orientation. First 180º around the Z axe, and then -90º around the Y axe.

```python
    # Gripper orientation correction
    R_z = Matrix([[    cos(np.pi),  -sin(np.pi),              0, 0],
                  [    sin(np.pi),   cos(np.pi),              0, 0],
                  [             0,            0,              1, 0],
                  [             0,            0,              0, 1]])
    R_y = Matrix([[ cos(-np.pi/2),            0,  sin(-np.pi/2), 0],
                  [             0,            1,              0, 0],
                  [-sin(-np.pi/2),            0,  cos(-np.pi/2), 0],
                  [             0,            0,              0, 1]])
    R_corr = R_z * R_y
```

so the total homogeneous transformation matrix is:
```python
    # Total homogeneous transform
    T_total = T0_G * R_corr
```

All the forward kinematics was implemented in a function outside the _calculate_ik_ service, in order to just do it once at the beginning.
The total transformation matrix was stored in a global variable to use it later for error calculation.
Also, as a previous step, I calculated the inverse of the rotation matrix from **frame 0** to **frame 3** for use it in the IK solver.


#### 3. Inverse Kinematics

As the robotic arm has six dof and has a spherical joint for the gripper, we can decouple the IK problem in inverse position kinematics and inverse orientation kinematics.
For the IPK, first we need to calculate the wrist center, having the desired position of the EE relative to frame 0 (0<sub>**r**EE/0</sub>), and the vector from the WC to the EE (0<sub>**r**EE/WC</sub>), we can calculate the WC position relative to the frame 0 as:

0<sub>**r**WC/0</sub> = 0<sub>**r**EE/0</sub> - 0<sub>**r**EE/WC</sub>

Due to I considered the gripper frame correction in the FK, the vector 0<sub>**r**EE/WC</sub> is the rotation of the vector [d, 0, 0], therefore:

\[\frac{2}{3}\]

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


