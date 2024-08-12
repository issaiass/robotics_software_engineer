# Assignments for Module #3 : Robot structure with URDF

### Assignment 3 - Task 1: Create a Custom Transform Tree
<p align="center">
<img src = "doc/task1_robotic_arm.gif?raw=true" center=true width="55%"/>
</p>

### Assignment 3 - Task 2: Add Joints and Visual Elements
<p align="center">
<img src = "doc/task2_robotic_arm.gif?raw=true" center=true width="55%"/>
</p>

### Assignment 3 - Task 3a: Build a Mobile Manipulator
<p align="center">
<img src = "doc/task3a_mobile_arm.gif?raw=true" center=true width="55%"/>
</p>

### Assignment 3 - Task 3b: Build a Mobile Manipulator
<p align="center">
<img src = "doc/task3b_ackerman.gif?raw=true" center=true width="55%"/>
</p>


<details open>
<summary> <b>Brief Review<b></summary>

- First part of the assignment are just links without visual of the robotic arm in a minimalistic view and without end effector joints

- The next assignment has the visuals of the robotic arm, this arm is an RP arm with end effectors each one R with mimic

- Next part has the arm over a differential drive robot

- Last part has a simple simulation of an ackerman drive robot

### <b>Project Tree</b>
<p align="center">
<img src = "doc/tree.PNG?raw=true" center=true width="35%"/>
</p>

</details>

<details close>
<summary> <b>Assignments<b></summary>


### Task 1: Create a Custom Transform Tree
- **Objective**: 
  - Design a robotic arm with 3 DOF using URDF defining the transform tree for the robotic arm without including visual tags focusin only of the correct transform over the joints.


### Task2 2: Task 2: Add Joints and Visual Elements
  - Enhance the robotic arm created earlier by adding joints like finger joints, use prismatic joints types for the finger.
  - Base Joint: The base joint should be of the continuous type.
  - All Other Joints: Set these as revolute joints.
  - Add visualization tags to your robot’s URDF to create the body, primarily using cylinder for easiest implementation.

### Task 3: Build a Mobile Manipulator

  - Place the robotic arm on top of a differential drive robot.
  - Connect the arm using the base_link of the differential drive robot.
  - Create an Ackerman Drive System:
  - Design a car-like robot structure that represents the front axle rotations for turning, simulating an Ackerman steering mechanism.
  - Learning Outcome

By completing this assignment, you will:

  - Learn to create custom robots for simulations using URDF.
  - Understand how to define and manipulate joints and transforms in URDF.
  - Gain experience in building and simulating mobile manipulators and drive systems in ROS 2.

<p align="center"> </p>
</details>

<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

- Create the workspace
```sh
    cd ~
    mkdir -p assignments_ws/src
    cd assignments_ws/src
```
- Fork (or clone) this repo in the `~/assignments_ws/src` folder by typing:
```sh 
    git clone https://github.com/Robotisim/robotics_software_engineer.git
```
- Next compile and source the repository
```sh
    cd ~/assignments_ws
    colcon build --packages-select module_3_assigment
    source install/setup.bash
```

- Run the demos for Assignment 2, Part 1
```sh
    ros2 launch module_3_assignment task:=task1
    ros2 launch module_3_assignment task:=task2
    ros2 launch module_3_assignment task:=task3a
    ros2 launch module_3_assignment task:=task3b  
```

</details>


<details open>
<summary> <b>Issues<b></summary>

- No issues found
</details>

<details open>
<summary> <b>Future Work<b></summary>

- No Future works.

</details>

<details open>
<summary> <b>Contributing<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>