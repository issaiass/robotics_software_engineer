# Assignments for Module #1 : C++ from Robotics Prespective

<details open>
<summary> <b>Brief Review<b></summary>

- Here we create and test different tasks, our goal is to understand C++ and OOP by giving you examples of different approaches

Below you will find this project tree

### <b>Project Tree</b>
<p align="center">
<img src = "doc/tree.PNG?raw=true" center=true width="85%"/>
</p>

</details>

<details close>
<summary> <b>Assignments<b></summary>

### Assignment 1: Simulating Sensors with Hardcoded Values
- **Objective**: 
  - Learn how to simulate sensor data using hardcoded values in C++ and apply C++ syntax to simulate basic robotics concepts.
- **Tasks**:  
  - This C++ program represents a robot equipped with temperature and distance sensors.
  - Values are hardcoded sensor readings (e.g., temperature, distance).
  - Later, this values are printed to the console, i.e. "Temperature: 20°C", "Distance: 100cm".

### Assignment 2: Introduction to Object-Oriented Programming (OOP)
- **Objective**: Introduce basic OOP concepts using C++ within a robotics context; instantiate the robot object to simulate actions invoking the methods.  Utilize namespaces for defining different robots.  Output each action to the console to show the robot's behavior and grasp OOP principles and their application in robotics software development.
- **Tasks**:
  - Define a Robot class with attributes
    - name
    - speed
    - Physical ( weight , size , number of sensors )
  - Methods for moving
    - moveForward
    - moveBackward
    - stopping.

### Assignment 3: Creating Custom Libraries for Robotics Components
- **Objective**: Learn how to create and use custom C++ libraries for reusable robotics components, and understand how to organize code into reusable libraries compiling with CMake.
- **Tasks**:
  - Design a simple sensor library that include a variety of sensors
    - TemperatureSensor
    - DistanceSensor
  - Use these classes in a main program to simulate getting readings from sensors.
  - Ensure proper documentation
  - Create Single Class Template to be utilized for multiple Sensors of different types
    - Double data
    - String Data
    - Character data

<p align="center"> </p>
</details>

<details open>
<summary> <b>Using The Package <b></summary>

- Follow the next steps to replicate the outcome...

- Create the workspace
~~~
    cd ~
    mkdir -p assignments_ws/src
    cd assignments_ws/src
~~~
- Fork (or clone) this repo in the `~/assignments_ws/src` folder by typing:
~~~ 
    git clone https://github.com/Robotisim/robotics_software_engineer.git
~~~
- Next compile and source the repository
~~~
    cd ~/assignments_ws
    colcon build --packages-select module_1_assigment
    source install/setup.bash
~~~
- Run the demos
~~~
    ros2 run module_1_assignment assignment1
    ros2 run module_1_assignment assignment2
    ros2 run module_1_assignment assignment3
~~~


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