## Selected Projects in Embedded Systems, Systems on Chip, Robotics, Logic Design and Automotive

---
### 1. Intelligent Multi-contact System using ATmega16

In many places like schools or offices, devices are often left on overnight, unnecessarily increasing energy costs. In this project I aimed to address this issue by creating intelligent multi-contacts capable of measuring the current consumption of connected devices and automatically shutting off to save energy. 

<img src="images/DiagramaControl angulogiro.png"  alt="Control Diagram"  style="width:450px; height: auto" />
<img src="images/multicontactoDb.png" alt="Blocks Diagram" style="width: 450px; height: auto;"/>


---
### 2.Steering angle system using STM32 Nucleo-H745Z1-Q

In this project, I undertook the following steps: firstly, the steering wheel angle was accurately measured to obtain precise data with an encoder AS5600, the measured data was displayed in a readable and precise manner, providing clear and understandable results while controling the steering direction from the cloud. A dashboard was created to control the system through an IoT interface, enabling connection to the cloud for remote monitoring. Then, a PID control system based on the measured values was implemented to ensure optimal system performance in DC Motor RS390. A real-time data reading system was developed to obtain accurate and reliable measurements to get information from the encoder and from a MPU6050 to analyse the motor's vibrations with FFT. 


<img src="images/Diagrama de Bloques2.png" alt="Blocks Diagram" style="width: 450px; height: auto;"/>
<img src="images/graficas.png" alt="Behavior of the system" style="width: 450px; height: auto;"/>
<img src="images/esquematico.png" alt="Schematic" style="width: 450px; height: auto;"/>
<video controls src="images/Reto_Final.mp4" title="Final Results" width="450" height="270"></video>

---
### 3. Electric Prototype Design and Manufacture

I was Co-Captain of the EcoVolt Racing Team and part of the design and development of cutting-edge electric prototypes to compete at Shell Eco-Marathon for 3 years. Furthermore I was in charge of Design and Aerodynamics, an area with a focus on optimizing the vehicle's body and the aerodynamic efficiency. In this role, I led a multidisciplinary team, contributing to the conceptualization, design, and manufacture of the body. My responsibilities included fabricating the vehicle bodie, designing and simulating car components using CAD software and working with a variety of machinery and tools. Additionally, I incorporated sustainable materials into the vehicle construction. We have been three-time champions of the Carbon Footprint Reduction Award. Throughout my tenure, I fostered a collaborative team environment, promoting open communication and teamwork to achieve project objectives. During my stay in the team I managed to participate in the construction of 3 prototypes.

<img src="images/volante.png" alt="Blocks Diagram" style="width: 450px; height: auto;"/>
<img src="images/render.jpg" alt="Blocks Diagram" style="width: 450px; height: auto;"/>
<img src="images/aerodinamica.jpg" alt="Blocks Diagram" style="width: 450px; height: auto;"/>

[View EcoVolt Racing Team Social Media](https://www.instagram.com/ecovoltccm/)
[View Teams's Video](https://www.instagram.com/reel/C2SM8KvsF1q/)


---
### 4. Fiel Oriented Current Control, Phase Transformations (Clark-Park Transformation )

During my time at EcoVolt Racing Team, I contributed to the development of codes for efficient control of electric motors as part of a firmware development project. My contribution focused on two key areas:

1. Implementation of Clarke and Park Transformations: I was responsible for implementing the Clarke and Park transformations. These transformations enable a clear and manageable representation of the motor's electrical variables, facilitating precise control of speed and torque.
2. Integration of Field-Oriented Control (FOC) Method: Additionally, I integrated the Field-Oriented Control (FOC) method into our electric motor control systems. This technique utilizes the Clarke and Park transformations to decouple the control of motor current and voltage, allowing independent control of magnetic flux and torque.


[View code on Github](https://github.com/anromero21/Shell-Firmware-2023.git)


