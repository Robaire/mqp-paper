# Background
## Mission Profile
There are several distinct mission phases that influence the design of the control systems. 

## Attitude Determination and Control
The purpose of the attitude determination and control subsystem (ADCS) is to properly position and orient the spacecraft to meet the needs of the mission. The ADCS is responsible for three distinct operations throughout the mission: detumbling, initial attitude determination, and attitude maintenance. Successful operation in all three phases is vital to the overall success of the mission. This is accomplished by combining a variety of sensors and actuators in a closed-loop control system. 

### Requirements
Each phase of the mission has different requirements. In order to successfully detumble the satellite must correct for the angular spin imparted during deployment. It is expected that such an angular velocity would not have a magnitude greater than ten degrees per second about any axis. Once the satellite has reduced its angular velocity to less than ??? degrees per second, it must determine its orientation with respect to the Earth inertial frame. From this point onwards the satellite must maintain its attitude within plus or minus five degrees. As the orbit dips lower into the atmosphere the effects of drag become significant. Should the angular orientation deviate further than this limit the torques induced by atmospheric drag risk overcoming the strength of the on board actuators, causing the spacecraft to enter an uncontrollable spin. The torque exerted on the spacecraft can be described as a function of atmospheric density $\rho$, cross sectional area $A$, velocity $V_{rel}$, drag coefficient $C_D$, center of pressure $c_p$, and center of gravity $c_g$, as shown in equation !@eq:drag-torque.

$$ \tau_d = \frac{1}{2} \rho A C_D V_{rel}^2 (c_p - c_g) $${#eq:drag-torque}

### Sensors


### Actuators

### Control Logic

## Orbital Determination and Control
### eLEO Orbit Decay
### Component Selection

## Command and Data Handling
### Data Handling Needs
### Component Selection