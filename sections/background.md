# Background
## Attitude Determination and Control
The purpose of the attitude determination and control subsystem (ADCS) is to properly position and orient the spacecraft to meet the needs of the mission. The ADCS is responsible for three distinct operations throughout the mission: detumbling, initial attitude determination, and attitude maintenance. Successful operation in all three phases is vital to the overall success of the mission. This is accomplished by combining a variety of sensors and actuators in a closed-loop control system. 

### Requirements
Each phase of the mission has different requirements. In order to successfully detumble the satellite must correct for the angular spin imparted during deployment. It is expected that such an angular velocity would not have a magnitude greater than ten degrees per second about any axis. Once the satellite has reduced its angular velocity to less than ??? degrees per second, it must determine its orientation with respect to the Earth inertial frame. From this point onwards the satellite must maintain its attitude within plus or minus five degrees. As the orbit dips lower into the atmosphere the effects of drag become significant. Should the angular orientation deviate further than this limit the torques induced by atmospheric drag risk overcoming the strength of the on board actuators, causing the spacecraft to enter an uncontrollable spin. The torque exerted on the spacecraft can be described as a function of atmospheric density $\rho$, cross sectional area $A$, velocity $V_{rel}$, drag coefficient $C_D$, center of pressure $c_p$, and center of gravity $c_g$, as shown in equation !@eq:drag-torque.

$$ \tau_d = \frac{1}{2} \rho A C_D V_{rel}^2 (c_p - c_g) $${#eq:drag-torque}

Velocity is a function of the orbital velocity and the pointing angle. As the velocity increases, it is evident that the torque experienced by the spacecraft increases as well. This relationship is shown in figure !@fig:angle-torque.

![Pointing Angle vs. Torque](./images/pointing-angle-torque.jpg){#fig:angle-torque}

### Sensor & Actuator Selection
A variety of sensors are included on board the satellite to allow it to determine its angular position and rotation rates. This includes a gyroscope, magnetometer, and sun sensor. 

Magnetorquers create a torque using an external magnetic field and a magnetic dipole moment. A unique property of the magnetorquer is that is has no moving parts, unlike a reaction wheel that relies on moving parts. Due to this unique characteristic, magnetorquers are less prone to malfunction, and along with being cheap, lightweight and simple, are great for CubeSats. Magnetorquers are commonly made from a metal rod wrapped in a copper wire and connected to a power source, usually a 5V battery. Magnetorquers induce a magnetic moment by having a current run through them. Through the magnetic moment, $\nu$, multiplied by the external magnetic field felt by the magnetorquer, B, the magnetic torque, $\tau$, can be found.

$$ \tau = \nu \times B $${#eq:magnetic-torque}

As mentioned earlier, magnetorquers are a method of controlling the attitude of a spacecraft, in this case a 4U CubeSat. This will be achieved by having magnetorquers, in combination with reaction wheels, interact with Earth’s magnetic field which allows for a method of dumping excess momentum into the wheels.

The magnetorquer chosen for this task is the NCTR-M002 Magnetorquer Rod, which is manufactured by New Space. The NCTR-M002 uses a magnetic alloy rod that produces an amplification effect over an air cored magnetorquer, which in turn produces less power. The NCTR-M002 consumes around 200mW of power from a 5V power supply while producing a magnetic moment greater than 0.2 $Am^2$. The residual moment left over from the magnetorquer rod is basically negligible at less than 0.001 $Am^2$. A picture of the NCTR-M002 Magnetorquer Rod can be seen below.

![NCTR-M002 Magnetorquer Rod](./images/magnetorquer-rod.jpg){#fig:magnetorquer}

The NCTR-M002 Magnetorquer Rod’s performance characteristics are listed below:

| Parameter | Value |
|:---------:|:-----:|
| Magnetic Moment | 0.2 $Am^2$ |
| Linearity | < $\pm 5$% |
| Residual Moment | < 0.005 $Am^2$ |
| Dimensions | $70mm \times \varnothing 10mm$ |
| Mass | < 30 $g$ |
| Power | 200 mW |
| Operating Temperature | $-20^{\circ} C$ to $60^{\circ} C$ |
| Vibration | 14 g($rms$) |

Table: Magnetorquer Parameters {#tbl:magnetorquer}

Reaction wheels, also referred to as momentum wheels, are internal components that store rotational energy, proving satellites with three-axis control without the need for external sources or torque, like a propulsion system, to reorient the spacecraft. Reaction wheels are used in CubeSats because of their ability to control a satellites attitude with very high precision, while also being lightweight, compact and cheap. Reaction wheels adjust the attitude of a spacecraft by using conservation of momentum. By adjusting the momentum of a weighted wheel in the body of a spacecraft, reaction wheels cause the spacecraft body to spin in the opposite direction.

The reaction wheel chosen for this particular mission was the RWP050, manufactured by Blue Canyon Technologies. The RWP050 Reaction Wheel can create a maximum torque of 0.007 Nm and a momentum of 0.050 Nms while operating at less than 1 W at full momentum.

![RWP050 Reaction Wheels](./images/reaction-wheel.png){#fig:reaction-wheel}

| Parameter | Value |
|:---------:|:-----:|
| Momentum | 0.050 $Nms$ |
| Mass | 0.24 $kg$ |
| Dimensions | $58 \times 58 \times 25$ mm |
| Voltage | 10 - 14 V |
| Power | <1 W |
| Operating Temperature | $-20^{\circ} C$ to $60^{\circ} C$ |

Table: Reaction Wheel Parameters {#tbl:reaction-wheel}

The attitude and determination subsystem will also included a Global Positioning System (GPS) that will receive information from the on-board GPS receiver that will pull data from the magnetic field and the sun reference models.

The NGPS-01-422 is a great choice for the on-board GPS for our 4U CubeSat. The NGPS-01-422, manufactured by New Space, will be able to pull information from sensors as well as provide information on the CubeSat’s position and velocity at any point along the CubeSat’s orbit.

![NGPS-01-422](./images/gps-receiver.jpg){#fig:gps-receiver width=50%}

| Parameter | Value |
|:---------:|:-----:|
| Mass | <500 g |
| Power Consumption | 1.5 W |
| Position | < 10 m |
| Velocity | < 25 $cm/s$ |
| Operating Temperature | $-10^{\circ} C$ to $50^{\circ} C$ |
| Dimensions | $155 \times 76 \times 34$ mm |

Table: GPS Parameters {#tbl:gps-parameters}

The NGPS-01-422 New Space GPS Receiver includes an antenna, NANT-
PTCL1,which is also included in the photo above, with the specifications below:

| Parameter | Value |
|:---------:|:-----:|
| Mass | <80 g |
| Power Consumption | 80 mW |
| Frequency | 1575.42 MHz |
| Bandwidth | 20 MHz |
| Operating Temperature | $-25^{\circ} C$ to $55^{\circ} C$ |
| Dimensions | $54 \times 54 \times 14.1$ mm |
| Active Gain (RHC) | > 16dBi |
| Noise Figure | < 2 dB |

Table: GPS Receiver Parameters {#tbl:gps-receiver-parameters}

The attitude determination and control subsystem will also include a gyroscope that will gather readings on the angular velocity and angular acceleration of the 4U CubeSat. Using these readings, the gyroscope will communicate with other sensors to adjust the orientation of the CubeSat as well as maintain the 5 degree pointing requirement.

The gyroscope our team has decided to move forward with is the gyroscope chosen by previous year’s MQPs, the EVAL-ADXRS453, manufactured by Analog Devices. The EVAL-ADXRS453 consumes very little power, 0.0189W and is very lightweight, only weight 56.7 g. A picture of the EVAL-ADXRS453 along with its performance characteristics can be seen below.

![EVAL-ADXRS453](./images/gyroscope.jpg){#fig:gyroscope}

| Parameter | Value |
|:---------:|:-----:|
| Mass | <56.7 g |
| Power Consumption | 18.9 mW |
| Operating Temperature | $-40^{\circ} C$ to $105^{\circ} C$ |
| Dimensions | $33 \times 33 \times 3$ mm |

Table: Gyroscope Parameters {#tbl:gyroscope-parameters}


Our group decided on a piece of hardware that would combine both the accelerometer and the magnetometer. The LSM303AGR is a triple-axis accelerometer and magnetometer that is incredibly small, lightweight, cheap and power efficient when being compared to other accelerometer and magnetometer components. The LSM303AGR will be able to take readings of Earth’s magnetic field relative to the CubeSats body-fixed axes and use them to communicate with other sensors that will reorient the spacecraft both in the detumbling phase and post-detumbling.

![LSM303AGR Triple Axis Accelerometer and Magnetometer](./images/accelerometer.jpg){#fig:accerlerometer}

| Parameter | Value |
|:---------:|:-----:|
| Mass | 10 mg |
| Power Consumption | 5 mW |
| Operating Temperature | $-40^{\circ} C$ to $85^{\circ} C$ |
| Dimensions | $2 \times 2 \times 1$ mm |
| Linear Acceleration | $\pm 0.01$% |
| Magnetic Sensitivity | $\pm 1$% |

Table: Accelerometer and Magnetometer Parameters {#tbl:accelerometer-parameters}

Sun sensors, in general, are used for accurate sun-tracking, pointing and attitude determination. Two axis fine sun sensors, when compared to coarse sun sensors, offer a higher accuracy when measuring the incident angle of a sun ray in two orthogonal axes. Fine sun sensors also require a small amount of power in order to operate, unlike coarse sun sensors, but offer more information.

Our team chose to use five Nano-SSOC-A60 analog sun sensors, manufactured by NewSpace. The Nano-SSOC-A60 is a two-axis, low cost sun sensor that will be used for the sun-tracking and attitude determination of our CubeSat. Using the Nano-SSOC-A60, we will be able to determine the location of the sun with respect to our spacecraft, which will then be used in the QUEST quaternion and attitude of our CubeSat. The specifications of the Nano-SSOC-A60 are listed below.

![Nano-SSOC-A60 Sun Sensor](./images/sun-sensor.jpg){#fig:sun-sensor width=50%}

| Parameter | Value |
|:---------:|:-----:|
| Mass | 4 g |
| Power Consumption | 10 mW |
| Operating Temperature | $-30^{\circ} C$ to $85^{\circ} C$ |
| Dimensions | $27.4 \times 14 \times 5.9$ mm |
| Field of View | $\pm 60^{\circ}$ |
| Accuracy | < $0.5^{\circ}$ |
| Precision | < $0.1^{\circ}$ |

Table: Sun Sensor Parameters {#tbl:accelerometer-parameters}

### Control Logic

#### Sun Pointing 
Sun Pointing Vector in the Body Fixed Frame

$$ 
\begin{bmatrix} V_1 \\ \vdots \\ V_N \end{bmatrix}
= C
\left(\begin{bmatrix} C_{K_1 \hat{n}_1} \\ \vdots \\ C_{K_N \hat{n}_N} \end{bmatrix} 
s +
\begin{bmatrix} \hat{C}_{K_1 \nu_{V1}} \\ \vdots \\ \hat{C}_{K_N \nu_{VN}}
\end{bmatrix}
\right)
$${#eq:css-voltage}

$$ V = C \frac{n^{T}s}{||v||||s||} $${#eq:css-voltage}

$$ C = \frac{V_{max}F_{o}}{F_{cal}} $${#eq:css-scale-factor}

$F_{o}$ is equal to the Solar Flux of the Sun which is $3.9x10^{26}$ W

$F_{cal}$ is the Calibration Flux Constant.
$F_{cal}$ is equal to the flux due to direct sunlight $F_{o}$ in an ideal situation which is $3.9x10^26$ W

$V_{max}$ is the maximum output voltage of the CSS which is directly proportional to $F_{d}$

$$ F_{d} = F_{o}(\frac{n^T s}{||n||||s||}) $${#eq:css-angle}

n is equal to the unit normal vector of the CSS.

s is equal to the direction vector from the spacecraft to the sun.

C is the calibration constant.

$C_{k}$ is a bias parameter which, in most cases, is considered equal to 1.

$\nu$ is equal to the vector measurement of errors. For simulation purposes, $\nu$ will be disregarded.

If the calibration constant C is not known, equation !@eq:css-voltage-2

$$ 
\begin{bmatrix} V_1 \\ \vdots \\ V_N \end{bmatrix}
= \begin{bmatrix} C_{K_1 \hat{n}_1} \\ \vdots \\ C_{K_N \hat{n}_N} \end{bmatrix} 
d +
\begin{bmatrix} \nu_{V1} \\ \vdots \\ \nu_{VN} \end{bmatrix}
$${#eq:css-voltage-2}

Equation @!eq:css-least-squares shows the equation in least squares form.

$$ \widetilde{y} = Hx + \nu $${#eq:css-least-squares}

As mentioned before, if there are at least three measurements the least squares method is used. To solve for the sun vector, the following estimate is used:

$$ \hat{x} = (H^TH)^{-1}H^T\widetilde{y} $${#eq:css-least-squares-2}

However, if there are less than three observations, the minimum norm method is used. Therefore, the following estimate is used:

$$ \hat{x} = H^T(HH^T)^{-1}\widetilde{y} $${#eq:css-minimum-norm}

It is obvious that this method has a lot of assumptions including negligible calibration errors and biases, but, although they are not true in flight, numerical simulation results demonstrate that this method is capable of achieving CSS pointing despite these biases.

#### Sun Pointing in the Inertial Frame

Precision up to $0.01^{\circ}$ (36'') for years between 1950 to 2050.

$$ n = JD - 2451545.0 $$ 

Where n is the number of days since Greenwich Noon, Terrestrial Time, on the 1st of January, 2000.

JD is the Julian date for a desired time.

You then find L, the mean longitude of the Sun:

$$ L = 280.46^{\circ} + 0.9856474^{\circ}n $$

Next, find the mean anomaly of the sun, g:

$$ g = 357.528^{\circ} + 0.9856003^{\circ}n $$

Then, find $\lambda$ which is the ecliptic longitude of the Sun.

$$ \lambda = L + 1.915^{\circ}sin(g) + 0.020^{\circ}sin(2g) $$

Note: L and g both need to be between $0^{\circ}$ and $360^{\circ}$. In order to do this, simply subtract or add $360^{\circ}$ until this happens.

Next, you find the distance from the Sun to the Earth in AU:

$$ R=1.00014 - 0.01671cos(g) - 0.00014cos(2g) $$

Finally, find the rectangular equatorial coordinates:

$$ X = Rcos(\epsilon)cos(\lambda) $$
$$ Y = Rcos(\epsilon)sin(\lambda) $$
$$ Z = Rsin(\epsilon) $$

Where $\epsilon$ is the obliquity of the ecliptic and can be found using the following equation:

$$ \epsilon = 23.439^{\circ} - 0.0000004^{\circ}n $$

#### Spacecraft Detumbling

Because of the universal nature of CubeSat deployment systems, no system can truly guarantee to deploy a cubesat with zero angular velocity or even with a known angular velocity. As such, any CubeSat deployed will immediately begin tumbling, even before the satellite can activate any sort of attitude control system. Because the angular velocity during this tumbling state is unknown, generally makes it difficult to complete mission requirements, and can be much higher than angular velocities that occur throughout the rest of the mission, it is important to detumble the CubeSat after deployment. 

#### B-dot Control
One of the detumbling methods common in CubeSat missions is B-dot control. The B-dot controller detumbles a spacecraft by commanding a magnetic dipole moment with magnetorquers while measuring the time derivative of the local geomagnetic field in the spacecraft body frame. This produces a resultant torque counter to the spin of the spacecraft, which will reduce the magnitude of angular velocity until a point at which the mission can begin. The magnetic dipole moment commanded by this control law can be expressed in vector notation as $\mu$, where

$$missing equation$${#eq:mag-dipole}

where $k$ is the controller gain, $\vec{B}$ is the Earth’s magnetic field, and 

$$vec{b} = \frac{\vec{B}}{\|\vec{B}\|}$${#eq:unitized-B}

If the spacecraft's angular velocity is known, such as through rate gyroscope measurements, the commanded torque can be written as

$$vec{m} = \frac{k}{\|\vec{B}\|}\vec{\omega}\times\vec{b}$${#eq:control-torque}

where $\vec{\omega}$ is the angular velocity of the spacecraft. This can be done by making the assumption that the change in the Earth’s magnetic field due to change in orbital position occurs much more slowly than the change in the magnetic field in the spacecraft body frame due to the tumbling motion. The B-dot control law can then be written as

$$\vec{L} = \vec{m}\times\vec{B}$${#eq:mxB}

In order to make it easier to prove the stability of the control law, !@eq:mxB can be rewritten as

$$\vec{L} = -k(I_3-\vec{b}\vec{b}^T)\vec{\omega}$${#eq:rewrtbdot}

To prove the stability of a control law, it has to be shown to reduce a positive Lyapunov function asymptotically to zero by making the derivative of the function less than or equal to zero for all cases. For detumbling, we can use the Lyapunov function

$$V = \frac{1}{2}\vec{\omega}^TJ\vec{\omega}$${#eq:Lyapunov}

where $J$ is the spacecraft moment of inertia matrix. This function is analogous to rotational kinetic energy, making it useful for this case, as the goal of the detumbling control law is to reduce the angular velocity. With this Lyapunov function and applying the B-dot control law written in !@eq:Lyapunov, the derivative of the Lyapunov function can be written as

$$\dot{V}=-k\vec{\omega}^T(I_3-\vec{b}\vec{b}^T)\vec{\omega}$${#eq:Vdot}

Because the eigenvalues of $(I_3-\vec{b}\vec{b}^T)$ are always 0, 1, and 1, $(I_3-\vec{b}\vec{b}^T)$ is positive semidefinite. Therefore, $\dot{V}$ is always less than or equal to zero. The only case where $\dot{V}$ is equal to zero and global asymptotic stability cannot be obtained is when $\vec{\omega}$ is parallel to $\vec{b}$, in which case the spacecraft is already not tumbling. In this case, it would be in spin, and the control algorithm can move to initial attitude determination.

In order to use !@eq:control-torque in a bang-bang implementation, it can be rewritten as

$$ \vec{L} = \sum_{i=1}^N-m_i^{max}sign(\vec{u_i} \cdot \dot{\vec{B}}) $${#eq:bbBdot}

where $N$ is the number of magnetorquers on the spacecraft, $m_i^{max}$ is the maximum moment the $i-th$ magnetorquer can delivery, and $u_i$ is the direction of the magnetic moment for the $i-th$ magnetorquer. This implementation is less computationally expensive than the other implementation, but less efficient in terms of power consumption. Ultimately, this is the implementation that we have chosen because of the computational savings.

## Orbital Determination and Control
### eLEO Orbit Decay
### Component Selection


## Command and Data Handling
### Data Handling Needs
### Component Selection

## ADCS Testbed
