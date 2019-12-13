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

### ADC Algorithms and Methods
The following subsections outline the ADC methods analyzed and chosen for our CubeSat. For each of the three attitude control phases, a different method of ADC was needed. With mission parameters specified, a generic block diagram was created to show a high level overview of the ADCS. Structure is shown in figure n below. From the graphic, we were able to gage how many of each sensor and actuator we would need, and what parameters we needed to model. 

![ADCS Flow Chart](./images/adc-flowchart.png){#fig:adc-flowchart width=50%}

#### Initial Attitude Determination
In order to begin to understand what attitude determination and control algorithms were needed for this project, the team first needed to understand the two main categories of ADC, and how previous project teams applied them to their cubesat. The first subcategory is recursive ADC. Recursive methods work by comparing the current attitude to the most recent attitude. A recursive function is a function that calls itself, so this form of ADC provides an updated state estimate using its previous state estimate. For example, if the current attitude quaternion is denoted by qn then the recursive method would estimate that value by using information from its previous estimate, qn-1. The second type of ADC method is deterministic ADC. Deterministic methods use current sensor readings as well as reference readings to calculate an attitude estimate based on the difference between the two values. 

Previous MQP teams used the deterministic TRIAD method for their attitude determination and control. This was used in unison with noise filtering methods in an attempt to minimize TRIAD error while also producing an optimal quaternion for initial attitude determination [2017 citation here]. The TRIAD method was used as a baseline to get a directional cosine matrix. With that information, newer methods were then used to produce an accurate quaternion from the TRIAD output (Farhat 2013). Due to the widespread application of this approach, as long with its efficiency, our team decided to maintain the same type of analysis for our project. 

#### TRIAD Algorithm
Harold Black’s 1964 TRIAD algorithm was the first published ADC method. It combines information from the unit vector to the Sun along with Earth’s magnetic field vector. This method requires two sets of each vector. Two observation vectors are obtained by the magnetometer and the sun sensors, while two reference vectors are obtained by using a geomagnetic model of the atmosphere as well as a sun vector model. Values for the reference vectors are determined by the position of the satellite at a given time in the geodetic (Earth-fixed) reference frame (Black). These two sets of unit vectors over determine the attitude of the cubesat, which is useful given that many of the on board components are commercial over the shelf parts with high enough error to be insufficient as the only reference sensor. 
The TRIAD method allows for coordinates in the body fixed reference frame to be rotated into the Earth-fixed inertial reference frame. This, of course, is very useful for understanding where in the orbit the Cubesat is. However, because vectors must be normalized for this method, it is possible that there could be a decent amount of sensor noise. Later research, specifically by Grace Wahba, poses a potential way of minimizing input sensor error, thus making the TRIAD method more accurate. Solutions to Wahba’s problem include Davenport’s q-method and QUEST  (Markley). These methods provide and optimum quaternion for expressing the spacecrafts attitude. 
The TRIAD method begins by defining two linearly independent body fixed vectors, b1and b2as well as their corresponding reference frame vectors r1and r2determined from Earth’s magnetic field model and the Sun’s direction to Earth. The attitude matrix, A, serves as the matrix that rotates the body fixed vectors into the earth fixed frame. Two rotation matrices can be defined in a way such that A1=A2,where:

$$ Ar_i = b_i \text{for} i = 1,2 $$

However due to sensor error, we can not confidently say that A1=A2. The TRIAD method also assumes that one of the unit vectors, b1, is much more accurately determined than its counterpart. Therefore equation 2.2 is exactly satisfied for i=1 but is only an approximate for the case where i=2. The TRIAD algorithm is based off of two sets of orthonormal right-handed triads of vectors. One for the  reference frame as well as one for the body frame (Markley). These triads are denoted as:

$$ M_{obs} = {v_1 \ v_2 \ v_3} $$

$$ v_1 = r_1 $$
$$ v_2 = r_x = \frac{r_1 \times r_2}{|r_1 \times r_2|} $$
$$ v_3 = r_1 \times r_x $$
$$ M_{ref} = {w_1 \ w_2 \ w_3} $$

$$ w_1 = b_1 $$
$$ w_2 = b_x = \frac{b_1 \times b_2}{|b_1 \times b_2|} $$
$$ w_3 = b_1 \times b_x $$

Substituting the triads into equation 2.2 the direct cosine matrix can be obtained. Equation 2.2 can then be rewritten as:

$$ AM_{obs} = M_{ref} $$

Substituting in the relations for each individual vector in each triad and simplifying, the equation for the direct cosine matrix becomes: 

$$ A_{TRIAD} = b_1 r_1^T + (b_1 \times b_x)(r_1 \times r_x)^T + b_x r_x^T $$

This allows for coordinates to be rotated to the body fixed frame and vice versa, using a little linear algebra. It is important to note however, that for cases where either the reference vectors or observed vectors are parallel or antiparallel equation 2.12 is undefined, meaning the attitude matrix cannot be found. It is also easy to see how the assumption made earlier (that b1is entirely accurate) can lead to errors in the cosine matrix. The formula for ATRIAD heavily relies on vectors with subscript 1 which in practice have some degree of error associated with them (Markley). This means solutions to Wahba’s problem have to be used to minimize this error. 

#### Wahba's Problem

Mathematician Grace Wahba attempted to describe issues associated with using direction cosine matrices in attitude estimation and provide a way to build upon the TRIAD method. Improvements include, adding a way to weigh sensor measurements and allowing for more than 2 sets of measurements to be used. To understand how to improve upon the TRIAD method, one must understand the problem Wahba proposed. Wahba’s problem serves to find an orthogonal matrix with a positive 1 determinant that minimizes the loss function:

$$ L(A) = \frac{1}{2} \sum\limits_{i=1}^N a_i ||b_i - A r_i||^2 $$

This essential finds the rotation matrix A that brings the first set of unit vectors (b1,b2,...,bn)into the best least squares coincidence with the second set of vectors (r1,r2,...,rn).
Where bi is a set of N unit vectors in the spacecraft body frame, ri is the corresponding set of vectors in the reference frame, and ai represents the non-negative weights of each sensor. These weights must be applied according to the accuracy of the sensors. This is necessary to relate Wahba’s problem to Maximum Likelihood Estimation, a technique that uses sensor accuracy to help to more accurately model a system. Using the orthogonality of A, the unit norm of the unit vectors, and the cyclic invariance of the trace we can rewrite L(A) as:

$$ L(A) = \lambda_0 - tr(AB^T) $$

$$ \lambda_0 = \sum\limits_{i=1}^N a_i \text{and} B = \sum\limits_{i=1}^N a_i b_i r_i^T $$

Solutions to Wahba’s problem include Davenport's q method, Singular Value Decomposition, Quaternion Estimator (QUEST) and Estimators of the Optimal Quaternion (ESOQ), and start with the rewritten error estimation above. Through various real world applications, QUEST and ESOQ are deemed significantly faster than their robust counterparts.

#### Quaternions

For the orientation of the spacecraft to be determined, controlled, or otherwise used, there must be a way of expressing it. A common way of doing this in other applications is with a sequence of Euler angles. This information can be determined and operated on as a three element vector and is generally enough to uniquely determine the orientation. For some applications, especially spacecraft, three Euler angles cannot determine the orientation because of singularities in the determination when an angle is near 90 degrees. For this reason, spacecraft often use quaternions, which are four element vectors with definitions for additional operations. Quaternions are composed of a three element “vector part” and a single “scalar part,” and because of the additional element, there are no singularities when expressing orientation with quaternions (Crassidis). Quaternions are also computationally easy to work with because most calculations can be done with quaternion operations instead of trigonometry. Just like most other attitude representations, quaternion attitudes can be converted to other systems, such as Euler angle representations, which are more intuitive to visualize. Because of these advantages, quaternion attitude representations are great for a cubesat mission, as the computations are less taxing for the OBC and the lack of singularities allows for diverse mission profiles.

#### Davenports Q Method

Mathematician Harold Davenport proposed a solution to Wahba's problem that maximizes the following gain function, which is a quantity from eq. (wahba's problem):

$$ g = tr(AB^T) $$

He specified a rotation matrix A that can be expressed in terms of euler parameters:

$$ A = (q_0^2 - \epsilon^T \epsilon) * [I_{3 \times 3}] + 2 \epsilon \epsilon^T - 2 q_0 |\epsilon| $$

Where $\epsilon$ =(q1,q2,q3)and q0 is the scalar term of the quaternion

The Gain function g() can then be written in terms of the 4x4 K matrix:

$$ g(\bar{q}) = q^T [K] q $$

$$ K = \begin{bmatrix} \sigma& Z^T \\ Z& S-\sigma[I_{3 \times 3}] \end{bmatrix} $$

$$ B = \sum\limits_{i=1}^N a_i b_i r_i^T $$
$$ |S| = [B] + [B]^T $$
$$ \sigma = tr([B]) $$
$$ [Z] = [B_{23} - B_{32} \ B_{31} - B_{13} \ B_{12} - B_{21}]^T $$

In order to maximize the gain function, we must abide by the unit length constraint, which means we cannot set the values to infinity. Due to this, a lagrange multiplier is needed to yield a new gain factor g'.

$$ g(q) = q^T[K]q - \lambda(q^T q-1) $$

Differentiating and setting the equation equal to zero, will find the extrema of the function .

$$ 0 = \frac{d}{dq}(g(q)) = 2[K]q - 2 \lambda q $$

which can, alternatively,  be expressed as:

$$ [K]q = \lambda_{max} q $$

Therefore, the desired euler parameter vector is the eigenvector of the K matrix. In order to maximize the gain, we have to choose the largest eigenvalue. 
Through substitution of equation 3.9 into equation 3.7, and applying some linear algebra,  it is easy to show that:

$$ g(p) = \lambda $$

#### QUEST
The QUEST method (QUaternion ESTimator) builds off of davenport's Q method, and allows for more frequent attitude computations. Currently, it is the most widely used algorithm for solving Wahba’s problem, and was first used in 1979 by the MAGSAT spacecraft (crassidis). The algorithm begins by rewriting equation (whatever 3.9 is in the new doc lmao), into two separate equations:

$$ [(\lambda_{max} + trB)I_3 -S]\hat{q}_{1:3} = \hat{q}_4 z $$ 

$$ (\lambda_{max} - trB) \hat{q}_4 - \hat{q}_{1:3}z = 0 $$

From there, we can rewrite q using the classical adjoint representation, knowing that the adjoint divided by the determinant gives the inverse of a matrix.:

$$ q_{1:3} = q_4((\lambda_{max} + trB)I_3 - S)^{-1}z = \frac{q_4(adj((\lambda_{max} + trB)I_3 - S)z)}{\det((\lambda_{max} + trB)I_3 - S)} $$

From there, we can use the Cayley-Hamilton theorem, which states that every square matrix over a real or complex field satisfies its own characteristic equation [cite], the previous equation can be rewritten in terms of the classical adjoint. This theorem holds for general quaternionic matrices, which is the case for this part of the QUEST algorithm and gives the optimal quaternion estimate as follows:

$$ q = \alpha \begin{bmatrix} adj(\rho I_3 - S)z \\ \det(\rho I_3 - S) \end{bmatrix} \rho = \lambda_{max} + trB $$

The term  is determined by normalizing the resultant q. This equation assumes we know the value of max, which is the solution to the characteristic equation of the K matrix. Finding the maximum eigenvalue of the characteristic equation is complicated for states using more than two observations. However, in the case of our CubeSat, we are only using two observations, which means the characteristic equation for K has a simple closed form solution. Several simplifications result from only having two observations, which ends up yielding the following equation for $\lambda_{max}$:

$$ \lambda_{max} = {a_1^2 + a_2^2 + 2a_1 a_2[(b_1 \odot b_2)(r_1 \odot r_2) + ||b_1 \otimes b_2|| ||r_1 \otimes r_2||]}^\frac{1}{2} $$

## Command and Data Handling
The Command and Data Handling (C&DH) subsystem is composed of many different components and systems whose purpose is to manage the entire spacecraft throughout its mission. This includes during the startup, deployment, detumble, and general mission phases. The key components of the C&DH system are the onboard computer, flight software, radio, and data storage systems. The C&DH system is responsible for all aspects of the spacecraft’s operation. It implements the control system, taking sensor data and issuing commands to the thrusters and ADC systems. It also collects, stores, and transmits sensor and payload data to ground stations. The individual component selection and overall architecture of the C&DH system is dependent on the overall needs of the mission and scientific payload.

### Data Handling Requirements
The needs of the scientific payload drives the majority of the data handling requirements. Given the processing power of modern processors, little consideration need be given to the needs of the ADC system. The mission profile of the satellite is relatively simple, requiring only basic slew operations, well within the capabilities of the onboard computer. In comparison the payload will be producing large amounts of data that needs to be stored, possibly operated on, and eventually transmitted to ground stations. Without information about the payload it is difficult to assess the exact needs of the system. Assume that the payload only produces data during the eLEO portion of the orbit, about five minutes worth of data. It would be impractical to attempt to transmit this data as it is collected. At a minimum the data must be stored for transmission later. Due to inconsistencies in ground station coverage it may not be possible to down-link all of the collected data at once depending on orbital position. Thus there must be sufficient storage space to collect data over the course of several orbits. At this time more information about the payload and ground station coverage is needed to make a comprehensive assessment of the data storage needs.

### Onboard Computer
We have selected the Clyde Space Kryten-M3 flight computer as our onboard computer (OBC). The Kryten-M3 is a flight proven OBC built around a Cortex-M3 processor core running at 50 MHz. Similar OBC’s from Clyde Space were also chosen by previous MQPs (cite previous MQP). The OBC includes 8 MB of MRAM and 4 GB of bulk flash memory. Both the MRAM and bulk storage include automatic error detection and correction (EDAC). This is especially important for correcting potential errors introduced by the high radiation environment of orbit. The system also includes space for an external SD card, increasing the bulk storage capacity (citation from clydespace).

![Kryten-M3 Onboard Computer](./images/obc.png){#fig:obc width=50%}

The Kryten-M3 is designed for use with FreeRTOS, a real-time operating system commonly used in high reliability embedded applications. FreeRTOS is highly configurable and capable of managing the entire spacecraft and scientific payload (citation from clyde).  Once more information is available about the payload there will be a discussion of the processing and data storage needs of the OBC.

### Radio Transceiver
The radio transceiver enables communication between the satellite and ground stations. Different radios operate in different bands and are capable of different data rates. Certain frequency bands are regulated by government agency and require specific approval to use. Depending on the data handling needs of the mission it may be necessary to use a radio capable of a higher data rate. As a result it may be necessary to consider the regulatory approval process needed to operate such radio systems. Once we have more information from the telecoms team about radio communications and ground coverage it will be discussed here.


## ADCS Testbed
