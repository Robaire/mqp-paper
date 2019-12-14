# Background
## Attitude Determination and Control
The purpose of the attitude determination and control subsystem (ADCS) is to properly position and orient the spacecraft to meet the needs of the mission. The ADCS is responsible for three distinct operations throughout the mission: detumbling, initial attitude determination, and attitude maintenance. Successful operation in all three phases is vital to the overall success of the mission. This is accomplished by combining a variety of sensors and actuators in a closed-loop control system. 

### Requirements
Each phase of the mission has different requirements. In order to successfully detumble the satellite must correct for the angular spin imparted during deployment. It is expected that such an angular velocity would not have a magnitude greater than ten degrees per second about any axis. Once the satellite has reduced its angular velocity about two of its axis it is considered detumbled. It must then determine its orientation with respect toe the Earth inertial frame. From this point onwards the satellite must maintain its attitude within plus or minus five degrees. As the orbit dips lower into the atmosphere the effects of drag become significant. Should the angular orientation deviate further than this limit the torques induced by atmospheric drag risk overcoming the strength of the on board actuators, causing the spacecraft to enter an uncontrollable spin. The torque exerted on the spacecraft can be described as a function of atmospheric density $\rho$, cross sectional area $A$, velocity $V_{rel}$, drag coefficient $C_D$, center of pressure $c_p$, and center of gravity $c_g$, as shown in equation !@eq:drag-torque.

$$ \tau_d = \frac{1}{2} \rho A C_D V_{rel}^2 (c_p - c_g) $${#eq:drag-torque}

Velocity is a function of the orbital velocity and the pointing angle. As the velocity increases, it is evident that the torque experienced by the spacecraft increases as well. This relationship is shown in figure !@fig:angle-torque.

![Pointing Angle vs. Torque](./images/pointing-angle-torque.jpg){#fig:angle-torque}

### Actuator Selection
Actuators are needed to effect change on the satellites orientation in space. Two different types of actuators are used to accomplish this, magnetorquers and reaction wheels. Magnetorquers create a torque using an external magnetic field and a magnetic dipole moment. A unique property of the magnetorquer is that is has no moving parts, unlike a reaction wheel that relies on moving parts. Due to this unique characteristic, magnetorquers are less prone to malfunction, and along with being cheap, lightweight and simple, are great for CubeSats. Magnetorquers are commonly made from a metal rod wrapped in a copper wire. When a current is run through the coil a magnetic moment is created. The interaction of this magnetic moment, $\nu$, with the Earth's magnetic field, $B$, induces a torque, $\tau$, on the space craft as shown in equation !@eq:magnetic-torque.

$$ \tau = \nu \times B $${#eq:magnetic-torque}

The magnetorquer chosen for this task is the NCTR-M002 Magnetorquer Rod, which is manufactured by *New Space*. The NCTR-M002 uses a magnetic alloy rod that produces an amplification effect over an air cored magnetorquer. The NCTR-M002 consumes 200mW of power from a 5V power supply while producing a magnetic moment greater than 0.2 $Am^2$. The residual moment left over from the magnetorquer rod is negligible at less than 0.001 $Am^2$. Specific performance characteristics are listed in table !@tbl:magnetorquer.

![NCTR-M002 Magnetorquer Rod](./images/magnetorquer-rod.jpg){#fig:magnetorquer}

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

In addition to the magnetorquers reaction wheels will be included onboard the CubeSat. Reaction wheels, also refereed to as momentum wheels, are internal components that store rotational momentum, providing satellites with three axis control. By adjusting the momentum of a weighted wheel the reaction wheel induces an equal and opposite torque on the space craft, as the total momentum of the space craft must remain constant. Reaction wheels provide very precise pointing control without the fuel requirements of conventional attitude control thrusters. The reaction wheel chosen for this mission is the RWP050, pictured in figure !@fig:reaction-wheel. This wheel can create a maximum torque of 0.007 Nm and create a total moment of 0.050 Nms while consuming less that 1W at full power. Operating parameters are available in table !@tbl:reaction-wheel.

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

### Sensor Selection
The CubeSat uses a collection of sensors to accurately determine its attitude and position relative to the Earth. A Global Positioning System (GPS) receiver allows the satellite to determine its position above the Earth. This information is then used to predict the expected magnetic field and sun vectors using well proven models. The NGPS-01-422, manufactured by *New Space* is a great choice for the on-board GPS. It is relatively low powered and provides sufficiently accurate readings. The GPS module includes an external antenna, the NANT-PTCL1. Specifications for the GPS and antenna are available in tables !@tbl:gps-parameters and !@tbl:gps-receiver-parameters respectively. 

![GPS Module and Antenna](./images/gps-receiver.jpg){#fig:gps-receiver width=50%}

| Parameter | Value |
|:---------:|:-----:|
| Mass | <500 g |
| Power Consumption | 1.5 W |
| Position | < 10 m |
| Velocity | < 25 $\frac{cm}{s}$ |
| Operating Temperature | $-10^{\circ} C$ to $50^{\circ} C$ |
| Dimensions | $155 \times 76 \times 34$ mm |

Table: GPS Parameters {#tbl:gps-parameters}

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

A gyroscope is also included to measure the angular rotation rates of the spacecraft. The gyroscope chosen is the ADXRS453 manufactured by Analog Devices. This gyroscope consumes very little power and is very light weight.

![ADXRS453](./images/gyroscope.jpg){#fig:gyroscope}

| Parameter | Value |
|:---------:|:-----:|
| Mass | <56.7 g |
| Power Consumption | 18.9 mW |
| Operating Temperature | $-40^{\circ} C$ to $105^{\circ} C$ |
| Dimensions | $33 \times 33 \times 3$ mm |

Table: Gyroscope Parameters {#tbl:gyroscope-parameters}

The LSM303AGR is a combination triple axis accelerometer and magnetometer. It is small, lightweight, low cost, and power efficient compared to other accelerometer and magnetometer options. Magnetometer readings allow the spacecraft to estimate its orientation based on the expected magnetic field given its position in orbit. 

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

Sun sensors are used to determine the position of the sun relative to the space craft. This provides a high accuracy method for determining the satellite's attitude. Two-axis, fine sun sensors measure the incident angle of the sun in two orthogonal axes. A total of five Nano-SSOC-A60 sun sensors are mounted to the satellite. Five sensors are used to provide near total coverage such that the sun is always in view of at least one sensor.

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

### Detumbling
All CubeSat deployment systems will induce some angular rotation to the CubeSat as it is deployed. As such, the CubeSat will immediately begin tumbling at the beginning of its mission, even before the satellite can activate any sort of attitude control system. Generally, the angular velocity induced by the deployer is unknown and would interfere with the operation of the space craft. It becomes necessary to detumble the satellite before it can begin its main mission. One of the most common methods used to detumble satellites is the B-Dot control algorithm. The B-Dot controller detumbles a spacecraft by commanding a magnetic dipole moment based on the rate of change of the Earth's magnetic field, hence the term 'B-Dot'. The torque produced by the interaction of the magnetorquers' magnetic field with the Earth's magnetic field, as described in equation !@eq:magnetic-torque. acts counter to the angular velocity of the spacecraft, slowly reducing its magnitude. If the spacecraft's angular velocity, $\vec{\omega}$, is known, as provided by gyroscopes, the command torque, $\vec{m}$, can be expressed in terms of the gain, $k$, and magnetic field as shown in !@eq:control-torque. The normal of the Earth's magnetic field is expressed as $\vec{b}$ in !@eq:unitized-B.

$$ \vec{b} = \frac{\vec{B}}{\|\vec{B}\|} $${#eq:unitized-B}

$$ \vec{m} = \frac{k}{\|\vec{B}\|}\vec{\omega}\times\vec{b} $${#eq:control-torque}

It is assumed that the change in the Earth's magnetic field due the change in orbital position over time occurs much more slowly than the change of the magnetic field in the spacecraft body frame due to the the tumbling motion. The B-Dot control law is expressed in !@eq:b-dot.

$$ \vec{L} = \vec{m} \times \vec{B} $${#eq:b-dot}

To make it simpler to prove the stability of the control law, !@eq:b-dot can be rewritten as !@eq:b-dot-stable.

$$ \vec{L} = -k(I_3-\vec{b}\vec{b}^T)\vec{\omega} $${#eq:b-dot-stable}

To prove the sability of the control law, it has to be shown to reduce a positive Lyapunov function asymptotically to zero by making the derivative of the function less than or equal to zero for all cases. For this application the Lyapunov function in !@eq:lyapunov is used; where $J$ is the spacecraft moment of inertia matrix. This function is analogous to the rotational kinetic energy. This is useful as the goal of the detumbling control law is to reduce the total angular velocity, and thus reduce the rotational kinetic energy. 

$$ V = \frac{1}{2}\vec{\omega}^TJ\vec{\omega} $${#eq:lyapunov}

Applying the B-Dot control law written in !@eq:lyapunov, its derivative is calculated as !@eq:v-dot.

$$ \dot{V}=-k\vec{\omega}^T(I_3-\vec{b}\vec{b}^T)\vec{\omega} $${#eq:v-dot}

The eigen values of $(I_3 - \vec{b} \vec{b}^T)$ are always 0, 1, and 1 implying that $(I_3 - \vec{b} \vec{b}^T)$ is positive semidefinite. Therefore, $\dot{V}$ is always less than or equal to zero. The only case where $\dot{V}$ is equal to zero and global asymptotic stability cannot be obtained is when $\vec{\omega}$ is parallel to $\vec{b}$, in which case the spacecraft is already not tumbling. This this case, the spacecraft would only be spinning around one axis and the mission can move into the attitude determination phase. If it was necessary to use !@eq:control-torque in a bang-bang application, it can be rewritten into !@eq:b-dot-bangbang, where $N$ is the number of magnetorquers on the spacecraft, $m_i^{max}$ is the maximum moment the $i-th$ magnetorquer can delivery, and $u_i$ is the direction of the magnetic moment for the $i$th magnetorquer. This implementation is less computationally expensive than the other implementation, but less efficient in terms of power consumption.

$$ \vec{L} = \sum_{i=1}^N-m_i^{max}sign(\vec{u_i} \cdot \dot{\vec{B}}) $${#eq:b-dot-bangbang}

<!-- #### Sun Pointing 
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

It is obvious that this method has a lot of assumptions including negligible calibration errors and biases, but, although they are not true in flight, numerical simulation results demonstrate that this method is capable of achieving CSS pointing despite these biases. -->

### Attitude Determination
There are two main types of methods used to determine the attitude of a spacecraft: recursive and deterministic. Recursive methods work by estimating the current attitude from a previous attitude measurement. To estimate the current attitude quaternion, $q_n$, the previous attitude quaternion, $q_{n-1}$, is integrated. An alternative to recursive methods are deterministic methods. Deterministic algorithms estimate the current attitude quaternion by comparing a set of sensor observations with their expected values. Previous MQP teams used the deterministic TRIAD method to determine attitude. The TRIAD algorithm is well understood, reliable, and computationally efficient, making it an ideal choice for simple attitude determination systems.

Harold Black's 1964 TRIAD algorithm was the first of its kind. It utilizes the sun and Earth magnetic field vectors to determine a spacecrafts orientation. To implement this algorithm four pieces of information must be known: the sun and magnetic field vectors in the spacecraft body-fixed frame, and the sun and magnetic field vectors in the Earth-fixed reference frame @triad. The sun and magnetic field vectors in the body-fixed frame are easily determined using the onboard magnetometer and sun sensors. Values for the sun and magnetic field vectors in the Earth-fixed frame can be determined using the position of the satellite in its orbit, provided by the GPS, and mathematical models. The World Magnetic Model (WMM) provides the Earth's magnetic field vector at any point in LEO given a satellite's longitude, latitude, and altitude @wmm. The sun vector in the Earth-fixed frame can be calculated using the time of the year with a precision of $0.01^{\circ}$ (36'') for years between 1950 to 2050. The number of days, $n$, since Greenwich Noon, Terrestrial Time, on the 1st of January, 2000 can be computed given the current Julian Date, $JD$, using !@eq:julian-date.

$$ n = JD - 2451545.0 $${#eq:julian-date}

The mean longitude of the sun, $L$, and the mean anomaly of the sun, $g$ can be determined by applying $n$ to !@eq:sun-longitude and !@eq:sun-anomaly respectively.

$$ L = 280.46^{\circ} + 0.9856474^{\circ}n $${#eq:sun-longitude}

$$ g = 357.528^{\circ} + 0.9856003^{\circ}n $${#eq:sun-anomaly}

From these values the ecliptic longitude of the sun, $\lambda$, and the distance to the sun in astronomical units, $R$, can be calculated using !@eq:sun-ecliptic and !@eq:sun-distance.

$$ \lambda = L + 1.915^{\circ} \sin(g) + 0.020^{\circ} \sin(2g) $${#eq:sun-ecliptic}

$$ R=1.00014 - 0.01671 \cos(g) - 0.00014 \cos(2g) $${#eq:sun-distance}

It then becomes possible to determine the position of the sun in equatorial coordinates using !@eq:sun-obliquity - !@eq:sun-z.

$$ \epsilon = 23.439^{\circ} - 0.0000004^{\circ}n $${#eq:sun-obliquity}
$$ X = Rcos(\epsilon)cos(\lambda) $${#eq:sun-x}
$$ Y = Rcos(\epsilon)sin(\lambda) $${#eq:sun-y}
$$ Z = Rsin(\epsilon) $${#eq:sun-z}


The TRIAD method allows for coordinates in the body fixed reference frame to be rotated into the Earth-fixed inertial reference frame. However, because vectors must be normalized for this method, it is possible that there could be a decent amount of sensor noise. Later research, specifically by Grace Wahba, poses a potential way of minimizing input sensor error, thus making the TRIAD method more accurate. Solutions to Wahba’s problem include Davenport’s q-method and QUEST. These methods provide and optimum quaternion for expressing the spacecrafts attitude.


The TRIAD method begins by defining two linearly independent body fixed vectors, $b_1$ and $b_2$, as well as their corresponding reference frame vectors, $r_1$ and $r_2$. The attitude matrix, $A$, is defined as a rotation from the body-fixed frame into the Earth-fixed frame in !@eq:body-earth-rotation.

$$ Ar_i = b_i $${#eq:body-earth-rotation}

Ideally, $A$ would be the same for both $i=1$ and $i=2$, however due to noise in each sensor input this may not be strictly true. For the TRIAD method to work, it is assumed that the transformation for $i=1$ is significantly more accurate than its counter part. Two sets of orthonormal right-handed triads of vectors are defined, one for the reference frame, $M_{ref}$, and one for the body frame, $M_{obs}$.

$$ M_{obs} = \langle r_1, \frac{r_1 \times r_2}{|r_1 \times r_2|}, r_1 \times \frac{r_1 \times r_2}{|r_1 \times r_2|} \rangle $${#eq:m_obs}

$$ M_{ref} = \langle b_1, \frac{b_1 \times b_2}{|b_1 \times b_2|}, b_1 \times \frac{b_1 \times b_2}{|b_1 \times b_2|} \rangle $${#eq:m_ref}

A direct cosine matrix can be obtained by substituting $M_{obs}$ and $M_{ref}$ for $b$ and $r$ in !@eq:body-earth-rotation. Thus, !@eq:body-earth-rotation can be rewritten as !@eq:body-earth-dcm.

$$ AM_{obs} = M_{ref} $${#eq:body-earth-dcm}

Expanding the relation for each individual vector and simplifying the expression for the direct cosine matrix becomes !@eq:body-earth-dcm-expanded.

<!-- $$ A_{\text{DCM}} = b_1 r_1^T + (b_1 \times b_x)(r_1 \times r_x)^T + b_x r_x^T $${#eq:body-earth-dcm-expanded}
 -->
$$ A_{\text{DCM}} = b_1 r_1^T + (b_1 \times \frac{b_1 \times b_2}{|b_1 \times b_2|})(r_1 \times \frac{r_1 \times r_2}{|r_1 \times r_2|})^T + \frac{b_1 \times b_2}{|b_1 \times b_2|} \frac{r_1 \times r_2}{|r_1 \times r_2|}^T $${#eq:body-earth-dcm-expanded}

This provides a mechanism for coordinates to be rotated from the body-fixed frame to the Earth-fixed and back. It is important to note however, that for cases where either the reference vectors or observed vectors are parallel or anti-parallel equation !@eq:body-earth-dcm-expanded becomes undefined. 

<!-- #### Initial Attitude Determination
In order to begin to understand what attitude determination and control algorithms were needed for this project, the team first needed to understand the two main categories of ADC, and how previous project teams applied them to their cubesat. The first subcategory is recursive ADC. Recursive methods work by comparing the current attitude to the most recent attitude. A recursive function is a function that calls itself, so this form of ADC provides an updated state estimate using its previous state estimate. For example, if the current attitude quaternion is denoted by qn then the recursive method would estimate that value by using information from its previous estimate, qn-1. The second type of ADC method is deterministic ADC. Deterministic methods use current sensor readings as well as reference readings to calculate an attitude estimate based on the difference between the two values. 

Previous MQP teams used the deterministic TRIAD method for their attitude determination and control. This was used in unison with noise filtering methods in an attempt to minimize TRIAD error while also producing an optimal quaternion for initial attitude determination [2017 citation here]. The TRIAD method was used as a baseline to get a directional cosine matrix. With that information, newer methods were then used to produce an accurate quaternion from the TRIAD output (Farhat 2013). Due to the widespread application of this approach, as long with its efficiency, our team decided to maintain the same type of analysis for our project.  -->
<!-- 
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

This allows for coordinates to be rotated to the body fixed frame and vice versa, using a little linear algebra. It is important to note however, that for cases where either the reference vectors or observed vectors are parallel or antiparallel equation 2.12 is undefined, meaning the attitude matrix cannot be found. It is also easy to see how the assumption made earlier (that b1is entirely accurate) can lead to errors in the cosine matrix. The formula for ATRIAD heavily relies on vectors with subscript 1 which in practice have some degree of error associated with them (Markley). This means solutions to Wahba’s problem have to be used to minimize this error.  -->

#### Wahba's Problem

Mathematician Grace Wahba attempted to describe issues associated with using direction cosine matrices in attitude estimation and provide a way to build upon the TRIAD method. Improvements include, adding a way to weigh sensor measurements and allowing for more than 2 sets of measurements to be used. To understand how to improve upon the TRIAD method, one must understand the problem Wahba proposed. Wahba’s problem serves to find an orthogonal matrix with a positive determinant that minimizes the loss function in !@eq:loss-function

$$ L(A) = \frac{1}{2} \sum\limits_{i=1}^N a_i ||b_i - A r_i||^2 $${#eq:loss-function}

This essential finds the rotation matrix $A$ that brings the first set of unit vectors $(b_1, b_2, \hdots, b_n)$ into the best least squares coincidence with the second set of vectors $(r_1, r_2, \hdots, r_n)$. Where $b_i$ is a set of $n$ unit vectors in the spacecraft body frame, $r_i$ is the corresponding set of vectors in the reference frame, and $a_i$ represents the non-negative weights of each sensor. These weights must be applied according to the accuracy of the sensors. This is necessary to relate Wahba’s problem to Maximum Likelihood Estimation, a technique that uses sensor accuracy to help to more accurately model a system. Using the orthogonality of $A$, the unit norm of the unit vectors, and the cyclic invariance of the trace we can rewrite $L(A)$ in !@eq:loss-function-2.

$$ L(A) = \lambda_0 - tr(AB^T) $${#eq:loss-function-2}

$$ \lambda_0 = \sum\limits_{i=1}^N a_i \ \ \text{and} \ \ B = \sum\limits_{i=1}^N a_i b_i r_i^T $$

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
The QUEST method (QUaternion ESTimator) builds off of davenport's Q method, and allows for more frequent attitude computations. Currently, it is the most widely used algorithm for solving Wahba’s problem, and was first used in 1979 by the MAGSAT spacecraft. The algorithm begins by rewriting into two separate equations:

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
The needs of the scientific payload drives the majority of the data handling requirements. Given the processing power of modern processors, little consideration need be given to the needs of the ADC system. The mission profile of the satellite is relatively simple, requiring only basic slew operations, well within the capabilities of the onboard computer. In comparison the payload will be producing large amounts of data that needs to be stored, possibly operated on, and eventually transmitted to ground stations. The payload produces data at 13.7 kbps. This data must be stored and eventually down-linked to a ground station. Based on the ground station coverage a total of 237.01 Mb of data may be downloaded each day.  Thus there must be sufficient storage space to collect data over the course of several orbits.

### Onboard Computer
We have selected the Clyde Space Kryten-M3 flight computer as our onboard computer (OBC). The Kryten-M3 is a flight proven OBC built around a Cortex-M3 processor core running at 50 MHz. Similar OBC’s from Clyde Space were also chosen by previous MQPs. The OBC includes 8 MB of MRAM and 4 GB of bulk flash memory. Both the MRAM and bulk storage include automatic error detection and correction (EDAC). This is especially important for correcting potential errors introduced by the high radiation environment of orbit. The system also includes space for an external SD card, increasing the bulk storage capacity.

![Kryten-M3 Onboard Computer](./images/obc.png){#fig:obc width=50%}

The Kryten-M3 is designed for use with FreeRTOS, a real-time operating system commonly used in high reliability embedded applications. FreeRTOS is highly configurable and capable of managing the entire spacecraft and scientific payload.  

### Radio Transceiver
The radio transceiver enables communication between the satellite and ground stations. Different radios operate in different bands and are capable of different data rates. Certain frequency bands are regulated by government agency and require specific approval to use. Depending on the data handling needs of the mission it may be necessary to use a radio capable of a higher data rate. As a result it may be necessary to consider the regulatory approval process needed to operate such radio systems.
