# Analysis

## Detumbling

Below is the analysis for the detumbling simulation, created in Simulink, the first stage of attitude control. Without successfully detumbling the spacecraft after deployment, the mission would not be able to continue to the next phase. The team simulated the detumbling of the CubeSat using Simulink. This was done by simulating the on board magnetometers and applying B-dot control theory to actuate the magnetometers. An option to use the CubeSat’s gyroscope was also simulated.

![Total Detumbling Simulink Simulation](./images/full-detumble.png){#fig:full-detumble width=50%}

Overall, the detumbling of the satellite was accomplished by providing inputs into our custom Simulink function ‘attdyn’ or attitude dynamics. The attitude dynamics block required an input control torque, quaternion, angular velocity, and moment of inertia, in order to output the rate of change of the quaternion ($\Dot{q}$) and the rate of change of the angular velocity ($\Dot{\omega}$). Moment of inertia was a constant input into the system and was given by the geometry of our CubeSat.

The attitude dynamics block uses the inputted quaternion to create a  quaternion product matrix, denoted $\Xi(q)$, that speeds up the quaternion multiplication process.This is because $\Xi(q)$ is the same as $[ q \odot$], which is a common quaternion operator.


From there, expressions for $\dot{q}$ and $\dot{\omega}$ can be derived and outputted.

$$ [L] = \begin{bmatrix} L(1) \\ L(2) \\ L(3) \end{bmatrix} $${#eq:detumble-L}

$$ [q] = \begin{bmatrix} q(1) \\ q(2) \\ q(3) \\ q(4) \end{bmatrix} $${#eq:detumble-q}

$$ [ \omega ] = \begin{bmatrix} \omega (1) \\ \omega (2) \\ \omega (3) \end{bmatrix} $${#eq:detumble-omega}

$$ [\Xi(q)] = \begin{bmatrix} q(4)&  -q(3)& q(2) \\ q(3)& q(4)& -q(1) \\ -q(2)& q(1)& q(4) \\ -q(1)& -q(2)& -q(3) \end{bmatrix} $$

$$ \dot{q} = \frac{1}{2}\Xi_q\omega $$

$$ \dot{\omega} = \frac{J}{\omega \cdot J \omega} $$

The quaternion and angular velocity inputs were first given by an initialized value from the simulations initialization file. Then, the outputted $\Dot{q}$ and $\Dot{\omega}$ got fed back into the Simulink loop, and integrated using a built in continuous integration block in Simulink. The resulting quaternion and angular velocity ($\omega$), then got passed back into the ‘attdyn’ block.

![Attitude Dynamics Plant](./images/attitude-dynamics.png){#fig:attitude-dynamics width=50%}

Inputting the control torques into the ‘attdyn’ block was a little more complicated and required the simulation of the magnetometers and the application of B dot control.

### Simulating the Input Control Torques
The first step in finding the necessary control torques for our attitude dynamics block, was to simulate the magnetometers. This was done with our custom Simulink function ‘Environment and Magnetometer Model.’ In order to use this function, we needed to take the inertial magnetic field reading and convert it to the body fixed frame by multiplying it by the spacecraft’s body fixed attitude.

To find the inertial magnetic field vector, we simulated the following path shown below.

![Simulation Path to Find Inertial Magnetic Field Vector](./images/orbit-prop.jpg){#fig:orbit-prop width=50%}

On the left, in the pink outlined boxes, are the constants that feed into the simple two-body time based orbit propagator. The constants fed into the orbit propagator block are the apogee, $r_a$, perigee, $r_p$, time, RA, inclination, i, argument of perigee, w, and the initial earth theta. The apogee, perigee and initial earth theta, $\theta_{i_earth}$ are unaffected constants. The RA, inclination and argument of perigee are entered as constants, in degrees, and translated into radians using a "Degrees to Radians" converter within Simulink. The "Degrees to Radians" converter follows the equation below.

$$ Radians = \theta * \frac{\pi}{180} $$

The final input, time, t, was specified by the clock input block which outputs the current simulation time as the simulation was running.

These seven inputs were then sent through the low accuracy orbit propagation block to output $V_{ecef}$ and $r_{ecef}$.

In order to find $V_{ecef}$ and $r_{ecef}$, the following process was used. To start, the $\mu$ of Earth, the semi-major axis, a, period, $p_t$, the mean motion, in radians, n, and time since perigee, TSP.

$$ \mu = 3.986e10^{14} $$

$$ a = \frac{r_a + r_p}{2} $$

$$ p_t = 2\pi*\sqrt{\frac{a^3}{\mu}} $$

$$ n = \frac{2\pi}{p_t} $$

Time Since Perigee is equal to the remainder of $\frac{t}{p_t}$.

After this, the mean anomaly, M, the specific energy, and the eccentricity are found.

$$ M = n TSP $$
$$ \epsilon = \frac{-\mu}{2a} $$
$$ e = \frac{r_a - r_p}{2a} $$

After this, we then need to compute the numerical approximation of the Inverse Kepler Equation. First, you need to find the eccentric anomaly, E. To do this, you initialize the values $E_n$ and $E_{nplus}$, being 10 and 0 respectively. Then, we ran $E_n$ and $E_{nplus}$ through a while-loop that states as long as the absolute value of $E_n$ and $E_{nplus}$ is greater than 0.001, $E_n$ and $E_{nplus}$ equal each other, therefore, $E_{nplus} = E_n - (\frac{(E_n - \epsilon \sin(E_n) - M}{1 - \epsilon \cos(E_n)})$.
Then, the output of the while-loop, $E_{nplus}$ is equal to the eccentric anomaly, E.
Using E, we then found the true anomaly, $\nu$, the distance, d, the angular momentum, h, and the orbital parameter, p.

$$ \nu = 2atan(\sqrt{\frac{1+e}{1-e}})tan(\frac{E}{2}) $$
$$ d = a(1-e \cos(E)) $$
$$ h = \sqrt{\mu a(1-e^2))} $$
$$ p = \frac{h^2}{\mu} $$

From here, we are then able to compute the position components of the spacecraft.

$$ x = d(\cos(RA) \cos(w+\nu)- \sin(RA) \sin(w+\nu \cos(i)) $$
$$ y = d(\sin(RA) \cos(w+\nu)- \cos(RA) \sin(w+\nu \cos(i)) $$
$$ z = d(\sin(i) \sin(w + \nu)) $$

Using these, we find our position vector:

$$ Position = [x y z] $$ 

From there, we are then able to find the velocity components.

$$ \Dot{x} = \frac{(x)(h)(e)}{(d)(p)} \sin(\nu)-\frac{h}{d}(\cos(RA) \sin(w+\nu)+ \sin(RA) \cos(w+\nu) \cos(i)) $$
$$ \Dot{y} = \frac{(y)(h)(e)}{(d)(p)} \sin(\nu)-\frac{h}{d}(\sin(RA) \sin(w+\nu)- \cos(RA) \cos(w+\nu) \cos(i)) $$
$$ \Dot{z} = \frac{(z)(h)(e)}{(d)(p)} \sin(\nu)+\frac{h}{d}\sin(i) \cos(w+\nu) $$

From there, we can calculate the velocity vector:

$$ Velocity = [\Dot{x} \  \Dot{y} \  \Dot{z}] $$

Using $\omega_{earth}$, which is equal to $7.29e^{-5}$ rad/s, we can find the true anomaly at time t, $\theta_t$.

$$ \theta_t = \omega_{earth}t + \theta_{i_{earth}} $$

Next, we defined a normal rotation matrix that includes $\theta_t$.

$$ [RM] = \left( \begin{bmatrix} cos(\theta_t)& sin(\theta_t)& 0 \\ -sin(\theta_t)& cos(\theta_t)& 0 \\ 0& 0& 1 \end{bmatrix} \right) $$

Finally, to find the position of the spacecraft in ECEF, we transposed the product of the rotation matrix and the initial position vector defined above. Finding the velocity of the spacecraft in ECEF was a bit more complicated, but defined below.

$$ [V_{ecef}] = RM \left( V \begin{bmatrix} \omega_{earthY} \\ \omega_{earthX} \\ 0 \end{bmatrix} \right) $$

Where V is the velocity vector defined above.

After we found $V_{ecef}$ and $r_{ecef}$, we then converted $r_{ecef}$ from ECEF to LLA using a built in Simulink block in the Aerospace toolbox. From there, the $r_LLA$ coordinates filter into a Demux block in Simulink. The Demux block split the vector signals into scalar values. This specific Demux block splits the input into three scalar outputs, height, h, in meters, latitude, $\mu$, in degrees, and longitude, l, in degrees. These three scalar outputs then flow into the World Magnetic Model, WMM2015, that is also built into Simulink, along with the decimal year, which is created by a clock and a built in MatLab function that converts the input clock time, that's equal to the time input in the orbital propagator, to decimal years. The World Magnetic Model implements the mathematical representation of the National Geospatial Intelligence Agency (NGA) World Magnetic Model, which is then used to find the Magnetic field vector, in nT, which is then fed directly into the Environmental and Magnetometer Model as one of the inputs.

To find the body fixed attitude, the quaternion within the detumbling loop was fed into a built in Simulink function that converted quaternions to a direct cosine matrix. Once we had both the inertial magnetic field reading, and the direct cosine matrix representation of the spacecraft’s attitude, a simple matrix multiplication yielded our true magnetic field in body fixed coordinates.

![Integration of the q and omega vectors and Simulink q to DCM block](./images/qtoDCM.png){#fig:q-to-dcm width=50%}


This output was then seeded with noise via a noise maker we created, to simulate how a real sensor would operate. The noise characteristics we inputted into the simulation are on par with those listed in the spec sheet for our magnetometer, and provided a random Gaussian distribution that was added to the true magnetic field vector. Next, the magnetic field with the addition of noise was passed through a Simulink Quantizer that discretizes our input to a specific interval. For our case, the quantization value was set to 200, which was an arbitrary value that we found worked well after some trial in error. The quantization block works by using a quantization algorithm with a round-to-nearest method to map signal values to quantized values at the output that are defined by the quantization interval. After the vector is quantized, it is passed through a built in Simulink function called ‘Zero-Order Hold.’ This block essentially sets the sample rate of our signal to any specific value. We set our value to $\frac{1}{220}$, or 220 Hz to be consistent with our sensors refresh rate. For comparison purposes, a ‘Discrete Derivative’ block was added in order to see the unfiltered rate of change of the magnetic field, or $\Dot{B}$. This was to ensure that once we passed the magnetic field through our low pass filter, the filter was properly working.

![Contents of the Magnetometer Simulation Block](./images/magnetometer-block.png){#fig:magnetometer-block width=50%}

The next step was to pass our outputted B vector from our ‘Environment and Magnetometer Model’ through a low pass filter to filter out the noise we previously added.

![Magnetometer Simulation Block and Low Pass Filter](./images/magnetic-field-block.png){#fig:magnetic-field-block width=50%}

Simulink allows for users to specify passband edge frequency and stopband edge frequency in Hz. Those values were set to 90 and 120 Hz respectively. This was due to the sample rate of our magnetometer being 220 Hz. Any fast, small changes in magnetic field vector are most likely errors, so we want to remove any changes that aren’t happening slowly.

![Contents of the Low Pass Filter Block](./images/low-pass.png){#fig:low-pass width=50%}

In theory, this also means that if we were spinning fast enough, it would filter out that too, and make the satellite spin faster. However, because we are using gyroscope measurements as a second reference, this situation would not occur. The derivative of the signal was then found using another ‘discrete derivative’ block, to give us the filtered rate of change of the magnetic field, or $\Dot{B}$.

## Reaction Wheel Sizing

The reaction wheels selected for the CubeSat had to be capable of producing enough torque to counteract the disturbance torques on the satellite. First, the gravitational torque was defined by:

$$ T_g = \frac{3M}{2R^3}|I_x - I_y|sin(2\theta) $$

Where $\theta$ is equal to 5$^{\circ}$, or 0.0872005 radians, M is the mass of the Earth, $3.9816x10^{14}\frac{m^3}{s^2}$ and R is the radius of the earth, $6.978x10^6m$. The next disturbance torque needed to be accounted for was the torque due to solar pressure. This was defined by:

$$ T_{sp} = F(C_{ps} - C_g) $$

Where $C_{ps}$ is the center of solar pressure and $C_g$ is the center of gravity. The flux due to solar pressure, $F$ is defined as:

$$ F = \frac{F_s}{c}A_s(1+q)cos(l) $$

Where l is the angle of incidence to the Sun, $F_s$, is the solar flux constant , $1.367 \frac{W}{m^2}$, c is the speed of light, $3x10^8 \frac{m}{s}$, $A_s$ is the surface area of the RAM facing surface, 0.02 m, and q is the coefficient of reflection, 0.6. Next We can then calculate the torque due to the magnetic field by the following equation:

$$ T_m = DB $$

Where D is equal to the dipole moment of the vehicle and B is the Earth's magnetic field. The final disturbance torque to consider was the torque due to aerodynamic pressure. This was defined as:

$$ T_a = 0.5[\rho C_d A_s V^2](C_{pa}-C_g) $$

Where V is the velocity of the vehicle and $C_{pa}$ is the center of aerodynamic pressure.
With all of the disturbance torques calculated and summed, we now had a known value for the disturbance torques we needed to overcome. To find the torque for the reaction wheel sizing, $T_{rw}$, we evaluated the following expression:

$$ T_{rw} = T_D(M_f) $$

Where $M_f$ is the margin factor to help calculate the torque of the reaction wheel for the disturbance rejection and $T_D$ is the reaction wheel torque for the worst case anticipated torque. The Reaction wheel torque, $T_{rw}$ must be equal to the worst case anticipated disturbance torque plus some margin.
Finally, the momentum storage can be calculated by:

$$ h = T_D\frac{t}{4}(0.707) $$

Where t is the orbital period, in seconds and 0.707 is the rms average of a sinusoidal function.

## Simulink Control

### Detumble Control Subsystem
This is the subsystem that takes the filtered measurement readings and uses them to command control torques to detumble the spacecraft. There are two different control laws that can be chosen in this block by setting a variable in the initialization file depending on the desired control method.

#### Bang-Bang B-dot Control
The first control law block made for this model implements the control law described in !@eq:bbBdot using an estimate of $\dot{\vec{B}}$ and a preset maximum for magnetorquer dipole moment. The block sums up the individual commanded dipole moments from each of the magnetorquers and the finds the resultant torque using !@eq:mxB. This resultant torque, $\vec{L}$, is then passed to the rest of the subsystem.

#### Proven-Stability B-dot Control
The second control law block, it is a direct improvement on the first control law because of the use of a direct measurement of the angular velocity. This block implements !@eq:rewrtbdot to find a resultant torque given measurements from the gyroscopes and the magnetometers. Because this control law does not use $\dot{\vec{B}}$, which is found by taking discrete differences of a corrupted signal, it is more effective than the other controller. In addition, as was shown in an earlier section, this control law is has proven Lyapunov stability. The advantages of this control law far outweigh the downside of having to also use the gyroscopes for this law, so this is the control law that is used for our analysis. The option of using the other law is still available, although discouraged.

#### Control Activation Delay
In order to meet any potential guidelines for launching from the ISS, this block gives us the option of delaying the activation of the active controls until a set amount of time has passed. This also has the benefit of allowing our controls to ignore the initial transient signals that result from differences between initial conditions and initial estimates upon simulation start up. This block is very simple and does not apply a signal delay to the commanded torques when they are allowed to pass. In addition, it does nothing to any disturbance torques that may be added to the simulation regardless of simulation time or set delay.

## STK

Systems Tool Kit, or STK, is a physics-based software packaged created by Analytical Graphics (AGI). STK allows engineers to perform analyses of ground, sea air and space objects in their respective environments. STK is used in government, commercial and defense applications around the world to determine the attitude, spatial relationships and time-dynamic positions of objects under many simultaneous constraints. STK’s ability to simulate subsystems like ADC is what makes the software so valuable to the aerospace industry. Our team considered using STK to simulate the detumbling and attitude control of our 6U CubeSat. An example of the simulation can be seen below.

![STK Satellite View](./images/stk.png){#fig:stk}

The figure above contains a snapshot of our CubeSat in its simulated elliptical orbit. STK has the ability to overlay particular orbital elements as vectors, such as sun vector and the nadir vector. Attitude determination in STK is done primarily with the use of quaternions. We originally intended on integrating directly between STK and MATLAB. MATLAB was to be used as our attitude determination and controller, whereas STK was simply to be used as a visual simulation, with the potential of utilizing it for sensor readings such as magnetic field and the sun vector. Utilizing the work of Xiang Fang and Yunhai Geng of the Research Center of Satellite Technology of Harbin Institute of Technology (CITATION) Fang and Gent laid out detailed steps into the integration of MATLAB into a real-time running STK environment. MATLAB was to be used to introduce sensor noise, whether that be from MATLAB simulated sensors, or STK sensors with their data passed into MATLAB. We would then pass the noisy sensor data through an extended Kalman Filter (EKF) prior to being sent to the controller. This EKF-processed data could be compared to the full state vector to check on the functionality of our EKF. Attitude control commands would then be sent to the controller flow, and then pass the data back into the STK environment for visual simulation, completing the loop.

![STK-Matlab Integration Workflow](./images/stk-matlab.png){#fig:stk-matlab width=50%}

Although without any code snippets or a codebase to be referenced, it is possible to integrate MATLAB into STK using the above figure and aforementioned paper as a guide. However, our focus changed from purely using MATLAB for our attitude and orbital simulation and control, to primarily utilizing Simulink into our control scheme.

![Matlab CubeSat Simulation Toolbox](./images/cubesat-toolbox.png){#fig:cubesat-toolbox}

Since changing our focus to Simulink, the MATLAB to STK integration has become less realistic of a goal, especially with the introduction of the Aerospace Blockset CubeSat Simulation Library, which included an orbital propagator, which simulates in 3-D the orbital and attitude elements of a CubeSat. We will utilize this CubeSat Simulation Library along with using a Level-2 S-Function block, which will connect our Simulink controller to a STK environment, similar to what we had planned with MATLAB. Similar to our approach outlined above, Simulink will do all of the spacecraft control, and STK will merely model and return whatever sensor data we request.

## System Power Requirements
