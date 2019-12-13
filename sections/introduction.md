# Introduction
## Purpose
The goal of this project is to design and conduct analysis of a CubeSat on an extreme low Earth orbit (eLEO) mission. The satellite will be carrying a mass spectrometer to conduct atmospheric observation. Following deployment from the ISS the satellite will enter a 250 - 600 kilometer orbit and maintain this orbit as long as possible. The overall project is composed of three separate MQP teams, each responsible for a different aspect of the satellite design and analysis. This portion of the overall project focuses on the attitude determination and control, orbital determination and control, and command subsystems of the satellite.

## Project Constraints
There are four major constraints on this project: the orbit profile, the primary propulsion system, the scientific payload, and the satellite form factor. As mentioned in the introduction, the satellite must enter and maintain a 250 - 600 kilometer orbit as long as possible. This is to allow the scientific payload, the miniature Ion Neutral Mass Spectrometer (mini-INMS), to conduct atmospheric analysis in low Earth orbit. A Busek electro-spray thruster has already been selected as the primary propulsion system for this mission profile and cannot be changed. It is expected that the satellite adhere to the CubeSat form factor, although the final size of the satellite is flexible.

## CubeSat Background
Cube satellites (CubeSats) are miniature satellites used for space research and technology development. There are a particular class of nano-satellite that was developed by the California Polytechnic State University and Stanford University in 1999 in order to promote the design, manufacture, and testing of satellite technology in low Earth orbit @cubesat. CubeSats are comprised of multiple 10 cm by 10 cm by 10 cm units, referred to as 'U's. Layouts can vary greatly, but the most common form factors are 1U and 3U @cubesat. In recent years, larger CubeSats have been developed to increase available space for mission payloads. Typically, CubeSats are deployed by a launch mechanism attached to the upper stage of a launch vehicle and offer an easy way to deploy CubeSats into Earth orbit.

## Educational and Social Impacts of CubeSats
The continued expansion and development of CubeSat technologies has yielded a variety of positive social, economic, and educational effects. The space industry has been heavily impacted by the surge of CubeSat and related space technology start-ups in the last five years. From 2000 to 2014 space start-ups have received a combined sum of $1.1 billion in venture capital investments @cite. The number has only increased in recent years, with a total investment of $3.9 billion in 2017 alone @cite. These companies provide satellite components, development, or launch and integration services. 

The wide availability of CubeSat components and hardware has significantly reduced the cost and complexity of creating a flight capable system. Many universities and several high schools have launched CubeSats thanks to these advantages @cite. Low development and launch cost also provides an opportunity for cost effective flight testing of new and experimental technologies. In a notable example, the *Mars InSight* mission carried two CubeSats as a secondary payload in order to test new miniaturized deep space communication equipment @insight. 

The expansion and promotion of CubeSats has also garnered interest from students at all levels in the space sector. Space agencies such as NASA and ESA are able to use CubeSats to inspire students to pursue an education and career in STEM fields. 

# Introduction
The primary goal of the MQP-2001 Cube Satellite Project is to create a conceptual design  of a 6U CubeSat, carrying a mini Ion Neutral Mass Spectrometer (mINMS) made by NASA Goddard for the purpose of scientific experimentation and data collection in the ionosphere. The 6U CubeSat will be deployed from the ISS via a NanoRacks DoubleDeployer at 400 km with a 51.6 degree inclination. It will then enter a 250-600km orbit after detumbling, maintaining a perigee within extreme Low Earth Orbit (eLEO) for as long as possible with the given propulsion system.

The MQP-2001 CubeSat Project is composed of three MQP sub-teams, each with a different focus. Team Two, led by Professor Demetriou with team members Robaire Galliath, Oliver Hasson, Andrew Montero, Chris Renfro and David Resmini, focuses on CubeSat attitude determination and control, orbital control, and command and data handling subsystems. Team 2 is also taking lead on the design and development of a test bed. Team One is led by Professor’s Taillefer and Gatsonis, with team members Tristan Andreani, Edward Beerbower, Roberto Clavijo, Samuel Joy, Benjamin Synder and Jeremiah Valero. These members are responsible for the orbital analysis and environmental effects analysis along with the power, propulsion and telecommunications subsystems. Lastly,  team three is led by Professor Karanjgoakar with team members Christian Anderson, Rory Cuerdon, Brian Kelsey and Nicole Petilli, with a focus on the mechanical design and the structural and thermal analysis.

While the overall goal of the project is to collect data from the ionosphere with a 250km perigee, there are several subsystems with their own main objectives that work towards the success of the mission. The objectives of this MQP are to perform mechanical design, structural analysis, and orbital analysis for the 6U CubeSat in compliance with NanoRacks deployer requirements. Additionally, thermal cycling effects will be considered for a given loading scenario on the frame of the 6U.

## Background and Literature Review
CubeSats are small, cost effective satellites that expand commercial access to space. Defined by the standardized scale,'U' (a cube with dimensions of 10cm x 10cm x 10cm, ~1.3kg), CubeSats are typically 1U, 2U, 3U, 6U, or 12U[1,11]. Each Unit is comprised of hardware components specifically selected to complete the satellite's mission. The concept began in 2000 as a plan to provide scientific and military laboratories another tool to grow their operations in space. Many colleges and high schools have programs that allow students to design and build their own CubeSats, illustrating that the concept lends itself to valuable educational experience [1 ,7].

In recent years, the CubeSat has become a unique tool in the scientific community. A cooperative culture has formed around the implementation of CubeSats into everyday space science. NASA's CubeSat Launch Initiative (CSLI) provides opportunities to launch small satellites aboard larger launch vehicles as secondary payloads[8]. The industry consists of companies (Clyde Space, ISIS, etc.) providing interested parties with components necessary to construct the satellite, who then work with integration services (NanoRacks, SpaceFlight Services, etc.) to facilitate the satellite's flight aboard a launch vehicle.

CubeSats are currently in an era of rapid growth in popularity and technological opportunity. It is estimated that the global CubeSat market was valued at \$152 million in 2018, and is projected to rise to nearly \$375 million. The coming years expect reduced mission costs, increased opportunity in government, military, and commercial applications, and a greater demand for data from earth observation in LEO. One promising project is SpaceX's Starlink, which aims to provide reliable and affordable broadband internet services around the globe. There are currently about 400 Starlink satellites in orbit, with plans to build the constellation to 12,000 by the completion of the project. At the minimum number of Starlinks necessary for operation, the program could bring its services to all U.S territories in time for 2021 hurricane season.

## Social, Economic, and Educational Considerations
The continual expansion and development of CubeSat opportunities has yielded a variety of positive social, economic and educational effects. Since CubeSats were first theorized and developed by Cal Polytech, many universities and even high schools have started similar micro satellite programs as a result of their affordable cost, in addition to a flux of new commercial companies [1].
The CubeSat industry has already had a large impact on the space industry, which has seen a surge of space start-ups in the last five years. From 2000 to 2014, space startups received a sum total of \$1.1 Billion in venture capital Investments. That number increased exponentially in the following 3 years, with more than 120 investors contributing \$3.9 billion to space start-ups in 2017 ALONE! A majority of these start-ups reflect new technologies and abilities related to CubeSat Components, Development, or Launch and Integration. CubeSats have also created a shift from cost plus to fixed cost payments, which are less risky for the government or investors and maintains performance control of contractors[4, 7].

One of the major benefits of CubeSats is their versatility. CubeSats themselves are a significantly cheaper method to test new technologies so that they can be developed further and flight tested before integration on larger missions. This is because of their low mass and thus low launch cost. Their smaller, modular size also simplifies development and testing, as many subsystem components are available off-the-shelf from different suppliers. Often CubeSat projects can be flight ready within one or two years. Many Universities and Institutions develop CubeSats with a specific payload in mind, including but not limited to remote sensors to communications modules [7, 9].

In recent years, CubeSats have seen expanded use with the ISS as a result of the Japanese Experiment Module Small Satellite Orbital Deployer, and have even flown with missions to the moon and to Mars. The first commercial entity to utilize the ISS as a deployment option was NanoRacks LLC in 2013. With NASA and JAXA, NanoRacks developed and launched a deployer directly tied to the Japanese Experiment Module, allowing CubeSats to be checked by astronauts before deployment. This is a quick and efficient method of launching CubeSats into LEO. Between 2014 and 2017, NanoRacks deployed a total of 176 CubeSats from their deployer on the ISS, and plans to ramp up these numbers in the next few years.
The many advantages and developments of CubeSats has attracted more start up companies in recent years. As of 2018, 51\% of CubeSats are developed by the private sector, showing CubeSats are no longer just for research by institutes and universities. The larger commercial flux has led to many impressive satellite technologies, including the first commercial optical communication downlink system (Analytical Space, Inc.) and the first CubeSat to employ a new hybrid (dual-purpose) antenna and solar power system [6,11].

Technological developments can be found in a variety of topics, including solar and space physics, Earth Science and applications from space, planetary science, astronomy and astrophysics, and biological and physical sciences in space. From 2000 to 2015, the number of publications on CubeSats has risen to 536 publications total outlining the many advances made [6].

Expansion and promotion of CubeSat use has also helped NASA and the ESA collaborate with interested schools and students to further promote the benefits of the space industry. CubeSats inspire students from many levels of education to pursue STEM fields and take part in an innovative and exciting new part of the space sector. These programs teach students about the many components in a CubeSat as well as the many fields of knowledge needed, organizational skills and communication necessary to efficiently design and build a CubeSat [9].

Unfortunately not all CubeSat effects are positive. More traffic complicates flight paths and adds an element of danger to many missions as the risk of collisions increase. Unlike larger satellites, CubeSats are generally designed without collision avoidance capabilities and without a specified deorbiting plan. Being so small makes them difficult to track, adding to the uncertainty of any area containing CubeSats being safe for other spacecraft. Additionally, having such a large number of satellites also increases the risk of collisions and increased debris resulting from those collisions. The European Space Agency has already experienced a collision due to CubeSat debris. Their Sentinel-1A was struck and had its solar panel destroyed by CubeSat related debris and the debris from that collision put their Sentinel-1B spacecraft at risk. A study using NASA’s LEGEND (LEO to-GEO Environment Debris) model, assuming a post mission disposal compliance rate of 90\%, shows that continuing the current increase of CubeSats could result in a 75.3\% increase of collisions in J1, a 342.2\% increase in J2, and an 89.8\% increase in J3. This increase of collisions and debris could make operations in LEO difficult and interrupt human spaceflight, or cause damage to the International Space Station. NASA estimates that of all launches into space, 94\% are now space debris, where 64\% of that are fragments (volume ~100cm2)[5].

There is also a risk of NanoSats being used to gather intelligence from other satellites, disrupting the operations of larger satellites or by spying directly. Their size makes them difficult to detect and prevent these activities. Additionally CubeSats being used for communications could become targets for hackers to cause disruptions. As CubeSats increase in accessibility, more effort and technology will be necessary to keep other satellites and communications secure[5, 10].

Sources:
1: https://www.hindawi.com/journals/ijae/2019/5063145/
2: https://www.nanosats.eu/
3: https://sst-soa.arc.nasa.gov/12-passive-deorbit-systems
4: https://www.marketsandmarkets.com/Market-Reports/cubesat-market-58068326.html
5: https://www.space.com/36506-cubesats-space-junk-apocalypse.html
$6: https://www.nasa.gov/mission_pages/cubesats/overview$
7: Commercial Marketplace
8: ESA2
9: ESA1
10: https://www.cbinsights.com/research/industries-disrupted-satellites/
11: ResearchPaper1

## Previous CubeSat MQPs
Yuh this part sucks

## Payload: mINMS
The payload of this mission is the mini Ion Neutral Mass Spectrometer or mINMS for short. It was developed at NASA’s Goddard Space Flight Center, with two apertures for detecting ions of densities between $10^3-10^8 /cm^3$ and Neutrals of densities between $10^4-10^9/cm^3$,  with very low energies between 0.1eV and 20eV. These apertures must be oriented in the RAM facing direction (the direction of movement), and are capable of making high resolution, in-situ measurements of [H], [He], [O], [N2], [O2] & [H+], [He+], [O+], [N2+], and [O2+]. The instrument occupies nearly 1.5U of volume and has a mass of 560 grams[12]. A picture of the mINMS is shown below (human hand for size comparison):

IMAGE ONE YUH

Since its creation, the mINMS has been used on a few missions, with more planned in the next few years. This includes the ExoCube 3U CubeSat launched in January 2015 on a rideshare mission and the Dellingr 6U launched in August 2017 from the ISS. The petitSat planned to be launched in August 2021 also from the ISS. The ExoCube was designed by Cal Polytech in collaboration with NASA Goddard Institute with the mINMS as its primary payload. It was a 3U CubeSat with mass of 4kg deployed in Low Earth Oribt (LEO). The mission had issues with transmitting power, however it was able to validate that the mINMS was operating. ExoCube, shown below, was in operation for 7 months[13].

IMAGE 2 YUH

Following the ExoCube mission was the Dellingr 6U mission, launched from the ISS NanoRacks Deployer. Despite many issues that arose, the team successfully achieved a resilient mission and provided many lessons and areas of growth for CubeSat missions, eventually inspiring the petitSAT, GTOSat and BurstCube missions. Dellinger was also able to provide clear detection of ionized hydrogen (H+), helium (He+) and oxygen (O+) in the atmosphere in May of 2018, proving Ion detection capabilities. As of October 2018, the team turned on the neutral mode, which is still the primary focus. These measurements are necessary for studies of the dynamic ionosphere-thermosphere-mesosphere system, or simply put to define the steady state background atmospheric conditions[14].

IMAGE 3 YUH

The final mission with the mINMS payload, the Plasma Enhancements in The Ionosphere-Thermosphere Satellite, or petitSat, is planned to be launched in 2021 with a very similar design to that of Dellingr. It is the first to utilize a Dellingr-X frame, designed based on lessons learned from the Dellingr mission, which is more reliable, cheaper, and protects electronics. Additionally, deployable solar arrays and a more advanced star tracker were included to avoid previous issues. The goal of the petitSat mission, run by NASA scientists at the Goddard Space Flight Center, is to determine how perturbations in the density of plasma within the ionosphere, also called “blobs,” distort the transmission of radio raves. These blobs commonly interfere with GPS and radar signals from Earth (get reflected back), and it is theorized that fast-traveling waves coming from the thermosphere may have an effect, as they lead to a phenomenon called Medium Scale Travelling Ionospheric Disturbances. Scientists are trying to determine the relationship between these two phenomena (see how closely related), therefore the INMS will continue to observe density changes in response to daily and seasonal cycles, while a  second instrument measures distribution, motion and velocity of ions[15].

Sources:
12: Mass Spectrometers for CubeSats (provided by Gatsonis as well)
13: Exocube
14: Dellingr and Dellinger 2: Specifics
15: Intro to PetitSat

## Project Goals
The cubesat analysis MQP team, comprised of three separate groups, was tasked with creating a 6U cubesat to gather atmospheric information in an extreme low Earth orbit (eLEO). Not many cube satellite missions orbit this low to the earth, so being able to take atmospheric readings from eLEO utilizing the NASA Goddard mINMS payload will provide researchers with valuable information from a scientifically rich area not frequently explored.

The principal goal of the subsystem design project team, Team 1 (GT), is to maximize the lifespan of the cubesat in LEO. By modifying orbital parameters and maximizing the efficiency of burns, the mission and lifespan of the satellite will be optimized, providing an increased duration of data collection, adding to the value of the mission.

The primary goal of the CubeSat analysis MAD team, Team 2, is to first successfully detumble the 6U CubeSat once it is seperated from its launch vehicle. After detumbling, Team 2 then handles the attitude determination and orbital control of the 6U. Team 2 is also in charge of handling the command and data handling that deals with storing and downloading payload data, along with creating a functioning testbed.

The first goal of the NAD project team, Team 3, is to perform mechanical design of the 6U eLEO cubesat, in accordance with components chosen by the previous SEG Teams, to meet NanoRacks deployer design requirements. The second goal of team 3 is to conduct thermal and structural analysis on the 6U eLEO cubesat to meet respective NanoRacks requirements and ensure all chosen components are suitable for the eLEO mission.

## Project Requirements and Constraints

### Deployer and Launch Options
With the exponential growth of CubeSat missions, one can find multiple deployer options for CubeSats based on size and target orbit. The first deployer ever designed was the P-POD(Poly-Picosatellite Orbital Deployer) by Stanford and CalPoly Tech SLO in the early 2000’s. The P-POD is a standard deployment system developed around the 10 cm x 10 cm x 10 cm initial CubeSat design, with a length of 34 cm to hold up to three 1U CubeSats. The design minimizes interaction with primary payload by enclosing cubesats in a dormant state, and uses a spring and pusher plate to guide CubeSats along interior rails and out the deployer door, as seen below. P-POD’s have a good flight heritage and have been used extensively for over a decade. An updated design can hold a 6U CubeSat in a 2x3U orientation. P-POD’s allow up to 1 kg per Unit, and can eject multiple Cubesats at once per deployer[16].

Illustration of P-POD Double Deployer (2x3U). Credit: CalPoly

ENTER IMAGE 1 YUHH

Since the invention of the P-POD, multiple similar designs have been used to decrease weight or increase the size or mass of payload. This includes Tyvak Launch Systems deployer, which promotes custom manufactured deployment mechanisms with flight tested COTS 3U, 6U, and 12U deployment mechanisms used on rideshare missions, as well as the ISIPOD or ISIS Payload Orbital Dispenser, later upgraded and renamed as Quadpacks. The Quadpacks deployer, developed by Planetary Sciences Corp, provides a variety of opportunities for various CubeSat sizes, ranging from 1U to 16U. The mechanism is very closely related to that used in a P-POD, however the main designs include a 12U dispenser broken into 4 sections of 3U areas (4x3U), or a 16U (4x6U). It offers a very flexible configuration or deployment sequence, with the ability to release many satellites at a time, for example four 3U’s, a combination of 3U’s, 2U’s and 1U’s, or 12 to 16 1U’s in total. Each section of the QuadPack can its door independently of the other 3 doors.

ENTER IMAGE 2 YUH
Quadpacks 12U dispenser for dnepr launch june 2014; Credit: ISIS  [1]

Quad Packs are also extensively used on SHERPA Kick Stage Vehicles (also known as the “space tug”) to move payloads into desired orbits. SHERPA’s can be carried on any EEVL (Evolved Expendable Launch Vehicle) including the Atlas V, Delta IV, and Falcon 9 (all certified), and can hold up to 1500 kg of payload, with 40+ QuadPacks[17].

The final and most recommended deployment mechanism is the NRCSD, or the NanoRacks CubeSat Deployer, located on the Japanses Experiment Module on the ISS. When CubeSat deployment operations begin, the NRCSDs are unpacked, mounted on the JAXA MPEP (Multi-Purpose Experiment Platform) and placed on the JEM airlock slide table for transfer outside the ISS. A crew member operates the JRMS (JEM-Remote Manipulating System) – to grapple and position for deployment. The CubeSats/nanosatellites are deployed when the JAXA ground controllers command a specific NRCSD. The NRCSD Configuration can be seen below:

ENTER IMAGE 3 YUHUHHHER
NRCSD Configuration; Credit: NanoRacks

ENTER IMAGE 4 YER IFRIT FRUIT

Since its first use in July 2014, 200+ payloads have been sent to the ISS and deployed from the NRCSD. The NRCSD is a self-contained system that is still electrically isolated from the ISS to protect the crew. Onboard the ISS, NanoRacks Platforms are installed in EXPRESS Rack inserts to supply power and USB data transfer capability for NanoRacks Modules, allowing CubeSats to conduct experiments on the ISS and be checked by astronauts. The NRCSD is able to launch CubeSats with a maximum length of 50 cm[18].

As noted above, a 6U CubeSat is a major limiting factor, such that only a few of deployers are designed for such a mission, including the NanoRacks and Quadpacks deployers. Based on these two options, there are certain launch opportunities able to carry a 6U cubesat.

To be deployed by NanoRacks, CubeSats must first reach the ISS by a Cygnus or Dragon Spacecraft. This of course limits CubeSats to a 51.64 degrees inclination and requires additional compliance with ISS safety regulations. Additionally, using any of the EELV’s, a SHERPA Spacecraft carrying our 6U CubeSat in a 16U Quadpack could deploy the satellite at a variety of orbits. The major disadvantage is waiting for the next SHERPA launch opportunity however.

Lastly, in certain cases, Antares and Falcon 9 rockets (not going to the ISS) will have additional space and can carry pico or micro satellites. This is known as a rideshare opportunity. Antares launches are currently limited to P-POD configurations, while the SpaceX rideshare website provides little information on the deployment options or capabilities, which is why the teams recommends to launch with a Falcon 9 carrying CubeSats in a Dragon Module to the ISS, and deploy from a NanoRacks deployer on the Japanese Experiment Module (JEM). There are many more opportunities as Dragon Spacecraft frequently visit the ISS[19].

16:https://directory.eoportal.org/web/eoportal/satellite-missions/c-missions/cubesat-concept
17: https://ieeexplore.ieee.org/document/7118918
18: https://ieeexplore.ieee.org/document/7118918
19:https://www.spacex.com/smallsat

### Launch Vehicle
The SEG team decided that the best deployer option would the NanoRacks deployer, requiring a Falcon 9 to carry the cubesat to the ISS in a Dragon Module. This was the best option as NanoRacks is one of the most flight tested deployers compatible with a 6U CubeSat, has a strong mission success flight record, and allows multiple launch options for our CubeSat to ride share to the ISS[20].

The NanoRacks requirements are broken up into various sections, some requiring further collaboration with the other two SEG teams. The topics are listed and further explained below:
- 10 Rail or 10 Tab Requirements
    -The first ten requirements all relate to the dimensions, placement, and material properties of the rails or tabs along which the CubeSat will be deployed from the NRCSD. It is the full SEG’s choice to decide between the rail or tab configuration. Both have similar requirements that are the responsibility of the design team, who create a final CAD model of the whole structure including the rails or tabs.
- 2 Major Design Requirements (Mass and COM)
    -Two major design requirements that are affected by all teams decisions are the total mass and center of geometry distance from center of mass along each axis. It is the design teams responsibility to ensure these requirements are met by adding components carefully, considering specifications provided by the other teams.
-12 Deployment Requirements (Switches, locations, deployables)
    -The deployment requirements denote the locations and directions for a variety of possible deployment switches, in accordance with the minimum of 3 deployment switches corresponding to independent electrical inhibits on the main power system. The 6U system must also consider deployment velocity and tip off rate.
-3 RBF/ABF and Electrical Switch Requirements
    -Remove Before Flight and Apply Before Flight features are necessary design considerations when utilizing NanoRacks Deployer to ensure the safety of the ISS and its habitants. The design team in collaboration with the command team must ensure that these features are included and that an access panel is placed on the +Y face for physical accessibility.
-10 Structural and Environment Requirements (7 Considered)
    -The structural analysis team must ensure the CubeSat is capable of withstanding the random vibration environment during launch though a vibration test report, as well as integrated loads of 1200N across all load points in the Z-direction and depressurization/ vacuum conditions. This will require a Safety Data Template.
    -Should the CubeSat contain any detachable parts, additional coordination with NanoRacks is required.
-14  Electronics Requirements (battery, capacitors and wiring)
    -An electric schematic and battery test report ensures that the battery and its charging methods are safe and that all wiring and circuitry is protected. This will require coordination between the design and power team.
-3 Material Based Requirements (outgassing, hazardous materials)
    -In addition to a materials list for the rails, the design team must provide a bill of materials for the entire CubeSat to ensure all materials are resistant to stress corrosion, comply with NASA guidelines from hazardous materials as well as outgassing regulations. Total Mass Loss (TML) must be less than 1%, and Collected Volatile Condensable Material (CVCM) less than 0.1%.
-3 Orbital Requirements (debris, re-entry)
    -Lastly, the thermal and orbital analysis teams are responsible for creating an Orbital Debris Assessment Requirements (ODAR) report should the CubeSat exceed 5 kg, or if it is determined that the cubesat will survive re-entry.

As can be seen above, meeting all chosen NanoRacks requirements will require the coordination of members from each team, in addition to continual updates to our important documents ensuring all notable components are safe and the best pick for this mission.

20: Nano-Racks Double Deployer

### Power Subsystem Requirements
The selected solar panels are the Photon 3U body mounted panels, manufactured by Clyde Space. These panels utilize Spectrolab XTJ Prime solar cells with a BOL efficiency of 30.7\% and EOL efficiency of 26.7\%. The Photon 3U panels have a standard operating temperature range of -40C to 80C, though they are advertised to have available testing for different ranges. Our 6U CubeSat will require 4 of these body mounted panels, one on each long side.

The chosen battery is the Optimus-40, manufactured by Clyde Space. The Optimus-40 has dimensions of 95.89 mm by 90.17 mm by 27.35 mm and a full discharge voltage of 6.2  V. The battery’s operating temperature is -10 C to 50 C. Our 6U CubeSat will require a single battery placed within the CubeSat.

The selected EPS is the Starbuck Nano-Plus, manufactured by Clyde Space. The EPS has dimensions of 95.89 mm by 90.17 mm by 20.82 mm and has PDMs with 10 latching current limiters. The Starbuck Nano-Plus has an operating temperature is -40 C to 85 C. The EPS can be placed anywhere inside the CubeSat.

### ADC Subsystem Requirements
The selected magnetorquer is the NCTR-M002, manufactured by New Space. The NCTR-M002 requires less than 200 mW of power, each, to operate and its dimensions are 70 mm by a diameter of 10 mm. The NCTR-M002 can operate within a range of -20 C to 60 C. Our 6U CubeSat will require three of the NCTR-M002 magnetorquers, one for each axis. There is no specific location the magnetorquers must be located in.

The selected reaction wheel is the RWP050, manufactured by Blue Canyon Technologies. The RWP050 requires less than 1 Watt of power, each, to operate and its dimensions are 58 mm by 58 mm by 25mm. The RWP050’s operating temperature was not specified within Blue Canyon’s provided spec sheets. Like the magnetorquers, our 6U CubeSat will require three reaction wheels, one for each axis, and can be placed wherever they fit within the CubeSat.

The selected GPS is the NGPS-01-422, manufactured by New Space. The NGPS-01-422 can operate within a range or -10 $^{\circ}$ C to  50 $^{\circ}$ C while consuming less than 1.0 W of power. The GPS can be placed anywhere on the CubeSat and has the dimensions of 155 mm by 76 mm by 34 mm with an antenna that has the dimensions of 54 mm x 54 mm x 14.1 mm.

The ADC decided on a component that combines the accelerometer and magnetometer into one piece of hardware. The combined accelerometer and magnetometer is the LSM303AGR that has the dimensions of 2 mm by 2 mm by 1 mm and consumes 1130 $\mu$A while operating and 2 $\mu$A while idle. The LSM303AGR can operate within a temperature range of -40 $^{\circ}$ C to 85 $^{\circ}$ C. This component must be located on the center of mass of the 6U CubeSat to function effectively.

The selected gyroscope is the EVAL-ADXRS453, manufactured by Analog Devices. The single gyroscope will be aligned in the center of all three primary axes. The EVAL-ADXRS453 consumes 0.0189 W, can operate at a temperature between -40 $^{\circ}$ C and 105 $^{\circ}$ C and its dimensions are 33 mm by 33mm by 3 mm.

The final component of the ADC subsystem are the fine sun sensors. The sun sensor chosen is the Nano-SSOC-A60, manufactured by New Space. Our 6U CubeSat will incorporate five fine sun sensors, located in all four corners with one attached to the front of the payload. To work effectively, the fine sun sensors should be located 90 degrees apart for best coverage. The sun sensors require $< 2$ mA of power to operate and its dimensions are 27.4 mm by 14 mm by 5.9 mm. The Nano-SSOC-A60 can operate within a temperature range of -30 $^{\circ}$ C to 85 $^{\circ}$ C.

### Structural Design Requirements
The following section outlines the various requirements and constraints relevant to each subsystem. Often these parameters require the teams to coordinate with one another to ensure all components and aspects of the mission will operate as planned. The requirements and constraints of the design team’s subsystems will be discussed first.

The design of the 6U itself has specific Center of Mass and rail requirements according to NanoRacks Double Deployer specifications, which can be found in Appendix. All components provided by the other teams (excluding solar panels) must fit within the walls of the 6U sized structure, which is in a 30 cm x 20 cm x 10 cm configuration (height, length, width).  On top of that, the components must be situated in such a way that the center of mass is as close as possible to the center of geometry to minimize the necessary attitude control computation time/ complexity. Additional Center of Mass requirements are provided by NanoRacks, as seen in the table below:

| Axis | Distance to Geometric Center |
|:----:|:----------------------------:|
| X-Axis | 5 cm |
| Y-Axis | 3 cm |
| Z-Axis | 8 cm |

Table: Center of Gravity Requirements {#tbl:cg-requirements}

Location requirements are provided by the other teams; examples include the accelerometer, which must be at the Center of Mass, the reaction wheels along each axis intersecting with the COM, and the payload which has its aperture in the RAM facing direction (opposite side of the engines). The design team must also create rails or tabs for proper ejection from the deployer. A visual of the rail design and load regions can be found below:

IMAGE OF RAILS BUT IT ISN'T INCLUDED BECAUSE ITS A SCREENSHOT OF GOOGLE DRIVE INSTEAD

Design of the 6U CubeSat must additionally comply with the following constraints.  The most important constraint provided is limiting the mass of the total 6U cubesat to a maximum of 12 kg. The design team must ensure that no hazardous materials are used according to section BLANK of the Deployer requirements, and must ensure Total Mass Loss (TML) is less than 1%, and Collected Volatile Condensable Material (CVCM) less than 0.1% due to outgassing.

The structural analysis team must ensure that the final design of the CubeSat is strong enough to withstand all conditions of the mission.  NASA requires that all vehicles entering space pass the ground structural tests outlined in the general environmental verification standard (GEVS).  For our project we followed the requirements for structural tests from nanoracks (sec. 1.3.2) because they are more relevant to CubeSats specifically and still comply with the GEVS requirements.  Nanoracks requires the vehicle to survive a random vibration test, a structural load test for 1200N in the z-axis direction, an airlock depressurization test, and must ensure no detachable parts come loose.

The thermal analysis team is responsible for ensuring that all selected components discussed above will operate within their allowable temperature ranges throughout the duration of the mission. It must also be ensured that any heat that these components produce within the internal CubeSat structure is negligible in order to maintain allowable internal temperature profiles.

## Overall Project Management
As previously mentioned, the Systems Engineering Team (SEG) is divided up into three teams by closely related subsystems and the relevant analysis required for each. Students and Professors are split among these teams based on the amount of responsibility given to each team. The figure below simplifies how the teams are divided and the major roles of each team:

ENTER IMAGE OF CRAPPY SEG GROUP BREAKDOWN YUR BUT YUH
FIGURE N: SEG GROUP STRUCTURE

Sub-Groups communicate via Slack and have a shared google drive for component lists and other necessary information that needs to be shared with the whole team. Proprietary documents, and CAD drawings were not share on the google drive. Sensitive documents deemed able to share with the whole group were stored on a secure OneDrive folder, while those deemed only sharable with a specific sub-group, were stored on one individual flash drive, to ensure that no confidential knowledge got leaked.

In addition to the above sub-team designated responsibilities, team members presented twice a week, first all together to their advisors on technical advances and weekly progress, then on a rotating schedule for the full SEG meetings. As team lead, Brian focused on managing team tasks and coordinating with the other teams more often then designing CAD models and assemblies. This included taking notes during meetings, tracked tasks based on a year long timeline for the three subsections, and coordinated with the other SEG teams to complete the Component List. During this coordination he contributed to research and writing on launch options, deployment options and requirements, and payload specifications. As the Design lead, Nicole was responsible for the flash drive with all sensitive documents, and ensuring these documents were safely shared when needed by other members for simulations or the final design assembly.

## MQP Objectives, Methos, and Standards

## Team 1
Power Objectives:
The power subsystem for this project is responsible for supplying the power that is generated, stored, and distributed throughout the CubeSat. Many of the CubeSat’s components will require continuous power draw in order for the CubeSat to remain functional during its lifespan. To account for this, a power budget was created considering all of the power consuming hardware that will be implemented into the design. To ensure proper power delivery, hardware power requirements and their operational priority and duration were taken into consideration. A power budget timeline of hardware was created to help illustrate and analyze the overall power consumption of the CubeSat. The timeline showed what hardware should be turned on and off throughout the mission for each orbit.

Propulsion Objectives:
The propulsion subsystem has two objectives in order to complete the goal of our CubeSat. The first objective is to determine the number of thrusters required to keep the CubeSat in orbit. It has been decided that the Busek electrospray thruster BET-300P will be the thruster used in the CubeSat. The second objective of the propulsion subsystem is to determine the burn time to optimize the lifespan of the CubeSat in orbit.

TeleCommunications Objectives:
The Telecommunications sub-system has three primary objectives. The first is to select  hardware that meets requirements posed by other subsystems, such as power usage and structural placement. The second is to identify a viable Ground Station Network (GSN) that will allow the CubeSat to transmit data at an acceptable daily rate. The ground station sites should be inside the satellite's coverage, given the orbital inclination, and also able to transmit and receive in X-band frequencies. The third is to establish the uplink and downlink budgets, for use by the payload and data handling instruments.

Environmental Effects Objectives
Understanding how the CubeSat will behave in the space environment is key to the success of the mission. In our team’s desired orbit there is atmospheric drag, solar radiation pressure and free electrons. This makes up the thermosphere and ionosphere. The environment will affect the structure of the CubeSat, along with the telecommunications, propulsion, and detumbling and control systems. In order to accurately predict the functional lifetime and success of the CubeSat, it must be designed with consideration to every environmental hazard. In addition, the temperature fluctuations must be taken into consideration as the thermosphere can range from 500 °C to 2000 °C. This is due to the ionosphere harboring extremely charged electrons to make a plasma environment. However, the atmospheric density is quite low, thus the ambient temperature would feel cold to the human skin.

Payload Objectives
The payload subsystem is the driving parameter behind the entire mission. At this current time, the team does not have the exact payload specifications beyond knowing it is approximately 1U, does not require specific pointing parameters, and collects data within the ionosphere. Once the technical parameters such as power draw, mass, and data transmission speed are determined, the team will properly interface the payload with the CubeSat and determine the majority of the design parameters and constraints for the mission.

Orbital Objectives
This MQP team is responsible for determining the orbital parameters for the mission. Utilizing Systems Toolkit (STK), the team will determine the propellant needed to transfer from initial orbit to the desired elliptical orbit, then maintain that orbit for as long as possible with the provided propulsion system. Other orbital parameters to analyze include the solar flux upon the spacecraft to determine power draw, the drag profile on the spacecraft, and environmental effects of eLEO such as electron flux.

For the ADC portion of the project, the first objective is to define the control modes, such as detumbling, based on system-level requirements. The next objective is to quantify the disturbance torques based on the mission profile. Once that is completed, the objective is to select spacecraft control methods (e.g. b-dot) for each control mode (e.g. detumble) based on mission requirements and constraints. Then, the objective is to select and size sensors and actuators in order to determine attitude and control spacecraft for each control method. The next objective is to define attitude determination algorithms and control algorithms based on capabilities, requirements, and constraints.
For the ODC portion of the project, the first objective is to define control modes based on system-level and mission requirements. The next objective is to quantify drag based on the mission profile. Once that is completed, the objective is to select orbit control methods (e.g. low impulse maneuver) for each control mode (e.g. apogee raising) based on mission requirements and constraints. Then, the objective is to select and size sensors (GPS) and actuators (thrusters) in order to determine orbital parameters and to control the spacecraft for each control method. The next objective is to define orbital determination algorithms and control algorithms based on capabilities, requirements, and constraints.
For the CDH portion of the project, the first objective is to define mission phases(e.g. pre-deployment, deployment, data acquisition) based on system-level requirements and mission profile. The next objective is to quantify data requirements. Once that is completed, the objective is to select data handling and spacecraft command methods for each mission phase (e.g. apogee raising) based on mission requirements and constraints. Then, the objective is to select computer components in order to handle data and command spacecraft. The next objective is to define data handling and transmission algorithms based on capabilities, requirements, and constraints. During the entire project and for all portions, iterating steps as necessary to achieve goals and document steps are objectives as well.
For the test bed portion of the project, the objective is to construct a 3 DOF lab testbed for the test and validation of ADCS control systems. Once the physical testbed is completed the next objective is to create an electronic control system for the remote control and analysis of the system.

## Tasks and Timetable