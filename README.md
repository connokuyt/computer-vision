# Mixed Kinematics and Camera Based Vehicle Dynamic Sideslip Estimation for an RC Scaled Model

_This paper, written by Conno Kuyt and Matteo Corno, has been published and presentaed at the 2018 IEEE Conference on Control Technology and Applications (CCTA) Copenhagen, Denmark, August 21-24, 2018._

https://ieeexplore.ieee.org/abstract/document/8511487

## Abstract

Vehicle side slip angle is at the basis of many vehicle dynamics control systems. Many methods are available to estimate side-slip angle using on board sensors (usually accelerometers and gyros). The technical advances pertaining autonomous vehicles made an additional kind of sensor available: cameras. This study develops a mixed kinematic vision-based side slip angle estimation. The proposed algorithm merges the information of a commercial grade inertial measurement system, wheel encoders and information from a camera. The camera measurement are integrated in a Kalman filter observer. The paper implements and tests the approach on an instrumented RC scale vehicle, comparing the proposed approach against a kinematic based estimation. Experimental results show a decrease of a factor between 2 and 10 (depending on the type of maneuver) of the estimation mean squared error.