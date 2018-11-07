# imagine_ades
All packages and code related to ADES framework, corresponding learning tools and also ADES databases

- uibk_ades: the ades library implementing the ADES data structure and the database software
- uibk_libades_ros: the ROS wrapper providing service/topic interface to the ADES database
- uibk_ades_db: a continuously evolving database of ADES (check release for fixed stable ADES set)

## Installing
First step is to build the ades C++ library: go to uibk_ades, and follow the Readme instructions (create a build folder, run cmake, make, then make install).
Then run catkin_make at the root of the catkin workspace.
