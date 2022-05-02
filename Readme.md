#Sophia_Pointing_IK_Solver
#Purpose: This is an IK solver for the Sophia robot from Hanson Robotics. Given a point of interest in space, the solver tries to find a reasonable arm and hand configuration that makes sophia looks like pointing at the given point. The solver follows a largely brute-force procedure, discretizing a large portion of the configuration space of Sophia's 7DoF arm.

#Installation: Source build or build a docker image using the Dockerfile provided in the "dokcer" folder. Note that the hr_msgs package should be provided to the build context of the Dockerfile.

#Test: run "pointing_ik_launch.bash" to see a demo. For docker image gui needs to be enabled.