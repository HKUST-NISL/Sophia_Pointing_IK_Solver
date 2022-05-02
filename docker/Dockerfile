# syntax=docker/dockerfile:1
FROM moveit/moveit:melodic-release
USER root

#Install dependencies using the pakcage manager
RUN apt-get update &&\
    #Dependencies of ik plugin
    apt-get install -y libblas-dev liblapack-dev python3-catkin-tools;\
    #Setup git lfs
    apt-get install -y curl && \ 
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt-get install -y git-lfs; \
    #We use a slightly modified moveit and therefore the moveit package comes along with the image is removed
    apt-get remove -y ros-melodic-moveit-core \
    ros-melodic-moveit-msgs       \
    ros-melodic-geometric-shapes  \
    ros-melodic-srdfdom; 

#Install hr_msgs from package files in the build context
ADD ./head-melodic-hr_msgs_0.3.21_amd64.deb /home/
RUN cd /home/ && dpkg -i /home/head-melodic-hr_msgs_0.3.21_amd64.deb;

#FCL needs source build
RUN cd /home/ && git clone https://github.com/flexible-collision-library/fcl.git && \
    cd /home/fcl/ && mkdir build && cd build && cmake .. && make install;

#Pull and build the solver
RUN cd /home/ && mkdir catkin_ws && cd catkin_ws && \
    git clone https://github.com/HKUST-NISL/Sophia_Pointing_IK_Solver.git && \
    mv Sophia_Pointing_IK_Solver src;
RUN apt-get install -y libpcl-dev
RUN cd /home/catkin_ws/ && catkin config --extend /opt/ros/melodic && catkin build;

