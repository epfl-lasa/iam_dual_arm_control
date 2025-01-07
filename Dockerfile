FROM iiwa_toolkit_ns:latest AS base

USER root
# Install catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-noetic-tf-conversions


# Install required libraries

# qpoases
WORKDIR /source
RUN git clone https://github.com/coin-or/qpOASES.git
RUN cd qpOASES && git checkout 67ae315f9033cc1d50f39bf22a913996c4eb52fd
RUN cd qpOASES && mkdir build && cd build && cmake ../ && make && sudo make install
RUN cd qpOASES && cd build && cmake -DBUILD_SHARED_LIBS=ON ../ && make && sudo make install

# # sg_differentiation
# WORKDIR /source
# RUN git clone -b cmake https://github.com/epfl-lasa/sg_differentiation.git
# # Fix error "Could not find GTest"
# # RUN sudo apt-get install libgtest-dev && sudo apt-get install cmake && cd /usr/src/gtest && sudo cmake CMakeLists.txt && sudo make && sudo cp *.a /usr/lib
# RUN cd sg_differentiation && mkdir build && cd build && cmake .. && make && sudo make install

USER ${USER}
WORKDIR ${HOME}/ros_ws/src

# sg_differentiation
WORKDIR ${HOME}/ros_ws/src
RUN git clone https://github.com/epfl-lasa/sg_differentiation.git

# dual_pre_grabbing
# Need to be root to clone private repo
USER root
WORKDIR ${HOME}/ros_ws/src
RUN --mount=type=ssh git clone -b feat/realrobots git@github.com:epfl-lasa/dual_pre_grabbing.git
USER ${USER}

# Copy iam_dual_arm_control folder inside docker
WORKDIR ${HOME}/ros_ws/src
COPY ./ ./iam_dual_arm_control

# Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y
# RUN source ${HOME}/.bashrc && source /opt/ros/noetic/setup.bash && catkin build

# Add the workspace to the bashrc
USER root
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${HOME}/ros_ws/devel/setup.bash" >> ~/.bashrc

USER ${USER}
CMD [ "bash" ]