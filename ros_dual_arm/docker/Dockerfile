ARG BASE_IMAGE_TAG=noetic
FROM epfl-lasa/iiwa_ros_mb_devel2:melodic

# Modify files inside docker
ARG HOST_GID=1001
ENV USER_GROUP=${USER}
USER root
RUN if [ "${HOST_GID}" != "1000" ];\
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Add ros packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-melodic-tf-conversions \
    ros-melodic-vrpn-client-ros \
    && sudo apt-get upgrade -y && sudo apt-get clean

# # Pinocchio
# RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
# RUN  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
# RUN  sudo apt update
# RUN  sudo apt install -qqy robotpkg-py3*-pinocchio

# # Install control libraire from epfl-lasa (will also install Pinocchio)
# USER root
# WORKDIR /home/${USER}
# RUN git clone https://github.com/epfl-lasa/control-libraries.git --branch v6.0.0
# WORKDIR /home/${USER}/control-libraries/source
# RUN sudo bash install.sh -y

# Need to be root to clone private repo
USER root


# iiwa_sim_models_poses
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/iiwa_sim_models_poses.git

#sim_objects_description
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/sim_objects_description.git

#dual_iiwa_toolkit - simulation control
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/dual_iiwa_toolkit.git

# #iiwa_toolkit_ns - for real robot control
# WORKDIR /home/${USER}/ros_ws/src
# RUN --mount=type=ssh git clone git@github.com:epfl-lasa/iiwa_toolkit_ns.git

# #dual_gen_torque_controller
# WORKDIR /home/${USER}/ros_ws/src
# RUN --mount=type=ssh git clone -b devel git@github.com:epfl-lasa/dual_gen_torque_controller.git


# sg_differentiation
WORKDIR /source
RUN git clone -b cmake https://github.com/epfl-lasa/sg_differentiation.git
# Fix error "Could not find GTest"
RUN sudo apt-get install libgtest-dev && sudo apt-get install cmake && cd /usr/src/gtest && sudo cmake CMakeLists.txt && sudo make && sudo cp *.a /usr/lib
RUN cd sg_differentiation && mkdir build && cd build && cmake .. && make && sudo make install

#dual arm control
WORKDIR /source
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/iam_dual_arm_control.git
RUN cd iam_dual_arm_control/dual_arm_control && mkdir build 
WORKDIR /source/iam_dual_arm_control/dual_arm_control/build 
RUN cmake ..
RUN sudo make install

USER ${USER}

# Needed if pip is needed inside docker
RUN pip install --upgrade pip --no-cache-dir
RUN pip install -U pip 

# Copy ros_dual_arm folder inside docker
WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ ./src/ros_dual_arm

ENV PYTHONPATH "${PYTHONPATH}:/opt/openrobots/lib/python3.8/site-packages/"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# ./docker/build-server.sh 
# aica-docker interactive epfl-lasa/ros_dual_arm_control_iam:melodic -u ros --net host --no-hostname --privileged --gpus all -v /path_to_folder/ros_dual_arm:/home/ros/ros_ws/src/ros_dual_arm


