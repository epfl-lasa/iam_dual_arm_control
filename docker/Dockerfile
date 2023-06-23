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

# sg_differentiation
WORKDIR /home/${USER}/ros_ws/src
RUN git clone https://github.com/epfl-lasa/sg_differentiation.git

# iiwa_sim_models_poses
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/iiwa_sim_models_poses.git

#sim_objects_description
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/sim_objects_description.git

#dual_iiwa_toolkit
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/dual_iiwa_toolkit.git

#iiwa_toolkit_ns
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/iiwa_toolkit_ns.git

# #dual_gen_torque_controller
# WORKDIR /home/${USER}/ros_ws/src
# RUN --mount=type=ssh git clone -b devel git@github.com:epfl-lasa/dual_gen_torque_controller.git

USER ${USER}

# Copy i_am_project folder inside docker
WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ ./src/iam_dual_arm_control

ENV PYTHONPATH "${PYTHONPATH}:/opt/openrobots/lib/python3.8/site-packages/"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# ./docker/build-server.sh 
# aica-docker interactive dual_arm_control_iam:melodic -u ros --net host --no-hostname --privileged --gpus all -v /home/elise/Documents/IAM/michael/iam_dual_arm_control:/home/ros/ros_ws/src/iam_dual_arm_control


