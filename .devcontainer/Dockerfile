FROM ros:jazzy
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV ROS2_DISTRO=jazzy

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS2_DISTRO}-example-interfaces \
    pre-commit

ENV SHELL=/bin/bash

RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc

RUN mkdir -p /home/ws && chown -R ${USERNAME}:${USERNAME} /home/ws

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
