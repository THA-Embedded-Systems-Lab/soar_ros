FROM ros:jazzy
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

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
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y python3-pip python3-venv
ENV SHELL=/bin/bash

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

RUN apt-get install -y \
    clang-format \
    htop \
    valgrind \
    kcachegrind \
    dbus-x11 \
    wget \
    unzip \ 
    default-jre

# Install Soar for the debugger only 
# The soar library is not on PATH, thus will not be used. Therefore, we can
# build Soar with the CMAKEBUILDTYPE of the workspace for better debugging.
ARG SOAR_VERSION=9.6.4
# SOAR Setup
WORKDIR /opt/soar
RUN wget https://github.com/SoarGroup/Soar/releases/download/releases%2F${SOAR_VERSION}/SoarSuite_${SOAR_VERSION}-Multiplatform.zip \
    && unzip SoarSuite_${SOAR_VERSION}-Multiplatform.zip

ENV SOAR_DIR=/opt/soar/SoarSuite_${SOAR_VERSION}-Multiplatform
RUN /opt/soar/SoarSuite_${SOAR_VERSION}-Multiplatform/SoarJavaDebugger.sh
WORKDIR /opt/soar/
RUN rm -rf **/*.zip
ENV SOAR_HOME=/opt/soar/SoarSuite_${SOAR_VERSION}-Multiplatform/bin/

RUN echo 'alias sbin="cd ${SOAR_DIR}/"' >> /home/${USERNAME}/.bashrc
RUN echo 'alias sjd="${SOAR_DIR}/SoarJavaDebugger.sh"' >> /home/${USERNAME}/.bashrc
RUN echo 'alias svis="${SOAR_DIR}/VisualSoar.sh"' >> /home/${USERNAME}/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
