ARG ROS_DISTRO=dashing

FROM ros:$ROS_DISTRO
ARG CODENAME=bionic

# Disable non-free repositories
RUN if [ "$(uname -m)" = "x86_64" ]; then \
        echo "\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME} main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-backports main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-updates main universe\n\
deb http://security.ubuntu.com/ubuntu/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    else \
        echo "\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME} main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-backports main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-updates main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    fi

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        gettext-base \
        locales \
        keyboard-configuration && \
    rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN echo deb https://deb.nodesource.com/node_10.x ${CODENAME} main | tee /etc/apt/sources.list.d/nodesource.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 1655A0AB68576280

COPY ros-deps /tmp/
RUN apt-get update && \
    apt-get install -y \
      python3-vcstool \
      ros-$ROS_DISTRO-cyclonedds \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    /bin/bash /tmp/ros-deps && \
    rm -rf /var/lib/apt/lists/* /tmp/ros-deps

COPY apt-packages /tmp/
RUN apt-get update && \
    apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

RUN if [ "$ROS_DISTRO" = "foxy" ] || [ "$ROS_DISTRO" = "galactic" ]; then \
        apt-get update && \
        apt-get install -y ros-$ROS_DISTRO-lgsvl-bridge && \
        rm -rf /var/lib/apt/lists/*; \
    fi

RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers


COPY pip3-packages /tmp/
RUN pip3 install -U \
        $(cut -d# -f1 </tmp/pip3-packages) \
    && rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages


RUN git clone https://github.com/rigtorp/udpreplay && mkdir -p udpreplay/build \
      && cd udpreplay/build && cmake .. && make && make install \
      && cd - && rm -rf udpreplay/


COPY bashrc-git-prompt /
RUN cat /bashrc-git-prompt >> /etc/skel/.bashrc && \
    rm /bashrc-git-prompt
COPY gdbinit /etc/gdb/

# Install a browser. Needed to configure SVL simulator
RUN if [ "$(uname -m)" = "x86_64" ]; then \
        curl -LO https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb && \
        apt-get install -y ./google-chrome-stable_current_amd64.deb && \
        rm google-chrome-stable_current_amd64.deb; \
    fi 


# ===================== CLEAN UP ZONE ===================== #
# Commands in the cleanup zone will be cleaned up before every release
# and put into the correct place.
RUN apt-get update \
  && apt-get install -y \
    ca-certificates \
    libgl1 \
    libgtk2.0-0 \
    libvulkan1 \
    libx11-6 \
    libxau6 \
    libxcb1 \
    libxdmcp6 \
    libxext6 \
    nodejs \
    unzip \
    vulkan-utils \
  && apt-get clean

ADD "https://gitlab.com/nvidia/container-images/vulkan/raw/master/nvidia_icd.json" /etc/vulkan/icd.d/nvidia_icd.json
RUN chmod 644 /etc/vulkan/icd.d/nvidia_icd.json
ADD "https://gitlab.com/nvidia/container-images/opengl/raw/ubuntu20.04/glvnd/runtime/10_nvidia.json" /etc/glvnd/egl_vendor.d/10_nvidia.json
RUN chmod 644 /etc/glvnd/egl_vendor.d/10_nvidia.json

# ===================== END OF CLEAN UP ZONE ===================== #

# Do full package upgrade as last step
# to avoid disrupting layer caching
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    rm -rf /var/lib/apt/lists/*

COPY env.sh /etc/profile.d/ade_env.sh
COPY gitconfig /etc/gitconfig
COPY entrypoint /ade_entrypoint
COPY colcon-defaults.yaml /usr/local/etc/colcon-defaults.yaml
RUN echo "export COLCON_DEFAULTS_FILE=/usr/local/etc/colcon-defaults.yaml" >> \
    /etc/skel/.bashrc
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
