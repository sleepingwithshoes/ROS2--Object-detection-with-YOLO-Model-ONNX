ARG ROS_DISTRO=jazzy
ARG TARGETARCH
# Set the ROS image variant to use (e.g., ros-base, desktop, etc.)
ARG ROS_IMAGE=desktop


# Base image with ROS packages
FROM osrf/ros:${ROS_DISTRO}-${ROS_IMAGE} 

# Install additional dependencies
COPY docker/dependencies.list /tmp/dependencies.list
RUN apt-get update && \
    sed "s/\${ROS_DISTRO}/${ROS_DISTRO}/g" /tmp/dependencies.list | xargs apt-get install -y && \
    rm -rf /var/lib/apt/lists/*
RUN rm /tmp/dependencies.list

ARG ORT_VERSION=1.23.1
# Download and install ONNX Runtime C++ library for the appropriate architecture
# (aarch64 for Raspberry Pi and ARM64 devices, x64 for x86_64 devices)
RUN apt-get update && apt-get install -y wget unzip && \
    if [ "$TARGETARCH" = "arm64" ]; then \
    ONNX_ARCH="aarch64"; \
    else \
    ONNX_ARCH="x64"; \
    fi && \
    wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-${ONNX_ARCH}-${ORT_VERSION}.tgz && \
    tar -xzf onnxruntime-linux-${ONNX_ARCH}-${ORT_VERSION}.tgz && \
    cp -r onnxruntime-linux-${ONNX_ARCH}-${ORT_VERSION}/include/* /usr/local/include/ && \
    cp -r onnxruntime-linux-${ONNX_ARCH}-${ORT_VERSION}/lib/* /usr/local/lib/ && \
    ldconfig && \
    rm -rf onnxruntime-linux-${ONNX_ARCH}-${ORT_VERSION}* && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure workspace
WORKDIR /workspace

# Install Python dependencies
COPY docker/requirements.txt /tmp/requirements.txt
# Install other dependencies
RUN pip install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
RUN rm /tmp/requirements.txt

# Setup entrypoint
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Create a non-root user to match host UID/GID
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Check and modify existing ROS user (UID 1000)
RUN if getent passwd 1000 >/dev/null; then \
    existing_user=$(getent passwd 1000 | cut -d: -f1); \
    usermod --uid 1001 $existing_user; \
    groupmod --gid 1001 $existing_user; \
    find / -xdev -user 1000 -exec chown -h 1001 {} \; 2>/dev/null || true; \
    find / -xdev -group 1000 -exec chgrp -h 1001 {} \; 2>/dev/null || true; \
    fi

# Create new group/user with host's UID/GID
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && (groupadd --system users || true) \
    && usermod -aG users ${USERNAME} || true

# Update environment for new user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

# Install Zsh and dependencies
RUN apt-get update && apt-get install -y zsh curl git

# Set Zsh as default shell for the user
RUN chsh -s $(which zsh) ${USERNAME}

# Add user to dialout group for serial port access
RUN usermod -aG dialout ${USERNAME}

# Add user to video group for camera access
RUN usermod -aG video ${USERNAME}

# Install Oh My Zsh and plugins as the user
USER ${USERNAME}

RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# Configure .zshrc
RUN sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' ~/.zshrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.zsh" >> ~/.zshrc
RUN echo '\n\
    if [ -f /workspace/install/setup.zsh ]; then\n\
    source /workspace/install/setup.zsh\n\
    fi\n\
    \n\
    eval "$(register-python-argcomplete ros2)"\n\
    eval "$(register-python-argcomplete colcon)"\n\
    ' >> ~/.zshrc

ENTRYPOINT ["/entrypoint.sh"]