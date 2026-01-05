# Debian Bookworm with OV2SLAM dependencies and development tools
FROM debian:bookworm

# Install basic system tools
RUN apt-get update && \
    apt-get install -y \
        curl \
        gnupg \
        sudo \
        bash \
        git \
        vim \
        nano \
        wget \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# Install Node.js 22 (NodeSource repository)
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get install -y nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Python 3 and pip
RUN apt-get update && \
    apt-get install -y \
        python3 \
        python3-pip \
        python3-venv \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# === OV2SLAM DEPENDENCIES ===

# Install Eigen3 (linear algebra library)
RUN apt-get update && \
    apt-get install -y \
        libeigen3-dev \
        libeigen3-doc \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# Install OpenCV 4 with contrib modules
RUN apt-get update && \
    apt-get install -y \
        libopencv-dev \
        libopencv-contrib-dev \
        libopencv-calib3d-dev \
        libopencv-features2d-dev \
        libopencv-imgproc-dev \
        libopencv-video-dev \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# Install additional OpenCV dependencies
RUN apt-get update && \
    apt-get install -y \
        libgtk2.0-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libtbbmalloc2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# Install Ceres Solver dependencies and build Ceres
RUN apt-get update && \
    apt-get install -y \
        libgoogle-glog-dev \
        libgflags-dev \
        libgtest-dev \
        libsuitesparse-dev \
        libatlas-base-dev \
        libboost-system-dev \
        libboost-filesystem-dev \
        libboost-python-dev \
        tmux \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# Ceres Solver - install from source if needed, or try package
# Debian bookworm has ceres-solver 2.1+
RUN apt-get update && \
    apt-get install -y \
        libceres-dev \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# NOTE: Sophus is NOT installed here
# OV2SLAM uses its own copy from Thirdparty/Sophus which will be built with the project

# Install OpenGV (optional but recommended)
RUN apt-get update && \
    apt-get install -y \
        libopengv-dev \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# Install GeographicLib (for GPS coordinate conversion)
RUN apt-get update && \
    apt-get install -y \
        libgeographic++-dev \
        && apt-get clean && \
        rm -rf /var/lib/apt/lists/*

# === OPTIONAL: iBoW-LCD (Loop Closure Detection) ===
# Build from Thirdparty directory when building OV2SLAM
# Included in project Thirdparty/iBoW-LCD

# === PYTHON PACKAGES (for data analysis, visualization) ===
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    scipy \
    matplotlib \
    pillow

# === INSTALL CLAUDE CODE ===
RUN npm install -g @anthropic-ai/claude-code

# === INSTALL RERUN (3D Visualization) ===
# Install latest rerun CLI for visualizing SLAM results
RUN RERUN_VERSION=$(curl -s https://api.github.com/repos/rerun-io/rerun/releases/latest | grep '"tag_name"' | sed -E 's/.*"([^"]+)".*/\1/') && \
    curl -L https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun-cli-${RERUN_VERSION}-x86_64-unknown-linux-gnu -o /usr/local/bin/rerun && \
    chmod +x /usr/local/bin/rerun && \
    /usr/local/bin/rerun --version

# === USER SETUP ===
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=wojtess

RUN groupadd -g ${GROUP_ID} ${USERNAME} && \
    useradd -u ${USER_ID} -g ${GROUP_ID} -m -s /bin/bash ${USERNAME} && \
    echo '${USERNAME} ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# === WORKSPACE SETUP ===
WORKDIR /workspace

# Default to bash shell
USER ${USERNAME}
CMD ["/bin/bash"]
