FROM  ros:jazzy-ros-base

ENV ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    TZ=America/Chicago

# Keep downloaded .debs in the BuildKit cache instead of deleting them
RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
    && apt-get upgrade -y clangd

RUN rosdep update --rosdistro ${ROS_DISTRO}

# Copy ONLY package.xml so this layer is reused unless dependencies change.
# The source itself is mounted at /workspaces by the devcontainer.
COPY robot/package.xml /tmp/deps/robot/package.xml

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
    && rosdep install --from-paths /tmp/deps -i -y --rosdistro ${ROS_DISTRO}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
