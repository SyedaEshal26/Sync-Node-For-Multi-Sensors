FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ros2_ws/src/sync_node_pkg
WORKDIR /ros2_ws/src/sync_node_pkg

# Copy package contents (build context should be the sync_node_pkg folder itself)
COPY CMakeLists.txt package.xml ./
COPY include ./include
COPY src ./src

WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select sync_node_pkg

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "sync_node_pkg", "sync_node"]
