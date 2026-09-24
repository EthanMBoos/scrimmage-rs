ARG DEPENDENCY_IMAGE=scrimmage-rs-reference-deps:ubuntu-24.04
FROM ${DEPENDENCY_IMAGE}

ARG SOURCE_COMMIT
ARG BUILD_JOBS=4
LABEL org.opencontainers.image.revision=${SOURCE_COMMIT}

# A git archive of the selected branch, not the host's patched/native build.
ADD source.tar /opt/
WORKDIR /opt/source
RUN cmake -S /opt/source -B /opt/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DSETUP_HOME_CONFIG=OFF -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
      -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF -DBUILD_ROS_PLUGINS=OFF \
      -DENABLE_GPU_ACCELERATION=OFF \
      -DCMAKE_DISABLE_FIND_PACKAGE_VTK=ON -DVTK_FOUND=OFF \
      -DCMAKE_DISABLE_FIND_PACKAGE_GRPC=ON \
      -DCMAKE_DISABLE_FIND_PACKAGE_OpenCV=ON \
    && cmake --build /opt/build --parallel ${BUILD_JOBS} --target \
      scrimmage-bin Straight_plugin SimpleAircraftControllerPID_plugin \
      SimpleAircraft_plugin SimpleCollision_plugin SimpleCollisionMetrics_plugin \
      LocalNetwork_plugin GlobalNetwork_plugin NoisyState_plugin \
      Boundary_plugin GroundCollision_plugin

# Separate layer keeps the established reference build cached.
RUN cmake --build /opt/build --parallel ${BUILD_JOBS} --target \
      FixedWing6DOF_plugin AircraftPIDController_plugin

ENV SCRIMMAGE_DIR=/opt/source \
    SCRIMMAGE_PLUGIN_PATH=/opt/build/plugin_libs:/opt/source/include/scrimmage/plugins \
    SCRIMMAGE_DATA_PATH=/opt/source/data \
    SCRIMMAGE_CONFIG_PATH=/opt/source/config \
    SCRIMMAGE_MISSION_PATH=/opt/source/missions \
    LD_LIBRARY_PATH=/opt/build/lib:/opt/build/plugin_libs

ENTRYPOINT ["/opt/build/bin/scrimmage"]
