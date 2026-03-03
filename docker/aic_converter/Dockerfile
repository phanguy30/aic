# --- STAGE 1: IsaacLab ---
FROM nvcr.io/nvidia/isaac-lab:2.3.2 AS aic_isaaclab

COPY aic_assets/models/Enclosure/enclosure_visual.glb /workspace/isaaclab/assets/enclosure_visual.glb
COPY aic_assets/models/Enclosure/light_visual.glb /workspace/isaaclab/assets/light_visual.glb
COPY aic_assets/models/Floor/floor_visual.glb /workspace/isaaclab/assets/floor_visual.glb
COPY aic_assets/models/Floor/walls_visual.glb /workspace/isaaclab/assets/walls_visual.glb

RUN set -euo pipefail && \
    cd /workspace/isaaclab/assets && \
    for in_file in *.glb */*.glb; do \
      [ -e "$in_file" ] || continue; \
      out_file="${in_file%.glb}.usd"; \
      echo "Converting $in_file -> $out_file"; \
      /workspace/isaaclab/isaaclab.sh -p ../scripts/tools/convert_mesh.py \
        --collision-approximation none \
        --headless --device cpu \
        "$in_file" "$out_file"; \
    done && \
    rm *.glb


# --- STAGE 2: ROS 2 Kilted for aic_ws and gz-usd ---
FROM ros:kilted-ros-base AS aic_kilted

# Copy build assets from aic_isaaclab_stage
COPY --from=aic_isaaclab /workspace/isaaclab/assets /ws_aic/src/aic/aic_utils/aic_isaac/assets/staging/USDs
COPY --from=aic_isaaclab /workspace/isaaclab/assets/textures /ws_aic/src/aic/aic_utils/aic_isaac/assets/textures

SHELL ["/bin/bash", "-c"]

# Add `packages.osrfoundation.org` to the apt sources list:
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Build the workspace
COPY --exclude=docker . /ws_aic/src/aic
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  # undeclared deps
  apt install python3-numpy && \
  cd /ws_aic/src && \
  vcs import . < /ws_aic/src/aic/aic.repos --recursive && \
  # Install Gazebo dependencies.
  apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') 
  
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  cd /ws_aic && \
  # Install ROS dependencies using rosdep.
  rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev" && \
  source /opt/ros/kilted/setup.bash && \
  GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --executor sequential

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  apt install libpyside2-dev \
  cmake \
  freeglut3-dev \
  libglu1-mesa-dev \
  mesa-common-dev \
  python3-opengl \
  vim -y && \
  cd /ws_aic && \
  git clone --depth 1 -b v24.08 https://github.com/PixarAnimationStudios/OpenUSD.git 

ENV USD_PATH=/ws_aic/OpenUSD
ENV PATH=$USD_PATH/bin:$PATH
ENV LD_LIBRARY_PATH=$USD_PATH/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=$USD_PATH:$CMAKE_PREFIX_PATH

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  cd /ws_aic/OpenUSD && \
  # Patch the dead JFrog URL with the active Boost Archives URL
  sed -i 's/boostorg.jfrog.io\/artifactory\/main/archives.boost.io/g' build_scripts/build_usd.py && \
  python3 build_scripts/build_usd.py --build-variant release --no-tests --no-examples --no-imaging --onetbb --no-tutorials --no-docs $USD_PATH && \
  cd /ws_aic && \
  git clone https://github.com/gazebosim/gz-usd && \
  source /ws_aic/install/setup.bash && \
  cd /ws_aic/gz-usd && \
  mkdir build && \
  cd /ws_aic/gz-usd/build && \
  export GZ_VERSION=ionic && \
  cmake .. && \
  make

ENV PYTHONPATH=$PYTHONPATH:$USD_PATH/lib/python
 
COPY docker/aic_converter/zenoh_router_config.json5 /zenoh_router_config.json5

RUN /bin/bash -c '\
    source /ws_aic/install/setup.bash && \
    \
    # 1. Configure Zenoh
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
    export ZENOH_ROUTER_CONFIG_URI=/zenoh_router_config.json5 && \
    \
    # 2. Start Zenoh Router in background
    ros2 run rmw_zenoh_cpp rmw_zenohd & \
    ZENOHD_PID=$! && \
    \
    # 3. Launch with timeout to allow file generation
    # Use "timeout" to ensure the build proceeds after the file is written
    echo "Launching Gazebo to generate SDF..." && \
    source /ws_aic/install/setup.bash && \
    timeout 60s ros2 launch aic_bringup aic_gz_bringup.launch.py \
        gazebo_gui:=false \
        launch_rviz:=false || true && \
    \
    # 4. Cleanup background processes
    kill -SIGINT $ZENOHD_PID && \
    \
    # 5. Validation: Ensure the file actually exists before finishing the layer
    if [ -f /tmp/aic.sdf ]; then \
        echo "SDF successfully generated."; \
    else \
        echo "Error: /tmp/aic.sdf not found!" && exit 1; \
    fi'

WORKDIR /ws_aic/src/aic/aic_utils/aic_isaac

# Copy SDF generated by AIC Gazebo scene to staging
RUN mkdir -p ./assets/staging/SDFs && \
  cp /tmp/aic.sdf ./assets/staging/SDFs/aic_raw.sdf

# Apply fitler rules to SDFs
RUN  python3 ./aic_converter/scripts/filter_sdf.py ./assets/staging/SDFs/aic_raw.sdf ./aic_converter/config/aic_world.yaml ./assets/staging/SDFs/aic_world.sdf && \
 python3 ./aic_converter/scripts/filter_sdf.py ./assets/staging/SDFs/aic_raw.sdf ./aic_converter/config/aic_enclosure.yaml ./assets/staging/SDFs/aic_enclosure.sdf

# Convert SDFs to USDs
RUN  source /ws_aic/install/setup.bash && \
  /ws_aic/gz-usd/build/bin/sdf2usd ./assets/staging/SDFs/aic_world.sdf ./assets/staging/USDs/aic_world.usd && \
  /ws_aic/gz-usd/build/bin/sdf2usd ./assets/staging/SDFs/aic_enclosure.sdf ./assets/staging/USDs/aic_enclosure.usd 

# Convert USDs to USDAs for debugging
RUN  mkdir -p ./assets/staging/USDAs && \
  /ws_aic/OpenUSD/bin/usdcat ./assets/staging/USDs/aic_world.usd -o ./assets/staging/USDAs/aic_world.usda && \
  /ws_aic/OpenUSD/bin/usdcat ./assets/staging/USDs/aic_enclosure.usd -o ./assets/staging/USDAs/aic_enclosure.usda  && \
  /ws_aic/OpenUSD/bin/usdcat ./assets/staging/USDs/floor_visual.usd -o ./assets/staging/USDAs/floor_visual.usda  && \
  /ws_aic/OpenUSD/bin/usdcat ./assets/staging/USDs/walls_visual.usd -o ./assets/staging/USDAs/walls_visual.usda  && \
  /ws_aic/OpenUSD/bin/usdcat ./assets/staging/USDs/light_visual.usd -o ./assets/staging/USDAs/light_visual.usda  && \
  /ws_aic/OpenUSD/bin/usdcat ./assets/staging/USDs/enclosure_visual.usd -o ./assets/staging/USDAs/enclosure_visual.usda  

# Apply USDA injector 
RUN python3 ./aic_converter/scripts/patch_usda.py ./assets/staging/USDAs/aic_world.usda ./aic_converter/config/aic_world.yaml ./assets/aic_world.usda && \
  python3 ./aic_converter/scripts/patch_usda.py ./assets/staging/USDAs/aic_enclosure.usda ./aic_converter/config/aic_enclosure.yaml ./assets/aic_enclosure.usda

# Generate Robot URDF
RUN mkdir -p ./assets/staging/URDFs && \
  source /ws_aic/install/setup.bash && \
  ros2 run xacro xacro /ws_aic/src/aic/aic_description/urdf/ur_gz.urdf.xacro -o ./assets/staging/URDFs/aic_robot_raw.urdf \
  name:=ur safety_limits:=false safety_pos_margin:=0.15 safety_k_position:=20

# Apply filter rules to URDF
RUN python3 ./aic_converter/scripts/filter_urdf.py ./assets/staging/URDFs/aic_robot_raw.urdf ./aic_converter/config/aic_robot.yaml ./assets/staging/URDFs/aic_robot.urdf


# --- STAGE 3: Final image ---
FROM aic_isaaclab AS aic_urdf

# Copy assets from aic_kilted
COPY --from=aic_kilted /ws_aic /ws_aic

WORKDIR /workspace/isaaclab

RUN apt-get update && apt-get install -y rename && \
    cd /ws_aic/src/aic/aic_assets/models && \
    find . -depth -name "*[- ]*" | while read -r f; do \
      dir=$(dirname "$f"); \
      base=$(basename "$f"); \
      newbase=$(echo "$base" | sed 's/[- ]/_/g'); \
      [ "$base" != "$newbase" ] && mv -n "$f" "$dir/$newbase"; \
    done

RUN cd /ws_aic/src/aic/aic_utils/aic_isaac && \
  /workspace/isaaclab/isaaclab.sh -p ./aic_converter/scripts/urdf_to_usd.py ./assets/staging/URDFs/aic_robot.urdf ./assets/aic_robot.usd --headless --device "cpu"


# --- STAGE 4: Final image ---
FROM aic_kilted AS export_assets

# Copy assets from aic_urdf
COPY --from=aic_urdf /ws_aic/src/aic/aic_utils/aic_isaac/assets/aic_robot.usd /ws_aic/src/aic/aic_utils/aic_isaac/assets/aic_robot.usd

# WORKDIR /ws_aic/src/aic/aic_utils/aic_isaac

RUN cat <<'EOF' > /entrypoint.sh
#!/bin/bash
set -e
# source /ws_aic/install/setup.bash
exec "$@"
EOF

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
