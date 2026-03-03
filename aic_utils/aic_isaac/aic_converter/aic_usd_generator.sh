#!/bin/bash

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DOCKER_DIR_PATH=$SCRIPT_DIR/../../../docker

cd $DOCKER_DIR_PATH

# -- Build ws_aic

# ARG to build aic_converter docker image 
docker compose build converter

# Debug aic_isaaclab_stage artifacts
# docker cp $(docker create --rm ghcr.io/intrinsic-dev/aic/aic_converter):/workspace/isaaclab/artifacts $SCRIPT_DIR/artifacts

# ARG to copy aic.sdf to build_artifacts on host
# docker cp $(docker create --rm ghcr.io/intrinsic-dev/aic/aic_converter):/tmp/aic.sdf $SCRIPT_DIR/build_artifacts/aic.sdf




# ARG to copy aic_world.usd to build_artifacts on host
docker cp $(docker create --rm ghcr.io/intrinsic-dev/aic/aic_converter):/ws_aic/src/aic/aic_utils/aic_isaac/assets $SCRIPT_DIR/../assets


# Display help function
# function display_help {

# }
