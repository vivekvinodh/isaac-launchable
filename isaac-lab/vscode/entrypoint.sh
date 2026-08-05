#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set -eux

WORKSPACE_DIR=${WORKSPACE_DIR:-/workspace}

# Set auth mode based on PASSWORD env var
if [ -z "${PASSWORD:-}" ]; then
    AUTH_MODE="none"
else
    AUTH_MODE="password"
fi

# Phase 2 — auto-launch PX4 SITL in background with Pegasus's canonical args.
# (spike errata #12: PX4_SIM_MODEL=gazebo-classic_iris is required for Pegasus's
# heartbeat protocol; positional ROMFS + source rcS match what Day 3 successfully
# connected against.) PX4 stdout/stderr to /tmp/px4.log so the cold-boot operator
# can tail it. Skipped silently if the binary isn't present (e.g., during local
# dev iteration without the full image build).
PX4_BIN=/home/isaaclab/PX4-Autopilot/build/px4_sitl_default/bin/px4
if [ -x "$PX4_BIN" ]; then
    PX4_SIM_MODEL=gazebo-classic_iris \
    "$PX4_BIN" \
        /home/isaaclab/PX4-Autopilot/ROMFS/px4fmu_common/ \
        -s /home/isaaclab/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS \
        -i 0 -d > /tmp/px4.log 2>&1 &
    echo "PX4 launched in background, PID=$!" >&2
fi

code-server --bind-addr=127.0.0.1:8080 \
    --auth="${AUTH_MODE}" \
    "${WORKSPACE_DIR}"
