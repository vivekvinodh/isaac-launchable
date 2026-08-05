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
FROM nvcr.io/nvidia/isaac-lab:3.0.0-beta2-post1

USER root

RUN apt update && \
    apt install -y \
	build-essential \
	cmake \
	curl \
	git

RUN curl -fsSL https://code-server.dev/install.sh | sh -s -- --version=4.96.4

RUN apt update && \
	apt install -y \
	unzip \
	dnsutils \
	iproute2 \
	jq

ENV WORKSPACE_DIR=/workspace

COPY --chown=isaaclab:isaaclab ./README.md ${WORKSPACE_DIR}/README.md
COPY --chown=isaaclab:isaaclab ./settings.json ${WORKSPACE_DIR}/.vscode/settings.json

USER isaaclab:isaaclab

RUN code-server \
    --install-extension ms-python.python \
    --install-extension ms-toolsai.jupyter \
    --force ${WORKSPACE_DIR}

USER root

COPY entrypoint.sh /entrypoint.sh

# OV Cache Directories
RUN mkdir -p /root/.cache/ov
RUN mkdir -p /root/.local/share/ov

# Fix permissions for isaaclab user
RUN chown -R isaaclab:isaaclab /workspace /root/.cache/ov /root/.local/share/ov /isaac-sim/kit/cache

# === Phase 2 — PX4 + Pegasus + MAVSDK + helper scripts ===
# See infrastructure/launchable/DESIGN-NOTES.md in the spike repo for design rationale.
ENV DEBIAN_FRONTEND=noninteractive

# PX4 SITL build deps (build-essential + cmake + git already installed above).
# gcc-12: PX4 v1.14.3 uses uint8_t without #include <cstdint> in some files,
# which GCC 13's stricter transitive-include policy rejects. gcc-12 still has
# the lenient behavior the spike's Ubuntu 22.04 host build relied on.
RUN apt update && apt install -y --no-install-recommends \
        ninja-build \
        libxml2-dev \
        libxml2-utils \
        libxslt-dev \
        libtool \
        python3-pip \
        python3-dev \
        python3-venv \
        gcc-12 \
        g++-12 \
    && rm -rf /var/lib/apt/lists/*

# Guarantee a writable home for the isaaclab user — the PX4 clone, the
# pip --user installs, and the SITL build below all write under it.
RUN mkdir -p /home/isaaclab && chown isaaclab:isaaclab /home/isaaclab

USER isaaclab
WORKDIR /home/isaaclab

RUN git clone --branch v1.14.3 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git

WORKDIR /home/isaaclab/PX4-Autopilot
# Spike errata #9 — modern pip rejects `>=N.M.*` in requirements.
RUN sed -i 's/>=\([0-9.]*\)\.\*/>=\1/g' Tools/setup/requirements.txt
# Spike errata #10 — PX4 uses em.RAW_OPT which empy 4.x removed.
# --break-system-packages: base image enforces PEP 668 (Ubuntu 24.04+).
# Spike's host build worked on 22.04/3.10 without this flag.
RUN pip3 install --user --break-system-packages "empy<4"
RUN pip3 install --user --break-system-packages -r Tools/setup/requirements.txt
# Force gcc-12 to avoid GCC 13's stricter transitive-include policy.
ENV CC=gcc-12
ENV CXX=g++-12
# Build SITL binary only (no model arg — `make px4_sitl_default none_iris`
# triggers respawn loop per spike errata #11).
RUN make px4_sitl_default

# Pegasus v5.1.0 — clone into kit's auto-discovery path.
# v5.1.0 is the newest Pegasus release and targets Isaac Sim 5.x; no Isaac
# Sim 6.x release exists yet, so this pairing with the 3.0.0-beta2-post1
# base (Isaac Sim 6.0.1) is unverified at runtime.
USER root
RUN mkdir -p /isaac-sim/extsUser && chown isaaclab:isaaclab /isaac-sim/extsUser
USER isaaclab
RUN cd /isaac-sim/extsUser && \
    git clone --branch v5.1.0 --depth 1 https://github.com/PegasusSimulator/PegasusSimulator.git && \
    ln -s /isaac-sim/extsUser/PegasusSimulator/extensions/pegasus.simulator /isaac-sim/extsUser/pegasus.simulator

# MAVSDK Python — pinned to spike's verified version.
RUN pip3 install --user --break-system-packages mavsdk==3.15.3

# Spike helper scripts available at a stable path inside the container.
USER root
COPY --chown=isaaclab:isaaclab ./spike-scripts /opt/spike-scripts
# === end Phase 2 ===

USER isaaclab:isaaclab

ENTRYPOINT ["/bin/bash"]

CMD ["/entrypoint.sh"]
