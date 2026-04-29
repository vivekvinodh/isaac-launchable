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
FROM nvcr.io/nvidia/isaac-lab:2.3.0

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

COPY --chown=isaac-sim:isaac-sim ./README.md ${WORKSPACE_DIR}/README.md
COPY --chown=isaac-sim:isaac-sim ./settings.json ${WORKSPACE_DIR}/.vscode/settings.json


RUN code-server \
    --install-extension ms-python.python \
    --install-extension ms-toolsai.jupyter \
    --force ${WORKSPACE_DIR}

COPY entrypoint.sh /entrypoint.sh

# OV Cache Directories
RUN mkdir -p /home/isaac-sim/.cache/ov
RUN mkdir -p /home/isaac-sim/.local/share/ov

# Fix permissions for ubuntu user
RUN chown -R isaac-sim /workspace /home/isaac-sim/.cache/ov /home/isaac-sim/.local/share/ov /isaac-sim/kit/cache

# Copy bashrc to get aliases
RUN cp /root/.bashrc /isaac-sim/.root-bashrc
RUN echo 'source $HOME/.root-bashrc' >> /isaac-sim/.bashrc

# === Phase 2 — PX4 + Pegasus + MAVSDK + helper scripts ===
# See infrastructure/launchable/DESIGN-NOTES.md in the spike repo for design rationale.
ENV DEBIAN_FRONTEND=noninteractive

# PX4 SITL build deps (build-essential + cmake + git already installed above).
RUN apt update && apt install -y --no-install-recommends \
        ninja-build \
        libxml2-dev \
        libxml2-utils \
        libxslt-dev \
        libtool \
        python3-pip \
        python3-dev \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Upstream's `mkdir -p /home/isaac-sim/.cache/ov` creates the parent as root,
# and their subsequent `chown -R isaac-sim` only targets specific subpaths.
# Without this fix, `git clone` as isaac-sim hits "Permission denied" in $HOME.
RUN chown isaac-sim:isaac-sim /home/isaac-sim

USER isaac-sim
WORKDIR /home/isaac-sim

RUN git clone --branch v1.14.3 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git

WORKDIR /home/isaac-sim/PX4-Autopilot
# Spike errata #9 — modern pip rejects `>=N.M.*` in requirements.
RUN sed -i 's/>=\([0-9.]*\)\.\*/>=\1/g' Tools/setup/requirements.txt
# Spike errata #10 — PX4 uses em.RAW_OPT which empy 4.x removed.
# --break-system-packages: base image is Ubuntu 24.04 / Python 3.12, which
# enforces PEP 668. Spike's host build worked on 22.04/3.10 without this flag.
RUN pip3 install --user --break-system-packages "empy<4"
RUN pip3 install --user --break-system-packages -r Tools/setup/requirements.txt
# Build SITL binary only (no model arg — `make px4_sitl_default none_iris`
# triggers respawn loop per spike errata #11).
RUN make px4_sitl_default

# Pegasus v5.1.0 — clone into kit's auto-discovery path.
USER root
RUN mkdir -p /isaac-sim/extsUser && chown isaac-sim:isaac-sim /isaac-sim/extsUser
USER isaac-sim
RUN cd /isaac-sim/extsUser && \
    git clone --branch v5.1.0 --depth 1 https://github.com/PegasusSimulator/PegasusSimulator.git && \
    ln -s /isaac-sim/extsUser/PegasusSimulator/extensions/pegasus.simulator /isaac-sim/extsUser/pegasus.simulator

# MAVSDK Python — pinned to spike's verified version.
RUN pip3 install --user --break-system-packages mavsdk==3.15.3

# Spike helper scripts available at a stable path inside the container.
USER root
COPY --chown=isaac-sim:isaac-sim ./spike-scripts /opt/spike-scripts
# === end Phase 2 ===

USER isaac-sim:isaac-sim

ENTRYPOINT ["/bin/bash"]

CMD ["/entrypoint.sh"]
