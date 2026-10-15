FROM mcr.microsoft.com/devcontainers/python:1-3.11-bookworm

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       libosmesa6 libosmesa6-dev libgl1 libgl1-mesa-dri \
       libglfw3 libglew2.2 libegl1 ffmpeg \
    && rm -rf /var/lib/apt/lists/*

ENV MUJOCO_GL=osmesa
ENV MPLBACKEND=Agg
