FROM ros:kinetic-ros-core

RUN apt-get update -qq && apt-get install -y python-pip \
    python-opencv \
    ros-kinetic-image-transport \
    ros-kinetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY animator/ /app/animator
COPY detectors/face_detection /app/face_detection
COPY ros-utils /app/ros-utils
RUN cd /app/animator && pip install --process-dependency-links -e .
RUN cd /app/face_detection && pip install --process-dependency-links -e .
RUN cd /app/ros-utils && pip install --process-dependency-links -e .
COPY ops/docker/scripts /
RUN /setup_ros.sh

ENTRYPOINT /entrypoint.sh
