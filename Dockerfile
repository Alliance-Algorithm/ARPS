FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y install \
    vim wget curl \
    libopencv-dev ros-humble-cv-bridge\
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY ./requirements.txt ./
RUN pip install -r ./requirements.txt && \
    rm -f ./requirements.txt

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics,video