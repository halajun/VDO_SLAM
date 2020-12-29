FROM ubuntu:16.04

RUN mkdir /app
WORKDIR /app

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository ppa:ubuntu-toolchain-r/test

# Install GCC-9
RUN apt-get update \
	&& DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
		build-essential \
		gcc-9 \
		g++-9 \
		gcc-9-multilib \
		g++-9-multilib \
		xutils-dev \
		patch \
		git \
		python3 \
		python3-pip \
		libpulse-dev \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 50 \
	&& update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 50

# Install OpenCV 3.4

RUN apt-get update
RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev \
libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev \
libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev unzip

RUN apt-get install -y wget
RUN apt-get install -y vim

# get and build OpenCV 3.4
RUN cd \
    && wget https://github.com/opencv/opencv/archive/3.4.0.zip \
    && unzip 3.4.0.zip \
    && cd opencv-3.4.0 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j8 \
    && make install \
    && cd \
    && rm 3.4.0.zip

# install and build opencv_contrib
RUN cd \
    && wget https://github.com/opencv/opencv_contrib/archive/3.4.0.zip \
    && unzip 3.4.0.zip \
    && cd opencv-3.4.0/build \
    && cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.0/modules/ .. \
    && make -j8 \
    && make install \
    && cd ../.. \
    && rm 3.4.0.zip


# Install Eigen3
RUN apt install libeigen3-dev

# install CSparse
RUN DEBIAN_FRONTEND=noninteractive apt install -y libsuitesparse-dev

COPY . .

# Build the Library
RUN chmod +x build.sh
RUN ./build.sh

CMD ["/bin/bash"]
