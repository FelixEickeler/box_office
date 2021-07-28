FROM debian:bullseye-slim
# FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV TERM xterm-256color

RUN apt-get clean && apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    geomview \
    libboost-dev \
    libboost-program-options-dev \
    libboost-thread-dev \
    libboost-filesystem-dev\
    libeigen3-dev \
    libglew1.5-dev \
    libgmp10-dev \
    libipe-dev \
    libmpfi-dev \
    libmpfr-dev \
    libqglviewer-dev-qt5 \
    qtbase5-dev \
    qtscript5-dev \
    qttools5-dev \
    qttools5-dev-tools \
    libqt5svg5-dev \
    libqt5opengl5-dev \
    tar \
    libtbb-dev \
    zlib1g-dev \
    python3 gosu sudo openssh-server gdb \
    htop nano python3-distutils libpython3-dev python3-pip libtbb-dev rsync valgrind bash-completion

# Get CGAL (optional I guess ?)Dependencies
RUN git clone https://github.com/STORM-IRIT/OpenGR.git --depth 1 \
 && cd ./OpenGR \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make -j"$(nproc)" \
 && make install \
 && mkdir -p /usr/local/lib/cmake/opengr \
 && cd ../.. \
 && rm -rf ./OpenGR

RUN git clone git://github.com/ethz-asl/libnabo.git \
 && cd libnabo \
 && SRC_DIR=`pwd` && BUILD_DIR=${SRC_DIR}/build \
 && mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR} \
 && cmake -DCMAKE_BUILD_TYPE=Release ${SRC_DIR} \
 && make install \
 && cd ../.. && rm -rf libnado

RUN git clone git://github.com/ethz-asl/libpointmatcher.git \
 && cd libpointmatcher \
 && SRC_DIR=`pwd` \
 && BUILD_DIR=${SRC_DIR}/build \
 && mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR} \
 && cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release ${SRC_DIR} \
 && make -j"$(nproc)" && make install \
 && cd ../.. && rm -rf libpointmatcher

 #compile cgal
 WORKDIR /home/boxy/
 RUN git clone https://github.com/CGAL/cgal.git \
  && cd cgal \
  && mkdir -p _build && cd _build \
  && cmake -DWITH_CGAL_Qt5:BOOL=OFF -DCMAKE_BUILD_TYPE=Release  .. \
  && make install


RUN pip3 install numpy scikit-learn scipy #open3d
RUN pip install --pre https://storage.googleapis.com/open3d-releases-master/python-wheels/open3d-0.13.0+7c62640-cp39-cp39-manylinux_2_27_x86_64.whl


# create user, ids are temporary
ARG USER_ID=1000
RUN useradd -m --no-log-init boxy && yes brucelee | passwd boxy
RUN usermod -aG sudo boxy
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN chown -R boxy:boxy /home/boxy


# WORKDIR /home/boxy/

#COPY ../ /home/boxy/BoxOffice
#RUN chmod -R go-w /home/boxy/BoxOffice

# Add helios-entrypoint, for user-managment (gosu e.t.c)
COPY boxoffice-entrypoint.sh /usr/local/bin/boxoffice-entrypoint.sh
COPY init.sh /usr/local/bin/init.sh
RUN chmod +x /usr/local/bin/boxoffice-entrypoint.sh
RUN chmod +x /usr/local/bin/init.sh
CMD ["/usr/local/bin/init.sh"]
ENTRYPOINT ["/usr/local/bin/boxoffice-entrypoint.sh"]
