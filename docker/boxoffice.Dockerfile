FROM debian:bullseye-slim

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8

RUN apt-get  -y update && apt-get install -y \
	build-essential python3 gosu sudo libcgal-dev openssh-server\
	htop nano cmake python3-distutils libpython3-dev python3-pip libtbb-dev

RUN pip3 install numpy
# create user, ids are temporary
ARG USER_ID=1000
RUN useradd -m --no-log-init boxy && yes brucelee | passwd boxy
RUN usermod -aG sudo boxy
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# clone Helios++
#WORKDIR /home/boxy
#RUN git clone https://github.com/3dgeo-heidelberg/helios.git helios++

# clone & build LAStools
#WORKDIR /home/phaethon/helios++
#RUN git clone https://github.com/LAStools/LAStools.git lib/LAStools
#RUN mkdir lib/LAStools/_build && cd lib/LAStools/_build && cmake .. && make -j $(nproc)

# build helios ++
#RUN mkdir _build && cd _build && cmake .. && make -j $(nproc)

#RUN chown -R phaethon:sudo "/home/phaethon/helios++"
#RUN chmod -R a=r,a+X,u+w "/home/phaethon/helios++"
#RUN chmod 755 "/home/phaethon/helios++/_build/helios"

#RUN echo "PATH=$PATH:/home/phaethon/helios++/_build" >> /home/phaethon/.profile
#ENV PATH "$PATH:/home/phaethon/helios++/_build"

WORKDIR /home/boxy/

#COPY ../ /home/boxy/BoxOffice
#RUN chmod -R go-w /home/boxy/BoxOffice

# Add helios-entrypoint, for user-managment (gosu e.t.c)
COPY boxoffice-entrypoint.sh /usr/local/bin/boxoffice-entrypoint.sh
RUN chmod +x /usr/local/bin/boxoffice-entrypoint.sh
ENTRYPOINT ["/usr/local/bin/boxoffice-entrypoint.sh"]
