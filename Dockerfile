From ubuntu:23.04

RUN apt update -y
RUN apt install -y libeigen3-dev
RUN apt install -y libboost-all-dev
RUN apt install -y curl
RUN apt install -y clang
RUN apt install -y wget
RUN apt install -y build-essential


ENV CC /usr/bin/clang-15
ENV CXX /usr/bin/clang++-15

RUN apt install -y git cmake

#RUN apt install python

#RUN curl https://ompl.kavrakilab.org/install-ompl-ubuntu.sh -o /tmp/install-ompl-ubuntu.sh && \
#    chmod u+x /tmp/install-ompl-ubuntu.sh && \
#    /tmp/install-ompl-ubuntu.sh

RUN cd /home && git clone --depth 1 -b 1.6.0 https://github.com/ompl/ompl.git ompl-1.6

RUN cd /home/ompl-1.6 && \ 
    mkdir -p build/Release && \
    cd build/Release && \
    cmake ../.. && \
    make

RUN cd /home/ompl-1.6 && \ 
    cd build/Release && \
    make install

RUN apt-get install -y \
    libcgal-dev \
    libgmp-dev \
    libmpfr-dev

# Install nlohmann_json
# RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp -O /usr/include/json.hpp
 
Copy ./mlmp /home/mlmp

RUN apt-get install -y \
	python3-pip \
	python3-venv

RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"


RUN pip install numpy matplotlib
