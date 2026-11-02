# syntax=docker/dockerfile:1
FROM ubuntu:24.04 AS builder
ARG OMPL_VERSION=1.7.0
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates \
        libboost-serialization-dev libboost-filesystem-dev libboost-system-dev \
        libboost-program-options-dev libeigen3-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 -b "${OMPL_VERSION}" https://github.com/ompl/ompl.git /src/ompl \
 && cmake -S /src/ompl -B /src/ompl/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOMPL_BUILD_TESTS=OFF \
        -DOMPL_REGISTRATION=OFF \
 && cmake --build /src/ompl/build --target ompl -j"$(nproc)" \
 && cmake --install /src/ompl/build \
 && DESTDIR=/stage cmake --install /src/ompl/build \
 && rm -rf /src/ompl/build /src/ompl/.git

COPY mlmp /src/mlmp
RUN cmake -S /src/mlmp -B /src/mlmp/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /src/mlmp/build -j"$(nproc)"

FROM ubuntu:24.04 AS runtime
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        libboost-serialization1.83.0 libboost-filesystem1.83.0 \
        libboost-program-options1.83.0 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /stage/usr/local/lib/ /usr/local/lib/
COPY --from=builder /src/mlmp/build/mlmp /usr/local/bin/mlmp
COPY --from=builder /src/mlmp/src/inputs/experiment/ /opt/mlmp/scenes/
COPY --from=builder /src/mlmp/src/inputs/*.json /opt/mlmp/scenes/dev/
RUN ldconfig
ENV MLMP_SCENE_DIR=/opt/mlmp/scenes
ENTRYPOINT ["/usr/local/bin/mlmp"]
CMD ["--help"]
