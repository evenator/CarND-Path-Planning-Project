FROM ubuntu:xenial
RUN apt-get update && apt-get install -y build-essential cmake libuv1-dev git libssl-dev zlib1g
RUN git clone https://github.com/uWebSockets/uWebSockets && \
    cd uWebSockets && \
    git checkout e94b6e1 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so && \
    cd ../.. && \
    rm -r uWebSockets

