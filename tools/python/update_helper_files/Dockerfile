# This Dockerfile sets up a container with the minimum number of things to make
# //tools/python:requirements.update target happy.

FROM debian:12

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y clang
RUN DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y python3
RUN DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y pkgconf
RUN DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y libcairo2-dev
