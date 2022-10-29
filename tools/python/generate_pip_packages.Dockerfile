FROM debian:bullseye

RUN apt-get update
RUN apt-get install -y \
    curl \
    clang-13

# Get latest patchelf for auditwheel.
RUN curl -L https://github.com/NixOS/patchelf/releases/download/0.15.0/patchelf-0.15.0-x86_64.tar.gz > /tmp/patchelf.tar.gz \
    && tar -xaf /tmp/patchelf.tar.gz -C /usr \
    && rm -f /tmp/patchelf.tar.gz

# Get the same Python that we're using for for actually running Python code.
RUN mkdir /opt/python/
RUN curl -SL \
    https://github.com/indygreg/python-build-standalone/releases/download/20220802/cpython-3.9.13+20220802-x86_64-unknown-linux-gnu-install_only.tar.gz \
    | tar -xz -C /opt/

# Install dependencies of the pip packages that we're compiling.
RUN apt-get install -y \
    libcairo2-dev \
    libgirepository1.0-dev \
    libglib2.0-0 \
    libgtk-3-dev

# Make some symlinks to satisfy assumptions some of the installer scripts (e.g.
# setup.py files) make about the system.
RUN ln -s /opt/python/ /install
RUN ln -s /usr/bin/clang-13 /usr/bin/clang && \
    ln -s /usr/bin/clang++-13 /usr/bin/clang++
