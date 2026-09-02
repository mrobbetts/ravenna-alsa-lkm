FROM ubuntu:devel

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install essential build tools, git, kmod, and generic kernel headers
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    kmod \
    linux-headers-generic \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src/ravenna-alsa-lkm

# 2. Clone the repository and checkout the target branch
RUN git clone --branch aes67-daemon https://github.com/bondagit/ravenna-alsa-lkm.git .

# 3. Compile by intercepting uname so that any script calling `uname -r` 
# returns the container's installed kernel version instead of the WSL2 host kernel.
RUN cd driver && \
    KERNEL_VER=$(ls /lib/modules | head -n 1) && \
    echo "Forcing build for kernel version: $KERNEL_VER" && \
    # Create a temporary wrapper for uname that returns the container kernel version
    mv /bin/uname /bin/uname.bak && \
    printf '#!/bin/sh\nif [ "$1" = "-r" ]; then echo "%s"; else /bin/uname.bak "$@"; fi\n' "$KERNEL_VER" > /bin/uname && \
    chmod +x /bin/uname && \
    # Run make
    make && \
    # Restore original uname
    mv /bin/uname.bak /bin/uname

CMD ["bash"]
