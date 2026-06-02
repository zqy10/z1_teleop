FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install VNC server, lightweight desktop, and ROS GUI tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    tigervnc-standalone-server \
    tigervnc-common \
    xfce4 \
    xfce4-terminal \
    dbus-x11 \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    x11-utils \
    x11-apps \
    wget curl git \
    && rm -rf /var/lib/apt/lists/*

# Fix drirc
RUN printf '<driconf>\n</driconf>\n' > /etc/drirc

# Software rendering env
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV LIBGL_ALWAYS_INDIRECT=0
ENV MESA_LOADER_DRIVER_OVERRIDE=swrast
ENV MESA_GL_VERSION_OVERRIDE=3.3
ENV MESA_GLSL_VERSION_OVERRIDE=330
ENV GALLIUM_DRIVER=llvmpipe
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:1

# VNC password
RUN mkdir -p /root/.vnc && \
    echo "ros1234" | vncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd

# VNC xstartup — let dbus launch naturally
RUN printf '#!/bin/bash\n\
unset DBUS_SESSION_BUS_ADDRESS\n\
export XDG_RUNTIME_DIR=/tmp/runtime-root\n\
mkdir -p $XDG_RUNTIME_DIR\n\
chmod 700 $XDG_RUNTIME_DIR\n\
exec dbus-launch --exit-with-session xfce4-session\n' \
    > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup

# VNC start script
RUN printf '#!/bin/bash\n\
vncserver -kill :1 2>/dev/null || true\n\
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true\n\
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no\n\
echo ""\n\
echo "VNC started — connect to localhost:5901 password: ros1234"\n\
echo ""\n\
tail -f /root/.vnc/*.log\n' \
    > /start_vnc.sh && \
    chmod +x /start_vnc.sh

# Source ROS in every shell
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

WORKDIR /workspace/z1_teleop
EXPOSE 5901

CMD ["/start_vnc.sh"]