#!/usr/bin/env bash
# Install TurboVNC and VirtualGL
# For hardware-accelerated VNC (useful for ZED camera in VNC sessions)
# Sources:
#   https://turbovnc.org/Downloads/YUM
#   https://virtualgl.org/Downloads/YUM

set -e

echo "TurboVNC + VirtualGL Installation"
echo "=================================="
echo ""
echo "This installs TurboVNC and VirtualGL for hardware-accelerated"
echo "remote desktop sessions (required for ZED camera in VNC)."
echo ""

# Only x86_64 is supported for now
ARCH="${ARCH:-$(uname -m)}"
if [[ "$ARCH" != "x86_64" ]]; then
    echo "Warning: TurboVNC/VirtualGL packages are typically only available for x86_64."
    echo "Your architecture: $ARCH"
    echo "You may need to build from source for other architectures."
    exit 1
fi

echo "Adding TurboVNC APT repository..."
# Add TurboVNC GPG key
wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/TurboVNC.gpg

# Add TurboVNC repository
sudo wget -q -O /etc/apt/sources.list.d/TurboVNC.list \
    https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list

echo "Adding VirtualGL APT repository..."
# Add VirtualGL GPG key
wget -q -O- https://packagecloud.io/dcommander/virtualgl/gpgkey | \
    sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/VirtualGL.gpg

# Add VirtualGL repository
sudo wget -q -O /etc/apt/sources.list.d/VirtualGL.list \
    https://raw.githubusercontent.com/VirtualGL/repo/main/VirtualGL.list

echo "Updating package lists..."
sudo apt-get update

echo "Installing TurboVNC and VirtualGL..."
sudo apt-get install -y turbovnc virtualgl

echo ""
echo "Configuring VirtualGL server..."
# Configure VirtualGL (unattended mode):
#   -config  = Configure server for use with VirtualGL (GLX + EGL back ends)
#   +s       = Open 3D X server access to all users (not restricted to vglusers)
#   +f       = Open framebuffer device access to all users (not restricted to vglusers)
#   -t       = Disable XTEST extension (default)
sudo /opt/VirtualGL/bin/vglserver_config -config +s +f

echo ""
echo "Setting up vncserver alternative..."
# Register TurboVNC's vncserver with update-alternatives
# Install the alternative with high priority, then force selection
sudo update-alternatives --install /usr/bin/vncserver vncserver /opt/TurboVNC/bin/vncserver 100
sudo update-alternatives --set vncserver /opt/TurboVNC/bin/vncserver

echo ""
echo "TurboVNC + VirtualGL installation complete!"
echo ""
echo "Quick start:"
echo "  1. Start TurboVNC server: vncserver"
echo "  2. Connect with TurboVNC viewer to :1"
echo "  3. Inside VNC, run OpenGL apps with: vglrun <application>"
echo ""
echo "For ZED camera in VNC, launch RViz/apps with: vglrun rviz2"
