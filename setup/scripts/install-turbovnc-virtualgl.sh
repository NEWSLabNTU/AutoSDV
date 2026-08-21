#!/usr/bin/env bash
# Install TurboVNC and VirtualGL
# For hardware-accelerated VNC (useful for ZED camera in VNC sessions)
# Sources:
#   https://turbovnc.org/Downloads/YUM
#   https://virtualgl.org/Downloads/YUM

set -e
set -o pipefail  # a failed wget in `wget | gpg` pipes must abort, not be masked

echo "TurboVNC + VirtualGL Installation"
echo "=================================="
echo ""
echo "This installs TurboVNC and VirtualGL for hardware-accelerated"
echo "remote desktop sessions (required for ZED camera in VNC)."
echo ""

echo "Adding TurboVNC APT repository..."
# Add TurboVNC GPG key
wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    sudo gpg --dearmor --yes -o /etc/apt/trusted.gpg.d/TurboVNC.gpg

# Add TurboVNC repository
sudo wget -q -O /etc/apt/sources.list.d/TurboVNC.list \
    https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list

echo "Adding VirtualGL APT repository..."
# Add VirtualGL GPG key
wget -q -O- https://packagecloud.io/dcommander/virtualgl/gpgkey | \
    sudo gpg --dearmor --yes -o /etc/apt/trusted.gpg.d/VirtualGL.gpg

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
# Create a wrapper script for TurboVNC's vncserver.
# The vncserver script looks for Xvnc and other binaries relative to its
# working directory. We create a wrapper that changes to /opt/TurboVNC/bin/
# before executing the real vncserver.
TURBOVNC_WRAPPER="/usr/local/bin/turbovnc-vncserver"

sudo tee "$TURBOVNC_WRAPPER" > /dev/null << 'EOF'
#!/usr/bin/env bash
# Wrapper script for TurboVNC vncserver
# Changes to TurboVNC bin directory so vncserver can find Xvnc and other binaries
cd /opt/TurboVNC/bin
exec ./vncserver "$@"
EOF

sudo chmod +x "$TURBOVNC_WRAPPER"

# Register the wrapper with update-alternatives
# Install the alternative with high priority, then force selection
sudo update-alternatives --install /usr/bin/vncserver vncserver "$TURBOVNC_WRAPPER" 100
sudo update-alternatives --set vncserver "$TURBOVNC_WRAPPER"

echo ""
echo "TurboVNC + VirtualGL installation complete!"
echo ""
echo "Quick start:"
echo "  1. Start TurboVNC server: vncserver"
echo "  2. Connect with TurboVNC viewer to :1"
echo "  3. Inside VNC, run OpenGL apps with: vglrun <application>"
echo ""
echo "For ZED camera in VNC, launch RViz/apps with: vglrun rviz2"
