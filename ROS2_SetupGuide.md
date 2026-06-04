
# ROS 2 Setup Guide for Ubuntu 26.04 (WSL2)

This guide outlines the standard operating procedure for configuring a local ROS 2 development environment on Windows Subsystem for Linux (WSL2).

## Prerequisites

* Windows 11 with WSL2 installed.
* An active **Ubuntu-26.04** distribution running in WSL2.

## Step 1: System & Locale Setup

First, ensure your Ubuntu environment supports the UTF-8 character set, which ROS 2 requires to parse logs and package data correctly.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

```

## Step 2: Add ROS 2 Repositories

Enable the Ubuntu Universe repository and add the official ROS 2 GPG keys.

```bash
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe

```

```bash
# Download and install the ROS 2 apt source key
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb


```

```bash
sudo apt update
```
```bash
sudo apt upgrade
```

## Step 3: Install ROS 2 Lyrical Luth

Install the full desktop environment, which includes simulation and visualization tools like RViz2.

```bash
sudo apt install ros-lyrical-desktop -y

```

Add the ROS 2 environment variables to your bash profile so they load automatically:

```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

## Step 4: Critical WSL2 Fixes

Developing ROS 2 inside WSL2 introduces specific networking and middleware challenges. You **must** apply these fixes, or your nodes will fail to communicate.

### Fix 1: The Cyclone DDS Middleware

By default, ROS 2 uses Fast DDS, which relies on shared memory. In WSL2, abruptly stopping a node (`Ctrl+C`) often locks this memory, causing subsequent nodes to fail silently. **We replace Fast DDS with Cyclone DDS** to ensure stable local communication.

```bash
# Install Cyclone DDS for Lyrical
sudo apt install ros-lyrical-rmw-cyclonedds-cpp -y

# Make it the default middleware
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

```

### Fix 2: Windows Mirrored Networking

To allow WSL2 to communicate with external hardware on your Wi-Fi network via UDP multicast, you must enable Mirrored Mode in Windows.

1. In Windows, press `Win + R`, type `%USERPROFILE%`, and hit Enter.
2. Create or edit a file named `.wslconfig` and add:
```ini
[wsl2]
networkingMode=mirrored

```


3. Open a Windows PowerShell as Administrator and restart WSL:
```powershell
wsl --shutdown

```


## Testing Your Installation

To verify your environment is working perfectly, open two separate Ubuntu terminals.

**Terminal 1:**

```bash
ros2 run demo_nodes_cpp talker

```

**Terminal 2:**

```bash
ros2 run demo_nodes_py listener

```

If the listener successfully prints the "Hello World" messages from the talker, your ROS 2 Lyrical pipeline is correctly configured!