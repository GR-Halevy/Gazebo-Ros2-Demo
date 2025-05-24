# Gazebo-ROS2 Demo


## Installation Guide

### 1. Recommended Environment

* **ROS 2:** Jazzy
* **Gazebo:** Harmonic (LTS)
* **OS:** Ubuntu 24.04

---

### 1.1 Recommended to add to bashrc
```bash
# Sourcing ROS2 environment
source /opt/ros/jazzy/setup.bash

# Using GPU rendering
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# Model directory, gazebo will look here for model references in .sdf (references like this: <uri>model://X1_S/</uri>)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/your/models/
```
---
### 2. Install Gazebo Harmonic

Follow instructions from the official [Gazebo Harmonic installation guide](https://gazebosim.org/docs/harmonic/install_ubuntu/), or use the steps below:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic
```

---

### 3. Install `ros-gz` Packages
Where ${ROS_DISTRO} is recommended to be 'jazzy'
```bash
echo ${ROS_DISTRO}
jazzy
```
```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

Official guide: [ROS-Gazebo Integration](https://gazebosim.org/docs/harmonic/ros_installation/)

---

### 4. Install ROS 2 Jazzy on Ubuntu 24.04

Follow the [official ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), or use the summary below:

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# Add repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-dev-tools
sudo apt install ros-jazzy-desktop
```

---

## Additional useful Resources

* Gazebo Plugins & API: [https://gazebosim.org/api/sim/9/index.html](https://gazebosim.org/api/sim/9/index.html)
* SDF Format Documentation: [http://sdformat.org/spec](http://sdformat.org/spec)

---
