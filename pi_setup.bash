sudo apt update -y && sudo apt upgrade -y

# ROS2 Installation
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update -y && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update -y && sudo apt install ros-dev-tools -y

sudo apt update -y && sudo apt upgrade -y
sudo apt install ros-jazzy-ros-base -y

echo "source /opt/ros/jazzy/setup.bash" >>~/.bashrc

# pigpio Installation
cd ~
git clone https://github.com/joan2937/pigpio.git
cd pigpio
git checkout v79
make
sudo make install

echo "[Unit]
Description=Daemon required to control GPIO pins via pigpio
[Service]
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill -s SIGKILL pigpiod
Type=forking
[Install]
WantedBy=multi-user.target" | sudo tee /lib/systemd/system/pigpiod.service >/dev/null

sudo systemctl enable pigpiod.service
sudo systemctl start pigpiod.service

# Install git lfs
sudo apt install git-lfs -y

# Install required ROS packages
sudo apt install ros-jazzy-camera-ros ros-jazzy-apriltag-ros ros-jazzy-xacro


# Clone git repo
cd ~
git config user.email "iyer.nikhil@ufl.edu"
git config user.name "Nikhil Iyer"
git clone https://github.com/sskonda/IEEE-Hardware-Team-2024-2025.git
