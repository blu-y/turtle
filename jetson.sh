sudo apt update
sudo apt install vim -y
sudo vi /etc/apt/sources.list +%s/ports.ubuntu.com/ftp.kaist.ac.kr +wq!
sudo apt update
sudo apt install python3-pip libopenblas-dev axel -y
sudo apt install firefox -y
pip install --upgrade pip
sudo -H pip install -U jetson-stats
pip install numpy=='1.26.1'
wget https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl -O torch-2.3.0-cp310-cp310-linux_aarch64.whl
wget https://nvidia.box.com/shared/static/9agsjfee0my4sxckdpuk9x9gt8agvjje.whl -O torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl
wget https://nvidia.box.com/shared/static/xpr06qe6ql3l6rj22cu3c45tz1wzi36p.whl -O torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev libomp-dev libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev -y
pip3 install 'Cython<3'
pip install torch-2.3.0-cp310-cp310-linux_aarch64.whl torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl
echo "For JetPack 6(L4T R36.2/R36.3) + CUDA 12.2 ONLY)"
echo "Else Follow this link to install pytorch on jetson"
echo "https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
echo "export PATH=$HOME/.local/bin:\$PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
wget https://github.com/rustdesk/rustdesk/releases/download/1.2.3-2/rustdesk-1.2.3-2-aarch64.deb
sudo apt install -fy  ./rustdesk-1.2.3-2-aarch64.deb
sudo systemctl enable rustdesk


sudo apt update
sudo apt upgrade -y
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
sudo apt install python3-colcon-common-extensions ros-humble-image-transport-plugins ros-humble-teleop-twist-keyboard ros-humble-tf-transformations -y
printenv | grep -i ROS_DISTRO
sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python -y
sudo rosdep init
rosdep update

echo "" >> ~/.bashrc
echo "# colcon_cd Setting " >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "" >> ~/.bashrc


echo "alias eb='gedit ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias pull='git pull'" >> ~/.bashrc
echo "alias push='git add . && today=\`date +%m%d\` && git commit -m \"\$today\" && unset today && git push origin'" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "" >> ~/.bashrc
source ~/.bashrc

echo "" >> ~/.bashrc
echo "" >> ~/.bashrc
eval "$(cat ~/.bashrc | tail -n +10)"
echo "Type 'source ~/.bashrc' to apply settings"
