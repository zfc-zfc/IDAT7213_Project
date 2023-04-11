# IDAT Project

## 1 Introduction of FAST-LIO1&2

**FAST-LIO** is a fast, robust, and versatile LiDAR-inertial odometry framework. Building on a highly efficient tightly coupled **iterated extended Kalman filter**, FAST-LIO can achieve high-accuracy localization and mapping. 

**FAST-LIO2** is the updated version of FAST-LIO with two key novelties. The first one is directly registering raw points to the map without extracting features. The second main novelty is maintaining a map by an incremental k-dimensional (k-d) tree data structure, incremeInntal k-d tree (ikd-Tree), that enables incremental updates and dynamic rebalancing. These two novelties make FAST-LIO2 be faster and more robust than previous FAST-LIO.

**Related video:**  the accompanying videos are now available on **YouTube** (click below images to open)

<div align="left">
    <a href="https://youtu.be/iYCY6T79oNU" target="_blank">
    <img src="image/FAST-LIO1.png" width=45% />
    <a href="https://youtu.be/2OvjGnxszf8" target="_blank">
    <img src="image/FAST-LIO2.png" width=45% />
</div>

**Related papers**: 

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

## 2 Project Mission

* Successfully run the given code of FAST-LIO2 on Linux system (i.e., Ubuntu20.04) via virtual machine.
* Modify some codes to acquire correct odometry output (The code provided will output incorrect result).
* Submit specific files (screenshots, pcd files and source code).

## 3 Preparation

### 3.1  Virtual Machine and Ubuntu Installation

FAST-LIO2 only works on Linux system which supports ROS (Robot Operating System). Thus we need to install a Linux system to make FAST-LIO2 work. Compared with installing a real linux system, here we choose an easier and safer way: installing a **virtual machine** of linux system on your windows system. A **virtual machine** (**VM**) is the [virtualization](https://en.wikipedia.org/wiki/Virtualization)/[emulation](https://en.wikipedia.org/wiki/Emulator) of a computer system. Virtual machines are based on computer architectures and provide functionality of a physical computer. 

* Download **Ubuntu20.04 mirror image** on [onedrive](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/zhufc_connect_hku_hk/ESz5E8X7Q09Ksr8d21LfO2UBSLpx7-_z3B7pXbfmMhs4ow?e=0Z4QCa).

* Download VMware Workstation Player for free at https://www.vmware.com/products/workstation-player.html.

  <div align="center"><img src="image/Player1.png" width=70% /></div>

* Select version 16.0 then click **GO TO DOWNLOADS**

  <div align="center"><img src="image/Player2.png" width=70% /></div>

* Double click the installation file and start installation, then click next

<div align="center"><img src="image/Picture1.png" width=50% /></div>

* Accept the license and click next

  <div align="center"><img src="image/Picture2.png" width=50% /></div>

* Select the installation path and then click next

  <div align="center"><img src="image/Picture3.png" width=50% /></div>

* Select the default user experience settings and the click next.

  <div align="center"><img src="image/Picture4.png" width=50% /></div>

* Create shortcuts and then click next.

  <div align="center"><img src="image/Picture5.png" width=50% /></div>

* Ready to install VMware Workstation Player, then click Install.

  <div align="center"><img src="image/Picture6.png" width=50% /></div>

* Wait for the installation finish.

  <div align="center"><img src="image/Picture7.png" width=50% /></div>

* After installation finish, click "finish".

  <div align="center"><img src="image/Screenshot_2.png" width=50% /></div>

* Double click **VMware Workstation 16 Player** we just installed, then we select "use it for non-commercial use".

  <div align="center"><img src="image/Activation.png" width=40% /></div>

* The installation of VMware Workstation Player is all completed, now let's create a new virtual machine.

  <div align="center"><img src="image/Picture8.png" width=60% /></div>

* Select [Install the operating system later below] and click [next]

  <div align="center"><img src="image/Picture10.png" width=40% /></div>

* Select the operating system and version as shown below.

  <div align="center"><img src="image/Picture11.png" width=40% /></div>

* Name the system and select the storage location of the virtual machine (it is not recommended to put it on the system disk, i.e., C disk).

  <div align="center"><img src="image/path.png" width=40% /></div>

* Specify the disk capacity. Here it is recommended that give at least 30 GB for ubuntu system.

  <div align="center"><img src="image/disk.png" width=40% /></div>

* Customize hardware

  <div align="center"><img src="image/hardware.png" width=40% /></div>

* Select the Ubuntu mirror image (**ubuntu-20.04.5-desktop-amd64.iso**) file, then close and finish

  <div align="center"><img src="image/iso.png" width=50% /></div>

* Click "Ubuntu 64-bit" button and then click "play virtual machine".

  <div align="center"><img src="image/start.png" width=50% /></div>

* Wait for the loading of ubuntu20.04.

  <div align="center"><img src="image/ubuntu_install.png" width=50% /></div>

* Start installing ubuntu 20.04.

  <div align="center"><img src="image/start_install.png" width=50% /></div>

* Select English and then click "continue". Due to an incorrect resolution setting on the virtual machine, the window display is not fully visible. We will adjust the resolution later to resolve this issue.

  <div align="center"><img src="image/language.png" width=50% /></div>

* Select "normal installation" and continue.

  <div align="center"><img src="image/normal_install.png" width=50% /></div>

* Select "Erase disk and install Ubuntu" and click "install now".

  <div align="center"><img src="image/erase.png" width=50% /></div>
  <div align="center"><img src="image/eras1.png" width=50% /></div>

* Select Hong Kong SAR and continue.

  <div align="center"><img src="image/HK.png" width=50% /></div>
  
* Set your account, fill in personal information and click "continue".

  <div align="center"><img src="image/ID.png" width=50% /></div>

* Wait for installation.

  <div align="center"><img src="image/WAIT.png" width=50% /></div>

* Restart the virtual machine.

  <div align="center"><img src="image/restart.png" width=50% /></div>

* If you see this, it means the installation of ubuntu is completed.

  <div align="center"><img src="image/finish.png" width=50% /></div>

* Adjust resolution: Click on the downward-facing triangle in the upper right corner of the screen, and then click on "Settings".

  <div align="center"><img src="image/resolution1.png" width=60% /></div>

* Click on "Displays" and then change the resolution to 2560 x 1440, and then click on "Apply".

  <div align="center"><img src="image/resolution2.png" width=60% /></div>







### 3.2 Dependencies Installation

#### 3.2.1 Ubuntu and ROS

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO2 to work normally. ROS >= Melodic. 

Here we choose Ubuntu 20.04, so the corresponding ROS version is Noetic.

* First we should open a new terminal by pushing **Ctrl + Alt + T** at the same time.

<div align="center"><img src="image/Terminal.png" width=60% /></div>

* Update Ubuntu Source: Type the following commands in the terminal and push "Enter".

  ```
  # Backup original source
  cd /etc/apt
  sudo mv sources.list sources.list.backup
  
  # Use new source Aliyun
  sudo gedit sources.list
  ```
  
  Copy the following contents into sources.list and save.
  
  ```
  deb http://mirrors.aliyun.com/ubuntu/ focal-backports main multiverse restricted universe
  deb http://mirrors.aliyun.com/ubuntu/ focal-proposed main multiverse restricted universe
  deb http://mirrors.aliyun.com/ubuntu/ focal-security main multiverse restricted universe
  deb http://mirrors.aliyun.com/ubuntu/ focal-updates main multiverse restricted universe
  deb-src http://mirrors.aliyun.com/ubuntu/ focal main multiverse restricted universe
  deb-src http://mirrors.aliyun.com/ubuntu/ focal-backports main multiverse restricted universe
  deb-src http://mirrors.aliyun.com/ubuntu/ focal-proposed main multiverse restricted universe
  deb-src http://mirrors.aliyun.com/ubuntu/ focal-security main multiverse restricted universe
  deb-src http://mirrors.aliyun.com/ubuntu/ focal-updates main multiverse restricted universe" > /etc/apt/sources.list'
  ```
  
  <div align="center"><img src="image/source.png" width=100% /></div>
  
  **Then update source**: Type the following command in the terminal and push "Enter".
  
  ```
  cd ~sudo apt update
  ```
  
* Then we copy the following commands in the terminal, and then push **Enter** to be ready to install ROS

  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl git
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
  wget http://packages.ros.org/ros.key
  sudo apt-key add ros.key
  sudo apt update --fix-missing
  ```

  Update software

  <div align="center"><img src="image/open_software.png" width=80% /></div>

  <div align="center"><img src="image/software_update.png" width=80% /></div>

  <div align="center"><img src="image/software_update1.png" width=80% /></div>

  <div align="center"><img src="image/software_update2.png" width=80% /></div>

  <div align="center"><img src="image/software_update3.png" width=80% /></div>

* **Install ROS-noetic** (Long Time Waiting)

  ```
  sudo apt install ros-noetic-desktop-full -y
  ```

* After installation finish, 

  ```
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

* Then we need to check if the installation was successful:

  Open a new terminal and type "roscore", if you see the following scene, the installation of ROS is completed.

  <div align="center"><img src="image/checkROS.gif" width=50% /></div>

* Source ROS 

  Open a new terminal and type "**sudo gedit ~/.bashrc**", then add the following command and then **save the file**.

  ```
  source /opt/ros/noetic/setup.bash
  ```

  <div align="center"><img src="image/sourceROS.png" width=70% /></div>

#### 3.2.2 PCL

Then we copy the following commands in the terminal, and then push **Enter** to install PCL

```
sudo apt install ros-noetic-pcl* -y
sudo ln -s /usr/include/pcl-1.10/pcl /usr/include/pcl
```

#### 3.2.3 Livox SDK Installation

(Follow https://github.com/Livox-SDK/Livox-SDK)

### 3.3 Build The Project

Open a new terminal: 

```
mkdir ~/fastlio2_ws
cd fastlio2_ws && mkdir src
cd src
git clone https://github.com/zfc-zfc/IDAT7213_Project.git
cd ..
catkin_make -j
source devel/setup.bash
```
### 3.4 Run with Rosbag 

The simulation.bag can be downloaded from [onedrive](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/zhufc_connect_hku_hk/EbF01otEef5PsFQ7fD-d6b4BKtsJzAaz6XUrwuqj4Hr12g?e=fsOtEI).

First write your **student ID** and **Name** like the following picture (for example, ID is 3035888888, then you should write as 3035888888.0).

 <div align="center"><img src="image/ID_Name.png" width=70% /></div>



Run:
```
cd ~/fastlio2_ws
source devel/setup.bash
roslaunch fast_lio2 simulation.launch
```

Open a new terminal, 

```
cd ~/Downloads # The path where simulation.bag exists
rosbag play simulation.bag
```

You will see a drift odometry and a messy point cloud map like this:

 <div align="center"><img src="image/WrongMap.png" width=70% /></div>





## 4 Code Modification & Result Submission

### 4.1 Code Modification

According to the course slides, try to modify the code (line 1040 to line 1049) in **IDAT7213_Project/FAST-LIO2/src/laserMapping.cpp**, and acquire **correct odometry and consistent point cloud map**.

 <div align="center"><img src="image/Code.png" width=70% /></div>

### 4.2 Result submission

#### 4.2.1 PCD File

Please save the pcd file by setting **pcd_save_en** to **true**, All the scans (in global frame) will be accumulated and saved to the file ``` fast_lio2/PCD/Map.pcd ``` after the FAST-LIO2 is terminated. Use ```pcl_viewer Map.pcd``` can visualize the point clouds.

```
sudo apt install pcl-tools
```

#### 4.2.2 Screenshots

The first screenshot is the consistent point cloud map in Rviz:

 <div align="center"><img src="image/rviz.png" width=70% /></div>

The second screenshot is the terminal (the following is an example).

 <div align="center"><img src="image/terminal1.png" width=40% /></div>

#### 4.2.3 Source Code

Compress your whole package "FAST-LIO2" and submit.

 <div align="center"><img src="image/Code.gif" width=50% /></div>
