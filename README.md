# TurtleBot-Perception-and-Navigation-System

## Prerequisites

- ROS 2 Humble
- Python 3 
- OpenCV
- PyTorch
- Torch
- Numpy
- Time
- OS(ubuntu 22.04 preferred else WSL2)

## 🔧 Setup Instructions

### 1. Clone the Repository

```bash
mkdir -p ros_perception_ws/src
cd ~/ros_perception_ws/src

git clone https://github.com/jonathandeepak/TurtleBot-Perception-and-Navigation-System.git .

cd ~/ros_perception_ws

# build and install the package
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash 
colcon build --symlink-install 

```

### 2. Launch Gazebo World

```bash
source install/setup.bash
source  /usr/share/gazebo/setup.bash      ## this step may not be needed
ros2 launch enpm673_final_proj enpm673_world.launch.py "verbose:=true"

```

### 3. Run the Python Node
Open a new terminal and run the below command.
```bash
cd ~/ros_perception_ws
export TURTLEBOT3_MODEL=waffle

source install/setup.bash
ros2 run enpm673_final_proj enpm673_final_proj_main.py
```
## Output
- Gazebo implementation
![Gazebo](Gazebo_implementation_gif.gif)
- Real World Implementation
![Real_world](Real_world_implementation_gif.gif)
- Stop Sign detection
- 
![Stop sign](Stop_sign_picture.jpg)
- Horizon line detection
- 
![horizon line](horizon_line_image.png)
- Optical flow

![optical flow](Optical_flow_picture.jpg)

## System flow
![system flow](System_flow_picture.png)

---

## 🔧 Setup Instructions for running the Simulation in Docker🐳
## 1. Create a Docker container

Download **src.zip** and **dockerfile** from docker branch into the same directory

Build the docker image
```bash
docker build -t enpm673 .
```
Run the Container with GUI support inside WSL **(Preferred)** or Ubuntu Desktop

```bash
xhost +local:root
```
```bash

docker run --rm -it --net=host --gpus all -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix enpm673
```

Run the Container with GUI support using Docker desktop + Xming server setup (for windows)
```bash
docker run --rm -it --gpus all -e DISPLAY=host.docker.internal:0 enpm673
```


### 2. Launch the simulation
Launch Terminator
```bash
terminator
```
**Vertical Split (side-by-side)** | `Ctrl` + `Shift` + `E` |

**Horizontal Split (top and bottom)** | `Ctrl` + `Shift` + `O` |

Terminal 1 – Launch Gazebo simulation:
```bash
ros2 launch enpm673_final_proj enpm673_world.launch.py "verbose:=true"
```
Terminal 2 – Run the perception and navigation script
```bash
ros2 run enpm673_final_proj enpm673_final_proj_main.py
```
