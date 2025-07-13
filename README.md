# TurtleBot-Perception-and-Navigation-System

## Prerequisites

- ROS 2 Humble
- Python 3 
- OpenCV
- PyTorch
- Torch
- Numpy
- Time
- OS

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
