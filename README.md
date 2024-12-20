# Realistic robot error generation - ROS Package

- [Realistic robot error generation - ROS Package](#realistic-robot-error-generation---ros-package)
  - [Usage](#usage)
  - [Setup](#setup)
    - [Python dependencies](#python-dependencies)
    - [Install in ROS2 worskpace](#install-in-ros2-worskpace)
    - [Install in ROS1 workspace](#install-in-ros1-workspace)
  - [Citation](#citation)

## Usage

Run error generation server
```
ros2 run realistic_error_generation error_generator_srv
```

Run example client
```
ros2 run realistic_error_generation example_client 
```

Using the error server from the command line (NOT WORKING YET)
```
ros2 service call /ObtainError realistic_error_generation_interfaces/srv/ObtainError
```

## Setup

### Python dependencies

To keep things clean, install pytorch dependencies in a virtual environment. 

```bash
python3 -m venv ~/venvs/pytorch_ros
source ~/venvs/pytorch_ros2/bin/activate
# pytorch dependency - https://discuss.pytorch.org/t/installing-pytorch-under-python-3-8-question-about-networkx-version/196740 
pip install networkx==3.1
# Last compatible version of pytorch with python 3.8 is 2.0.0
pip install torch==2.0.0 torchvision==0.15.1 torchaudio==2.0.1 --index-url https://download.pytorch.org/whl/cu118
```

Close this terminal after finishing.

### Install in ROS2 worskpace 

Create your ROS 2 workspace and clone all repositories using `vcs`:

```bash
source /opt/ros/galactic/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
vcs import --input https://raw.githubusercontent.com/surgical-robotics-ai/realistic_error_generation/refs/heads/main/ros2-error-generation-devel.repos.yaml  
```

Compile using `colcon`

```bash
cd ~/ros2_ws
colcon build 
source ~/ros2_ws/install/setup.bash
## Make sure you can find your pytorch dependencies
export PYTHONPATH=~/venvs/pytorch_ros/lib/python3.8/site-packages:$PYTHONPATH
```

### Install in ROS1 workspace

TODO

## Citation

```
@misc{barragan2024improving,
      title={Improving the realism of robotic surgery simulation through injection of learning-based estimated errors}, 
      author={Juan Antonio Barragan and Hisashi Ishida and Adnan Munawar and Peter Kazanzides},
      year={2024},
      eprint={2406.07375},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
