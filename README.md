# IRLab Interfaces

ROS Metapackage for robot interfaces in Intelligent Robotics Lab at School of Computer Science, University of Birmingham, UK.

## Setup Instructions

### Docker setup (recommended method)

See instructions in the [install](https://github.com/IRLabUoB/irlab_interfaces/tree/install) branch.

### Local installation

If you already have a ROS Melodic environment:

1. Install the following dependencies:
   1. `libfranka` and `franka_ros` (for melodic: `sudo apt install ros-melodic-franka-ros ros-melodic-libfranka`)

2. Clone this repository to the `src` directory of your catkin workspace (`git clone -b melodic-devel https://github.com/IRLabUoB/irlab_interfaces`).

    - Steps 3 and 4 can be automated by running `./build_ws.sh` from `<catkin_ws>/src/irlab_interfaces`.

3. Update dependency packages:

    ```bash
        wstool init
        wstool merge irlab_interfaces/dependencies.rosinstall
        wstool up

        cd .. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```

4. Once the dependencies are met, the package can be installed using catkin:

```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)
    source devel/setup.bash
```
