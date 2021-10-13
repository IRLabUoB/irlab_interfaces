# IRLab Interfaces

Installation instructions for IRLab Interfaces (IRLI). Note that this branch is only for installation scripts and utilities to build the irlab_interfaces packages in a docker environment.

## Local installation

To install locally (without docker), see instructions in the corresponding branches (e.g. `main`).

## Setting Up

0. If your network is behind a proxy, make sure the instructions [here](https://gist.github.com/justagist/7b544626136537774961c5c5f563d18d) are followed for proper installation of the IRLab Interfaces Docker Image.

1. Install Docker CE (v >= 19.0), see [installation instructions](https://docs.docker.com/engine/installation/).

   * Also perform the [post installation instructions](https://docs.docker.com/engine/installation/linux/linux-postinstall/), so that docker can be run without requiring root privileges by a non-root user. (this is optional but recommended, otherwise scripts will have to be run as root)

2. (optional) If you have an NVidia GPU and want to use GPU acceleration/CUDA in docker (required mainly for tensorflow and for boosting some visualisers), follow [these instructions](#irli-docker-cuda-preinstallation-setup) before setting up the IRLI Docker. If not, go to [Building IRLI Docker](#building-irli-docker).

### IRLI Docker CUDA Preinstallation Setup

You need to have nvidia graphics card. Skip this section if you don't.

1. Install nvidia driver (>= nvidia-418)

   * Recommended method:

   ```bash
   sudo add-apt-repository ppa:graphics-drivers/ppa
   sudo apt update

   ```

   Then, on Ubuntu from the menu / Dash, click on the "Additional Drivers" and on the tab with the same name, select the driver you want to use, and "Apply changes". Wait until the driver is downloaded and installed, and reboot.

2. Install [nvidia container toolkit](https://github.com/NVIDIA/nvidia-docker):

   ```bash
   # Add the package repositories
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

   sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
   sudo systemctl restart docker
   ```

3. Install [nvidia-docker2](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)):

   ```bash
   sudo apt-get install nvidia-docker2
   sudo pkill -SIGHUP dockerd

   ```

Once completed, go to [Building IRLI Docker](#building-irli-docker).

### Building IRLI Docker

The default (recommended) options will be applied and the library should be fully installed automatically by running one of the following commands. It sets up a docker image with ubuntu 18.04, (optionally GPU acceleration,) ROS melodic (Python 2) and all other required dependencies for IRLI. It creates a default catkin workspace located at `$HOME/Projects/irlab_ws`. The host machine **does not** need to have ROS installed. It only needs to have docker and (optionally) nvidia-docker as described above.

NOTE: `sudo` privileges and `github` authentications may be required during installation.

```bash
# Without CUDA:
bash -c "$(curl -fsSL https://raw.githubusercontent.com/IRLabUoB/irlab_interfaces/install/install.sh)" melodic

# With CUDA (only if you have followed steps mentioned in 'irlabdocker CUDA Preinstallation Setup'):
bash -c "$(curl -fsSL https://raw.githubusercontent.com/IRLabUoB/irlab_interfaces/install/install.sh)" melodic-cuda
```

After running the script line above you should be able to see a new image set up on your docker and tagged as `dev:melodic-cuda` or `dev:melodic`. You should be able to list it by running `docker images`. Go to [Using the Container](#using-the-container) section for instructions to use the docker container.

#### Advanced Installation Options (for developers)

`bash -c "$(curl -fsSL https://raw.githubusercontent.com/IRLabUoB/irlab_interfaces/install/install.sh)" <optional keyword arguments> [build-name]`

Keyword arguments:

```bash
  --workspace <absolute path> : path to create IRLab catkin workspace (Default: $HOME/Projects/irlab_ws)
  --branch <branch name> : IRLab Interfaces git branch to use (default 'melodic-devel')
 ```

You can choose other docker builds. See list below:

* melodic-cuda
* melodic

<!-- * melodic-cuda-python3
* melodic-python3 -->

### Using the container

Now in the IRLI docker folder located at `$HOME/Projects/irlab_ws/src/irlab_interfaces/dockerstuff/scripts` you will find a set of scripts that will help you run your docker container, among other examples. For instance, if you want to open a bash shell to the docker container just built, then execute or source the script:

`$HOME/Projects/irlab_ws/src/irlab_interfaces/dockerstuff/scripts/bash.sh dev:<NAME_OF_DOCKER_BUILD_CHOSEN>`
(NOTE: image tag is passed as argument to the script).

This should open a bash shell and spin the container. Check if RVIZ is running in the container by running in the bash shell opened:

```bash
roscore > /dev/null &
rosrun rviz rviz
```

This should open an X window on your host machine with RVIZ. If you installed the CUDA variant of IRLI Docker, this means your docker is being hardware accelerated and able to use the host GPU for 3D rendering.

The container will close once the session exits (`exit` command or `CTRL+D`).

To open another terminal session into an existing container, open a new terminal and run `./exec_container $(./get_containerId.sh)`. The container will not close when this session exits. (Note: All terminal sessions will be closed if the main (parent) container is stopped.) Multiple terminal sessions can be opened in this fashion.

Alternatively, setup aliases for easier loading of image and opening new terminal session by adding the following lines to your '.bashrc' file:

```bash
# Alias to start a new IRLI Docker
function irlabdocker () { $IRLI_DIR/dockerstuff/scripts/bash.sh irli:<NAME_OF_DOCKER_BUILD_CHOSEN>; cd $IRLI_DIR/../../; }

# Alias to connect to an existing IRLI Docker image instance (multiple terminals can be opened like this, but only one image instance should be running).
function newdockterm (){ $IRLI_DIR/dockerstuff/scripts/exec_container.sh $($IRLI_DIR/dockerstuff/scripts/get_containerId.sh); cd $IRLI_DIR/../../; }
```

To exit any container, use `exit` command, or `CTRL+D` keys.

Any files written to `$HOME/Projects/` or other directories mounted inside the container will be observable from the host machine as well as the container. The intention is that development can be done on the host machine, but run inside the container.

### In-Docker Instructions

Start IRLI Docker by running `irlabdocker`, unless it is already running.

If your computer is in the `robot network`, update the files `intera.sh`,`franka.sh` and `baxter.sh` with your IP. By running `source <file_name>` using either of these files, you will be connected to the robot (for the Franka robot, make sure you have followed the instructions from the [franka_ros_interface](https://github.com/justagist/franka_ros_interface) repository for connecting to the robot).

Some of the external simulators (for franka, sawyer, baxter) can be used regardless of whether you are in the robot network. All the above files can be run using the `sim` argument to enter the simulation environment for the robots (eg: `source intera.sh sim`). All simulation environments are same and will simply start a ROS environment locally.

Test the robot simulators using any of the below commands:

```bash
roslaunch sawyer_gazebo sawyer_world.launch  # for sawyer robot
roslaunch baxter_gazebo baxter_world.launch  # for baxter robot
roslaunch panda_gazebo panda_world.launch    # for panda robot

```

These simulators should each mimic the behaviour of the real robot. This can be tested by running `rostopic list` in another terminal (open new terminal, run `newdockterm`). All (or most of the) ROS topics published and subscribed by the real robot should be listed for the simulated robot as well. These can be accessed using the ROS functionalities, or the interfaces and scripts in IRLI, just like the real robot.

## Important Notes on Robot and Software Versions

IRLab Interfaces is written for specific versions of the robot firmware and driver software. See information in the appropriate IRLab Interface Github branch.
