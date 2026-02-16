# robocomp-INSIGHT

## 1. Project introduction

This repository is intended for the code related to the INSIGHT project. This project, carried out by different universities across Spain, aims to enable a robot to identify objects in its environment that could pose an obstacle to its proper functioning.

To achieve this functionality, different Agents have been developed that carry out a task flow as explained below:

#### Basic mission

The Mission Control Agent establishes the mission to follow a person. For this, the Agent sets the robot's position **(with respect to the room?)** in the working memory, as well as the IMU measurements of the robot itself.

#### Internal simulation

The same Mission Control Agent also carries out an internal simulation to represent what the robot believes the robot is going to do and to be able to sense when the real robot has not had the expected behavior, which implies that it has encountered some kind of obstacle to its mission.

#### Robot knowledge - Ontologies

Work in progress...

#### Simulation of problematic events

When the robot finds that its behavior in the simulation has not been the same as the behavior in the real world, it generates different causes based on what it knows. Once the list of possible causes is established, the Mission Control Agent stops the mission to try to simulate the possible causes to try and find the real cause of the problem.

#### Obstacle detection

Once the cause of the problem is detected, the robot will try to build a detector for the obstacle it has encountered, so that it does not pose a problem for it again in the future.


## 2. Environment and dependencies

### Main libraries
* **RoboComp**: Robotics framework created at Robolab
* **Webots**: Real-time physics simulation 
* **PyBullet**: Real-time physics simulation
* **NumPy**: Numerical operations
* **SciPy**: Scientific computing
* **Pandas**: Data manipulation
* **Matplotlib**: Data visualization
* **Program-manager**: Monitoring the execution process

## 3. Installation

### RoboComp installation

Download and execute the installation script:

```bash
wget https://raw.githubusercontent.com/robocomp/robocomp/development/robocomp_install.sh && bash robocomp_install.sh
```

Install the main repository and the others necessaries repositories:

```bash
cd $ROBOCOMP/components
git clone https://github.com/jcalderon12/robocomp-insight.git
cd $ROBOCOMP/components/robocomp-insight
vcs import $ROBOCOMP/components < insight.repos --recursive
```
### Python packages

Install the dependencies from the requirements file:

```bash
pip install -r requirements.txt
```

### Program manager installation and configuration

Clone the repository in the software dir:
```bash
mkdir -p ~/software
cd ~/software
git clone https://github.com/alfiTH/Program-manager.git
```

Install the dependencies
```bash
pip install -r requirements.txt
#Codium download
mkdir ~/software 2> /dev/null; cd ~/software && wget https://github.com/VSCodium/vscodium/releases/download/1.109.21026/vscodium-reh-web-linux-x64-1.109.21026.tar.gz && mkdir vscodium-server && tar -xzf vscodium-reh-web-linux-x64-1.109.21026.tar.gz -C vscodium-server && cd -
```

Create a SSL certificate necesary to this software:
```bash
cd ~/software/Program-manager/certificates
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
```

Create a new user (**This user must be used in the future to login in the software**):
```bash
cd ~/software/Program-manager
python src/utils/addUser.py
```

Copy the config file from the INSIGHT project:
```bash
cp $ROBOCOMP/components/robocomp-insight/insight_config.json ~/software/Program-manager/etc/
```

## 4. Usage

To launch all the enviroment just execute Program manager with the INSIGHT configuration file:

```bash
cd ~/software/Program-manager
python src/ProgramManager.py --config etc/insight_config.json
```

## DSR Graph Index

### Nodes
___

- **ROOT** - This node does not have much functionality; it simply ensures that the graph is never empty and causes problems as a result. 
- **ROBOT** - This node represents the state of the robot, although it only contains the target speed at which the robot will attempt to move, both linearly and angularly.
- **BOTTLE** - This node represents the bottle in the graph and does not contain any relevant information within it.
- **PERSON** - This node represents the person in the graph and does not contain any relevant information within it.
- **_FOLLOW_ME_** - This node represents a possibility of the node. This node has a **Boolean** variable to indicate whether the possibility is active or paused.
- **IMU** and **IMU_SINTETIC** - These nodes represent the robot's two IMUs, the first being the "real" IMU from Webots and the second being the "synthetic" IMU from the internal simulator, Pybullet.

### Edges
___

- **RT** - These types of edges represent a spatial difference between nodes and therefore have attributes to represent that spatial difference. They are sometimes used generically to establish two nodes, for example, in the case of the root node.
- **TARGET** - These types of edges represents the robot's objective, or what it has focused its attention on.
- **HAS** - These types of edges represent that the node from which they originate has something and nothing else.
- **HAS_INTENTION** - These types of edges represent that the robot has some intention regarding another node.