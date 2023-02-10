# Code for RSA Hackathon (February 25, 2023)

## Requirements:
Make sure you have all the required packages/libraries and interpreters for the following:
- ROS (I used Noetic).
- C++.
- Python.

## Instructions:
In a terminal:
1. Clone this repository in any directory (here, in the home directory).
```bash
cd ~
git clone https://github.com/fcortevargas/hackathon.git
```
2. Create ROS workspace:
```bash
mkdir [workspace-name]
cd [workspace-name]/
mkdir src
cd src/
```
3. Create ROS package. For this, `qr_vision` was used in the `CMakeLists.txt` and `package.xml`:
```bash
catkin_create_pkg qr_vision
```
4. Move to ROS package directory and copy contents of repository:
```bash
cd qr_vision/
cp -r ~/hackathon/* .
``` 
6. Build package:
```bash
cd ../..
catkin_make
``` 
7. Make Python files executables.
```bash
chmod u+x src/qr_vision/src/*.py
```
8. Source bash setup configuration.
```bash
source devel/setup.bash
```
9. Run ROS node as needed. `[ros_node_name]` can be `publisher` (C++), `publisher.py` (Python), or `subscriber` (C++), or `subscriber.py` (Python).
```bash
[roscore] # In a different terminal
rosrun qr_vision [ros_node_name]
```
## Credits:
Code implemented by Fernando Corte Vargas.
