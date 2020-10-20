# pyPhysX: MABA (SPD) test env

[project page](https://arpspoof.github.io/project/spd/spd.html), 
[paper](https://www.cs.sfu.ca/~kkyin/papers/fastSPD.pdf), 
[demo](https://www.cs.sfu.ca/~kkyin/papers/fastSPD.mp4), 
[talk](https://www.youtube.com/watch?v=wo7cK-DClhw)

### Test system
Our tests are performed on Archlinux. CMake is used to build the system. The system should generally work on other linux systems as well. On Windows there are some dependency issues. 

### PhysX setup
- Clone official repo from [github](https://github.com/NVIDIAGameWorks/PhysX). Make sure the version is 4.1.1.27006925. Since PhysX usually change low level code during upgrades, other versions will most likely not work. 
- Apply MABA-on-PhysX4.1.1.27006925.patch
- Follow [PhysX build instructions](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/BuildingWithPhysX.html) to build binary files and install headers. After this procedure, you should have ```---/PhysX/physx/bin/linux.clang/release``` and ```---/PhysX/physx/install/linux```

### Simulation rendering setup
By default, a built-in renderer is used which is ugly. Press ```Space``` key to start simulation and use mouse drag and ```WASD``` to adjust view port. No other setup is required if you use this renderer.

If you would like to achieve the rendering effect in the paper or video, please first set up the [Unity renderer](https://github.com/arpspoof/UnityKinematics). Detailed documentation and tutorials are available. Then enable ```ENABLE_UNITY_KINEMATICS``` CMake option when configuring this project. Search for ```UR_Init``` in the code to change IP address and port required by Unity renderer (if needed). 

### Build
First create a folder named ```build```, then please use ```cmake-gui``` to build this project. We have included hints for how to configure this project. You will need to specify PhysX binary and installation path, Eigen path. You can basically leave other options as default. 
- Enable ```ENABLE_UNITY_KINEMATICS``` to use Unity renderer.
- Enable ```BUILD_API``` to get a simple python binding of PhysX character simulation.

### Run the SPD tests
In the ```build``` folder:

#### Quasi-physics tracking:
```
./poseTracking -m run.txt
```
- ```-m [mocap_file]```. Mocap files are in resources folder.
- Use ```-c 0/1/2/3``` option to specify SPD controller. 0 is MABA, 1 is sparse LLT, 2 is dense LLT (traditional), 3 is kinematic (for debugging).
- Use ```--dog``` to use the dog model.
- Use ```-N [x]``` to specify number of characters, use ```--column``` to specify number of columns to arrange these characters.
- Use ```-t [x]``` to specify simulation time step.
- Use ```-p``` to measure performance (no rendering).

#### Snake
```
./snake -L 48
```
- Use ```-L [x]``` to specify snake length

#### Other notes
- SPD components are in src/articulations/spd folder, jump here if you just need to look at the code. 
