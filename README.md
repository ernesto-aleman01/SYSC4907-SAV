### Simulated Autonomous Vehicle Engine

### Setting up the Environment

Windows was used for development. Using WSL is possible, but it is harder to set up. At time of writing, AirSim does not have
executables for MacOS and only some for Linux.

1. Install ROS noetic for windows. Follow the instructions here: http://wiki.ros.org/noetic/Installation/Windows

    > When following step 5, only step 5.1 is needed


2. Download an airsim release. These can be found here: https://github.com/Microsoft/AirSim/releases

   >Version 1.6 was used, but restrictions for not using other versions are not known
   >
   >7z is recommended for extracting if the scene consists of multiple zipped files (read "Downloading large environments" in a release section)
   
    >After extracting a sceen and trying to launch its executable, an error about not having DirectX installed may appear. In this case, DirectX runtime may have to be installed- this can be found here: https://www.microsoft.com/en-ca/download/confirmation.aspx?id=35
    > * Extra important: If installing DirectX, read each installation screen carefully, and make sure to uncheck "Install the Bing Bar"- you're better than that

3. ***Optional*** Install Pycharm IDE- this was used during development. Any environment that allows Python to be written works

### First Usage

1. Open up a ROS console, and navigate to the folder with the cloned repository.
   > Execute: catkin_make
   > 
   > Certain dependencies are needed. To get all of them, execute: pip install --no-deps -r dependencies.txt
   
    * Follow next step "Subsequent Usage"

### Subsequent Usage

1. Launch Airsim. A popup window may appear asking if a Car Simulation should be used. Click Yes.
    > In the top left corner, a line will say "Loaded settings from ...". This file is equivalent to the settings.json file in this repository. 
     Make sure that that file mentioned in AirSim is the same as the settings file in this repository.
    >
    > To run the simulation at a specific window simulation in windowed mode, the command line can be used.
      In the terminal, navigate to the AirSim executable and invoke it wth the following flags:  -ResX=width -ResY=height -windowed
    >
    > For example: AirsimNH -ResX=640 -ResY=480 -windowed 

2. Open up a ROS console, and navigate to the folder with the cloned repository.
    > ***(Any time a new ROS node is added)*** Execute: catkin_make
    >
    > ***(Required everytime ROS console is opened)*** Execute: .\devel\setup.bat
    >
    > Execute: roslaunch central_control full_system.launch
    > 
    > There is a chance of getting this message in the ROS console: WARNING:tornado.general:Connect error on fd 1372: WSAECONNREFUSED.
     To fix this, search the code for "airsim.CarClient(ip=host_ip)" and replace it with "airsim.CarClient()" .