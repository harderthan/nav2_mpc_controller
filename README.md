> This is not official package for [**navigation2**](https://github.com/ros-planning/navigation2) 
  
# Nav2_MPC_Controller Plugin  

This is ros2 package and plugin for controller interface of Navigation2.You can use this plugin with controller server of navigation2 without any special implementation.  
  
![demo_video](asset/demo_video.gif)

### Installation

Follow the official [documentation](https://navigation.ros.org/build_instructions/index.html#install) below to install.
Requires Navigation2 and ROS2 (Default Foxy) installation. 

### How to use

This is an implementation of the plugin defined by Nav2.  
`bringup` package of Navigation2 package is a basic execution example.
There are two main settings to run this plug-in.  

* Dependency Settings on `package.xml`  
  Register this repository(pakcage). 
```
"in package.xml"
...
<exec_depend>nav2_mpc_controller</exec_depend>
...
```
  
* Loading Params on `.py` from launch files
  Load params .yaml file inclued this plugin configuration.
  
```
"in tb3_simulation_with_mpc_launch.py"
...
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_mpc_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes') 
...
```

  Need to create new param file like `nav2_mpc_params.yaml`.
  It should include the configuration for this plugin like below.

```
...
    FollowPath:
      plugin: "nav2_mpc_controller::MPCController"
...
```
