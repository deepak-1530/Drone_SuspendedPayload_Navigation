# Path Planning for drones with cable suspended payload in cluttered spaces


## Packages

### 1. payloadDroneNav
    Contains code for planning and control for the system
    Planner used is kinodynamic A* planner 
    Controller is geometric control

#### Running the custom model
* cp -r https://github.com/deepak-1530/Drone_SuspendedPayload_Navigation/tree/master/models/drone/aruco_visual_marker_7 PX4-Autopilot/Tools/sitl_gazebo/models
* cp https://github.com/deepak-1530/Drone_SuspendedPayload_Navigation/tree/master/models/drone/iris_depth_camera.sdf PX4-Autopilot/Tools/sitl_gazebo/models/iris_depth_camera
* This would replace the original iris depth camera with the new model
* 
#### Running the pipeline
* Terminal-1
    > cd scripts

    > python3 loadPose.py (publishes payload pose)

* Terminal-2
    > cd launch
    
    > roslaunch map.launch 
* Terminal-3
    > rosrun payloadDroneNav Planner

    > When prompted 'start over ?', enter 1 or 0

* Terminal-4 (for PX4 Controller)
    > rosrun payloadDroneNav Controller

    > Enter delay between control points when prompted (around 0.01-0.15)


* Terminal-5 (for geometric control)
    > cd src/payloadDroneNav
    
    > cd scripts
    
    > python3 geometricControl.py
