# DeepRacer Control Package

## Overview

The DeepRacer Control ROS package creates the *ctrl_node* which  is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-deepracer/aws-deepracer-launcher).

This is a main node with services exposed to be used by webserver backend API calls. This node manages the different mode of the device [manual, autonomous, calibration]. It allows us to maintain the device in a single mode at any point of time so that the overlapping functionalities (like servo messages, etc) are not conflicting each other. Also, this can be extended to add more modes as done in the Follow the Leader sample application.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the ctrl_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The ctrl_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.
1. *camera_pkg* - The DeepRacer Camera ROS package creates the *camera_node* which is part of the core AWS DeepRacer application.
1. *servo_pkg* - The DeepRacer Servo ROS package creates the *servo_node* which is part of the core AWS DeepRacer application.
1. *inference_pkg* - The DeepRacer Inference ROS package creates the *inference_node* which is part of the core AWS DeepRacer application.
1. *model_optimizer_pkg* - The DeepRacer Model Optimizer ROS package creates the *model_optimizer_node* which is part of the core AWS DeepRacer application.
1. *deepracer_navigation_pkg* - The DeepRacer Navigation ROS package creates the *deepracer_navigation_node* which is part of the core AWS DeepRacer application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the ctrl_pkg on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-ctrl-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-ctrl-pkg
        rosws update

1. Resolve the ctrl_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-ctrl-pkg && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the ctrl_pkg, camera_pkg, servo_pkg, inference_pkg, model_optimizer_pkg, deepracer_navigation_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-ctrl-pkg && colcon build --packages-select ctrl_pkg camera_pkg servo_pkg inference_pkg model_optimizer_pkg deepracer_navigation_pkg deepracer_interfaces_pkg

## Usage

The ctrl_node provides basic system level functionality for the AWS DeepRacer application to work. Although the node is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built ctrl_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-ctrl-pkg/install/setup.bash  

1. Launch the ctrl_node using the launch script:

        ros2 launch ctrl_pkg ctrl_pkg_launch.py

## Launch Files

The ctrl_node provides the core functionality to manage the different modes of operation of the device. The  ctrl_pkg_launch.py is also included in this package that gives an example of how to launch the nodes independently from the core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='ctrl_pkg',
                    namespace='ctrl_pkg',
                    executable='ctrl_node',
                    name='ctrl_node'
                )
            ])

## Node Details

### ctrl_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/deepracer_navigation_pkg/auto_drive|ServoCtrlMsg|Message with scaled steering angle and throttle data as per action space values sent to the servo package to move the car in autonomous mode.|
|/webserver_pkg/calibration_drive|ServoCtrlMsg|Message with raw pwm values for steering angle and throttle data sent to the servo package to calibrate the car in calibration mode.|
|/webserver_pkg/manual_drive|ServoCtrlMsg|Message with steering angle and throttle data sent to the servo package to move the car in manual mode.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/*ctrl_pkg*/raw_pwm|ServoCtrlMsg|Publisher to send a message with raw pwm values for steering angle and throttle data sent to the servo package to calibrate the car.|
|/*ctrl_pkg*/servo_msg|ServoCtrlMsg|Publisher to send a message with steering angle and throttle data sent to the servo package to move the car.|

#### Service Clients

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|/model_optimizer_pkg/model_optimizer_server|ModelOptimizeSrv|Client to the model optimizer service that is called to trigger the model optimizing script for the current model selected in autonomous mode.|
|/inference_pkg/load_model|LoadModelSrv|Client to the load model service from the inference package that is called to set the preprocessing and inference task details in autonomous mode.|
|/inference_pkg/inference_state|InferenceStateSrv|Client to service that is called to enable/disable inference function on the stream of sensor messages in autonomous mode|
|/servo_pkg/servo_gpio|ServoGPIOSrv|Client of service that is called to enable/disable the servo GPIO to allow setting the PWM values on the servo and motor.|
|/servo_pkg/set_calibration|SetCalibrationSrv|Client to set the calibration value for the steering or throttle in calibration mode.|
|/servo_pkg/get_calibration|GetCalibrationSrv|Client to get the current calibration value for the steering or throttle in calibration mode.|
|/servo_pkg/get_led_state|GetLedCtrlSrv|Client to get car led service to get the tail light LED channel PWM values in calibration mode.|
|/servo_pkg/set_led_state|SetLedCtrlSrv|Client to set car led service to set the tail light LED channel PWM values in calibration mode.|
|/deepracer_navigation_pkg/navigation_throttle|NavThrottleSrv|Client to service that is called set the scale value for the navigation throttle in autonomous mode to control the speed of the device.|
|/deepracer_navigation_pkg/load_action_space|LoadModelSrv|Client to load action space service that is called to set the action space for the model in deepracer_navigation_node to be used while mapping the inference results to the action values in autonomous mode.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|vehicle_state|ActiveStateSrv|Service that is called to set the vehicle mode by deactivating the current vehicle state and set the new mode.|
|enable_state|EnableStateSrv|Service that is called to activate and deactivate the current vehicle mode.|
|get_car_cal|GetCalibrationSrv|Service that is called to get the current calibration PWM values [min, mid, max, polarity] for the steering or throttle.|
|set_car_cal|SetCalibrationSrv|Service that is called to set the calibration PWM values  [min, mid, max, polarity] for the steering or throttle.|
|set_car_led|SetLedCtrlSrv|Service that is called to set the tail light LED [red, green, blue] channel PWM values.|
|get_car_led|GetLedCtrlSrv|Service that is called to get the tail light LED [red, green, blue] channel PWM values.|
|get_ctrl_modes| GetCtrlModesSrv | Service that is called to get the available modes of operation for vehicle |
|model_state|ModelStateSrv|Service that is called to execute the load model services in background thread.|
|autonomous_throttle|NavThrottleSrv|Service that is called to set the scale value to multiply to the throttle during autonomous navigation.|
|is_model_loading|GetModelLoadingStatusSrv|Service that is called to know if there is load model operation going on right now on the device in autonomous mode.|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
