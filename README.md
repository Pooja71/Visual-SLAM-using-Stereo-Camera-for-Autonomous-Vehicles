# Visual SLAM using Stereo Camera for Autonomous Vehicles.
This project is a sub-part of a project that was aimed at developing a robotic system capable of performing autonomous navigation in unknown environments, both Indoor and Outdoor. The three major software modules skteched for the project were:
* Mapping and Localization
* Perception
* Navigation and Control </br>

The objective of our team was to develop a SLAM (Simulatenous Localization and Mapping) for the robotic platform to enable it to create a map of its surroundings, localize itself on the map and track itself. This project used ORB_SLAM2 with ZED stereo camera to achieve SLAM and has a custom 2D occupancy grid mapping algorithm. The state-of-art ORB SLAM2 is developed by:
>@article{murTRO2015,
  title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
  author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={31},
  number={5},
  pages={1147--1163},
  doi = {10.1109/TRO.2015.2463671},
  year={2015}
 }

**Authors:**  
- Luke Favruzzo 
- Pooja Bansal 
- Turner Richmond 
- You Zhou 

**Advisor:**
- Dr. Kevin Han 
 
 This repository consists of four parts:
 - Zed camera ros wrapper(modified as per requirement).
 - ORB_SLAM2
 - depthimage_to_laserscan
 - pointcloud_builder. 
 
 As per current status, the pointcloud_builder is incomplete. </br>
 A detailed report on this project can be found [here](https://drive.google.com/file/d/0B0e5EynMwcxoYkVlNDFjc0t0a28/view?usp=sharing) and the video of the SLAM in action at different locations has been posted on [youtube.](https://www.youtube.com/watch?v=ibuZn6Tqo2Q)
 
 ## Hardware:
 * Husky robot (vehicle)
 * Jetson TX1 (computer)
 * ZED Stereo Camera (Visual SLAM)
 
 ## Software:
 * Ubuntu 16.0.4
 * ROS Kinetic
 
 ## Dependencies:
 * Jetson TX1 - We refer to [this](http://www.jetsonhacks.com/2017/01/28/install-samsung-ssd-on-nvidia-jetson-tx1/) page in mounting the SSD on the TX1.  
 * ORB SLAM2 - Install all the pre-requisites for ORB_SLAM2 as mentioned in the original work by the authors from [here](https://github.com/raulmur/ORB_SLAM2).
 * ZED Stereo Camera - We already have the zed-ros-wrapper in this repositry. Please install the Zed SDK and all pre-requisities following the instructions [here](https://github.com/stereolabs/zed-ros-wrapper)
 
 ## Running the code
 Change the confidence paramater in the launch file of zed camera (zed/src/zed-ros-wrapper/launch/zed.launch). We used a value of 80 in order to clip depth. Reason for doing so has been mentioned below.
 
* roslaunch zed_ros_wrapper zed.launch
* cd SLAM/ORB_SLAM2</br>
 ** Note the directory may differ based on the location of the git pull
* rosrun ORB_SLAM2 Stereo Vocabulary/ORBVoc.txt Examples/ROS/ORB_SLAM2/Zed.yaml false
* rosrun tf static_transform_publisher 0 0 0 0 0 0 map ORB_SLAM/odom

## Changes Achieved

* Clipping the Zed point cloud to only obtain points with high confidence (referenced in troubles faced section below).
* Modifying the ORB_SLAM2 code to publish keyframes and keypoints to view in ROS and be captured by another subscriber.
* Set up the keypoints to be published as PointCloud2 ROS messages (the same message type as published by the Zed-Ros-wrapper).


## Attempted changes

Below we refer to changes we attempted to make along the way which we never completed due to various difficulties.  We try to explain the difficulties faced for each change in case it is attempted in the future.

* Attempted to re-calibrate the camera parameters of the Zed camera.
** We didn't continue with the effort since the factory settings pulled from Zed were specific to the camera and we did not see any significant improvements in manually calibrating the camera.

* In order to achieve a dense point cloud, we attempted to fuse the zed point cloud with the ORB_SLAM2 keyframes to build a complete map.  We want to add subscriber as a nodelet under the same node as ORB_SLAM2 and Zed-ROS-Wrapper (where each of these is a nodelet.  By having each code base as a nodelet under the same node, we would reduce the computation needed for passing the point cloud at each keyframe and obtain improved performance over other teams subscribing to the topic. 
**We were unable to successfully merge the existing node code into nodelets under a single node.


## Troubles faced 

* TX1 comes with a specific version of OpenCV which was not compatible when compiling ORB_SLAM2.
** Require the use of OpenCV that comes with ROS in the CMakeLists.txt under the ORB_SLAM2 code.
* The Zed camera faces accuracy issues when calculating depth of points greater than 3 meters away.
** We were able to go into the Zed settings file and clip the calculation to not get points further than 3 meters.
* In the outdoor environments, the Zed camera is not picking up some features that the monocular cameras are able to see especially in locations where piles of material are located with shadows.
** We captured the scene during different lighting conditions at varying speeds, framerates, and resolution without any noticeable improvements.
