# Chess Piece Detector

This is a wrapper ROS package for the [yolov5_pytorch_ros](https://github.com/raghavauppuluri13/yolov5_pytorch_ros) package that houses YOLOv5 model weights and launch files to easily run object detection.

The chess piece detection model was trained on [this](https://www.amazon.com/Chess-Armory-Wooden-Interior-Storage/dp/B01256V578/ref=sr_1_29?dchild=1&keywords=chessboard&qid=1625791438&sr=8-29) chessboard, so use that for optimal results. 

## Quick Start

> Make sure to [setup](https://github.com/purdue-arc/arc_robot_arm/blob/main/README.md) your robot arm environment if you haven't already

1. Launch the robot arm in the Gazebo sim with the chessboard **and** the Realsense ROS camera [here](https://github.com/purdue-arc/arc_robot_arm/tree/refactoring/protoarm_bringup)

2. Launch the detector
```
roslaunch chess_piece_detector detector.launch
```
2. Open rqt 

```
# In another window (make sure to activate conda env and source ROS workspace in the new env if not done automatically)
rqt
```
3. Click on `Plugins > Visualization > Image View` 
4. Select the `/detections_image_topic` topic and see the camera output of the detections. If you have the chess board linked above, open up some chess pieces and see the detections.

![dets](https://github.com/purdue-arc/arc_robot_arm/blob/main/assets/images/obj_det_may_21.png)
