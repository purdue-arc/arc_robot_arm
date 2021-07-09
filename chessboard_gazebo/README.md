# Chessboard Gazebo

## What's in here?

### Models
- All chess pieces model files

### Worlds
- `chessboard.world`: A full chessboard with all rectangular pieces 
- `test_visual_servoing.world`: A chessboard with one king, used for testing visual servoing

## Setup

1. Add models to ~/.bashrc to allow gazebo to find the model files
```
export GAZEBO_MODEL_PATH=/home/user/catkin_ws/src/arc_robot_arm/chessboard_gazebo/models
```

## Credits

Adapted from [chesslab](https://gitioc.upc.edu/jan.rosell/chesslab_setup) project.
