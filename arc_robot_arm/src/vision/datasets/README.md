Aim
 - Determine if gripper picked up object

System
- Inputs
  - JPG Image matrix
- Outputs
  - Gripping object
  - Not gripping object

Gripper States
 1. Open
 2. Closed
  - Succeeded in pick up
  - Failed in pick up
 3. Unknown (gripper out of scene)

Potential Roadblocks

* Gripper must be in the scene when executing grasp
* Grasp may be identified too early
* Grasp may be incorrectly identified due to depth illusion as only single camera is used

Data Generation
1. Types of images generated for failed picks
  open_gripper_with_object_between
  gripper_closed
  open_gripper_with_object_between_2
  closed_gripper_diff_back_color
  no_gripper_visible
  gripper_open
  open_gripper_different_background_color

2. Types of images generated for successful picks
  obj_1_grip
  obj_2_pick
