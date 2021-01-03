## Aim
 - Determine if gripper picked up object

## System
- Inputs
  - JPG Image matrix
- Outputs
  - Gripping object
  - Not gripping object

## Gripper States
 1. Open
 2. Closed
  - Succeeded in pick up
  - Failed in pick up
 3. Unknown (gripper out of scene)

## Potential Roadblocks

* Gripper must be in the scene when executing grasp
* Grasp may be identified too early
* Grasp may be incorrectly identified due to depth illusion as only single camera is used

## Data Generation
* The following are file prefixes for a variety of different situations that I could think of for each 
state outcome of the CNN, there are around 1000 images for each of the file prefixes shown below

1. Types of images generated for failed picks
 - ![closed_gripper_diff_back_color](pick_fail/closed_gripper_diff_back_color_0.jpg)
 - ![no_gripper_visible](pick_fail/no_gripper_visible_0.jpg)
 - ![gripper_open](pick_fail/gripper_open_0.jpg)
 - ![gripper_closed](pick_fail/gripper_closed_0.jpg)
 - ![open_gripper_with_object_between](pick_fail/open_gripper_with_object_between_0.jpg)
 - ![open_gripper_with_object_between_2](pick_fail/open_gripper_with_object_between_2_0.jpg)
 - ![open_gripper_different_background_color](pick_fail/open_gripper_different_background_color_0.jpg)

2. Types of images generated for succesful picks
 - ![obj_2_pick](pick_success/obj_2_pick_0.jpg)
 - ![obj_1_grip](pick_success/obj_1_grip_0.jpg)
