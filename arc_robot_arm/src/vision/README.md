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

## Download datasets

1. Download `datasets` folder from Google Drive link belowinto the vision directory

'''
https://drive.google.com/drive/folders/1mUtn5_v9Ag-IwrnxTEcV91XlIggmoOxV?usp=sharing
'''
