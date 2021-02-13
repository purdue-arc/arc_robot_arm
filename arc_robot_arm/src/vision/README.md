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

## Getting started

1. Create a python virtual environment (preferrably in vision folder for easy access)

`python3 -m venv robot-arm-vision-venv`

2. Source virtual environment

`source robot-arm-vision-venv/bin/activate`

3. Download all dependencies into virtual environment

`pip install -r requirements.txt`

4. Install pytorch by following the link below to download pytorch

'''
https://pytorch.org/get-started/locally/
'''

## Optional datasets

1. Download `datasets` folder from Google Drive link below into the vision directory

'''
https://drive.google.com/drive/folders/1mUtn5_v9Ag-IwrnxTEcV91XlIggmoOxV?usp=sharing
'''
