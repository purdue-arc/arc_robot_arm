import cv2 as cv
import numpy as np
import os
import sys
from pynput import keyboard
import time

expect_input = False
generate_dataset = False
i=0

cap = cv.VideoCapture(0)
cv.namedWindow('frame',cv.WINDOW_NORMAL)
print('Input folder name in datasets folder to add data in: ')
dataset_path = os.path.dirname(os.path.abspath(__file__)) + '/' + input()

print('Input initial file_prefix name: ')
file_prefix = input() + '_'

try:
    os.chdir(dataset_path)
except Exception:
    print("Incorrect folder name! Stopping...")
    exit()

def listen_to_keyboard():
    def on_press(key):
        global generate_dataset
        global file_prefix
        global expect_input

        if expect_input:
            try:
                if key == keyboard.Key.enter:
                    expect_input = False
                    print("To START data collection, press s. \nTo STOP data collection, press x.\nTo set prefix of img files generated, press p")
                elif key == keyboard.Key.backspace:
                    file_prefix = file_prefix[:-1]
                    print(file_prefix)
                else:
                    file_prefix += key.char
                    print(file_prefix)
            except AttributeError:
                pass
        else:
            if key == keyboard.KeyCode.from_char('s'):
                print('Generating...')
                generate_dataset = True
            elif key == keyboard.KeyCode.from_char('p'):
                print('Enter file prefix (press enter to confirm new file prefix):')
                expect_input = True
                file_prefix = ""
            elif key == keyboard.KeyCode.from_char('x'):
                print('Stopped listener. Enter CTRL+C to exit.')
                generate_dataset = False
                return False
    listener = keyboard.Listener(on_press=on_press,suppress=True)
    listener.start()

listen_to_keyboard()

print("To START data collection, press s. \nTo STOP data collection, press x.\nTo set prefix of img files generated, press p")

if not cap.isOpened():
    print("Cannot open Camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Cannot get frame")
        break
    if generate_dataset:
        filename = file_prefix + str(i) + ".jpg"
        cv.imwrite(filename, frame)
        i+=1

    # Display the resulting frame
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
