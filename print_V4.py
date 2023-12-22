# import smbus
# import spidev
from PIL import Image, ImageOps
# import RPi.GPIO as GPIO
import time
# import socket
import pandas as pd
from StepperMotorClass import StepperMotor
from DMDClass import DigitalMicromirrorDevice

# initialize the motor
motor = StepperMotor(max_speed_delay=30, min_speed_delay=50)
motor.rotate(direction=1, distance=5)  # Example call to rotate the motor
motor.rotate(direction=-1, distance=5)  # Example call to rotate the motor

# initialize DMD
# Example usage
dmd = DigitalMicromirrorDevice()

### Start printing
# log file name based on date
log_file_name = time.strftime("log-%Y-%m-%d", time.localtime()) + ".txt"
with open(log_file_name, "a") as f:
    f.write("TIME" + "\t" + "IMG NAME" + "\t" + "EXPOSURE TIME" + "\n")

# Load excel file
EXCEL_PATH = "Slice2.xlsx"
df_detail = pd.read_excel("Slice2.xlsx", sheet_name="detail", dtype=str)

# last row of the excel file
last_img = df_detail.shape[0]

stage_location = 0
layer_th = 0.01
stage_move_distance = 5


# loop to exposure images
for i in range(1, last_img):
    # set time
    exposure_time = float(df_detail.at[i, 'Expo_time'])
    wait_time = 0.5
    sleep_time = exposure_time + wait_time + 1

    # defining file name with padded zeros for consistency
    file_name = df_detail.at[i, 'Prefix'] + df_detail.at[i, 'Number'] + ".png"
    # print("file_name is", file_name)
    # process image
    img = Image.open(file_name).convert('L')
    dmd.img_transfer(img)  # Example call to transfer an image
    
    print(time.strftime("%H:%M:%S", time.localtime()), "\t", file_name, "\t", exposure_time, "s")
    dmd.exposure_start(exposure_time, wait_time, sleep_time)  # Example call to start exposure

    # write current date, time, and exposure time to file
    with open(log_file_name, "a") as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + "\t" + file_name + "\t" + str(exposure_time) + "\n")

    ########
    # WAIT SIGNAL FOR MOTOR
    ########

    time.sleep(0.1)
    motor.rotate(direction=1, distance=stage_move_distance)
    stage_location = stage_location + stage_move_distance
    print("stage moved to ", stage_location)
    
    motor.rotate(direction=1, distance=stage_move_distance-layer_th)
    stage_location = stage_location - (stage_move_distance-layer_th)
    print("stage moved to ", stage_location)
