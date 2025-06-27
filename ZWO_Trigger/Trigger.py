import argparse
import os
import sys
import time
import zwoasi
from Camera_Trigger import *

def RAW16_photo(filename, gain, exposure):
    camera.set_control_value(zwoasi.ASI_GAIN, gain)
    camera.set_control_value(zwoasi.ASI_EXPOSURE, exposure)


    filename = 'image_mono16.tiff'
    camera.set_image_type(zwoasi.ASI_IMG_RAW16)
    camera.capture(filename=filename)
    print('Saved to %s' % filename)
    save_control_values(filename, camera.get_control_values())

def RGB_photo(filename, gain, exposure):
    camera.set_control_value(zwoasi.ASI_GAIN, gain)
    camera.set_control_value(zwoasi.ASI_EXPOSURE, exposure)

    filename = 'image_color.jpg'
    camera.set_image_type(zwoasi.ASI_IMG_RGB24)
    print('Capturing a single, color image')
    camera.capture(filename=filename)
    print('Saved to %s' % filename)
    save_control_values(filename, camera.get_control_values())  

    
