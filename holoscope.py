# PALACE prueba de la cámara Basler para el holoscope daA3840-45um monocroma
#  https://github.com/basler/pypylon-samples

# TO-DO: monitor image, 1

import time
import os
import pypylon.pylon as pylon # pip install pypylon
import cv2  # pip install opencv-python
import numpy as np #pip install numpy
import serial # pip install pyserial
import keyboard # pip install keyboard
import threading

global img_calibrate  # the image to calibrate the exposure time
global img_reference # the background image to substract
global ser # serial port handle
global stopar
global camera

serialPort = 'COM13'
pumpSteps = 1 # number of pump steps per interval / frame


datetimepath = time.strftime("Holoscope_data_%Y%m%d-%H%M%S") # create a folder with the date and time
os.makedirs(datetimepath, exist_ok=True) # create the folder if it does not exist

# show the image in a window every x seconds in a separate thread
def show_monitor_image():
    while True:
        if camera.IsGrabbing():
            try: # gives error from time to time due to the main thread using the camera    
                grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    # Access the image data
                    img = grab_result.Array
                    grab_result.Release()
                    img = cv2.resize(img*16,(960, 540))
                    cv2.imshow("Monitor image", img)
                    cv2.waitKey(1)
            except:
                pass
            time.sleep(1) # update every 3 seconds
        #else: # eliminado porque se cerraba al calibrar la exposición  
            #return # end thread if camera is not grabbing

# function to increase pump
def increase_pump():
    global pumpSteps
    pumpSteps = pumpSteps *2
    if pumpSteps > 999:
        pumpSteps = 999
    print(f"Pump steps set to : {pumpSteps}")

# function to decrease pump
def decrease_pump():
    global pumpSteps
    pumpSteps = int(pumpSteps / 2)
    if pumpSteps < 1:
        pumpSteps = 1
    print(f"Pump steps set to : {pumpSteps}")

# Function to stop the program
def stop_doing_things():
    global stopar
    stopar = True

# pump control
def pump(steps):
    global ser
    if steps > 0:
        ser.read() # clear buffer

        ser.write(f'd{steps}'.encode('ascii'))
    if steps < 0:
        ser.read() # clear buffer
        ser.write(f'i{-steps}'.encode('ascii'))

# Function to configure the camera
def configure_camera(exposure_time, brightness):
    global camera
    # Open the camera
    camera.Open()
    camera.GainAuto.Value = "Off"
    # Set exposure time (in microseconds)
    camera.ExposureTime.SetValue(exposure_time)
    # Set brightness (using gain)
    camera.Gain.SetValue(brightness)
    # Start the grabbing of images
    if not camera.IsGrabbing():
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

# Function to capture and save frames
def capture_frames(capture_interval, output_folder):
    global camera
    frame_number = 0
    global stopar
    stopar = False
    while camera.IsGrabbing():

        # Wait for an image and then retrieve it
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        
        if grab_result.GrabSucceeded():
            # Access the image data
            img = grab_result.Array
            
            # Convert the image to RGB format (if necessary)
            #if len(img.shape) == 2:  # Grayscale image
            #    img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            #else:  # Already in RGB format
            #    img_rgb = img
            
            #display de image
            cv2.imshow("Image", img*16) # range 16bits
            cv2.waitKey(1)

            # Save the image to disk
            filename = f"{output_folder}/frame_{frame_number:04d}.png"
            cv2.imwrite(filename, img*16) # se multiplica por 16 para el rango de 16 bits del format grayscale16
            print(f"Saved {filename}")
            
            # Increment the frame number
            frame_number += 1
            
            # Wait for the next capture interval
            time.sleep(capture_interval)
        if stopar:
            break
        # Release the grab result
        grab_result.Release()



# a function to find the best exposure time for the camera so the image is not too dark or too bright
# probada 19mayo2024 parece que el tiempo mínimo es 51us y que hay un incremento mínimo sin que se note cambio
def findBestExposureTime_incremental():
    global camera
    global img_calibrate
    stopar = False
    exposure_time = 51 # minimum value for mono12
    while camera.IsGrabbing():
        # configure the camera
        camera.StopGrabbing() # es necesario stopar el grab para cambiar el exposure time
        #change camera exposure time    


        configure_camera( exposure_time, 0.0) # 
        #camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) # already done inside configure_camera
        # Wait for an image and then retrieve it
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            # Access the image data
            img = grab_result.Array        
            avgPixel = np.average(img)
            print(f"avgPixel for {exposure_time} is {avgPixel}")
            grab_result.Release()            
            exposure_time = int(exposure_time * 1.1)
            #exposure_time= exposure_time + 5
        if exposure_time > 1000 or avgPixel > 2047 or stopar: # si se ha probado todo o se ha llegado al promedio
            img_calibrate = img
            filename = f"./{datetimepath}/calibrate.png"
            cv2.imwrite(filename, img*16) # se multiplica por 16 para el rango de 16 bits del format grayscale16
            break


# get a bunch of reference images. there should be just water in the microslide
def get_background_images():
    global camera
    global img_reference
    for n in range(8):
            grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result.GrabSucceeded():
                # Access the image data
                img = grab_result.Array      
                grab_result.Release()
                avgPixel = np.average(img)
                print(f"Background image with avgPixel = {avgPixel}")
                if n==0:
                    img_reference = img 
                else:
                    img_reference = img_reference + img
                filename = f"./{datetimepath}/reference{n:02d}.png"
                cv2.imwrite(filename, img*16) # se multiplica por 16 para el rango de 16 bits del format grayscale16
    filename = f"./{datetimepath}/reference.png"
    img_reference = (img_reference / 8).astype(np.uint16) # average the images, format /= is important, otherwise it gives floats
    cv2.imwrite(filename, img_reference*16) # /8 * 16


# Function to capture and save frames every x seconds
def interval_capture( capture_interval, output_folder):
    global camera
    frame_number = 0
    global stopar
    stopar = False
    while camera.IsGrabbing():
        # pump 
        pump(pumpSteps)
        time.sleep(0.2)
        # Wait for an image and then retrieve it
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        
        if grab_result.GrabSucceeded():
            # Access the image data
            img = grab_result.Array
            grab_result.Release()
            img_raw = cv2.resize(img*16,(960, 540))
            cv2.imshow("Raw image", img_raw) # range 16bits
            img_subs = cv2.resize((img-img_reference+2048)*16,(960, 540))
            cv2.imshow("Background substracted", img_subs) # range 16bits
            cv2.waitKey(1)
            # Save the image to disk
            filename = f"{output_folder}/frame_{frame_number:04d}.png"
            cv2.imwrite(filename, img*16) # se multiplica por 16 para el rango de 16 bits del format grayscale16
            print(f"Saved {filename}")
            # Increment the frame number
            frame_number += 1
            # Wait for the next capture interval
            time.sleep(capture_interval-0.3)
        if stopar:
            break




# Main function
def main():
    global ser
    global camera
    # Create an instant camera object
    try:
         camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    except:
         print('Error: no se ha conectado la cámara Basler.')
         exit()
    camera.Open()
    # esto debe hacerse antes de que la cámara esté grabbing
    print(camera.GetDeviceInfo().GetModelName(), "-", camera.GetDeviceInfo().GetSerialNumber())
    print(camera.PixelFormat.Symbolics) # get pixel format of the camera
    #set pixel format to Mono12
    camera.TLParamsLocked.Value = False
    camera.PixelFormat.SetValue("Mono12") # da error not writ1able, la cámara debe estar open pero no grabbing
    camera.TLParamsLocked.Value = True
    # Configure the camera settings
    exposure_time = 100  # in microseconds, to start with
    brightness = 0.0  # gain value, in dB
    configure_camera(exposure_time, brightness)

    # open serial port for the pump
    try: 
        ser = serial.Serial(serialPort, 115200, timeout=1) # open serial port
    except:
        pumpSteps= 0 # no pump available
    keyboard.add_hotkey('q', stop_doing_things) # stop the program with q
    keyboard.add_hotkey('+', increase_pump) # increase pump with +
    keyboard.add_hotkey('-', decrease_pump) # increase pump with +
    
    # Start the monitor image thread
    t = threading.Thread(target=show_monitor_image)
    t.start()

    # Create a terminal menu
    while True:
        print('1: Calibrate exposure time, 2: get background -water- images, 3: save every x seconds, 4: save when detect, 5: flush pump, 0:exit')
        try:
            x = int(input())
        except:
            x = -1 
        if x == 1:
            findBestExposureTime_incremental()
        elif x == 2:
            get_background_images()
        elif x == 3:
            print('q: exit, +: increase pump, -: decrease pump')
            capture_interval = 1.0  # capture interval in seconds
            interval_capture(capture_interval, datetimepath)
        elif x == 4:
                pass
        elif x == 5:
                pump(999)
        elif x == 0:
                break

       
        
    # quiting... close the camera
    camera.StopGrabbing()
    t.join() # wait for the monitor image thread to finish
    camera.Close()
    try: 
        ser.close()
    except:
        pass

 
if __name__ == "__main__":
    main()
