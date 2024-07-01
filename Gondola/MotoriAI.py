import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
import os
import serial

# Height and width of the window
w = 800
h = 600

# Motor pins
IN1 = 5
IN2 = 6
IN3 = 19
IN4 = 13
ENA = 22
ENB = 26
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin configuration asa OUTPUT
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)


PWMA = GPIO.PWM(ENA, 100)
PWMA.start(0)
PWMB = GPIO.PWM(ENB, 100)
PWMB.start(0)

MOTOR1_GPIO = 24
MOTOR2_GPIO = 16
ENCODER1_A_GPIO = 12
ENCODER2_A_GPIO = 21

# Initialize variables
posizione1 = 0
posizione2 = 0

# St motor and encoder pins
GPIO.setup(MOTOR1_GPIO, GPIO.OUT)
GPIO.setup(MOTOR2_GPIO, GPIO.OUT)
GPIO.setup(ENCODER1_A_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER2_A_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set interrupts for encoders
GPIO.setup(MOTOR2_GPIO, GPIO.OUT)
GPIO.setup(ENCODER1_A_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER2_A_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm1 = GPIO.PWM(MOTOR1_GPIO, 100)  # 100 Hz frequency
pwm2 = GPIO.PWM(MOTOR2_GPIO, 100)

pwmdx = 30 #visto da dietro
pwmsx = 30

# Callback functions to handle encoders
def update_encoder1(channel):
    global posizione1
    posizione1 += 1

def update_encoder2(channel):
    global posizione2
    posizione2 += 1

def get_posizione1():
    return posizione1

def get_posizione2():
    return posizione2

#
def straigthAI(seconds):
    my_camera = cv2.VideoCapture(0)
    my_camera.set(3, w)
    my_camera.set(4, h)
    pwm1.start(pwmdx)
    pwm2.start(pwmsx)
    start_time = time.time()
    while time.time() - start_time < seconds:
        success, image = my_camera.read()
        if not success:
            break

        image = cv2.flip(image, 1)
        image_blur = cv2.GaussianBlur(image, (5, 5), 0)
        image_HSV = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)

        #Red range
        lower_pink = np.array([0,220,20])
        upper_pink = np.array([15,255,190])
        
        mask = cv2.inRange(image_HSV,lower_pink,upper_pink)
        mask = cv2.GaussianBlur(mask,(5,5),0)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            
            if moments['m00'] != 0:
                target_x = int(moments['m10'] / moments['m00'])
                target_y = int(moments['m01'] / moments['m00'])

                diam = int(np.sqrt(cv2.contourArea(largest_contour)) / 4)
                cv2.putText(image, f"X: {target_x}, Y: {target_y}", (w - 200, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.circle(image, (target_x, target_y), diam, (0, 255, 0), 1)
                cv2.line(image, (target_x - 2 * diam, target_y), (target_x + 2 * diam, target_y), (0, 255, 0), 1)
                cv2.line(image, (target_x, target_y - 2 * diam), (target_x, target_y + 2 * diam), (0, 255, 0), 1)
            
                target_x = int(moments['m10'] / moments['m00'])

                #Calculate the difference between the position of the ball and the center of the window
                difference = target_x - w / 2

                # Normalize the difference to obtain a value between -1 and 1
                normalized_difference = difference / (w / 2)

                # Calculate the PWM values for the left and right wheels
                pwm_left = max(0, 30 * (1 - normalized_difference))
                pwm_right = max(0, 30 * (1 + normalized_difference))

                print(f"X: {target_x}, PWM Left: {pwm_left}, PWM Right: {pwm_right}")

                pwm1.ChangeDutyCycle(pwm_left)
                pwm2.ChangeDutyCycle(pwm_right)
        else:
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
            print("Motori off")

        cv2.line(image, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)  # Asse Y
        cv2.line(image, (0, h // 2), (w, h // 2), (255, 0, 0), 2)  # Asse X

    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

    cv2.destroyAllWindows()
    my_camera.release()
    cv2.waitKey(10)  # Fix OpenCV bug
    time.sleep(0.1)
    cv2.waitKey(10)  # Fix OpenCV bug
    cv2.waitKey(10)  # Fix OpenCV bug

import cv2
import numpy as np
import time

# Function to move the robot to the left for a specified duration, attempting to find a pink object.
def leftAI(tempo):
    # Initialize the camera
    my_camera = cv2.VideoCapture(0)
    my_camera.set(3, w)  # Set the width of the video capture
    my_camera.set(4, h)  # Set the height of the video capture

    # Start moving left
    pwm1.ChangeDutyCycle(0)  # Stop the right wheel
    pwm2.start(30)  # Start the left wheel with 30% power
    time.sleep(2)  # Wait for 2 seconds to ensure the movement has started

    found_ball = False  # Flag to indicate if the pink object has been found
    start_time = time.time()  # Record the start time

    # Loop until the pink object is found or the specified time has elapsed
    while not found_ball and time.time() - start_time <= tempo:
        success, image = my_camera.read()  # Read an image frame from the camera
        if not success:
            break  # If the frame could not be read, exit the loop

        image = cv2.flip(image, 1)  # Flip the image horizontally
        image_blur = cv2.GaussianBlur(image, (5, 5), 0)  # Apply Gaussian blur to the image
        image_HSV = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)  # Convert the image from BGR to HSV color space

        # Define the HSV range for the color pink
        lower_pink = np.array([0,220,20])
        upper_pink = np.array([15,255,190])
        
        # Create a mask that only includes pixels within the pink range
        mask = cv2.inRange(image_HSV,lower_pink,upper_pink)
        mask = cv2.GaussianBlur(mask,(5,5),0)  # Apply Gaussian blur to the mask

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If any contours are found
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)  # Find the largest contour
            moments = cv2.moments(largest_contour)  # Calculate moments for the largest contour
            if moments['m00'] != 0:
                # Calculate the centroid of the contour
                target_x = int(moments['m10'] / moments['m00'])
                target_y = int(moments['m01'] / moments['m00'])
                found_ball = True  # Set the flag to indicate the pink object has been found
                print("VISTA!")  # Print a message indicating the object has been seen
                time.sleep(0.5)  # Wait for half a second
                stop()  # Stop the robot

                # Calculate the diameter of the object for visualization
                diam = int(np.sqrt(cv2.contourArea(largest_contour)) / 4)
                # Draw the centroid and the crosshair on the image
                cv2.putText(image, f"X: {target_x}, Y: {target_y}", (w - 200, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.circle(image, (target_x, target_y), diam, (0, 255, 0), 1)
                cv2.line(image, (target_x - 2 * diam, target_y), (target_x + 2 * diam, target_y), (0, 255, 0), 1)
                cv2.line(image, (target_x, target_y - 2 * diam), (target_x, target_y + 2 * diam), (0, 255, 0), 1)

                # Calculate the difference between the object's position and the center of the window
                difference = target_x - w / 2

                # Normalize the difference to get a value between -1 and 1
                normalized_difference = difference / (w / 2)

                # Calculate the PWM value for the right wheel based on the normalized difference
                pwm_right = max(0, 30 * (1 + normalized_difference))

                print(f"X: {target_x}, PWM Right: {pwm_right}")  # Print the target position and PWM value for the right wheel
                # Here you would send the PWM values to the robot's motors

        # Draw the center lines on the image for reference
        cv2.line(image, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)  # Vertical line
        cv2.line(image, (0, h // 2), (w, h // 2), (255, 0, 0), 2)  # Horizontal line

    stop()  # Ensure the robot is stopped at the end of the function
    cv2.destroyAllWindows()  # Close all OpenCV windows
    my_camera.release()  # Release the camera resource
    cv2.waitKey(10)
    time.sleep(0.1)
    cv2.waitKey(10)
    cv2.waitKey(10)

def straight():
    pwm1.start(pwmdx)
    pwm2.start(pwmsx)

    # Print encoder positions
    print("Posizione dell'encoder 1: " + str(get_posizione1()))
    print("Posizione dell'encoder 2: " + str(get_posizione2()))
    time.sleep(0.01)

def straight_time(duration):
    run_motors_and_get_positions(pwmdx, pwmsx, duration)

def stop():
    pwm1.stop()
    pwm2.stop()

def left(duration):
    run_motors_and_get_positions(0, pwmdx, duration)

def right(duration):
    run_motors_and_get_positions(0, pwmsx, duration)

def run_motors_and_get_positions(duty_cycle1, duty_cycle2, duration):
    # Start motors with 0% duty cycle
    pwm1.start(0)
    pwm2.start(0)

    # Start time
    start_time = time.time()

    # Main loop
    while True:
        # Check if duration has passed
        if time.time() - start_time >= duration:
            # Stop motors
            pwm1.stop()
            pwm2.stop()
            break

        # Set duty cycle
        pwm1.ChangeDutyCycle(duty_cycle1)
        pwm2.ChangeDutyCycle(duty_cycle2)

        # Print encoder positions
        print("Posizione dell'encoder 1: " + str(get_posizione1()))
        print("Posizione dell'encoder 2: " + str(get_posizione2()))
        time.sleep(0.01)

# Function to open the door
def openDoor(tempo):
    # Set motor directions to open the door
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(ENA, GPIO.HIGH)  # Enable motor A
    GPIO.output(ENB, GPIO.HIGH)  # Enable motor B

    # Set PWM duty cycle to 100% for both motors
    PWMA.ChangeDutyCycle(100)
    PWMB.ChangeDutyCycle(100)

    # Wait for the specified time to open the door
    time.sleep(tempo)

    # Turn off the motors
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    PWMA.ChangeDutyCycle(0)
    PWMB.ChangeDutyCycle(0)

# Function to close the door
def closeDoor(tempo):
    # Set motor directions to close the door
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(ENA, GPIO.HIGH)  # Enable motor A
    GPIO.output(ENB, GPIO.HIGH)  # Enable motor B

    # Set PWM duty cycle to 40% for both motors
    PWMA.ChangeDutyCycle(40)
    PWMB.ChangeDutyCycle(40)

    # Wait for the specified time to close the door
    time.sleep(tempo)

    # Turn off the motors
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    PWMA.ChangeDutyCycle(0)
    PWMB.ChangeDutyCycle(0)
