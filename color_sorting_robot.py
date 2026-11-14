#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep
import math
import sys


# ============================================================================
# HARDWARE SETUP
# ============================================================================

ev3 = EV3Brick()

# Initialize motors
gripper = Motor(Port.A)  
arm = Motor(Port.B)      
base = Motor(Port.C)     
conveyer = Motor(Port.D)

# Initialize sensors - FIXED CONFIGURATION
arm_home = TouchSensor(Port.S2)  # Changed from S2
base_home = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S3)  # Changed from S3


# ============================================================================
# ROBOT PARAMETERS - UPDATED TO WORKING VALUES
# ============================================================================

L0 = 40.0   # Changed from 55.0
L1 = 50.0   
L2 = 95.0   
L3 = 185.0  

# Calculate combined arm length (L12 + L3)
L12 = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(math.radians(135)))
L_Arm = L12 + L3

gear_base = 36.0 / 12.0  
gear_arm = 40.0 / 8.0    

current_base_angle = 0


# ============================================================================
# HELPER FUNCTION FOR CONSOLE OUTPUT
# ============================================================================

def log(message):
    """Print message and force flush to console"""
    print(message)
    


# ============================================================================
# INVERSE KINEMATICS - FIXED TO WORKING VERSION
# ============================================================================

def inverse_kinematics_theta1(x, y):
    """Calculate base rotation angle"""
    theta1 = math.degrees(math.atan2(y, x))
    return theta1


def inverse_kinematics_theta2(x, z):
    """Calculate arm angle - SIMPLIFIED WORKING VERSION"""
    dz = z - L0
    dz = max(min(dz, L_Arm), -L_Arm)
    theta = math.asin(dz / L_Arm)
    return -math.degrees(theta)


# ============================================================================
# HOMING OPERATION
# ============================================================================

def home_joint(motor, sensor, direction, home_offset, speed=100):
    motor.run(direction * speed)
    while not sensor.pressed():
        wait(10)
    motor.stop(Stop.BRAKE)
    motor.run_angle(-direction * speed, home_offset, Stop.HOLD)
    motor.reset_angle(0)


def perform_homing():
    log("\n========================================")
    log("HOMING SEQUENCE STARTED")
    log("========================================")
    
    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Homing...")
    ev3.speaker.say("Homing")
    
    log("Homing base motor...")
    home_joint(motor=base, sensor=base_home, direction=1, home_offset=120 * gear_base)
    log("Base homed successfully")
    
    log("Homing arm motor...")
    home_joint(motor=arm, sensor=arm_home, direction=-1, home_offset=15)
    log("Arm homed successfully")
    
    base.reset_angle(0)
    arm.reset_angle(0)
    
    # CRITICAL FIX: Synchronize gripper physical position with angle tracking
    log("Synchronizing gripper position...")
    # Move gripper to known closed position
    gripper.run_angle(200, -90, Stop.HOLD)  # Ensure physically closed
    # NOW reset angle to 0 (representing closed state)
    gripper.reset_angle(0)
    log("Gripper synchronized at CLOSED position (angle=0)")

    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Home Complete")
    ev3.speaker.say("Homing done")
    
    log("HOMING COMPLETE")
    log("========================================\n")
    wait(1000)


# ============================================================================
# GRIPPER FUNCTIONS - FIXED TO WORKING VALUES
# ============================================================================

def gripper_open():
    """Open the gripper"""
    log("  -> Opening gripper...")
    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Opening gripper")
    gripper.run_angle(90, 90, Stop.HOLD)  # SWAPPED to +90
    wait(500)
    log("  -> Gripper opened")


def gripper_close():
    """Close the gripper to grab"""
    log("  -> Closing gripper...")
    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Closing gripper")
    gripper.run_angle(90, -90, Stop.HOLD)  # SWAPPED to -90
    wait(500)
    log("  -> Gripper closed")


# ============================================================================
# MOVEMENT FUNCTIONS - UPDATED TO USE FIXED INVERSE KINEMATICS
# ============================================================================

def move_arm_to(x, z, speed=120):
    global current_base_angle
    
    if current_base_angle != 0:
        y = x * math.tan(math.radians(current_base_angle))
    else:
        y = 0
    
    log("  -> Moving arm to: x=" + str(x) + ", z=" + str(z) + ", y=" + str(round(y, 1)))
    
    # Use FIXED inverse kinematics
    theta2 = inverse_kinematics_theta2(x, z)
    motor_angle = theta2 * gear_arm
    
    log("  -> Calculated theta2=" + str(round(theta2, 2)) + " deg, motor_angle=" + str(round(motor_angle, 2)) + " deg")
    
    arm.run_angle(speed, motor_angle - arm.angle(), Stop.HOLD)
    log("  -> Arm moved successfully")


def move_base_to(x, y, speed=150):
    global current_base_angle
    
    log("  -> Moving base to: x=" + str(x) + ", y=" + str(y))
    
    theta1 = inverse_kinematics_theta1(x, y)
    motor_target = theta1 * gear_base
    
    log("  -> Calculated theta1=" + str(round(theta1, 2)) + " deg, motor_angle=" + str(round(motor_target, 2)) + " deg")
    
    base.run_angle(speed, motor_target - base.angle(), Stop.HOLD)
    current_base_angle = theta1
    log("  -> Base moved successfully")


# ============================================================================
# COLOR DETECTION FUNCTIONS
# ============================================================================

SAMPLES = 20
TOL = 20


def read_average_color():
    r_total, g_total, b_total = 0, 0, 0
    
    for _ in range(SAMPLES):
        r, g, b = color_sensor.rgb()
        r_total += r
        g_total += g
        b_total += b
        sleep(0.05)

    return (
        r_total // SAMPLES,
        g_total // SAMPLES,
        b_total // SAMPLES
    )


def is_within_tolerance(measured, reference, tol=TOL):
    return (abs(measured[0] - reference[0]) <= tol and
            abs(measured[1] - reference[1]) <= tol and
            abs(measured[2] - reference[2]) <= tol)


# ============================================================================
# MAIN PROGRAM
# ============================================================================

log("\n" + "="*60)
log("EV3 ROBOT MANIPULATOR - PICK AND PLACE (FIXED VERSION)")
log("="*60 + "\n")

# Perform initial homing
ev3.screen.clear()
ev3.screen.draw_text(10, 50, "Starting...")
wait(500)

perform_homing()

# Color calibration
log("\n========================================")
log("COLOR CALIBRATION STARTED")
log("========================================")

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place BLUE ball")
ev3.speaker.say("Calibrating RGB values")
ev3.speaker.say("Place BLUE ball")
sleep(10)
BLUE = read_average_color()
log("BLUE calibrated: R=" + str(BLUE[0]) + ", G=" + str(BLUE[1]) + ", B=" + str(BLUE[2]))
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place BLACK ball")
ev3.speaker.say("Place BLACK ball")
sleep(10)
BLACK = read_average_color()
log("BLACK calibrated: R=" + str(BLACK[0]) + ", G=" + str(BLACK[1]) + ", B=" + str(BLACK[2]))
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place RED ball")
ev3.speaker.say("Place Red ball")
sleep(10)
RED = read_average_color()
log("RED calibrated: R=" + str(RED[0]) + ", G=" + str(RED[1]) + ", B=" + str(RED[2]))
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place GREEN ball")
ev3.speaker.say("Place GREEN ball")
sleep(10)
GREEN = read_average_color()
log("GREEN calibrated: R=" + str(GREEN[0]) + ", G=" + str(GREEN[1]) + ", B=" + str(GREEN[2]))
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 50, "Calibration Done")
ev3.speaker.say("Calibration complete")

log("========================================")
log("COLOR CALIBRATION COMPLETE")
log("========================================\n")
wait(2000)


# ============================================================================
# MAIN PICK AND PLACE LOOP
# ============================================================================

log("\n" + "="*60)
log("STARTING PICK AND PLACE PROCESS")
log("="*60 + "\n")

ev3.screen.clear()
ev3.screen.draw_text(10, 50, "Starting Process")
ev3.speaker.say("Starting process")
wait(1000)

# Main loop - process 4 balls
for i in range(40):
    log("\n" + "="*60)
    log("PROCESSING BALL " + str(i+1) + " OF 4")
    log("="*60)
    
    # Display ball number
    ev3.screen.clear()
    ev3.screen.draw_text(10, 30, "Ball " + str(i+1) + " of 4")
    ev3.screen.draw_text(10, 60, "Place ball now")
    ev3.speaker.say("Place ball " + str(i+1))
    
    log("Waiting for ball " + str(i+1) + "...")
    
    # Wait for ball
    wait(1000)  # Reduced from 3000 to match working code
    
    # Detect color
    color = color_sensor.color()
    rgb = color_sensor.rgb()
    
    log("\nCOLOR DETECTED:")
    log("  Color enum: " + str(color))
    log("  RGB values: R=" + str(rgb[0]) + ", G=" + str(rgb[1]) + ", B=" + str(rgb[2]))
    
    # Display detected color on screen
    ev3.screen.clear()
    ev3.screen.draw_text(10, 30, "Detected:")
    ev3.screen.draw_text(10, 60, str(color))
    ev3.screen.draw_text(10, 90, "RGB:" + str(rgb[0]) + "," + str(rgb[1]) + "," + str(rgb[2]))
    wait(1000)
    
    if color == Color.BLACK:
        log("\n>>> PROCESSING BLACK BALL <<<")
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "BLACK ball")
        ev3.speaker.say("Color is Black")
        
        log("Running conveyor for BLACK...")
        conveyer.stop()
        conveyer.run_angle(80, -700, Stop.HOLD)
        log("BLACK ball processing complete\n")
        
    elif color == Color.GREEN:
        log("\n>>> PROCESSING GREEN BALL <<<")
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "GREEN ball")
        ev3.speaker.say("Color is Green")
        
        log("Running conveyor for GREEN...")
        conveyer.stop()
        conveyer.run_angle(80, 70, Stop.HOLD)
        log("GREEN ball processing complete\n")
        
    elif color == Color.RED:
        log("\n>>> PROCESSING RED BALL <<<")
        log("RED ball detected - starting pick and place sequence")
        
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "RED ball")
        ev3.speaker.say("Color is red")
        
        conveyer.stop()
        log("Running conveyor for RED...")
        conveyer.run_angle(80, -300, Stop.HOLD)
        
        # Pick and place sequence
        try:
            log("\nSTEP 1: Opening gripper")
            gripper_open()
            
            log("\nSTEP 2: Moving to pick position")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Moving to pick")
            wait(500)
            move_arm_to(-110, -180)
            
            log("\nSTEP 3: Closing gripper to grab ball")
            wait(500)
            gripper_close()
            
            log("\nSTEP 4: Lifting arm with ball")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lifting")
            wait(500)
            move_arm_to(50, 50)
            
            log("\nSTEP 5: Rotating base to drop position (RED station)")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Rotating base")
            wait(500)
            move_base_to(0, 50)
            
            log("\nSTEP 6: Lowering to drop position")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lowering")
            wait(500)
            move_arm_to(-200, -250)
            
            log("\nSTEP 7: Opening gripper to release ball")
            wait(500)
            gripper_open()
            
            log("\nSTEP 8: Returning arm up")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Returning")
            wait(500)
            move_arm_to(50, 50)
            
            log("\nSTEP 9: Returning to home position")
            wait(500)
            perform_homing()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "RED complete")
            ev3.speaker.beep()
            log("RED ball processing COMPLETE\n")
            wait(1000)
            
        except Exception as e:
            log("\n!!! ERROR processing RED ball: " + str(e))
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "ERROR!")
            ev3.speaker.say("Error occurred")
            wait(2000)
            
    elif color == Color.BLUE:
        log("\n>>> PROCESSING BLUE BALL <<<")
        log("BLUE ball detected - starting pick and place sequence")
        
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "BLUE ball")
        ev3.speaker.say("Color is blue")
        
        conveyer.stop()
        log("Running conveyor for BLUE...")
        conveyer.run_angle(80, -300, Stop.HOLD)
        
        # Pick and place sequence
        try:
            log("\nSTEP 1: Opening gripper")
            gripper_open()
            
            log("\nSTEP 2: Moving to pick position")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Moving to pick")
            wait(500)
            move_arm_to(-110, -180)
            
            log("\nSTEP 3: Closing gripper to grab ball")
            wait(500)
            gripper_close()
            
            log("\nSTEP 4: Lifting arm with ball")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lifting")
            wait(500)
            move_arm_to(50, 50)
            
            log("\nSTEP 5: Rotating base to drop position (BLUE station)")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Rotating base")
            wait(500)
            move_base_to(0, -50)
            
            log("\nSTEP 6: Lowering to drop position")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lowering")
            wait(500)
            move_arm_to(-200, -250)
            
            log("\nSTEP 7: Opening gripper to release ball")
            wait(500)
            gripper_open()
            
            log("\nSTEP 8: Returning arm up")
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Returning")
            wait(500)
            move_arm_to(50, 30)
            
            log("\nSTEP 9: Returning to home position")
            wait(500)
            perform_homing()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "BLUE complete")
            ev3.speaker.beep()
            log("BLUE ball processing COMPLETE\n")
            wait(1000)
            
        except Exception as e:
            log("\n!!! ERROR processing BLUE ball: " + str(e))
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "ERROR!")
            ev3.speaker.say("Error occurred")
            wait(2000)
            
    else:
        log("\n!!! UNKNOWN COLOR DETECTED !!!")
        log("Color value: " + str(color))
        log("RGB: R=" + str(rgb[0]) + ", G=" + str(rgb[1]) + ", B=" + str(rgb[2]))
        log("No matching color - skipping this ball\n")
        
        ev3.screen.clear()
        ev3.screen.draw_text(10, 30, "UNKNOWN color")
        ev3.screen.draw_text(10, 60, "Value: " + str(color))
        ev3.screen.draw_text(10, 90, "Skipping...")
        ev3.speaker.say("Unknown color")
        wait(2000)

# Process complete
conveyer.stop()
ev3.screen.clear()
ev3.screen.draw_text(10, 50, "ALL DONE!")
ev3.speaker.say("PROCESS COMPLETE")

log("\n" + "="*60)
log("ROBOT OPERATION COMPLETED SUCCESSFULLY")
log("="*60 + "\n")

wait(3000)
