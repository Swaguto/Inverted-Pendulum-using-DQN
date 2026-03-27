import time
import serial
import numpy as np

def calibrate_hardware():
    print("--- Inverted Pendulum Hardware Calibration ---")
    print("This script will help you verify that your sensors and motors are pointing in the right direction.")
    
    try:
        ser = serial.Serial('COM3', 115200, timeout=1)
        time.sleep(2)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("\n1. SENSOR TEST: TILT")
    print("Hold the pendulum at the BOTTOM (hanging down).")
    print("Slowly tilt the pendulum to the RIGHT (Clockwise).")
    print("Monitoring... (Press Ctrl+C to stop this step)")
    
    start_time = time.time()
    try:
        while time.time() - start_time < 10:
            ser.reset_input_buffer()
            ser.write(b"S\n")
            line = ser.readline().decode('utf-8').strip()
            if ',' in line:
                cart, pend = map(int, line.split(','))
                print(f"  Pendulum Count: {pend} (Should INCREASE when tilting RIGHT)", end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    print("\n\n2. SENSOR TEST: CART")
    print("Push the cart manually to the RIGHT.")
    print("Monitoring... (Press Ctrl+C to stop this step)")
    
    start_time = time.time()
    try:
        while time.time() - start_time < 10:
            ser.reset_input_buffer()
            ser.write(b"S\n")
            line = ser.readline().decode('utf-8').strip()
            if ',' in line:
                cart, pend = map(int, line.split(','))
                print(f"  Cart Count: {cart} (Should INCREASE when moving RIGHT)", end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    print("\n\n3. MOTOR TEST")
    print("The motor will now move RIGHT.")
    ser.write(b"A5000\n")
    time.sleep(2)
    ser.write(b"A0\n")
    print("Did the cart move to the RIGHT?")
    
    ser.close()

if __name__ == "__main__":
    calibrate_hardware()
