import serial
import threading
import time
import keyboard  # Requires admin/root privileges on some systems


Port = 'COM13' # configure according to your case
BaudRate = 115200 # configure according to your case
ser = None

# --- Telemetry Read Thread ---
def readSerial():
    while True:
        try:
            if ser and ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[ARDUINO] {line}")
        except:
            break


def main():
    global ser
    
    try:
        ser = serial.Serial(Port, BaudRate, timeout=0.1)
        print(f"Successfully connected to {Port} at {BaudRate} baud.")
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    threading.Thread(target=readSerial, daemon=True).start()

    print("--- Python Serial Keyboard Controller ---")
    print("Press ESC to exit the script.")
    print("Waiting for inputs...")

    # --- Main Control Loop ---
    lastToggleTime = 0.0
    TOGGLE_DEBOUNCE_SEC = 0.25 

    try:
        while True:
            if keyboard.is_pressed('esc'):
                print("Exiting...")
                break

            currentTime = time.time()

            # Watchdog Defeater: Continuous Jogging Commands
            if keyboard.is_pressed('d'): ser.write(b'D')
            elif keyboard.is_pressed('a'): ser.write(b'A')
            elif keyboard.is_pressed('c') or keyboard.is_pressed('p'): ser.write(b'C')
            elif keyboard.is_pressed('o'): ser.write(b'O')

            # Single-press Commands (Axis selection, speed, enable, zero, stop)
            if (currentTime - lastToggleTime) > TOGGLE_DEBOUNCE_SEC:
                if keyboard.is_pressed('1'): ser.write(b'1'); lastToggleTime = currentTime
                elif keyboard.is_pressed('2'): ser.write(b'2'); lastToggleTime = currentTime
                elif keyboard.is_pressed('3'): ser.write(b'3'); lastToggleTime = currentTime
                elif keyboard.is_pressed('4'): ser.write(b'4'); lastToggleTime = currentTime
                elif keyboard.is_pressed('5'): ser.write(b'5'); lastToggleTime = currentTime
                
                elif keyboard.is_pressed('+') or keyboard.is_pressed('='): ser.write(b'+'); lastToggleTime = currentTime
                elif keyboard.is_pressed('-') or keyboard.is_pressed('_'): ser.write(b'-'); lastToggleTime = currentTime
                
                elif keyboard.is_pressed('e'): ser.write(b'E'); lastToggleTime = currentTime
                elif keyboard.is_pressed('w'): ser.write(b'W'); lastToggleTime = currentTime
                elif keyboard.is_pressed('s'): ser.write(b'S'); lastToggleTime = currentTime

            # 50ms tick rate (20Hz) to keep Arduino watchdog happy
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Script terminated by user.")
    finally:
        if ser:
            ser.close()

if __name__ == "__main__":
    main()