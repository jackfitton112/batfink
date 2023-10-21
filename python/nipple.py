import asyncio
from bleak import BleakClient
import queue
from evdev import InputDevice, ecodes, UInput, KeyEvent
import time
import threading

ADDRESS = "46:76:11:c7:c1:ec"
CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"

client = BleakClient(ADDRESS)
command_queue = queue.Queue()
exit_event = threading.Event()

async def connect_to_robot():
    try:
        await client.connect()
        if client.is_connected:
            print(f"Connected to {ADDRESS}")
        else:
            print("Failed to connect to the robot.")
            return False
    except Exception as e:
        print(f"Error while connecting: {e}")
        return False
    return True

async def send_command_to_robot(command):
    try:
        await client.write_gatt_char(CHARACTERISTIC_UUID, [command])
        print(f"Sent command: {command}")
    except Exception as e:
        print(f"Error while sending command: {e}")
        await connect_to_robot()

async def process_commands():
    await connect_to_robot()
    while not exit_event.is_set():
        command = command_queue.get()
        print(f"sending command: {command}")
        await send_command_to_robot(command)

def get_trackpoint_pos():
    trackpoint = InputDevice('/dev/input/event9')
    x, y = 0, 0
    for event in trackpoint.read_loop():
        if exit_event.is_set():
            break
        
        if event.type == ecodes.EV_REL:
            if event.code == ecodes.REL_X:
                x += event.value
            elif event.code == ecodes.REL_Y:
                y += event.value

        if y > 5:
            print("fwd")
            command_queue.put(1)  # fwd
            y = 0
        elif y < -5:
            print("bk")
            command_queue.put(2)  # bk
            y = 0

        #if less than 5 but more than -5, stop
        if y < 5 and y > -5:
            command_queue.put(0)

def trackpoint_thread():
    get_trackpoint_pos()

def command_thread():
    asyncio.run(process_commands())

def monitor_keyboard():
    # Identify the device path for your keyboard. You may need to adjust the path.
    keyboard_device_path = '/dev/input/event3'  # Adjust this path as needed.
    keyboard = InputDevice(keyboard_device_path)

    for event in keyboard.read_loop():
        if event.type == ecodes.EV_KEY:
            key_event = KeyEvent(event)
            if key_event.keycode == 'KEY_SPACE' and key_event.keystate == KeyEvent.key_down:
                exit_event.set()
                return


if __name__ == "__main__":
    # Start the thread that monitors the trackpoint
    t1 = threading.Thread(target=trackpoint_thread)
    t1.start()

    # Start the thread that connects to the robot and sends commands
    t2 = threading.Thread(target=command_thread)
    t2.start()

    # Monitor the keyboard in the main thread
    monitor_keyboard()

    t1.join()
    t2.join()
