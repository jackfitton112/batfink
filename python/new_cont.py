import asyncio
from bleak import BleakClient
from pynput import keyboard
import time
import threading
import queue

ADDRESS = "46:76:11:c7:c1:ec"
UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"

# Sensor UUIDs
FRONT_USONIC = "19B10001-E8F2-537E-4F6C-D104768A1214"
LEFT_USONIC = "19B10002-E8F2-537E-4F6C-D104768A1214"
RIGHT_USONIC = "19B10003-E8F2-537E-4F6C-D104768A1214"

# Start/Stop UUID
START_STOP = "19B10007-E8F2-537E-4F6C-D104768A1214"

commands = queue.Queue()

client = BleakClient(ADDRESS)
stop_event = asyncio.Event()

values = {}

async def connect_to_robot():
    if client.is_connected:
        print(f"Already connected to {ADDRESS}")
        return True
    
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


async def disconnect_from_robot():
    if client.is_connected:
        await client.disconnect()
        print(f"Disconnected from {ADDRESS}")
        return True
    else:
        print(f"Already disconnected from {ADDRESS}")
        return False
    
async def start_robot():
    #send ascii 1 to start
    await client.write_gatt_char(START_STOP, b'1')

async def stop_robot():
    #send ascii 0 to stop
    await client.write_gatt_char(START_STOP, b'0')

async def rave_robot():
    print("rave")
    await client.write_gatt_char(START_STOP, b'2')

async def get_sensor_values():

    # Get the sensor values
    front_usonic = await client.read_gatt_char(FRONT_USONIC)
    left_usonic = await client.read_gatt_char(LEFT_USONIC)
    right_usonic = await client.read_gatt_char(RIGHT_USONIC)

    # Convert the sensor values to integers
    front_usonic = int.from_bytes(front_usonic, byteorder='little')
    left_usonic = int.from_bytes(left_usonic, byteorder='little')
    right_usonic = int.from_bytes(right_usonic, byteorder='little')

    # Return the sensor values
    return front_usonic, left_usonic, right_usonic

def input_thread():
    # Initialize robot_running as a list to allow modification inside on_press
    robot_running = [False]

    # Read input from keyboard
    def on_press(key):
        # Access the robot_running list from the outer scope
        nonlocal robot_running

        # If space is pressed, toggle start/stop
        if key == keyboard.Key.space:
            if robot_running[0]:
                commands.put("stop")
                robot_running[0] = False
            else:
                commands.put("start")
                robot_running[0] = True

        # If escape is pressed, stop the robot and disconnect
        if key == keyboard.Key.esc:
            commands.put("stop")
            commands.put("disconnect")
            robot_running[0] = False

        #if r is pressed, put the robot into rave mode
        if key == keyboard.KeyCode(char='r'):
            commands.put("rave")

    # Start the keyboard listener - this will run in a separate thread
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()



async def main():

    #connect to the robot
    await connect_to_robot()

    #start the input thread
    threading.Thread(target=input_thread).start()

    robot_running = False

    while True:

        #check if there are any commands in the queue
        if not commands.empty():
            command = commands.get()
            if command == "start":
                await start_robot()
                robot_running = True
            elif command == "stop":
                await stop_robot()
                robot_running = False
            elif command == "rave":
                await rave_robot()
                robot_running = True
            elif command == "disconnect":
                await stop_robot()
                await disconnect_from_robot()
                break

        #if the robot is running, get the sensor values
        if client.is_connected:
            front_usonic, left_usonic, right_usonic = await get_sensor_values()
            print(f"Front: {front_usonic}, Left: {left_usonic}, Right: {right_usonic}")


        #sleep for 0.1 seconds
        time.sleep(0.1)

    print("Exiting main loop")

asyncio.run(main())





