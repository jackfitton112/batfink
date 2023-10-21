import asyncio
from bleak import BleakClient
from pynput import keyboard
import queue
import struct


ADDRESS = "46:76:11:c7:c1:ec"
CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
KP_LEFT_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
KI_LEFT_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
KD_LEFT_UUID = "19B10004-E8F2-537E-4F6C-D104768A1214"
KP_RIGHT_UUID = "19B10005-E8F2-537E-4F6C-D104768A1214"
KI_RIGHT_UUID = "19B10006-E8F2-537E-4F6C-D104768A1214"
KD_RIGHT_UUID = "19B10007-E8F2-537E-4F6C-D104768A1214"


client = BleakClient(ADDRESS)
command_queue = queue.Queue()


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

async def send_float_to_robot(uuid, value):
    try:
        # Convert the float value to bytes
        value_bytes = bytearray(struct.pack("f", value))
        await client.write_gatt_char(uuid, value_bytes)
        print(f"Sent {value} to UUID: {uuid}")
    except Exception as e:
        print(f"Error while sending float value: {e}")
        await connect_to_robot()

def on_key_release(key):
    if key == keyboard.Key.up:
        command_queue.put(1)  # Forward
    elif key == keyboard.Key.down:
        command_queue.put(2)  # Backward
    elif key == keyboard.Key.left:
        command_queue.put(3)  # Left
    elif key == keyboard.Key.right:
        command_queue.put(4)  # Right
    elif key == keyboard.Key.space:
        command_queue.put(0)  # Stop
    elif key == keyboard.Key.esc:
        command_queue.put(-1)  # Signal to end
        return False

import struct  # Add this import at the top of the code

async def process_commands():
    while True:
        # Process robot commands
        if not command_queue.empty():
            command = command_queue.get()
            if command == -1:
                break
            await send_command_to_robot(command)

        await asyncio.sleep(0.01)  # Add a short delay to avoid busy waiting



async def main():
    await connect_to_robot()



    # Run the listener in a separate thread within the event loop
    loop = asyncio.get_running_loop()
    listener = keyboard.Listener(on_release=on_key_release)
    listener_thread = loop.run_in_executor(None, listener.start)

    # Process commands until we receive an exit signal
    await process_commands()

    await disconnect_from_robot()

    # Stop the listener thread
    listener.stop()
    await listener_thread



async def disconnect_from_robot():
    if client.is_connected:
        await client.disconnect()
        print(f"Disconnected from {ADDRESS}")

if __name__ == "__main__":
    asyncio.run(main())
