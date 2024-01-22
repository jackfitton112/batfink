# Title: Controller
# Description: BLE controller for the robot (DEPRECATED AS BLE KEEPS CRASHING)
# Author: Jack Fitton
# Version: 1.0
# Date: 21/01/2024
# (c) Copyright 2024 Jack Fitton

import asyncio
import struct
from bleak import BleakClient
import queue
import threading
import time
import csv
import logging




# BLE UUIDs
BLE_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"

BLE_CHARACTERISTIC_ROBOT_STATE_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_ROBOT_X_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_ROBOT_Y_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_ROBOT_THETA_UUID = "19B10004-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_ROBOT_FSEN_UUID = "19B10005-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_ROBOT_LSEN_UUID = "19B10006-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_ROBOT_RSEN_UUID = "19B10007-E8F2-537E-4F6C-D104768A1214"

BLE_CHARACTERISTIC_CONTROL_DIR_UUID = "19B10008-E8F2-537E-4F6C-D104768A1214"
BLE_CHARACTERISTIC_CONTROL_DIST_UUID = "19B10009-E8F2-537E-4F6C-D104768A1214"


logger = logging.getLogger(__name__)

BLE_ADDR = "3F:7A:2D:AE:E6:86"


async def run_ble_client(queue: asyncio.Queue):

    device = BLE_ADDR

    logger.info("connecting to device...")

    async def callback_handler(_, data):
        await queue.put((time.time(), data))

    async with BleakClient(device) as client:
        logger.info("connected")
        data = await client.start_notify(BLE_CHARACTERISTIC_ROBOT_FSEN_UUID, callback_handler)
        #async loop forever
        while True:
            asyncio.get_event_loop()


def callback_handler(sender, data):
    print(sender, data)

def main():
    #setup logging
    logging.basicConfig(level=logging.INFO)

    #setup queue
    q = asyncio.Queue()

    #start BLE client
    loop = asyncio.get_event_loop()
    loop.create_task(run_ble_client(q))

    #start main loop
    while True:
        #get data from queue
        data = loop.run_until_complete(q.get())
        print(data)

if __name__ == "__main__":
    main()