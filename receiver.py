"""
UART Service
-------------

An example showing how to write a simple program using the Nordic Semiconductor
(nRF) UART service.

"""

import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator
import csv
import numpy as np
import time
from datetime import datetime

import matplotlib.pyplot as plt
from drawnow import *

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

values = np.zeros(1000)
plt.ion()#打开交互模式

#预加载虚拟数据

plt.legend(loc='upper right')
plt.title('SAADC from NRF52800')
plt.grid(True)
plt.ylabel('Data')
 
def plotValues():
    plt.plot(values, label='Data')

count = 0
startReceiveFlag = 0
startTime = 0
writeCount = 0

filenm = str(datetime.now()) + ".csv"

with open(filenm, mode='w', newline='') as file:
# TIP: you can get this function and more from the ``more-itertools`` package.

    async def uart_terminal():
        """This is a simple "terminal" program that uses the Nordic Semiconductor
        (nRF) UART service. It reads from stdin and sends each line of data to the
        remote device. Any data received from the device is printed to stdout.
        """

        def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
            # This assumes that the device includes the UART service UUID in the
            # advertising data. This test may need to be adjusted depending on the
            # actual advertising data supplied20 by the device.
            if UART_SERVICE_UUID.lower() in adv.service_uuids:
                return True

            return False

        device = await BleakScanner.find_device_by_filter(match_nus_uuid)

        if device is None:
            print("no matching device found, you may need to edit match")
            sys.exit(1)

        def handle_disconnect(_: BleakClient):
            print("Device was disconnected, goodbye.")
            file.close()
            # cancelling all tasks effectively ends the program
            for task in asyncio.all_tasks():
                task.cancel()
            exit()

        def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
            global count,startReceiveFlag,startTime,writeCount
            
            if startReceiveFlag == 0:
                startReceiveFlag = 1
                startTime = int(time.time()*10000)

            #print("received:", data.hex())
            #print(len(data))
            count = count + 1

            recstr = data.hex()
            strlen = len(data)

            aver = 0

            for i in range(0,int(strlen/2)):
                inttmp1 = int(recstr[i*4:i*4+2],16)
                inttmp2 = int(recstr[i*4+2:i*4+4],16)
                ans = inttmp2 * 256 + inttmp1 - 3000
                file.write(str((int(time.time()*10000) - startTime) * 0.0001 )+","+str(ans)+"\n")
                writeCount += 1
                aver += ans

            aver /= int(strlen/2)
            values[:-1] = values[1:]; values[-1] = aver
                
            if (count % 30 == 0):
                try:
                    #print(valueInInt)
                    drawnow(plotValues)
                    #valueInInt = 0  
                except ValueError:
                    print("Invalid! cannot cast")

            if (count % 1000 == 0):
                file.flush()

            if (count % 300 == 0):
                print(writeCount / ((int(time.time()*10000) - startTime) * 0.0001))
                
        '''
            valueInInt = float(int(str(data.decode('utf-8'))))

            if (valueInInt > 10000):
                return

            if (count % 5 == 0):
                try:
                    #print(valueInInt)
                    values[:-1] = values[1:]; values[-1] = valueInInt
                    file.write(str(valueInInt)+"\n")
                    drawnow(plotValues)
                except ValueError:
                    print("Invalid! cannot cast")

            if (count % 200 == 0):
                file.flush()'''


        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            await client.start_notify(UART_TX_CHAR_UUID, handle_rx)
            loop = asyncio.get_running_loop()
            nus = client.services.get_service(UART_SERVICE_UUID)

            while True:
                # This waits until you type a line and press ENTER.
                # A real terminal program might put stdin in raw mode so that things
                # like CTRL+C get passed to the remote device.
                data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

                # data will be empty on EOF (e.g. CTRL+D on *nix)
                if not data:
                    break

                # some devices, like devices running MicroPython, expect Windows
                # line endings (uncomment line below if needed)
                # data = data.replace(b"\n", b"\r\n")

                # Writing without response requires that the data can fit in a
                # single BLE packet. We can use the max_write_without_response_size
                # property to split the data into chunks that will fit.

    if __name__ == "__main__":
        try:
            asyncio.run(uart_terminal())
        except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
            pass
