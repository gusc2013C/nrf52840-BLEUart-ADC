# BLEUart-ADC 手册 Ver 1.0

## nrf52840代码使用 

* prj.conf ：驱动开关 如果不添加新功能不需要动
* app.overlay ：自定义DeviceTree ADC端口定义在这里 
* Kconfig ：通讯设置

核心代码部分：
``` cpp
void ble_write_thread(void)  //蓝牙发送线程
{
	while (1)
	{
		struct fifo_data *buf = k_fifo_get(&fifo_bt_data,  
						     K_FOREVER);  //异步运行需要通过队列发送信息 如果队列没有信息 这个线程会被暂停
		if (bt_nus_send(NULL, buf -> str, buf -> len)) ; //发送信息 如果失败则... （目前为空）
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);  //定义一个从main开始就执行的线程 PRIORITY为3 main为0 main的优先级大于blewrite 所以main需要通过k_sleep为ble线程留出时间执行
```

``` cpp

	groupcnt = 10;  //每组发送的数据个数

	int64_t time_stamp;
	int64_t milliseconds_spent;
	int16_t cntnum = 0;  //避免取余运算占用时间

	/* capture initial time stamp */

	while (1) {
		//printk("ADC reading[%u]:\n", count++);
		count++;
		cntnum++;

		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {  //目前adc_channels只有一个通道，可以通过修改app.overlay重新定义通道，实现多通道采集
			int32_t val_mv;

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			err = adc_read(adc_channels[i].dev, &sequence); //读adc数据
			if (err < 0) {
				printk("Could not read (%d)\n", err);
				continue;
			}

			if (adc_channels[i].channel_cfg.differential) {
				val_mv = (int32_t)((int16_t)buf);
			} else {
				val_mv = (int32_t)buf;
			}  
			//printk("%"PRId32, val_mv);
			err = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv); //将原始数据转换为mV

			if (err < 0) {
				printk(" (value in mV not available)\n");
			} else {
				//printk(" = %"PRId32" mV\n", val_mv);
			}

			val_mv += 3000; //加3000避免出现负数，在接收端减去3000为原值

			if (cntnum == groupcnt) //这次采集需要蓝牙发送数据
			{
				str[((groupcnt-1) << 1)] = val_mv & 0xFF;
				str[((groupcnt-1) << 1) | 1] = val_mv >> 8;
				str[((groupcnt-1) << 1) + 2] = '\0'; //位运算 第2n-2个数表示后八位，第2n-1个数表示前八位

				tx_data.len = strlen(str);
				strcpy(tx_data.str, str);

				k_fifo_put(&fifo_bt_data, &tx_data); //送到队列里等待蓝牙线程发送
				memset(str ,0 , sizeof str);
				flagSend = true; //代码忘删了
				cntnum = 0;
			}
			else
			{
				str[((cntnum-1) << 1)] = val_mv & 0xFF;
				str[((cntnum-1) << 1) | 1] = val_mv >> 8;
			}

			if (count == 10000)
			{
				printk("START\n"); //性能测试 可以删除 测量10000次测量需要的时间 通过串口发送
				time_stamp = k_uptime_get();
			}
			else if (count == 20000)
			{
				printk("%lli\n", (k_uptime_get() - time_stamp));
			}
		}

		//dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_usleep(3000); //单位微秒，延时3ms采样率在320左右 延时2ms在450左右 过低采集数据会出问题，但我现在还没有测出来最低可以到多少

	}

```

* app.overlay定义：
``` DeviceTree
/ {
	zephyr,user {
		io-channels = <&adc 7>; //在这里添加其他通道 定义在下面
	};
	chosen {
		nordic,nus-uart = &uart0;
	};
};
&adc {
	#address-cells = <1>;
	#size-cells = <0>;

	channel@0 {
		reg = <0>;
		zephyr,gain = "ADC_GAIN_1_5"; //增益 影响噪音和分辨率
		zephyr,reference = "ADC_REF_INTERNAL"; //参考电压 决定adc量程和噪声 VDD参考噪音很大 可以外接供电来做参考电压
		zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>; //没动过 别动
		zephyr,input-positive = <NRF_SAADC_AIN1>; /* P0.03 */ //定义正极针脚 序号定义见datasheet or google
		zephyr,resolution = <12>; //分辨率 硬件最大支持12 14需要超采样 似乎会降精度
	};

	channel@1 {
		reg = <1>;
		zephyr,gain = "ADC_GAIN_1_6";
		zephyr,reference = "ADC_REF_INTERNAL";
		zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
		zephyr,input-positive = <NRF_SAADC_VDD>;
		zephyr,resolution = <14>; //使用超采样
		zephyr,oversampling = <8>;
	};

	channel@7 {
		reg = <7>;
		zephyr,gain = "ADC_GAIN_1";
		zephyr,reference = "ADC_REF_INTERNAL";
		zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
		zephyr,input-positive = <NRF_SAADC_AIN1>; /* P0.30 */
		zephyr,input-negative = <NRF_SAADC_AIN2>; /* P0.31 */ //定义参考负极
		zephyr,resolution = <12>;
	};
};

```

# Receiver使用

## Setup
目前我使用的是Bleak库，可能是目前唯一的跨平台BLE python库，但是对除Linux外的其他系统支持很差，Windows和Mac比较随缘，应该跑不起来。之后我会想办法在别的系统跑起来

物理机Ubuntu系统正常使用即可

虚拟机需要将蓝牙设备发送到虚拟机才可运行

* 首先将宿主机的蓝牙开关断开，在Vmware的虚拟机->可移动设备中找到蓝牙，点连接到虚拟机

* 在虚拟机的设置中找到蓝牙，开启后看是否能够正常搜索设备，如果能搜到设备，那么可以运行receiver

需要安装的库：

    可以用conda安装： numpy matplotlib
    只能用pip安装： bleak drawnow asyncio 

运行：

    python receiver.py

## 代码

``` python
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
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" #代码寻找正确的UUID进行连接

values = np.zeros(1000) #储存数据的数组 开的越大显示的点越多 图像移动越慢 过大可能导致画图速度追不上接收速度

plt.ion()#打开交互模式

#预加载虚拟数据

plt.legend(loc='upper right')    #对图像的定义 实际没有工作
plt.title('SAADC from NRF52800')
plt.grid(True)
plt.ylabel('Data')
 
def plotValues():
    plt.plot(values, label='Data') #图像实时更新函数 把上方代码复制进去可以让上方代码工作 但会降低运行效率

count = 0
startReceiveFlag = 0
startTime = 0
writeCount = 0

filenm = str(datetime.now()) + ".csv"

with open(filenm, mode='w', newline='') as file:

    async def uart_terminal():

        def match_nus_uuid(device: BLEDevice, adv: AdvertisementData): #匹配uuid连接nrf52840
            if UART_SERVICE_UUID.lower() in adv.service_uuids:
                return True

            return False

        device = await BleakScanner.find_device_by_filter(match_nus_uuid) #连接

        if device is None:
            print("no matching device found, you may need to edit match")
            sys.exit(1)

        def handle_disconnect(_: BleakClient):
            print("Device was disconnected, goodbye.")
            file.close()
            # cancelling all tasks effectively ends the program
            for task in asyncio.all_tasks():
                task.cancel()
            exit() #我没做好多线程程序退出 所以需要手动在console输入ctrl+c停止程序

        def handle_rx(_: BleakGATTCharacteristic, data: bytearray): #处理接受数据
            global count,startReceiveFlag,startTime,writeCount
            
            if startReceiveFlag == 0:
                startReceiveFlag = 1
                startTime = int(time.time()*10000) #保留0.1ms精度时间戳

            #print("received:", data.hex())
            #print(len(data))
            count = count + 1

            recstr = data.hex() #将bytearray转换为16进制字符串
            strlen = len(data)

            aver = 0

            for i in range(0,int(strlen/2)):
                inttmp1 = int(recstr[i*4:i*4+2],16) #把两个8进制字符串拼成16进制数
                inttmp2 = int(recstr[i*4+2:i*4+4],16)
                ans = inttmp2 * 256 + inttmp1 - 3000
                file.write(str((int(time.time()*10000) - startTime) * 0.0001 )+","+str(ans)+"\n") #写入数据及时间戳
                writeCount += 1
                aver += ans 

            aver /= int(strlen/2) #计算平均数 为节约性能 图像显示的是这十个点的平均值
            values[:-1] = values[1:]; values[-1] = aver
                
            if (count % 30 == 0): #每30次接受数据更新一次图像 最小值大概是5 再低会卡
                try:
                    #print(valueInInt)
                    drawnow(plotValues)
                    #valueInInt = 0  
                except ValueError:
                    print("Invalid! cannot cast")

            if (count % 1000 == 0): #约20秒保存一次文件 似乎有点慢 可以把数字减小一点点
                file.flush()

            if (count % 300 == 0): #约10秒再console里输出当前时间的平均采样率
                print(writeCount / ((int(time.time()*10000) - startTime) * 0.0001))


        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            await client.start_notify(UART_TX_CHAR_UUID, handle_rx)
            loop = asyncio.get_running_loop()
            nus = client.services.get_service(UART_SERVICE_UUID)

            while True:

                data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

                # data will be empty on EOF (e.g. CTRL+D on *nix)
                if not data:
                    break

    if __name__ == "__main__":
        try:
            asyncio.run(uart_terminal())
        except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
            pass


```