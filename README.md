# MPU9255-Quaternion-AHRS-STM32
Quaternion Based AHRS Estimation Using MPU9250 and STM32G431

Board used is: NUCLEO-G431RB

Detailed information: [NUCLEO-G431RB](https://www.st.com/en/evaluation-tools/nucleo-g431rb.html#overview)

[YouTube, Demo 1](https://www.youtube.com/watch?v=jB_gVwflhkY)

[YouTube, Demo 2](https://www.youtube.com/watch?v=BbwvnWbsJSQ)

[Blog Article](https://ibrahimcahitozdemir.com/2022/01/08/quaternion-based-ahrs-estimation-using-mpu9250-and-stm32g431/)

## Circuit:

![](https://raw.githubusercontent.com/ibrahimcahit/MPU9255-Quaternion-AHRS-STM32/main/MPU9250%20NEW/photo_2022-01-08_06-23-47.jpg)

## How to use?

Include needed librarys:

```
#include <stdio.h>
#include "mpu9255.h"
```

Create instance:

```
MPU9255_t MPU9255;
```

Add printf function properties:

```
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
```

Define needed variables:

```
float pitch;
float yaw;
float roll;
```

Init sensor and wait until init completes.
Also, add your I2C instance, &hi2c2 or &hi2c1 or &hi2c3...

```
while (MPU9255_Init(&hi2c2) == 1);
```

Delay 3 seconds, this is for after mag. calibration completes

```
HAL_Delay(3000);
```

Read all sensor data and calculate Quaternions:

```
readAll(&hi2c2, &MPU9255);
```

Assign calculated values to the variables

```
pitch = MPU9255.pitch;
yaw = MPU9255.yaw;
roll= MPU9255.roll;
```

## Bonus: 3D Visualising using Python and PyGame

Create variable count, right before nain While:

```
int count;
```

In while loop, print all axis data every 10 microsecond. This is for prevent UART overflow:

```
if (count == 10){
  printf("%f\t", yaw);
  printf("%f\t", pitch);
  printf("%f\t\n", roll);
  count = 0;
}
count++;
```

Download 3dBody.py and change: 

```
ser = serial.Serial('<PORT_NAME>', 115200)
```

<PORT_NAME> to the your Nucleo's COM port. 
