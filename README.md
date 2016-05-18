# python code for 7bot arm control
=========

This code base uses python3, and the pySerial package to send commands to 7bot.

Follow the instructions [here](https://www.dropbox.com/s/nhwv8j5zvaynnf8/Getting%20Started%20with%207Bot%20v1.0.pdf?dl=0) to install the softwareSystem.

Use the following command to figure out the Serial port and change the port_name in config.py:
python3 -m serial.tools.list_ports

On my computer, it is /dev/cu.usbmodem1411.

simple test:
-----------
```python
>> from arm import Arm;
>> arm = Arm();
>> angles = {1:95,2:65}
>> arm.setAngle(angles)
>> arm.setForceStatus(0)
```

Reference:

https://github.com/7Bot/7Bot-Processing-Examples/blob/master/Arm7Bot_Com_Test/Arm7Bot_Com_Test.pde
