from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

rtde_io_ = RTDEIO("127.0.0.1")
rtde_receive_ = RTDEReceive("127.0.0.1")

# How-to set and get standard and tool digital outputs. Notice that we need the
# RTDEIOInterface for setting an output and RTDEReceiveInterface for getting the state
# of an output.

if rtde_receive_.getDigitalOutState(7):
    print("Standard digital out (7) is HIGH")
else:
    print("Standard digital out (7) is LOW")

if rtde_receive_.getDigitalOutState(16):
    print("Tool digital out (16) is HIGH")
else:
    print("Tool digital out (16) is LOW")

rtde_io_.setStandardDigitalOut(7, True)
rtde_io_.setToolDigitalOut(0, True)
time.sleep(0.01)

if rtde_receive_.getDigitalOutState(7):
    print("Standard digital out (7) is HIGH")
else:
    print("Standard digital out (7) is LOW")

if rtde_receive_.getDigitalOutState(16):
    print("Tool digital out (16) is HIGH")
else:
    print("Tool digital out (16) is LOW")

# How to set a analog output with a specified current ratio
rtde_io_.setAnalogOutputCurrent(1, 0.25)
