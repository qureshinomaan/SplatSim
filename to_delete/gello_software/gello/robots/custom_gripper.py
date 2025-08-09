import socket
import threading
import time
from enum import Enum
from typing import OrderedDict, Tuple, Union
import os
from dynamixel_sdk import * 

class CustomGripper:
    """A class representing a custom gripper."""
    
    def __init__(self):
        """Initialize the custom gripper."""
    
        # Control table address
        self.ADDR_MX_TORQUE_ENABLE      = 24 #64               # Control table address is different in Dynamixel model
        self.ADDR_MX_GOAL_POSITION      = 30 #116
        self.ADDR_MX_PRESENT_POSITION   = 36 #132

        # Protocol version
        self.PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = 8                 # Dynamixel ID : 8
        self.BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 2600           # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 3900            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.index = 0
        self.dxl_goal_position = [self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE]         # Goal position


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        
    def connect(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            # getch()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            # getch()
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")
            
    def move(self, trigger = False):
        
        # while 1:
        # if getch() == chr(0x1b):
            # break
        # if trigger:
        # Write goal position
        print('here in custom gripper, moving')
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_GOAL_POSITION, self.dxl_goal_position[int(trigger)])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # while 1:
        #     # Read present position
        #     dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_PRESENT_POSITION)
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.DXL_ID, self.dxl_goal_position[int(trigger)], dxl_present_position))

        #     if not abs(self.dxl_goal_position[int(trigger)] - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
        #         break

    
            
    def get_current_position(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result)) 
            print('error in position reading')
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print('error in position reading')
            
        print('current position', dxl_present_position)
        return dxl_present_position
            

if __name__ == "__main__":
    gripper = CustomGripper()
    
    gripper.connect()
        