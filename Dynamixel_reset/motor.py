from dynamixel_sdk import *
import time

class Motor:
    def __init__(self):
        # Information for motors XM430-W350 (T and R)
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.BAUDRATE                    = 1000000    
        self.ADDR_VELOCITY               = 104

        self.TORQUE_ENABLE               = 1     # Value for enabling the torque
        self.TORQUE_DISABLE              = 0     # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

        # Check dynamixel wizard 2.0 for this information
        self.PROTOCOL_VERSION            = 2.0
        self.DXL_ID                      = 14
        self.DEVICENAME                  = '/dev/ttyUSB0'

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.open_port()
        self.change_baudrate()
        self.enable_torque()

        # Specific for the set up
        self.BOTTOM_POSITION = self.get_current_pos() #22168
        self.HEIGHT_RANGE = 17000

    def open_port(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            return

    def change_baudrate(self):
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            return
        
    def enable_torque(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return
        else:
            print("Dynamixel has been successfully connected")

    def get_current_pos(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return None
        else:
            return dxl_present_position

    def set_velocity(self, velocity = 0):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_VELOCITY, velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def move_top(self):
        # TODO check None
        self.set_velocity(velocity = 200)
        while self.get_current_pos() < self.BOTTOM_POSITION + self.HEIGHT_RANGE:
            time.sleep(0.1)

        self.set_velocity(velocity = 0)


    def move_bottom(self):
        self.set_velocity(velocity = -200)
        while self.get_current_pos() > self.BOTTOM_POSITION:
            time.sleep(0.1)

        self.set_velocity(velocity = 0)

    def reset(self):
        self.move_top()
        self.move_bottom()