# -*- coding: utf-8 -*-
"""
Created on Mon Apr 23 13:39:45 2018

@author: Nguyen Thanh Tung
"""
import binascii

import serial
import time

DATA_LENGTH, DATA_NUMBER, ACC_X_MSB, ACC_X_LSB, ACC_Y_MSB, ACC_Y_LSB, ACC_Z_MSB, ACC_Z_LSB, GYR_X_MSB, GYR_X_LSB, GYR_Y_MSB, GYR_Y_LSB, GRY_Z_MSB, GYR_Z_LSB, JOY_FRONT, JOY_SIDE, BATTERY_POWER, BATTERY_CURRENT_MSG, BATTERY_CURRETN_LSB, RIGHT_MOTOR_ANGLE_MSG, RIGHT_MOTOR_ANGLE_LSB, LEFT_MOTOR_ANGLE_MSB, LEFT_MOTOR_ANGLE_LSB, RIGHT_MOTOR_SPEED_MSB, RIGHT_MOTOR_SPEED_LSB, LEFT_MOTOR_SPEED_MSB, LEFT_MOTOR_SPEED_LSB, POWER_ON, ERROR = range(0, 29)

def to_bytes(n, length, endianess='big'):
    h = '%x' % n
    s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
    return s if endianess == 'big' else s[::-1]

def my_to_bytes(n, length):
    return bytes( (n >> i*8) & 0xff for i in reversed(range(length)))


class WHILL_Cmd:
    def __init__(self):
        self.StartSendingData = b'\x00'
        self.StopSendingData = b'\x01'
        self.SetPower = b'\x02'
        self.SetJoystick = b'\x03'
        self.SetSpeedProfile = b'\x04'
        self.SetBatteryVoltageOut = b'\x05'
        self.Send_data_10_ms=b'\x0A'
        
class WHILL:
    def __init__(self, comport):
        self.port = comport
        self.Initialize()
        
    def Initialize(self):
        self.cp = serial.Serial(port = self.port, 
                           baudrate = 38400,
                                stopbits = 2)
        try:
            if (self.cp.isOpen()):
                self.cp.close()
            self.cp.open()
        except ValueError as e:
            raise ValueError('Error to open port')
                       
    def Send_CtrlCmd(self, length, CtrlCmd):
        cmd = bytearray()
        # Protocol sign
        protocol = int('af', 16)
        cmd.append(protocol)
        
        # Add len & control cmd        
        cmd.append(length)
        for i in CtrlCmd:
            cmd.append(i)
        
        # Create checksum
        checksum = 0
        for i in range(len(cmd)):
            checksum ^= cmd[i]
            
        cmd.append(checksum)
        
        # Send cmd
        self.cp.write(cmd)
        self.cp.flush()
        #print(cmd)
        return cmd
    
    def StartSendingData(self, DataSet, Interval, SpeedMode):
        cmd = bytearray()
        cmd.append(0)
        # D0
        if DataSet == 0:
            cmd.append(0)
        else:
            cmd.append(1)

        temp = int(Interval)
        #_int = temp.to_bytes(2, 'big')
        _int = to_bytes(Interval, 2)

        #print str(_int[0]), _int[1], "hex:", hex(Interval)
        
        # T0 - T1 
        cmd.append(_int[0]) # LSB T0
        cmd.append(_int[1]) # MSB T1
        #Send_data_10_ms=b'\x0A'
        #cmd.append(Send_data_10_ms)
        
        # Speed Mode
        cmd.append(SpeedMode)
        
        # Send the command
        self.Send_CtrlCmd(6, cmd)
        
    def StopSendingData(self):
        cmd = bytearray()
        cmd.append(1)
    
        # Send the command
        self.Send_CtrlCmd(2, cmd)
        
    def SetPower(self, P0):
        """
        Set Power parameters
        :param P0: Turn ON/OFF WHILL from Host machine,  0 - OFF, 1 - ON
        :return: None
        """
        cmd = bytearray()
        cmd.append(2)
        # P0
        if P0 == 0:
            cmd.append(0)
        else:
            cmd.append(1)
        
        # Send the command
        self.Send_CtrlCmd(3, cmd)        
        
    def SetJoystick(self, U0, FB, LR):
        """
        Set Joystick parameters
        :param U0: Active/Deactive control joystick from Host machine,  0 - Active, 1 - Deactive
        :param FB: Move forward/backward, FB > 0 - foward, FB < 0 - backward
        :param LR: Move right/left, LR > 0 - right, LR < 0 - left
        :return: None
        """
        cmd = bytearray()
        cmd.append(3)
        # U0
        if U0 == 0:
            cmd.append(0)
        else:
            cmd.append(1)
        
        # FB, LR 
        if (U0 == 0):
            if (FB < 0):
                FB = 256 + FB            
            if (LR < 0):
                LR = 256 + LR
                
            cmd.append(FB) # FB
            cmd.append(LR) # LR
        else:
            cmd.append(0)
            cmd.append(0)
        
        # Send the command
        self.Send_CtrlCmd(5, cmd)
        
        

    def GetData( self ):
        """
        Get WHILLstate data
        """
        #cmd    = bytearray()
        #header = bytearray()
        #cmd = self.cp.read(31)

        while True:
            header = self.cp.read(1)
            #if binascii.hexlify(bytearray( cmd )) == '0xAF':
            if ord(header[0]) == 175:
                break
        cmd = self.cp.read(30)
        self.cp.flush()

        data_length           = ord(cmd[DATA_LENGTH])
        data_number           = ord(cmd[DATA_NUMBER])
        joy_front             = ord(cmd[JOY_FRONT])
        joy_side              = ord(cmd[JOY_SIDE])
        battery_power         = ord(cmd[BATTERY_POWER])
        right_motor_speed_msb = ord(cmd[RIGHT_MOTOR_SPEED_MSB])
        right_motor_speed_lsb = ord(cmd[RIGHT_MOTOR_SPEED_LSB])
        left_motor_speed_msb  = ord(cmd[LEFT_MOTOR_SPEED_MSB])
        left_motor_speed_lsb  = ord(cmd[LEFT_MOTOR_SPEED_LSB])
        power_on              = ord(cmd[POWER_ON])
        error                 = ord(cmd[ERROR])


        if joy_front > 150:
            joy_front -= 256

        if joy_side > 150:
            joy_side -= 256
        right_motor_speed = (right_motor_speed_msb <<8) | right_motor_speed_lsb
        left_motor_speed  = (left_motor_speed_msb <<8) | left_motor_speed_lsb
        
        if right_motor_speed_msb >= 250:
            if right_motor_speed_lsb == 0:
                right_motor_speed = 0
            else:                
                right_motor_speed = right_motor_speed - 65535


        if left_motor_speed_msb >= 250:
            if left_motor_speed_lsb == 0:
                left_motor_speed = 0
            else:                
                left_motor_speed = left_motor_speed - 65535


        right_motor_speed = right_motor_speed * -0.00111 #m/sec
        left_motor_speed  = left_motor_speed  * 0.00111  #m/sec

        #print  "right", right_motor_speed_msb,  (right_motor_speed_lsb ), (right_motor_speed) , right_motor_speed
        #print  "left ", left_motor_speed_msb,  left_motor_speed_lsb, left_motor_speed 

        
        return joy_front, joy_side, battery_power, right_motor_speed, left_motor_speed, power_on, error


if __name__=='__main__':
    whill = WHILL('/dev/cu.usbserial')
    
    #for i in range(15):
        #whill.SetJoystick(0, 100, 50)
        #time.sleep(0.1)
    whill.SetPower(0)
    #whill.StartSendingData(0, 10, 0)
    #print(rec)
    #print(cmd)
        
