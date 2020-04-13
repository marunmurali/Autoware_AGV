# -*- coding: utf-8 -*-
"""
Created on Mon Apr 23 13:39:45 2018

@author: Nguyen Thanh Tung
@changed by: Shunichiro Sugiyama
"""
import binascii

import serial
import time

DATA_LENGTH, DATA_NUMBER, ACC_X_MSB, ACC_X_LSB, ACC_Y_MSB, ACC_Y_LSB, ACC_Z_MSB, ACC_Z_LSB, GYR_X_MSB, GYR_X_LSB, GYR_Y_MSB, GYR_Y_LSB, GYR_Z_MSB, GYR_Z_LSB, JOY_FRONT, JOY_SIDE, BATTERY_POWER, BATTERY_CURRENT_MSB, BATTERY_CURRETN_LSB, SPEED_MSB, SPEED_LSB, SEATSENSOR, POWER_ON, IS_SEAT_BACK, ERROR, ID_POSITION = range(0, 26)

FORWARD_SPEED_MAX, FORWARD_ACCEL, FORWARD_DECEL, REVERSE_SPEED_MAX, REVERSE_ACCEL, REVERSE_DECEL, TURN_SPEED_MAX, TURN_ACCEL, TURN_DECEL, HAND = range(2, 12)

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
                           baudrate = 19200,
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

    def StartSendingProfile(self, Interval):
        cmd = bytearray()
        cmd.append(0)
        # D0
        cmd.append(0)

        temp = int(Interval)
        #_int = temp.to_bytes(2, 'big')
        _int = to_bytes(Interval, 2)

        #print str(_int[0]), _int[1], "hex:", hex(Interval)

        # T0 - T1
        cmd.append(_int[0]) # LSB T0
        cmd.append(_int[1]) # MSB T1
        #Send_data_10_ms=b'\x0A'
        #cmd.append(Send_data_10_ms)

        # Send the command
        self.Send_CtrlCmd(5, cmd)

    def StartSendingData(self, Interval):
        cmd = bytearray()
        cmd.append(0)
        # D0
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

        # Send the command
        self.Send_CtrlCmd(5, cmd)

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
        cmd.append(4)
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

    def SetForward(self, max_speed, acc, dcc):
        """
        Set forward speed profiles.
        :param max_speed: Max speed (0~100)
        :param acc: Acceleration (0~100)
        :param dcc: Deceleration (0~100)
        :return: None
        """
        cmd = bytearray()
        cmd.append(5)

        cmd.append(max_speed)
        cmd.append(acc)
        cmd.append(dcc)

        # Send the command
        self.Send_CtrlCmd(5, cmd)
        time.sleep(1)
        print("Forward speed profile were changed.")

    def SetReverse(self, max_speed, acc, dcc):
        """
        Set Reverse speed profiles.
        :param max_speed: Max speed (0~100)
        :param acc: Acceleration (0~100)
        :param dcc: Deceleration (0~100)
        :return: None
        """
        cmd = bytearray()
        cmd.append(6)

        cmd.append(max_speed)
        cmd.append(acc)
        cmd.append(dcc)

        # Send the command
        self.Send_CtrlCmd(5, cmd)
        time.sleep(1)
        print("Reverse speed profile were changed.")

    def SetTurn(self, max_speed, acc, dcc):
        """
        Set Turn speed profiles.
        :param max_speed: Max speed (0~100)
        :param acc: Acceleration (0~100)
        :param dcc: Deceleration (0~100)
        :return: None
        """
        cmd = bytearray()
        cmd.append(7)

        cmd.append(max_speed)
        cmd.append(acc)
        cmd.append(dcc)

        # Send the command
        self.Send_CtrlCmd(5, cmd)
        time.sleep(1)
        print("Turn speed profile were changed.")

    def GetProfile( self ):
        """
        Get WHILLstate data (speed profile)
        """

        while True:
            header = self.cp.read(1)
            #if binascii.hexlify(bytearray( cmd )) == '0xAF':
            if ord(header[0]) == 175:
                break
        cmd = self.cp.read(12)
        self.cp.flush()

        data_length           = ord(cmd[DATA_LENGTH])
        data_number           = ord(cmd[DATA_NUMBER])
        fwd_max               = ord(cmd[FORWARD_SPEED_MAX])
        fwd_acc               = ord(cmd[FORWARD_ACCEL])
        fwd_dcc               = ord(cmd[FORWARD_DECEL])
        rev_max               = ord(cmd[REVERSE_SPEED_MAX])
        rev_acc               = ord(cmd[REVERSE_ACCEL])
        rev_dcc               = ord(cmd[REVERSE_DECEL])
        trn_max               = ord(cmd[TURN_SPEED_MAX])
        trn_acc               = ord(cmd[TURN_ACCEL])
        trn_dcc               = ord(cmd[TURN_DECEL])

        print("fwd: max=" + str(fwd_max) + " acc=" + str(fwd_acc) + " dcc=" + str(fwd_dcc))
        print("rev: max=" + str(rev_max) + " acc=" + str(rev_acc) + " dcc=" + str(rev_dcc))
        print("trn: max=" + str(trn_max) + " acc=" + str(trn_acc) + " dcc=" + str(trn_dcc))
        print("")

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

        acc_x_msb             = ord(cmd[ACC_X_MSB])
        acc_x_lsb             = ord(cmd[ACC_X_LSB])
        acc_y_msb             = ord(cmd[ACC_Y_MSB])
        acc_y_lsb             = ord(cmd[ACC_Y_LSB])
        acc_z_msb             = ord(cmd[ACC_Z_MSB])
        acc_z_lsb             = ord(cmd[ACC_Z_LSB])
        gyr_x_msb             = ord(cmd[GYR_X_MSB])
        gyr_x_lsb             = ord(cmd[GYR_X_LSB])
        gyr_y_msb             = ord(cmd[GYR_Y_MSB])
        gyr_y_lsb             = ord(cmd[GYR_Y_LSB])
        gyr_z_msb             = ord(cmd[GYR_Z_MSB])
        gyr_z_lsb             = ord(cmd[GYR_Z_LSB])

        joy_front             = ord(cmd[JOY_FRONT])
        joy_side              = ord(cmd[JOY_SIDE])
        battery_power         = ord(cmd[BATTERY_POWER])
        speed_msb             = ord(cmd[SPEED_MSB])
        speed_lsb             = ord(cmd[SPEED_LSB])
        power_on              = ord(cmd[POWER_ON])
        error                 = ord(cmd[ERROR])
        id_position           = ord(cmd[ID_POSITION])

        acc_x = (acc_x_msb <<8) | acc_x_lsb
        acc_y = (acc_y_msb <<8) | acc_y_lsb
        acc_z = (acc_z_msb <<8) | acc_z_lsb
        gyr_x = (gyr_x_msb <<8) | gyr_x_lsb
        gyr_y = (gyr_y_msb <<8) | gyr_y_lsb
        gyr_z = (gyr_z_msb <<8) | gyr_z_lsb
        speed = (speed_msb <<8) | speed_lsb

        if acc_x_msb >= 250:
            acc_x = acc_x - 65535
        if acc_y_msb >= 250:
            acc_y = acc_y - 65535
        if acc_z_msb >= 250:
            acc_z = acc_z - 65535
        if gyr_x_msb >= 250:
            gyr_x = gyr_x - 65535
        if gyr_y_msb >= 250:
            gyr_y = gyr_y - 65535
        if gyr_z_msb >= 250:
            gyr_z = gyr_z - 65535

        speed = int(speed_msb)
        if (speed >= 0):
            speed = speed + 0.1 * int(speed_lsb)
        else:
            speed = speed - 0.1 * int(speed_lsb)

        acc_x = acc_x * 0.122 * 0.001   #[g]
        acc_y = acc_y * 0.122 * 0.001   #[g]
        acc_z = acc_z * 0.122 * 0.001   #[g]
        gyr_x = gyr_x * 4.375 * 0.001   #[dps]
        gyr_y = gyr_y * 4.375 * 0.001   #[dps]
        gyr_z = gyr_z * 4.375 * 0.001   #[dps]

        if joy_front > 150:
            joy_front -= 256

        if joy_side > 150:
            joy_side -= 256

        if gyr_z_msb >= 250:
            gyr_z = gyr_z - 65535
        return acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, joy_front, joy_side, battery_power, speed, power_on, error, id_position

if __name__=='__main__':
    whill = WHILL('/dev/cu.usbserial')

    #for i in range(15):
        #whill.SetJoystick(0, 100, 50)
        #time.sleep(0.1)
    whill.SetPower(0)
    #whill.StartSendingData(0, 10, 0)
    #print(rec)
    #print(cmd)
