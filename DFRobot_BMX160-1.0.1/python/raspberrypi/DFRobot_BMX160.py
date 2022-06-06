'''!
  @file DFRobot_BMX160.py
  @brief define DFRobot_BMX160 class infrastructure, the implementation of basic methods
  @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author [luoyufeng] (yufeng.luo@dfrobot.com)
  @maintainer [Fary](feng.yang@dfrobot.com)
  @version  V1.0
  @date  2021-10-20
  @url https://github.com/DFRobot/DFRobot_BMX160
 '''
from itertools import count
import sys
sys.path.append('../')
import smbus2
import time


Low_power_prest = (0x0)

class BMX160:
    """
    Class that represents the BMX160 sensor

    Returns:
        bus( int ): number of the i2c bus to which the sensor is connected to
    """
    
    def __init__(self, bus):
        self.i2cbus = smbus2.SMBus(bus)
        self.i2c_addr = self._BMX160_I2C_ADDR
        time.sleep(0.16)
        
    #bmx160 i2c addresses
    _BMX160_I2C_ADDR                 = (0x68)
    _BMX160_I2C_ADDR_ALT             = (0x69)
    #bmx160 Register Map
    _BMX160_CHIP_ID_ADDR             = (0x00)
    _BMX160_ERROR_REG_ADDR           = (0x02)
    _BMX160_MAG_DATA_ADDR            = (0x04)
    _BMX160_GYRO_DATA_ADDR           = (0x0C)
    _BMX160_ACCEL_DATA_ADDR          = (0x12)
    _BMX160_STATUS_ADDR              = (0x1B)
    _BMX160_INT_STATUS_ADDR          = (0x1C)
    _BMX160_FIFO_LENGTH_ADDR         = (0x22)
    _BMX160_FIFO_DATA_ADDR           = (0x24)
    _BMX160_ACCEL_CONFIG_ADDR        = (0x40)
    _BMX160_ACCEL_RANGE_ADDR         = (0x41)
    _BMX160_GYRO_CONFIG_ADDR         = (0x42)
    _BMX160_GYRO_RANGE_ADDR          = (0x43)
    _BMX160_MAGN_CONFIG_ADDR         = (0x44)
    _BMX160_FIFO_DOWN_ADDR           = (0x45)
    _BMX160_FIFO_CONFIG_0_ADDR       = (0x46)
    _BMX160_FIFO_CONFIG_1_ADDR       = (0x47)
    _BMX160_MAGN_RANGE_ADDR          = (0x4B)
    _BMX160_MAGN_IF_0_ADDR           = (0x4C)
    _BMX160_MAGN_IF_1_ADDR           = (0x4D)
    _BMX160_MAGN_IF_2_ADDR           = (0x4E)
    _BMX160_MAGN_IF_3_ADDR           = (0x4F)
    _BMX160_INT_ENABLE_0_ADDR        = (0x50)
    _BMX160_INT_ENABLE_1_ADDR        = (0x51)
    _BMX160_INT_ENABLE_2_ADDR        = (0x52)
    _BMX160_INT_OUT_CTRL_ADDR        = (0x53)
    _BMX160_INT_LATCH_ADDR           = (0x54)
    _BMX160_INT_MAP_0_ADDR           = (0x55)
    _BMX160_INT_MAP_1_ADDR           = (0x56)
    _BMX160_INT_MAP_2_ADDR           = (0x57)
    _BMX160_INT_DATA_0_ADDR          = (0x58)
    _BMX160_INT_DATA_1_ADDR          = (0x59)
    _BMX160_INT_LOWHIGH_0_ADDR       = (0x5A)
    _BMX160_INT_LOWHIGH_1_ADDR       = (0x5B)
    _BMX160_INT_LOWHIGH_2_ADDR       = (0x5C)
    _BMX160_INT_LOWHIGH_3_ADDR       = (0x5D)
    _BMX160_INT_LOWHIGH_4_ADDR       = (0x5E)
    _BMX160_INT_MOTION_0_ADDR        = (0x5F)
    _BMX160_INT_MOTION_1_ADDR        = (0x60)
    _BMX160_INT_MOTION_2_ADDR        = (0x61)
    _BMX160_INT_MOTION_3_ADDR        = (0x62)
    _BMX160_INT_TAP_0_ADDR           = (0x63)
    _BMX160_INT_TAP_1_ADDR           = (0x64)
    _BMX160_INT_ORIENT_0_ADDR        = (0x65)
    _BMX160_INT_ORIENT_1_ADDR        = (0x66)
    _BMX160_INT_FLAT_0_ADDR          = (0x67)
    _BMX160_INT_FLAT_1_ADDR          = (0x68)
    _BMX160_FOC_CONF_ADDR            = (0x69)
    _BMX160_CONF_ADDR                = (0x6A)
    _BMX160_IF_CONF_ADDR             = (0x6B)
    _BMX160_SELF_TEST_ADDR           = (0x6D)
    _BMX160_OFFSET_ADDR              = (0x71)
    _BMX160_OFFSET_CONF_ADDR         = (0x77)
    _BMX160_INT_STEP_CNT_0_ADDR      = (0x78)
    _BMX160_INT_STEP_CONFIG_0_ADDR   = (0x7A)
    _BMX160_INT_STEP_CONFIG_1_ADDR   = (0x7B)
    _BMX160_COMMAND_REG_ADDR         = (0x7E)
    
    BMX160_SOFT_RESET_CMD           = (0xb6)
    BMX160_MAGN_UT_LSB              = (0.3)
    _BMX160_ACCEL_MG_LSB_2G          = (0.000061035)
    _BMX160_ACCEL_MG_LSB_4G          = (0.000122070)
    _BMX160_ACCEL_MG_LSB_8G          = (0.000244141)
    _BMX160_ACCEL_MG_LSB_16G         = (0.000488281)
    
    _BMX160_GYRO_SENSITIVITY_125DPS  = (0.0038110)
    _BMX160_GYRO_SENSITIVITY_250DPS  = (0.0076220)
    _BMX160_GYRO_SENSITIVITY_500DPS  = (0.0152439)
    _BMX160_GYRO_SENSITIVITY_1000DPS = (0.0304878)
    _BMX160_GYRO_SENSITIVITY_2000DPS = (0.0609756)
    
    GyroRange_125DPS                 = (0x00)
    GyroRange_250DPS                 = (0x01)
    GyroRange_500DPS                 = (0x02)
    GyroRange_1000DPS                = (0x03)
    GyroRange_2000DPS                = (0x04)
    
    AccelRange_2G                    = (0x00)
    AccelRange_4G                    = (0x01)
    AccelRange_8G                    = (0x02)
    AccelRange_16G                   = (0x03)
    
    accelRange = _BMX160_ACCEL_MG_LSB_2G
    gyroRange = _BMX160_GYRO_SENSITIVITY_250DPS
    
    #Magnometer Preset values
    _LOW_POWER_PRESET                = (0x01, 0x02)
    _REGULAR_PRESET                  = (0x04, 0x0E)
    _ENHANCED_REGULAR_PRESET         = (0x07, 0x1A)
    _HIGH_ACCURACY_PRESET            = (0x17, 0x52)
    
    #PMU MODES
    _PMU_ACCEL_NORMAL                = (0x11)
    _PMU_ACCEL_LP                    = (0x12)
    _PMU_GYRO_NROMAL                 = (0x15)   
    _PMU_GYRO_STARTUP                = (0x17)
    _PMU_MAG_IF_SUS                  = (0x18)
    _PMU_MAG_IF_NORMAL               = (0x19)
    _PMU_MAG_IF_LP                   = (0x1A)
    
    def begin(self):
        '''!
          @brief initialization the i2c.
          @return returns the initialization status
          @retval True Initialization succeeded
          @retval False Initialization  failed
        '''
        if not self.scan():
            return False
        else:
            self.soft_reset()
            self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x11)
            time.sleep(0.05)
            self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x15)
            time.sleep(0.1)
            self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x19)
            time.sleep(0.01)
            #Enable headerless mode and Set fifo to receive magne data
            self.write_bmx_reg(self._BMX160_FIFO_CONFIG_1_ADDR, 0x20)
            self.set_magn_conf()

            return True
        

    def set_low_power(self):
        '''!
          @brief disabled the the magn, gyro sensor to reduce power consumption
        '''
        self.soft_reset()
        time.sleep(0.1)
        self.set_magn_conf()
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x12)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x17)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x1A)
        time.sleep(0.1)

    def wake_up(self):
        '''!
          @brief enabled the the magn, gyro sensor
        '''
        self.soft_reset()
        time.sleep(0.1)
        self.set_magn_conf()
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x11)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x15)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x19)
        time.sleep(0.1)

    def soft_reset(self):
        '''!
          @brief reset bmx160 hardware
          @return returns the reset status
          @retval True reset succeeded
          @retval False reset  failed
        '''
        data = self.BMX160_SOFT_RESET_CMD
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, data)
        time.sleep(0.015)
        #self.defaultParamSettg()
        return True

    def set_magn_conf(self):
        '''!
          @brief  set magnetometer Config
        '''
        #Set MAG_IF into Manual/Setup Mode
        self.write_bmx_reg(self._BMX160_MAGN_IF_0_ADDR, 0x80)
        time.sleep(0.05)
        
        #Set Mag into sleep mode, this is required in order to set Preset vals
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x01)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x4B)
        
        #Config REPXY
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x17)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x51)
        
        #Config REPZ
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x52)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x52)
        
        #Prepare MAG_IF for Data mode
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x02)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x4C)
        self.write_bmx_reg(self._BMX160_MAGN_IF_1_ADDR, 0x42)
        
        #Set ODR
        self.write_bmx_reg(self._BMX160_MAGN_CONFIG_ADDR, 0x05)
        
        #Set MAG_IF in Data mode w/ 3 byte config to be read from magnetometer
        self.write_bmx_reg(self._BMX160_MAGN_IF_0_ADDR, 0x03)
        time.sleep(0.05)

    def set_gyro_range(self, bits):
        '''!
          @brief set gyroscope angular rate range and resolution.
          @param bits 
          @n       GyroRange_125DPS      Gyroscope sensitivity at 125dps
          @n       GyroRange_250DPS      Gyroscope sensitivity at 250dps
          @n       GyroRange_500DPS      Gyroscope sensitivity at 500dps
          @n       GyroRange_1000DPS     Gyroscope sensitivity at 1000dps
          @n       GyroRange_2000DPS     Gyroscope sensitivity at 2000dps
        '''
        if bits == 0:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_125DPS
        elif bits == 1:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_250DPS
        elif bits == 2:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_500DPS
        elif bits == 3:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_1000DPS
        elif bits == 4:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_2000DPS
        else:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_250DPS

    def set_accel_range(self, bits):
        '''!
          @brief allow the selection of the accelerometer g-range.
          @param bits 
          @n       AccelRange_2G        Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) 
          @n       AccelRange_4G        Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) 
          @n       AccelRange_8G        Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) 
          @n       AccelRange_16G       Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg)
        '''
        if bits == 0:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_2G
        elif bits == 1:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_4G
        elif bits == 2:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_8G
        elif bits == 3:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_16G
        else:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_2G

    def get_all_data(self):
        '''!
          @brief get the magn, gyro and accel data 
          @return all data
        '''
        data = self.read_bmx_reg(self._BMX160_MAG_DATA_ADDR, 20)
        if (data[1] & 0x80):
            magnx = - 0x10000 + ((data[1] << 8) | (data[0]))
        else:
            magnx =  (data[1] << 8) | (data[0])
        if (data[3] & 0x80):
            magny = - 0x10000 + ((data[3] << 8) | (data[2]))
        else:
            magny =  (data[3] << 8) | (data[2])
        if (data[5] & 0x80):
            magnz = - 0x10000 + ((data[5] << 8) | (data[4]))
        else:
            magnz =  (data[5] << 8) | (data[4])

        if (data[9] & 0x80):
            gyrox = - 0x10000 + ((data[9] << 8) | (data[8]))
        else:
            gyrox =  (data[9] << 8) | (data[8])
        if (data[11] & 0x80):
            gyroy = - 0x10000 + ((data[11] << 8) | (data[10]))
        else:
            gyroy =  (data[11] << 8) | (data[10])
        if (data[13] & 0x80):
            gyroz = - 0x10000 + ((data[13] << 8) | (data[12]))
        else:
            gyroz =  (data[13] << 8) | (data[12])

        if (data[15] & 0x80):
            accelx = - 0x10000 + ((data[15] << 8) | (data[14]))
        else:
            accelx =  (data[15] << 8) | (data[14])
        if (data[17] & 0x80):
            accely = - 0x10000 + ((data[17] << 8) | (data[16]))
        else:
            accely =  (data[17] << 8) | (data[16])
        if (data[19] & 0x80):
            accelz = - 0x10000 + ((data[19] << 8) | (data[18]))
        else:
            accelz =  (data[19] << 8) | (data[18])
        
        magnx *= self.BMX160_MAGN_UT_LSB
        magny *= self.BMX160_MAGN_UT_LSB
        magnz *= self.BMX160_MAGN_UT_LSB
        
        gyrox *= self.gyroRange
        gyroy *= self.gyroRange
        gyroz *= self.gyroRange
        
        accelx *= self.accelRange * 9.8
        accely *= self.accelRange * 9.8
        accelz *= self.accelRange * 9.8
        out_put = []
        out_put.append(magnx)
        out_put.append(magny)
        out_put.append(magnz)
        out_put.append(gyrox)
        out_put.append(gyroy)
        out_put.append(gyroz)
        out_put.append(accelx)
        out_put.append(accely)
        out_put.append(accelz)
        return out_put

    def write_bmx_reg(self, register, value):
        '''!
          @brief Write data to the BMX register
          @param register register
          @param value  Data written to the BMX register
          @return return the actually written length
        '''
        self.i2cbus.write_byte_data(self.i2c_addr, register, value)

    def read_bmx_reg(self, register, block_size):
        '''!
          @brief Read BMX register data
          @param register register
          @return data
        '''
        return self.i2cbus.read_i2c_block_data(self.i2c_addr, register, block_size)

    def scan(self):
        '''!
          @brief  iic scan function
          @return scan result
          @retval True sensor exist
          @retval False There is no sensor
        '''
        try:
            self.i2cbus.read_byte(self.i2c_addr)
            return True
        except:
            print("I2C init fail")
            return False
        
    def get_fifo_size(self):
        """
         Get the number of frames to read from fifo
        Returns:
            size ( list ): 2 bytes representing the number of frames to read in fifo
        """
        data_size = self.i2cbus.read_i2c_block_data(self.i2c_addr, self._BMX160_FIFO_LENGTH_ADDR, 2)
        return ( ( data_size[1] << 8 ) | data_size[0] )
    
    def get_fifo_data(self):
        #Check if there is data to be read
        data_size = self.get_fifo_size()
        
        if data_size == 0:
            return False
        else:
            print(data_size)
            if data_size > 32:
                data_size = 32
            Data = self.i2cbus.read_i2c_block_data(self.i2c_addr, self._BMX160_FIFO_DATA_ADDR, data_size)
        
        #Check curent fifo config in order to know how to parse the frames
        fifo_conf = self.i2cbus.read_i2c_block_data( self.i2c_addr, self._BMX160_FIFO_CONFIG_1_ADDR, 1)
 
        #in case only magnetometer data is writen to fifo
        if fifo_conf[0] == 32:
            magneto_x = []
            magneto_y = []
            magneto_z = []
            magnet_data = []
            counter = 0
            while( counter != data_size ):
                #store each axis in it's own array / Perform also conversion from bytes to int in 2complement
                magneto_x.append( int.from_bytes(   ( ( Data[ counter + 1] << 8 ) | ( Data[ counter ] ) ).to_bytes( 2, byteorder=sys.byteorder ), 
                                                    sys.byteorder, signed=True ) )
                magneto_y.append( int.from_bytes(   ( ( Data[ counter + 3] << 8 ) | ( Data[ counter + 2] ) ).to_bytes( 2, byteorder=sys.byteorder ), 
                                                    sys.byteorder, signed=True ) )
                magneto_z.append( int.from_bytes(   ( ( Data[ counter + 5] << 8 ) | ( Data[ counter + 4] ) ).to_bytes( 2, byteorder=sys.byteorder ), 
                                                    sys.byteorder, signed=True ) )
                counter += 8
            magnet_data.append( magneto_x )
            magnet_data.append( magneto_y )
            magnet_data.append( magneto_z )
            return magnet_data
        return False
        