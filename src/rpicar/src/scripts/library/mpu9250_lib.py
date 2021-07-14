import smbus
import time

class mpu9250:
    # MPU6050 Registers
    MPU6050_ADDR = 0x68
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_PIN_CFG  = 0x37
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    TEMP_OUT_H   = 0x41
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47
    #AK8963 registers
    AK8963_ADDR   = 0x0C
    AK8963_ST1    = 0x02
    HXH          = 0x04
    HYH          = 0x06
    HZH          = 0x08
    AK8963_ST1   = 0x02
    AK8963_ST2   = 0x09
    AK8963_CNTL  = 0x0A
    AK8963_ASAX = 0x10
    def __init__(self):
        self.mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT

        # start I2C driver
        self.bus = smbus.SMBus(1) # start comm with i2c bus
        time.sleep(0.1)
        self.gyro_sens, self.accel_sens = self.MPU6050_start() # instantiate gyro/accel
        time.sleep(0.1)
        AK8963_coeffs = self.AK8963_start() # instantiate magnetometer
        time.sleep(0.1)
        
    def MPU6050_start(self):
        # reset all sensors
        self.bus.write_byte_data(self.MPU6050_ADDR,self.PWR_MGMT_1,0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(self.MPU6050_ADDR,self.PWR_MGMT_1,0x00)
        time.sleep(0.1)
        # power management and crystal settings
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # alter sample rate (stability)
        samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, samp_rate_div)
        time.sleep(0.1)
        #Write to Configuration register
        self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 0)
        time.sleep(0.1)
        #Write to Gyro configuration register
        gyro_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
        gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
        gyro_indx = 0
        self.bus.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
        time.sleep(0.1)
        #Write to Accel configuration register
        accel_config_sel = [0b00000, 0b01000, 0b10000, 0b11000] # byte registers
        accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
        accel_indx = 0
        self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
        time.sleep(0.1)
        # interrupt register (related to overflow of data [FIFO])
        self.bus.write_byte_data(self.MPU6050_ADDR,self.INT_PIN_CFG,0x22)
        time.sleep(0.1)
        # enable the AK8963 magnetometer in pass-through mode
        self.bus.write_byte_data(self.MPU6050_ADDR, self.INT_ENABLE, 1)
        time.sleep(0.1)
        return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]

    def read_raw_bits(self, register):
        # read accel and gyro values
        high = self.bus.read_byte_data(self.MPU6050_ADDR, register)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, register+1)

        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        
        # convert to +- value
        if(value > 32768):
            value -= 65536
        return value

    def mpu6050_conv(self):
        # raw acceleration bits
        acc_x = self.read_raw_bits(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_bits(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_bits(self.ACCEL_ZOUT_H)
        
        # raw gyroscope bits
        gyro_x = self.read_raw_bits(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_bits(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_bits(self.GYRO_ZOUT_H)

        #convert to acceleration in g and gyro dps
        a_x = (acc_x/(2.0**15.0))*self.accel_sens
        a_y = (acc_y/(2.0**15.0))*self.accel_sens
        a_z = (acc_z/(2.0**15.0))*self.accel_sens

        w_x = (gyro_x/(2.0**15.0))*self.gyro_sens
        w_y = (gyro_y/(2.0**15.0))*self.gyro_sens
        w_z = (gyro_z/(2.0**15.0))*self.gyro_sens
        
        return a_x,a_y,a_z,w_x,w_y,w_z
    
    def read_temp(self):
        temp = self.read_raw_bits(self.TEMP_OUT_H)
        temp = ((temp - 21.0)/333.87) + 21.0
        return temp

    def AK8963_start(self):
        self.bus.write_byte_data(self.AK8963_ADDR,self.AK8963_CNTL,0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.AK8963_ADDR,self.AK8963_CNTL,0x0F)
        time.sleep(0.1)
        coeff_data = self.bus.read_i2c_block_data(self.AK8963_ADDR,self.AK8963_ASAX,3)
        AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
        AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
        AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
        time.sleep(0.1)
        self.bus.write_byte_data(self.AK8963_ADDR,self.AK8963_CNTL,0x00)
        time.sleep(0.1)
        AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
        AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
        AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
        self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL, AK8963_mode)
        time.sleep(0.1)
        return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
    
    def AK8963_reader(self, register):
        # read magnetometer values
        low = self.bus.read_byte_data(self.AK8963_ADDR, register-1)
        high = self.bus.read_byte_data(self.AK8963_ADDR, register)
        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        # convert to +- value
        if(value > 32768):
            value -= 65536
        
        return value

    def AK8963_conv(self):
        # raw magnetometer bits
        while 1:
    ##        if ((bus.read_byte_data(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
    ##            return 0,0,0
            mag_x = self.AK8963_reader(self.HXH)
            mag_y = self.AK8963_reader(self.HYH)
            mag_z = self.AK8963_reader(self.HZH)

            # the next line is needed for AK8963
            if (self.bus.read_byte_data(self.AK8963_ADDR, self.AK8963_ST2)) & 0x08!=0x08:
                break
            
        #convert to acceleration in g and gyro dps
    ##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
    ##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
    ##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
        m_x = (mag_x/(2.0**15.0))*self.mag_sens
        m_y = (mag_y/(2.0**15.0))*self.mag_sens
        m_z = (mag_z/(2.0**15.0))*self.mag_sens
        return m_x,m_y,m_z