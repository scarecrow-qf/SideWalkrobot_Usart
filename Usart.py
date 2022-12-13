import time
import serial
import struct
import binascii
'''
    Author: Liu Yuxiang 
    Time: 2022.12.13
    description: IMU底层串口收发代码
'''
send_data = []
# imu接收数据类型
class IMU():
    def __init__(self,Usart_port) -> None:
        # 串口初始化
        self.IMU_Usart = serial.Serial(
            port = Usart_port,          # 串口
            baudrate=115200,            # 波特率
            timeout = 0.001             # 由于后续使用read_all按照一个timeout周期时间读取数据
                                        # imu在波特率115200返回数据时间大概是1ms,9600下大概是10ms
                                        # 所以读取时间设置0.001s
        )
        # 接收数据初始化
        self.ACC_X = 0                  # X轴加速度
        self.ACC_Y = 0                  # Y轴加速度
        self.ACC_Z = 0                  # Z轴加速度
        self.GYRO_X = 0                 # X轴陀螺仪
        self.GYRO_Y = 0                 # Y轴陀螺仪
        self.GYRO_Z = 0                 # Z轴陀螺仪
        self.roll = 0                   # 横滚角    
        self.pitch = 0                  # 俯仰角
        self.yaw = 0                    # 航向角
        self.leve = 0                   # 磁场校准精度
        self.temp = 0                   # 器件温度
        self.MAG_X = 0                  # 磁场X轴
        self.MAG_Y = 0                  # 磁场Y轴
        self.MAG_Z = 0                  # 磁场Z轴
        self.Q0 = 0                     # 四元数Q0
        self.Q1 = 0                     # 四元数Q1
        self.Q2 = 0                     # 四元数Q2
        self.Q3 = 0                     # 四元数Q3

    def Send_ReadCommand(self):
        '''
        Author: Liu Yuxiang 
        Time: 2022.12.13
        description: 发送读取IMU内部数据指令
        · 第一个寄存器0x08 最后一个读取寄存器0x2A 共35个
        · 读寄存器例子,读取模块内部温度,主站发送帧为:A4 03 1B 02 C4
            |   A4    |    03    |    1B   |     02    |    C4
            |  帧头ID  | 读功能码 |起始寄存器| 寄存器数量 |校验和低 8 位
        '''
        # 使用优雅的方式发送串口数据
        send_data = [0xA4,0x03,0x08,0x23,0xD2]                      #需要发送的串口包
        send_data=struct.pack("%dB"%(len(send_data)),*send_data)    #解析成16进制
        self.IMU_Usart.write(send_data)                             #发送

    def Read_data(self):
        '''
        Author: Liu Yuxiang 
        Time: 2022.12.13
        description: 读取IMU数据
        '''
        # 初始化数据
        counter = 0 
        Recv_flag = 0
        Read_buffer = []
        # 接收数据至缓存区
        Read_buffer=self.IMU_Usart.read_all()
        # 状态机判断收包数据是否准确
        while(1):
            # 第1帧是否是帧头ID 0xA4
            if (counter == 0):
                if(Read_buffer[0] != 0xA4):
                    break    

            # 第2帧是否是读功能码 0x03  
            elif (counter == 1):
                if(Read_buffer[1] != 0x03):
                    counter=0
                    break

            # 第3帧判断起始帧        
            elif (counter == 2):
                if(Read_buffer[2] < 0x2c):
                    start_reg=Read_buffer[2]
                else:
                    counter=0 

            # 第4帧判断帧有多少数量 
            elif (counter == 3):
                if((start_reg+Read_buffer[3]) < 0x2C): # 最大寄存器为2C 大于0x2C说明数据肯定错了
                    len=Read_buffer[3]
                else:
                    counter=0
                    break                  
                 
            else:
                if(len+5==counter):
                    Recv_flag=1

            # 接包完毕
            if(Recv_flag):
                Recv_flag = 0
                sum = 0
                for i in range(0,counter):         
                    sum += Read_buffer[i]
                counter=0
                if(sum == Read_buffer[i]):          # 判断帧尾是否符合要求
                    data = str((binascii.b2a_hex(Read_buffer)))
                    self.ACC_X = data[4:6]                      
                    self.ACC_Y = data[6:8]                   
                    self.ACC_Z = data[8:10]                  
                    self.GYRO_X =data[10:12]                 
                    self.GYRO_Y =data[12:14]                 
                    self.GYRO_Z =data[14:16]                 
                    self.roll =  float(data[16:18]/100)                   
                    self.pitch = float(data[18:20]/100)               
                    self.yaw =   float(data[20:22]/100)               
                    self.leve =  data[22:23]                 
                    self.temp =  data[23:25]/100               
                    self.MAG_X = data[25:27]               
                    self.MAG_Y = data[27:29]               
                    self.MAG_Z = data[29:31]               
                    self.Q0 =    float(data[31:33]/10000)                 
                    self.Q1 =    float(data[33:35]/10000)                 
                    self.Q2 =    float(data[35:37]/10000)                 
                    self.Q3 =    float(data[37:39]/10000)                 

            else:
                counter += 1                        # 遍历整个接收数据的buffer

if __name__ == '__main__':
    # 变量初始化---------------------------------------------
    imu = IMU()    
    every_time = time.strftime('%Y-%m-%d %H:%M:%S')# 时间戳
    # 判断串口是否打开成功
    if imu.IMU_Usart.isOpen():
       print("open success")
    else:
        print("open failed")
    # ------------------------------------------------------

    # 发送读取指令-------------------------------------------
    imu.Send_ReadCommand()
    #-------------------------------------------------------

    # 循环读取IMU的内部数据-----------------------------------
    try:
        while True:
            count = imu.IMU_Usart.inWaiting()
            if count > 0:
                imu.Read_data()

    except KeyboardInterrupt:
        if serial != None:
            imu.IMU_Usart.close()
    
    #--------------------------------------------------------


'''
░░░░░░░░░░░░░░░░░░░░░░░░▄░░
░░░░░░░░░▐█░░░░░░░░░░░▄▀▒▌░
░░░░░░░░▐▀▒█░░░░░░░░▄▀▒▒▒▐
░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐
░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐
░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌
░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒
░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐
░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄
░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒
▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒
狗狗保佑代码无bug！
'''