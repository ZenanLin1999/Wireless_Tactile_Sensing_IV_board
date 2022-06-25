//TBSI-SSR LinZenan
//修改于2021年10月26日，此程序供tactile sensing后期压力解耦项目采集数据使用
//2021年10月6日
#include "BluetoothSerial.h"

#include<Wire.h>
//IIC设备
//#define SDA 4
//#define SCL 15
#define SDA 14
#define SCL 13
#define test_mode 1//0--直接串口查看数据 1--串口上位机查看数据


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

//定时器中断
#define SET_US 100 //设置定时器中断时间
hw_timer_t *timer=NULL;//创建一个定时器结构体
volatile SemaphoreHandle_t timerSemaphore;//创建一个定时器信号量
int time_count=0;//用于设置标志位
int time_status_0_5s = 0; //0.5s标志位
int time_status_0_2ms = 0; //0.1ms标志位
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//bluetooth
BluetoothSerial SerialBT;

//led-test
const int output02 = 5;
static bool led_status = true;

// adc port*4
const int potPin_01 = 32;
const int potPin_02 = 33;
const int potPin_03 = 34;
const int potPin_04 = 35;
const int potPin_05 = 36;
const int potPin_06 = 37;
const int potPin_07 = 38;
const int potPin_08 = 39;

// some parameters 
unsigned char DataToSend[30];//send to ANOV7
int data[8];//raw data
int data_mpu[10];//raw data

//mpu9250器件地址
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//mode
int send_mode = 1;


//timer中断
void IRAM_ATTR Timer_Hander()
{
  portENTER_CRITICAL_ISR(&timerMux);//进入临界段
  time_count++;
  if(time_count%80==1)
  {
    time_status_0_2ms = 1; //0.2ms标志位，实际是100us*50 = 5ms标志位，因为发现手机蓝牙app波形发太快会卡住，所以调慢了（是app问题，因为数据已经发出去了）
  }
  if(time_count == 5000)
  {
    time_status_0_5s = 1; //0.5s标志位
    time_count = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);//退出临界段
  xSemaphoreGiveFromISR(timerSemaphore, NULL);//释放一个二值信号量timerSemaphore
}

void setup() {
  Wire.begin(SDA, SCL);
  //设置acc量程±4G
  /*
   *0x1c          0x00      ±2G
    0x1c          0x08      ±4G
    0x1c          0x10      ±8G
    0x1c          0x18      ±16G
   */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // PWR_MGMT_1 register
  Wire.write(0x08); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  //设置gyro量程±1000°/s
  /*
   *  0x1b          0x00      ±250°/s
      0x1b          0x08      ±500°/s
      0x1b          0x10      ±1000°/s
      0x1b          0x18      ±2000°/s
   */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); // PWR_MGMT_1 register
  Wire.write(0x10); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  //开启MPU9250设备
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(115200);
  SerialBT.begin("ESP32-TBSI-SSR"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

 //配置定时器中断
  timerSemaphore=xSemaphoreCreateBinary();//定义信号量
  timer = timerBegin(0, 80, true);//初始化定时器0 80分频  向上计数
  // 配置定时器中断函数
  timerAttachInterrupt(timer, &Timer_Hander, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, SET_US, true);//每计数100次触发定时器中断 自动重载开启。单位us
  // Start an alarm
  timerAlarmEnable(timer);//使能定时器函数

  pinMode(output02, OUTPUT);
  delay(1000);
}

void loop() {
  //简单任务调度
    if(time_status_0_2ms == 1)
    {
      ADC_read(8 ,data);
      read_raw_data();
      ANOv7_TENG_DATA_short_Send(data);
      ANOv7_TENG_DATA_short_Send2(data);
//      mini_balance_bluetooth_app(4,data);
      time_status_0_2ms = 0;
    }
    if(time_status_0_5s == 1)
    {
      led_status = !led_status;
      if(led_status==true)
        digitalWrite(output02, LOW);
      else
        digitalWrite(output02, HIGH);
      time_status_0_5s = 0;
    }
}

//电压采集
void ADC_read(int num ,int data[])
{
  for(int i = 0;i<num;i++)
  { 
    data[i] = analogRead(potPin_01+i);  
//    delay(10);//10ms
  }
}

//读取一次原始数据
void read_raw_data(void)
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)  

  data_mpu[0] = AcX/10;
  data_mpu[1] = AcY/10;
  data_mpu[2] = AcZ/10;
  data_mpu[3] = GyX;
  data_mpu[4] = GyY;
  data_mpu[5] = GyZ;
  data_mpu[6] = 100*((Tmp-21)/333.87 +21);
}

//mini-balance手机蓝牙上位机（最多支持5路)
//波形协议: {B%d:%d:%d:%d:%d},a,b,c,d,e
//num: 发送通道数
//data: 发送的数据
int mini_balance_bluetooth_app(int num,int data[])
{
  if(num <= 0 || num > 5)
    return 0;
  else
  {
    SerialBT.print("{B");
    for(int i = 0;i<num ; i++)
    {
      if(i!=0)
        SerialBT.print(":");
      SerialBT.print(data[i]);
        
    }
    SerialBT.print("}$");
    return 1;
  }
}

//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF1+NUM(byte)+DATA(低位先发)+SC+AC 8个通道的ADC值
void ANOv7_TENG_DATA_short_Send(int data[])
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值

  /********************* 协议帧 发送过程（0xF1）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0xF1;
  DataToSend[_cnt_send++] = 8*2;//short发送占两个字节
  //数据
  for(_cnt_data = 0;_cnt_data < 8;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(data[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(data[_cnt_data]);
  }
  //双校验
  for(unsigned char i = 0;i < (8*2+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0xF1）  *********************/
}

//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF2+NUM(byte)+DATA(低位先发)+SC+AC 10个通道的值（accX,accY,accZ,gyroX,gyroY,gyroZ,temp,time0,time1）
void ANOv7_TENG_DATA_short_Send2(int data[])
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值
  unsigned long time; 
  /********************* 协议帧 发送过程（0xF2）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0xF2;
  DataToSend[_cnt_send++] = 9*2;//short发送占两个字节
  //数据
  for(_cnt_data = 0;_cnt_data < 7;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(data_mpu[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(data_mpu[_cnt_data]);
  }
  //加入时间刻度
  time = millis();
  DataToSend[_cnt_send++] = BYTE0(time);
  DataToSend[_cnt_send++] = BYTE1(time);
  DataToSend[_cnt_send++] = BYTE2(time);
  DataToSend[_cnt_send++] = BYTE3(time);
  
  //双校验
  for(unsigned char i = 0;i < (9*2+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0xF2）  *********************/
}

