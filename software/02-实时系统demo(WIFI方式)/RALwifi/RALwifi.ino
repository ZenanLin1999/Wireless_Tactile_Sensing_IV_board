#include <WiFi.h>  //wifi功能需要的库 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>必须在MPU之后
WiFiUDP Udp;//声明UDP对象
const char* wifi_SSID="mate20pro";  //"1111C"存储AP的名称信息
const char* wifi_Password="cn1234567890";  //"ssrgroup"存储AP的密码信息
uint16_t udp_port=1122;  //存储需要监听的端口号
char incomingPacket[255];  //存储Udp客户端发过来的数据
char To_IP[] = "192.168.43.238";
int Port = 8090;
float accelX = 0;
int tim = 0;

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
int data[8];//raw data

union myCo{
  float datafloat;
  int dataint;
  uint8_t s[4];
  };
myCo myc;

//电压采集
void ADC_read(int num ,int data[]){
  for(int i = 0;i<num;i++){ 
    data[i] = analogRead(potPin_01+i);  
  }
}

[[noreturn]] void Task_Send(void *parameter){
  int num = 0;
  Udp.beginPacket(To_IP, Port);  //准备发送数据到目标IP和目标端口
  Udp.println(WiFi.localIP());
  Udp.endPacket();  //向目标IP目标端口发送数据
  delay(500);
  Udp.beginPacket(To_IP, Port);  //准备发送数据到目标IP和目标端口
  Udp.println(WiFi.localIP());
  Udp.endPacket();  //向目标IP目标端口发送数据
  delay(500);
  Udp.beginPacket(To_IP, Port);  //准备发送数据到目标IP和目标端口
  Udp.println(WiFi.localIP());
  Udp.endPacket();  //向目标IP目标端口发送数据
  delay(500);
    while(1){
        if(1){
           Udp.beginPacket(To_IP, Port);  //准备发送数据到目标IP和目标端口


           myc.dataint = millis();
           Udp.write(myc.s,4);

           ADC_read(8 ,data);
           
           myc.dataint = data[2];
           Udp.write(myc.s,4);
           
           myc.dataint = data[3];
           Udp.write(myc.s,4);

           myc.dataint = data[4];
           Udp.write(myc.s,4);

           myc.dataint = data[5];
           Udp.write(myc.s,4);
           
           Udp.println();  //将接收到的数据放入发送的缓冲区
           Udp.endPacket();  //向目标IP目标端口发送数据
        }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

[[noreturn]] void Task_Receive(void *parameter){
    while(1){
        if(Udp.parsePacket()>0){//如果有数据那么Data_length不为0，无数据Data_length为0
            int len = Udp.read(incomingPacket, 255);  //读取数据，将数据保存在数组incomingPacket中
            if (len > 0){  //为了避免获取的数据后面乱码做的判断
                Serial.println(incomingPacket);
                incomingPacket[len] = 0;
            }
        }
        vTaskDelay(10);
    }
    vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
      WiFi.begin(wifi_SSID, wifi_Password);
    while (WiFi.status() != WL_CONNECTED){
        delay(1000);
        Serial.println("...");
    }
    Serial.print("WiFi connected with IP: ");
    Serial.println(WiFi.localIP());
    Udp.begin(udp_port);//启动UDP监听这个端口
    Serial.println(xPortGetCoreID());
    xTaskCreate(Task_Send, "Task_Send", 10000, NULL, 1, NULL);
    xTaskCreate(Task_Receive, "Task_Receive", 10000, NULL, 1, NULL);
    delay(1000);
    pinMode(A0,INPUT);
    pinMode(output02, OUTPUT);
    delay(1000);
}

void loop(){
    delay(1000);
      led_status = !led_status;
      if(led_status==true)
        digitalWrite(output02, LOW);
      else
        digitalWrite(output02, HIGH);
}
