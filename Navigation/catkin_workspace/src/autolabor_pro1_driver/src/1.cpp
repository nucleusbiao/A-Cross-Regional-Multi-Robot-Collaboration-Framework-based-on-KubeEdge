#include <ros.h>                   ////ros 库
#include <PS2X_lib.h>              ////手柄库
#include <geometry_msgs/Twist.h>   ////ros Twist格式消息库

#define L 0.8 //定义车长，单位m
#define pi 3.1415  //定义圆周率

///////设备远端控制信号输入口/////////
#define modeInPin 52
#define powerInPin 50
#define steelInPin 9

///////控制信号初始设置//////////
int mode1 = 1;  //初始状态，前进
int power = 0;  //初始供能，无
int steel = 532; //初始方向盘角度

///////手柄设置//////
#define PS2_DAT        13
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        12

#define pressures   true
//#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x;

int error = 0;
byte type = 0;
byte vibrate = 100;

//////////速度测量相关设置///////
#define sample_time 100        //速度计算采样时间
#define item_number 80         //码盘遮挡条纹数量

//////////速度测量准备量//////////
unsigned long int c1 = 0;     //前一次采样下降沿的个数
unsigned long int c2 = 0;     //本次采样下降沿的个数
double c = 0.0;               //采样时间内下降沿的个数
double w_measurement = 0.0;                //转速测量值
double v_measurement = 0.0;                //实际速度测量值

//////////上一次采样时间和本次采样时间//////////
unsigned long int now = 0;
unsigned long int before = 0;

//////////pid参数设置///////
double kp = 0.15;
double ki = 0.1;                         //最小不低于0.06 最大不超过0.2
double kd = 0.0;

//////////速度pid采样参数设置///////
double last_delta_v = 0.0;
double delta_v = 0.0;
double i_delta_v = 0.0;
double d_delta_v = 0.0;

//////////角度参数设置///////
double angle;              ///-24度（左）~24度（右）

//////////PWM输出控制端口//////////
const int pwm1 = 2;     //后轮电机的PWM控制信号输出管脚
const int pwm2 = 4;     //前轮电机的PWM控制信号输出管脚

//////////电机方向控制端口//////////
const int ina1 = 22;    //后轮电机的正转反转逻辑控制端口1
const int inb1 = 24;    //后轮电机的正转反转逻辑控制端口2
const int ina2 = 30;    //前轮电机的正转反转逻辑控制端口1
const int inb2 = 32;    //前轮电机的正转反转逻辑控制端口2
const int speedinPin = 21; //后轮速度信号读取,21号口永远的神，中断信号终于能给准确了！！！
const int angleInPin = A0; //前轮角度读取输入信号控制端口

//////////////状态标志//////////////
bool ros_serial_flag=false;
bool remote_control_flag = false;  //后轮速度信号读取,21号口永远的神，中断信号终于能给准确了！！！后轮速度信号读取,21号口永远的神，中断信号终于能给准确了！！！
bool ps2_config_again_flag = true;

//////////PWM占空比参数//////////
int val1 = 0;                     //后轮PWM占空比调节0-255
int val2 = 0;                     //前轮PWM占空比调节0-255

///////////////导航控制信号/////////////
double ctrl_v=0;                //控制速度
double ctrl_a=0;                //控制角度

ros::NodeHandle nh;               //ros 节点句柄
geometry_msgs::Twist vel;         //ros Twist格式文件对象

//////////前进后退控制//////////
void back_direction_ctrl(int a1, int b1)  {
  digitalWrite(ina1, a1);
  digitalWrite(inb1, b1);
}

//////////前进//////////
void go_ahead() {
  back_direction_ctrl(0, 1);
  analogWrite(pwm1, val1);
}

//////////后退//////////
void go_back() {
  back_direction_ctrl(1, 0);
  analogWrite(pwm1, val1);
}

//////////紧急制动//////////
void brake() {
  digitalWrite(ina1, LOW);
  digitalWrite(inb1, LOW);
  digitalWrite(ina2, LOW);
  digitalWrite(inb2, LOW);
}

//////////后轮中断计数/////////
void count() {
  c2 = c2 + 1;
  //Serial.println(c2);
}

//////////前轮左右控制//////////
void front_direction_ctrl(int a2, int b2)  {
  digitalWrite(ina2, a2);
  digitalWrite(inb2, b2);
}

//////////前轮左转//////////
void turn_left() {
  front_direction_ctrl(1, 0);
  analogWrite(pwm2, val2);
  //angle = analogRead(angleInPin);
  //Serial.print(angle);
  //Serial.print('\t');
  if (angle < 308) {
    analogWrite(pwm2, 0);
  }
}

//////////前轮右转//////////
void turn_right() {
  front_direction_ctrl(0, 1);
  analogWrite(pwm2, val2);
  //angle = analogRead(angleInPin);
  //Serial.print(angle);
  //Serial.print('\t');
  if (angle > 715) {
    analogWrite(pwm2, 0);
  }
}


//////////前轮转向角度读取/////////
void angle_read() {
  angle = analogRead(angleInPin);
  //Serial.println(angle);
}

//////////速度信号采样，计算和PID控制////////////
void speed_pid(double v_setpoint) {
  now = millis();
  if (now - before >= sample_time) {
    c = c2 - c1;
    //Serial.println(c);
    w_measurement = (c / item_number) / (sample_time / 1000.0);
    v_measurement = (w_measurement * 3.14 * 2 * 120) ;
    Serial.print(v_setpoint);
    Serial.print('\t');
    Serial.println(v_measurement);
    delta_v = v_setpoint - v_measurement;          //误差
    i_delta_v += (delta_v * (sample_time / 1000.0)); //积分
    d_delta_v = (delta_v - last_delta_v) / (sample_time / 1000.0); //微分
    if (abs(delta_v) >= 100) {
      val1 = kp * delta_v + ki * i_delta_v + kd * d_delta_v;
    }
    if (val1 > 200) {
      val1 = 200;
    }
    if (val1 < 0) {
      val1 = 0;
    }
    //Serial.println(val1);
    //Serial.print(c);
    //Serial.print('\t');
    //Serial.print(w_measurement);                 //转速输出
    //Serial.print('\t');
    //Serial.println(v_measurement); //实际速度输出
    //Serial.print('\n');
    last_delta_v = delta_v;
    c1 = c2;
    before = now;
  }
}

///////////角度调节//////////////
void angle_adjust(double angle_sp) {
  angle_sp = 7.76667 * angle_sp + 532 ;       //角度转换为0-1023
  //Serial.println(angle_sp);
  int temp = int(angle_sp) + 1;                    //转换值取整+1
  if (abs(angle_sp - temp) > 0.5) {
    angle_sp = int(angle_sp);
  }
  else {
    angle_sp = int(angle_sp) + 1;
  }
  //Serial.println(angle_sp);
  //Serial.println(angle);
  if (abs(angle - angle_sp) < 12) {
    analogWrite(pwm2, 0);
  }
  if ((angle - angle_sp) >= 12) {
    turn_left();
  }
  if ((angle_sp - angle) >= 12) {
    turn_right();
  }
}

/////////////ps2手柄控制小车////////////
void ps2_control() {
  //Serial.println("test");
  if (ps2_config_again_flag == true)
  {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    delay(300);
  }
  if (error == 0) {
    Serial.println("Found Controller, configured successful !");
    ps2_config_again_flag == false;
  }
  else if (error == 1)
    {//Serial.println("No controller found !");
     return;
    }
  else if (error == 2)
    {//Serial.println("Controller found but not accepting commands !");
     return;
    }
  else if (error == 3)
    {//Serial.println("Controller refusing to enter Pressures mode ! ");
    }
  //  Serial.print(ps2x.Analog(1), HEX);
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found !");
      break;
    case 1:
      {
        Serial.println("DualShock Controller found !");
        ps2_config_again_flag = false;
      }
      break;
    case 2:
      Serial.print("GuitarHero Controller found !");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found !");
      break;
  }
  if (type == 1) { //DualShock Controller
    ps2_config_again_flag=false;
    ps2x.read_gamepad(false, vibrate);
    if (ps2x.Button(PSB_START))
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    if (ps2x.Button(PSB_PAD_UP)) {
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }
    vibrate = ps2x.Analog(PSAB_CROSS);
    if (ps2x.ButtonPressed(PSB_CIRCLE))
    {
      Serial.println("Circle just pressed");
      ros_serial_flag=true;
      val1=0;
      val2=0;
      before=millis();
      attachInterrupt(2, count, FALLING);
    }
    if (ps2x.ButtonReleased(PSB_SQUARE))
    {
      Serial.println("Square just released");
      val1 = 160;
      val2 = 80;
    }
    if (ps2x.NewButtonState(PSB_CROSS))
    {
      Serial.println("X just changed");
      val1 = 120;
      val2 = 80;
    }
    if (ps2x.ButtonPressed(PSB_TRIANGLE))
    {
      Serial.println("Triangle pressed");
      remote_control_flag = true;
      val1 = 0;
      val2 = 0;
    }
    Serial.print(ps2x.Analog(PSS_LY));
    Serial.print(',');
    Serial.print(ps2x.Analog(PSS_LX));
    Serial.print(',');
    Serial.print(ps2x.Analog(PSS_RY));
    Serial.print(',');
    Serial.print(ps2x.Analog(PSS_RX));
    Serial.print('\n');
    if (abs(ps2x.Analog(PSS_LY) - 127) < 50) {
      analogWrite(pwm1, 0);
    }
    if (abs(ps2x.Analog(PSS_RX) - 128) < 50) {
      angle_adjust(0);
    }
    if ( ps2x.Analog(PSS_LY) <= 77) {
      go_ahead();
    }
    if (ps2x.Analog(PSS_LY) >= 197) {
      go_back();
    }
    if (ps2x.Analog(PSS_RX) <= 78) {
      turn_left();
    }
    if (ps2x.Analog(PSS_RX) >= 198) {
      turn_right();
    }
  }
}

//////远端控制信号控制逻辑处理////
void remote_control()
{
  ps2x.read_gamepad(false, vibrate);    //手柄和Arduino接收端继续保持联系
  //vibrate = ps2x.Analog(PSAB_CROSS);
  if (ps2x.ButtonPressed(PSB_TRIANGLE))
  {
    //Serial.println("Triangle pressed");
    remote_control_flag = false;
    ps2_config_again_flag = true;
  }
  steel = pulseIn(steelInPin, HIGH);
  //Serial.print("steel:");
  //Serial.println(steel);
  if (steel > 600)
  {
    val2 = 80;
    turn_right();
  }
  if (steel < 450)
  {
    val2 = 80;
    turn_left();
  }
  if (steel <= 550 && steel >= 500)
  {
    angle_adjust(0);   //自动回正功能
  }
  power = digitalRead(powerInPin);
  Serial.print("power:");
  Serial.println(power);
  if (power == 1)
  {
    val1 = 150;
    mode1 = digitalRead(modeInPin);
    Serial.print("mode:");
    Serial.println(mode1);
    if (mode1 == 0)
      go_back();
    if (mode1 == 1)
      go_ahead();
  }
  else
  {
    analogWrite(pwm1, 0);
  }
}

////////////////控制模式选择/////////////////
void control_mode_select()
{
  if (remote_control_flag == true&&ros_serial_flag==false)
  { remote_control();
    //Serial.println("ko");
  }
  else if(ros_serial_flag==true&&remote_control_flag == false)
  {
    ros_serial();
  }
  else if(ros_serial_flag==false&&remote_control_flag == false)
  { ps2_control();
  }
}

////////////导航函数///////////
void ros_serial()
{
  speed_pid(abs(ctrl_v));
  if(ctrl_v>0)
  {
    go_ahead();
  }
  if(ctrl_v<0)
  {
    go_back();
  }
  val2=80;
  angle_adjust(ctrl_a);
  nh.spinOnce();
}

////////////从ros topic 接收消息后的回调函数/////////////
void messageCb(const geometry_msgs::Twist& vel) 
{  
    if(vel.linear.x==0&&vel.angular.z==0)
    {
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
    }
    ctrl_v=vel.linear.x*1000; 
    if(ctrl_v<-1000)
    {
      ctrl_v=-1000;
    }
    if(ctrl_v>1000)
    {
      ctrl_v=1000;
    }
    if(ctrl_v==0)
    {
      ctrl_a=0;
    }
    else
    {
      double temp= double(L*(1000.0*vel.angular.z))/abs(ctrl_v);
      if(temp>0.445228)
      {
        temp=0.445228;
      }
      if(temp<-0.445228)
      {
        temp=-0.445228;
      }
      ctrl_a=(-asin(temp)*180)/pi; 
    }  
} 

//订阅topic，设置回调函数
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb);

void setup() {
  Serial.begin(57600);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(ina1, OUTPUT);
  pinMode(inb1, OUTPUT);
  pinMode(ina2, OUTPUT);
  pinMode(inb2, OUTPUT);
  pinMode(ina2, OUTPUT);
  pinMode(inb2, OUTPUT);
  pinMode(modeInPin, INPUT);
  pinMode(powerInPin, INPUT);
  pinMode(steelInPin, INPUT);
  pinMode(speedinPin, INPUT);
  nh.initNode();   //节点初始化
  nh.subscribe(sub);  //订阅
}

void loop() {
  //double angle_sp = 0;              //单位：度
  //double v_sp = 1000;           //单位：毫米/秒
  //control();
  //speed_pid(v_sp);
  //Serial.print(v_sp);
  //Serial.print('\t');
  //Serial.println(v_measurement);
  //angle_adjust(angle_sp);
  //speed_read();
  //turn_left();
  //turn_right();
  //go_ahead();
  //go_back();
  //remote_control();
  //ps2_control();
  //remote_control();
  //delay(1000);
  angle_read();
  control_mode_select();
}

//rostopic pub -r 10 /cmd_vel geometry_msgs/Twist
//rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2 _baud:=57600