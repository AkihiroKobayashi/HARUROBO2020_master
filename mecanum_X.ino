//関東春ロボコン2020のマスター兼足回りのプログラム
//作成者 こばやしあきひろ
//備考 ここにあるクラスは先生が作成された物を活用させて頂いています。
//git remote add origin <git@github.com:AkihiroKobayashi/HARUROBO2020_master.git>
#include "Arduino.h"
#include "MsTimer2.h"
#include "define.h"
#include "Button.h"
#include "ManualControl.h"
#include "RoboClaw.h"
#include "PIDclass.h"
#include "math.h"
#include "lpms_me1Peach.h"
#include "phaseCounterPeach.h"

#define INT_TIME 0.01           //制御周期
#define SERIAL_ROBOCLAW Serial4 //Roboclawとの通信
#define SERIAL_LEONARDO Serial5 //pro microとの通信
#define SERIAL_LPC Serial0      //上半身LPCとの通信
#define LPMS_ME1 Serial1        //ジャイロセンサとの通信
#define PI 3.14159264
#define Z_A 5       //ギア比
#define Z_B 6       //ギア比
#define Enc_RES 800 //足回りのエンコーダの分解能
#define a 0.25
#define b 0.20
#define DEG_INCREMENT 0.02 //角度の最大加算値
#define TWEAK 0.4

PID degPID(4.0, 0.0, 0.0, INT_TIME);
lpms_me1 lpms(&Serial1);
RoboClaw MD(&SERIAL_ROBOCLAW, 1);
ManualControl Controller;
phaseCounter enc1(1);
phaseCounter enc2(2);
/*********************************グローバル変数の設定****************************************/
unsigned int ButtonState = 0, LJoyX = 127, LJoyY = 127, RJoyX = 127, RJoyY = 127; //コントローラデータ
int send_data = 0;                                                                //通信のデータの格納
int countL1 = 0;
double coe; //速度係数
double angle_z, ref_ang;
double tweakM1, tweakM2, tweakM3, tweakM4; //微調整の変数
bool flag_10ms = false;                    // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;
bool flag_meters = false; //一定距離進んだ判定のフラグ
double refVel;
//coords refV = {0.0, 0.0, 0.0};
int state_count = 0;
// グローバル変数の設定
double gPosix = 0.0, gPosiy = 0.0, gPosiz = 0.0;
//double refVx, refVy, refVz;
double angle_rad;
int encX = 0, encY = 0;       // X,Y軸エンコーダのカウント値
int preEncX = 0, preEncY = 0; // X,Y軸エンコーダの"1サンプル前の"カウント値
double Enc_X, Enc_Y, CORR;    //PhaseCountorを使う必要あり
double sokudo_houkou = -1;    //方向切り替えの係数（今使ってない）
double speed_UP;              //倍速にする為の速度係数
double GVx = 0;
double GVy = 0;
double LVx;
double LVy; //グローバルとローカルの速度変数
/****************************LEDをチカチカさせるための関数************************************/
void LEDblink(byte pin, int times, int interval)
{
  analogWrite(pin, 0);
  for (int i = 0; i < times; i++)
  {
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

/***********************コントローラデータを取得する部分***************************************/
void controller_receive()
{
  static int recv_num = 0;
  static int checksum = 0;
  static char recv_msgs[9];
  char c;
  while (SERIAL_LEONARDO.available())
  {
    c = SERIAL_LEONARDO.read();
    if (c == '\n')
    {
      if (recv_num == 9)
      { // && (checksum & 0x3F == recv_msgs[recv_num-1] - 0x20)){
        ButtonState = 0, LJoyX = 0, LJoyY = 0, RJoyX = 0, RJoyY = 0;
        ButtonState |= recv_msgs[0] - 0x20;
        ButtonState |= (recv_msgs[1] - 0x20) << 6;
        ButtonState |= (recv_msgs[2] - 0x20) << 12;

        LJoyX |= (recv_msgs[3] - 0x20);
        LJoyX |= ((recv_msgs[4] - 0x20) & 0x03) << 6;

        LJoyY |= ((recv_msgs[4] - 0x20) & 0x3C) >> 2;
        LJoyY |= ((recv_msgs[5] - 0x20) & 0x0F) << 4;

        RJoyX |= ((recv_msgs[5] - 0x20) & 0x30) >> 4;
        RJoyX |= ((recv_msgs[6] - 0x20) & 0x3F) << 2;

        RJoyY |= (recv_msgs[7] - 0x20);
        RJoyY |= ((recv_msgs[8] - 0x20) & 0x03) << 6;
      }
      recv_num = 0;
    }
    else
    {
      recv_msgs[recv_num] = c;
      recv_num++;
    }
  }
}

/**********************************************割り込み処理************************************/
void timer_warikomi()
{
  // LEDの処理
  digitalWrite(PIN_LED_1, HIGH);
  digitalWrite(PIN_LED_2, HIGH);
  digitalWrite(PIN_LED_3, HIGH);
  digitalWrite(PIN_LED_4, HIGH);
  static int count = 0;
  static int count_flag = 0;
  count += 4; // ここで光る周期を変えられる(はず)
  count_flag++;

  if (count < 255)
  {
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }
  else if (count < 255 * 2)
  {
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }
  else if (count < 255 * 3)
  {
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }
  else
  {
    count = 0;
  }

  /*******************************微調整の低速走行モード***************************************/
  //今は使ってない
  /********************************倍速モードの処理********************************************/
  if (ButtonState & BUTTON_L2)
  {
    speed_UP = 0.5;
  }
  else if (ButtonState & BUTTON_R2)
  {
    speed_UP = 1.5;
  }
  else
  {
    speed_UP = 1;
  }
  /**************************************機体角度のPID*********************************************/
  angle_z = (double)lpms.get_z_angle() / 0.996;
  if (RJoyY > 132)
  {
    ref_ang += -((double)RJoyY - 127) * DEG_INCREMENT / 127;
  }
  else if (RJoyY < 122)
  {
    ref_ang += -((double)RJoyY - 127) * DEG_INCREMENT / 127;
  }
  else
  {
    ref_ang += 0;
  }
  refVel = degPID.getCmd(ref_ang, angle_z, 4.71);
  /*****************************ジョイスティックをフィールドに固定**********************************/
  coords refV = Controller.getVel(LJoyX, LJoyY, RJoyY);
  GVx = refV.x;
  GVy = refV.y;
  LVx = GVx * cos(angle_z) + GVy * sin(angle_z);
  LVy = GVy * cos(angle_z) - GVx * sin(angle_z);
  /*********************************自己位置推定の処理**********************************************/
  // 自己位置推定用エンコーダのカウント値取得
  encY = -enc2.getCount();
  encX = enc1.getCount();
  // エンコーダのカウント値から角度の変化量を計算する
  double angX, angY;
  angX = (double)(encX - preEncX) * 2 * PI / 800;
  angY = (double)(encY - preEncY) * 2 * PI / 800;

  angle_rad = angle_z;
  // ローカル座標系での変化量を計算(zは角度)
  double Posix, Posiy, Posiz;
  static double pre_angle_rad = angle_rad, angle_diff;
  angle_diff = angle_rad - pre_angle_rad; // 角度の変化量を計算
  Posiz = angle_diff;
  Posix = 0.019 * angX;
  Posiy = 0.018 * angY;

  gPosiz += Posiz;
  gPosix += Posix * cos(gPosiz) - Posiy * sin(gPosiz);
  gPosiy += Posix * sin(gPosiz) + Posiy * cos(gPosiz);

  // 1サンプル前のデータとして今回取得したデータを格納
  preEncX = encX;
  preEncY = encY;
  pre_angle_rad = angle_rad;

  /***************************LPCへの通信処理生成*************************************/
  send_data = 0;        // 送信データの初期化
  controller_receive(); //コントローラ情報を取得
  if (gPosix >= 4.0)    //自己位置から上半身に送る情報を設定
  {
    flag_meters = true;
  }
  else
  {
    flag_meters = false;
  }

  if (ButtonState & BUTTON_Y) //移住船リセット
  {
    send_data |= 0x06;
  }
  else if (ButtonState & BUTTON_R1 || digitalRead(31) == 0) //実験用にオンボードのスイッチからでも動くようにしている
  {
    send_data |= 0x01; //移住船ハンド上昇
  }
  else if (ButtonState & BUTTON_L1 || digitalRead(32) == 0)
  {
    send_data |= 0x02; //移住船ハンド下降
  }
  else if (ButtonState & BUTTON_RIGHT)
  {
    send_data |= 0x03;
  }
  else if (ButtonState & BUTTON_LEFT)
  {
    send_data |= 0x04;
  }

  if (ButtonState & BUTTON_A)
  {
    //digitalWrite(PIN_LED_1, HIGH);
    send_data |= 0x07; //物資リセット
    state_count = 0;
  }
  else if (ButtonState & BUTTON_X)
  {
    send_data |= 0x05; //物資回収
    state_count++;
  }
  else if (ButtonState & BUTTON_B)
  {
    send_data |= 0x08; //物資を入れる
    state_count = 0;
  }
  else if (flag_meters && state_count != 0)
  {
    send_data |= 0x09;
  }

  //Serial.println(send_data,HEX);//16進数で送信
  SERIAL_LPC.println(send_data, HEX);
  /******************************************************************************************/
  flag_10ms = true;
  if (count_flag >= 100)
  {
    flag_100ms = true;
    count_flag = 0;
  }
}

/*****************************************************************************************/
void setup()
{
  bool ready_to_start = false;
  Serial.begin(115200);
  SERIAL_ROBOCLAW.begin(115200);
  SERIAL_LEONARDO.begin(115200);
  SERIAL_LPC.begin(115200);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(28, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  Serial.println("power on");
  LEDblink(PIN_LED_GREEN, 2, 100);
  LEDblink(PIN_LED_RED, 2, 100);
  Serial.print("Setting Finished...");
  lpms.init();
  LEDblink(PIN_LED_BLUE, 2, 100);
     // 初期が終わった証拠にブリンク
  Serial.println("LPMS-ME1 init done!");
  // digitalWrite(PIN_LED_1,HIGH);
  while (!ready_to_start)
  {
    controller_receive();
    if (ButtonState & BUTTON_START || digitalRead(28) != 1)
    {
      ready_to_start = true;
    }
  }
  delay(750);
  enc1.init();
  enc2.init();
  degPID.PIDinit(0.0, 0.0);
  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  controller_receive(); // コントローラからの受信

  // 10msに1回ピン情報を出力する
  if (flag_10ms)
  {
    double refOmegaM1, refOmegaM2, refOmegaM3, refOmegaM4;
    //速度[m/s]から角速度[rad/s]に変換
    refOmegaM1 = (LVx - LVy - (a + b) * refVel) / 0.076 * Z_A / Z_B;
    refOmegaM2 = (LVx + LVy - (a + b) * refVel) / 0.076 * Z_A / Z_B;
    refOmegaM3 = (-LVx - LVy - (a + b) * refVel) / 0.076 * Z_A / Z_B;
    refOmegaM4 = (-LVx + LVy - (a + b) * refVel) / 0.076 * Z_A / Z_B;
    // RoboClawの指令値に変換
    double mdCmdM1, mdCmdM2, mdCmdM3, mdCmdM4;
    mdCmdM1 = refOmegaM1 * Enc_RES * 4 / (2 * PI) * speed_UP;
    mdCmdM2 = refOmegaM2 * Enc_RES * 4 / (2 * PI) * speed_UP;
    mdCmdM3 = refOmegaM3 * Enc_RES * 4 / (2 * PI) * speed_UP;
    mdCmdM4 = refOmegaM4 * Enc_RES * 4 / (2 * PI) * speed_UP;

    // モータにcmdを送り，回す
    MD.SpeedM1(129, (int)mdCmdM1); // 右前
    MD.SpeedM2(129, (int)mdCmdM2); // 左前
    MD.SpeedM1(130, (int)mdCmdM3); // 右後
    MD.SpeedM2(130, (int)mdCmdM4); // 左後
    flag_10ms = false;
  }
}

/*
エラーコードを貼る欄
*/