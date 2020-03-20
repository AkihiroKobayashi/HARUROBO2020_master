#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

// スイッチやLEDのピン設定
#define PIN_DIP1 25
#define PIN_DIP2 24
#define PIN_DIP3 69
#define PIN_DIP4 70

#define PIN_SW_UP    32
#define PIN_SW_LEFT  33
#define PIN_SW_RIGHT 31
#define PIN_SW_DOWN  30

#define PIN_SW_WHITE  29
#define PIN_SW_YELLOW 28

#define PIN_ENC_A  26
#define PIN_ENC_B  27

#define PIN_LED_1 20
#define PIN_LED_2 36
#define PIN_LED_3 37
#define PIN_LED_4 38
#define PIN_LED_ENC 40

// Lernardo からのコントローラ用データのマスクデータ
#define BUTTON_X  0x0001
#define BUTTON_Y  0x0002
#define BUTTON_A  0x0004
#define BUTTON_B  0x0008

#define BUTTON_L1     0x0010
#define BUTTON_R1     0x0020
#define BUTTON_L2     0x0040
#define BUTTON_R2     0x0080

#define BUTTON_JOY_L   0x0100
#define BUTTON_JOY_R   0x0200
#define BUTTON_BACK    0x0400
#define BUTTON_START   0x0800

#define BUTTON_UP     0x1000
#define BUTTON_RIGHT  0x2000
#define BUTTON_DOWN   0x4000
#define BUTTON_LEFT   0x8000

//#define PIN_CTRL    ( A1 )
//#define PIN_XY      (  )

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )


// VL53L0X
#define SENSOR_NUM  4 // 使用するセンサーの数
#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)

// 自己位置推定用エンコーダ関連
#define _2PI_RES4   ( 2 * 3.141592 / 800 ) // res = 200
#define RADIUS_X    ( 0.020 )
#define RADIUS_Y    ( 0.020 )

#define DRIVE_MECHANUM      ( 0 )
#define DRIVE_OMNI4WHEEL    ( 1 )
#define DRIVE_OMNI3WHEEL    ( 2 )
#define DRIVE_DUALWHEEL     ( 3 )

#define DRIVE_MODE  ( DRIVE_OMNI3WHEEL )

#if DRIVE_MODE == DRIVE_DUALWHEEL
    // 双輪キャスター関連
    #define PIN_CSB     ( 10 )    // turntableのPIN(CSB)
    #define RADIUS_R    ( 0.04 )    // wheel radius
    #define RADIUS_L    ( 0.04 )    // wheel radius
    #define W           ( 0.265 )    // tread
    #define GEARRATIO   ( 5.5 )
    #define TT_RES4     ( 4096 )    // turntableの分解能
    #define _2RES_PI    ( 2 * 2048 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数  
    #define _2RES_PI_T  ( 2 * 500 / 3.141592 ) //  ターンテーブルの角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数
#elif DRIVE_MODE == DRIVE_MECHANUM
    // メカナム関連
    #define MECANUM_RES			( 500 )
    #define MECANUM_HANKEI		( 0.05 )
    #define MECANUM_HANKEI_D	( 0.15561 )
    #define MECANUM_HANKEI_L	( 0.26023 )
#elif DRIVE_MODE == DRIVE_OMNI3WHEEL
    #define _2RES_PI    ( 2 * 3 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数  
    #define WHEEL_R		( 0.019 )
    #define DIST2WHEEL  ( 0.120 )
    #define GEARRATIO   ( 51.45 )
    #define COS_PI_6    ( 0.86602540378 )
    #define SIN_PI_6    ( 0.5 )
#endif

// RoboClaw関連
#define ADR_MD1             ( 128 )
#define ADR_MD2             ( 129 )

#endif
