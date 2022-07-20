/* License
  SPDX-License-Identifier:MIT
  Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
*/
/* 参考サイト様リンク
  https://veresk.hatenablog.com/entry/2019/06/29/192525
  https://skpme.com/211/
*/   
/* 協力者様
　Yuya Mochizuki, GitHub: https://github.com/Mochizuki12
*/
/*
　ライントレースと速度の制御をおこなうプログラムです。
  マイコンは Arduino Mega に対応しています。
  このプログラムは、Arduino言語で書かれています。C言語とC++を元とし、Arduinoに対応した言語です。
　
　だた、速度制御が上手く機能していない可能性があります。
　・目標速度が0.3[m/s]であるのにもかかわらず、実測だと速度が約1[m/s]となっている。
*/

// ROS関係
#include <ros.h>
#include <intelligent_robot_b_2022/rosbagdata.h>
ros::NodeHandle nh;
intelligent_robot_b_2022::rosbagdata msg;
ros::Publisher linetrace("linetrace", &msg);

#define CPR 400  // エンコーダのCPR
#define Pi 3.141592  // 円周率
#define WHEEL 0.071  // 車輪直径
#define MAX_V 1.5  // 最大速度[m/s](未使用)

// 速度のPID制御のパラメータ
#define V_KP 1 
#define V_KI 0.001
#define V_KD 0

// ライントレースのPID制御のパラメータ
#define KP 0.3 
#define KI 0
#define KD 0.01

// ラインが十字かの判定
int Check = 1;

// 微小時間
float dt = 0.002;  // １週期: 0.002[s] = 2000[μs]

float Circumference = WHEEL * Pi;  // 円周
float Cir_CPR = Circumference / CPR;  // １パルス分の変位

volatile long L_encoder = 0;
volatile int L_old_value = 0;
volatile long R_encoder = 0;
volatile int R_old_value = 0;

// 変位、速度制御
float L_x_0 = 0, 
      R_x_0 = 0, 
      L_x = 0, 
      R_x = 0;
float L_v = 0,
      R_v = 0,
      v = 0;
float v_target = 0.3;  // 目標速度
float vel_diff = 0,
      old_vel_diff = 0;
float vel_integral = 0;

// ライントレース制御  センサ発光：700, センサ消灯：300 (0~1023)  黒：発光、白：消灯
/* 環境に応じて変える */
//         端から,  1,      2,      3,      4 番目のセンサ値に対応。4番目だけONを目指す。黒の値は環境によく左右される。
float L_s_target = 200*6 + 200*4 + 200*2 + 530,  // 目標の値
      R_s_target = 200*6 + 200*4 + 200*2 + 530;
float L_s_diff = 0,
      old_L_s_diff = 0,
      R_s_diff = 0,
      old_R_s_diff = 0;
float L_s_integral = 0,
      R_s_integral = 0;

// 十字カウント
int count = 0;

// プログラム開始ボタン判定
int flag = 0;

// 時間(未使用)
int T = 0, T_0 = 0;



// 速度の制御器(PID制御)
float v_PID_Control(const float v)  // 現在の速度を入力
{
  float p, i, d, pid;

   vel_diff = v_target - v;  // 現在値と目標値との差
  // vel_diff = v - v_target;

  vel_integral += vel_diff * dt;  // 積分
  
  p = V_KP * vel_diff;  // Gain-P
  i = V_KI * vel_integral;  // Gain-I
  d = V_KD * (vel_diff - old_vel_diff) / dt;  // Gain-D

  pid = p + i + d;  // 制御量

  old_vel_diff = vel_diff;  // 今回の差分を記録

  // 165：速度の大きさと、その時のデューティー比の値(0~255)の比。これを、算出された速度の制御量に掛けてやることで、デューティー比に変換している。
  return constrain(pid * 165, -255, 255);  // 制御量(デューティー比)を出力
}



// ライントレースによる左車輪の制御器(PID制御)
float L_PID_Control(const float L_s)  // 現在の、左側４つのフォトリフレクタの値の合計を、入力
{
  float p, i, d, pid;

  L_s_diff = L_s_target - L_s;
  // L_s_diff = L_s - L_s_target;

  L_s_integral += L_s_diff * dt;
  
  p = KP * L_s_diff;
  i = KI * L_s_integral;
  d = KD * (L_s_diff - old_L_s_diff) / dt;

  pid = p + i + d;

  old_L_s_diff = L_s_diff;

  return constrain(pid, -255, 255);  // 制御量(デューティー比)を出力
}



// ライントレースによる右車輪の制御器(PID制御)
float R_PID_Control(const float R_s)  // 現在の、右側４つのフォトリフレクタの値の合計を、入力
{
  float p, i, d, pid;

   R_s_diff = R_s_target - R_s;
  // R_s_diff = R_s - R_s_target;

  R_s_integral += R_s_diff * dt;
  
  p = KP * R_s_diff;
  i = KI * R_s_integral;
  d = KD * (R_s_diff - old_R_s_diff) / dt;

  pid = p + i + d;

  old_R_s_diff = R_s_diff;

  return constrain(pid, -255, 255);  // 制御量(デューティー比)を出力
}



void setup()
{
  //ROS_setup
  nh.initNode();
  nh.advertise(linetrace);

  //Serial
  // Serial.begin(9600);

  //L_MOTOR
  pinMode(44, OUTPUT);  // DIR_ROL
  pinMode(6, OUTPUT);  // PWM
  digitalWrite(44, HIGH);  // L_MOTOR_DIR_ROL
  //R_MOTOR
  pinMode(46, OUTPUT);  // DIR_ROL
  pinMode(7, OUTPUT);  // PWM
  digitalWrite(46, LOW);  // R_MOTOR_DIR_ROL

  //PhotoReflecter(LineTracer)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  //StartButton
  pinMode(42, INPUT);

  //L_ENCODER
  pinMode(18, INPUT);  // A相  yellow
  pinMode(19, INPUT);  // B相  white
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);
  attachInterrupt(digitalPinToInterrupt(18), L_encoderUpdate, CHANGE);  // 割り込みの設定
  attachInterrupt(digitalPinToInterrupt(19), L_encoderUpdate, CHANGE);
  //R_ENCODER
  pinMode(3, INPUT);  // A相
  pinMode(2, INPUT);  // B相
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), R_encoderUpdate, CHANGE);  // 割り込みの設定
  attachInterrupt(digitalPinToInterrupt(2), R_encoderUpdate, CHANGE);

  delay(100);  // 適当に待つ
}



void loop()
{
  // int i=0, val=0;
  // for(i=0;i<8;i++){
  // val=analogRead(i);
  // if(i == 7){ Serial.println(val);}
  // else{Serial.print(val);
  // Serial.print(' ');}
  // }
  // delay(1000);

  // プログラム開始スイッチの押下を待つ
  while(flag == 0)
  {
    delay(40);
    if(HIGH == digitalRead(42))
    {
      flag = 1;
      delay(1000);
    }
  }
  while(HIGH == digitalRead(42)){ delay(1000); }  // プログラム開始スイッチ
    
  // T_0 = micros();

  int i=0, t=0, t_0=0;
  // PID制御値格納
  float v_pid=0, L_pid=0, R_pid=0, L_pid_t=0, R_pid_t=0, L_s=0, R_s=0;
  // フォトリフレクタの値を記録
  float s1 = analogRead(A0),
        s2 = analogRead(A1),
        s3 = analogRead(A2),
        s4 = analogRead(A3),
        s5 = analogRead(A4),
        s6 = analogRead(A5),
        s7 = analogRead(A6),
        s8 = analogRead(A7);

  // 距離ｘ：エンコーダ値＊1パルス分の円周＝走行距離
  L_x_0 = L_x;
  R_x_0 = R_x;
  L_x = Cir_CPR * L_encoder * -1;  // エンコーダの値が負なため、正に整形
  R_x = Cir_CPR * R_encoder * -1;

  // 速度ｄｘ
  L_v = (L_x - L_x_0) / dt;  // 左車輪の速度
  R_v = (R_x - R_x_0) / dt;  // 右車輪の速度
  v = (L_v + R_v) / 2;  // ロボットの速度

  // センサ値を格納
  L_s += s1*6;  // センサからの値に重みをつける。
  L_s += s2*4;  // 端から、6, 4, 2 ,1 の重みを付けて、左右のセンサの値の合計に代入する。
  L_s += s3*2;  // 重みをつけることにより、ロボットがラインを外れている大きさに応じて、ラインに戻るための制御量
  L_s += s4;
  R_s += s5;
  R_s += s6*2;
  R_s += s7*4;
  R_s += s8*6;

  // 十字かのチェック
  if(s1 >= 450){ i++; }  // 黒色と判定しているセンサの数をカウント。５個以上であれば、十字を読んでいると判定
  if(s2 >= 450){ i++; }
  if(s3 >= 450){ i++; }
  if(s4 >= 450){ i++; }
  if(s5 >= 450){ i++; }
  if(s6 >= 450){ i++; }
  if(s7 >= 450){ i++; }
  if(s8 >= 450){ i++; }

  // 各制御値の更新。今回操作量の取得
  v_pid = v_PID_Control(v);  // (0 ~ 255)  ロボットの速度を入力として、PID制御の制御器からデューティー比を得る

  L_pid = (L_PID_Control(L_s));  // 左４つのフォトリフレクタの値の合計値を入力として、PID制御の制御器からデューティー比を得る
  R_pid = (R_PID_Control(R_s));  // 右４つのフォトリフレクタの値の合計値を入力として、PID制御の制御器からデューティー比を得る

  //L_pid_t = map(v_pid + L_pid, -255, 500, -255, 255);
  //R_pid_t = map(v_pid + R_pid, -255, 500, -255, 255);
  // L_pid_t = map(v_pid + (L_pid - R_pid), -500, 500, -255, 255);
  // R_pid_t = map(v_pid + (R_pid - L_pid), -500, 500, -255, 255);
  
  // 十字の判定
  if(/*true*/i >= 5)
  {
    if(Check == 0){  // 十字のカウント判定
      Check = 1;
      count++;
      if(count == 4)  // 十字のカウント回数判定( count == 止めたい十字のカウント数 )    ４回、十字を見たらモータを停止させる
      { 
        // デューティー比０
        analogWrite(6, 0);  // L
        analogWrite(7, 0);  // R  
        while(true){ delay(1000); }
      }
    }
    // 速度制御のみ適用
    analogWrite(6, constrain(v_pid, 0, 255));  // L
    analogWrite(7, constrain(v_pid, 0, 255));  // R  
  }
  else
  {
    Check = 0;
    // 速度制御＋ライントレース制御を適用
    // analogWrite関数：デューティー比を0~255の値で与えてやると、PWM制御をしてくれる
        // analogWrite(6, constrain(L_pid_t, 0, 255));  // L
        // analogWrite(7, constrain(R_pid_t, 0, 255));  // R
    analogWrite(6, constrain(v_pid + L_pid, 0, 255));  // L    ロボットの速度の制御量(デューティー比) ＋ 左車輪の制御量(デューティー比)　　
    analogWrite(7, constrain(v_pid + R_pid, 0, 255));  // R    ロボットの速度の制御量(デューティー比) ＋ 右車輪の制御量(デューティー比)
  }
  // Serial.print(L_pid);
  // Serial.print("   ");
  // Serial.println(R_pid);


  // 微小時間 dt にプログラムの周期を合わせる処理
      //Serial.println(v);  // 260[μs] 
  t_0 = micros();
  t = micros();
  while((t - t_0) <= 254)  // 260[μs] ほど待つ。”(t - t_0) <= 254”これでSerial.println(v)と同じ処理時間になる。
  {
    t = micros();
  }

  // T = micros();

  // Serial.println(T - T_0);
  // while(true){ delay(1000); }

  // ROS関係
  msg.L_encoder = abs(L_encoder);
  msg.R_encoder = abs(R_encoder);
  msg.L_x = L_x;
  msg.R_x = R_x;
  msg.L_v = L_v;
  msg.R_v = R_v;
  msg.v = v;
  linetrace.publish(&msg);
  nh.spinOnce();

  // １周期：1956[μs]
}



// 左エンコーダ値の更新  値が変化するたびに割り込む
void L_encoderUpdate()
{
  int L_valueA, L_valueB, L_value, L_rolate;

  L_valueA = digitalRead(18);
  L_valueB = digitalRead(19);

  L_value = (L_valueA << 1) | L_valueB;
  L_rolate = (L_old_value << 2) | L_value;

  if(L_rolate == 0b0010 || L_rolate == 0b0100 || L_rolate == 0b1011 || L_rolate == 0b1101){ L_encoder++; }
  if(L_rolate == 0b0001 || L_rolate == 0b0111 || L_rolate == 0b1000 || L_rolate == 0b1110){ L_encoder--; }

  L_old_value = L_value;
}



// 右エンコーダ値の更新  値が変化するたびに割り込む
void R_encoderUpdate()
{
  int R_valueA, R_valueB, R_value, R_rolate;

  R_valueA = digitalRead(18);
  R_valueB = digitalRead(19);

  R_value = (R_valueA << 1) | R_valueB;
  R_rolate = (R_old_value << 2) | R_value;

  if(R_rolate == 0b0010 || R_rolate == 0b0100 || R_rolate == 0b1011 || R_rolate == 0b1101){ R_encoder++; }
  if(R_rolate == 0b0001 || R_rolate == 0b0111 || R_rolate == 0b1000 || R_rolate == 0b1110){ R_encoder--; }

  R_old_value = R_value;
}