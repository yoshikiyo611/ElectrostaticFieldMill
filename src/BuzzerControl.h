//*****************************************************************************
// ファイル名       BuzzerControl.h
// 対象マイコン     RP2040
// ファイル内容     ブザー制御
//*****************************************************************************
#ifndef BUZZERCONTROL_H_
#define BUZZERCONTROL_H_

//=============================================================================
//定数設定
//=============================================================================
#define Pwm2PRD 15385  //60MHz/PWM周波数3.9[kHz]=15385 (ブザー用PWM)

//=============================================================================
//プロトタイプ宣言
//=============================================================================
void init_pwm();
void init_beep( void );
void beep_out( int f );
void set_beep_pattern( unsigned int data );
void beep_process( void );

#endif
//*****************************************************************************
// 終わり
//*****************************************************************************
