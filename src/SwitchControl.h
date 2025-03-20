//*****************************************************************************
// ファイル名       SwitchControl.h
// 対象マイコン     RP2040
// ファイル内容     スイッチ制御用ライブラリ
//*****************************************************************************
#ifndef SWITCHCONTROL_H_
#define SWITCHCONTROL_H_

//=============================================================================
//シンボル定義
//=============================================================================
#define SW_0                (0x01 << 0) // get_sw_now関数のbit0の位置(キースイッチ)
#define SW_1                (0x01 << 1) // get_sw_now関数のbit1の位置
#define SW_2                (0x01 << 2) // get_sw_now関数のbit2の位置
#define SW_3                (0x01 << 3) // get_sw_now関数のbit3の位置
#define SW_4                (0x01 << 4) // get_sw_now関数のbit4の位置

#define SWITCH_BIT          5           // スイッチの数
#define SWITCH_INTERVAL     1           // スイッチ処理インターバル
#define SWITCH_ON_TIME      100         // ONと判断するまでの時間
#define SWITCH_REPEAT_INTERVAL  300     // いったんONしてからリピートするまでの時間
#define SWITCH_REPEAT_TIME  300         // リピート間隔

//=============================================================================
//プロトタイプ宣言
//=============================================================================
void switch_init(void);
unsigned int get_sw_now(void);
unsigned int get_sw_flag(unsigned int flag);
void switch_process(void);

#endif
//*****************************************************************************
// 終わり
//*****************************************************************************