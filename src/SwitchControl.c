//*****************************************************************************
// ファイル名       SwitchControl.c
// 対象マイコン     RP2040
// ファイル内容     スイッチ制御用ライブラリ
//*****************************************************************************
//=============================================================================
//include
//=============================================================================
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "hardware/gpio.h"
#include "SwitchControl.h"

//=============================================================================
//グローバル変数の宣言
//=============================================================================
unsigned int    switch_mode[SWITCH_BIT];    // スイッチ処理モードNo用
unsigned int    switch_run_count;           // 何回毎に処理するかのカウント
unsigned int    sw_flag;                    // 全ビットまとめたスイッチフラグ

struct {
    unsigned int    time;       // 信号が続いている時間
    unsigned int    flag;       // "0"→"1"で"1"になる
    unsigned int    count;      // カウント
} sw_data[SWITCH_BIT];

//*****************************************************************************
// スイッチ関連変数初期化
//*****************************************************************************
void switch_init(void) {
    unsigned int i;

    switch_run_count = 0;
    sw_flag = 0x00;

    for (i = 0; i < SWITCH_BIT; i++) {
        switch_mode[i] = 0;
        sw_data[i].time = 0;
        sw_data[i].flag = 0;
        sw_data[i].count = 0;
    }
}

//*****************************************************************************
// スイッチ現在状態取得
//*****************************************************************************
unsigned int get_sw_now(void) {
    unsigned char sw = 0;

    sw = gpio_get(14);           // SW_0 (SWITCH_PIN_5)
    sw |= (gpio_get(13) << 1);   // SW_1 (SWITCH_PIN_4)
    sw |= (gpio_get(12) << 2);   // SW_2 (SWITCH_PIN_3)
    sw |= (gpio_get(11) << 3);   // SW_3 (SWITCH_PIN_2)
    sw |= (gpio_get(10) << 4);   // SW_4 (SWITCH_PIN_1)

    return (~sw) & 0x1f;         // LOWでON、5ビットのみ有効
}

//*****************************************************************************
// スイッチフラグ状態取得
//*****************************************************************************
unsigned int get_sw_flag(unsigned int flag) {
    unsigned int ret;

    ret = sw_flag & flag;
    sw_flag &= ~flag;            // フラグをクリア

    return ret;
}

//*****************************************************************************
// スイッチ処理
//*****************************************************************************
void switch_process(void) {
    unsigned int i;
    unsigned int sw, flag = 0;

    // 処理間隔のチェック
    switch_run_count++;
    if (switch_run_count < SWITCH_INTERVAL) return;
    switch_run_count = 0;

    sw = get_sw_now();

    // スイッチのビット毎に処理
    for (i = 0; i < SWITCH_BIT; i++) {
        switch (switch_mode[i]) {
            case 0: // ONの判定
                if (sw & 0x01) {
                    sw_data[i].time++;
                    if (sw_data[i].time >= SWITCH_ON_TIME) {
                        switch_mode[i] = 1;
                        sw_data[i].count++;
                        sw_data[i].flag = 1;
                        flag |= 0x80;
                        sw_data[i].time = 0;
                    }
                } else {
                    sw_data[i].time = 0;
                }
                break;

            case 1: // キーリピートの判定
                if (sw & 0x01) {
                    sw_data[i].time++;
                    if (sw_data[i].time >= SWITCH_REPEAT_INTERVAL) {
                        switch_mode[i] = 2;
                        sw_data[i].count++;
                        sw_data[i].flag = 1;
                        flag |= 0x80;
                        sw_data[i].time = 0;
                    }
                } else {
                    switch_mode[i] = 0;
                    sw_data[i].time = 0;
                }
                break;

            case 2: // キーリピート中、等間隔でフラグON
                if (sw & 0x01) {
                    sw_data[i].time++;
                    if (sw_data[i].time >= SWITCH_REPEAT_TIME) {
                        sw_data[i].count++;
                        sw_data[i].flag = 1;
                        flag |= 0x80;
                        sw_data[i].time = 0;
                    }
                } else {
                    switch_mode[i] = 0;
                    sw_data[i].time = 0;
                }
                break;
        }
        sw >>= 1;
        if (i < 7) flag >>= 1;
    }
    sw_flag |= flag >> (7 - SWITCH_BIT);
}

//*****************************************************************************
// 終わり
//*****************************************************************************