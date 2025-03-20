//*****************************************************************************
// ファイル名       LcdControl.c
// 対象マイコン     RP2040
// ファイル内容     LCD制御用ライブラリ
//*****************************************************************************
//=============================================================================
//include
//=============================================================================
#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "hardware/gpio.h"
#include "LcdControl.h"

//=============================================================================
//グローバル変数宣言
//=============================================================================
char            buff_lcd_data[LCD_MAX_X * LCD_MAX_Y];
char            buff_lcd_data2[LCD_MAX_X * LCD_MAX_Y];
int             lcd_pos;                // バッファに書き込む位置
int             lcd_mode = 1;
int             lcd_now_locate;
int             f_lcd_refresh;

//=============================================================================
//プロトタイプ宣言(ローカル)
//=============================================================================
void lcd_locate(int x, int y);
void lcd_out(char command, char data);
void lcd_out2(char data2);
void lcd_dr(char data);
void lcd_nop(volatile int i);

//*****************************************************************************
// 液晶処理 初期化
//*****************************************************************************
int lcd_init(void) {
    int i;

    for (i = 0; i <= LCD_MAX_X * LCD_MAX_Y - 1; i++) {
        buff_lcd_data[i] = ' ';
    }

    sleep_ms(20);
    lcd_out2(0x03);
    sleep_ms(5);
    lcd_out2(0x03);
    sleep_ms(5);
    lcd_out2(0x03);
    lcd_out2(0x02);
    sleep_ms(5);

    lcd_out(LCD_INST, 0x28);  // 4-bit mode, 2 lines, 5x7 font
    sleep_ms(5);
    lcd_out(LCD_INST, 0x08);  // Display off
    sleep_ms(5);
    lcd_out(LCD_INST, 0x01);  // Display clear
    sleep_ms(5);
    lcd_out(LCD_INST, 0x06);  // Entry mode: increment, no shift
    sleep_ms(5);
    lcd_out(LCD_INST, 0x0c);  // Display on, cursor off, blink off
    sleep_ms(5);

    return 1;
}

//*****************************************************************************
// 液晶カーソル移動
//*****************************************************************************
void lcd_locate(int x, int y) {
    unsigned char work = 0x80;

    work += x;
    if (y == 1) {
        work += 0x40;
    } else if (y == 2) {
        work += 0x14;
    } else if (y == 3) {
        work += 0x54;
    }
    lcd_out(LCD_INST, work);
}

//*****************************************************************************
// 液晶コマンド出力
//*****************************************************************************
void lcd_out(char command, char data) {
    unsigned char work;

    work = (unsigned char)command | ((unsigned char)data >> 4);
    lcd_out2(work);

    work = (unsigned char)command | ((unsigned char)data & 0x0f);
    lcd_out2(work);
}

//*****************************************************************************
// 液晶データ出力
//*****************************************************************************
void lcd_out2(char data2) {
    lcd_dr(data2);
    lcd_nop(60);
    lcd_dr(data2 | LCD_BIT_E);
    lcd_nop(60);
    lcd_dr(data2);
    lcd_nop(60);
}

//*****************************************************************************
// IOへデータ出力
//*****************************************************************************
void lcd_dr(char data) {
    gpio_put(0, (data & 0x01));  // D4
    gpio_put(1, (data & 0x02));  // D5
    gpio_put(2, (data & 0x04));  // D6
    gpio_put(3, (data & 0x08));  // D7
    gpio_put(5, (data & 0x10));  // RS
    gpio_put(4, (data & 0x40));  // E
}

//*****************************************************************************
// 液晶表示処理
//*****************************************************************************
void lcd_process(void) {
    switch (lcd_mode) {
        case 1:
            if (f_lcd_refresh) {
                f_lcd_refresh = 0;
                lcd_mode = 2;
            }
            break;

        case 2:
            lcd_now_locate = 0;
            lcd_locate(0, 0);
            lcd_mode = 3;
            break;

        case 3:
            if (lcd_now_locate % LCD_MAX_X == 0) {
                lcd_locate(0, lcd_now_locate / LCD_MAX_X);
            }
            lcd_mode = 4;
            break;

        case 4:
            lcd_out(LCD_DATA, buff_lcd_data[lcd_now_locate++]);
            if (lcd_now_locate >= LCD_MAX_X * LCD_MAX_Y) {
                lcd_mode = 1;
            } else {
                lcd_mode = 3;
            }
            break;

        default:
            lcd_mode = 1;
            break;
    }
}

//*****************************************************************************
// 液晶へ表示
//*****************************************************************************
int lcd_printf(char *format, ...) {
    va_list argptr;
    char    *p;
    int     ret = 0;

    va_start(argptr, format);
    ret = vsprintf(buff_lcd_data2, format, argptr);
    va_end(argptr);

    if (ret > 0) {
        p = buff_lcd_data2;
        while (*p) {
            buff_lcd_data[lcd_pos++] = *p++;
            if (lcd_pos >= LCD_MAX_X * LCD_MAX_Y) {
                lcd_pos = 0;
            }
        }
        f_lcd_refresh = 1;
    }

    return ret;
}

//*****************************************************************************
// 液晶の表示位置指定
//*****************************************************************************
void lcd_position(char x, char y) {
    if (x >= LCD_MAX_X) return;
    if (y >= LCD_MAX_Y) return;

    lcd_pos = x + y * LCD_MAX_X;
}

//*****************************************************************************
// 極短いタイミング用タイマ
//*****************************************************************************
void lcd_nop(volatile int i) {
    i = i * 15;
    while (i--);
}

//*****************************************************************************
// 終わり
//*****************************************************************************