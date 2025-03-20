//*****************************************************************************
// ファイル名       ElectrostaticFieldMill.c
// 対象マイコン     RP2040
// ファイル内容     表面電位計
//*****************************************************************************
//=============================================================================
//include
//=============================================================================
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "LcdControl.h"
#include "SwitchControl.h"
#include "BuzzerControl.h"

//=============================================================================
// マクロ定義
//=============================================================================
// GPIOピンの定義
#define LCD_PIN_D4           0   // LCDデータピン4
#define LCD_PIN_D5           1   // LCDデータピン5
#define LCD_PIN_D6           2   // LCDデータピン6
#define LCD_PIN_D7           3   // LCDデータピン7
#define LCD_PIN_E            4   // LCDイネーブルピン
#define LCD_PIN_RS           5   // LCDレジスタ選択ピン
#define SWITCH_PIN_1        10   // スイッチ入力1
#define SWITCH_PIN_2        11   // スイッチ入力2
#define SWITCH_PIN_3        12   // スイッチ入力3
#define SWITCH_PIN_4        13   // スイッチ入力4
#define SWITCH_PIN_5        14   // スイッチ入力5
#define DEBUG_GPIO_PIN      15   // デバッグ用GPIOピン
#define DEBUG_OUT_PIN       16   // デバッグ出力ピン
#define LED_RED_PIN         17   // 赤LEDピン
#define LED_BLUE_PIN        18   // 青LEDピン
#define DHT11_PIN           19   // DHT11データピン
#define SHUTTER_SENSOR_PIN  20   // シャッター位置検出センサー
#define PWM_PIN             22   // PWM出力ピン
#define ADC_PIN             26   // ADC入力ピン (GPIO26)

// 定数の定義
#define ADC_MID_VALUE       2048  // ADCの中間値 (12ビット)
#define SHUTTER_CYCLE_THRESHOLD 10  // シャッター回転数の閾値
#define POTENTIAL_CONVERSION_FACTOR 0.01028f  // 表面電位変換係数 [kV/ADC値]
#define ADC_CLOCK_FREQ      48000000  // ADCクロック周波数 (48MHz)
#define ADC_SAMPLE_FREQ_HZ  25000  // ADCサンプリング周波数 (25kHz)
#define PWM_CLOCK_FREQ      125000000  // PWMクロック周波数 (125MHz)
#define PWM_FREQ_HZ         20000  // PWM周波数 (20kHz)
#define PWM_WRAP_VALUE      ((PWM_CLOCK_FREQ / PWM_FREQ_HZ) - 1)  // PWMラップ値
#define STARTUP_DELAY_MS    1000  // 起動時の待機時間 (ms)
#define BEEP_PATTERN_START  0xA   // 起動時のビープパターン

//=============================================================================
// 表面電位計測用構造体定義
//=============================================================================
typedef struct {
    uint32_t shutter_count;    // シャッター回転数
    uint32_t sample_count;     // サンプル数
    int64_t sync_value;        // 同期検波値
    int64_t positive_sum;      // 正側合計
    int64_t negative_sum;      // 負側合計
    bool prev_shutter_state;   // 前回のシャッター状態
} SyncDemodState;

//=============================================================================
// グローバル変数
//=============================================================================
SyncDemodState demod_state = {0, 0, 0, 0, 0, false};
int32_t adc_average = 0;         // ADC平均値
int16_t surface_potential_sign = 0; // 表面電位の符号 (1:正, -1:負)
float surface_potential_kv = 0.0f;  // 表面電位 [kV]
uint pwm_slice_num;              // PWMスライス番号
float temperature = 0.0f;        // 温度 [℃]
float humidity = 0.0f;           // 湿度 [%RH]

//=============================================================================
// 関数プロトタイプ宣言
//=============================================================================
bool timer_callback(repeating_timer_t *rt);
void init_rp2040(void);
void core1_main(void);
void display_process(void);
static void adc_irq_handler(void);
bool read_dht11(float *temp, float *hum);

//*****************************************************************************
// コア0: メイン処理
//*****************************************************************************
int main(void) {
    init_rp2040();

    // LCDに起動メッセージを表示
    lcd_position(0, 0);
    lcd_printf("Surface         ");
    lcd_position(0, 1);
    lcd_printf(" potential meter");

    // 起動音
    set_beep_pattern(BEEP_PATTERN_START);
    sleep_ms(STARTUP_DELAY_MS);

    // メインループ
    while (true) {
        // 表面電位を計算 (kV単位)
        surface_potential_kv = fabsf(adc_average) * POTENTIAL_CONVERSION_FACTOR;

        // DHT11から温湿度を取得 (2秒ごとに更新)
        static uint64_t last_dht11_read = 0;
        if (time_us_64() - last_dht11_read >= 2000000) { // 2秒間隔
            read_dht11(&temperature, &humidity);
            last_dht11_read = time_us_64();
        }

        display_process();
    }
}

//*****************************************************************************
// コア1: サブ処理 (未実装)
//*****************************************************************************
void core1_main(void) {
    while (true) {
        // 必要に応じて実装
    }
}

//*****************************************************************************
// RP2040の周辺機器初期化
//*****************************************************************************
void init_rp2040(void) {
    stdio_init_all(); // USBシリアル初期化

    // === GPIO設定 ===
    const uint lcd_pins[] = {LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7, LCD_PIN_E, LCD_PIN_RS};
    for (int i = 0; i < 6; i++) {
        gpio_init(lcd_pins[i]);
        gpio_set_dir(lcd_pins[i], GPIO_OUT);
    }

    const uint switch_pins[] = {SWITCH_PIN_1, SWITCH_PIN_2, SWITCH_PIN_3, SWITCH_PIN_4, SWITCH_PIN_5};
    for (int i = 0; i < 5; i++) {
        gpio_init(switch_pins[i]);
        gpio_set_dir(switch_pins[i], GPIO_IN);
    }

    gpio_init(DEBUG_GPIO_PIN);
    gpio_set_dir(DEBUG_GPIO_PIN, GPIO_OUT);
    gpio_init(DEBUG_OUT_PIN);
    gpio_set_dir(DEBUG_OUT_PIN, GPIO_OUT);

    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);

    gpio_init(DHT11_PIN);           // DHT11ピンの初期化
    gpio_set_dir(DHT11_PIN, GPIO_OUT); // 初期状態は出力
    gpio_put(DHT11_PIN, 1);         // HIGHに設定

    gpio_init(SHUTTER_SENSOR_PIN);
    gpio_set_dir(SHUTTER_SENSOR_PIN, GPIO_IN);
    gpio_disable_pulls(SHUTTER_SENSOR_PIN);

    // === タイマー設定 ===
    static repeating_timer_t timer;
    add_repeating_timer_ms(-1, timer_callback, NULL, &timer);

    // === ADC設定 ===
    adc_init();
    adc_set_clkdiv((ADC_CLOCK_FREQ / ADC_SAMPLE_FREQ_HZ) - 1.0f); // 25kHz
    adc_gpio_init(ADC_PIN);       // ADC0
    adc_select_input(0);          // ADC0を選択
    adc_set_round_robin(0);       // ラウンドロビン無効

    adc_fifo_setup(true, false, 1, false, false);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);
    irq_set_enabled(ADC_IRQ_FIFO, true);
    adc_irq_set_enabled(true);
    irq_set_priority(ADC_IRQ_FIFO, 0);

    sleep_ms(1);
    adc_run(true); // ADCフリーラン開始

    // === PWM設定 ===
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_output_polarity(pwm_slice_num, PWM_CHAN_A, true);
    pwm_set_wrap(pwm_slice_num, PWM_WRAP_VALUE); // 20kHz
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // === ライブラリ初期化 ===
    lcd_init();
    switch_init();
    init_beep();
}

//*****************************************************************************
// LCDとスイッチを使った表示処理
//*****************************************************************************
void display_process(void) {
    static unsigned int parameter_pattern = 1;
    unsigned int set_min, set_max;
    static bool dot_blink = false; // ドットの点滅状態を管理
    static uint64_t last_blink_update = 0; // 最後の点滅更新時刻

    set_min = 1;
    set_max = 3;

    // DHT11のサンプリングタイミング（2秒ごと）に合わせて点滅を更新
    if (time_us_64() - last_blink_update >= 2000000) { // 2秒間隔
        dot_blink = !dot_blink; // 点滅状態を反転
        last_blink_update = time_us_64();
    }

    if (get_sw_flag(SW_1)) {
        parameter_pattern++;
        if (parameter_pattern > set_max) parameter_pattern = set_min;
    }
    if (get_sw_flag(SW_2)) {
        parameter_pattern--;
        if (parameter_pattern < set_min) parameter_pattern = set_max;
    }

    switch (parameter_pattern) {
        case 1:
            lcd_position(0, 0);
            lcd_printf("Surf. Potential ");
            lcd_position(0, 1);
            lcd_printf("   = %+6.2f [kV]", surface_potential_kv * surface_potential_sign);
            if (get_sw_flag(SW_3)) {
                set_beep_pattern(0xA);
                pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 650);
            }
            if (get_sw_flag(SW_4)) {
                set_beep_pattern(0xF);
                pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
            }
            break;

        case 2:
            lcd_position(0, 0);
            lcd_printf("ADC Count       ");
            lcd_position(0, 1);
            lcd_printf("         = %+5ld", abs(adc_average));
            break;

            case 3:
            lcd_position(0, 0);
            lcd_printf("Temp:%5.1f C   %c", temperature, dot_blink ? '.' : ' ');
            lcd_position(0, 1);
            lcd_printf("Hum: %5.1f %%RH %c", humidity, dot_blink ? '.' : ' ');
            break;

        default:
            break;
    }
}

//*****************************************************************************
// タイマー割り込み処理 (1msごと)
//*****************************************************************************
bool timer_callback(repeating_timer_t *rt) {

    lcd_process();
    switch_process();
    beep_process();

    return true;
}

//*****************************************************************************
// ADC割り込み処理
//*****************************************************************************
static void adc_irq_handler(void) {
    // ADC値を取得
    int32_t adc_value = adc_fifo_get() - ADC_MID_VALUE;

    // シャッター状態を取得
    bool shutter_open = gpio_get(SHUTTER_SENSOR_PIN);

    // 同期検波
    int32_t sync_factor = shutter_open ? 1 : -1;
    demod_state.sync_value += adc_value * sync_factor;
    if (shutter_open) {
        demod_state.positive_sum += adc_value;
    } else {
        demod_state.negative_sum += adc_value;
    }
    demod_state.sample_count++;

    // シャッター状態変化を検出
    if (shutter_open != demod_state.prev_shutter_state) {
        demod_state.shutter_count++;
        demod_state.prev_shutter_state = shutter_open;
    }

    // 指定回転数ごとに平均値を計算
    if (demod_state.shutter_count >= SHUTTER_CYCLE_THRESHOLD) {
        adc_average = demod_state.sync_value / demod_state.sample_count;

        // 電位の正負判定とLED制御
        surface_potential_sign = (demod_state.positive_sum > demod_state.negative_sum) ? 1 : -1;
        gpio_put(LED_RED_PIN, surface_potential_sign > 0);  // 赤LED
        gpio_put(LED_BLUE_PIN, surface_potential_sign < 0); // 青LED

        // 状態リセット
        demod_state.sync_value = 0;
        demod_state.sample_count = 0;
        demod_state.positive_sum = 0;
        demod_state.negative_sum = 0;
        demod_state.shutter_count = 0;
    }

    // デバッグ用出力
    gpio_put(DEBUG_OUT_PIN, shutter_open);
}

//*****************************************************************************
// DHT11から温湿度を読み取る
//*****************************************************************************
bool read_dht11(float *temp, float *hum) {
    uint8_t data[5] = {0}; // DHT11は40ビットのデータ (5バイト) を送信
    uint32_t timeout;

    sleep_ms(100); // 前回の読み取りから十分な時間を確保

    // ピンを出力に設定し、開始信号を送信 (20ms LOW, その後HIGH)
    gpio_set_dir(DHT11_PIN, GPIO_OUT);
    gpio_put(DHT11_PIN, 0);
    sleep_ms(20);
    gpio_put(DHT11_PIN, 1);
    sleep_us(40);

    // ピンを入力に切り替え、DHT11の応答を待つ
    gpio_set_dir(DHT11_PIN, GPIO_IN);
    timeout = 10000;
    while (gpio_get(DHT11_PIN) && timeout--) sleep_us(1); // LOWを待つ
    if (timeout == 0) {
        irq_set_enabled(ADC_IRQ_FIFO, true); // 割り込み復帰
        return false;
    }
    timeout = 10000;
    while (!gpio_get(DHT11_PIN) && timeout--) sleep_us(1); // HIGHを待つ
    if (timeout == 0) {
        irq_set_enabled(ADC_IRQ_FIFO, true); // 割り込み復帰
        return false;
    }
    timeout = 10000;
    while (gpio_get(DHT11_PIN) && timeout--) sleep_us(1); // LOWを待つ
    if (timeout == 0) {
        irq_set_enabled(ADC_IRQ_FIFO, true); // 割り込み復帰
        return false;
    }

    // 40ビットのデータを受信
    for (int i = 0; i < 5; i++) {
        for (int j = 7; j >= 0; j--) {
            timeout = 10000;
            while (!gpio_get(DHT11_PIN) && timeout--) sleep_us(1); // HIGHを待つ
            if (timeout == 0) {
                irq_set_enabled(ADC_IRQ_FIFO, true); // 割り込み復帰
                return false;
            }
            sleep_us(60); // タイミング調整
            if (gpio_get(DHT11_PIN)) {
                data[i] |= (1 << j); // HIGHなら1
            }
            timeout = 10000;
            while (gpio_get(DHT11_PIN) && timeout--) sleep_us(1); // LOWを待つ
            if (timeout == 0) {
                irq_set_enabled(ADC_IRQ_FIFO, true); // 割り込み復帰
                return false;
            }
        }
    }

    // チェックサム検証
    uint8_t calc_checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (data[4] == calc_checksum) {
        *hum = (float)data[0] + (float)data[1] / 10.0f;  // 湿度 (小数点対応)
        *temp = (float)data[2] + (float)data[3] / 10.0f; // 温度 (小数点対応)
        return true;
    } else {
        return false;
    }
}

//*****************************************************************************
// 終わり
//*****************************************************************************