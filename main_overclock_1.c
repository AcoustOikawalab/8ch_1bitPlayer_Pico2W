#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000   // 5 秒待つ
#define PICO_STDIO_USB_POST_CONNECT_WAIT_DELAY_MS 500 // さらに 0.5 秒

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "ff.h"
#include "diskio.h"
#include "wsd_player.pio.h"
#include "hw_config.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/irq.h"
#include "hardware/structs/sio.h"
#include "hardware/regs/dma.h"    // DMA_BASE, DMA1_BASE
#include "hardware/structs/dma.h" // dma_hw_t
#include <stdbool.h>
#include "pico/time.h"


// 1bit出力用GPIO
#define PIN_a 14  
#define PIN_b 15
#define PIN_c 16
#define PIN_d 17
#define PIN_e 18
#define PIN_f 19
#define PIN_g 20
#define PIN_h 21

#define SAMPLE_RATE   3000000
#define CYCLE_PER_BIT 2
#define WSD_PATH      "DayByDay_8ch.wsd"  // 再生ファイル名
#define BUF_SIZE   (32 * 1024)

//static FATFS fs;
static FIL fil;

// 先頭のバッファ定義を置き換え
#define NUM_BUF 4
#define QDEPTH  8   // キュー深さも余裕を

#define LOOP_PLAY 0  // 1=無限ループ再生、0=一回で終了

static volatile bool     g_loop_enable = LOOP_PLAY;
static volatile uint32_t g_frame_ofs   = 0;      // ヘッダ解析で決めた先頭位置を格納

static uint8_t  buf[NUM_BUF][BUF_SIZE];
static uint32_t packedAB[NUM_BUF][BUF_SIZE/8];         // 4byte→1word（A/B）
static uint32_t packedCD[NUM_BUF][BUF_SIZE/8];         // 4byte→1word（C/D）
volatile bool   buf_ready[NUM_BUF] = {0};
volatile bool   buf_done[NUM_BUF]  = {0};
static UINT     br[NUM_BUF];
static size_t   br_word[NUM_BUF];
static volatile uint8_t use_cnt[NUM_BUF] = {0};  // idx=0/1 が残り何回消費されるか（AB+CDで2）

PIO pio = pio0;
static uint sm_ab, sm_cd;         // SM 2本

// 供給アンダーフロー/ポップ/IRQ/解放の統計
static volatile uint32_t uf_ab = 0, uf_cd = 0;         // q_pop失敗（供給不足）
static volatile uint32_t pop_ab = 0, pop_cd = 0;       // q_pop成功（供給できた）
static volatile uint32_t isr_ab0 = 0, isr_ab1 = 0;     // 各DMAの完了割り込み回数
static volatile uint32_t isr_cd0 = 0, isr_cd1 = 0;
static volatile uint32_t rel0 = 0, rel1 = 0;           // use_cnt==0 で解放された回数（buf0/1）
static volatile int cur_ab0 = -1, cur_ab1 = -1, cur_cd0 = -1, cur_cd1 = -1;

// ==== DMA 連鎖（ping-pong）用 ====
static void dma_isr_audio(void);  // ISR 前方宣言

// 各SM（AB/CD）にDMAを2本：AB0<->AB1、CD0<->CD1
static int dma_ab0, dma_ab1, dma_cd0, dma_cd1;

// --- リングキュー（AB/CD）と参照カウント ---
static volatile int q_ab[QDEPTH], q_cd[QDEPTH];
static volatile uint8_t qh_ab=0, qt_ab=0, qh_cd=0, qt_cd=0;

// ==== DMA helper (RP2350両対応) ====
// ch>=16 を DMA1 とみなす（RP2350: 各コントローラ 0..15）
static inline bool ch_is_dma1(int ch) { return (ch >= 16); }
static inline uint32_t ch_bit(int ch) { return 1u << (ch & 0x0F); }

// チャネルから該当コントローラの dma_hw_t* を取得
static inline dma_hw_t* dma_ctrl_for_ch(int ch) {
#ifdef DMA1_BASE
    return ch_is_dma1(ch) ? (dma_hw_t*)DMA1_BASE : dma_hw;
#else
    return dma_hw; // RP2040 等、DMA が1個の時
#endif
}

static inline void clear_irq_for_ch(int ch) {
    dma_ctrl_for_ch(ch)->ints0 = ch_bit(ch);
}

// 空きチャネルを 2 本確保し、必ず同一 DMA 内に揃える
static void claim_pair_same_dma(int *ch0, int *ch1) {
    *ch0 = dma_claim_unused_channel(true);
    for (;;) {
        int c = dma_claim_unused_channel(true);
        if (ch_is_dma1(c) == ch_is_dma1(*ch0)) { *ch1 = c; break; }
        dma_channel_unclaim(c);
    }
}

// ==== リングキュー（AB/CD 用）のプッシュ／ポップ ====
// 既に QDEPTH, q_* 配列や qh_*, qt_* が宣言済みならそのまま使えます
static inline bool q_push(volatile int *q, volatile uint8_t *qh, volatile uint8_t *qt, int v){
    uint8_t nt = (uint8_t)((*qt + 1) & (QDEPTH-1));
    if (nt == *qh) return false;  // full
    q[*qt] = v; *qt = nt; return true;
}
static inline bool q_pop(volatile int *q, volatile uint8_t *qh, volatile uint8_t *qt, int *out){
    if (*qh == *qt) return false; // empty
    *out = q[*qh]; *qh = (uint8_t)((*qh + 1) & (QDEPTH-1)); return true;
}

// === 再生完了判定用 ===
static volatile bool g_core1_eof = false;   // core1 が EOF を見たら true

static inline void spi_flush(spi_inst_t *spi) {
    // RX FIFO が空になるまで読み捨て
    while (spi_is_readable(spi)) {
        (void) spi_get_hw(spi)->dr;
    }
    // オーバーラン／タイムアウト・フラグもクリア
    spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS | SPI_SSPICR_RTIC_BITS;
}

// PIOステートマシン初期化
void pio_setup_dual() {
    uint offset = pio_add_program(pio, &wsd_player_program);
    /* 指定したPIOのインストラクションメモリにプログラムをロードする */
    /* プログラムの先頭アドレス(オフセット)が戻り値として返ってくる */

    // AB: 16..19
    sm_ab = pio_claim_unused_sm(pio, true);
    for(int p=PIN_a; p<=PIN_d; ++p) pio_gpio_init(pio, p);
    pio_sm_config cab = wsd_player_program_get_default_config(offset);
    sm_config_set_out_pins (&cab, PIN_a, 4);
    sm_config_set_set_pins (&cab, PIN_a, 4);
    sm_config_set_out_shift(&cab, /*shift_right=*/true, /*autopull=*/true, 32);
    sm_config_set_fifo_join(&cab, PIO_FIFO_JOIN_TX);
    pio_sm_set_consecutive_pindirs(pio, sm_ab, PIN_a, 4, true);

    // CD: 20..23
    sm_cd = pio_claim_unused_sm(pio, true);
    for(int p=PIN_e; p<=PIN_h; ++p) pio_gpio_init(pio, p);
    pio_sm_config ccd = wsd_player_program_get_default_config(offset);
    sm_config_set_out_pins (&ccd, PIN_e, 4);
    sm_config_set_set_pins (&ccd, PIN_e, 4);
    sm_config_set_out_shift(&ccd, /*shift_right=*/true, /*autopull=*/true, 32);
    sm_config_set_fifo_join(&ccd, PIO_FIFO_JOIN_TX);
    pio_sm_set_consecutive_pindirs(pio, sm_cd, PIN_e, 4, true);

    // 3 MHz / 1bit = 2命令（out + jmp） → target = 2 * 3e6 = 6e6
    const uint32_t sys = clock_get_hz(clk_sys);
    const double target = CYCLE_PER_BIT * (double)SAMPLE_RATE;
    const double div    = (double)sys / target;       // 150e6 / 6e6 = 25.0
    uint16_t div_int  = (uint16_t)div;
    uint8_t  div_frac = (uint8_t)((div - div_int) * 256.0 + 0.5);

    sm_config_set_clkdiv_int_frac(&cab, div_int, div_frac);
    sm_config_set_clkdiv_int_frac(&ccd, div_int, div_frac);

    printf("sys=%u  div=%u+%u/256  bit=%.0f Hz\n",
       sys, div_int, div_frac,
       sys / (CYCLE_PER_BIT * (div_int + div_frac / 256.0)));

    pio_sm_init(pio, sm_ab, offset, &cab);
    pio_sm_init(pio, sm_cd, offset, &ccd);
    pio_sm_set_enabled(pio, sm_ab, true);
    pio_sm_set_enabled(pio, sm_cd, true);

    printf("pio set up done\n");
}

static void dma_setup_chain(void){
    // === AB側：2本を相互連鎖 ===
    claim_pair_same_dma(&dma_ab0, &dma_ab1);

    #ifdef DMA1_BASE
    bool ab_is_dma1 = ch_is_dma1(dma_ab0);
    do { dma_cd0 = dma_claim_unused_channel(true); } while (ch_is_dma1(dma_cd0) == ab_is_dma1);
    do { dma_cd1 = dma_claim_unused_channel(true); } while (ch_is_dma1(dma_cd1) != ch_is_dma1(dma_cd0));
    #else
    claim_pair_same_dma(&dma_cd0, &dma_cd1);
    #endif
    
    dma_channel_config cA0 = dma_channel_get_default_config(dma_ab0);
    channel_config_set_transfer_data_size(&cA0, DMA_SIZE_32);
    channel_config_set_read_increment    (&cA0, true);
    channel_config_set_write_increment   (&cA0, false);
    channel_config_set_dreq              (&cA0, pio_get_dreq(pio, sm_ab, true));
    channel_config_set_chain_to          (&cA0, dma_ab1);
    dma_channel_configure(dma_ab0, &cA0, &pio->txf[sm_ab], NULL, 0, false);

    dma_channel_config cA1 = dma_channel_get_default_config(dma_ab1);
    channel_config_set_transfer_data_size(&cA1, DMA_SIZE_32);
    channel_config_set_read_increment    (&cA1, true);
    channel_config_set_write_increment   (&cA1, false);
    channel_config_set_dreq              (&cA1, pio_get_dreq(pio, sm_ab, true));
    channel_config_set_chain_to          (&cA1, dma_ab0);
    dma_channel_configure(dma_ab1, &cA1, &pio->txf[sm_ab], NULL, 0, false);

    dma_channel_config cC0 = dma_channel_get_default_config(dma_cd0);
    channel_config_set_transfer_data_size(&cC0, DMA_SIZE_32);
    channel_config_set_read_increment    (&cC0, true);
    channel_config_set_write_increment   (&cC0, false);
    channel_config_set_dreq              (&cC0, pio_get_dreq(pio, sm_cd, true));
    channel_config_set_chain_to          (&cC0, dma_cd1);
    dma_channel_configure(dma_cd0, &cC0, &pio->txf[sm_cd], NULL, 0, false);

    dma_channel_config cC1 = dma_channel_get_default_config(dma_cd1);
    channel_config_set_transfer_data_size(&cC1, DMA_SIZE_32);
    channel_config_set_read_increment    (&cC1, true);
    channel_config_set_write_increment   (&cC1, false);
    channel_config_set_dreq              (&cC1, pio_get_dreq(pio, sm_cd, true));
    channel_config_set_chain_to          (&cC1, dma_cd0);
    dma_channel_configure(dma_cd1, &cC1, &pio->txf[sm_cd], NULL, 0, false);

    // 取れたチャネルを表示（どのDMAに入ったかの確認用）
    printf("DMA AB : %d & %d  (DMA%d)\n", dma_ab0, dma_ab1, ch_is_dma1(dma_ab0)?1:0);
    printf("DMA CD : %d & %d  (DMA%d)\n", dma_cd0, dma_cd1, ch_is_dma1(dma_cd0)?1:0);

    // 完了割り込み 4本分を有効化
    dma_channel_set_irq0_enabled(dma_ab0, true);
    dma_channel_set_irq0_enabled(dma_ab1, true);
    dma_channel_set_irq0_enabled(dma_cd0, true);
    dma_channel_set_irq0_enabled(dma_cd1, true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr_audio);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_isr_audio);
    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_enabled(DMA_IRQ_1, true);

    printf("DMA set up done\n");
}

static uint32_t P4[256];   // 1byte→ 32bit: bit i を [4*i] へ展開
static uint8_t  REV8[256]; // 1byte→ bit並び反転

static inline void build_tables(void){
    for (int x = 0; x < 256; x++) {
        uint8_t r = 0; for (int i = 0; i < 8; i++) if (x & (1u<<i)) r |= 1u << (7 - i);
        REV8[x] = r;
        uint32_t m = 0; for (int i = 0; i < 8; i++) if (x & (1u<<i)) m |= 1u << (i * 4);
        P4[x] = m;
    }
}

#ifndef DSD_MSB_FIRST
#define DSD_MSB_FIRST 0   // 1=MSB-first, 0=LSB-first
#endif

static inline uint32_t __not_in_flash_func(pack_32bit)(uint8_t a, uint8_t b, uint8_t c, uint8_t d){
#if !DSD_MSB_FIRST
    a = REV8[a]; b = REV8[b]; c = REV8[c]; d = REV8[d];
#endif
    uint32_t Ap = P4[a];                // aP を [4*i+0]
    uint32_t Bp = P4[b];                // bP を [4*i+0]
    uint32_t Cp = P4[c];                // cP を [4*i+0]
    uint32_t Dp = P4[d];                // dP を [4*i+0]

    return  Ap | (Bp << 1) | (Cp << 2) | (Dp << 3);
}

// Core1: SD読み込み担当
void __not_in_flash_func(core1_entry)(void) {
    int idx = 0;
    while (true) {
        if (!buf_ready[idx]) {
            //absolute_time_t t0 = get_absolute_time();
            FRESULT fr = f_read(&fil, buf[idx], BUF_SIZE, &br[idx]);
            //absolute_time_t t1 = get_absolute_time();
            //printf("[READ] br=%u bytes, time=%d us\n", br[idx], (int)absolute_time_diff_us(t0,t1));
            if (fr != FR_OK || br[idx] == 0) {
                if (g_loop_enable) {
                    // ★ループ再生：ファイル先頭（純DSDの開始位置）へ戻して続行
                    f_lseek(&fil, g_frame_ofs);
                    // この idx にはまだ何も書いていないので、そのまま continue でOK
                    continue;
                } else {
                    buf_done[idx] = true;
                    g_core1_eof   = true;
                    return;
                }
            }

            uint32_t *dAB=packedAB[idx];
            uint32_t *dCD=packedCD[idx];
            br[idx] &= ~7u;           // 8byte境界に整列（安全）
            for(size_t i = 0; i < br[idx]; i += 8){         // 1loop = L,R 1byte
                *dAB++ = pack_32bit(buf[idx][i], buf[idx][i+1], buf[idx][i+2], buf[idx][i+3]);
                *dCD++ = pack_32bit(buf[idx][i+4], buf[idx][i+5], buf[idx][i+6], buf[idx][i+7]);
            }
            br_word[idx] = br[idx]/8;                // 32bit ワード数
            buf_ready[idx] = true;     // バッファ n が埋まった

            // 参照カウントを 2 にセット（AB と CD の両方がこの idx を消費する）
            use_cnt[idx] = 2;

            while (!q_push(q_ab, &qh_ab, &qt_ab, idx)) tight_loop_contents();
            while (!q_push(q_cd, &qh_cd, &qt_cd, idx)) tight_loop_contents();
            
            idx = idx + 1;
            if (idx >= NUM_BUF) idx = 0;
        }
        tight_loop_contents();
    }
}

// 簡易ヘルパ（キュー残量）
static inline uint8_t q_count(volatile uint8_t h, volatile uint8_t t){
    return (t - h) & (QDEPTH - 1);
}

// バッファ送り出し
static void playback_start_chain(void){
    // ★起動前に、各キューに最低2エントリ溜まるまで待つ
    while (q_count(qh_ab, qt_ab) < 4 || q_count(qh_cd, qt_cd) < 4) {
        tight_loop_contents();
    }

    int a0, a1, c0, c1;
    q_pop(q_ab, &qh_ab, &qt_ab, &a0);
    q_pop(q_ab, &qh_ab, &qt_ab, &a1);
    q_pop(q_cd, &qh_cd, &qt_cd, &c0);
    q_pop(q_cd, &qh_cd, &qt_cd, &c1);

    // ABペアに 0面/1面、CDペアに 0面/1面 を“あらかじめ”装填（ping-pong初期化）
    dma_channel_set_read_addr(dma_ab0, packedAB[a0], false);
    dma_channel_set_trans_count(dma_ab0, br_word[a0], false);
    dma_channel_set_read_addr(dma_ab1, packedAB[a1], false);
    dma_channel_set_trans_count(dma_ab1, br_word[a1], false);

    dma_channel_set_read_addr(dma_cd0, packedCD[c0], false);
    dma_channel_set_trans_count(dma_cd0, br_word[c0], false);
    dma_channel_set_read_addr(dma_cd1, packedCD[c1], false);
    dma_channel_set_trans_count(dma_cd1, br_word[c1], false);

    // 現在面の記録
    cur_ab0 = a0; cur_ab1 = a1;
    cur_cd0 = c0; cur_cd1 = c1;

    // 片側ずつ確実にスタート
    dma_channel_start(dma_ab0);
    dma_channel_start(dma_cd0);
}

static inline bool dma_pending_ch(int ch, uint32_t m0, uint32_t m1){
    return ch_is_dma1(ch) ? (m1 & ch_bit(ch)) != 0 : (m0 & ch_bit(ch)) != 0;
}

static void __not_in_flash_func(dma_isr_audio)(void){
    uint32_t m0 = dma_ctrl_for_ch(0)->ints0;
#ifdef DMA1_BASE
    uint32_t m1 = dma_ctrl_for_ch(16)->ints0;
#else
    uint32_t m1 = 0;
#endif

    if (dma_pending_ch(dma_ab0, m0, m1)) {
        clear_irq_for_ch(dma_ab0);
        isr_ab0++;

        // ★いま使い終わった“古い面”をデクリメント
        int finished = cur_ab0;
        if (finished >= 0 && use_cnt[finished] > 0 && --use_cnt[finished] == 0) {
            buf_ready[finished] = false;               // ここで初めて再利用可
            if (finished == 0) rel0++; else rel1++;
        }

        // ★次に使う面をキューから取得（無ければ同じ面を繰り返す：グリッチ回避）
        int next_idx;
        if (q_pop(q_ab, &qh_ab, &qt_ab, &next_idx)) {
            pop_ab++;
        } else {
            uf_ab++;
            next_idx = (finished >= 0) ? finished : 0; // フェイルセーフ
        }

        dma_channel_set_read_addr(dma_ab0, packedAB[next_idx], false);
        dma_channel_set_trans_count(dma_ab0, br_word[next_idx], false);
        cur_ab0 = next_idx;   // ★現在の面を更新
    }

    if (dma_pending_ch(dma_ab1, m0, m1)) {
        clear_irq_for_ch(dma_ab1);
        isr_ab1++;

        // ★いま使い終わった“古い面”をデクリメント
        int finished = cur_ab1;
        if (finished >= 0 && use_cnt[finished] > 0 && --use_cnt[finished] == 0) {
            buf_ready[finished] = false;               // ここで初めて再利用可
            if (finished == 0) rel0++; else rel1++;
        }

        // ★次に使う面をキューから取得（無ければ同じ面を繰り返す：グリッチ回避）
        int next_idx;
        if (q_pop(q_ab, &qh_ab, &qt_ab, &next_idx)) {
            pop_ab++;
        } else {
            uf_ab++;
            next_idx = (finished >= 0) ? finished : 0; // フェイルセーフ
        }

        dma_channel_set_read_addr(dma_ab1, packedAB[next_idx], false);
        dma_channel_set_trans_count(dma_ab1, br_word[next_idx], false);
        cur_ab1 = next_idx;   // ★現在の面を更新
    }

    if (dma_pending_ch(dma_cd0, m0, m1)) {
        clear_irq_for_ch(dma_cd0);
        isr_cd0++;

        // ★いま使い終わった“古い面”をデクリメント
        int finished = cur_cd0;
        if (finished >= 0 && use_cnt[finished] > 0 && --use_cnt[finished] == 0) {
            buf_ready[finished] = false;               // ここで初めて再利用可
            if (finished == 0) rel0++; else rel1++;
        }

        // ★次に使う面をキューから取得（無ければ同じ面を繰り返す：グリッチ回避）
        int next_idx;
        if (q_pop(q_cd, &qh_cd, &qt_cd, &next_idx)) {
            pop_cd++;
        } else {
            uf_cd++;
            next_idx = (finished >= 0) ? finished : 0; // フェイルセーフ
        }

        dma_channel_set_read_addr(dma_cd0, packedCD[next_idx], false);
        dma_channel_set_trans_count(dma_cd0, br_word[next_idx], false);
        cur_cd0 = next_idx;   // ★現在の面を更新
    }

    if (dma_pending_ch(dma_cd1, m0, m1)) {
        clear_irq_for_ch(dma_cd1);
        isr_cd1++;

        // ★いま使い終わった“古い面”をデクリメント
        int finished = cur_cd1;
        if (finished >= 0 && use_cnt[finished] > 0 && --use_cnt[finished] == 0) {
            buf_ready[finished] = false;               // ここで初めて再利用可
            if (finished == 0) rel0++; else rel1++;
        }

        // ★次に使う面をキューから取得（無ければ同じ面を繰り返す：グリッチ回避）
        int next_idx;
        if (q_pop(q_cd, &qh_cd, &qt_cd, &next_idx)) {
            pop_cd++;
        } else {
            uf_cd++;
            next_idx = (finished >= 0) ? finished : 0; // フェイルセーフ
        }

        dma_channel_set_read_addr(dma_cd1, packedCD[next_idx], false);
        dma_channel_set_trans_count(dma_cd1, br_word[next_idx], false);
        cur_cd1 = next_idx;   // ★現在の面を更新
    }
}

static bool stats_timer_cb(repeating_timer_t *t){
    static uint32_t p_uf_ab=0, p_uf_cd=0, p_pop_ab=0, p_pop_cd=0;
    static uint32_t p_isr_ab0=0, p_isr_ab1=0, p_isr_cd0=0, p_isr_cd1=0, p_rel0=0, p_rel1=0;

    uint32_t d_uf_ab  = uf_ab  - p_uf_ab;   p_uf_ab  = uf_ab;
    uint32_t d_uf_cd  = uf_cd  - p_uf_cd;   p_uf_cd  = uf_cd;
    uint32_t d_pop_ab = pop_ab - p_pop_ab;  p_pop_ab = pop_ab;
    uint32_t d_pop_cd = pop_cd - p_pop_cd;  p_pop_cd = pop_cd;
    uint32_t d_iab0   = isr_ab0- p_isr_ab0; p_isr_ab0= isr_ab0;
    uint32_t d_iab1   = isr_ab1- p_isr_ab1; p_isr_ab1= isr_ab1;
    uint32_t d_icd0   = isr_cd0- p_isr_cd0; p_isr_cd0= isr_cd0;
    uint32_t d_icd1   = isr_cd1- p_isr_cd1; p_isr_cd1= isr_cd1;
    uint32_t d_rel0   = rel0   - p_rel0;    p_rel0   = rel0;
    uint32_t d_rel1   = rel1   - p_rel1;    p_rel1   = rel1;

    printf("[STAT] AB: pop=%u uf=%u irq0=%u irq1=%u | CD: pop=%u uf=%u irq0=%u irq1=%u | rel: %u/%u\n",
           d_pop_ab, d_uf_ab, d_iab0, d_iab1,
           d_pop_cd, d_uf_cd, d_icd0, d_icd1,
           d_rel0, d_rel1);
    return true; // 継続
}

int main() {
    stdio_init_all();
    // バッファリング無効
    build_tables(); 
    setvbuf(stdout, NULL, _IONBF, 0);

    set_sys_clock_khz(180000, true);

    // 2. clk_peri を clk_sys にリンクし直す
    uint32_t sys_hz = clock_get_hz(clk_sys);
    clock_configure(clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        sys_hz,   // 入力
        sys_hz);  // 出力

    printf("sys=%u, peri=%u\n", clock_get_hz(clk_sys), clock_get_hz(clk_peri));

    // DTR が上がるまでブロック
    while (!stdio_usb_connected()) tight_loop_contents();

    printf("sys=%u Hz\n", clock_get_hz(clk_sys));  // 動作確認用

    printf("USB ready - WSD player start\n");

    gpio_init(5); gpio_set_dir(5, GPIO_OUT); gpio_put(5, 1);  // CS = GP5
    sleep_us(10);                                             // 安定待ち

    //my_spi_init(sd_get_by_num(0)->spi_if_p->spi); // FatFS 内部でも呼ばれるが安全のため
    spi_flush(spi0);

    FATFS fs;
    FRESULT fr;

    // SDカード初期化とマウントは hw_config.c 側で行われる想定
    printf("mount...\n");

    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("Mount failed: %d\n", fr);
        return -1;
    }
    printf("mount done\n");

    sd_card_t *card = sd_get_by_num(0);

    spi_inst_t *sd_spi_hw = card->spi_if_p->spi->hw_inst;
    spi_set_baudrate(sd_spi_hw, 45000000);    // はじめは 25MHz 推奨（後で上げられます）
    printf("SPI baud = %u\n", spi_get_baudrate(sd_spi_hw));

    printf("open...\n");
    absolute_time_t t0 = get_absolute_time();
    do {
        fr = f_open(&fil, WSD_PATH, FA_READ);
    } while ( (fr == FR_NOT_READY || fr == FR_LOCKED) &&
            absolute_time_diff_us(t0, get_absolute_time()) < 1000000); // 1 s

    printf("f_open=%d\n", fr);        // ← ここが出れば USB は生きている
    if (fr) return -1;

    uint32_t frame_ofs = 0;         // フレーム（純 DSD）の開始位置
    UINT     br_tmp;

    uint32_t le = 0, be = 0;
    /* 0x3C を読む */
    f_lseek(&fil, 0x3C);
    f_read (&fil, &le, 4, &br_tmp);
    be = __builtin_bswap32(le);

    uint32_t fsize = f_size(&fil);
    if (le >= 0x100 && le < fsize)          frame_ofs = le;   // LE 正常
    else if (be >= 0x100 && be < fsize)     frame_ofs = be;   // BE 正常
    else                                    frame_ofs = 0x400;/* フォールバック */
    printf("use frame_ofs=0x%X\n", frame_ofs);
    g_frame_ofs = frame_ofs;          // ★Core1が参照できるように共有
    f_lseek(&fil, frame_ofs);   // ← ここから先を再生

    // PIOセットアップ＆Core1起動
    pio_setup_dual();
    dma_setup_chain();

    repeating_timer_t stats_timer;
    add_repeating_timer_ms(10000, stats_timer_cb, NULL, &stats_timer);

    multicore_launch_core1(core1_entry);

    playback_start_chain();

    #if LOOP_PLAY
    // ループ再生時は EOF を待たず、以後は何もせず回しっぱなし
    for(;;) { tight_loop_contents(); }
    #else
    while (!g_core1_eof) { tight_loop_contents(); }

    // （ここから下は一回再生で終了したいときだけ有効）
    cancel_repeating_timer(&stats_timer);

    // DMA IRQ を無効化（有効化は先に行っています）
    irq_set_enabled(DMA_IRQ_0, false);
    irq_set_enabled(DMA_IRQ_1, false);

    // 連鎖DMAを止める
    dma_channel_abort(dma_ab0);
    dma_channel_abort(dma_ab1);
    dma_channel_abort(dma_cd0);
    dma_channel_abort(dma_cd1);

    // PIO を停止
    pio_sm_set_enabled(pio, sm_ab, false);
    pio_sm_set_enabled(pio, sm_cd, false);

    // SD ファイルを閉じて終了
    f_close(&fil);
    printf("Playback done\n");

    return 0;
    #endif
}