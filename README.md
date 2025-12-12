# 8ch_1bitPlayer_Pico2W

Raspberry Pi Pico 2 W (RP2350) で  
microSD 上の 8ch 1bit オーディオを再生するためのプレイヤー。

- SD カードから 1bit ストリームを読み出し
- PIO + DMA + マルチコアで 8ch 分の 1bit 信号を連続出力
- FatFs + no-OS-FatFS-SD-SDIO-SPI-RPi-Pico を使用した SPI モード SD ドライブ

研究用の検証コードであり、API や仕様は今後変更される可能性がある。

---

## 機能概要

- サンプリングレート 3 MHz の 1bit（DSD 相当）ストリームを再生
- 8ch 出力（4ch×2 SM）  
  - SM_AB: GPIO 14–17  
  - SM_CD: GPIO 18–21
- microSD（SPI 接続, FAT/FAT32）からストリームを連続読み出し
- core0: PIO + DMA / ログ出力  
  core1: SD 読み出し & パッキング
- 4 バッファ（各 32 KiB）× リングキューでバッファアンダーフローを回避
- DMA 4 チャネル（AB 用 2本 + CD 用 2本）を相互にチェインして連続転送
- 再生ファイルは `DayByDay_8ch.wsd` 固定（現状）  

---

## ハードウェア構成

### 必要なもの

- Raspberry Pi Pico 2 W
- microSD カードスロット（SPI 接続のブレイクアウト基板など）
- 8ch 1bit 出力を受けるドライバ回路
  - CMOS ドライバ、整合抵抗 など

### SD カード用ピン配置（SPI0）

`hw_config.c` に定義されている設定。

- SCK : GPIO 2  
- MOSI: GPIO 3  
- MISO: GPIO 4  
- CS  : GPIO 1  

SPI0 を 200 kHz 付近で初期化したあと、マウント完了後に 45 MHz まで引き上げる実装になっている。  

### 1bit 出力ピン

`main.c` の定義。

- CH A–D: GPIO 14, 15, 16, 17（SM_AB）
- CH E–H: GPIO 18, 19, 20, 21（SM_CD）

PIO から 32bit ワードを送り出し、その下位 4bit をそれぞれ A/B/C/D（E/F/G/H）に割り当てる形で 8ch をパラレル出力する。

---

## ソフトウェア構成

### 主要ファイル

- `main.c`  
  - クロック設定（sys 180 MHz に OC）  
  - USB stdio 初期化 & DTR 待ち  
  - FatFs による SD マウント  
  - WSD ヘッダからフレーム開始位置（DSD 本体）を取得  
  - PIO / DMA セットアップ  
  - core1 を起動し、再生を開始  
  - 10 秒ごとに DMA / バッファ統計を USB に出力

- `hw_config.c`  
  - no-OS-FatFS-SD-SDIO-SPI-RPi-Pico 用の SPI & SD カード設定  
  - SPI0、SCK/MOSI/MISO/CS の GPIO 番号と初期ボーレートの定義

- `wsd_player.pio`  
  - 1bit ストリームを 8ch パラレルで出力する PIO プログラム  
  - `pico_generate_pio_header` でヘッダに変換され、`main.c` から利用される

- `CMakeLists.txt`  
  - `pico2_w` ボード指定  
  - 依存ライブラリ（pico_stdlib, pico_multicore, hardware_pio, hardware_spi, hardware_clocks, no-OS-FatFS-SD-SDIO-SPI-RPi-Pico）  
  - USB stdio 有効化（UART は無効）

- `src/`  
  - no-OS-FatFS-SD-SDIO-SPI-RPi-Pico ライブラリ一式（SD/FatFs ドライバ）

---

## WSD ファイルの扱い

- 再生ファイル名は `main.c` 内の `WSD_PATH` マクロで決める（デフォルト: `DayByDay_8ch.wsd`）。
- FatFs でルート直下のファイルを開き、WSD ヘッダ中のオフセットフィールド（0x3C の 32bit 値, LE/BE 両対応）を見て DSD フレーム開始位置を決める。  
  - オフセット値が異常な場合はフォールバックとして `0x400` から再生を開始する。
- 以降の純 1bit データを 32 KiB 単位で読み出し、  
  8 byte ごと（8ch×1byte）に 32bit ワードへパックし直して PIO に送る。

WSD フォーマット Ver.2.0 を前提にしているが、現状は「ヘッダからフレーム先頭位置が分かれば OK」という最小限の実装になっている。

---

## ビルド手順

前提:

- pico-sdk がインストール済みで `PICO_SDK_PATH` が通っていること
- ARM GCC (`arm-none-eabi-gcc`) がインストール済みで PATH に通っていること
- CMake / Ninja などのビルドツールが利用可能であること

```sh
git clone https://github.com/AcoustOikawalab/8ch_1bitPlayer_Pico2W.git
cd 8ch_1bitPlayer_Pico2W

mkdir build
cd build

cmake .. -DPICO_BOARD=pico2_w
cmake --build . --config Release
