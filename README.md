# 99L Communication Board

ESP32-S3上でMission BoardのClassic CANを受信し、最新の`Natsu-B/Vault` 99L仕様に従うcompact LoRa packetへ中継する通信基板です。GNSS、通信基板microSD、Recovery中継もこのrepositoryが所有します。

## Architecture

- `src/can/protocol.rs`: 11-bit standard CAN codec。125 kbit/s、little-endian、DLCとreserved bitを検証します。
- `src/can/cache.rs`: CAN IDごとのlatest value、受信時刻、freshness、MissionEvent OR latchを保持します。
- `src/can/command.rs`: pending最大16件とresult cache 16件を管理し、duplicate replayと同一ID異payload拒否を行います。
- `src/can/recovery.rs`: Recovery command lifecycle、6-byte CAN fragmentの16-byte A6結合、sequence gapとresume offsetを管理します。
- `src/tasks/can_communication.rs`: TWAI唯一owner。RX、raw CAN logging、優先TX、bus-off recoveryを行います。
- `src/tasks/lora_task.rs`: LoRa UART RX/TX唯一owner。uplink dispatchとA0〜A6/B0/B1生成を行います。
- `src/tasks/gnss_task.rs`: GNSS UART/enable唯一owner。receiver、configuration、fix、invalid、staleを区別します。
- `src/tasks/sd_task.rs`: SD SPI唯一owner。受信したraw CANを`CAN.CSV`へ記録します。

CAN、LoRa、GNSS、SDはowner task以外から直接操作せず、bounded channelで要求を渡します。緊急uplinkは通常command queueを経由せず、専用CAN safety channelへ渡します。

## Build / flash / run

ESP Rust toolchainとXtensa GCCが必要です。

```sh
cargo fmt --all -- --check
cargo check
cargo test
cargo clippy --all-targets -- -D warnings

PATH=/path/to/xtensa-esp-elf/bin:$PATH \
  cargo build --release --features firmware \
  --target xtensa-esp32s3-none-elf -Z build-std=core
```

接続先をVID/PID/serialで確認してから、生成したELFを`espflash`でESP32-S3へ書き込みます。port番号を推測して指定しないでください。

```sh
espflash flash --chip esp32s3 --port /dev/ttyACM<N> \
  target/xtensa-esp32s3-none-elf/release/c99l_comboard
```

本作業環境ではユーザー確認により`/dev/ttyACM3`がComBoardですが、基板側の問題が発生したため、今回のflash/boot/実機通信試験は意図的に実施していません。

## Protocol

CANは11-bit standard ID、125 kbit/s、payload 8 byte以下です。対応IDは`0x001`、`0x002`、`0x008`、`0x010`〜`0x013`、`0x020`、`0x100`〜`0x109`です。Mission由来raw量子化値は再量子化せず、LoRa packet組立時にcache snapshotを使用します。

freshness初期値は100 Hz系30 ms、LPS 120 ms、10 Hz系300 ms、GNSS 3000 msです。stale値は04aのreserved rawへ置換します。MissionEventはsequence duplicateを抑止し、A1〜A3 Flight packetへ実際に載せたbitだけ送信成功後にclearします。

LoRa downlinkはA0 22 byte、A1〜A3 24 byte、A4 15 byte、A5 12 byte、A6 24 byte、B0 10 byte、B1 3 byteです。E220固定送信prefixは`00 00 04`、bit packingはLSB-first、XORはheaderからpaddingまでです。A1〜A4は約0.5秒、A5 RecoveryBeaconは約10秒、A6は最大5 Hzです。

uplinkは固定11 byteです。

```text
55 kind transaction_id command args[6] xor
```

transaction ID 0、checksum不一致、kind不明、Emergencyの非zero予約fieldは拒否します。Generic commandはCAN送信成功後にのみStartSequence連動のSD/GNSS副作用を開始します。CAN送信失敗結果もcacheし、同一requestへreplayします。

Recovery A6はtransfer IDとsequenceを検証します。gap、encode失敗、LoRa queue overflow時は結合を停止し、確認できたresume offsetをB0 detailへ返し、MissionへStopLogDumpを試行します。

## Hardware assumptions

- ESP32-S3、TWAI GPIO7/16、LoRa UART2 GPIO11/12、AUX GPIO8。
- GNSS UART1 GPIO14/21、enable GPIO13。9600 baudで設定後115200 baudへ切替。
- SD SPI GPIO41/42/40、CS GPIO2。既存PHY、pin、LED、task配置は変更していません。
- E220の固定address/channel、RSSI付加、AUX timeoutは実機readbackが必要です。

## Test

`testdata/99l_protocol_golden_vectors.txt`はMission/Groundと同一です。host testは全CAN field、LoRa A0〜A6/B0/B1、uplink、reserved/error raw、freshness、MissionEvent、transaction replay、Recovery gap/partial/EOF、GNSS rangeを検証します。

```sh
cargo fmt --all -- --check
cargo check
cargo test
cargo clippy --all-targets -- -D warnings
```

GNSS設定応答専用firmwareは次で型検査できますが、実機実行はhardware復旧後に行います。

```sh
cargo check --test gnss_setting_response --features hardware-test \
  --target xtensa-esp32s3-none-elf -Z build-std=core
```

## Known limitations

- ComBoard側の問題により、今回のflash/boot、Mission↔Com CAN、Com↔Ground LoRa、GNSS receiver ACK/fix/stale、SD書込みの実機検証は未実施です。
- GNSS設定は各UBX commandのACK/NAKを検証しますが、最後のbaud変更commandはbaud切替を伴うためACK未確認です。
- GGAから位置/fixを取得します。NMEA日付をUnix時刻へ変換するperiodic GNSS `TimeSync`は未実装で、MissionのTimeRequestはGround B1/B0経路を使用します。
- A0/A6/B0/B1 layout、A0 status割当、local command code、freshness、Recovery rateはVaultの実装仮定台帳に記録した暫定値です。
- A6はRAM上の16-byte queueへspoolします。通信基板microSDへの全Recovery dump spoolと自動resume requestは未実装です。
- 実機の100 Hz CAN負荷、Emergency end-to-end latency、E220 packet loss、GNSS屋外fixはhardware復旧後に測定が必要です。
