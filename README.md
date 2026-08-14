# 99L Communication Board

ESP32-S3上でMission BoardのClassic CANを受信し、最新の`Natsu-B/Vault` 99L仕様に従うcompact LoRa packetへ中継する通信基板です。GNSS、通信基板microSD、Recovery中継もこのrepositoryが所有します。

## Architecture

- `src/can/protocol.rs`: 11-bit standard CAN codec。125 kbit/s、little-endian、DLCとreserved bitを検証します。
- `src/can/cache.rs`: CAN IDごとのlatest value、受信時刻、freshness、MissionEvent OR latchを保持します。
- `src/can/command.rs`: pending最大16件とresult cache 16件を管理し、duplicate replayと同一ID異payload拒否を行います。
- `src/can/recovery.rs`: Recovery command lifecycle、6-byte CAN fragmentの16-byte A6結合、sequence gapとresume offsetを管理します。
- `src/lora_scheduler.rs`: 500 ms absolute deadline、送信source優先度、B1 queue policy、Recovery fairnessとmissed slot処理を定義します。
- `src/tasks/can_communication.rs`: TWAI唯一owner。RX、raw CAN logging、優先TX、bus-off recoveryを行います。
- `src/tasks/lora_task.rs`: LoRa UART RX/TX唯一owner。uplink dispatch、A0〜A6/B0/B1生成、AUX Low→High完了確認とRX activity guardを行います。
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

LoRa timingの実機診断では`lora-timing-debug`を追加します。10送信ごとにsource別件数、request/queue/AUX/UART/物理送信/idle、missed slotとtimestamp異常を`LORA_TIMING`へ出力します。

```sh
PATH=/path/to/xtensa-esp-elf/bin:$PATH \
  cargo build --release --features firmware,lora-timing-debug \
  --target xtensa-esp32s3-none-elf -Z build-std=core
```

接続先をVID/PID/serialで確認してから、生成したELFを`espflash`でESP32-S3へ書き込みます。port番号を推測して指定しないでください。

```sh
espflash flash --chip esp32s3 --port /dev/ttyACM<N> \
  target/xtensa-esp32s3-none-elf/release/c99l_comboard
```

2026-08-14の3基板実機試験では`/dev/ttyACM0`を使用しました。portは環境ごとに変わるため、通常はUSB identityを照合してから指定してください。実測結果は[docs/hardware_test_results.md](docs/hardware_test_results.md)に記録しています。

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

LoRa TXは直前送信からの相対待機ではなく、500 msのabsolute deadlineを維持します。期限超過slotはburstで追送せずmissedとして進めます。送信優先度は`EmergencyResult > CommandResult(B0) > GroundTimeRequest(B1) > Recovery(A6) > periodic(A0〜A5)`です。B1は専用bounded queueを使い、同一request IDの連続duplicateを抑止し、満杯時はoldestをdropしてcounterへ記録します。

送信前はAUX Highを確認し、最後のLoRa RX activityから60 msを確保してAUX Highを再確認します。UART flush後は15 ms以内のAUX Low開始と、その後のHigh復帰を物理送信完了条件にします。旧`fixed350` post-TX waitは使いません。実測値と判定は[docs/hardware_test_results.md](docs/hardware_test_results.md#lora-absolute-scheduler追試2026-08-14)に記録しています。

## Hardware assumptions

- ESP32-S3、TWAI GPIO7/16、LoRa UART2 GPIO11/12、AUX GPIO8。
- GNSS UART1 GPIO14/21、enable GPIO13。9600 baudで設定後115200 baudへ切替。
- SD SPI GPIO41/42/40、CS GPIO2。既存PHY、pin、LED、task配置は変更していません。
- Ground/ComBoard双方のE220 register readbackは`C1 00 08 00 00 EC 81 04 C3 00 00`で一致しました。搭載moduleの正確な型番を一次資料で確定できていないため、fieldの意味はこのrepositoryでは推測しません。

## Test

`testdata/99l_protocol_golden_vectors.txt`はMission/Groundと同一です。host testは全CAN field、LoRa A0〜A6/B0/B1、uplink、reserved/error raw、freshness、MissionEvent、transaction replay、Recovery gap/partial/EOF、GNSS rangeを検証します。

```sh
cargo fmt --all -- --check
cargo check
cargo test
cargo clippy --all-targets -- -D warnings
```

GNSS設定応答、E220 readback、microSD read-only確認用firmwareは次で型検査できます。Mission Board用firmwareではないため、ComBoard以外へflashしないでください。

```sh
cargo check --test gnss_setting_response --features hardware-test \
  --target xtensa-esp32s3-none-elf -Z build-std=core
cargo check --test e220_readback --features hardware-test \
  --target xtensa-esp32s3-none-elf -Z build-std=core
cargo check --test sd_log_verify --features hardware-test \
  --target xtensa-esp32s3-none-elf -Z build-std=core
```

## Known limitations

- microSDは同一実機で初期化成功後、後続試験では`TimeoutCommand(41)`が継続しました。loggingと`CAN.CSV`実データ確認はBLOCKEDで、card/hardware/初期化条件の切り分けが残っています。
- GNSS設定は各UBX commandのACK/NAKを検証しますが、最後のbaud変更commandはbaud切替を伴うためACK未確認です。
- GGAから位置/fixを取得します。NMEA日付をUnix時刻へ変換するperiodic GNSS `TimeSync`は未実装で、MissionのTimeRequestはB1とGroundTimeResponse uplinkの経路を使用します。
- A0/A6/B0/B1 layout、A0 status割当、local command code、freshness、Recovery rateはVaultの実装仮定台帳に記録した暫定値です。
- A6はRAM上の16-byte queueへspoolします。通信基板microSDへの全Recovery dump spoolと自動resume requestは未実装です。
- GNSS receiver/configurationと屋内NO_FIX/OFF stale-clearは実機確認済みです。屋外fixは未評価です。
- 3基板でCAN 100 Hz系とLoRa 130秒連続通信を確認済みです。長時間・離距離・RF干渉・電源変動を含む耐久試験は別途必要です。
- Emergency resultはgeneric transaction trackerに登録せず安全優先CAN queueで送ります。F0/F1はCAN ownerをblockしない専用priority LoRa queueへ送り、正常resultは実機PASS、CAN TX失敗時の`Failed/Timeout` B0はbuild/test PASSです。Mission非接続の実機失敗経路は未評価です。
- **PASS**: ComBoardのabsolute scheduler、B1専用queue、AUX-RX guardはA0-onlyとA0+B1連続試験で確認済みです。Groundの安全boundary/AUX hardeningを含む最終artifactでは、0〜450 msの100位相試験が100/100成功し、`g 0x7F`/`le`の最大遅延は968.323/968.250 msでした。production復帰後は69.501秒で140 packet、全packet間隔平均500.007 ms、`g 0x7F`/`le`は909.386/909.339 msでfinal B0を返しました。CAN/LoRa/AUX/queue drop errorは0、診断logはproductionに残っていません。
- **PARTIAL**: 全source同時競合のpriorityとB1満杯時oldest dropはhost testのみで、実機stressは未実施です。
