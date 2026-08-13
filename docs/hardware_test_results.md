# 3基板実機試験結果（2026-08-14）

## 条件

- ComBoard base HEAD: `6b44a62a49cdc96c599b55fbe8c367fec6946604`（本試験修正を含むfirmwareで評価）
- ComBoard: `/dev/ttyACM0`、ESP32-S3 rev 0.2、MAC `B8:F8:62:C5:FA:7C`
- Mission: `/dev/ttyACM1`
- Ground: `/dev/ttyUSB0`
- ComBoard boot logのESP-IDF: `v5.4.1-426-g3ad36321ea`
- raw log: `/tmp/99l_hwtest_20260814_G8Eh5f/`

actuator motion、motor arm、parachute駆動、STS motion、flight enableは実行していません。

## 結果

| 対象 | 判定 | 実測 |
|---|---|---|
| ComBoard production boot | PASS | panic、watchdog、reset loopなし。CAN/LoRa/GNSS/SD owner task起動を確認（SD結果は別行） |
| Mission→ComBoard CAN | PASS | Mission修正後30秒で受信error 0、TEC/REC 0。下表の周期frameを継続受信 |
| ComBoard→Ground LoRa | PASS | 130秒で203 packet、decode/XOR error 0、queue drop 0、AUX timeout 0 |
| Ground→Mission command roundtrip | PASS | unknown、Cancel、liftoff emergency、recovery error path、actuator emergencyの安全側経路が期待resultと一致 |
| TimeRequest/response | PASS | B1 request ID 2へのGroundTimeResponseをLoRa→CAN `0x013`へ転送後、MissionのTimeRequest停止を確認 |
| GNSS receiver/configuration | PASS | MAX-M10Sを検出し、productionで8 command ACK。NMEA/GGAを115200 baudで受信 |
| GNSS satellite fix | NO_FIX | 屋内GGAはfix 0。receiver/interface failureとは判定しない |
| GNSS OFF/old sample clear | PASS | OFF後にreserved unavailableへ戻り、過去のNO_FIXを再利用しなかった |
| GNSS stale timeout | NOT_EVALUATED | receiver ONのままNMEAを途絶させる試験は未実施 |
| E220 raw readback | PASS | Ground/ComBoardとも`C1 00 08 00 00 EC 81 04 C3 00 00` |
| E220 field解釈 | PARTIAL | 両moduleのraw一致まで確認。正確な型番を一次資料で確定していないためfield意味は未評価 |
| microSD interface | PARTIAL | 初期試験では初期化・31,266,439,168 byte・1 MHz切替に成功。その後は`TimeoutCommand(41)`が継続 |
| microSD logging / `CAN.CSV` | BLOCKED | Start/Stop command resultはCompletedだがSD非active。read-only verifierもACMD41でvolumeを開けず、row内容は未評価 |
| GNSS periodic TimeSync | NOT_IMPLEMENTED | GroundのB1応答経路とは別の、GNSS由来periodic TimeSyncは未実装 |
| Recovery dump本体 | NOT_IMPLEMENTED | production reader未接続。安全なSourceUnavailable/InvalidState error pathのみ確認 |

### CAN ID別計測

Mission側の周期送信欠落修正前は、30秒時点で`0x103/0x107/0x109`が0件、`0x100/0x102/0x108`にもsequence gapがありました。修正後の同じhardwareでの30秒値は次のとおりです。

| CAN ID | count | rate | sequence gap | 判定 |
|---|---:|---:|---:|---|
| `0x012` TimeRequest | 30 | 1 Hz | 対象外 | PASS |
| `0x020` MissionEvent | 1 | event | 0 | PASS |
| `0x100` Kinematics | 3000 | 100 Hz | 0 | PASS |
| `0x101` Control | 0 | state依存 | 0 | NOT_EXERCISED |
| `0x102` MissionStatus | 301 | 約10 Hz | 0 | PASS |
| `0x103` PowerTime | 300 | 10 Hz | 0 | PASS |
| `0x104` DescentCore | 0 | state依存 | 0 | NOT_EXERCISED |
| `0x105` RecoveryStatus | 0 | event | 対象外 | NOT_EXERCISED |
| `0x106` RecoveryLogData | 0 | 未実装 | 0 | NOT_IMPLEMENTED |
| `0x107` AttitudeTilt | 300 | 10 Hz | 0 | PASS |
| `0x108` LPS | 750 | 25 Hz | 0 | PASS |
| `0x109` Airspeed | 3000 | 100 Hz | 0 | PASS |

SSC差圧計は未接続です。`0x109`は停止や0 m/s化をせず、`ssc_not_initialized`のreserved rawを送信しGroundまで伝搬しました。

### LoRa連続通信

130秒captureのheader内訳はA0 116、B1 84、B0 3です。packet間隔はmin 0.598秒、avg 0.642秒、max 1.320秒で、1秒超は1回、5秒超は0回でした。RSSIは203/203 packetで`-107 dBm`、ComBoardのLoRa TX/RX error、AUX timeout、TX queue drop、command dropはいずれも0でした。

最終productionの3基板同時captureは65秒で、A0 38、B1 63、B0 2の計103 packet、host間隔avg 0.631秒/max 1.219秒、1秒超gap 1、5秒超gap 0、RSSI missing 0でした。CAN RX/TX error、TEC/REC、LoRa error/AUX timeout/queue dropは0、`g 0x7F`と`le`はどちらもuplink送信後0.360秒で期待の終端B0を返し、panic/reset loopとEmergency未照合warningはありません。rawは`/tmp/99l_hwtest_20260814_G8Eh5f/final_production_65s_v5/`です。

当初、ComBoardがA0/B1を連続送信してE220のhalf-duplex受信時間を塞ぎ、Ground uplinkを受信できませんでした。`lora_tx_task`が送信成功後に350 msの受信windowを確保し、Groundがdownlink直後にuplinkを送る同期を加えた後、双方向通信が安定しました。

### 安全command

latencyはGroundが`uplink sent`を表示してからfinal B0を受信するまでです。

| command | final result | latency | 判定 |
|---|---|---:|---|
| `g 0x7F` | Rejected / NotSupported | 0.959 s | PASS |
| `g 0x02` | Rejected / InvalidState | 0.361 s | PASS |
| `le` | Rejected / InvalidState | 0.961 s | PASS |
| `local 0x66` | Accepted → Failed / DeviceUnavailable | 0.982 s | PASS |
| `local 0x73` | Accepted → Failed / DeviceUnavailable | 0.982 s | PASS |
| `local 0x78` | Accepted → Rejected / InvalidState | 1.582 s | PASS |
| `ae` | Completed / None | 0.960 s | PASS |

`ae`はmotor coast、Para power OFFの安全状態を確認して最後に実行し、Missionをresetしてproductionへ戻しました。transaction IDはすべて非zeroで、final result後に次IDへ進み、観測範囲でpending leakはありませんでした。

## 今回の変更

- `src/tasks/lora_task.rs` / `LORA_POST_TX_RX_WINDOW_MS`: E220送信後に受信windowを設け、連続downlinkによるuplink starvationを解消。
- `src/tasks/gnss_task.rs`: 起動待ち中もNMEAを読みFIFO overflowを防止し、OFF後のqueued sentenceがstale値を復活させないようにした。
- `src/tasks/gnss_task.rs`: GGA parse中のOFF/再ON raceに対し、書込みlock内でもstateを再確認し、`Starting`中の古いsentenceを拒否する。
- `src/gnss/settings.rs`: 実測約620 msの起動直後ACKに合わせ、有限timeoutを1秒へ変更。
- `src/tasks/sd_task.rs`: StopLogging後にactive stateとLEDを確実に解除。
- `src/tasks/can_communication.rs`、`src/bin/main.rs`、`src/state.rs`: CAN ID/sequenceとCAN・LoRa・SD・GNSSの実測counterを追加。
- `src/tasks/can_communication.rs` / `EMERGENCY_RESULT_LORA_CHANNEL`: generic tracker対象外のF0/F1過剰warningを止め、CAN ownerをblockしない専用priority queueで正常/失敗resultを送る。修正後Liftoff Emergencyは実機PASS。
- `test/e220_readback.rs`: registerを書かない`C1 00 08` readback test。
- `test/sd_log_verify.rs`: `CAN.CSV`をtruncate/create/appendしないread-only verifier。

## 残件

- microSDのACMD41 timeoutをcard、電源、CS/SPI条件で切り分け、logging後にread-only verifierで`CAN.CSV`のheader/row/DLC/時刻/CAN IDを確認する。
- MAX-M10Sの屋外ValidFixとstale遷移を確認する。
- E220の正確な型番をschematic/BOMで確定し、対応するEBYTE一次資料でreadback fieldを評価する。
- RF距離、干渉、電源変動を含む長時間試験を行う。
