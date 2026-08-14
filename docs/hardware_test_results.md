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

当初、ComBoardがA0/B1を連続送信してE220のhalf-duplex受信時間を塞ぎ、Ground uplinkを受信できませんでした。旧`fixed350`とGroundのdownlink直後送信を組み合わせた試験では双方向通信が成立しましたが、これはその位相での結果です。後続のAUX timing計測とphase sweepで、旧実装は物理送信完了を正しく判定せず、任意位相のuplinkを保証しないことが分かりました。詳細は次節です。

## LoRa absolute scheduler追試（2026-08-14）

### 条件

- ComBoard source: base `0fa134348ab75b295cd32c9b3f409c83be2600e5`にabsolute scheduler、source別queue、AUX-RX guard、timing計測のworktree変更を適用。
- Mission source: `abca85c19691d449004dbb608a73989b59af7099`のproduction。試験後もproductionへ復帰。
- raw log: `/tmp/99l_lora_scheduler_20260814T141137/`。
- `baseline_fixed350_timing_70s`だけは旧`fixed350`へ診断用AUX Low観測を追加した比較firmware。`final_a0_only_125s`と`final_a0_b1_125s`はabsolute scheduler firmware。
- `lora-timing-debug`は10送信単位で計測値を出すため、Ground packet数とreport対象送信数が1件ずれるcaptureがあります。

actuator motion、motor arm、parachute駆動、STS motion、flight enableは実行していません。

### 判定

| 対象 | 判定 | 実測 |
|---|---|---|
| `fixed350` root cause | PASS | UART flush後、AUXは1〜3 us（110送信のreport平均2.091 us）だけHighのまま残ってからLowへ遷移した。旧productionの即時`wait_for_aux_high`はこのHighを送信完了と誤認し、350 ms timerを物理送信前から開始していた |
| 500 ms absolute scheduler / A0-only | PASS | 250 timing sampleのwrite開始間隔平均499.999680 ms、missed slot 0。Groundは124.658808 sでA0 251 packet、1秒超gap 0 |
| A0+B1 scheduling / B1通常queue | PASS | 124.256532 sでA0 125、B1 125。B1 request ID 12〜136は連続し、B1 duplicate/drop 0、LoRa queue drop 0 |
| source priority / overflow policy | PARTIAL | `EmergencyResult > CommandResult(B0) > GroundTimeRequest(B1) > Recovery(A6) > periodic`、B1 duplicate抑止と満杯時oldest drop、pre-write再選択はhost test PASS。全source同時競合とB1満杯は実機未実施 |
| AUX Low→High完了 / RX activity guard | PASS | final 500送信でAUX Low観測500/500、未観測0、AUX timeout 0。RX byte受信時から60 ms以内はTXを延期し、延期後にAUX Highを再確認する |
| v4 arbitrary-phase command | PARTIAL | phase 0 msは`g 0x7F` 5/5、`le` 5/5 PASS。phase 50/100/150 msは各0/10、phase 200 msは0/2で、計10/42 PASS。失敗32件はfinal B0 timeout |
| Ground downlink-boundary alignment | PASS | 0〜450 msを50 ms刻み、各phaseで`g 0x7F`/`le`を各5回、計100/100 roundtrip PASS。timeout、release失敗、late final、fallbackは0 |

### `fixed350` root causeと比較baseline

旧productionはUART flush直後にAUXがHighなら送信完了としていました。診断buildでLow edgeを観測すると全110送信でflush後1〜3 usにLowへ入り、その後A0は平均315.757 ms、B1は平均251.985 msを要してHighへ戻りました。したがって旧productionはLow開始前のHighを拾い、`fixed350`を実際のE220送信と重ねていました。350 msの受信windowを物理送信完了後に保証した実装ではありません。

`baseline_fixed350_timing_70s`のcapture spanは69.605719 s、Ground受信はA0 41、B1 70の計111 packetです。全packet間隔はmin 0.160999 / avg 0.632779 / p50 0.602740 / p95 0.669900 / p99 0.720598 / max 1.256313 s、1秒超gap 1、5秒超gap 0でした。完全なtiming report 110送信ではA0 41、B1 69、write開始間隔平均630.716 ms、物理完了後idle平均355.564 ms、AUX Low未観測0、missed slot 0、timestamp異常0です。

この比較captureは診断用Low観測が旧productionの挙動を変え、CAN RX overrunもHWSTAT差分で57件あったため、scheduler acceptanceには使用せず`fixed350`のroot-cause計測に限定します。

### final A0-only

`final_a0_only_125s`はcapture span 124.658808 sです。

| 計測 | 実測 |
|---|---:|
| Ground packet | A0 251 |
| Ground A0間隔 | count 250、min 0.155903、avg 0.498635、p50 0.500025、p95 0.501228、p99 0.502105、max 0.519931 s |
| ComBoard timing sample | periodic 250 |
| request間隔 / write開始間隔 / complete間隔 | avg 499.999600 / 499.999680 / 499.996920 ms |
| A0 TX total | min 314.196、report平均315.461、max 338.164 ms |
| idle gap | report平均184.638 ms |
| AUX Low / missed / invalid timestamp | 250/250 / 0 / 0 |

HWSTAT有効区間110.535892 sの差分はCAN RX +28,182、LoRa TX +221です。CAN RX/TX error、LoRa RX/TX error、AUX timeout、全LoRa queue dropは0で、analyzer faultも0でした。先頭の短いGround間隔を含むpacket統計と、board内absolute deadline統計は混同せず併記しています。

### final A0+B1

`final_a0_b1_125s`はcapture span 124.256532 sです。

| 計測 | 実測 |
|---|---:|
| Ground packet | A0 125、B1 125、計250 |
| 全packet間隔 | count 249、min 0.229507、avg 0.499022、p50 0.255534、p95 0.750761、p99 0.751806、max 0.771681 s |
| A0→B1 | count 125、min 0.229507、avg 0.250170、p50 0.250487、p95 0.252274、p99 0.255187、max 0.255534 s |
| B1→A0 | count 124、min 0.743748、avg 0.749881、p50 0.749593、p95 0.751567、p99 0.769572、max 0.771681 s |
| A0周期 | count 124、min 0.979615、avg 1.000058、p50 1.000077、p95 1.001229、p99 1.018887、max 1.020040 s |
| B1周期 | count 124、min 0.994106、avg 1.000044、p50 1.000078、p95 1.002066、p99 1.005772、max 1.007083 s |
| ComBoard timing sample | periodic 125、B1 125 |
| write開始間隔 / complete間隔 | report平均499.993680 / 499.992840 ms |
| A0 / B1 TX total | report平均315.558 / 252.062 ms |
| idle gap | report平均216.254 ms |
| AUX Low / invalid timestamp | 250/250 / 0 |

HWSTAT有効区間110.545799 sの差分はCAN RX +28,294、LoRa TX +222、periodic missed +111です。B1がdue periodic slotを優先置換するたびにmissedを計上するpolicyのため、timing report 250送信では125 slotを計上しました。A0/B1は各1 Hzを維持しており、CAN RX/TX error、LoRa RX/TX error、AUX timeout、B1 duplicate/drop、全LoRa queue drop、1秒超gap、analyzer faultは0でした。

### v4 phase collision

`final_phase_commands_v4`はA0-only状態で、各A0 decode境界から0/50/100/150/200 ms後に`g 0x7F`と`le`を各5回送る計画でした。phase 200 msの2件目でtransaction 42のrelease行が別出力と連結し、runner parserが認識できず42件で停止しました。rawにはrelease済み記録があり、hardware release失敗ではありません。

- phase 0 ms: 10/10 PASS。`g 0x7F`は`Rejected/NotSupported`、`le`は`Rejected/InvalidState`。input→finalは`g 0x7F`がmin 538.557 / avg 590.144 / max 795.288 ms、`le`がmin 532.777 / avg 533.665 / max 534.326 ms。
- phase 50/100/150 ms: 各0/10。phase 200 ms: 0/2。全32件が`FinalB0Timeout`でlate finalもありません。
- Ground側は42/42でAUX Lowを観測し、物理uplink時間は`g 0x7F` 273.569〜275.094 ms、`le` 273.959〜275.959 msでした。
- 成功送信開始はGround device clockで境界後11.05〜21.62 ms、最初の失敗は63.15 msでした。ComBoard側は失敗区間でLoRa RX byte、decode error、CAN TXが増えず、受信途中の破損ではなくE220 half-duplex衝突によるzero-byte lossと判定します。21.62〜63.15 msの境界は未評価です。

A0 TXは実測約316 ms、absolute周期は500 msなので自然idleは約184 msです。約274〜276 msのuplinkを任意位相で収めることはできません。ComBoardのscheduler phaseを動かさず、Groundが実際にdecodeしたdownlink境界直後へuplinkを整列する方針としました。B1 active時はA0直後にB1が約250 msで続くため、TimeResponseはB1境界、通常commandは次の安全なperiodic境界を使います。

### Ground boundary alignment後

`final_phase_commands_aligned_v2`はv4と同じ安全条件で、A0 decodeからの入力phase 0〜450 msを50 ms刻み、各phaseで`g 0x7F`と`le`を各5回、計100件を実行しました。入力時点がどのphaseでも、Groundは各commandを次の未使用periodic boundaryへ移し、1 boundaryにつきuplinkを1件だけ開始します。

| 計測 | 実測 |
|---|---:|
| phase command | 100/100 PASS、timeout 0、release失敗0、late final 0 |
| `g 0x7F` input→final | count 50、min 526.740、avg 742.818、p95 966.976、max 967.438 ms |
| `le` input→final | count 50、min 532.875、avg 743.610、p95 966.603、max 968.389 ms |
| 1秒以上 / 1.6秒以上 | 0 / 0 |
| boundary age / fallback | 100/100が20 ms以内（min 0.043、avg 1.418、p95 11.369、max 18.347 ms）/ 0 |
| Ground AUX Low | 100/100 |
| Ground packet | 148.783995 sで299 packet（A0 197、B1 2、B0 100） |
| 全packet間隔 | count 298、min 0.250578、avg 0.499275、p50 0.501123、p95 0.506396、p99 0.521380、max 0.748587 s |

Ground uplinkの物理時間は`g 0x7F`平均274.453 ms、`le`平均274.367 msです。ComBoard HWSTAT差分はCAN TX +91、LoRa RX success +96、LoRa TX +281、periodic missed +96でした。最後のreport未満sampleとHWSTAT有効区間外を含むためcommand件数とは一致しません。B0がdue periodic slotを優先置換したmissedであり、全packetの1秒超gapは0です。CAN/LoRa error、AUX timeout、全queue dropは差分0、analyzer faultも0でした。ComBoard emergency resultのreport済み49 sampleはqueue待ち加重平均56.222 msです。

### 最終production復帰

診断featureを外して3基板をproductionへ戻し、`final_production_75s`で73.997553秒同時観測しました。Ground受信は149 packet（A0 145、B1 2、B0 2）、全packet間隔は平均499.983 ms、p95 501.140 ms、最大749.540 ms、1秒超gapと5秒超gapは0でした。`g 0x7F`は692.940 msで`Rejected/NotSupported`、`le`は692.220 msで`Rejected/InvalidState`のfinal B0を受信しました。

HWSTAT有効区間60.189740秒の差分はCAN RX +15,349、CAN TX +2、LoRa TX +120、LoRa RX +2です。CAN RX/TX error、TEC/REC、LoRa RX/TX error、AUX timeout、全LoRa queue dropは0でした。Missionは`safe_outputs=ESP_OK`、`flight_enabled=false`、encoder begin/status/pipeline、IMU、CAN beginがすべて`ESP_OK`で、3基板ともpanic/reset loopはありません。raw logは`/tmp/99l_lora_scheduler_20260814T141137/final_production_75s/`です。

Groundの単一boundary queue、B1/Periodic境界policy、AUX-before-boundary、commit直前freshness再確認とabsolute 2200 ms deadlineをすべて反映した後、`final_current_mixed_b1`でA0+B1混在中の`g 0x7F`/`le`を各2回実行し、4/4がB1完了境界で送信・期待B0受信までPASSしました。fallbackは0、AUX Lowは4/4、境界再選択は1回です。物理capacityのためinput→B0は1.169〜2.169秒で、A0+B1混在中は1秒engineering targetを満たしません。

同じcurrent codeのproductionを3基板へ戻した`final_current_production_70s`は69.500066秒で140 packet（A0 136、B1 2、B0 2）でした。全packet間隔は平均500.000 ms、p95 502.279 ms、p99 749.474 ms、最大751.239 ms、1秒/5秒超gap 0です。`g 0x7F`は926.277 msで`Rejected/NotSupported`、`le`は927.211 msで`Rejected/InvalidState`を受信しました。HWSTAT 60.198429秒の差分はCAN RX +15,350、CAN TX +1、LoRa TX +121、LoRa RX +2、periodic missed +2で、CAN/LoRa/AUX/queue drop errorは0です。Missionのsafe output、runtime、encoder health/pipeline、CANは正常、panic/reset loopは0、productionの`LORA_TIMING`/`GROUND_LORA_TIMING`も0です。raw logは`/tmp/99l_lora_scheduler_20260814T141137/final_current_production_70s/`です。

最終Ground hardening artifactでも0〜450 msを50 ms刻み、`g 0x7F`/`le`を各位相5回ずつ送る100試行を再実行し、100/100が期待したfinal B0までPASSしました。timeout、late final、fallback、invalidated境界は0、AUX Low→Highは時刻応答を含む101/101です。input→B0のmin/avg/p95/maxは`g 0x7F`が528.693/742.953/967.395/968.323 ms、`le`が531.953/743.734/966.374/968.250 msで、1秒/1.6秒以上のtailは0でした。

その後3基板を再びproductionへ戻した69.500925秒captureは140 packet（A0 138、B0 2）、全packet間隔は平均500.007 ms、p95 502.390 ms、p99 519.764 ms、最大520.596 ms、1秒/5秒超gap 0でした。`g 0x7F`は909.386 ms、`le`は909.339 msで期待したfinal B0を受信しました。完全なHWSTAT間50.166秒の差分はCAN RX/TX +12,794/+2、LoRa TX/RX +100/+2、periodic missed +2で、CAN/LoRa/AUX/queue drop error、panic/reset loop、production timing診断lineは0です。rawとanalysisは`/tmp/99l_final_current_sweep_LpJ8fHJC/`です。

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

- `src/lora_scheduler.rs` / `src/tasks/lora_task.rs`: 500 ms absolute deadlineを維持し、期限超過slotをburst送信せずskipしてcounterへ記録する。
- `src/tasks/lora_task.rs` / `src/constants.rs`: UART flush後のAUX Low開始とHigh復帰を送信完了条件にし、旧`LORA_POST_TX_RX_WINDOW_MS=350`を削除する。RX activity後60 ms guardとAUX再確認は維持する。
- `src/state.rs` / `src/tasks/can_communication.rs`: `EmergencyResult > CommandResult > GroundTimeRequest > Recovery > periodic`のsource別queueを設ける。B1は同一ID duplicateを抑止し、満杯時oldest dropを専用counterへ記録する。
- `src/lora_timing.rs` / `lora-timing-debug`: source別queue待ち、AUX/UART/物理送信、absolute interval、missed slotを10送信ごとに計測する。
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
