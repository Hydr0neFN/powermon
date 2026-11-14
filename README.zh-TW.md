[English](README.md) · **繁體中文**

# powermon

ESP8266 電表脈衝計數器 -> MQTT -> Home Assistant。

用來取代遺失的 `Duco RF Gateway + Power Monitor v1.0` 韌體。程式是從 flash dump
（`duco-esp-flash.bin`，2026-09-08）撈出的字串，加上下面的硬體筆記重建而成。CC1101/Duco
那一半沒有重做。

## 概覽

| | |
|---|---|
| 問題 | broker 位址寫死在編譯期。broker 一搬家，韌體就沒地方改。 |
| 做了什麼 | 重寫的 ESP8266 韌體：電表上的光二極體脈衝計數、透過 MQTT 送進 Home Assistant、3 位數顯示器，還有一個網頁設定頁。 |
| 電表常數 | 0.1 Wh/pulse（10000 imp/kWh） |
| 技術堆疊 | ESP8266、Arduino framework、PlatformIO、LittleFS、PubSubClient |

## 硬體

| 功能       | 腳位       | 說明 |
|------------|------------|------|
| 光二極體   | `A0`       | 類比腳：電表 LED 視窗上的訊號擺幅太小，數位輸入抓不住 |
| TM1637 CLK | `D3` GPIO0 | boot-strap 腳位，reset 時必須為 high -- TM1637 閒置時就是 high，所以剛好合用 |
| TM1637 DIO | `D4` GPIO2 | boot-strap 腳位，理由同上 |

## 運作方式

```
meter LED --> photodiode --> A0 (ADC polled every 5 ms)
                                 |
              PulseDetector: baseline, peak follower,
              hysteresis, min gap 15 ms, arm swing 36
                                 |
              one pulse = 0.1 Wh; total = offset + pulses
                                 |
              every 10 s: watts averaged over the window
                                 |
           +-----------+---------+------------+
           |           |                      |
      TM1637      <prefix>/<nodeId>/state   HA discovery
      3 digits    (JSON)                   (4 sensors)
```

`src/pulse.h` 直接用訊號本身決定門檻：慢速 EWMA 追暗態基準線，另一個尖峰追蹤器上升快、
下降慢，用來追閃光的亮度；觸發點放在兩者之間，再加遲滯。`src/main.cpp` 每 5 ms
讀一次 `A0`，經過 30 秒暖機後，每次閃光算一個脈衝。每 10 秒把這段窗期的瓦數取平均再送出去。

## 舊版為何壞掉

broker 位址是寫死的常數 `<old-broker-ip>`。broker 搬到 `<new-broker-ip>` 之後，韌體沒有
任何地方能改。這一版在執行時才解析 `mqtt_host`，預設值是 mDNS 名稱，所以之後換位址不必
重新燒錄。

`src/` 裡的程式註解交代了其他設計決定：

- **門檻由訊號決定。** 雜訊下擺幅不到 12 counts（要累積到 36 才會開始計數），直流準位
  又會跟著室內光線飄，固定門檻得一直重新校正（`src/pulse.h`）。
- **擺幅還沒建立前不計數。** 不然室內燈光的閃爍就會觸發門檻；開機後讀到 ~10 kW 就是這
  樣來的（`kArmSwing`）。
- **30 秒暖機。** 開機頭幾秒的脈衝不可信；直到收到第一個真脈衝前，顯示器顯示 `-1`、也
  不送出資料（`kWarmupMs`）。
- **`A0` 每 5 ms 讀一次，不跑每 1 ms。** `analogRead()` 跟 RF 校正共用同一個 SAR ADC，跑到 1 kHz 時
  接收端會漏掉 beacon，連上約 20 秒後就掉線（`loop()`）。
- **關掉 WiFi sleep，並記錄 802.11 斷線原因。** modem sleep 會讓 AP 把 station 踢掉；
  原因碼能分開 AP 踢人（8）、beacon timeout（200）和握手失敗（15）（`setup()`）。
- **keepalive 設 60 秒。** PubSubClient 預設 15 秒、broker 按 1.5 倍卡；`loop()` 一卡住
  就得多重連一次（`mqttConnect()`）。
- **電量每 0.01 kWh 或每 10 分鐘存一次，哪個先到就存。** flash 抹寫次數有限；斷電最多
  就是少這一段（`energyPersist()`）。
- **顯示器在本地時間 00:30 到 07:30 熄掉。** 它剛好在臥室視線範圍內；SNTP 對時完成前
  會一直亮著（`isNight()`）。
- **沒有備援 SoftAP。** 要重新設定就用 USB 接線（`setup()`）。

## 設定

設定放在 LittleFS（`/cfg.json`），可以在 `http://<device>/` 修改 -- broker 主機、port、
帳密、topic 前綴、每 kWh 脈衝數，以及用來跟實體電表讀數對齊的電量偏移量。

## 燒錄

```
pio run -t upload                      # serial, first time
pio run -t upload -e esp12e_ota        # thereafter
```

## 專案結構

| 路徑 | 內容 |
|---|---|
| `src/main.cpp` | 設定、電量、MQTT、網頁伺服器、`setup()` 與 `loop()` |
| `src/pulse.h` | `PulseDetector` |
| `src/tm1637.h` | TM1637 顯示器驅動 |
| `platformio.ini` | PlatformIO 環境（`esp12e`） |

## 限制與待辦

- CC1101/Duco 那一半沒有重做。
- 沒有備援 SoftAP。WiFi 設定寫錯時要靠 USB 接線處理。

## 授權

這個 repo 裡沒有 `LICENSE` 檔案。
