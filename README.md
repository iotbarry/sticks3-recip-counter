# StickS3 往复电机次数统计套装

基于 **M5Stack StickS3 (SKU K150)** 的往复运动次数统计设备 + Web Bluetooth 数据可视化大屏。

```
sticks3-recip-counter/
├── firmware/StickS3_RecipCounter/StickS3_RecipCounter.ino   # 设备固件 (Arduino)
├── web/dashboard.html                                        # 数据可视化大屏 (单文件·零依赖)
├── tools/ble_scan.py                                         # BLE 自检: 扫描 / --gatt / --probe / --watch
├── tools/serial_dump.py                                      # 串口抓日志(不复位芯片)
├── tools/post_upload_reset.py                                # PIO 上传后看门狗复位钩子
└── README.md
```

---

## 一、设备操作逻辑

| 动作 | 效果 |
|---|---|
| 按下 **正面大按键 (A)** | 开始计时 + 往复计数（IMU 检测每次往复 +1） |
| 再次按下 **A** | 停止统计，屏幕冻结显示 **往复次数 + 本次时长 + 平均频率** |
| **右侧按键 (B) 长按 1 秒** | 存档本场记录到 NVS（掉电不丢，**带本场开始时刻时间戳**）并复位回待机 |
| 设备放置方式 | 横置、屏幕朝上（若画面上下颠倒，把固件 `LCD_ROT` 改为 `3`） |

屏幕底部中央显示 **+8 本地时间 `HH:MM`**（未与网页校时时显示 `--:--`）。

- 存档上限 **40 条**，超出自动覆盖最旧。
- 检测灵敏度阈值默认 `0.35g`，可通过网页滑杆实时下发（存入设备 NVS）。
  - 数值越**小**越**灵敏**；环境振动大/误计数 → 调大；轻微微动漏计 → 调小。
- 屏幕界面为数字/英文（M5GFX 默认字库不含中文），完整中文界面在网页大屏。

## 二、固件烧录（PlatformIO · 已实测通过）

```bash
cd firmware/StickS3_RecipCounter
pio run -t upload      # 编译 + 烧录 + 自动复位启动
pio device monitor     # 串口日志 115200（心跳每 3 秒一行）
```

- 串口号在 `platformio.ini` 里改 `upload_port` / `monitor_port`（本机实测 COM9）。
- 首次编译会自动拉取 M5Unified + M5GFX（源码与 Arduino IDE 共用同一份 `.ino`）。
- **踩坑 1｜烧完不跑**：StickS3 走原生 USB-Serial/JTAG，esptool 的 RTS 硬复位无效，烧录后芯片卡在 ROM 下载模式（现象：烧录显示 SUCCESS，但不广播、无日志）。
  解决：工程用 `tools/post_upload_reset.py` 在上传后执行 **看门狗复位** 自动启动应用。
- **踩坑 2｜IMU 初始化失败**：M5Unified 首次初始化 BMI270 会失败（板型已正确识别为 26=M5StickS3，I2C 也能读到 ID `0x24`，但 8KB 配置上传时机过早）。
  解决：固件内置兜底 —— 外设就绪后显式再调用一次 `M5.Imu.begin()`，实测 `imu=1` 正常。
- **踩坑 3｜GATT 里找不到自定义服务**：core 3.x 的 `createServer()` 之后立刻 `createService()`，第一个服务会静默丢进黑洞（广播里有 UUID，宿主机枚举却看不到）。
  解决：`createServer()` 后 `delay(500)`，先建 16 位服务（电量 `0x180F`）打底，并且**每建一个服务马上 `start()`**，不要攒到最后一起 start。
- **踩坑 4｜订阅时报 `Write Not Permitted`**：core 3.x **不会**自动给 Notify 特征补 CCCD(`0x2902`)。
  解决：显式 `pStatus->addDescriptor(new BLE2902())`（2.x/3.x 都要），且必须在 `start()` 之前加。
- **踩坑 5｜收得到命令但收不到数据**：行协议要求每条 JSON 以 `\n` 结尾，否则网页端/脚本按换行分帧永远凑不出一帧。
- **踩坑 6｜网页按钮点了没反应**：CMD 特征只有 `PROPERTY_WRITE` 时，浏览器 `writeValueWithoutResponse()` 会抛 `NotSupportedError`，命令静默失败。
  解决：固件给 CMD 同时加 `PROPERTY_WRITE_NR`；网页端也按 `chCmd.properties` 选择写入方式。
- **踩坑 7｜断开后设备"消失"**：在 `onDisconnect()` 回调里直接 `startAdvertising()` 无效（BLE 栈尚未释放）。
  解决：回调里只打标记，回主循环延迟 500ms 再重启广播。
- **自检**：串口心跳形如
  `[HB] st=0 cnt=0 ms=0 ble=0 imu=1 heap=243744 th=0.35 board=26 bmi=0x24 svc=0x0037 bat=0x0028`
  （`imu=1` 计数可用；`board=26` = M5StickS3；`bmi=0x24` BMI270 在线；`svc/bat` = GATT 句柄，非 0 即注册成功）
  紧接着一行是时间状态：
  `[HB] clock=SET tz=+480min rtc=1 ts=2026-10-08 02:39:46`
  （`clock=SET` 已校时；`tz=+480min` 即 +8 东八区；`ts` 为设备当前本地时间）
- **BLE 自检**（无需网页）：
  ```bash
  pip install bleak pyserial
  python tools/ble_scan.py            # 扫描，应看到 StickS3-Recip + 7a5f1000 服务
  python tools/ble_scan.py --gatt     # 枚举全部服务/特征（查 CCCD、查服务是否注册）
  python tools/ble_scan.py --probe    # 端到端：ping/tsync/th/start/stop/archive/dump
  python tools/serial_dump.py COM9 12 # 抓串口日志（不会复位芯片）
  ```

### 实测链路（本机 COM9 / ESP32-S3-PICO-1）

```
[BLE] bat=0x0028 svc=0x0037 dis=0x0046        # 三个服务句柄均正常
[服务] 7a5f1000-…  handle=55
   [特征] 7a5f1001  props=notify   [描述符] 2902
   [特征] 7a5f1002  props=write,write-without-response
>> {"cmd":"th","v":0.4}     << {"ev":"ok","cmd":"th"}
>> {"cmd":"start"} …        << {"ev":"st","st":1,"cnt":…}   # 200ms 一条
>> {"cmd":"stop"}           << {"ev":"st","st":2,"ms":5091}
>> {"cmd":"dump"}           << {"ev":"arc",…} x7 + {"ev":"dumpend","n":7}
```

---

## 二-B、固件烧录（Arduino IDE，手动方式）

1. **开发板支持包**：安装 `esp32 by Espressif Systems` **3.x**（Boards Manager）。
2. **库**（Library Manager）：
   - `M5Unified`（最新版，自动带 M5GFX）—— **必须**
   - `M5PM1` —— 可选（电量百分比显示用；不装也能编译，网页电量显示 `--`）
3. **板型设置**：
   - Board: `ESP32S3 Dev Module`
   - USB CDC On Boot: `Enabled`
   - Flash Size: `8MB (64Mb)`，PSRAM: `OPI PSRAM`
   - Partition Scheme: `8M with spiffs`（默认含 NVS 即可）
4. 进入下载模式烧录：USB 连接 → 长按侧边复位键至绿色 LED 闪烁 → 选择串口 → 上传。
5. 复位后设备将以 **`StickS3-Recip`** 名称广播 BLE。

> 若正/侧按键功能相反（不同批次），交换代码中 `BtnA`/`BtnB` 即可。

## 三、网页大屏使用

1. 打开 `web/dashboard.html`（**Chrome / Edge 桌面版或 Android Chrome**；iOS Safari 不支持 Web Bluetooth）。
   - 直接双击打开即可；若点击连接无弹窗，用本地服务器：`python -m http.server` 后访问 `http://localhost:8000/web/dashboard.html`。
2. 页面默认运行 **演示模式**（模拟数据，方便预览大屏效果）。
3. 点击 **「连接设备」** → 选择 `StickS3-Recip`：
   - 自动下发**时间同步**（存档时间戳以此为准）
   - 自动拉取设备内全部**历史存档**
   - 自动关闭演示模式，切换为真实数据
4. 大屏功能：
   - 6 项 KPI：运行状态 / 本次次数 / 本次时长 / 实时频率（含迷你趋势）/ 累计场次 / 累计总次数
   - **本场次累计曲线**：往复次数（青色实线，左轴）+ 频率（紫色虚线，右轴），200ms 实时刷新
   - **场次对比**：最近 16 场柱状图
   - **存档记录表** + **导出 CSV**（带 BOM，Excel 直接打开不乱码）
   - 远程控制：开始/停止、存档复位、读取历史、灵敏度滑杆
5. 断开后点 **「重新连接」** 可直接回连同一设备（无需重新配对）。

## 四、BLE 协议（自定义服务）

| 项 | UUID | 方向 | 说明 |
|---|---|---|---|
| 服务 | `7a5f1000-3c17-4a5e-b6a1-2d9c8e4f6a01` | — | 主服务 |
| STATUS | `7a5f1001-…` | 设备→网页 Notify | 换行分隔 JSON |
| CMD | `7a5f1002-…` | 网页→设备 Write | 单行 JSON |
| 电量 | 标准 `0x180F` / `0x2A19` | Notify | 未装 M5PM1 时上报 255=未知 |

**STATUS 推送**（运行中每 200ms，空闲每 1s）：

```json
{"ev":"st","st":1,"cnt":123,"ms":45678,"f":161.7,"th":0.35,
 "ts":1760000000,"tsz":480,"sts":1759999500}
//   st: 0空闲 1运行 2停止
//   ts:  设备当前时间 (UTC 秒)
//   tsz: 设备显示用偏移 (分钟, 480 = +8 东八区)
//   sts: 本场开始时刻 (UTC 秒, 存档时间戳的来源)
{"ev":"arc","i":12,"cnt":456,"ms":178000,"ts":1759999500}     // 存档记录 (i=累计场次号, ts=本场开始时刻)
{"ev":"dumpend","n":12}                                        // 历史拉取结束
{"ev":"cfg","th":0.40} {"ev":"ok","cmd":"start"}               // 配置回显 / 命令确认
```

**网页命令**：

```json
{"cmd":"start"} {"cmd":"stop"} {"cmd":"archive"} {"cmd":"dump"}
{"cmd":"tsync","v":1760000000}      // 下发 UTC 秒, 建立设备系统时钟 (与时区无关)
{"cmd":"tzsync","v":480}            // 下发显示时区偏移(分钟): 北京/东八区=480
{"cmd":"th","v":0.40}
```

### 时间戳机制

设备**没有联网能力**，时间戳走两条路径，互为兜底：

| 场景 | 时间来源 | 精度 |
|---|---|---|
| 网页在线 | 网页下发 `tsync`，设备 `settimeofday()` 建立系统时钟，硬件 RTC 自走时 | 秒级（实测偏差 ≤1s） |
| 纯离线运行 | NVS 里持久化的上次已知时刻 + 本次开机秒数外推 | 取决于上次校时距今多久 |

- **时区**：设备时间戳统一为 **UTC 秒**（与时区无关），显示时再按 `tzOffsetS` 换算，**默认 +8 东八区**。
  网页连接时会自动下发 `tzsync` 校正，并写入 NVS，因此离线重开机后时区依然正确。
- **存档时间戳记的是"本场开始时刻"**，不是存档动作发生的时刻——这样一场跨小时的测试也能准确定位。
- 设备屏幕底部中央显示 `HH:MM`（+8 本地时间），未校时时显示 `--:--`。
- 网页右上角显示设备实时时钟，并可提示设备与本机的时间偏差（≤2s 显示绿色）。
- 时间基准每 **5 分钟**自动落盘一次，存档时也会落盘，保证掉电重启后外推基准足够新。

## 五、往复检测原理与调优

- BMI270 加速度合成幅值 `|a|` 偏离 1g 的量 `dev` 做 **Schmitt 迟滞触发**：`dev ≥ 阈值` 计数一次，回落到 `0.4×阈值` 重新武装 → 每个完整往复周期计 1 次。
- 两次计数最小间隔 120ms（防机械抖动重复计数），可在固件 `MIN_STROKE_MS` 调整。
- 设备需**刚性贴合**机体（背面磁吸可用）；悬空放置会漏检。
- 若电机频率极高（>8Hz）或振动极强，适当调大阈值与最小间隔。

## 六、常见问题

| 现象 | 处理 |
|---|---|
| 连接无蓝牙弹窗 | 换 Chrome/Edge；或用 localhost 方式打开；确认系统蓝牙已开 |
| 扫描不到设备 | 设备**已被别的主机连上**（另一个浏览器标签页/脚本）时不会广播，先在那边断开；或用 `tools/ble_scan.py --gatt` 确认 |
| 连上后一直没数据 | 看串口是否有 `[NTF] …`；确认固件每条通知以 `\n` 结尾（见踩坑 5） |
| 屏幕时间是 `--:--` | 设备尚未与网页校时。连一次网页即可；纯离线使用需要至少校时过一次 |
| 存档时间戳是 `0` / 显示 `--` | 该条存档是在校时之前录的（旧记录不会补时间戳）；重新校时后新存档即正常 |
| 存档时间差几小时 | 时区问题：网页连接会自动下发 `tzsync`，确认页面右上角显示的时区标签正确 |
| 计数偏多 | 网页灵敏度调大（如 0.5~0.8g），或调大 `MIN_STROKE_MS` |
| 计数偏少 | 调小阈值（如 0.2g），检查设备是否贴紧机体 |
| 电量一直 `--` | Library Manager 安装 `M5PM1` 库后重新编译 |
| 画面上下颠倒 | 固件 `LCD_ROT` 改为 `3` |
| M5PM1 编译报错 | 电量为可选功能，删除固件中 `HAS_PM1` 相关 3 处即可 |
