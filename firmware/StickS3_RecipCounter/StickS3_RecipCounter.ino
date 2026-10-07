/* ==================================================================
 * StickS3 往复电机次数统计器  Recip-Counter v1.0
 * ------------------------------------------------------------------
 * 硬件 : M5Stack StickS3 (SKU K150, ESP32-S3-PICO-1-N8R8)
 *        1.14" ST7789P3 135x240 / BMI270 6轴IMU / M5PM1 电源管理
 * 功能 :
 *   [正面大按键 BtnA] 开始统计 -> 再按停止 (冻结次数与时长)
 *   [侧面按键  BtnB] 长按1秒  -> 存档 + 复位 (NVS 持久化, 最多40条)
 *   设备横置(屏幕朝上), IMU 检测往复运动, 迟滞(Schmitt)触发计数
 *   BLE GATT: 实时状态 Notify / 命令 Write / 标准电量服务
 *   网页端: 配套 dashboard.html (Web Bluetooth 数据可视化大屏)
 * ------------------------------------------------------------------
 * 依赖库: M5Unified (最新版) + M5GFX + M5PM1(可选, 电量显示)
 *         ESP32 Arduino core 3.x (内置 BLE 库, 无需额外安装)
 * ================================================================== */

#include <M5Unified.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>

/* ---------- 可选: M5PM1 电量读取 (未安装该库也能编译) ---------- */
#if __has_include(<M5PM1.h>)
  #include <M5PM1.h>
  #define HAS_PM1 1
  static M5PM1 pm1;
#else
  #define HAS_PM1 0
#endif

/* ---------- Arduino-ESP32 core 版本兼容 ----------
   注意: core 3.x 也不会自动给 Notify 特征补 CCCD(0x2902),
   必须显式 addDescriptor(new BLE2902()), 否则宿主订阅时报
   "Write Not Permitted"。2.x / 3.x 都要加。 */
#include <BLE2902.h>

/* ================= 用户可调参数 ================= */
#define LCD_ROT          1      // 屏幕旋转: 1 或 3 (若画面颠倒改成 3)
#define HIST_MAX         40     // NVS 存档条数上限 (循环覆盖最旧)
#define MIN_STROKE_MS    120    // 两次计数最小间隔 ms (防抖)
#define TH_DEFAULT       0.35f  // 默认触发阈值 (g), 网页可调
#define NOTIFY_RUN_MS    200    // 运行中状态推送间隔
#define NOTIFY_IDLE_MS   1000   // 空闲/停止状态推送间隔
#define BAT_PERIOD_MS    30000  // 电量刷新周期

/* ================= BLE UUID ================= */
#define SERVICE_UUID  "7a5f1000-3c17-4a5e-b6a1-2d9c8e4f6a01"
#define STATUS_UUID   "7a5f1001-3c17-4a5e-b6a1-2d9c8e4f6a01"   // Notify
#define CMD_UUID      "7a5f1002-3c17-4a5e-b6a1-2d9c8e4f6a01"   // Write

/* ================= 颜色 (RGB565 用 24bit 传入) ================= */
#define C_HDRBG  0x0A1626
#define C_ACCENT 0x22D3EE
#define C_GREEN  0x34D399
#define C_RED    0xF87171
#define C_DIM    0x7D93AD
#define C_WHITE  0xE6F1FF

/* ================= 状态机与数据 ================= */
enum RunState : uint8_t { S_IDLE = 0, S_RUN = 1, S_STOP = 2 };
static RunState st        = S_IDLE;
static uint32_t cnt       = 0;   // 运行中的计数
static uint32_t t0        = 0;   // 本场开始时刻 (millis)
static uint32_t lastCnt   = 0;   // 停止后冻结的次数
static uint32_t lastMs    = 0;   // 停止后冻结的时长
static float    th        = TH_DEFAULT;
static bool     armed     = true;
static uint32_t lastTrig  = 0;
static uint32_t epochBase  = 0;     // 离线外推基准 (上一次已知 UTC 秒)
static uint32_t startEpoch = 0;     // 本场开始时刻的 UTC 秒 (存档时间戳来源)
static bool     clockSet  = false;  // settimeofday() 是否已成功
static int32_t  tzOffsetS = 8 * 3600; // 时区偏移(秒), 默认 +8 东八区
static bool     tzFromNvs = false;  // 是否来自 NVS 持久化值
static bool     tzFromHost= false;  // 本次上电是否已被网页校正
static char     hourCache[16] = "--:--";  // 屏幕小时级缓存
static int      hourCacheH = -999;
static bool     imuOK     = false;
static uint8_t  batPct    = 255; // 255 = 未知
static bool     bleConn   = false;
static bool     needAdv   = false;  // 断开后需要重新广播
static uint32_t advAt     = 0;      // 重新广播的时间点
static bool     bLongDone = false;
static int      dbgBoard  = -1;   // 诊断: M5 识别到的板型
static uint8_t  dbgBmi    = 0;    // 诊断: BMI270 芯片 ID (应为 0x24)
static int      dbgRtry   = 0;    // 诊断: IMU 重试结果
static uint16_t dbgSvcH   = 0;    // 诊断: 自定义服务 GATT 句柄
static uint16_t dbgBatH   = 0;    // 诊断: 电量服务 GATT 句柄
static uint16_t dbgDisH   = 0;    // 诊断: 设备信息服务(0x180A) GATT 句柄

static Preferences prefs;
static uint16_t    histN = 0;   // 累计存档数(含已覆盖)

/* ================= BLE ================= */
static BLECharacteristic* pStatus = nullptr;
static BLEServer*         gServer = nullptr;   // 供通知兜底发送使用
static BLECharacteristic* pBat    = nullptr;

/* 来自 BLE 回调的命令, 转交主循环处理 (避免跨任务刷屏) */
static char    cmdBuf[64];
static volatile bool cmdFlag = false;

struct Rec { uint32_t cnt; uint32_t ms; uint32_t ts; };

static String recKey(uint16_t idx) { return String("r") + String(idx % HIST_MAX); }

/* ---------- 时间基准 ----------
   两条路径:
   A. 网页实时连接 -> 下发 Unix 时间戳, 设备 settimeofday() 建 RTC, 时间随漂移自校正
   B. 纯离线运行  -> 用 NVS 里持久化的上次时间 + 本次开机后经过的秒数 外推
   两者都返回 UTC 秒, 再用 tzOffsetS 换算成 +8 本地时间显示。 */
static uint32_t nowEpoch() {
  time_t t = time(nullptr);
  if (t > 1600000000L) return (uint32_t)t;    // 系统时钟已建立
  return epochBase ? epochBase + millis() / 1000 : 0;  // 离线外推
}

static uint32_t nowTs() { return nowEpoch(); }

/* 把 Unix 秒 + 时区偏移换算成日历字段 (UTC 版, 不依赖 TZ 环境变量) */
static void tsToLocal(uint32_t epoch, int32_t off, int& Y, int& M, int& D,
                      int& h, int& m, int& s, int& wd) {
  time_t   t = (time_t)epoch + off;
  struct tm g;
  gmtime_r(&t, &g);
  Y = g.tm_year + 1900; M = g.tm_mon + 1; D = g.tm_mday;
  h = g.tm_hour; m = g.tm_min; s = g.tm_sec; wd = g.tm_wday;
}

/* 屏幕用: 小时级缓存, 避免每帧 gmtime_r */
static const char* hourStr() {
  uint32_t e = nowEpoch();
  if (!e) { return "--:--"; }
  int Y, M, D, h, m, s, wd;
  tsToLocal(e, tzOffsetS, Y, M, D, h, m, s, wd);
  if (h != hourCacheH) {
    hourCacheH = h;
    snprintf(hourCache, sizeof(hourCache), "%02d:%02d", h, 0);
  }
  return hourCache;
}

static uint32_t lastNtfLog = 0;

static void notifyJson(const char* s) {
  if (!pStatus || !gServer) return;
  /* 行协议: 每条 JSON 以 '\n' 结尾, 网页端按换行分帧 */
  char line[220];
  size_t n = snprintf(line, sizeof(line), "%s\n", s);
  BLE2902* d = (BLE2902*)pStatus->getDescriptorByUUID((uint16_t)0x2902);
  bool cccd = d ? d->getNotifications() : false;
  uint32_t conns = gServer->getConnectedCount();

  if (millis() - lastNtfLog >= 3000) {          // 限流, 避免刷屏
    lastNtfLog = millis();
    Serial.printf("[NTF] ble=%d conns=%u cccd=%d len=%u\n",
                  (int)bleConn, (unsigned)conns, (int)cccd, (unsigned)n);
  }

  pStatus->setValue((uint8_t*)line, n);
  if (!bleConn || conns == 0) return;

  /* 兜底: 宿主写了 CCCD, 但 Bluedroid 没把写事件回传给 BLE2902,
     此时 notify() 会被 "notifications disabled" 闸门静默丢弃。
     这里补上置位, 保证网页一定能收到数据。 */
  if (d && !cccd) {
    d->setNotifications(true);
    Serial.println("[NTF] 补置 CCCD notify 位");
  }
  pStatus->notify();
}

static void sendStatus() {
  uint32_t c, ms;
  if (st == S_RUN)       { c = cnt;     ms = millis() - t0; }
  else if (st == S_STOP) { c = lastCnt; ms = lastMs; }
  else                   { c = 0;       ms = 0; }
  float f = (st != S_IDLE && ms > 0) ? c * 60000.0f / ms : 0.0f;
  char b[128];
  /* ts/tsz: 设备端当前时间与显示用偏移, 供网页校时比对;
     sts: 本场开始时刻(仅运行/停止态有意义) */
  snprintf(b, sizeof(b),
           "{\"ev\":\"st\",\"st\":%u,\"cnt\":%lu,\"ms\":%lu,\"f\":%.1f,\"th\":%.2f,"
           "\"ts\":%lu,\"tsz\":%d,\"sts\":%lu}",
           (unsigned)st, (unsigned long)c, (unsigned long)ms, f, th,
           (unsigned long)nowEpoch(), (int)(tzOffsetS / 60),
           (unsigned long)startEpoch);
  notifyJson(b);
}

static void sendArc(uint32_t idx, const Rec& r) {
  char b[96];
  snprintf(b, sizeof(b),
           "{\"ev\":\"arc\",\"i\":%lu,\"cnt\":%lu,\"ms\":%lu,\"ts\":%lu}",
           (unsigned long)idx, (unsigned long)r.cnt,
           (unsigned long)r.ms, (unsigned long)r.ts);
  notifyJson(b);
}

static void sendAck(const char* cmd) {
  char b[48];
  snprintf(b, sizeof(b), "{\"ev\":\"ok\",\"cmd\":\"%s\"}", cmd);
  notifyJson(b);
}

/* ================= 动作 ================= */
static void startSession() {
  cnt = 0; t0 = millis(); armed = true; lastTrig = 0;
  startEpoch = nowEpoch();                 // 拍摄本场开始时刻(存档用)
  st = S_RUN;
  sendStatus();
}

static void stopSession() {
  lastCnt = cnt;
  lastMs  = millis() - t0;
  st = S_STOP;
  sendStatus();
}

static void doArchive() {
  if (st == S_RUN) stopSession();          // 运行中先停止
  if (st == S_STOP && lastCnt > 0) {
    Rec r;
    r.cnt = lastCnt;
    r.ms  = lastMs;
    /* 关键: 记录"本场开始时刻"。startEpoch 在 startSession() 时拍摄,
       此时 lastMs 是纯时长, 不要用 nowEpoch()-lastMs 反推, 否则会被
       中途的时钟校正污染。 */
    r.ts  = startEpoch ? startEpoch : nowEpoch();
    prefs.putBytes(recKey(histN).c_str(), &r, sizeof(r));
    histN++;
    prefs.putUShort("n", histN);
    sendArc(histN, r);                     // 下发带编号的存档事件
  }
  /* 存档后把时间基准落盘, 保证下次纯离线开机也能给出正确时间戳 */
  persistTimebase();
  cnt = 0; lastCnt = 0; lastMs = 0;
  st = S_IDLE;
  sendStatus();
  sendAck("archive");
}

static void dumpHistory() {
  uint32_t start = (histN > HIST_MAX) ? histN - HIST_MAX : 0;
  for (uint32_t i = start; i < histN; i++) {
    Rec r;
    prefs.getBytes(recKey(i).c_str(), &r, sizeof(r));
    sendArc(i + 1, r);
    delay(15);                             // 避免通知缓冲溢出
  }
  char b[48];
  snprintf(b, sizeof(b), "{\"ev\":\"dumpend\",\"n\":%u}", (unsigned)histN);
  notifyJson(b);
}

/* ================= 命令解析 ================= */
/* 极简 JSON 取值: 找 "key" 后取冒号后的数字 */
static long jsonInt(const String& s, const char* key) {
  int p = s.indexOf(key);
  if (p < 0) return 0;
  p = s.indexOf(':', p);
  if (p < 0) return 0;
  return s.substring(p + 1).toInt();
}

static float jsonFloat(const String& s, const char* key) {
  int p = s.indexOf(key);
  if (p < 0) return NAN;
  p = s.indexOf(':', p);
  if (p < 0) return NAN;
  return s.substring(p + 1).toFloat();
}

static void handleCmd(const String& s) {
  Serial.printf("[CMD] %s\n", s.c_str());
  if      (s.indexOf("\"start\"")   >= 0) { if (st != S_RUN) startSession(); }
  else if (s.indexOf("\"stop\"")    >= 0) { if (st == S_RUN) stopSession();  }
  else if (s.indexOf("\"archive\"") >= 0) { doArchive(); }
  else if (s.indexOf("\"dump\"")    >= 0) { dumpHistory(); }
  else if (s.indexOf("\"tsync\"")   >= 0) {
    /* v = UTC Unix 秒 (网页用 Date.now()/1000, 与时区无关)。
       直接建立系统时钟, 后续由硬件 RTC 自走时, 比 millis() 外推准得多;
       同时写一份 epochBase 作为掉电后的离线外推种子。 */
    long v = jsonInt(s, "\"v\"");
    if (v > 1600000000L) {
      struct timeval tv = { .tv_sec = (time_t)v, .tv_usec = 0 };
      clockSet = settimeofday(&tv, nullptr) == 0;
      epochBase = (uint32_t)v - millis() / 1000;
      persistTimebase();
      hourCacheH = -999;                   // 强制屏幕刷新时间
      Serial.printf("[TIME] tsync -> %ld (settimeofday %s)\n",
                    v, clockSet ? "OK" : "FAIL");
      sendAck("tsync");
    }
  }
  else if (s.indexOf("\"tzsync\"")  >= 0) {
    /* v = 相对 UTC 的偏移分钟数: 北京 480, 东京 540, UTC 0, 纽约 -300。
       只影响显示与日历分解, 不改变 Unix 时间戳本身。 */
    long v = jsonInt(s, "\"v\"");
    if (v >= -720 && v <= 840) {
      tzOffsetS = (int32_t)v * 60;
      prefs.putInt("tzmin", (int32_t)v);
      tzFromHost = true;
      hourCacheH = -999;
      Serial.printf("[TIME] tzsync -> %ld 分钟\n", v);
      sendAck("tzsync");
    }
  }
  else if (s.indexOf("\"th\"")      >= 0) {
    float v = jsonFloat(s, "\"v\"");
    if (!isnan(v) && v >= 0.05f && v <= 2.0f) {
      th = v;
      prefs.putFloat("th", th);
      sendAck("th");
    }
  }
  else if (s.indexOf("\"ping\"")    >= 0) { sendAck("ping"); }
}

/* ================= BLE 回调 ================= */
class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override   { bleConn = true;  Serial.println("[BLE] connected"); }
  void onDisconnect(BLEServer*) override {
    bleConn = false;
    /* 不能在回调里直接 startAdvertising()——栈还没释放完, 会静默失败导致
       设备"消失"。打个标记, 回主循环延迟 500ms 再重启广播。 */
    needAdv = true;
    advAt   = millis() + 500;
    Serial.println("[BLE] disconnected");
  }
};

class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    /* core 2.x 返回 String, core 3.x 返回 std::string —— 统一转换 */
    String v = String(c->getValue().c_str());
    if (!v.length()) return;
    size_t n = v.length();
    if (n > sizeof(cmdBuf) - 1) n = sizeof(cmdBuf) - 1;
    memcpy(cmdBuf, v.c_str(), n);
    cmdBuf[n] = 0;
    cmdFlag = true;                       // 主循环处理
  }
};

/* ================= 屏幕绘制 ================= */
static M5Canvas cvs(&M5.Display);

static void drawHeaderFooter(const char* l, uint32_t lc, const char* r) {
  cvs.fillRect(0, 0, 240, 22, C_HDRBG);
  cvs.fillRect(0, 113, 240, 22, C_HDRBG);
  cvs.setTextFont(2);
  cvs.setTextColor(lc, C_HDRBG);
  cvs.setTextDatum(textdatum_t::middle_left);
  cvs.drawString(l, 8, 11);
  cvs.setTextColor(C_DIM, C_HDRBG);
  cvs.setTextDatum(textdatum_t::middle_right);
  cvs.drawString(r, 232, 11);

  /* 底部条: 时间始终居中显示, 与 A/B 提示共存
     (时间已同步显示 +8 时:分, 未同步显示 --:--) */
  const char* hs = hourStr();
  cvs.setTextFont(1);
  cvs.setTextColor(C_ACCENT, C_HDRBG);
  cvs.setTextDatum(textdatum_t::middle_center);
  cvs.drawString(hs, 120, 124);
}

/* ---------- 时间基准 NVS 持久化 ---------- */
static void persistTimebase() {
  uint32_t e = nowEpoch();
  if (!e) return;
  prefs.putUInt("tbase", e);
  prefs.putUInt("tupd", millis() / 1000);
}

/* 取用 NVS 里的时间基准: 上次已知时刻 + 本次已开机秒数 外推 */
static void loadTimebase() {
  uint32_t b = prefs.getUInt("tbase", 0);
  if (b > 1600000000UL) {
    epochBase = b + prefs.getUInt("tupd", 0);
    Serial.printf("[TIME] 离线基准 %lu (来自 NVS)\n", (unsigned long)epochBase);
  }
}

/* 底部提示靠左, 给中间的时间留位 */
static void drawHints(const char* a, const char* b) {
  cvs.setTextFont(1);
  cvs.setTextColor(C_DIM, C_HDRBG);
  cvs.setTextDatum(textdatum_t::middle_left);
  String s = String(a) + "  " + b;
  cvs.drawString(s, 6, 124);
}

static void drawIdle() {
  cvs.fillSprite(TFT_BLACK);
  char rb[24];
  snprintf(rb, sizeof(rb), "BAT %s", batPct <= 100 ? String(batPct).c_str() : "--");
  drawHeaderFooter("REC-COUNTER", C_ACCENT, rb);

  cvs.setTextDatum(textdatum_t::middle_center);
  cvs.setTextFont(4);
  cvs.setTextColor(C_ACCENT, TFT_BLACK);
  cvs.drawString("READY", 120, 44);

  cvs.setTextFont(2);
  cvs.setTextColor(C_WHITE, TFT_BLACK);
  cvs.drawString("Push [A] to count", 120, 72);

  cvs.setTextFont(1);
  cvs.setTextColor(C_DIM, TFT_BLACK);
  char info[48];
  snprintf(info, sizeof(info), "SAVED %u  |  TH %.2fg",
           (unsigned)(histN > HIST_MAX ? HIST_MAX : histN), th);
  cvs.drawString(info, 120, 92);
  cvs.drawString("Hold [B] 1s = Save & Reset", 120, 104);

  drawHints("[A] START/STOP", "[B]HOLD1S SAVE");
  cvs.pushSprite(0, 0);
}

static void drawRun() {
  cvs.fillSprite(TFT_BLACK);
  drawHeaderFooter("* RUNNING", C_RED, "");

  /* 计时 */
  uint32_t ms = millis() - t0;
  char tb[16];
  snprintf(tb, sizeof(tb), "%02lu:%02lu", (unsigned long)(ms / 60000),
           (unsigned long)((ms / 1000) % 60));
  cvs.setTextFont(4);
  cvs.setTextColor(C_WHITE, TFT_BLACK);
  cvs.setTextDatum(textdatum_t::middle_center);
  cvs.drawString(tb, 120, 36);

  /* 大数字次数 */
  cvs.setTextFont(7);
  cvs.setTextColor(C_ACCENT, TFT_BLACK);
  cvs.drawString(String(cnt).c_str(), 120, 78);

  cvs.setTextFont(1);
  cvs.setTextColor(C_DIM, TFT_BLACK);
  float f = ms > 0 ? cnt * 60000.0f / ms : 0;
  char sb[40];
  snprintf(sb, sizeof(sb), "RATE %.0f /MIN   TH %.2FG", f, th);
  cvs.drawString(sb, 120, 106);

  drawHints("[A] STOP", "[B]HOLD1S SAVE");
  cvs.pushSprite(0, 0);
}

static void drawStop() {
  cvs.fillSprite(TFT_BLACK);
  char rb[24];
  snprintf(rb, sizeof(rb), "BAT %s", batPct <= 100 ? String(batPct).c_str() : "--");
  drawHeaderFooter("FINISHED", C_GREEN, rb);

  cvs.setTextFont(7);
  cvs.setTextColor(C_GREEN, TFT_BLACK);
  cvs.setTextDatum(textdatum_t::middle_center);
  cvs.drawString(String(lastCnt).c_str(), 120, 62);

  cvs.setTextFont(2);
  cvs.setTextColor(C_WHITE, TFT_BLACK);
  char tb[48];
  float f = lastMs > 0 ? lastCnt * 60000.0f / lastMs : 0;
  snprintf(tb, sizeof(tb), "%02lu:%02lu   AVG %.0f/MIN",
           (unsigned long)(lastMs / 60000), (unsigned long)((lastMs / 1000) % 60), f);
  cvs.drawString(tb, 120, 98);

  drawHints("[A] NEW RUN", "[B]HOLD1S SAVE");
  cvs.pushSprite(0, 0);
}

/* ================= 电量 ================= */
static void updateBattery(bool notify) {
#if HAS_PM1
  uint16_t mv = 0;
  if (pm1.readVbat(&mv) == M5PM1_OK && mv > 3000) {
    if      (mv >= 4150) batPct = 100;
    else if (mv <= 3300) batPct = 0;
    else                 batPct = (uint8_t)((mv - 3300) * 100 / 850);
  }
#endif
  if (pBat && notify) {
    pBat->setValue(&batPct, 1);
    pBat->notify();
  }
}

/* ================= 主流程 ================= */
void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(LCD_ROT);
  cvs.createSprite(240, 135);

  prefs.begin("recip", false);
  th    = prefs.getFloat("th", TH_DEFAULT);
  histN = prefs.getUShort("n", 0);

  /* 时区: 默认 +8 (东八区)。网页可用 {"cmd":"tzsync","v":480} 校正。 */
  tzOffsetS = (int32_t)prefs.getInt("tzmin", 8 * 60) * 60;
  tzFromNvs = (prefs.getInt("tzmin", -9999) != -9999);
  Serial.printf("[TIME] 时区偏移 %+d 分钟 (来源: %s), 默认东八区\n",
                (int)(tzOffsetS / 60), tzFromNvs ? "NVS" : "默认");
  loadTimebase();

  /* IMU 自检 + 诊断信息 */
  float ax, ay, az;
  imuOK = M5.Imu.getAccel(&ax, &ay, &az);
  dbgBoard = (int)M5.getBoard();
  dbgBmi   = M5.In_I2C.readRegister8(0x68, 0x00, 100000);   // BMI270 ID = 0x24
  dbgRtry  = 0;
  if (!imuOK) {
    /* 兜底: 外设全部就绪后, 显式重新初始化一次 BMI270 */
    dbgRtry = 1;
    delay(50);
    M5.Imu.begin(&M5.In_I2C, M5.getBoard());
    delay(50);
    imuOK = M5.Imu.getAccel(&ax, &ay, &az);
    if (!imuOK) dbgRtry = 2;                                // 2 = 重试后仍失败
  }
  Serial.printf("[IMU] %s  board=%d type=%d bmi270=0x%02X retry=%d\n",
                imuOK ? "OK" : "NOT FOUND", dbgBoard, (int)M5.Imu.getType(),
                dbgBmi, dbgRtry);

#if HAS_PM1
  /* M5PM1 与 M5Unified 共用 I2C (SDA=G47 SCL=G48) */
  if (pm1.begin(&M5.In_I2C, M5PM1_DEFAULT_ADDR, 100000) == M5PM1_OK) {
    updateBattery(false);
    Serial.println("[PM1] OK");
  }
#endif

  /* ---------- BLE ---------- */
  BLEDevice::init("StickS3-Recip");
  BLEDevice::setMTU(247);
  BLEServer*  server = BLEDevice::createServer();
  gServer = server;
  server->setCallbacks(new SrvCb());
  /* core 3.x 的 GATT app 注册(ESP_GATTS_REG_EVT)是异步的:
     不等它完成就创建服务, 第一个服务会静默失败(广播里有 UUID 但 GATT 里没有)。*/
  delay(500);

  /* 建三个服务: 先 16 位电量, 再自定义 128 位, 最后 16 位设备信息。
     这样从宿主机枚举结果就能判断"第几个服务没进 GATT 表"。 */
  BLEService* batSvc = server->createService(BLEUUID((uint16_t)0x180F));
  pBat = batSvc->createCharacteristic(
      BLEUUID((uint16_t)0x2A19),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBat->addDescriptor(new BLE2902());          // 必须: 否则无法订阅
  batSvc->start();

  BLEService* svc = server->createService(SERVICE_UUID);
  pStatus = svc->createCharacteristic(STATUS_UUID,
                                      BLECharacteristic::PROPERTY_NOTIFY);
  pStatus->addDescriptor(new BLE2902());
  /* WRITE + WRITE_NR 都给: 网页端 writeValueWithoutResponse 需要后者,
     否则浏览器会抛 NotSupportedError, 命令全部静默失败。 */
  BLECharacteristic* pCmd = svc->createCharacteristic(
      CMD_UUID, BLECharacteristic::PROPERTY_WRITE |
                BLECharacteristic::PROPERTY_WRITE_NR);
  pCmd->setCallbacks(new CmdCb());
  svc->start();

  BLEService* disSvc = server->createService(BLEUUID((uint16_t)0x180A));
  BLECharacteristic* pMan = disSvc->createCharacteristic(
      BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
  pMan->setValue("M5Stack");
  disSvc->start();

  /* GATT 句柄自检 (0xFFFF = 该服务未成功注册) */
  dbgBatH = batSvc->getHandle();
  dbgSvcH = svc->getHandle();
  dbgDisH = disSvc->getHandle();
  Serial.printf("[BLE] bat=0x%04X svc=0x%04X dis=0x%04X\n",
                dbgBatH, dbgSvcH, dbgDisH);

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->addServiceUUID(BLEUUID((uint16_t)0x180F));
  adv->addServiceUUID(BLEUUID((uint16_t)0x180A));
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] advertising as StickS3-Recip");

  drawIdle();
}

void loop() {
  M5.update();

  /* --- BLE 命令 (主循环上下文处理, 避免跨任务刷屏) --- */
  if (cmdFlag) {
    cmdFlag = false;
    handleCmd(String(cmdBuf));
  }

  /* --- 断开后延迟重启广播 --- */
  if (needAdv && !bleConn && millis() >= advAt) {
    needAdv = false;
    BLEDevice::startAdvertising();
    Serial.println("[BLE] re-advertising");
  }

  /* --- 按键 --- */
  if (M5.BtnA.wasPressed()) {
    if (st == S_RUN) { drawStop(); stopSession(); }   // 先画再停, 保持顺序无所谓
    else             startSession();
  }
  if (M5.BtnB.isPressed() && M5.BtnB.pressedFor(1000) && !bLongDone) {
    bLongDone = true;
    doArchive();
  }
  if (M5.BtnB.wasReleased()) bLongDone = false;

  /* --- IMU 往复检测 (仅运行中) --- */
  if (st == S_RUN && imuOK) {
    float ax, ay, az;
    if (M5.Imu.getAccel(&ax, &ay, &az)) {
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      float dev = fabsf(mag - 1.0f);          // 偏离 1g 的幅度
      if (armed) {
        if (dev >= th && millis() - lastTrig >= MIN_STROKE_MS) {
          cnt++;
          lastTrig = millis();
          armed = false;
        }
      } else if (dev <= th * 0.4f) {
        armed = true;                          // 迟滞回臂
      }
    }
  }

  /* --- 状态推送 + 刷屏 (200ms / 1s) --- */
  static uint32_t lastTick = 0, lastBat = 0, lastHb = 0;
  uint32_t now = millis();
  uint32_t period = (st == S_RUN) ? NOTIFY_RUN_MS : NOTIFY_IDLE_MS;
  if (now - lastTick >= period) {
    lastTick = now;
    sendStatus();
    if      (st == S_RUN)  drawRun();
    else if (st == S_STOP) drawStop();
    else                   drawIdle();       // 空闲时也低频刷新(电量/阈值)
  }
  if (now - lastBat >= BAT_PERIOD_MS) {
    lastBat = now;
    updateBattery(true);
  }
  /* 时间基准落盘 (每 5 分钟一次, 覆盖极小的 NVS 写)
     用途: 掉电重启后仍能外推出大致正确的时间戳 */
  static uint32_t lastTsave = 0;
  if (now - lastTsave >= 300000UL) {
    lastTsave = now;
    persistTimebase();
  }

  /* USB 心跳日志 (仅在串口终端连上时输出, 便于排查) */
  if (Serial && now - lastHb >= 3000) {
    lastHb = now;
    uint32_t e = nowEpoch();
    int Y, M, D, h, mi, se, wd;
    char tsb[24] = "--";
    if (e) { tsToLocal(e, tzOffsetS, Y, M, D, h, mi, se, wd);
             snprintf(tsb, sizeof(tsb), "%04d-%02d-%02d %02d:%02d:%02d", Y, M, D, h, mi, se); }
    Serial.printf("[HB] st=%d cnt=%lu ms=%lu ble=%d imu=%d heap=%u th=%.2f board=%d bmi=0x%02X svc=0x%04X bat=0x%04X\n",
                  (int)st, (unsigned long)cnt,
                  (unsigned long)(st == S_RUN ? millis() - t0 : lastMs),
                  (int)bleConn, (int)imuOK, ESP.getFreeHeap(), th, dbgBoard, dbgBmi,
                  dbgSvcH, dbgBatH);
    Serial.printf("[HB] clock=%s tz=%+dmin rtc=%d ts=%s\n",
                  clockSet ? "SET" : "NO", (int)(tzOffsetS / 60), (int)clockSet, tsb);
  }

  delay(2);                                  // ~300-500Hz 采样
}
