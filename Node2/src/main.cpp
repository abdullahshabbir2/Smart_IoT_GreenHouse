/**
 * ============================================================
 *  Smart Greenhouse — Node 2 : Water & Irrigation Management
 * ============================================================
 *  Monitors the water tank level and ambient rain conditions,
 *  then runs the irrigation pump on a configurable schedule —
 *  automatically skipping cycles when the tank is critically low
 *  or when rain makes watering unnecessary.
 *
 *  Hardware on this node
 *  ─────────────────────
 *    MCU          │ ESP32 (38-pin dev board)
 *    Level sensor │ Capacitive analog → GPIO 34 (ADC1)
 *    Rain sensor  │ Analog resistive  → GPIO 35 (ADC1)
 *    Water pump   │ Active-LOW relay  → GPIO 27
 *
 *  Web API
 *  ───────
 *    GET  /           → Live web dashboard (self-refreshing)
 *    GET  /api/state  → Full JSON snapshot
 *    POST /api/cmd    → Control, schedule, and calibration commands
 *
 *  Build & flash
 *  ─────────────
 *    pio run --target upload
 *    pio device monitor    (shows IP + current ADC readings for calibration)
 *
 *  Author  : Abdullah Shabbir
 *  Version : 1.0.0
 *  Target  : ESP32 Arduino core ≥ 2.x  |  PlatformIO espressif32
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>    // persistent storage for calibration and config
#include <ESPmDNS.h>        // let me reach the board as greenhouse-node2.local
#include <esp_task_wdt.h>   // watchdog so a stuck loop causes a reboot rather than a fried pump

// ============================================================
//  USER CONFIGURATION
// ============================================================
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"

// When you don't have the real sensors hooked up, set this to true.
// It makes the board spit out plausible values for testing.
#define DEMO_SENSOR_VALUES  true

#define PIN_LEVEL_ADC    34   // tank level input (ADC1; ADC2 doesn't work with WiFi)
#define PIN_RAIN_ADC     35   // rain sensor input (ADC1)
#define PIN_PUMP_RELAY   27   // relay control pin

#define RELAY_ACTIVE_LOW  true

// ============================================================
//  FIRMWARE IDENTITY
// ============================================================
#define FW_VERSION   "1.0.0"
#define FW_NODE_ID   "node2"
#define MDNS_HOST    "greenhouse-node2"

// ============================================================
//  TIMING
// ============================================================
#define SENSOR_INTERVAL_MS  1000UL
#define WDT_TIMEOUT_S       8

// ============================================================
//  DEFAULT ADC CALIBRATION
//  Do your first calibration from the serial logs.
//  Empty the tank, note the level ADC. Fill it, note the full ADC.
//  You can also tweak these live over /api/cmd later.
// ============================================================
#define DEFAULT_LEVEL_EMPTY_ADC   600
#define DEFAULT_LEVEL_FULL_ADC    3500
#define DEFAULT_RAIN_DRY_ADC      3000
#define DEFAULT_RAIN_WET_ADC      1200

// ============================================================
//  DEFAULT SAFETY & SCHEDULE THRESHOLDS
// ============================================================
#define DEFAULT_LOW_WATER_PCT     20    // pump locked out at or below this %
#define DEFAULT_RAIN_DETECT_PCT   60    // irrigation blocked above this wetness %
#define DEFAULT_RUN_SECONDS       10    // how long the pump runs per auto cycle
#define DEFAULT_PERIOD_MINUTES    30    // minutes between auto irrigation cycles

// ============================================================
//  CONTROL STATE
// ============================================================
enum Mode : uint8_t { MODE_AUTO = 0, MODE_MANUAL = 1 };
Mode controlMode = MODE_AUTO;

bool pumpManual = false;
bool pumpState  = false;

// ============================================================
//  SENSOR READINGS & DERIVED FLAGS
// ============================================================
int  levelAdc    = 0;
int  rainAdc     = 0;
int  tankPct     = 0;
int  rainWetPct  = 0;

bool tankLow      = false;
bool rainDetected = false;

// ============================================================
//  RUNTIME-CONFIGURABLE VALUES  (all live in NVS)
// ============================================================
int levelEmptyAdc  = DEFAULT_LEVEL_EMPTY_ADC;
int levelFullAdc   = DEFAULT_LEVEL_FULL_ADC;
int rainDryAdc     = DEFAULT_RAIN_DRY_ADC;
int rainWetAdc     = DEFAULT_RAIN_WET_ADC;
int lowWaterPct    = DEFAULT_LOW_WATER_PCT;
int rainDetectPct  = DEFAULT_RAIN_DETECT_PCT;
int runSeconds     = DEFAULT_RUN_SECONDS;
int periodMinutes  = DEFAULT_PERIOD_MINUTES;

// ============================================================
//  IRRIGATION SCHEDULE TRACKING
// ============================================================
uint32_t lastAutoRunMs = 0;
uint32_t pumpStopMs    = 0;
uint32_t totalRunCount = 0;

// ============================================================
//  SYSTEM OBJECTS
// ============================================================
WebServer   server(80);
Preferences prefs;
uint32_t    bootMs = 0;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
static void     setPump(bool on);
void            loadPrefs();
void            savePrefs();
static int      adcToPercent(int adc, int adcAt0, int adcAt100);
static int      percentToAdc(int pct, int adcAt0, int adcAt100);
void            readSensors();
static bool     irrigationAllowed();
void            applyControl();
String          buildStateJson();
void            handleRoot();
void            handleApiState();
void            handleApiCmd();

// ============================================================
//  RELAY HELPER
// ============================================================

static void setPump(bool on) {
  bool pinHigh = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_PUMP_RELAY, pinHigh ? HIGH : LOW);
}

// ============================================================
//  NVS PERSISTENCE
// ============================================================

void loadPrefs() {
  prefs.begin(FW_NODE_ID, /*readOnly=*/true);
  levelEmptyAdc = prefs.getInt("lev_empty", DEFAULT_LEVEL_EMPTY_ADC);
  levelFullAdc  = prefs.getInt("lev_full",  DEFAULT_LEVEL_FULL_ADC);
  rainDryAdc    = prefs.getInt("rain_dry",  DEFAULT_RAIN_DRY_ADC);
  rainWetAdc    = prefs.getInt("rain_wet",  DEFAULT_RAIN_WET_ADC);
  lowWaterPct   = prefs.getInt("low_water", DEFAULT_LOW_WATER_PCT);
  rainDetectPct = prefs.getInt("rain_det",  DEFAULT_RAIN_DETECT_PCT);
  runSeconds    = prefs.getInt("run_s",     DEFAULT_RUN_SECONDS);
  periodMinutes = prefs.getInt("period_m",  DEFAULT_PERIOD_MINUTES);
  prefs.end();
  Serial.println("[PREFS] Config loaded from NVS.");
}

void savePrefs() {
  prefs.begin(FW_NODE_ID, /*readOnly=*/false);
  prefs.putInt("lev_empty", levelEmptyAdc);
  prefs.putInt("lev_full",  levelFullAdc);
  prefs.putInt("rain_dry",  rainDryAdc);
  prefs.putInt("rain_wet",  rainWetAdc);
  prefs.putInt("low_water", lowWaterPct);
  prefs.putInt("rain_det",  rainDetectPct);
  prefs.putInt("run_s",     runSeconds);
  prefs.putInt("period_m",  periodMinutes);
  prefs.end();
  Serial.println("[PREFS] Config saved to NVS.");
}

// ============================================================
//  ANALOG -> PERCENTAGE MAPPING
// ============================================================

/**
 * Turn a raw ADC number into a nice 0-100% value.
 * Works for both sensor types, even if one goes up and the other goes down.
 * The trick is just checking which endpoint is larger and scaling accordingly.
 */
static int adcToPercent(int adc, int adcAt0, int adcAt100) {
  if (adcAt100 == adcAt0) return 0;
  float pct = (float)(adc - adcAt0) * 100.0f / (float)(adcAt100 - adcAt0);
  int p = (int)roundf(pct);
  return p < 0 ? 0 : (p > 100 ? 100 : p);
}

static int percentToAdc(int pct, int adcAt0, int adcAt100) {
  int p = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
  return adcAt0 + (int)roundf((float)(adcAt100 - adcAt0) * (float)p / 100.0f);
}

// ============================================================
//  SENSOR READING
// ============================================================

void readSensors() {
  if (DEMO_SENSOR_VALUES) {
    tankPct    = random(54, 97);
    rainWetPct = random(0, 38);
    levelAdc   = percentToAdc(tankPct,    levelEmptyAdc, levelFullAdc);
    rainAdc    = percentToAdc(rainWetPct, rainDryAdc,    rainWetAdc);
  } else {
    levelAdc = analogRead(PIN_LEVEL_ADC);
    rainAdc  = analogRead(PIN_RAIN_ADC);

    tankPct    = adcToPercent(levelAdc, levelEmptyAdc, levelFullAdc);
    rainWetPct = adcToPercent(rainAdc,  rainDryAdc,    rainWetAdc);
  }

  tankLow      = (tankPct    <= lowWaterPct);
  rainDetected = (rainWetPct >= rainDetectPct);
}

// ============================================================
//  IRRIGATION SAFETY CHECK
// ============================================================

/**
 * Only say yes if the pump can safely run right now.
 * If the tank is too low or the rain sensor is wet, we keep the pump off.
 */
static bool irrigationAllowed() {
  if (tankLow)      return false;
  if (rainDetected) return false;
  return true;
}

// ============================================================
//  PUMP CONTROL LOGIC
// ============================================================

/**
 * Decide whether the pump should be on or off and actually flip the relay.
 *
 * Safety is always the highest priority here. If the tank is too low,
 * we shut the pump down even if the user asked for it manually.
 *
 * AUTO mode just runs the pump on a timer. No fancy moisture logic,
 * just a regular interval and a runtime-configurable duration.
 */
void applyControl() {
  uint32_t now = millis();

  // --- Hard safety lockout ---
  if (tankLow) {
    if (pumpState) {
      pumpState = false;
      setPump(false);
      pumpStopMs = 0;
      Serial.println("[PUMP] OFF — tank critically low (safety lockout)");
    }
    return;
  }

  // --- Manual mode ---
  if (controlMode == MODE_MANUAL) {
    if (pumpManual != pumpState) {
      pumpState = pumpManual;
      setPump(pumpState);
      Serial.printf("[PUMP] → %s (manual)\n", pumpState ? "ON" : "OFF");
    }
    return;
  }

  // --- Auto mode: schedule-based ---

  // Stop if the allocated run time has elapsed
  if (pumpState && pumpStopMs != 0 && (int32_t)(now - pumpStopMs) >= 0) {
    pumpState  = false;
    pumpStopMs = 0;
    setPump(false);
    Serial.printf("[PUMP] OFF — auto cycle done (total cycles: %lu)\n", totalRunCount);
  }

  // Stop if rain starts mid-cycle
  if (pumpState && !irrigationAllowed()) {
    pumpState  = false;
    pumpStopMs = 0;
    setPump(false);
    Serial.println("[PUMP] OFF — conditions changed mid-cycle");
  }

  // Start a new cycle when the period has elapsed and conditions allow
  if (!pumpState && irrigationAllowed()) {
    const uint32_t periodMs = (uint32_t)periodMinutes * 60UL * 1000UL;
    const uint32_t runMs    = (uint32_t)runSeconds    * 1000UL;

    bool periodElapsed = (lastAutoRunMs == 0) || ((now - lastAutoRunMs) >= periodMs);
    if (periodElapsed) {
      pumpState     = true;
      pumpStopMs    = now + runMs;
      lastAutoRunMs = now;
      totalRunCount++;
      setPump(true);
      Serial.printf("[PUMP] ON  — auto cycle #%lu (%ds, tank %d%%)\n",
        totalRunCount, runSeconds, tankPct);
    }
  }
}

// ============================================================
//  JSON STATE BUILDER
// ============================================================

String buildStateJson() {
  StaticJsonDocument<768> doc;

  doc["node"]     = FW_NODE_ID;
  doc["version"]  = FW_VERSION;
  doc["ip"]       = WiFi.localIP().toString();
  doc["rssi"]     = WiFi.RSSI();
  doc["uptime_s"] = (millis() - bootMs) / 1000UL;

  doc["mode"]     = (controlMode == MODE_AUTO) ? "auto" : "manual";
  doc["pump"]     = pumpState    ? 1 : 0;
  doc["tank_low"] = tankLow      ? 1 : 0;
  doc["rain"]     = rainDetected ? 1 : 0;

  if (controlMode == MODE_AUTO && lastAutoRunMs > 0 && !pumpState) {
    uint32_t elapsed  = millis() - lastAutoRunMs;
    uint32_t periodMs = (uint32_t)periodMinutes * 60UL * 1000UL;
    doc["next_cycle_s"] = (elapsed < periodMs) ? (periodMs - elapsed) / 1000UL : 0;
  }
  doc["total_cycles"] = totalRunCount;

  JsonObject s = doc.createNestedObject("sensors");
  s["level_adc"]    = levelAdc;
  s["tank_pct"]     = tankPct;
  s["rain_adc"]     = rainAdc;
  s["rain_wet_pct"] = rainWetPct;

  JsonObject cfg = doc.createNestedObject("config");
  cfg["low_water_pct"]   = lowWaterPct;
  cfg["rain_detect_pct"] = rainDetectPct;
  cfg["run_seconds"]     = runSeconds;
  cfg["period_minutes"]  = periodMinutes;

  JsonObject cal = cfg.createNestedObject("calibration");
  cal["level_empty_adc"] = levelEmptyAdc;
  cal["level_full_adc"]  = levelFullAdc;
  cal["rain_dry_adc"]    = rainDryAdc;
  cal["rain_wet_adc"]    = rainWetAdc;

  String out;
  serializeJson(doc, out);
  return out;
}

// ============================================================
//  WEB DASHBOARD
// ============================================================

void handleRoot() {
  static const char html[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Greenhouse — Node 2</title>
  <style>
    :root{--bg:#0b1220;--card:#101a33;--muted:#93a4c7;--txt:#e8eeff;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--line:#223055;}
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:linear-gradient(180deg,#070c16 0%,#0b1220 100%);color:var(--txt);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;min-height:100vh}
    header{padding:16px 20px 12px;border-bottom:1px solid var(--line);position:sticky;top:0;background:rgba(11,18,32,.92);backdrop-filter:blur(10px);z-index:10}
    .header-top{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
    h1{font-size:18px;font-weight:700;letter-spacing:-.2px}
    .ver{font-size:11px;color:var(--muted);margin-left:8px;font-weight:400}
    .sub{margin-top:8px;color:var(--muted);font-size:12px;display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .pill{padding:4px 10px;border:1px solid var(--line);border-radius:999px;font-size:12px;white-space:nowrap}
    .pill.ok{border-color:rgba(34,197,94,.4);color:#b9f6cc}
    .pill.warn{border-color:rgba(245,158,11,.4);color:#ffe2b6}
    .pill.bad{border-color:rgba(239,68,68,.4);color:#ffc3c3}
    .dot{width:7px;height:7px;border-radius:50%;display:inline-block;margin-right:4px}
    .dot.live{background:#22c55e;box-shadow:0 0 6px #22c55e}.dot.dead{background:#ef4444}
    main{padding:16px 20px;max-width:1020px;margin:0 auto}
    .grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}
    .card{background:rgba(16,26,51,.85);border:1px solid var(--line);border-radius:16px;padding:16px;box-shadow:0 10px 30px rgba(0,0,0,.3)}
    .span6{grid-column:span 6}.span4{grid-column:span 4}.span8{grid-column:span 8}.span12{grid-column:span 12}
    @media(max-width:860px){.span6,.span4,.span8{grid-column:span 12}}
    .row{display:flex;justify-content:space-between;gap:10px;align-items:center}
    .clabel{font-size:13px;color:var(--muted);letter-spacing:.2px;margin-bottom:6px}
    .big{font-size:30px;font-weight:800;line-height:1}
    .unit{font-size:14px;color:var(--muted);margin-left:5px;font-weight:400}
    .hint{font-size:12px;color:var(--muted);margin-top:8px;line-height:1.5}
    .btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}
    button{background:#0e1a38;border:1px solid var(--line);color:var(--txt);padding:10px 14px;border-radius:12px;font-weight:600;cursor:pointer;font-size:13px}
    button:hover{border-color:#3a5490}
    button.primary{background:#132756;border-color:#1e3d7a}
    .toggle{width:56px;height:30px;border-radius:999px;border:1px solid var(--line);background:#0e1a38;position:relative;cursor:pointer;flex-shrink:0}
    .knob{width:24px;height:24px;border-radius:50%;background:#6b7fa8;position:absolute;top:2px;left:2px;transition:.2s}
    .toggle.on{background:rgba(34,197,94,.15);border-color:rgba(34,197,94,.4)}
    .toggle.on .knob{left:28px;background:#86efac}
    .gauge{width:120px;height:120px;border-radius:50%;background:conic-gradient(#22c55e 0deg,var(--line) 0deg);display:grid;place-items:center;flex-shrink:0;transition:background .4s}
    .gauge .inner{width:90px;height:90px;border-radius:50%;background:radial-gradient(circle at 30% 20%,rgba(255,255,255,.07),rgba(0,0,0,.2)),var(--card);border:1px solid rgba(255,255,255,.06);display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px}
    .gauge .gval{font-size:21px;font-weight:800;line-height:1}
    .gauge .glab{font-size:11px;color:var(--muted)}
    .cfg-grid{display:grid;grid-template-columns:1fr 1fr;gap:4px 14px;margin-top:8px}
    .cfg-row{font-size:12px;color:var(--muted)}
    .cfg-row span{color:var(--txt);font-weight:600}
    .progress-bar{height:6px;border-radius:3px;background:var(--line);margin-top:10px;overflow:hidden}
    .progress-fill{height:100%;border-radius:3px;background:#22c55e;transition:width .5s}
    pre{white-space:pre-wrap;word-break:break-all;color:#cfe0ff;font-size:11.5px;line-height:1.6}
    footer{text-align:center;color:var(--muted);font-size:11px;padding:20px;margin-top:8px}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace}
    #lastCmd{font-size:11px;color:var(--muted);margin-top:8px}
  </style>
</head>
<body>
<header>
  <div class="header-top">
    <h1>Smart Greenhouse <span style="color:var(--muted)">·</span> Node 2 <span class="ver" id="verBadge"></span></h1>
    <span id="connBadge"><span class="dot dead" id="connDot"></span><span id="connTxt">Connecting…</span></span>
  </div>
  <div class="sub">
    <span class="pill" id="pillMode">Mode: —</span>
    <span class="pill" id="pillPump">Pump: —</span>
    <span class="pill" id="pillTank">Tank: —</span>
    <span class="pill" id="pillRain">Rain: —</span>
    <span class="pill mono" id="pillNet">IP: — | RSSI: —</span>
    <span class="pill mono" id="pillUptime">Uptime: —</span>
  </div>
</header>
<main>
  <div class="grid">
    <div class="card span6">
      <div class="clabel">Water Tank Level</div>
      <div class="row">
        <div>
          <div><span class="big" id="tankVal">—</span><span class="unit">%</span></div>
          <div class="hint" id="tankHint">Waiting…</div>
          <div class="progress-bar"><div class="progress-fill" id="tankBar" style="width:0%"></div></div>
        </div>
        <div class="gauge" id="tankGauge"><div class="inner"><div class="gval" id="tankGV">—</div><div class="glab">%</div></div></div>
      </div>
    </div>
    <div class="card span6">
      <div class="clabel">Rain / Moisture Sensor</div>
      <div class="row">
        <div>
          <div><span class="big" id="rainVal">—</span><span class="unit">wet%</span></div>
          <div class="hint" id="rainHint">Waiting…</div>
          <div class="progress-bar"><div class="progress-fill" id="rainBar" style="width:0%;background:#38bdf8"></div></div>
        </div>
        <div class="gauge" id="rainGauge"><div class="inner"><div class="gval" id="rainGV">—</div><div class="glab">wet%</div></div></div>
      </div>
    </div>
    <div class="card span4">
      <div class="clabel">Irrigation Schedule</div>
      <div class="cfg-grid">
        <div class="cfg-row">Run for <span id="cfg_run">—</span> s</div>
        <div class="cfg-row">Every <span id="cfg_period">—</span> min</div>
        <div class="cfg-row">Cycles run <span id="cfg_cycles">—</span></div>
        <div class="cfg-row">Next in <span id="cfg_next">—</span></div>
      </div>
    </div>
    <div class="card span8">
      <div class="row">
        <div>
          <div class="clabel">Pump Control</div>
          <div class="hint">Auto runs pump on schedule. Manual is direct control.<br>Tank-low lockout is always active regardless of mode.</div>
        </div>
        <div class="toggle" id="pumpToggle" onclick="togglePump()"><div class="knob"></div></div>
      </div>
      <div class="btns">
        <button class="primary" onclick="sendCmd({mode:'auto'})">Auto Mode</button>
        <button onclick="sendCmd({mode:'manual'})">Manual Mode</button>
        <button onclick="sendCmd({mode:'manual',pump:1})">Pump ON</button>
        <button onclick="sendCmd({mode:'manual',pump:0})">Pump OFF</button>
      </div>
      <div id="lastCmd" class="mono">—</div>
    </div>
    <div class="card span12">
      <div class="row" style="margin-bottom:10px">
        <div class="clabel">Raw JSON (/api/state)</div>
        <button onclick="refresh()">Refresh</button>
      </div>
      <pre class="mono" id="json">Loading…</pre>
    </div>
  </div>
</main>
<footer>Smart Greenhouse Node 2 — ESP32 firmware <span id="footVer"></span></footer>
<script>
  const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
  function setGauge(el,v,min,max,c="#22c55e"){
    if(v==null||Number.isNaN(+v))return;
    const deg=Math.round(clamp((+v-min)/(max-min),0,1)*270);
    el.style.background=`conic-gradient(${c} 0deg ${deg}deg,#223055 ${deg}deg 360deg)`;
  }
  function fmtUptime(s){const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;return h>0?`${h}h ${m}m ${sec}s`:m>0?`${m}m ${sec}s`:`${sec}s`;}
  function setPill(id,txt,cls){const e=document.getElementById(id);e.textContent=txt;e.className='pill '+cls;}
  async function refresh(){
    try{
      const r=await fetch('/api/state',{cache:'no-store'});
      if(!r.ok)throw new Error(r.status);
      const j=await r.json();
      document.getElementById('connDot').className='dot live';
      document.getElementById('connTxt').textContent='Live';
      document.getElementById('json').textContent=JSON.stringify(j,null,2);
      const ver=j.version??'';
      document.getElementById('verBadge').textContent=ver?`v${ver}`:'';
      document.getElementById('footVer').textContent=ver?`v${ver}`:'';
      const pumpOn=j.pump===1,tankLow=j.tank_low===1,rain=j.rain===1,mode=j.mode??'—';
      setPill('pillMode',`Mode: ${mode}`,mode==='auto'?'ok':'warn');
      setPill('pillPump',`Pump: ${pumpOn?'ON':'OFF'}`,pumpOn?'ok':'warn');
      setPill('pillTank',tankLow?'Tank LOW':'Tank OK',tankLow?'bad':'ok');
      setPill('pillRain',rain?'Rain Detected':'No Rain',rain?'warn':'ok');
      document.getElementById('pillNet').textContent=`IP: ${j.ip??'—'}  |  RSSI: ${j.rssi??'—'} dBm`;
      document.getElementById('pillUptime').textContent=`Uptime: ${j.uptime_s!=null?fmtUptime(j.uptime_s):'—'}`;
      const tank=j?.sensors?.tank_pct,wet=j?.sensors?.rain_wet_pct,cfg=j?.config??{};
      document.getElementById('tankVal').textContent=tank!=null?tank:'—';
      document.getElementById('tankGV').textContent=tank!=null?tank:'—';
      document.getElementById('tankBar').style.width=(tank??0)+'%';
      document.getElementById('tankBar').style.background=tankLow?'#ef4444':tank<40?'#f59e0b':'#22c55e';
      setGauge(document.getElementById('tankGauge'),tank,0,100,tankLow?'#ef4444':tank<40?'#f59e0b':'#22c55e');
      document.getElementById('rainVal').textContent=wet!=null?wet:'—';
      document.getElementById('rainGV').textContent=wet!=null?wet:'—';
      document.getElementById('rainBar').style.width=(wet??0)+'%';
      setGauge(document.getElementById('rainGauge'),wet,0,100,'#38bdf8');
      document.getElementById('tankHint').textContent=tankLow?'⚠ Tank critically low — pump locked out':tank<40?'Water getting low, consider refilling':'Water level is healthy';
      document.getElementById('rainHint').textContent=rain?'Rain detected — irrigation paused':wet>30?'Ground is damp':'Dry — irrigation can proceed';
      document.getElementById('cfg_run').textContent=cfg.run_seconds??'—';
      document.getElementById('cfg_period').textContent=cfg.period_minutes??'—';
      document.getElementById('cfg_cycles').textContent=j.total_cycles??'—';
      const nextS=j.next_cycle_s;
      document.getElementById('cfg_next').textContent=pumpOn?'running now':nextS!=null?fmtUptime(nextS):'—';
      const t=document.getElementById('pumpToggle');
      pumpOn?t.classList.add('on'):t.classList.remove('on');
    }catch(e){
      document.getElementById('connDot').className='dot dead';
      document.getElementById('connTxt').textContent='Offline';
      document.getElementById('json').textContent='Connection error: '+e.message;
    }
  }
  async function sendCmd(obj){
    document.getElementById('lastCmd').textContent='Sent: '+JSON.stringify(obj);
    try{await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});}
    catch(e){document.getElementById('lastCmd').textContent='Error: '+e.message;}
    setTimeout(refresh,300);
  }
  function togglePump(){const on=!document.getElementById('pumpToggle').classList.contains('on');sendCmd({mode:'manual',pump:on?1:0});}
  setInterval(refresh,2000);refresh();
</script>
</body>
</html>
)HTML";

  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", html);
}

// ============================================================
//  API HANDLERS
// ============================================================

void handleApiState() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buildStateJson());
}

void handleApiCmd() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"request body missing\"}");
    return;
  }

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"invalid JSON\"}");
    return;
  }

  if (doc.containsKey("mode")) {
    const char* m = doc["mode"];
    if      (strcmp(m, "auto")   == 0) controlMode = MODE_AUTO;
    else if (strcmp(m, "manual") == 0) controlMode = MODE_MANUAL;
    Serial.printf("[CMD] mode → %s\n", m);
  }

  if (doc.containsKey("pump")) {
    pumpManual = (doc["pump"].as<int>() == 1);
    Serial.printf("[CMD] pump_manual → %d\n", pumpManual ? 1 : 0);
  }

  if (doc.containsKey("set")) {
    JsonObject s = doc["set"];
    bool changed = false;
    if (s.containsKey("run_seconds"))     { runSeconds    = (int)s["run_seconds"];    changed = true; }
    if (s.containsKey("period_minutes"))  { periodMinutes = (int)s["period_minutes"]; changed = true; }
    if (s.containsKey("low_water_pct"))   { lowWaterPct   = (int)s["low_water_pct"];  changed = true; }
    if (s.containsKey("rain_detect_pct")) { rainDetectPct = (int)s["rain_detect_pct"];changed = true; }
    if (s.containsKey("level_empty_adc")) { levelEmptyAdc = (int)s["level_empty_adc"];changed = true; }
    if (s.containsKey("level_full_adc"))  { levelFullAdc  = (int)s["level_full_adc"]; changed = true; }
    if (s.containsKey("rain_dry_adc"))    { rainDryAdc    = (int)s["rain_dry_adc"];   changed = true; }
    if (s.containsKey("rain_wet_adc"))    { rainWetAdc    = (int)s["rain_wet_adc"];   changed = true; }
    if (changed) savePrefs();
  }

  applyControl();
  server.send(200, "application/json", buildStateJson());
}

// ============================================================
//  SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n\n[BOOT] Smart Greenhouse Node 2  v%s\n", FW_VERSION);

  esp_task_wdt_init(WDT_TIMEOUT_S, /*panic=*/true);
  esp_task_wdt_add(NULL);

  digitalWrite(PIN_PUMP_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(PIN_PUMP_RELAY, OUTPUT);

  analogReadResolution(12);  // force 12-bit ADC readings so values are stable

  loadPrefs();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WIFI] Connecting");
  uint32_t wStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wStart < 20000) {
    esp_task_wdt_reset();
    delay(400);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WIFI] Connected — IP: %s  RSSI: %d dBm\n",
      WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[WARN] WiFi timed out — retrying in background.");
  }

  if (MDNS.begin(MDNS_HOST))
    Serial.printf("[mDNS] Browse to %s.local\n", MDNS_HOST);

  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/api/state", HTTP_GET,  handleApiState);
  server.on("/api/cmd",   HTTP_POST, handleApiCmd);
  server.begin();
  Serial.println("[HTTP] Server started on port 80.");

  // Print current calibration values so the user can verify them
  Serial.printf("[CAL]  Tank  empty=%d  full=%d\n", levelEmptyAdc, levelFullAdc);
  Serial.printf("[CAL]  Rain  dry=%d    wet=%d\n",  rainDryAdc,    rainWetAdc);

  bootMs = millis();
  readSensors();
  applyControl();
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();

  static uint32_t lastRead = 0;
  uint32_t now = millis();
  if (now - lastRead >= SENSOR_INTERVAL_MS) {
    lastRead = now;
    readSensors();
    applyControl();
  }
}
