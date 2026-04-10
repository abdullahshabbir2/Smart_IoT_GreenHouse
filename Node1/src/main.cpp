/**
 * ============================================================
 *  Smart Greenhouse — Node 1 : Climate & Air Management
 * ============================================================
 *  Keeps the greenhouse atmosphere healthy.
 *  I watch temp, humidity, light, and pressure, then run the fan
 *  whenever the air needs a reset.
 *
 *  Hardware on this node
 *  ─────────────────────
 *    MCU     │ ESP32 (38-pin dev board)
 *    DHT22   │ Temperature + Humidity  → GPIO 4
 *    BH1750  │ Ambient light (lux)     → I2C (SDA=21, SCL=22)
 *    BMP180  │ Pressure + backup temp  → I2C (shared bus, optional)
 *    Fan     │ Active-LOW relay module → GPIO 26
 *
 *  Web API
 *  ───────
 *    GET  /           → Live web dashboard (self-refreshing)
 *    GET  /api/state  → Full JSON snapshot of sensor + control state
 *    POST /api/cmd    → JSON command — change mode, fan, or thresholds
 *
 *  POST /api/cmd  examples
 *  ───────────────────────
 *    Switch to auto mode          : {"mode":"auto"}
 *    Manual fan on                : {"mode":"manual","fan":1}
 *    Adjust temperature threshold : {"set":{"temp_on":30.0,"temp_off":27.5}}
 *
 *  Build & flash
 *  ─────────────
 *    pio run --target upload        (from the Node1/ directory)
 *    pio device monitor             (to see the assigned IP address)
 *
 *  Author  : Abdullah Shabbir
 *  Version : 1.0.0
 *  Target  : ESP32 Arduino core ≥ 2.x  |  PlatformIO espressif32
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>    // save calibration/config so I don't lose it on power off
#include <ESPmDNS.h>        // let me reach the board at greenhouse-node1.local
#include <esp_task_wdt.h>   // watchdog so a stuck loop reboots instead of frying the fan

#include <DHT.h>
#include <BH1750.h>
#include <Adafruit_BMP085.h>    // supports both BMP085 and BMP180


// ============================================================
//  USER CONFIGURATION
// ============================================================
#define WIFI_SSID     "YOUR_WIFI_SSID"      // network name
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"  // WiFi password

// If the sensors aren't connected yet, leave this true and the board
// will fake believable readings for the dashboard.
#define DEMO_SENSOR_VALUES  true

// GPIO assignments
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_DHT         4
#define PIN_FAN_RELAY  26

// Most cheap relay boards trigger on LOW — flip to false if yours is reversed
#define RELAY_ACTIVE_LOW  true

// ============================================================
//  FIRMWARE IDENTITY
// ============================================================
#define FW_VERSION    "1.0.0"
#define FW_NODE_ID    "node1"
#define MDNS_HOST     "greenhouse-node1"

// ============================================================
//  TIMING
// ============================================================
#define SENSOR_INTERVAL_MS  2000UL   // DHT22 needs at least 2 s between reads
#define WDT_TIMEOUT_S       8        // watchdog reboot timeout

// ============================================================
//  DEFAULT THRESHOLDS
//  First-boot values. After a /api/cmd "set" command they come
//  from NVS flash and survive reboots automatically.
// ============================================================
#define DEFAULT_TEMP_ON     28.0f   // °C — fan turns ON  above this
#define DEFAULT_TEMP_OFF    26.5f   // °C — fan turns OFF below this
#define DEFAULT_HUM_ON      75.0f   // %  — fan turns ON  above this
#define DEFAULT_HUM_OFF     70.0f   // %  — fan turns OFF below this
#define DEFAULT_LUX_DAY     60.0f   // lux — above this = daytime
#define DEFAULT_LUX_NIGHT   20.0f   // lux — below this = nighttime

// ============================================================
//  CONTROL STATE
// ============================================================
enum Mode : uint8_t { MODE_AUTO = 0, MODE_MANUAL = 1 };
Mode controlMode = MODE_AUTO;

struct Thresholds {
  float temp_on   = DEFAULT_TEMP_ON;
  float temp_off  = DEFAULT_TEMP_OFF;
  float hum_on    = DEFAULT_HUM_ON;
  float hum_off   = DEFAULT_HUM_OFF;
  float lux_day   = DEFAULT_LUX_DAY;
  float lux_night = DEFAULT_LUX_NIGHT;
} th;

bool fanManual = false;
bool fanState  = false;
bool isDay     = false;

// ============================================================
//  SENSOR OBJECTS & LIVE READINGS
// ============================================================
DHT             dht(PIN_DHT, DHT22);
BH1750          lightMeter;
Adafruit_BMP085 bmp;

bool bh_ok  = false;
bool bmp_ok = false;

float tempC      = NAN;
float humPct     = NAN;
float luxVal     = NAN;
float bmpTempC   = NAN;
float pressurePa = NAN;

// ============================================================
//  SYSTEM OBJECTS
// ============================================================
WebServer   server(80);
Preferences prefs;
uint32_t    bootMs = 0;

// ============================================================
//  FORWARD DECLARATIONS
//  PlatformIO needs explicit prototypes, so I list them instead of
//  relying on the Arduino IDE's implicit behavior.
// ============================================================
static void setFan(bool on);
float       randomFloat(float minVal, float maxVal);
void        loadPrefs();
void        savePrefs();
void        readSensors();
void        applyAutomation();
String      buildStateJson();
void        handleRoot();
void        handleApiState();
void        handleApiCmd();

// ============================================================
//  RELAY HELPER
// ============================================================

/**
 * Turn the fan on or off, while hiding the relay wiring detail.
 * If RELAY_ACTIVE_LOW is true, the pin value is inverted here.
 */
static void setFan(bool on) {
  bool pinHigh = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_FAN_RELAY, pinHigh ? HIGH : LOW);
}

float randomFloat(float minVal, float maxVal) {
  return minVal + (float)random(0, 10001) * (maxVal - minVal) / 10000.0f;
}

// ============================================================
//  NVS PERSISTENCE
// ============================================================

/**
 * Load the saved thresholds from NVS.
 * If this is the first boot, the defaults above are used instead.
 */
void loadPrefs() {
  prefs.begin(FW_NODE_ID, /*readOnly=*/true);
  th.temp_on   = prefs.getFloat("temp_on",   DEFAULT_TEMP_ON);
  th.temp_off  = prefs.getFloat("temp_off",  DEFAULT_TEMP_OFF);
  th.hum_on    = prefs.getFloat("hum_on",    DEFAULT_HUM_ON);
  th.hum_off   = prefs.getFloat("hum_off",   DEFAULT_HUM_OFF);
  th.lux_day   = prefs.getFloat("lux_day",   DEFAULT_LUX_DAY);
  th.lux_night = prefs.getFloat("lux_night", DEFAULT_LUX_NIGHT);
  prefs.end();
  Serial.println("[PREFS] Thresholds loaded from NVS.");
}

/**
 * Save the current thresholds to NVS so they stick across reboots.
 * This is what keeps your tuning from disappearing after power loss.
 */
void savePrefs() {
  prefs.begin(FW_NODE_ID, /*readOnly=*/false);
  prefs.putFloat("temp_on",   th.temp_on);
  prefs.putFloat("temp_off",  th.temp_off);
  prefs.putFloat("hum_on",    th.hum_on);
  prefs.putFloat("hum_off",   th.hum_off);
  prefs.putFloat("lux_day",   th.lux_day);
  prefs.putFloat("lux_night", th.lux_night);
  prefs.end();
  Serial.println("[PREFS] Thresholds saved to NVS.");
}

// ============================================================
//  SENSOR READING
// ============================================================

/**
 * Read the sensors and update the current values.
 * If a sensor hiccups, keep the last good reading for one cycle.
 * DHT22 isn't perfect, so this avoids the fan bouncing on a single bad read.
 */
void readSensors() {
  if (DEMO_SENSOR_VALUES) {
    bh_ok      = true;
    bmp_ok     = true;
    tempC      = randomFloat(22.0f, 33.5f);
    humPct     = randomFloat(46.0f, 82.0f);
    luxVal     = randomFloat(180.0f, 1650.0f);
    bmpTempC   = tempC + randomFloat(-0.4f, 0.4f);
    pressurePa = randomFloat(100600.0f, 102300.0f);
  } else {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      tempC  = t;
      humPct = h;
    }

    if (bh_ok) {
      float l = lightMeter.readLightLevel();
      if (l >= 0.0f) luxVal = l;
    }

    if (bmp_ok) {
      bmpTempC   = bmp.readTemperature();
      pressurePa = (float)bmp.readPressure();
    }
  }

  // Day/night hysteresis — two different thresholds prevent "flickering"
  // at dawn and dusk when light levels are borderline
  if (!isnan(luxVal)) {
    if (!isDay && luxVal >= th.lux_day)   isDay = true;
    if ( isDay && luxVal <= th.lux_night) isDay = false;
  }
}

// ============================================================
//  AUTOMATION LOGIC
// ============================================================

/**
 * Decide whether the fan should be on and write the relay state.
 *
 * AUTO mode uses a hysteresis controller so the relay doesn't chatter.
 * That means it turns on when temp or humidity is too high, and only
 * turns off once both have fallen back below the lower threshold.
 */
void applyAutomation() {
  bool wantFan = fanState;

  if (controlMode == MODE_MANUAL) {
    wantFan = fanManual;
  } else {
    bool tempTooHigh = !isnan(tempC)  && (tempC  >= th.temp_on);
    bool humTooHigh  = !isnan(humPct) && (humPct >= th.hum_on);
    bool tempOk      = !isnan(tempC)  && (tempC  <= th.temp_off);
    bool humOk       = !isnan(humPct) && (humPct <= th.hum_off);

    if (tempTooHigh || humTooHigh) wantFan = true;

    if (wantFan) {
      bool allClear = true;
      if (!isnan(tempC))  allClear &= tempOk;
      if (!isnan(humPct)) allClear &= humOk;
      if (allClear) wantFan = false;
    }
  }

  // Only write to the relay when the state actually changes —
  // avoids chattering and keeps the Serial log readable
  if (wantFan != fanState) {
    fanState = wantFan;
    setFan(fanState);
    Serial.printf("[FAN] → %s  (mode=%s  T=%.1f°C  H=%.1f%%)\n",
      fanState ? "ON" : "OFF",
      controlMode == MODE_AUTO ? "auto" : "manual",
      isnan(tempC)  ? 0.0f : tempC,
      isnan(humPct) ? 0.0f : humPct);
  }
}

// ============================================================
//  JSON STATE BUILDER
// ============================================================

/**
 * Serialise the complete node state into a JSON string.
 * Shared by the embedded dashboard JS and any external API consumer.
 */
String buildStateJson() {
  StaticJsonDocument<768> doc;

  doc["node"]     = FW_NODE_ID;
  doc["version"]  = FW_VERSION;
  doc["ip"]       = WiFi.localIP().toString();
  doc["rssi"]     = WiFi.RSSI();
  doc["uptime_s"] = (millis() - bootMs) / 1000UL;

  doc["mode"] = (controlMode == MODE_AUTO) ? "auto" : "manual";
  doc["fan"]  = fanState ? 1 : 0;
  doc["day"]  = isDay    ? 1 : 0;

  JsonObject s = doc.createNestedObject("sensors");
  if (!isnan(tempC))  s["temp_c"]  = tempC;
  if (!isnan(humPct)) s["hum_pct"] = humPct;
  if (!isnan(luxVal)) s["lux"]     = (long)luxVal;

  JsonObject b = s.createNestedObject("bmp180");
  b["ok"] = bmp_ok ? 1 : 0;
  if (bmp_ok) {
    if (!isnan(bmpTempC))   b["temp_c"]      = bmpTempC;
    if (!isnan(pressurePa)) b["pressure_pa"] = (long)pressurePa;
  }

  JsonObject t = doc.createNestedObject("thresholds");
  t["temp_on"]   = th.temp_on;
  t["temp_off"]  = th.temp_off;
  t["hum_on"]    = th.hum_on;
  t["hum_off"]   = th.hum_off;
  t["lux_day"]   = th.lux_day;
  t["lux_night"] = th.lux_night;

  String out;
  serializeJson(doc, out);
  return out;
}

// ============================================================
//  WEB DASHBOARD
// ============================================================

/**
 * Serve the full-page dashboard.
 * The page is self-contained — it polls /api/state every 2 seconds
 * and updates the DOM in JavaScript. The ESP32 only serves compact
 * JSON on each tick, not full HTML, which keeps server load minimal.
 */
void handleRoot() {
  static const char html[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Greenhouse — Node 1</title>
  <style>
    :root{
      --bg:#0b1220;--card:#101a33;--muted:#93a4c7;--txt:#e8eeff;
      --ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--line:#223055;
    }
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:linear-gradient(180deg,#070c16 0%,#0b1220 100%);
         color:var(--txt);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;min-height:100vh}
    header{padding:16px 20px 12px;border-bottom:1px solid var(--line);
           position:sticky;top:0;background:rgba(11,18,32,.92);backdrop-filter:blur(10px);z-index:10}
    .header-top{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
    h1{font-size:18px;font-weight:700;letter-spacing:-.2px}
    .ver{font-size:11px;color:var(--muted);margin-left:8px;font-weight:400}
    .sub{margin-top:8px;color:var(--muted);font-size:12px;display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .pill{padding:4px 10px;border:1px solid var(--line);border-radius:999px;font-size:12px;white-space:nowrap}
    .pill.ok {border-color:rgba(34,197,94,.4);color:#b9f6cc}
    .pill.warn{border-color:rgba(245,158,11,.4);color:#ffe2b6}
    .pill.bad {border-color:rgba(239,68,68,.4);color:#ffc3c3}
    .dot{width:7px;height:7px;border-radius:50%;display:inline-block;margin-right:4px}
    .dot.live{background:#22c55e;box-shadow:0 0 6px #22c55e}
    .dot.dead{background:#ef4444}
    main{padding:16px 20px;max-width:1020px;margin:0 auto}
    .grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}
    .card{background:rgba(16,26,51,.85);border:1px solid var(--line);
          border-radius:16px;padding:16px;box-shadow:0 10px 30px rgba(0,0,0,.3)}
    .span6{grid-column:span 6}.span4{grid-column:span 4}
    .span8{grid-column:span 8}.span12{grid-column:span 12}
    @media(max-width:860px){.span6,.span4,.span8{grid-column:span 12}}
    .row{display:flex;justify-content:space-between;gap:10px;align-items:center}
    .clabel{font-size:13px;color:var(--muted);letter-spacing:.2px;margin-bottom:6px}
    .big{font-size:30px;font-weight:800;line-height:1}
    .unit{font-size:14px;color:var(--muted);margin-left:5px;font-weight:400}
    .hint{font-size:12px;color:var(--muted);margin-top:8px;line-height:1.5}
    .btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}
    button{background:#0e1a38;border:1px solid var(--line);color:var(--txt);
           padding:10px 14px;border-radius:12px;font-weight:600;cursor:pointer;font-size:13px}
    button:hover{border-color:#3a5490}
    button.primary{background:#132756;border-color:#1e3d7a}
    .toggle{width:56px;height:30px;border-radius:999px;border:1px solid var(--line);
            background:#0e1a38;position:relative;cursor:pointer;flex-shrink:0}
    .knob{width:24px;height:24px;border-radius:50%;background:#6b7fa8;
          position:absolute;top:2px;left:2px;transition:.2s}
    .toggle.on{background:rgba(34,197,94,.15);border-color:rgba(34,197,94,.4)}
    .toggle.on .knob{left:28px;background:#86efac}
    .gauge{width:120px;height:120px;border-radius:50%;
           background:conic-gradient(#22c55e 0deg,var(--line) 0deg);
           display:grid;place-items:center;flex-shrink:0;transition:background .4s}
    .gauge .inner{width:90px;height:90px;border-radius:50%;
                  background:radial-gradient(circle at 30% 20%,rgba(255,255,255,.07),rgba(0,0,0,.2)),var(--card);
                  border:1px solid rgba(255,255,255,.06);
                  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px}
    .gauge .gval{font-size:21px;font-weight:800;line-height:1}
    .gauge .glab{font-size:11px;color:var(--muted)}
    .th-grid{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px;margin-top:8px}
    .th-row{font-size:12px;color:var(--muted)}
    .th-row span{color:var(--txt);font-weight:600}
    pre{white-space:pre-wrap;word-break:break-all;color:#cfe0ff;font-size:11.5px;line-height:1.6}
    footer{text-align:center;color:var(--muted);font-size:11px;padding:20px;margin-top:8px}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace}
    #lastCmd{font-size:11px;color:var(--muted);margin-top:8px}
  </style>
</head>
<body>
<header>
  <div class="header-top">
    <h1>Smart Greenhouse <span style="color:var(--muted)">·</span> Node 1 <span class="ver" id="verBadge"></span></h1>
    <span id="connBadge"><span class="dot dead" id="connDot"></span><span id="connTxt">Connecting…</span></span>
  </div>
  <div class="sub">
    <span class="pill" id="pillMode">Mode: —</span>
    <span class="pill" id="pillFan">Fan: —</span>
    <span class="pill" id="pillDay">—</span>
    <span class="pill mono" id="pillNet">IP: — | RSSI: —</span>
    <span class="pill mono" id="pillUptime">Uptime: —</span>
  </div>
</header>
<main>
  <div class="grid">
    <div class="card span6">
      <div class="clabel">Temperature (DHT22)</div>
      <div class="row">
        <div><div><span class="big" id="tVal">—</span><span class="unit">°C</span></div><div class="hint" id="tHint">Waiting…</div></div>
        <div class="gauge" id="tGauge"><div class="inner"><div class="gval" id="tGV">—</div><div class="glab">°C</div></div></div>
      </div>
    </div>
    <div class="card span6">
      <div class="clabel">Humidity (DHT22)</div>
      <div class="row">
        <div><div><span class="big" id="hVal">—</span><span class="unit">%</span></div><div class="hint" id="hHint">Waiting…</div></div>
        <div class="gauge" id="hGauge"><div class="inner"><div class="gval" id="hGV">—</div><div class="glab">%</div></div></div>
      </div>
    </div>
    <div class="card span6">
      <div class="clabel">Ambient Light (BH1750)</div>
      <div class="row">
        <div><div><span class="big" id="lVal">—</span><span class="unit">lux</span></div><div class="hint" id="lHint">Waiting…</div></div>
        <div class="gauge" id="lGauge"><div class="inner"><div class="gval" id="lGV">—</div><div class="glab">lux</div></div></div>
      </div>
    </div>
    <div class="card span6">
      <div class="clabel">Barometric Pressure (BMP180)</div>
      <div class="row">
        <div><div><span class="big" id="pVal">—</span><span class="unit">hPa</span></div><div class="hint" id="pHint">Waiting…</div></div>
        <div class="gauge" id="pGauge"><div class="inner"><div class="gval" id="pGV">—</div><div class="glab">hPa</div></div></div>
      </div>
    </div>
    <div class="card span8">
      <div class="row">
        <div>
          <div class="clabel">Fan Control</div>
          <div class="hint">Auto manages the fan automatically. Manual gives you direct override.</div>
        </div>
        <div class="toggle" id="fanToggle" onclick="toggleFan()"><div class="knob"></div></div>
      </div>
      <div class="btns">
        <button class="primary" onclick="sendCmd({mode:'auto'})">Auto Mode</button>
        <button onclick="sendCmd({mode:'manual'})">Manual Mode</button>
        <button onclick="sendCmd({mode:'manual',fan:1})">Fan ON</button>
        <button onclick="sendCmd({mode:'manual',fan:0})">Fan OFF</button>
      </div>
      <div id="lastCmd" class="mono">—</div>
    </div>
    <div class="card span4">
      <div class="clabel">Active Thresholds</div>
      <div class="th-grid">
        <div class="th-row">temp on <span id="th_ton">—</span>°C</div>
        <div class="th-row">temp off <span id="th_toff">—</span>°C</div>
        <div class="th-row">hum on <span id="th_hon">—</span>%</div>
        <div class="th-row">hum off <span id="th_hoff">—</span>%</div>
        <div class="th-row">lux day <span id="th_lday">—</span></div>
        <div class="th-row">lux night <span id="th_lnight">—</span></div>
      </div>
      <div class="hint" style="margin-top:10px">Adjust via POST /api/cmd {"set":{…}}</div>
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
<footer>Smart Greenhouse Node 1 — ESP32 firmware <span id="footVer"></span></footer>
<script>
  const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
  const fmt=(v,dp=1)=>(v==null||Number.isNaN(+v))?"—":(+v).toFixed(dp);
  function setGauge(el,v,min,max,c="#22c55e"){
    if(v==null||Number.isNaN(+v))return;
    const deg=Math.round(clamp((+v-min)/(max-min),0,1)*270);
    el.style.background=`conic-gradient(${c} 0deg ${deg}deg,#223055 ${deg}deg 360deg)`;
  }
  function fmtUptime(s){
    const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
    return h>0?`${h}h ${m}m ${sec}s`:m>0?`${m}m ${sec}s`:`${sec}s`;
  }
  function setPill(id,txt,cls){const e=document.getElementById(id);e.textContent=txt;e.className='pill '+cls;}
  function setHint(id,v,hi,lo,mHi,mLo,mOk){
    document.getElementById(id).textContent=v==null?'No data':v>=hi?mHi:v<=lo?mLo:mOk;
  }
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
      const mode=j.mode??'—',fan=j.fan,day=j.day;
      setPill('pillMode',`Mode: ${mode}`,mode==='auto'?'ok':'warn');
      setPill('pillFan',`Fan: ${fan===1?'ON':fan===0?'OFF':'—'}`,fan===1?'ok':'warn');
      setPill('pillDay',day===1?'☀ Day':'☾ Night','ok');
      document.getElementById('pillNet').textContent=`IP: ${j.ip??'—'}  |  RSSI: ${j.rssi??'—'} dBm`;
      document.getElementById('pillUptime').textContent=`Uptime: ${j.uptime_s!=null?fmtUptime(j.uptime_s):'—'}`;
      const temp=j?.sensors?.temp_c,hum=j?.sensors?.hum_pct,lux=j?.sensors?.lux;
      const bmpOk=j?.sensors?.bmp180?.ok===1;
      const presPa=bmpOk?j?.sensors?.bmp180?.pressure_pa:null;
      const presHpa=presPa!=null?presPa/100:null;
      document.getElementById('tVal').textContent=fmt(temp,1);document.getElementById('tGV').textContent=fmt(temp,1);
      setGauge(document.getElementById('tGauge'),temp,0,50);
      document.getElementById('hVal').textContent=fmt(hum,0);document.getElementById('hGV').textContent=fmt(hum,0);
      setGauge(document.getElementById('hGauge'),hum,0,100,'#3b82f6');
      document.getElementById('lVal').textContent=lux!=null?Math.round(lux):'—';document.getElementById('lGV').textContent=lux!=null?Math.round(lux):'—';
      setGauge(document.getElementById('lGauge'),lux,0,2000,'#f59e0b');
      document.getElementById('pVal').textContent=fmt(presHpa,0);document.getElementById('pGV').textContent=fmt(presHpa,0);
      setGauge(document.getElementById('pGauge'),presHpa,900,1100,'#a78bfa');
      const th=j.thresholds||{};
      setHint('tHint',temp,th.temp_on,th.temp_off,'Temp high → fan may turn ON','Temp low → fan may turn OFF','Temperature in range');
      setHint('hHint',hum,th.hum_on,th.hum_off,'Humidity high → fan may turn ON','Humidity low → fan may turn OFF','Humidity in range');
      document.getElementById('lHint').textContent=lux==null?'No light data':day===1?'Day cycle active':'Night cycle active';
      document.getElementById('pHint').textContent=bmpOk?`BMP180 OK — backup ${fmt(j?.sensors?.bmp180?.temp_c,1)}°C`:'BMP180 not detected (optional)';
      document.getElementById('th_ton').textContent=th.temp_on??'—';document.getElementById('th_toff').textContent=th.temp_off??'—';
      document.getElementById('th_hon').textContent=th.hum_on??'—';document.getElementById('th_hoff').textContent=th.hum_off??'—';
      document.getElementById('th_lday').textContent=th.lux_day??'—';document.getElementById('th_lnight').textContent=th.lux_night??'—';
      const t=document.getElementById('fanToggle');
      fan===1?t.classList.add('on'):t.classList.remove('on');
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
  function toggleFan(){const on=!document.getElementById('fanToggle').classList.contains('on');sendCmd({mode:'manual',fan:on?1:0});}
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

/**
 * GET /api/state
 */
void handleApiState() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buildStateJson());
}

/**
 * POST /api/cmd
 * Accepts JSON body — see API reference in README.md.
 */
void handleApiCmd() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"request body missing\"}");
    return;
  }

  StaticJsonDocument<384> doc;
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

  if (doc.containsKey("fan")) {
    fanManual = (doc["fan"].as<int>() == 1);
    Serial.printf("[CMD] fan_manual → %d\n", fanManual ? 1 : 0);
  }

  if (doc.containsKey("set")) {
    JsonObject s = doc["set"];
    bool changed = false;
    if (s.containsKey("temp_on"))   { th.temp_on   = s["temp_on"];   changed = true; }
    if (s.containsKey("temp_off"))  { th.temp_off  = s["temp_off"];  changed = true; }
    if (s.containsKey("hum_on"))    { th.hum_on    = s["hum_on"];    changed = true; }
    if (s.containsKey("hum_off"))   { th.hum_off   = s["hum_off"];   changed = true; }
    if (s.containsKey("lux_day"))   { th.lux_day   = s["lux_day"];   changed = true; }
    if (s.containsKey("lux_night")) { th.lux_night = s["lux_night"]; changed = true; }
    if (changed) savePrefs();
  }

  applyAutomation();
  server.send(200, "application/json", buildStateJson());
}

// ============================================================
//  SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n\n[BOOT] Smart Greenhouse Node 1  v%s\n", FW_VERSION);

  // Hardware watchdog — reboots if loop() stalls for WDT_TIMEOUT_S seconds
  esp_task_wdt_init(WDT_TIMEOUT_S, /*panic=*/true);
  esp_task_wdt_add(NULL);

  // Set relay pin before configuring as OUTPUT to prevent an unwanted
  // pulse during the pinMode() transition
  digitalWrite(PIN_FAN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(PIN_FAN_RELAY, OUTPUT);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);  // 100 kHz — safe for longer greenhouse wiring runs

  dht.begin();

  bh_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  if (!bh_ok) Serial.println("[WARN] BH1750 not found on I2C bus.");

  bmp_ok = bmp.begin();
  if (!bmp_ok) Serial.println("[INFO] BMP180 not found (optional — skipping).");

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
    Serial.println("[WARN] WiFi timed out — will retry in background.");
  }

  if (MDNS.begin(MDNS_HOST))
    Serial.printf("[mDNS] Browse to %s.local\n", MDNS_HOST);

  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/api/state",  HTTP_GET,  handleApiState);
  server.on("/api/cmd",    HTTP_POST, handleApiCmd);
  server.begin();
  Serial.println("[HTTP] Server started on port 80.");

  bootMs = millis();
  readSensors();
  applyAutomation();
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();

  static uint32_t lastRead = 0;
  uint32_t now = millis();
  if (now - lastRead >= SENSOR_INTERVAL_MS) {
    lastRead = now;
    readSensors();
    applyAutomation();
  }
}
