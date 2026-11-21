/*
 * WallSight 360 - Fixed-position room surface mapping, v0.2.
 * ESP32-S3 DevKitC-1 N8R8 + LD19 + LD2450 + SPI ILI9341 320x240.
 * Prototype only; test power, cabling, and sensors before enclosure.
 *
 * LD19:   TX -> GPIO15  (UART2 / 230400), PWM unconnected for genuine LD19
 * LD2450: TX -> GPIO4, RX <- GPIO5 (UART1 / 256000)
 * TFT:    SCK=12, MOSI=11, CS=10, DC=9, RST=8
 * Button: GPIO47 -> GND (internal pullup)
 * Both sensor modules: 5V and common GND. TFT: per module specifications.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Preferences.h>
#include <math.h>
#include "radar_protocol.h"
#include "wall_map.h"

// ------ Pins and physical alignment ------
constexpr int TFT_SCK=12, TFT_MOSI=11, TFT_CS=10, TFT_DC=9, TFT_RST=8;
constexpr int LD19_RX=15, LD2450_RX=4, LD2450_TX=5, BUTTON_PIN=47;
// The 0 degree LiDAR bearing must point in the same direction as the radar.
// Adjust after assembling, in DEGREES, clockwise positive.
constexpr float LIDAR_ROTATION_DEG=0.0f;
// Radar rotation CLOCKWISE relative to the LiDAR 'front', in degrees.
constexpr float RADAR_ROTATION_DEG=0.0f;
// Set -1 if radar X axis is inverted relative to physical right side.
constexpr float RADAR_X_SIGN=1.0f;
// Physical LD2450 center relative to the LD19 center, in millimeters.
constexpr float RADAR_X_OFFSET_MM=0.0f;
constexpr float RADAR_Y_OFFSET_MM=0.0f;
// Set this to suit the room; display diameter is 2*DISPLAY_RANGE_MM.
constexpr float DISPLAY_RANGE_MM=6000.f;

constexpr int SCREEN_W=320, SCREEN_H=240;
constexpr int CX=119, CY=124, MAP_R=94;
constexpr uint32_t DRAW_INTERVAL_MS=240;
constexpr uint32_t RADAR_STALE_MS=600;
constexpr uint32_t LIDAR_STALE_MS=1200;
constexpr float TARGET_MASK_RADIUS_MM=500.f;

HardwareSerial lidarSerial(2), radarSerial(1);
Adafruit_ILI9341 tft(TFT_CS,TFT_DC,TFT_RST);
// Offscreen buffer reduces flicker. If allocation fails, draw to TFT directly.
GFXcanvas16 canvas(SCREEN_W,SCREEN_H);
roommap::PolarMap mapped;
roommap::Segment lineSegments[32];
int lineCount=0;
uint8_t lidarFrame[47], radarFrame[30];
uint8_t lidarPos=0, radarPos=0;
uint32_t lidarFrames=0, radarFrames=0, lidarBad=0, radarBad=0;
uint32_t lastLidar=0,lastRadar=0,lastPaint=0,lastLog=0;

enum class Mode {SCAN, LIVE};
Mode mode=Mode::SCAN;

struct Target {
  radarproto::Target raw{};
  uint32_t lastSeen=0;
};
Target tracked[3];

// Convert radar-local (X-right, Y-forward) to map-local (X-right, Y-forward).
void targetXY(const Target &t, float &x, float &y) {
  const float a=RADAR_ROTATION_DEG*roommap::RAD;
  const float rx=t.raw.xMm*RADAR_X_SIGN;
  const float ry=t.raw.yMm;
  const float c=cosf(a), s=sinf(a);
  x=rx*c+ry*s+RADAR_X_OFFSET_MM;
  y=-rx*s+ry*c+RADAR_Y_OFFSET_MM;
}
bool targetActive(int i, uint32_t now) {
  return tracked[i].raw.valid && tracked[i].lastSeen &&
         (uint32_t)(now-tracked[i].lastSeen)<RADAR_STALE_MS;
}
bool isNearPerson(float x, float y, uint32_t now) {
  for (int i=0;i<3;++i) {
    if (!targetActive(i,now)) continue;
    float px,py; targetXY(tracked[i],px,py);
    if (hypotf(x-px,y-py)<TARGET_MASK_RADIUS_MM) return true;
  }
  return false;
}

void decodeRadar(const uint8_t *p) {
  radarproto::Target ts[3];
  if (!radarproto::decodeLD2450(p,30,ts)) { ++radarBad;return; }
  ++radarFrames;
  uint32_t now=millis();lastRadar=now;
  for (int i=0;i<3;++i) {
    tracked[i].raw=ts[i];
    if (ts[i].valid) tracked[i].lastSeen=now;
    else tracked[i].lastSeen=0;
  }
}
void pollRadar() {
  const uint8_t header[]={0xAA,0xFF,0x03,0x00};
  while (radarSerial.available()) {
    uint8_t b=(uint8_t)radarSerial.read();
    if (radarPos<4 && b!=header[radarPos]) {
      radarPos=(b==0xAA)?1:0;
      if (radarPos==1) radarFrame[0]=b;
      continue;
    }
    radarFrame[radarPos++]=b;
    if (radarPos==sizeof(radarFrame)) {
      decodeRadar(radarFrame);
      radarPos=0;
    }
  }
}
void decodeLidar(const uint8_t *p) {
  if (!radarproto::validLD19(p,47)) {++lidarBad;return;}
  ++lidarFrames;
  uint32_t now=millis();lastLidar=now;
  if (mode!=Mode::SCAN) return;
  const float start=radarproto::le16(p+4)/100.f;
  float end=radarproto::le16(p+42)/100.f;
  if (end<start) end+=360.f;
  for (int i=0;i<12;++i) {
    uint16_t range=radarproto::le16(p+6+3*i);
    uint8_t intensity=p[8+3*i];
    if (range<roommap::MIN_MM || range>roommap::MAX_MM || intensity==0) continue;
    float deg=start+(end-start)*(float)i/11.f+LIDAR_ROTATION_DEG;
    const int bin=roommap::wrapAngle((int)lroundf(deg));
    float x,y;roommap::polarToCartesian(bin,range,x,y);
    // Exclude near-radar moving people (within its ~120-degree frontal FOV).
    if (isNearPerson(x,y,now)) continue;
    mapped.observe(bin,range);
  }
}
void pollLidar() {
  while (lidarSerial.available()) {
    uint8_t b=(uint8_t)lidarSerial.read();
    if (lidarPos==0 && b!=0x54) continue;
    if (lidarPos==1 && b!=0x2C) {
      lidarPos=(b==0x54)?1:0;
      if (lidarPos==1) lidarFrame[0]=b;
      continue;
    }
    lidarFrame[lidarPos++]=b;
    if (lidarPos==sizeof(lidarFrame)) {
      decodeLidar(lidarFrame);
      lidarPos=0;
    }
  }
}

bool loadMap() {
  Preferences pref;
  if (!pref.begin("radar360",true)) return false;
  if (pref.getBytesLength("map")!=sizeof(roommap::StoredMap)) {
    pref.end();return false;
  }
  roommap::StoredMap snap;
  size_t n=pref.getBytes("map",&snap,sizeof(snap));pref.end();
  return n==sizeof(snap) && roommap::restoreStored(snap,mapped);
}
bool saveMap() {
  if (mapped.countStable()<40) return false;
  roommap::StoredMap snap;
  roommap::buildStored(mapped,snap);
  Preferences pref;
  if (!pref.begin("radar360",false)) return false;
  size_t n=pref.putBytes("map",&snap,sizeof(snap));pref.end();
  return n==sizeof(snap);
}
void clearMap() {
  mapped.clear();
  Preferences pref;
  if (pref.begin("radar360",false)) {pref.remove("map");pref.end();}
  mode=Mode::SCAN;
  Serial.println("[MAP] cleared; SCAN mode");
}
void changeMode() {
  if (mode==Mode::SCAN) {
    if (saveMap()) {mode=Mode::LIVE;Serial.println("[MAP] saved; LIVE mode");}
    else Serial.println("[MAP] at least 40 stable bins needed; still scanning");
  } else {
    mode=Mode::SCAN;
    Serial.println("[MAP] resumed SCAN; old map retained until new samples converge");
  }
}
void exportMapSerial() {
  Serial.println("MAP_JSON_BEGIN");
  Serial.print("{\"format\":\"radar360-v2\",\"bin_degrees\":1,\"distance_mm\":[");
  for (int i=0;i<360;++i) {
    if (i) Serial.print(',');
    Serial.print(mapped.mm[i]);
  }
  Serial.println("]}");
  Serial.println("MAP_JSON_END");
}
void pollSerialCommands() {
  while (Serial.available()) {
    char c=(char)Serial.read();
    if (c=='s' || c=='S') changeMode();
    else if (c=='n' || c=='N') clearMap();
    else if (c=='e' || c=='E') exportMapSerial();
  }
}
void pollButton() {
  static bool previousRead=HIGH, stable=HIGH;
  static uint32_t lastSwitchMs=0, pressedAt=0;
  const uint32_t now=millis();
  const bool read=digitalRead(BUTTON_PIN);
  if (read!=previousRead) {previousRead=read;lastSwitchMs=now;}
  if (read!=stable && (uint32_t)(now-lastSwitchMs)>=35) {
    stable=read;
    if (stable==LOW) pressedAt=now;
    else if ((uint32_t)(now-pressedAt)>=2500) clearMap();
    else changeMode();
  }
}

int pixelX(float x) {return CX+(int)lroundf(x*MAP_R/DISPLAY_RANGE_MM);}
int pixelY(float y) {return CY-(int)lroundf(y*MAP_R/DISPLAY_RANGE_MM);}
bool onMap(int x,int y) {
  int dx=x-CX,dy=y-CY;
  return dx*dx+dy*dy<=MAP_R*MAP_R;
}
void renderScreen() {
  Adafruit_GFX &g=(canvas.getBuffer() ? (Adafruit_GFX&)canvas : (Adafruit_GFX&)tft);
  const uint16_t back=0x0821;
  const uint16_t grid=tft.color565(36,64,73);
  const uint16_t weak=tft.color565(82,118,133);
  const uint16_t wall=tft.color565(112,233,227);
  const uint16_t people=tft.color565(255,160,91);
  const uint16_t white=tft.color565(228,247,245);
  const uint16_t green=tft.color565(116,227,142);
  const uint16_t red=tft.color565(251,96,92);
  g.fillScreen(back);
  g.setTextSize(2);g.setTextColor(white);
  g.setCursor(9,4);g.print("WALLSIGHT 360");
  g.drawFastHLine(0,22,SCREEN_W,grid);
  // Fixed 2D coordinate system; circle limited by DISPLAY_RANGE_MM.
  for (int m=1;m<=3;++m)g.drawCircle(CX,CY,MAP_R*m/3,grid);
  g.drawLine(CX-MAP_R,CY,CX+MAP_R,CY,grid);
  g.drawLine(CX,CY-MAP_R,CX,CY+MAP_R,grid);
  // Paint measured stable surfaces as thin background dots.
  for (int angle=0;angle<360;++angle) {
    if (mapped.mm[angle]==0 || mapped.confidence[angle]<roommap::MIN_LINE_CONF) continue;
    float x,y;roommap::polarToCartesian(angle,mapped.mm[angle],x,y);
    int sx=pixelX(x),sy=pixelY(y);
    if (onMap(sx,sy))g.drawPixel(sx,sy,weak);
  }
  // Straight surface candidates: prominent line segments on the map.
  lineCount=mapped.getSegments(lineSegments,32);
  for (int i=0;i<lineCount;++i) {
    const auto &w=lineSegments[i];
    const int x1=pixelX(w.x1),y1=pixelY(w.y1),x2=pixelX(w.x2),y2=pixelY(w.y2);
    if (onMap(x1,y1) && onMap(x2,y2))g.drawLine(x1,y1,x2,y2,wall);
  }
  // Overlay people, with the LD2450 coordinate frame aligned to LiDAR.
  uint32_t now=millis();
  int activeTargets=0;
  for (int i=0;i<3;++i) {
    if (!targetActive(i,now)) continue;
    float x,y;targetXY(tracked[i],x,y);
    const int sx=pixelX(x),sy=pixelY(y);
    if (!onMap(sx,sy)) continue;
    ++activeTargets;
    g.drawCircle(sx,sy,6,people);
    g.fillCircle(sx,sy,3,people);
    g.setTextColor(people);g.setTextSize(1);
    g.setCursor(sx+8,sy-6);g.print(i+1);
  }
  g.fillCircle(CX,CY,3,white);
  // Compact sidebar: mode, sensors and diagnostics.
  g.drawFastVLine(222,26,194,grid);
  g.setTextSize(2);g.setTextColor(mode==Mode::SCAN?green:wall);
  g.setCursor(232,39);g.print(mode==Mode::SCAN?"SCAN":"LIVE");
  g.setTextSize(1);g.setTextColor(white);
  g.setCursor(230,72);g.print("MAP ");g.print(mapped.countStable());g.print("/360");
  g.setCursor(230,90);g.print("LINES ");g.print(lineCount);
  g.setCursor(230,108);g.print("PEOPLE ");g.print(activeTargets);
  bool lidarOk=lastLidar && now-lastLidar<LIDAR_STALE_MS;
  bool radarOk=lastRadar && now-lastRadar<RADAR_STALE_MS*2;
  g.setCursor(230,138);g.setTextColor(lidarOk?green:red);
  g.print("LD19 ");g.print(lidarOk?"OK":"--");
  g.setCursor(230,156);g.setTextColor(radarOk?green:red);
  g.print("RADR ");g.print(radarOk?"OK":"--");
  g.setTextColor(weak);g.setCursor(229,187);
  g.print("R=");g.print((int)(DISPLAY_RANGE_MM/1000.f));g.print("m");
  g.drawFastHLine(0,224,SCREEN_W,grid);
  g.setTextColor(white);g.setCursor(7,230);
  g.print(mode==Mode::SCAN?"CLICK: SAVE MAP -> LIVE":"CLICK: SCAN AGAIN");
  if (canvas.getBuffer()) tft.drawRGBBitmap(0,0,canvas.getBuffer(),SCREEN_W,SCREEN_H);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  // UARTs are independent, with large receive buffers during display redraws.
  radarSerial.setRxBufferSize(1024);
  lidarSerial.setRxBufferSize(8192);
  radarSerial.begin(256000,SERIAL_8N1,LD2450_RX,LD2450_TX);
  lidarSerial.begin(230400,SERIAL_8N1,LD19_RX,-1);
  SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS);
  tft.begin(40000000);
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  mode=loadMap()?Mode::LIVE:Mode::SCAN;
  Serial.printf("WallSight 360 v0.2, mode=%s, stable=%d, framebuf=%s\n",
                mode==Mode::LIVE?"LIVE":"SCAN",mapped.countStable(),
                canvas.getBuffer()?"ready":"direct");
  Serial.println("Commands: s=save/switch mode, n=clear/new scan, e=export JSON");
}
void loop() {
  pollRadar();
  pollLidar();
  pollButton();
  pollSerialCommands();
  uint32_t now=millis();
  if ((uint32_t)(now-lastPaint)>DRAW_INTERVAL_MS) {
    lastPaint=now;
    renderScreen();
  }
  if ((uint32_t)(now-lastLog)>3000) {
    lastLog=now;
    Serial.printf("mode=%s bins=%d lines=%d LD19=%lu (bad=%lu) LD2450=%lu (bad=%lu)\n",
                  mode==Mode::SCAN?"SCAN":"LIVE",mapped.countStable(),lineCount,
                  (unsigned long)lidarFrames,(unsigned long)lidarBad,
                  (unsigned long)radarFrames,(unsigned long)radarBad);
  }
  delay(1);
}
