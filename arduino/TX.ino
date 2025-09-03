#include <TinyGPSPlus.h>
#include <NimBLEDevice.h>
#include <math.h>

// ---------- 설정 ----------
static const char* DEVICE_NAME   = "AMB-TX";
static const uint16_t ADV_INTERVAL_MS = 250;    // 200~300 권장

// 배선: GPS TX -> ESP32 GPIO16(RX2), GPS RX -> ESP32 GPIO17(TX2) (RX만 연결)
static const int GPS_RX_PIN = 16;
static const int GPS_TX_PIN = 17;
static const uint32_t GPS_BAUD = 9600;

static const bool EMERGENCY_ON = true;

// 실내 완화
#define INDOOR_TEST 1
#if INDOOR_TEST
  static const int   REQ_SATS = 3;
  static const float MAX_HDOP = 8.0;
#else
  static const int   REQ_SATS = 4;
  static const float MAX_HDOP = 6.0;
#endif

// fix 끊겨도 잠깐 유지
static const uint32_t HOLD_LAST_MS = 5000;

// ---------- 전역 ----------
TinyGPSPlus gps;
HardwareSerial GPSser(2);
NimBLEAdvertising* adv = nullptr;
uint16_t g_seq = 0;

double lastLat=0,lastLon=0,lastSpd=0,lastHdg=0;
uint32_t lastFixMs=0;
bool haveLast=false;

// LE helpers
static inline void le32(uint8_t* p, int32_t v){ p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static inline void le16(uint8_t* p, int16_t v){ p[0]=v; p[1]=v>>8; }

bool readGPS(double &lat,double &lon,double &spd,double &hdg,bool &fix) {
  // 1) NMEA 바이트를 1초 동안 읽어서 바로 에코(디버그)
  unsigned long st = millis();
  bool sawAny = false;
  while (millis() - st < 1000) {
    while (GPSser.available()) {
      char c = GPSser.read();
      gps.encode(c);
      Serial.write(c);      // ← 모니터에 NMEA 그대로 보임
      sawAny = true;
    }
    delay(1);
  }

  const bool locValid  = gps.location.isValid();
  const bool satValid  = gps.satellites.isValid();
  const bool hdopValid = gps.hdop.isValid();
  const uint32_t sats  = satValid  ? gps.satellites.value() : 0;
  const double  hdop   = hdopValid ? gps.hdop.hdop()        : 99.0;

  fix = locValid && (sats >= (uint32_t)REQ_SATS) && (hdop > 0) && (hdop <= MAX_HDOP);

  if (locValid) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    spd = gps.speed.kmph();
    hdg = gps.course.deg();
  }

  // 상태 로그
  Serial.printf("\nGPS: saw=%d loc=%d sats=%u hdop=%.1f fix=%d lat=%.6f lon=%.6f spd=%.1f hdg=%.1f\n",
                sawAny, locValid, sats, hdop, fix, lat, lon, spd, hdg);
  return sawAny;  // 바이트를 한 번이라도 받았는지
}

void setup() {
  Serial.begin(115200);
  GPSser.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  adv = NimBLEDevice::getAdvertising();

  // scan response 비활성화(빈 데이터)
  NimBLEAdvertisementData emptyScanResp;
  adv->setScanResponseData(emptyScanResp);

  adv->setMinInterval(400);   // 250ms
  adv->setMaxInterval(400);

  Serial.println("\nTX ready (debug echo ON)");
}

void loop() {
  double lat=0, lon=0, spd=0, hdg=0;
  bool fix=false;
  bool gotBytes = readGPS(lat,lon,spd,hdg,fix);

  // 20B 제조사 데이터
  uint8_t p[20]={0};
  p[0]='E'; p[1]='V';
  le16(p+2, g_seq++);

  bool usedLast = false;
  if (fix) {
    le32(p+4,  (int32_t)llround(lat*1e7));
    le32(p+8,  (int32_t)llround(lon*1e7));
    le16(p+12, (int16_t)llround(spd*10));
    le16(p+14, (int16_t)llround(hdg*10));
    p[16] = 0x01;
    lastLat=lat; lastLon=lon; lastSpd=spd; lastHdg=hdg; haveLast=true; lastFixMs=millis();
  } else if (haveLast && (millis()-lastFixMs <= HOLD_LAST_MS)) {
    le32(p+4,  (int32_t)llround(lastLat*1e7));
    le32(p+8,  (int32_t)llround(lastLon*1e7));
    le16(p+12, (int16_t)llround(lastSpd*10));
    le16(p+14, (int16_t)llround(lastHdg*10));
    p[16] = 0x00; usedLast = true;
  } else {
    p[16] = 0x00;
  }
  if (EMERGENCY_ON) p[16] |= 0x02;

  NimBLEAdvertisementData data;
  data.setFlags(0x04);
  data.setManufacturerData(std::string((char*)p, sizeof(p)));
  adv->setAdvertisementData(data);

  adv->start();
  delay(ADV_INTERVAL_MS);
  adv->stop();

  Serial.printf("TX seq=%u fix=%c %s lat=%.6f lon=%.6f spd=%.1f hdg=%.1f flags=0x%02X\n",
    g_seq-1, (p[16]&0x01)?'Y':'N', usedLast?"(hold)":"",
    (usedLast?lastLat: (fix?lat:0)),
    (usedLast?lastLon: (fix?lon:0)),
    (usedLast?lastSpd: (fix?spd:0)),
    (usedLast?lastHdg: (fix?hdg:0)),
    p[16]);
}
