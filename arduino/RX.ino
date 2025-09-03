#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

const uint8_t  DAC_ADDR=0x60;
const float    V_MIN=0.90f, V_MAX=4.10f, V_RANGE=(V_MAX-V_MIN);
const float    V_RAMP_RATE=0.50f;   // V/s
const uint32_t DEADMAN_MS=300;
const uint8_t  ESTOP_PIN=7, LED_PIN=13;

float target_v=V_MIN, actual_v=V_MIN; uint32_t last_rx_ms=0;
float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
float throttleToVolt(float t){t=clampf(t,0,1); return V_MIN+t*V_RANGE;}
uint16_t voltToCode(float v){v=clampf(v,0,5); return (uint16_t)round((v/5.0)*4095.0);}
void setV(float v){actual_v=v; dac.setVoltage(voltToCode(v),false);}
void estop(){setV(V_MIN); digitalWrite(LED_PIN,HIGH);}

void setup(){
  pinMode(ESTOP_PIN,INPUT_PULLUP); pinMode(LED_PIN,OUTPUT);
  Wire.begin(); dac.begin(DAC_ADDR);
  Serial.begin(115200); setV(V_MIN); last_rx_ms=millis();
}

void loop(){
  // 직렬 수신: "SPD:0~1000" 또는 "STOP"
  static char buf[32]; static uint8_t idx=0;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'){ buf[idx]=0; idx=0;
      if(strncmp(buf,"SPD:",4)==0){
        int val=constrain(atoi(buf+4),0,1000);
        target_v=throttleToVolt(val/1000.0f);
        last_rx_ms=millis();
      }else if(strcmp(buf,"STOP")==0){ target_v=V_MIN; last_rx_ms=millis(); }
    }else if(idx<sizeof(buf)-1){ buf[idx++]=c; }
  }

  bool eStop=(digitalRead(ESTOP_PIN)==LOW);
  bool timeout=(millis()-last_rx_ms>DEADMAN_MS);
  if(eStop||timeout){ estop(); }
  else{
    digitalWrite(LED_PIN,LOW);
    static uint32_t last=millis(); uint32_t now=millis();
    float dt=(now-last)/1000.0f; last=now;
    float dv=target_v-actual_v, dvmax=V_RAMP_RATE*dt;
    if(fabs(dv)>dvmax) dv=(dv>0?dvmax:-dvmax);
    setV(actual_v+dv);
  }
  delay(2);
}
