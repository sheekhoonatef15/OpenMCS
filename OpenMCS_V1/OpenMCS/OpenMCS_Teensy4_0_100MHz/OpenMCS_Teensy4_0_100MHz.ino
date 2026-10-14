#include <Arduino.h>
// OpenMCS Teensy controller for the original full-feature GUI and the
// CHAMPION_100MHz cumulative FPGA.  GUI appearance/features stay unchanged.
static constexpr uint32_t FPGA_BAUD=5000000;
// 8192 extra bytes absorb about 16.4 ms at 5 Mbaud (8N1).
// This is burst tolerance, not flow control for an indefinitely stalled PC.
static uint8_t fpga_rx_extra[8192];
static uint8_t usb_forward_buffer[256];
static size_t usb_forward_size=0, usb_forward_offset=0;
static inline uint32_t cpuHz() { return (uint32_t)F_CPU_ACTUAL; }
static constexpr int TRIGGER_PIN=8, EVENT_PIN=9, PASS_TOGGLE_PIN=13;
static constexpr int ACCEPT_ENABLE_PIN=16, STOP_REQUEST_PIN=22, STOP_ACK_PIN=23;
char cmd[96]; uint8_t cmdlen=0;
volatile uint32_t max_passes=0;
volatile uint32_t accepted_passes=0;
volatile bool max_pass_stop_latched=false;
uint32_t trigger_hz=100;
bool internal_trigger=false, external_mode=false, selftest=false, internal_signal=false;
uint8_t pass_last=0; uint32_t next_trigger_cycles=0;
uint16_t st_n=1; uint32_t st_pass=0, st_t0=0;
uint32_t st_offset_cycles[141];
volatile uint32_t selftest_late_edges=0, selftest_max_late_cycles=0;
uint16_t configured_bins=0; uint32_t configured_dwell=0; bool configuration_known=false;

// The FPGA-to-PC stream has priority over PC-to-FPGA commands.  Keep draining
// Serial3 while a frame is arriving, but do not consume new USB commands until
// the FPGA has been quiet long enough for the tail of the frame to leave USB.
static constexpr uint32_t FPGA_STREAM_QUIET_US = 2000;
bool fpga_stream_active=false;
uint32_t fpga_last_byte_us=0;

// Set after measuring TRIGGER_PIN -> EVENT_PIN with a calibrated timebase.
// Positive values lengthen the waveform; negative values shorten it.
static constexpr int32_t SELFTEST_CALIBRATION_PPM = 0;
static constexpr uint32_t SELFTEST_GUARD_NS = 500;
static constexpr uint32_t SELFTEST_PULSE_NS = 40;

// Hardware-PWM self-test. Both frequencies divide the Teensy 4.x PWM clock
// exactly, so their 20:1 relationship does not depend on loop(), USB, or UART.
// With 15-bit duty control these values produce approximately 50 ns on H2 and
// 40 ns on H4. The phase is established when enabled and then remains fixed.
static constexpr uint32_t PWM_RESOLUTION_BITS = 15;
static constexpr uint32_t PWM_FULL_SCALE = 1UL << PWM_RESOLUTION_BITS;
static constexpr float SELFTEST_TRIGGER_HZ = 500.0f;
static constexpr float SELFTEST_EVENT_HZ = 10000.0f;
static constexpr uint32_t SELFTEST_TRIGGER_DUTY = 1;
static constexpr uint32_t SELFTEST_EVENT_DUTY = 13;

static uint8_t crc8_next(uint8_t c,uint8_t d){c^=d;for(int i=0;i<8;i++)c=(c&0x80)?uint8_t((c<<1)^0x07):uint8_t(c<<1);return c;}
static void cfg5(uint16_t bins,uint32_t dwell){uint8_t p[11]={'C','F','G','5',uint8_t(bins),uint8_t(bins>>8),uint8_t(dwell),uint8_t(dwell>>8),uint8_t(dwell>>16),uint8_t(dwell>>24),0};uint8_t c=0;for(int i=0;i<10;i++)c=crc8_next(c,p[i]);p[10]=c;Serial3.write(p,11);}
static void sendPassLimit(uint32_t limit){uint8_t p[5]={'L',uint8_t(limit),uint8_t(limit>>8),uint8_t(limit>>16),uint8_t(limit>>24)};Serial3.write(p,5);}
static inline void forwardFpga(){
  // Bound work per call so hardware controls/internal triggers remain serviced.
  // Hold a short USB write for the next call; never consume/reorder those bytes.
  if(usb_forward_offset==usb_forward_size){
    usb_forward_offset=usb_forward_size=0;
    int ready=Serial3.available();
    int capacity=Serial.availableForWrite();
    size_t count=ready>0 ? (size_t)ready : 0;
    if(capacity<=0) count=0;
    else if(count>(size_t)capacity) count=(size_t)capacity;
    if(count>sizeof(usb_forward_buffer)) count=sizeof(usb_forward_buffer);
    for(size_t i=0;i<count;i++){
      int value=Serial3.read();
      if(value<0) break;
      usb_forward_buffer[usb_forward_size++]=(uint8_t)value;
    }
    if(usb_forward_size){
      fpga_stream_active=true;
      fpga_last_byte_us=micros();
    }
  }
  if(usb_forward_offset<usb_forward_size){
    int capacity=Serial.availableForWrite();
    size_t count=usb_forward_size-usb_forward_offset;
    if(capacity>0){
      if(count>(size_t)capacity) count=(size_t)capacity;
      usb_forward_offset+=Serial.write(usb_forward_buffer+usb_forward_offset,count);
      fpga_last_byte_us=micros();
    }
  }
  if(fpga_stream_active && usb_forward_offset==usb_forward_size &&
     !Serial3.available() &&
     (uint32_t)(micros()-fpga_last_byte_us)>=FPGA_STREAM_QUIET_US){
    fpga_stream_active=false;
  }
}
static inline void pulse(int pin,uint32_t ns=40){digitalWriteFast(pin,HIGH);delayNanoseconds(ns);digitalWriteFast(pin,LOW);}

static inline bool cycleReached(uint32_t now,uint32_t target){return (int32_t)(now-target)>=0;}
static inline void waitUntilCycle(uint32_t target){while(!cycleReached(ARM_DWT_CYCCNT,target)){};}
static inline uint32_t cyclesFromNsCeil(uint32_t ns){return (uint32_t)(((uint64_t)ns*cpuHz()+999999999ULL)/1000000000ULL);}
static inline void pulseCycles(int pin,uint32_t cycles){
  digitalWriteFast(pin,HIGH);
  uint32_t started=ARM_DWT_CYCCNT;
  waitUntilCycle(started+cycles);
  digitalWriteFast(pin,LOW);
}
static void prepareSelfTestTiming(){
  for(uint32_t n=1;n<=140;n++){
    uint64_t target_ns=80ULL*n*n;
    uint64_t base=(target_ns*(uint64_t)cpuHz()+999999999ULL)/1000000000ULL;
    int64_t corrected=(int64_t)base+((int64_t)base*SELFTEST_CALIBRATION_PPM)/1000000LL;
    st_offset_cycles[n]=(uint32_t)((corrected>1)?corrected:1);
  }
}


static void stopHardware(){
  internal_trigger=false;
  external_mode=false;
  digitalWriteFast(ACCEPT_ENABLE_PIN,LOW);   // A10: reject all future H2
  digitalWriteFast(STOP_REQUEST_PIN,HIGH);  // C10: hardware STOP
}
static void arm(){
  digitalWriteFast(STOP_REQUEST_PIN,LOW);
  noInterrupts();
  accepted_passes=0;
  max_pass_stop_latched=false;
  pass_last=digitalReadFast(PASS_TOGGLE_PIN);
  interrupts();
  digitalWriteFast(ACCEPT_ENABLE_PIN,HIGH);
}
static void startMcs(bool make_internal){arm();sendPassLimit(max_passes);Serial3.write('R');internal_trigger=make_internal;external_mode=!make_internal;if(make_internal)next_trigger_cycles=ARM_DWT_CYCCNT+cpuHz()/max(1UL,trigger_hz);}
static void startCounter(){arm();Serial3.write('K');}
static void setSelfTestStimulus(bool enabled){
  if(enabled){
    if(!selftest){
      internal_trigger=false;internal_signal=false;
      // Start H4 first and H2 second. They run autonomously after setup; USB
      // and UART activity cannot move any subsequent edge.
      analogWriteFrequency(EVENT_PIN,SELFTEST_EVENT_HZ);
      analogWrite(EVENT_PIN,SELFTEST_EVENT_DUTY);
      analogWriteFrequency(TRIGGER_PIN,SELFTEST_TRIGGER_HZ);
      analogWrite(TRIGGER_PIN,SELFTEST_TRIGGER_DUTY);
      selftest=true;st_n=1;st_pass=0;selftest_late_edges=0;selftest_max_late_cycles=0;
    }
  }else{
    selftest=false;
    analogWrite(TRIGGER_PIN,0);
    analogWrite(EVENT_PIN,0);
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(EVENT_PIN,OUTPUT);
    digitalWriteFast(TRIGGER_PIN,LOW);
    digitalWriteFast(EVENT_PIN,LOW);
  }
}

static void handle(){cmd[cmdlen]=0;unsigned long a,b;
 if(!strcmp(cmd,"i")){Serial.println("@@OPENMCS:identity:OPENMCS_TEENSY4_V4_MAX_ACCURACY");}
 else if(sscanf(cmd,"c %lu %lu",&a,&b)==2){cfg5((uint16_t)a,(uint32_t)b);configured_bins=(uint16_t)a;configured_dwell=(uint32_t)b;configuration_known=true;}
 else if(sscanf(cmd,"m %lu",&a)==1){
   noInterrupts();
   max_passes=(uint32_t)a;
   interrupts();
 }
 else if(sscanf(cmd,"f %lu",&a)==1){trigger_hz=max(1UL,a);}
 else if(!strcmp(cmd,"r")){startMcs(true);}
 else if(!strcmp(cmd,"e")){startMcs(false);}
 else if(!strcmp(cmd,"k")){startCounter();}
 else if(!strcmp(cmd,"p")){stopHardware();}
 else if(!strcmp(cmd,"t")){pulse(TRIGGER_PIN);}
 else if(sscanf(cmd,"h %lu",&a)==1){setSelfTestStimulus(a!=0);}
 else if(!strcmp(cmd,"z")){
   // Explicit GUI Clear: stop trigger generation and request an FPGA memory
   // clear without changing the active bins/dwell configuration.
     stopHardware(); Serial3.write('X');
 }
 else if(sscanf(cmd,"g %lu",&a)==1){internal_signal=(a!=0);digitalWriteFast(EVENT_PIN,internal_signal?HIGH:LOW);}
 else if(!strcmp(cmd,"j")){
   Serial.print("@@OPENMCS:info:Hardware PWM self-test ");
   Serial.print(selftest?"running":"stopped");
   Serial.print(", H2=");Serial.print(SELFTEST_TRIGGER_HZ,3);
   Serial.print(" Hz, H4=");Serial.print(SELFTEST_EVENT_HZ,3);
   Serial.println(" Hz, ratio=20:1");
 }
 else if(cmd[0]=='q' && cmd[1]==' '){Serial3.write((cmd[2]=='d'||cmd[2]=='D')?'D':'U');}
 else if(sscanf(cmd,"y %lu %lu",&a,&b)==2){
   // Configure only when geometry changes. The self-test is a continuous
   // stimulus source; the independently programmed FPGA quota decides how
   // many triggers are accepted by each Start.
   bool stimulus_was_running=selftest;
   bool geometry_changed=!configuration_known || configured_bins!=(uint16_t)b || configured_dwell!=(uint32_t)a;
   if(geometry_changed){
     if(stimulus_was_running)setSelfTestStimulus(false);
     cfg5((uint16_t)b,(uint32_t)a); delay(3);
     configured_bins=(uint16_t)b;configured_dwell=(uint32_t)a;configuration_known=true;
   }
   arm(); sendPassLimit(max_passes); Serial3.write('R');
   // Allow the R toggle and bundled quota to settle in the acquisition domain
   // before issuing the first physical self-test trigger.
   delayMicroseconds(5);
   internal_trigger=false;external_mode=false;
   if(!stimulus_was_running || geometry_changed)setSelfTestStimulus(true);
 }
 cmdlen=0;
}
static void passAcceptedISR(){
  // K5 is a toggle: every CHANGE is exactly one FPGA-accepted MCS pass.
  uint32_t n=accepted_passes+1;
  accepted_passes=n;

  uint32_t limit=max_passes;
  if(limit && n>=limit && !max_pass_stop_latched){
    // Status only. The FPGA quota has already rejected all future passes on
    // this exact acceptance edge. Never stop or gate the physical stimulus.
    max_pass_stop_latched=true;
  }
}

static void servicePassLimit(){
  // The FPGA owns exact pass stopping. Stimulus generation intentionally
  // continues after the quota, matching an experiment with continuously
  // arriving trigger and event signals.
}
static void serviceInternalTrigger(){if(!internal_trigger)return;uint32_t now=ARM_DWT_CYCCNT;if((int32_t)(now-next_trigger_cycles)>=0){pulse(TRIGGER_PIN);uint32_t period=cpuHz()/max(1UL,trigger_hz);do{next_trigger_cycles+=period;}while((int32_t)(now-next_trigger_cycles)>=0);}}
static void serviceSelfTest(){
  // Hardware FlexPWM continues without CPU service.
}
void setup(){ARM_DEMCR|=ARM_DEMCR_TRCENA;ARM_DWT_CTRL|=ARM_DWT_CTRL_CYCCNTENA;prepareSelfTestTiming();pinMode(TRIGGER_PIN,OUTPUT);pinMode(EVENT_PIN,OUTPUT);pinMode(PASS_TOGGLE_PIN,INPUT);
attachInterrupt(digitalPinToInterrupt(PASS_TOGGLE_PIN),passAcceptedISR,CHANGE);
pinMode(ACCEPT_ENABLE_PIN,OUTPUT);pinMode(STOP_REQUEST_PIN,OUTPUT);pinMode(STOP_ACK_PIN,INPUT);digitalWriteFast(TRIGGER_PIN,LOW);digitalWriteFast(EVENT_PIN,LOW);digitalWriteFast(ACCEPT_ENABLE_PIN,LOW);digitalWriteFast(STOP_REQUEST_PIN,LOW);analogWriteResolution(PWM_RESOLUTION_BITS);Serial.begin(115200);Serial3.begin(FPGA_BAUD);Serial3.addMemoryForRead(fpga_rx_extra,sizeof(fpga_rx_extra));}
void loop(){
  serviceSelfTest();
  forwardFpga();
  serviceSelfTest();

  // A live-preview stream can be continuous, so waiting for a 2 ms UART gap
  // before reading STOP/CLEAR can starve those commands forever.  Recognize
  // only the two urgent, single-letter commands while a frame is in flight.
  // All configuration commands remain queued until the binary stream is idle.
  if(fpga_stream_active){
    if(cmdlen==0 && Serial.available()){
      int c=Serial.peek();
      if(c=='p' || c=='z'){
        Serial.read();
        if(c=='p') stopHardware();
        else { stopHardware(); Serial3.write('X'); }

        // Normally the GUI has already delivered the line ending.  Consume it
        // when present; a later-arriving line ending is harmlessly handled as
        // an empty command after the FPGA stream becomes idle.
        while(Serial.available()){
          int eol=Serial.peek();
          if(eol!='\r' && eol!='\n') break;
          Serial.read();
        }
      }
    }
  }else{
    while(Serial.available()){
      char c=Serial.read();
      if(c=='\r')continue;
      if(c=='\n')handle();
      else if(cmdlen<sizeof(cmd)-1)cmd[cmdlen++]=c;
      else cmdlen=0;
    }
  }

  serviceSelfTest();
  servicePassLimit();
  serviceInternalTrigger();
  serviceSelfTest();
  forwardFpga();
}
