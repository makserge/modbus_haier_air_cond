#include <ModbusSlave.h>
#include <IWatchdog.h>

const uint8_t SLAVE_ID = 30;
const uint16_t RS485_BAUDRATE = 9600;

#define RX_PIN PA2
#define TX_PIN PA3

#define RS485_TX_ENABLE_PIN PA_0
#define RS485_RX_PIN PA10
#define RS485_TX_PIN PA9

#define CUR_TMP_BYTE 13 //Cur temp
#define CMD_BYTE 17
#define MODE_BYTE 23 //00 - smart, 01 - cool, 02 - heat,  03 - vent, 04 - dry
#define FAN_SPD_BYTE 25  //00 - max, 01 - mid, 02 - min, 03 - auto
#define SWING_BYTE 27 //00 - off, 01 - up and down on, 02 - left and right on, 03 - both on
#define LOCK_REM_BYTE 28 //00 - off, 80 - on
#define POWER_BYTE 29 //00 - off, 01 - on, 09 - quiet
#define FRESH_BYTE 31 //00 - off, 01 - on
#define SET_TMP_BYTE 35 //Set temp

const uint16_t WATCHDOG_TIMEOUT = 10000000; //10s
const uint16_t PERIODICAL_TIMER_FREQUENCY = 200000; //5s

uint8_t condStatusCommand[13] = { 255, 255, 10, 0, 0, 0, 0, 0, 1, 1, 77, 1, 90 };
uint8_t condOnCommand[] = { 255, 255, 10, 0, 0, 0, 0, 0, 1, 1, 77, 2, 91 };
uint8_t condOffCommand[]  = { 255, 255, 10, 0, 0, 0, 0, 0, 1, 1, 77, 3, 92 };

uint8_t condData[37] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t inCheck = 0;
uint16_t fresh, power, lockRem;
uint8_t lastPowerValue = 0;

const uint8_t FRESH = 0;
const uint8_t LOCK_REMOTE = 1;
const uint8_t POWER = 2;
const uint8_t SWING = 3;
const uint8_t SPEED = 4;
const uint8_t MODE = 5;
const uint8_t SET_TEMP = 6;
const uint8_t TEMP = 7;

const uint8_t MAX_TEMP = 30;

uint16_t inputRegister[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

HardwareSerial sp(RX_PIN, TX_PIN);
Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);

// Handle the function code Read Holding Registers (FC=03) and write back the values from the air cond.
uint8_t readState(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    slave.writeRegisterToBuffer(i, inputRegister[address + i]);
  }
  return STATUS_OK;
}

void setRegister(uint8_t data[], size_t size) {
  fresh = data[FRESH_BYTE];
  lockRem = data[LOCK_REM_BYTE];
  power = data[POWER_BYTE];

  if (data[SWING_BYTE] <= 3) { 
    inputRegister[SWING] =  data[SWING_BYTE];
  }

  if (data[FAN_SPD_BYTE] <= 3) {
    inputRegister[SPEED] = data[FAN_SPD_BYTE];
  }

  if (data[MODE_BYTE] <= 4) {  
    inputRegister[MODE] = data[MODE_BYTE];
  }
  
  inputRegister[SET_TEMP] = data[SET_TMP_BYTE] + 16;
  inputRegister[TEMP] = data[CUR_TMP_BYTE];
  
  if (fresh == 0x00) {
    inputRegister[FRESH] = 0;
  } else if (fresh == 0x01) {
    inputRegister[FRESH] = 1;
  }
  
  if (lockRem == 0x00){
    inputRegister[LOCK_REMOTE] = 0;
  } else if (lockRem == 0x80) {
    inputRegister[LOCK_REMOTE] = 1;
  }
  
  switch (power) {
    case 0x00:
    case 0x10:
      inputRegister[POWER] = 0;
      break;
    case 0x01:
    case 0x11:
      inputRegister[POWER] = 1;
      break;
    case 0x09:
      inputRegister[POWER] = 2;
      break;
  }
}

uint8_t getCRC(uint8_t data[], size_t size) {
  uint8_t crc = 0;
  for (uint16_t i = 2; i < size; i++) {
      crc += data[i];
  }
  return crc;
}

void sendData(uint8_t data[], size_t size) {
  sp.write(data, size - 1);
  sp.write(getCRC(data, size - 1));
}

// Handle the function codes Write Holding Register(s) (FC=06, FC=16) and write data to the air cond.
uint8_t writeState(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t powerValue; 
  for (uint16_t i = 0; i < length; i++) {
    uint8_t value = slave.readRegisterFromBuffer(i + address);
    if (i == SET_TEMP) {
      value -= 16;
      if (MAX_TEMP <= 30) {
        condData[SET_TMP_BYTE] = value;      
      }
    } else if ((i == MODE) && (value <= 4)) {
      condData[MODE_BYTE] = value;
    } else if ((i == SPEED) && (value <= 3)) {
      condData[FAN_SPD_BYTE] = value;
    } else if ((i == SWING) && (value <= 3)) {
      condData[SWING_BYTE] = value;
    } else if ((i == LOCK_REMOTE) && (value <= 1)) {
      condData[LOCK_REM_BYTE] = (value == 1) ? 80 : 0;
    } else if ((i == POWER) && (value <= 1)) {
      if (value == 9) {
        condData[POWER_BYTE] = value;
      }
      powerValue = value;
    }
  }
  
  condData[CMD_BYTE] = 0;
  condData[9] = 1;
  condData[10] = 77;
  condData[11] = 95;
  sendData(condData, sizeof(condData)/sizeof(byte));

  if (powerValue != lastPowerValue) {
    lastPowerValue = powerValue;
    if (powerValue == 1) {
      sendData(condOnCommand, sizeof(condOnCommand)/sizeof(byte));
    } else if (powerValue == 0) {
      sendData(condOffCommand, sizeof(condOffCommand)/sizeof(byte));
    }
  }
  
  return STATUS_OK;
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, MICROSEC_FORMAT);
  timer->attachInterrupt(updateStatus);
  timer->resume();
}

void updateStatus() {
  sendData(condStatusCommand, sizeof(condStatusCommand) / sizeof(byte));
}

void setup() {
  IWatchdog.begin(WATCHDOG_TIMEOUT);
    
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readState;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeState;
  
  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);

  initPeriodicalTimer();
}

void loop() {
  if (sp.available() > 0){
    sp.readBytes(condData, 37);
    while(sp.available()){
      delay(2);
      sp.read();
    }
    if (condData[36] != inCheck){
      inCheck = condData[36];
      setRegister(condData, 37);
    }
  }
  
  slave.poll(); 
  IWatchdog.reload();
}
