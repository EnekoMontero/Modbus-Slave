#include <SPI.h>
#include <Modbus.h>
#include <RH_RF95.h>
#include <ModbusLoRa_RFM95.h>

// LoRa
#define RFM95_RST P1_5
#define CSPin P1_3
#define interruptPin P1_4

#define CLIENT_ADDRESS 10

#define RF95_FREQ 869.0

#define TEMPERATURE_LSB  0x55
#define TEMPERATURE_MSB  0x56

// Singleton instance of the radio driver
RH_RF95 driver(P1_3, P1_4);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// ModbusLoRa Object
ModbusLoRa modbus(manager);

const int ledPin = RED_LED;

union u_float
{
  float     flt;
  word      words[2];
};

u_float sensor_value;

unsigned long inc_time = 0;

uint8_t buf[BUFFER_SIZE];

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  Serial.begin(9600);
  Serial.println("Hello! - ModBus Serial Slave");

  // Config Modbus Serial (port, speed, byte format)
#ifdef DEBUG_MODE
  modbus.configLoRa(&Serial, RFM95_RST, buf, sizeof(buf));
#else
  modbus.configLoRa(RFM95_RST, buf, sizeof(buf));
#endif

  // Set the Slave ID (1-247)
  modbus.setSlaveId(CLIENT_ADDRESS);

  modbus.addIreg(TEMPERATURE_LSB);
  modbus.addIreg(TEMPERATURE_MSB);

  sensor_value.flt = 20.132;

  word msb = sensor_value.words[0];
  word lsb = sensor_value.words[1];

  modbus.Ireg(TEMPERATURE_LSB, lsb);
  modbus.Ireg(TEMPERATURE_MSB, msb);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  // For higher power levels the launchpad may require an external regulator to power the radio.
  // The MSP launchpad 3.3v regulator cannot supply enought current for the radio and can cause a system reset
  driver.setTxPower(23, false);

  // Configure the radio for long range
  // < Bw = 125 kHz, Cr = 4/5, Sf = 4096chips/symbol, CRC on. Slower + longer range
  //  RH_RF95::ModemConfig RFM95config = { 0x72,   0xc4,    0x00}; // Bw125Cr45Sf4096
  // < Bw = 125 kHz, Cr = 4/5, Sf = 512chips/symbol, CRC on. Slow + long range
  RH_RF95::ModemConfig RFM95config = { 0x72,   0x94,    0x00};  // Bw125Cr45Sf512
  driver.setModemRegisters(&RFM95config);

  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
  //  driver.setCADTimeout(10000);

  // Set the frequency of the RFM95
  if (!driver.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  digitalWrite(ledPin, LOW);
}

void loop() {
  //Call once inside loop() - all magic here
  modbus.task();

  if (millis() > inc_time + 500) {
    inc_time = millis();

    sensor_value.flt += 0.3;

    if ( sensor_value.flt >= 30)
      sensor_value.flt = 20.132;

    // Serial.print("Temp: ");
    // Serial.println(sensor_value.flt);

    word msb = sensor_value.words[0];
    word lsb = sensor_value.words[1];

    modbus.Ireg(TEMPERATURE_LSB, lsb);
    modbus.Ireg(TEMPERATURE_MSB, msb);
  }
}






