

#include <EEPROM.h>

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/

#define DEVICE_ID_ADDR 0x00 // Write to this address in EEPROM
uint8_t client_id = 0;//Write Client ID: 255


#define DRY_VAL_ADDR_1 0x01
#define DRY_VAL_ADDR_2 0x02
uint16_t dryValue = 0;

#define HALT_RUN_ADDR 0x03
boolean haltIrrigation = false;

#define RUN_INTERVAL_ADDR_1 0x04
#define RUN_INTERVAL_ADDR_2 0x05
#define RUN_INTERVAL_ADDR_3 0x06
#define RUN_INTERVAL_ADDR_4 0x07
uint32_t runIrrigationInterval = 0;//14400000 = 4hrs

#define VALVE_OPEN_ADDR_1 0x12
#define VALVE_OPEN_ADDR_2 0x13
#define VALVE_OPEN_ADDR_3 0x14
#define VALVE_OPEN_ADDR_4 0x15
uint32_t valveOpenLength = 0;

union ArrayToInteger {
  byte array[4];
 uint32_t integer;
};

void setup() 
{
  Serial.begin(9600);

  while (!Serial);

  Serial.print("EPROM Length:");Serial.println( EEPROM.length() );

    EEPROM.write(DEVICE_ID_ADDR, 1);
    
    EEPROM.write(DRY_VAL_ADDR_1, highByte(960));
    EEPROM.write(DRY_VAL_ADDR_2, lowByte(960));

    EEPROM.write(HALT_RUN_ADDR, false);

    uint32_t int32Val = 14400000;//14400000 = 4hrs, 3600000 = 1hr
    uint8_t *vp = (uint8_t *)&int32Val;
    EEPROM.write(RUN_INTERVAL_ADDR_1, vp[0]);
    EEPROM.write(RUN_INTERVAL_ADDR_2, vp[1]);
    EEPROM.write(RUN_INTERVAL_ADDR_3, vp[2]);
    EEPROM.write(RUN_INTERVAL_ADDR_4, vp[3]); 

    int32Val = 180000;//180000 = 3mins, 240000 = 4mins
    vp = (uint8_t *)&int32Val;
    EEPROM.write(VALVE_OPEN_ADDR_1, vp[0]);
    EEPROM.write(VALVE_OPEN_ADDR_2, vp[1]);
    EEPROM.write(VALVE_OPEN_ADDR_3, vp[2]);
    EEPROM.write(VALVE_OPEN_ADDR_4, vp[3]); 

    ////

    client_id = EEPROM.read(DEVICE_ID_ADDR);
  
    dryValue = word( EEPROM.read(DRY_VAL_ADDR_1), EEPROM.read(DRY_VAL_ADDR_2) );

    haltIrrigation = EEPROM.read(HALT_RUN_ADDR);

     ArrayToInteger converter; //Create a converter
     converter.array[0] = EEPROM.read(RUN_INTERVAL_ADDR_1); //save something to each byte in the array
     converter.array[1] = EEPROM.read(RUN_INTERVAL_ADDR_2); //save something to each byte in the array
     converter.array[2] = EEPROM.read(RUN_INTERVAL_ADDR_3); //save something to each byte in the array
     converter.array[3] = EEPROM.read(RUN_INTERVAL_ADDR_4); //save something to each byte in the array
    
     runIrrigationInterval = converter.integer;
     converter.array[0] = EEPROM.read(VALVE_OPEN_ADDR_1); //save something to each byte in the array
     converter.array[1] = EEPROM.read(VALVE_OPEN_ADDR_2); //save something to each byte in the array
     converter.array[2] = EEPROM.read(VALVE_OPEN_ADDR_3); //save something to each byte in the array
     converter.array[3] = EEPROM.read(VALVE_OPEN_ADDR_4); //save something to each byte in the array
     valveOpenLength = converter.integer;

    Serial.print("client_id:");Serial.println(client_id);
    Serial.print("dryValue:");Serial.println(dryValue);
    Serial.print("haltIrrigation:");Serial.println(haltIrrigation);
    Serial.print("runIrrigationInterval:");Serial.println(runIrrigationInterval);
    Serial.print("valveOpenLength:");Serial.println(valveOpenLength);
    
}

void loop() {

}
