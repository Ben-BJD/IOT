#include <TimedAction.h>
#include <RH_RF95.h>
#include <EEPROM.h>
#include "encrypt.h"
#include <dht.h>

/***********
// ------- BEGIN: lora vars ------------------------------------------------------------------------------------------------------------------------
************/

RH_RF95 rf95;
ENCRYPT encrypt_decrypt;

//Define the timeout to re-start to listen the broadcast info from server to establish network.
//Default: 10 minutes 
#define TIMEOUT 600000

//Define the LoRa frequency use for this client
float frequency = 915.0;

//Define the encrypt encryptkey. so different group of LoRa devices won't communicate with each other.
//TODO: Change this to your own key
unsigned char encryptkey[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//encryption encryptkey

uint8_t sent_count = 0;//Client send count, increase after sent data. 

uint16_t crcdata = 0; // crc data
uint16_t recCRCData = 0; // 
int detected = 0; // check if detected server's broadcast message
int flag = 0; // 
long start = 0;
long total_time = 0;//check how long doesn't receive a server message

bool msgSent = true;
bool ackTX = true;
uint8_t ackRetries = 0; 
uint8_t MAX_ACK_RETIRES = 5;

/***********
// ------- END: lora vars ------------------------------------------------------------------------------------------------------------------------
************/




/***********
// ------- BEGIN: program vars ------------------------------------------------------------------------------------------------------------------------
************/

boolean debug = false;

#define DHT11_PIN 4
#define RELAY_PIN 3

dht DHT;

double temperature = DHTLIB_INVALID_VALUE;
double humidity = DHTLIB_INVALID_VALUE;

// Analog input pin that the soil moisture sensor is attached to
const int moistureSensorPin = A0;  

// value read from the soil moisture sensor
int moistureSensorValue = 0; 

#define RUN_RESULT_INIT 0
#define RUN_RESULT_TEMP_FAIL 1
#define RUN_RESULT_TEMP_LOW 2
#define RUN_RESULT_WATER 3
#define RUN_RESULT_NO_WATER 4
#define RUN_RESULT_HALTED 5
int runIrrigationResult = RUN_RESULT_INIT;

#define TEMP_SAMPLE_DELAY 500
#define TEMP_SAMPLE_CAP 10

boolean radioCommsEnabled = false;
bool runIrrigationFlag = true;

bool cancelRadioFlag = false;
bool valveOpen = false;

//Eprom vars

// Client ID address in EEPROM.

#define ID_ADDRESS 0x00
uint8_t client_id = 0;

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

void radioCommCancel()
{
    detected = 0;
    flag = 0;
    radioCommsEnabled = false;
}

void closeValveTrigger()
{
    valveOpen = false;
}

TimedAction closeValveAction = TimedAction(valveOpenLength,closeValveTrigger);

void runIrrigation()
{
    runIrrigationFlag = false;
  
    if(debug)
    {
        Serial.println(F(" --- --- Begin RUN Irrigation --- ---"));
    }
    
    if(!haltIrrigation) 
    {
         temperature = DHTLIB_INVALID_VALUE;
         humidity    = DHTLIB_INVALID_VALUE;
    
        //loop below with a delay to get temp
          //if undefined fault

        for(int i=0;i<TEMP_SAMPLE_CAP;i++)
        {
            int chk = DHT.read11(DHT11_PIN);

            if(debug)
            {
                Serial.println(DHT.humidity);
                Serial.println(DHT.temperature);
            }
            
            if( DHT.humidity != DHTLIB_INVALID_VALUE )
            {
              humidity = DHT.humidity;
              if(debug)
              {
                  Serial.print(F("Humidity = "));
                  Serial.println(humidity);
              }
            }
      
            if( DHT.temperature != DHTLIB_INVALID_VALUE )
            {
              temperature = DHT.temperature;
              
              if(debug)
              {
                  Serial.print(F("Temperature = "));
                  Serial.println(temperature);
              }
              break;
            }
          
            delay(TEMP_SAMPLE_DELAY); 
        }
    
        if( temperature != DHTLIB_INVALID_VALUE )
        {
            if(temperature < 4.5)
            {
              runIrrigationResult = RUN_RESULT_TEMP_LOW;
            }
            else
            {
              //soil check
              moistureSensorValue = analogRead(moistureSensorPin);
              if(debug)
              {
                  Serial.print(F("Moisture: "));Serial.println(moistureSensorValue);
              }
    
              if(moistureSensorValue >= dryValue)//dryval = 850
              {
                  //routine for releasing the valve
                  closeValveAction.reset();
                  valveOpen = true;
                  
    
                  runIrrigationResult = RUN_RESULT_WATER;
              }
              else
              {
                  runIrrigationResult = RUN_RESULT_NO_WATER;
              }
            }
        }
        else
        {
            runIrrigationResult = RUN_RESULT_TEMP_FAIL;
        }
    }
    else
    {
        runIrrigationResult = RUN_RESULT_HALTED; 
    }
    
    if(debug)
    {
        Serial.println(F(" --- --- End RUN Irrigation --- ---"));
    }

    msgSent = false;
    radioCommsEnabled = true;
  
}


void runIrrigationTrigger()
{
    runIrrigationFlag = true;
}

TimedAction runIrrigationAction = TimedAction(runIrrigationInterval,runIrrigationTrigger);



void openValve()
{
  digitalWrite(RELAY_PIN, HIGH);
}

void closeValve()
{
  digitalWrite(RELAY_PIN, LOW);
}

void setup()
{
  if(debug)
  {
      Serial.begin(9600);
      Serial.print(F("Init"));
  }
  pinMode(RELAY_PIN, OUTPUT);
  closeValve();

  rf95.init();
  //Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);
  
  client_id = EEPROM.read(ID_ADDRESS);//Get Client id.
  
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

    runIrrigationAction.setInterval( runIrrigationInterval );
    closeValveAction.setInterval( valveOpenLength );
    
    if(debug)
     {
        Serial.print(F("dryValue set at: "));Serial.println(dryValue);
        
        Serial.print(F("haltIrrigation set at: "));Serial.println(haltIrrigation);
    
        Serial.print(F("runIrrigationInterval set at: "));Serial.println(runIrrigationInterval);
        
        Serial.print(F("valveOpenLength set at: "));Serial.println(valveOpenLength);
     }
}

void loop()
{
  runIrrigationAction.check();
  closeValveAction.check();

  if(!valveOpen)
  {
    closeValve();
  }
  else
  {
    openValve();
  }

  if( runIrrigationFlag )
  {
      runIrrigation();
  }

  if( cancelRadioFlag )
  {
      radioCommCancel();
      cancelRadioFlag = false;
  }
  
  if(radioCommsEnabled)
  { 
      if(detected == 0)//has not joined to the LoRa Network, listen the broadcast. 
      {
        //detect if there is server broadcast package and join the LoRa Network
        listen_server();   
     }
     else
     {
        polling_detect();
     }
  }

  delay(100);
  
}

//radio functions


uint16_t calcByte(uint16_t crc, uint8_t b)
{
    uint32_t i;
    crc = crc ^ (uint32_t)b << 8;
    for ( i = 0; i < 8; i++)
     {
       if ((crc & 0x8000) == 0x8000)
       crc = crc << 1 ^ 0x1021;
       else
       crc = crc << 1;
     }  
    return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer,uint32_t length)
{
    uint16_t wCRC16=0;
    uint32_t i;
    if (( pBuffer==0 )||( length==0 ))
    {
      return 0;
    }
    for ( i = 0; i < length; i++)
    { 
      wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}

uint16_t recdata( unsigned char* recbuf,int Length)
{
    crcdata = CRC16(recbuf,Length-2);//Calculate the CRC for the received message
    recCRCData = recbuf[Length-1];//get the CRC high byte 
    recCRCData = recCRCData<<8;// get the CRC low byte
    recCRCData |= recbuf[Length-2];//get the receive CRC
}

void listen_server(void)
{  
    if (rf95.waitAvailableTimeout(100))
    { 
       if(debug){ Serial.println(F("Get Message. ")); }
       uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//buffer to store the server response message
       uint8_t len = sizeof(buf);// data length
       if (rf95.recv(buf, &len))//check if receive data is correct 
       {    
            if(debug){Serial.println(F("waiting for broadcast message"));   }      
            encrypt_decrypt.btea(buf, -len, encryptkey);// decode receive message
            if(debug){
            Serial.println(buf[0]);
            Serial.println(buf[1]);
            Serial.println(buf[2]);
            Serial.println(buf[3]);}
            if(buf[0] == 'B' && buf[1] == 'C' && buf[2] == ':' && buf[3] == 255  )//Get Broadcast message from Server, send a join request                                                                                          
            {
               int delay_ms = 0;
               delay_ms= random(100, 500);//generate a random delay. this is to avoid the channel congestion.This may happen when all clients start 
                                          //to send the join message to the server at the same time after get the broadcast message
               delay(delay_ms);

               uint8_t join[4] = {0}; // Construct a join message
               join[0] = 'J';
               join[1] = 'R';
               join[2] = ':';
               join[3] = client_id;// Put Client ID
               if(debug){
               Serial.print(F("Send a Join Request Message, Client ID is:"));
               Serial.println(join[3]);
               }
               int length = sizeof(join);//get data length 
               encrypt_decrypt.btea(join, length, encryptkey);//encrypt the outgoing message.
               rf95.send(join, sizeof(join));// Send a Join Message
               rf95.waitPacketSent();// wait for send finished 
               flag = 1;
            }
           if(flag == 1) 
            {
              if(debug){
                Serial.println(F("send Join request,waiting for Join ACK"));
                Serial.println(buf[0]);
                Serial.println(buf[1]);
                Serial.println(buf[2]);
                Serial.println(buf[3]);
              }
               if(buf[0] == 'J' && buf[1] == 'A'  && buf[2] == ':' && buf[3] == client_id)  //successful if get join ACK, otherwise, listen for broadcast again. 
                {
                  if(debug){
                    Serial.println(F("Get Join ACK, Join successfulm entering polling mode"));
                  }
                  detected = 1;
                  flag = 0;     
               }
            }
       }
    } 
   delay(100);
}

 void polling_detect(void)
 {
    if(debug && !msgSent)
    {
        Serial.println(F("Message Needs to Be sent"));
    }
  
   // detect if there is timeout to get response from server.
   if (rf95.waitAvailableTimeout(500))//check if there is polling request
    { 
      start = millis();
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//get message 
      uint8_t len = sizeof(buf);//get message length
      if (rf95.recv(buf, &len))//check if receive message is correct
      { 
        encrypt_decrypt.btea(buf, -len, encryptkey);//decode
    
       if(buf[0] == 'D' && buf[1] == 'R' && buf[2] == ':' && buf[3] == client_id )//check if we receive a data request message
       {            
          if( buf[4] == '1' )// rx message
          {
              int msgLen = ( len/sizeof(uint8_t) );
              if(debug){
              Serial.print(F("Received an RX Message: "));
              }

              //set any eprom/local vars 

              if(buf[5] == '1')
              {
                  EEPROM.write(DRY_VAL_ADDR_1, buf[6]);
                  EEPROM.write(DRY_VAL_ADDR_2, buf[7]);
                  dryValue = word( buf[6], buf[7] );

                  if(debug)
                   {
                      Serial.print(F("dryValue set at: "));Serial.println(dryValue);
                   }
              }

              if(buf[8] == '1')
              {
                  EEPROM.write(HALT_RUN_ADDR, buf[9]);
                  haltIrrigation = buf[9];

                  if(debug)
                   {
                      Serial.print(F("haltIrrigation set at: "));Serial.println(haltIrrigation);
                   }
              }

              ArrayToInteger converter; //Create a converter
              if(buf[10] == '1')
              {
                  EEPROM.write(RUN_INTERVAL_ADDR_1, buf[11] );
                  EEPROM.write(RUN_INTERVAL_ADDR_2, buf[12] );
                  EEPROM.write(RUN_INTERVAL_ADDR_3, buf[13] );
                  EEPROM.write(RUN_INTERVAL_ADDR_4, buf[14] );
                   
                   converter.array[0] = buf[11]; //save something to each byte in the array
                   converter.array[1] = buf[12]; //save something to each byte in the array
                   converter.array[2] = buf[13]; //save something to each byte in the array
                   converter.array[3] = buf[14]; //save something to each byte in the array
                  
                   runIrrigationInterval = converter.integer;

                   if(debug)
                   {
                      Serial.print(F("runIrrigationInterval set at: "));Serial.println(runIrrigationInterval);
                   }
              }

              if(buf[20] == '1')
              {
                  EEPROM.write(VALVE_OPEN_ADDR_1, buf[21] );
                  EEPROM.write(VALVE_OPEN_ADDR_2, buf[22] );
                  EEPROM.write(VALVE_OPEN_ADDR_3, buf[23] );
                  EEPROM.write(VALVE_OPEN_ADDR_4, buf[24] );
                   
                   converter.array[0] = buf[21]; //save something to each byte in the array
                   converter.array[1] = buf[22]; //save something to each byte in the array
                   converter.array[2] = buf[23]; //save something to each byte in the array
                   converter.array[3] = buf[24]; //save something to each byte in the array
                  
                   valveOpenLength = converter.integer;

                   if(debug)
                   {
                      Serial.print(F("valveOpenLength set at: "));Serial.println(valveOpenLength);
                   }
              }
              
              if(debug){
              Serial.println(F(""));
              }
          }
          else if( buf[4] == '2' )// ack message
          {
            msgSent = true;
            ackRetries = 0;
            if(debug)
            {
              Serial.println(F("TX ACKED"));
              
            }
            
            cancelRadioFlag = true;
          }
          else
          {
            if(debug){
            Serial.println(F("No RX"));}
          }

          if(sent_count == 255)
          {
            sent_count = 0;
          }
          
          sent_count++;
                    
        
        char  data[50] = {0};//data to be sent
        data[0] =  'D';
        data[1] =  'S';
        data[2] = ':';
        data[3] = client_id;//put client ID
        data[4] = ':';
        data[5] = sent_count;//increase after sent

        if( !msgSent )
        {
            if(ackTX)
            {
              data[6] = '2';
              
            }
            else
            {
              data[6] = '1';
            }

            // temperature
           
            char temperatureBuff[4] = "";
            if(temperature == DHTLIB_INVALID_VALUE)
            {
              temperatureBuff[0] = '-';
              temperatureBuff[1] = '9';
              temperatureBuff[2] = '9';
              temperatureBuff[3] = '\0';
            }
            else
            {
              dtostrf(temperature, 3, 0, temperatureBuff);
            }
            
            if(debug)
            {
                Serial.println(F("----- RADIO VARS ----"));
                Serial.println(F(""));
                Serial.print(F("TempAsDouble:: "));Serial.println(temperature);
                Serial.print(F("TempAsChar:: "));Serial.println(temperatureBuff);
            }

            data[7] = 'T';
            data[8] = temperatureBuff[0];
            data[9] = temperatureBuff[1];
            data[10] = temperatureBuff[2];

            // humidity

            char humidityBuff[4] = "";
            if(humidity == DHTLIB_INVALID_VALUE)
            {
              humidityBuff[0] = '-';
              humidityBuff[1] = '9';
              humidityBuff[2] = '9';
              humidityBuff[3] = '\0';
            }
            else
            {
              dtostrf(humidity, 3, 0, humidityBuff);
            }
            
            if(debug)
            {
                Serial.print(F("HumidityAsDouble:: "));Serial.println(humidity);
                Serial.print(F("HumidityAsChar:: "));Serial.println(humidityBuff);
            }

            data[11] = 'H';
            data[12] = humidityBuff[0];
            data[13] = humidityBuff[1];
            data[14] = humidityBuff[2];
            
            // moistureSensorValue

            char moistureBuff [5];
            sprintf( moistureBuff, "%04i", moistureSensorValue );

            if(debug)
            {
                Serial.print(F("Moisture as int:: "));Serial.println(moistureSensorValue);
                Serial.print(F("Moisture AsChar:: "));Serial.println(moistureBuff);
            }

            data[15] = 'S';
            data[16] = moistureBuff[0];
            data[17] = moistureBuff[1];
            data[18] = moistureBuff[2];
            data[19] = moistureBuff[3];
            
            // runIrrigationResult
            
            char runResultBuff [2];
            sprintf( runResultBuff, "%01i", runIrrigationResult );

            if(debug)
            {
                Serial.print(F("Run Rsult as int:: "));Serial.println(runIrrigationResult);
                Serial.print(F("Run Rsult AsChar:: "));Serial.println(runResultBuff);
            }

            data[20] = 'R';
            data[21] = runResultBuff[0];

            if(debug){
            Serial.println(F("TX queued to Send"));}

            msgSent = true;

            if(ackTX)
            {
              msgSent = false;
              ackRetries++; 

              if(ackRetries == MAX_ACK_RETIRES)
              {
                ackRetries = 0;
                msgSent = true;
              }
            }
        }
        else
        {
            data[6] = '0';//0 = No Data, 1= TX MSG, 2= ACK
        } 
        
        int dataLength = strlen(data);//get data length
        uint16_t crcData = CRC16((unsigned char*)data,dataLength);//calculate CRC
         
        unsigned char sendBuf[50]={0};
        strcpy((char*)sendBuf,data);//copy data to sendbuf
        
        sendBuf[dataLength] = (unsigned char)crcData;
     
        sendBuf[dataLength+1] = (unsigned char)(crcData>>8);

        int length = strlen((char*)sendBuf);//get data length 
        encrypt_decrypt.btea(sendBuf, length, encryptkey);//encryption
        
        rf95.send(sendBuf, strlen((char*)sendBuf));//send message
        rf95.waitPacketSent();//wait till send finished 
        
        detected = 1; 
        total_time = 0;  
        
     }
     else
      {
        //Check how long we have not received a data request.Client will enter into listening mode if timeout
        if(debug){
        Serial.println(F("Get message, but not data request message"));}
        total_time += millis( )-start;//get total time out
        if(total_time > TIMEOUT)
         {
           detected = 0;
           total_time = 0;
           if(debug){
           Serial.println(F("polling listening time out, listening network set up again"));}
         }
       }
     }
   else
    {
      if(debug){
      Serial.println(F("No reply, is rf95_server running?"));//didn't get a LoRa message. 
      }
    }
  }  
}

