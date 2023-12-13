#include <avr/pgmspace.h>
#include <SPI.h>

#include "SdFat.h"
#include "sdios.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(PE1, PE0); // RX, TX

const int8_t DISABLE_CS_PIN = -1;
static SPIClass mySPI1(PA7, PA6, PA5);

static SPIClass mySPI2(PB15, PB14, PB13);
#define SD_CONFIG SdSpiConfig(PD2, DEDICATED_SPI, SD_SCK_MHZ(18), &mySPI2)
SdFs sd;
/***********************************************************///not copy
// SCI Registers

const uint8_t SCI_MODE = 0x0;
const uint8_t SCI_STATUS = 0x1;
const uint8_t SCI_BASS = 0x2;
const uint8_t SCI_CLOCKF = 0x3;
const uint8_t SCI_DECODE_TIME = 0x4;
const uint8_t SCI_AUDATA = 0x5;
const uint8_t SCI_WRAM = 0x6;
const uint8_t SCI_WRAMADDR = 0x7;
const uint8_t SCI_HDAT0 = 0x8;
const uint8_t SCI_HDAT1 = 0x9;
const uint8_t SCI_AIADDR = 0xa;
const uint8_t SCI_VOL = 0xb;
const uint8_t SCI_AICTRL0 = 0xc;
const uint8_t SCI_AICTRL1 = 0xd;
const uint8_t SCI_AICTRL2 = 0xe;
const uint8_t SCI_AICTRL3 = 0xf;
const uint8_t SCI_num_registers = 0xf;

// SCI_MODE bits

const uint8_t SM_DIFF = 0;
const uint8_t SM_LAYER12 = 1;
const uint8_t SM_RESET = 2;
const uint8_t SM_OUTOFWAV = 3;
const uint8_t SM_EARSPEAKER_LO = 4;
const uint8_t SM_TESTS = 5;
const uint8_t SM_STREAM = 6;
const uint8_t SM_EARSPEAKER_HI = 7;
const uint8_t SM_DACT = 8;
const uint8_t SM_SDIORD = 9;
const uint8_t SM_SDISHARE = 10;
const uint8_t SM_SDINEW = 11;
const uint8_t SM_ADPCM = 12;
const uint8_t SM_ADCPM_HP = 13;
const uint8_t SM_LINE_IN = 14;

//unsigned char HelloMP3[] = {
//0xFF,0xF2,0x40,0xC0,0x19,0xB7,0x00,0x14,0x02,0xE6,0x5C, /* ..@.......\ */
//};

// VS1003 SCI Write Command byte is 0x02
#define VS_WRITE_COMMAND 0x02

// VS1003 SCI Read COmmand byte is 0x03
#define VS_READ_COMMAND  0x03

const int xCs = PF7;
const int xDcs = PF6;
const int xDreq = PC13;
const int xReset = PE6;
/***********************************************************************///not copy
const int sdCs = PD2;
const int ctrlBtn_player = PE3;

const int vs1003_chunk_size = 32;//uint8_t

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

File myFile;

String mp3_1 = "1.mp3";
String mp3_2 = "2.mp3";
String mp3_3 = "3.mp3";
String mp3_4 = "5.wav";

String audioName = mp3_2;

//String mp3Array[4] = {"wavFile6.WAV", "wavFile6.WAV", "wavFile6.WAV", "wavFile6.WAV"};
uint8_t mp3Pos = 0;
bool true_auduio_steup = true;
/****************************************************************************///not copy
uint16_t read_register(uint8_t _reg)
{
  uint16_t result;
  control_mode_on();
  delayMicroseconds(1); // tXCSS
  mySPI1.transfer(VS_READ_COMMAND); // Read operation
  mySPI1.transfer(_reg); // Which register
  result = mySPI1.transfer(0xff) << 8; // read high byte
  result |= mySPI1.transfer(0xff); // read low byte
  delayMicroseconds(1); // tXCSH
  await_data_request();
  control_mode_off();
  return result;
}

/****************************************************************************////not copy

void write_register(uint8_t _reg,uint16_t _value)
{
  control_mode_on();
  delayMicroseconds(1); // tXCSS
  mySPI1.transfer(VS_WRITE_COMMAND); // Write operation
  mySPI1.transfer(_reg); // Which register
  mySPI1.transfer(_value >> 8); // Send hi byte
  mySPI1.transfer(_value & 0xff); // Send lo byte
  delayMicroseconds(1); // tXCSH
  await_data_request();
  control_mode_off();
}
/*************************************************************************////not copy
void control_mode_on(void)
{
  digitalWrite(xDcs,HIGH);
  digitalWrite(xCs,LOW);
}

void await_data_request(void)
{
  while(!digitalRead(xDreq));
}

void control_mode_off(void)
{
  digitalWrite(xCs, HIGH);
}

void data_mode_on(void)
{
  digitalWrite(xCs,HIGH);
  digitalWrite(xDcs,LOW);
}

void data_mode_off(void)
{
  digitalWrite(xDcs,HIGH);
}
/***********************************************************************///not copy
void sdi_send_buffer(const uint8_t* data, int len)
{
  data_mode_on();
  while ( len )
  {
    await_data_request();
    delayMicroseconds(3);

    int chunk_length = min(len, vs1003_chunk_size);
    len -= chunk_length;
    while ( chunk_length-- )
      mySPI1.transfer(*data++);
      //Serial.println(*data++);
  }
  data_mode_off();
}

void disableVS1003(void)
{
  digitalWrite(xCs, HIGH); 
  digitalWrite(sdCs, LOW);  
}

void enableVS1003(void)
{
  digitalWrite(xCs, LOW);  
  digitalWrite(sdCs, HIGH);  
}
/*******************************************************************/
void playMp3fromSDcard(String &mp3)
{
  int byteCount;
  uint8_t mp3Data[32];

  disableVS1003();
  
  myFile = sd.open(mp3);

  while(myFile.available())
  {
    await_data_request();
    delayMicroseconds(3);

    byteCount = myFile.read(mp3Data, 32);

    if(byteCount == 0)
    {
      myFile.close();
      break;
    }
    
    digitalWrite(xDcs, LOW);
    //enableVS1003();
    
    for(uint8_t i=0;i<32;i++)
    {
      mySPI1.transfer(mp3Data[i]);
    }
    
    //disableVS1003();
    digitalWrite(xDcs, HIGH);

    if(buttonPushCounter % 2 == 0)
    {
      myFile.close();
      mp3Pos++;
      if(mp3Pos >= 5)
      {
        mp3Pos = 0;
      }

      delay(1000);
      return;
   //   buttonPushCounter++;
    }
  }

  enableVS1003();
}
/*******************************************************************/
void setup_player(){


  mySPI1.begin();

  mySPI1.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));


  digitalWrite(xReset, HIGH);
  
  await_data_request();

  write_register(SCI_AUDATA, 44101); // 44.1kHz stereo

  write_register(SCI_MODE, _BV(SM_SDINEW) | _BV(SM_RESET));
 
  write_register(SCI_CLOCKF, 0xE800); 
  
  
  
  }
  /**************************************************************************/
void Interrupt_player(){
  
  buttonState = digitalRead(ctrlBtn_player);
  // put your main code here, to run repeatedly:
  //sdi_send_buffer(HelloMP3, sizeof(HelloMP3));
   if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }

  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
 // mySerial.println(true_auduio_steup);
 setup_player();
playMp3fromSDcard(audioName);
  }
  
void setup() {

  mySerial.begin(115200);
  
  // put your setup code here, to run once:
  pinMode(xReset, OUTPUT);
  pinMode(xCs, OUTPUT);
  pinMode(xDcs, OUTPUT);
  pinMode(xDreq, INPUT);
  pinMode(sdCs, OUTPUT);
  pinMode(ctrlBtn_player, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ctrlBtn_player), Interrupt_player, CHANGE);
  disableVS1003();

  while(!sd.begin(SD_CONFIG))//sdCs
  {
    mySerial.println("SD initialization failed...");
  }

  mySerial.println("SD initialization successfull...");
  enableVS1003();
  
  digitalWrite(xReset, LOW);
  digitalWrite(xCs, HIGH);
  digitalWrite(xDcs, HIGH);
  digitalWrite(sdCs, HIGH);
  digitalWrite(xCs, LOW);
  digitalWrite(xReset, HIGH);
  
  
}

void loop() {

        mySerial.println(buttonPushCounter);
      

//  buttonPushCounter++;
  
  //mp3Pos++;
  //Serial.println(mp3Pos);
  //configure();
}
