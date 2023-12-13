                            #include <avr/pgmspace.h>
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"
#include <SoftwareSerial.h>
////////////////////////////////////////////////////////////////spi class Definition
const int8_t DISABLE_CS_PIN = -1;
static SPIClass mySPI1(PB5, PB4, PB3);

static SPIClass mySPI2(PB15, PB14, PB13);//MOSI  MISO  SCK
#define SD_CONFIG SdSpiConfig(PA8, DEDICATED_SPI, SD_SCK_MHZ(18), &mySPI2)
SdFs sd;
//////////////////////////////////////////////////////////////serial class Definition
//SoftwareSerial mySerial(PA10, PA9); // RX, TX

///////////////////////////////////////////////////////////// VS1003 SCI Write Command byte is 0x02
#define VS_WRITE_COMMAND 0x02

//////////////////////////////////////////////////////////// VS1003 SCI Read COmmand byte is 0x03
#define VS_READ_COMMAND  0x03
volatile unsigned long T1 = 0;
// Size of the "wav" file set to 100*512 bytes including the header files
const unsigned char RIFFHeader0fn[] = {
    'R' , 'I' , 'F' , 'F' , // Chunk ID (RIFF)
    0xF8, 0xC7, 0x00, 0x00, // Chunk payload size (calculate after rec!)
    'W' , 'A' , 'V' , 'E' , // RIFF resource format type

    'f' , 'm' , 't' , ' ' , // Chunk ID (fmt )
    0x14, 0x00, 0x00, 0x00, // Chunk payload size (0x14 = 20 bytes)
    0x11, 0x00,             // Format Tag (IMA ADPCM)
    0x01, 0x00,             // Channels (1)
    0x40, 0x1f, 0x00, 0x00, // Sample Rate, 0x3e80 = 16.0kHz
    0x75, 0x12, 0x00, 0x00, // Average Bytes Per Second
    0x00, 0x01,             // Data Block Size (256 bytes) 
    0x04, 0x00,             // ADPCM encoded bits per sample (4 bits)
    0x02, 0x00,             // Extra data (2 bytes)
    0xf9, 0x01,             // Samples per Block (505 samples)

    'f' , 'a' , 'c' , 't' , // Chunk ID (fact)
    0xc8, 0x01, 0x00, 0x00, // Chunk payload size (456 bytes (zeropad!))
    0x96, 0x86, 0x01, 0x00  // Number of Samples (calculate after rec!)
};

const unsigned char RIFFHeader504fn[] = {
    'd' , 'a' , 't' , 'a' , // Chunk ID (data)
    0x00, 0xC6, 0x00, 0x00, // Chunk payload size (calculate after rec!)
};

const int xCs = PB1;
const int xDcs = PB0;
const int xDreq = PC13;
const int xReset = PB11;

const int sdCs = PA8;
const int ctrlBtn_recorder = PA1;
const int ctrlBtn_player = PA0;

bool ctrlBtnState = false;
bool ctrlBtnPressed = false;
bool ctrlBtnReleased = false;
bool MesaIsaFlaga = false;
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

uint16_t t, w;
bool recordModeOn;

int iCount = 0;
long int loopCount = 0;

char fileName[] = "mp3_6.mp3";
const char com[3][10]={
  "mp3_6.mp3" , "mp3_5.mp3" ,"mp3_4.mp3"
  
  };
File myFile;
byte data[4];

unsigned char db[512];


const int vs1003_chunk_size = 32;//uint8_t

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

int buttonPushCounter_recorder = 0;   // counter for the number of button presses
int buttonState_recorder = 0;         // current state of the button
int lastButtonState_recorder = 0;     // previous state of the button

String mp3_1 = "1.mp3";
String mp3_2 = "2.mp3";
String mp3_3 = "3.mp3";
String mp3_4 = "5.wav";

String audioName = "mp3_6.mp3";//mp3_2//"mp3_6.mp3"

//String mp3Array[4] = {"wavFile6.WAV", "wavFile6.WAV", "wavFile6.WAV", "wavFile6.WAV"};
uint8_t mp3Pos = 0;

void control_mode_on(void)
{
  digitalWrite(xDcs, HIGH);
  digitalWrite(xCs,LOW);
}

void await_data_request(void)
{
  while (!digitalRead(xDreq));
}

void control_mode_off(void)
{
  digitalWrite(xCs, HIGH);
}

uint16_t read_register_my_own(uint8_t _reg)
{
  while(!digitalRead(xDreq));
  digitalWrite(xCs, LOW);
  mySPI1.transfer(VS_READ_COMMAND);
  mySPI1.transfer(_reg);
  unsigned char response1 = mySPI1.transfer(0xFF);
  unsigned char response2 = mySPI1.transfer(0xFF);
  digitalWrite(xCs, HIGH);
  return ((unsigned int) response1 << 8) | (response2 & 0xFF);
}

void write_register(uint8_t _reg, uint16_t _value)
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

void data_mode_on(void)
{
  digitalWrite(xCs, HIGH);
  digitalWrite(xDcs, LOW);
}

void data_mode_off(void)
{
  digitalWrite(xDcs, HIGH);
}

void setVolume(uint8_t vol)
{
  uint16_t value = vol;
  value <<= 8;
  value |= vol;

  write_register(SCI_VOL, value); // VOL
}
/********************************************************************************/
void updateAndCloseAudioFile()
{
    Serial.println(myFile.size());
  myFile.close();
  Serial.println("Recording off...");
  Serial.println("Closed the Audio file...");

  if(!(myFile = sd.open(fileName, O_WRITE)))
  {
    Serial.println("Failed to open record file in O_WRITE mode");
  }

  unsigned long int paySize1 = myFile.size() - 8;
  unsigned long int numSamp = ((myFile.size() - 512) / 256) * 505;
  unsigned long int paySize2 = myFile.size() - 512;

  data[3] = paySize1 >> 24; // shift it over so only last byte exists
  data[2] = paySize1 >> 16;
  data[1] = paySize1 >> 8;
  data[0] = paySize1;

  //    Update "RIFF" header chunk at byte 4
  myFile.seek(4);
  myFile.write(data, sizeof(data));

  data[3] = numSamp >> 24;  // shift it over so only last byte exists
  data[2] = numSamp >> 16;
  data[1] = numSamp >> 8;
  data[0] = numSamp;
  
  //    Update "FACT" header chunk at byte 48
  myFile.seek(48);
  myFile.write(data, sizeof(data));
  
  data[3] = paySize2 >> 24; // shift it over so only last byte exists
  data[2] = paySize2 >> 16;
  data[1] = paySize2 >> 8;
  data[0] = paySize2;
  
  //    Update "DATA" header chunk at byte 508
  myFile.seek(508);
  myFile.write(data, sizeof(data));

  Serial.println(myFile.size());
    
  myFile.close();
}
/*****************************************************************************/
void setup_recorder(){
   // put your setup code here, to run once:
  // Keep the chip in reset until we are ready
   if(sd.exists(fileName))
  {
    sd.remove(fileName);
  }

  (myFile = sd.open(fileName, FILE_WRITE)) ? Serial.println("SD opening successfull") : Serial.println("SD opening failed");

  myFile.write(RIFFHeader0fn, sizeof(RIFFHeader0fn));

  // Write '0' (0x30) from address 51 to 503 
  for (int i = 0; i<452; i++)
  {
    myFile.write('0');
  }

  // write second RIFF header from 504 to 511
  myFile.write(RIFFHeader504fn, sizeof(RIFFHeader504fn));

  // initiate SPI
  mySPI1.begin();
  mySPI1.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));

  write_register(SCI_VOL, 0x0000);
  write_register(SCI_BASS, 0);
  write_register(SCI_CLOCKF, 0x4430);

  write_register(SCI_AICTRL0, 12);
  delay(100);

  write_register(SCI_AICTRL1, 1024);
  delay(100);

  write_register(SCI_MODE, 0x1804);

  mySPI1.endTransaction();

  //myFile.close();
  }
  /*************************************************************************/
  void loop_recorder(){
  //  mySerial.println("loop_recorder..");
    if(!digitalRead(ctrlBtn_recorder))
  {
    ctrlBtnPressed = true;
    delay(50);
  }
  
  else if(ctrlBtnPressed && !digitalRead(ctrlBtn_recorder))
  {
    ctrlBtnReleased = true;
  }

  if(ctrlBtnPressed && ctrlBtnReleased)
  {
    ctrlBtnPressed = false;
    ctrlBtnReleased = false;

    if(recordModeOn)
    {
      recordModeOn = false;

      updateAndCloseAudioFile();
    }
    else
    {
      recordModeOn = true;
      Serial.println("Recording on...");
      T1 = millis();
    }

    // Serial.println(recordModeOn);
  }

  if(recordModeOn)
  {
    if (millis() - T1 > 30000)
    {
      recordModeOn = false;
      updateAndCloseAudioFile();
    }
    mySPI1.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
    t = read_register_my_own(SCI_HDAT1);
//    Serial.println(t);
    while(t >= 256 && t < 896)
    {
      for(int i=0;i<512;i=i+2)
      {
        w = read_register_my_own(SCI_HDAT0);
        db[i] = w >> 8;
        db[i+1] = w & 0xFF;
      }
  
      mySPI1.endTransaction();

//      for(int i=0;i<512;i=i+1)
//      {
//        Serial.print(i);
//        Serial.print(" : ");
//        Serial.println((int)db[i]);
//      }
      
      myFile.write(db, sizeof(db));

      t-=256;
    }    
  }
  
  else
  {
    //Serial.println("Record mode off");
  }    
 }
 //////////////////////////////////////////////////////////////////////////////player
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
/******************************************************************/
void playMp3fromSDcard(String &mp3)
{
   Serial.println("start playy..");
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
    //  NVIC_SystemReset();
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
   //   NVIC_SystemReset();
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
  // myFile.close();
    Serial.println("end playy..");
  NVIC_SystemReset();

  enableVS1003();
}
/**********************************************************************/
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
 
//setup_recorder();

  }
/*****************************************************************************/
void toggle(){
 
  MesaIsaFlaga = true;
  
  }

 /************************************************************************/
void setup() {
 Serial.begin(115200);
   pinMode(xReset, OUTPUT);
  pinMode(xCs, OUTPUT);
  pinMode(xDcs, OUTPUT);
  pinMode(xDreq, INPUT); 
  pinMode(sdCs, OUTPUT);
  pinMode(ctrlBtn_recorder, INPUT_PULLUP);
  pinMode(ctrlBtn_player, INPUT_PULLUP);
 // pinMode(PA8, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ctrlBtn_player), Interrupt_player, RISING);//Interrupt_player
  attachInterrupt(digitalPinToInterrupt(ctrlBtn_recorder), toggle, RISING);//Interrupt_player //CHANGE
  
  digitalWrite(xReset, LOW);
  digitalWrite(xCs, HIGH);
  digitalWrite(xDcs, HIGH);
  digitalWrite(xReset, HIGH);

  digitalWrite(sdCs, HIGH);
 // disableVS1003();
  //while(!SD.begin(sdCs))
  while(!sd.begin(SD_CONFIG))
  {
    Serial.println("SD initialization failed..");
  }

  Serial.println("SD initialization successfull..");
  // enableVS1003();
 // setup_player();
  setup_recorder();
}

void loop() {
 // NVIC_SystemReset();
  //Serial.println("loop..");
  if(MesaIsaFlaga){
    
   // setup_recorder();
      loop_recorder();
      MesaIsaFlaga ==false;
    }

  


}
