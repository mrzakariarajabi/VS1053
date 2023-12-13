#include <SPI.h>
#include <SD.h>
#include "SdFat.h"
#include "SdFatConfig.h"
//SdFat SD;

//SPIClass mySPI;
//#define SPI1_NSS_PIN PA4  
//#define SPI2_NSS_PIN PB12
SPIClass mySPI;
static SPIClass mySPI2(PB15, PB14, PB13);
#define ENABLE_DEDICATED_SPI 1
//#define SD_CONFIG SdSpiConfig(PD2, DEDICATED_SPI, SD_SCK_MHZ(18), &mySPI2)
#define SD_CONFIG SdSpiConfig(PD2, 1, 18, &mySPI2) //DEDICATED_SPI
//SdFs SD;

// VS1003 SCI Write Command byte is 0x02
#define VS_WRITE_COMMAND 0x02

// VS1003 SCI Read COmmand byte is 0x03
#define VS_READ_COMMAND  0x03

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

const int xCs = PF_7;
const int xDcs = PF_6;
const int xDreq = PC13;
const int xReset = PE6;

const int sdCs = PD2;
const int ctrlBtn = PE2;
bool ctrlBtnState = false;
bool ctrlBtnPressed = false;
bool ctrlBtnReleased = false;

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

char fileName[] = "wavFile6.WAV";
File myFile;
byte data[4];

unsigned char db[512];

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
  SPI.transfer(VS_READ_COMMAND);
  SPI.transfer(_reg);
  unsigned char response1 = SPI.transfer(0xFF);
  unsigned char response2 = SPI.transfer(0xFF);
  digitalWrite(xCs, HIGH);
  return ((unsigned int) response1 << 8) | (response2 & 0xFF);
}

void write_register(uint8_t _reg, uint16_t _value)
{
  control_mode_on();
  delayMicroseconds(1); // tXCSS
  SPI.transfer(VS_WRITE_COMMAND); // Write operation
  SPI.transfer(_reg); // Which register
  SPI.transfer(_value >> 8); // Send hi byte
  SPI.transfer(_value & 0xff); // Send lo byte
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

void setup() {
  // put your setup code here, to run once:
  /*
 SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(SPI1_NSS_PIN, OUTPUT);*/
  // Setup SPI 2
 /* SPI_2.begin(); //Initialize the SPI_2 port.
  SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
  pinMode(SPI2_NSS_PIN, OUTPUT);
    SPI==SPI_2;*/
  // initiate a serial port at 57600
  Serial.begin(115200);

  
  // Keep the chip in reset until we are ready
  pinMode(xReset, OUTPUT);
  pinMode(xCs, OUTPUT);
  pinMode(xDcs, OUTPUT);
  pinMode(xDreq, INPUT); 
  pinMode(sdCs, OUTPUT);
  pinMode(ctrlBtn, INPUT);

  digitalWrite(xReset, LOW);
  digitalWrite(xCs, HIGH);
  digitalWrite(xDcs, HIGH);
  digitalWrite(xReset, HIGH);

  digitalWrite(sdCs, HIGH);

  //while(!SD.begin(sdCs))
  while(!SD.begin(SD_CONFIG))
  {
    Serial.println("SD initialization failed..");
  }

  Serial.println("SD initialization successfull..");

  if(SD.exists(fileName))
  {
    SD.remove(fileName);
  }

  (myFile = SD.open(fileName, FILE_WRITE)) ? Serial.println("SD opening successfull") : Serial.println("SD opening failed");

  myFile.write(RIFFHeader0fn, sizeof(RIFFHeader0fn));

  // Write '0' (0x30) from address 51 to 503 
  for (int i = 0; i<452; i++)
  {
    myFile.write('0');
  }

  // write second RIFF header from 504 to 511
  myFile.write(RIFFHeader504fn, sizeof(RIFFHeader504fn));

  // initiate SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));

  write_register(SCI_VOL, 0x0000);
  write_register(SCI_BASS, 0);
  write_register(SCI_CLOCKF, 0x4430);

  write_register(SCI_AICTRL0, 12);
  delay(100);

  write_register(SCI_AICTRL1, 1024);
  delay(100);

  write_register(SCI_MODE, 0x1804);

  SPI.endTransaction();

  //myFile.close();

}

void updateAndCloseAudioFile()
{
  Serial.println(myFile.size());
  myFile.close();
  Serial.println("Recording off...");
  Serial.println("Closed the Audio file...");

  if(!(myFile = SD.open(fileName, O_WRITE)))
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

void loop() {
  
  if(digitalRead(ctrlBtn))
  {
    ctrlBtnPressed = true;
    delay(50);
  }
  
  else if(ctrlBtnPressed && !digitalRead(ctrlBtn))
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
    }

    // Serial.println(recordModeOn);
  }

  if(recordModeOn)
  {
    SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
    t = read_register_my_own(SCI_HDAT1);

    while(t >= 256 && t < 896)
    {
      for(int i=0;i<512;i=i+2)
      {
        w = read_register_my_own(SCI_HDAT0);
        db[i] = w >> 8;
        db[i+1] = w & 0xFF;
      }
  
      SPI.endTransaction();
  
      myFile.write(db, sizeof(db));

      t-=256;
    }    
  }
  
  else
  {
    //Serial.println("Record mode off");
  }
}
