// WIRING 
// MPU6050
// SCL - A5
// SDA - A4
//
// SD READER
// CS - 10
// SCK -13
// MOSI-11
// MISO-12 



#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CS_PIN SS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define LOG_INTERVAL_USEC 600
#define MAX_FILE_SIZE 10000000UL // is about 104 minutes of logging
#define BUFFER_SIZE 168  // 7 samples * 16 bytes (4 timestamp + 12 IMU) per chunk


//#define UTF8_SD
#define DEBUG

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#include <SD.h>
#include "SdFat.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class DummySerial : public Stream
{
private:
	int peek_buffer;
public:
	DummySerial() { peek_buffer = -1; };
	void begin(unsigned long) {};
	void begin(unsigned long, uint8_t){};
	void end(void){};

	virtual int available(void){return 0;};
	virtual int peek(void){return 0;};
	virtual int read(void){return 0;};
	virtual int availableForWrite(void){return 0;};
	virtual void flush(void){};
	virtual size_t write(uint8_t){return 0;};
	virtual size_t write(const uint8_t*, size_t){return 0;};
	using Print::write; // pull in write(str) and write(buf, size) from Print
	operator bool();
	uint32_t baud(){return 0;};
	uint8_t stopbits(){return 0;};
	uint8_t paritytype(){return 0;};
	uint8_t numbits(){return 0;};
	bool dtr(){return 0;};
	bool rts(){return 0;};
	enum {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2,
	};
	enum {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4,
	};

};

DummySerial dummySerial;

#ifdef DEBUG
#define mySerial Serial
#else
#define mySerial dummySerial
#endif

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const int chipSelect = 10;
SdFs sd;
FsFile file;
#define LED_PIN 13
bool blinkState = false;

uint8_t counter = 0;
uint16_t fileNum = 0;
uint16_t syncCounter = 0;
unsigned long start;
void openNewFile() {
  if (file.isOpen()) {
    file.close();
  }

  char filename[10];
  // Find first non-existent file number
  while (fileNum < 100) {
    sprintf(filename, "%02d.bin", fileNum);
    if (!sd.exists(filename)) {
      break;  // Found available filename
    }
    fileNum++;
  }
  
  file = sd.open(filename, O_RDWR | O_CREAT | O_EXCL);
  if (!file) {
    while (1);  // Halt on error
  }

  if (!file.preAllocate(MAX_FILE_SIZE)) {
    file.close();
    while (1);
  }
  
  fileNum++;  // Ready for next file
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mySerial.begin(115200);
    // initialize MPU6050
    //mySerial.println(F("Initializing I2C devices..."));
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    accelgyro.setDLPFMode(0);  // 8kHz base rate for gyro
    // rate=0 -> 8000Hz, rate=1 -> 4000Hz, rate=3 -> 2000Hz, rate=7 -> 1000Hz
    accelgyro.setRate(7);  // 8kHz (but accel limited to 1kHz max)
    // setup MPU6050 FIFO
    accelgyro.setFIFOEnabled(1);
    accelgyro.setAccelFIFOEnabled(1);
    accelgyro.setXGyroFIFOEnabled(1);
    accelgyro.setYGyroFIFOEnabled(1);
    accelgyro.setZGyroFIFOEnabled(1);
    accelgyro.setTempFIFOEnabled(0);
    accelgyro.setSlave0FIFOEnabled(0);
    accelgyro.setSlave1FIFOEnabled(0);
    accelgyro.setSlave2FIFOEnabled(0);
    accelgyro.setSlave3FIFOEnabled(0);
    
    // verify connection
    //mySerial.println(F("Testing device connections..."));
    //mySerial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // use the code below to change accel/gyro offset values
    
//    mySerial.println(F("Updating internal sensor offsets..."));
    // -76	-2359	1688	0	0	0
    mySerial.print(F("\n"));
    accelgyro.setXGyroOffset(97);
    accelgyro.setYGyroOffset(112);
    accelgyro.setZGyroOffset(-22);

    accelgyro.setXAccelOffset(-1911);  // Test with no offsets
    accelgyro.setYAccelOffset(-553);
    accelgyro.setZAccelOffset(1433);
    mySerial.println("stay still calibrating");
    delay(1000);
    accelgyro.CalibrateAccel(10);
    mySerial.println("calibrating gyro");
    accelgyro.CalibrateGyro(10);
    mySerial.println("calibation finished");
    Serial.println("\nat 600 Readings");
    //accelgyro.PrintActiveOffsets();

    mySerial.print(accelgyro.getXAccelOffset()); mySerial.print(F("\t")); // -76
    mySerial.print(accelgyro.getYAccelOffset()); mySerial.print(F("\t")); // -2359
    mySerial.print(accelgyro.getZAccelOffset()); mySerial.print(F("\t")); // 1688
    mySerial.print(accelgyro.getXGyroOffset()); mySerial.print(F("\t")); // 0
    mySerial.print(accelgyro.getYGyroOffset()); mySerial.print(F("\t")); // 0
    mySerial.print(accelgyro.getZGyroOffset()); mySerial.print(F("\t")); // 0
    mySerial.print(F("\n"));
    
    // Debug: Check actual range settings
    mySerial.print(F("Accel range: "));
    mySerial.println(accelgyro.getFullScaleAccelRange());  // Should be 0 for ±2g
    mySerial.print(F("Gyro range: "));
    mySerial.println(accelgyro.getFullScaleGyroRange());  // Should be 0 for ±250°/s

    //while(1){}
    mySerial.println(F("Initializing SD card..."));
    
    // Close any open files and end previous SD session
    if (file.isOpen()) {
      file.close();
    }
    sd.end();
    
    // Reset CS pin to ensure clean state
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(10);
    
    // Try multiple times to initialize
    bool sdInitialized = false;
    for (int attempt = 0; attempt < 200; attempt++) {
      if (sd.begin(SD_CONFIG)) {
        sdInitialized = true;
        break;
      }
 //     mySerial.print(F("SD init attempt "));
      mySerial.print(attempt + 1);
      mySerial.println(F(" failed, retrying..."));
      sd.end();
      delay(1000);
    }
    
    if (!sdInitialized) {
      mySerial.println(F("FF"));
      while (true);
    }
  
    openNewFile();
    
 //   Serial.println(F("initialization done."));
    pinMode(LED_PIN, OUTPUT);
    accelgyro.resetFIFO();
    mySerial.println(F("logging"));
    start = millis();
}

void loop() {
    uint16_t fifoCount = accelgyro.getFIFOCount();
    
    // Check for FIFO overflow (max is 1024 bytes)
    if (fifoCount >= 1024) {
      mySerial.println(F("FIFO overflow - resetting"));
      accelgyro.resetFIFO();
      return;
    }
    
    // Read IMU data and add timestamps (16 bytes per sample: 4 timestamp + 12 IMU)
    if (fifoCount >= BUFFER_SIZE && millis() ) {  // 14 samples * 12 bytes - start < 10000
      //mySerial.println(accelgyro.getFIFOCount());
      
      char imuData[BUFFER_SIZE];  // 14 samples of raw IMU data
      char outputData[224];  // 14 samples with timestamps (14 * 16)
      
      // Read IMU data in two chunks
      accelgyro.getFIFOBytes((uint8_t*)imuData, 84);  // 7 samples
      accelgyro.getFIFOBytes((uint8_t*)imuData + 84, 84);  // 7 more samples
      
      // Interpolate timestamps for each sample
      // Sample rate = 1000 Hz (1 ms per sample) from setRate(7) with DLPF=0
      unsigned long currentTime = millis()-start;
      for (int i = 0; i < 14; i++) {
        // Interpolate: oldest sample first, newest sample last
        // Sample 0 was captured 13 ms ago, sample 13 was just captured
        unsigned long timestamp = currentTime - (13 - i);
        
        // Write timestamp (4 bytes, big-endian)
        outputData[i * 16 + 0] = (timestamp >> 24) & 0xFF;
        outputData[i * 16 + 1] = (timestamp >> 16) & 0xFF;
        outputData[i * 16 + 2] = (timestamp >> 8) & 0xFF;
        outputData[i * 16 + 3] = timestamp & 0xFF;
        // Copy IMU data (12 bytes)
        memcpy(&outputData[i * 16 + 4], &imuData[i * 12], 12);
      }
      
      file.write(outputData, 224);
      //mySerial.println(accelgyro.getFIFOCount());
      
      // Only sync every 10 writes to reduce SD overhead
      syncCounter++;
      if (syncCounter >= 20) {
        file.sync();
        syncCounter = 0;
      }
    }
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}