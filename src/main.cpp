#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CS_PIN SS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define LOG_INTERVAL_USEC 600
#define MAX_FILE_SIZE 10000000UL
#define BUFFER_SIZE 120  // 10 samples * 12 bytes (accel+gyro) per chunk

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"

#include "MPU6050.h"
//#include <SD.h>
#include "SdFat.h"


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


	// This method allows processing "SEND_BREAK" requests sent by
	// the USB host. Those requests indicate that the host wants to
	// send a BREAK signal and are accompanied by a single uint16_t
	// value, specifying the duration of the break. The value 0
	// means to end any current break, while the value 0xffff means
	// to start an indefinite break.
	// readBreak() will return the value of the most recent break
	// request, but will return it at most once, returning -1 when
	// readBreak() is called again (until another break request is
	// received, which is again returned once).
	// This also mean that if two break requests are received
	// without readBreak() being called in between, the value of the
	// first request is lost.
	// Note that the value returned is a long, so it can return
	// 0-0xffff as well as -1.
	int32_t readBreak();

	// These return the settings specified by the USB host for the
	// serial port. These aren't really used, but are offered here
	// in case a sketch wants to act on these settings.
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

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

//#define UTF8_SD
#define DEBUG

DummySerial dummySerial;

#ifdef DEBUG
#define mySerial Serial
#else
#define mySerial dummySerial
#endif

const int chipSelect = 10;
SdFs sd;
FsFile file;
#define LED_PIN 13
bool blinkState = false;

uint8_t counter = 0;
//uint8_t buffer[42*3*2];
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
    // initialize device
    mySerial.println("Initializing I2C devices...");
    accelgyro.initialize();
    
    // Set DLPF to 0 for 8kHz gyro rate (or keep at 1+ for 1kHz)
    // DLPF_CFG=0: 8kHz sample rate, 260Hz accel BW, 256Hz gyro BW
    // DLPF_CFG=1: 1kHz sample rate, 184Hz accel BW, 188Hz gyro BW
    accelgyro.setDLPFMode(0);  // 8kHz base rate for gyro
    
    // Sample Rate = Base Rate / (1 + SMPLRT_DIV)
    // With DLPF=0: 8kHz / (1 + rate)
    // rate=0 -> 8000Hz, rate=1 -> 4000Hz, rate=3 -> 2000Hz, rate=7 -> 1000Hz
    accelgyro.setRate(7);  // 8kHz (but accel limited to 1kHz max)
    
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
    
    //accelgyro.resetFIFO();  // Clear any garbage data

    // verify connection
//    mySerial.println("Testing device connections...");
 //   mySerial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
//    mySerial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    mySerial.print(accelgyro.getXAccelOffset()); mySerial.print("\t"); // -76
    mySerial.print(accelgyro.getYAccelOffset()); mySerial.print("\t"); // -2359
    mySerial.print(accelgyro.getZAccelOffset()); mySerial.print("\t"); // 1688
    mySerial.print(accelgyro.getXGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print(accelgyro.getYGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print(accelgyro.getZGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print("\n");
    accelgyro.setXGyroOffset(97);
    accelgyro.setYGyroOffset(112);
    accelgyro.setZGyroOffset(-22);

    accelgyro.setXAccelOffset(-836);
    accelgyro.setYAccelOffset(-1713);
    accelgyro.setZAccelOffset(3524);

    mySerial.print(accelgyro.getXAccelOffset()); mySerial.print("\t"); // -76
    mySerial.print(accelgyro.getYAccelOffset()); mySerial.print("\t"); // -2359
    mySerial.print(accelgyro.getZAccelOffset()); mySerial.print("\t"); // 1688
    mySerial.print(accelgyro.getXGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print(accelgyro.getYGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print(accelgyro.getZGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print("\n");


    mySerial.println("Initializing SD card...");
    
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
 //     mySerial.print("SD init attempt ");
      mySerial.print(attempt + 1);
      mySerial.println(" failed, retrying...");
      sd.end();
      delay(100);
    }
    
    if (!sdInitialized) {
      mySerial.println("FF");
      while (true);
    }
  
    openNewFile();
    
 //   Serial.println("initialization done.");
    pinMode(LED_PIN, OUTPUT);
    accelgyro.resetFIFO();
    mySerial.println("logging");
    start = millis();
}

void loop() {
    uint16_t fifoCount = accelgyro.getFIFOCount();
    
    // Check for FIFO overflow (max is 1024 bytes)
    if (fifoCount >= 1024) {
      mySerial.println("FIFO overflow - resetting");
      accelgyro.resetFIFO();
      return;
    }
    
    // Read in two 120-byte chunks (20 samples total with accel+gyro)
    if (fifoCount >= 240 && millis() - start < 10000) {
      mySerial.println(accelgyro.getFIFOCount());
      
      char data[240];
      // First chunk: 120 bytes (10 samples)
      accelgyro.getFIFOBytes((uint8_t*)data, 120);
      // Second chunk: 120 bytes (10 samples)
      accelgyro.getFIFOBytes((uint8_t*)data + 120, 120);
      
      file.write(data, 240);
      mySerial.println(accelgyro.getFIFOCount());
      
      // Only sync every 10 writes to reduce SD overhead
      syncCounter++;
      if (syncCounter >= 10) {
        file.sync();
        syncCounter = 0;
      }
    }
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}