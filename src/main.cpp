// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SD.h>


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
File myFile;


#define LED_PIN 13
bool blinkState = false;

//int16_t ax, ay, az;

uint8_t counter = 0;
uint8_t buffer[42*3*2];
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    mySerial.begin(115200);

    // initialize device
    mySerial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setRate(7);
    //accelgyro.setRate(2);

    accelgyro.setFIFOEnabled(1);
    accelgyro.setAccelFIFOEnabled(1);
    accelgyro.setXGyroFIFOEnabled(0);
    accelgyro.setYGyroFIFOEnabled(0);
    accelgyro.setZGyroFIFOEnabled(0);
    //accelgyro.resetFIFO();  // Clear any garbage data

    // verify connection
    mySerial.println("Testing device connections...");
    mySerial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    mySerial.println("Updating internal sensor offsets...");
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

    if (!SD.begin(SPI_FULL_SPEED,chipSelect)) {
      mySerial.println("initialization failed. Things to check:");
      mySerial.println("1. is a card inserted?");
      mySerial.println("2. is your wiring correct?");
      mySerial.println("3. did you change the chipSelect pin to match your shield or module?");
      mySerial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
      while (true);
    }
    //delay(5000);
    
    Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
    myFile = SD.open("test.txt",  O_CREAT | O_WRITE | O_APPEND);
    
    accelgyro.resetFIFO();
}

void loop() {
    // read raw accel/gyro measurements from device
    char data[120];
    
    

    mySerial.println(accelgyro.getFIFOCount());
    // Only read when we have at least one complete 6-byte packet (accel only)
    if (accelgyro.getFIFOCount() >= 120){
        // Read 6-byte packet (accel XYZ only, gyro FIFO is disabled)
        accelgyro.getFIFOBytes((uint8_t*)data, 120);
        mySerial.println(accelgyro.getFIFOCount());
        // Parse and print accel X, Y, Z (signed 16-bit)
        int16_t ax = (int16_t)((data[0] << 8) | data[1]);
        int16_t ay = (int16_t)((data[2] << 8) | data[3]);
        int16_t az = (int16_t)((data[4] << 8) | data[5]);
        myFile = SD.open("test.txt",  O_CREAT | O_WRITE | O_APPEND);
        myFile.print(data);
        myFile.close();
        //mySerial.print(ax); mySerial.print(" ");
        //mySerial.print(ay); mySerial.print(" ");
        //mySerial.print(az);
        //mySerial.println();
    }

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        mySerial.print("t/a/g:\t");
        mySerial.print(micros());
        mySerial.print("\t");
        mySerial.print(ax); mySerial.print("\t");
        mySerial.print(ay); mySerial.print("\t");
        mySerial.print(az); mySerial.print("\t");
        mySerial.print(gx); mySerial.print("\t");
        mySerial.print(gy); mySerial.print("\t");
        mySerial.println(gz);

    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
      myFile = SD.open("test.txt", FILE_WRITE);
      myFile.write((uint8_t)(ax >> 8)); myFile.write((uint8_t)(ax & 0xFF));
      myFile.write((uint8_t)(ay >> 8)); myFile.write((uint8_t)(ay & 0xFF));
      myFile.write((uint8_t)(az >> 8)); myFile.write((uint8_t)(az & 0xFF));
      myFile.write((uint8_t)(gx >> 8)); myFile.write((uint8_t)(gx & 0xFF));
      myFile.write((uint8_t)(gy >> 8)); myFile.write((uint8_t)(gy & 0xFF));
      myFile.write((uint8_t)(gz >> 8)); myFile.write((uint8_t)(gz & 0xFF));// MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
      myFile.close();
    #endif


    
    #ifdef UTF8_SD
      
      char buffer[80];
      sprintf(buffer, "t/a/g:\t%lu\t%d\t%d\t%d\t%d\t%d\t%d\n", micros(), ax, ay, az, gx, gy, gz);
      myFile.print(buffer);
      
      counter +=1;
      if (counter == 100){
        myFile.flush();
        counter = 0;
      }

    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    //delay(100);
    
}