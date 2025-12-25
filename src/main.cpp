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
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

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
    mySerial.begin(38400);

    // initialize device
    mySerial.println("Initializing I2C devices...");
    accelgyro.initialize();

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

    accelgyro.setXAccelOffset(959);
    accelgyro.setXAccelOffset(-1629);
    accelgyro.setXAccelOffset(1577);

    mySerial.print(accelgyro.getXAccelOffset()); mySerial.print("\t"); // -76
    mySerial.print(accelgyro.getYAccelOffset()); mySerial.print("\t"); // -2359
    mySerial.print(accelgyro.getZAccelOffset()); mySerial.print("\t"); // 1688
    mySerial.print(accelgyro.getXGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print(accelgyro.getYGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print(accelgyro.getZGyroOffset()); mySerial.print("\t"); // 0
    mySerial.print("\n");


  //  mySerial.print("Initializing SD card...");
//
  //  if (!SD.begin(chipSelect)) {
  //    mySerial.println("initialization failed. Things to check:");
  //    mySerial.println("1. is a card inserted?");
  //    mySerial.println("2. is your wiring correct?");
  //    mySerial.println("3. did you change the chipSelect pin to match your shield or module?");
  //    mySerial.println("Note: press reset button on the board and reopen this mySerial Monitor after fixing your issue!");
  //  while (true);
  //}
    

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

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
        mySerial.write((uint8_t)(ax >> 8)); mySerial.write((uint8_t)(ax & 0xFF));
        mySerial.write((uint8_t)(ay >> 8)); mySerial.write((uint8_t)(ay & 0xFF));
        mySerial.write((uint8_t)(az >> 8)); mySerial.write((uint8_t)(az & 0xFF));
        mySerial.write((uint8_t)(gx >> 8)); mySerial.write((uint8_t)(gx & 0xFF));
        mySerial.write((uint8_t)(gy >> 8)); mySerial.write((uint8_t)(gy & 0xFF));
        mySerial.write((uint8_t)(gz >> 8)); mySerial.write((uint8_t)(gz & 0xFF));// MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(100);
}