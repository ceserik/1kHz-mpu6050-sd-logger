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


      mySerial.print("Initializing SD card...");

    if (!SD.begin(chipSelect)) {
      mySerial.println("initialization failed. Things to check:");
      mySerial.println("1. is a card inserted?");
      mySerial.println("2. is your wiring correct?");
      mySerial.println("3. did you change the chipSelect pin to match your shield or module?");
      mySerial.println("Note: press reset button on the board and reopen this mySerial Monitor after fixing your issue!");
    while (true);
  }
    

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
// 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-11 - added PID offset generation at begninning Generates first offsets 
//                 - in @ 6 seconds and completes with 4 more sets @ 10 seconds
//                 - then continues with origional 2016 calibration code.
//      2016-11-25 - added delays to reduce sampling rate to ~200 Hz
//                   added temporizing printing during long computations
//      2016-10-25 - requires inequality (Low < Target, High > Target) during expansion
//                   dynamic speed change when closing in
//      2016-10-22 - cosmetic changes
//      2016-10-19 - initial release of IMU_Zero
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release of MPU6050_RAW

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

  If an MPU6050 
      * is an ideal member of its tribe, 
      * is properly warmed up, 
      * is at rest in a neutral position, 
      * is in a location where the pull of gravity is exactly 1g, and 
      * has been loaded with the best possible offsets, 
then it will report 0 for all accelerations and displacements, except for 
Z acceleration, for which it will report 16384 (that is, 2^14).  Your device 
probably won't do quite this well, but good offsets will all get the baseline 
outputs close to these target values.

  Put the MPU6050 on a flat and horizontal surface, and leave it operating for 
5-10 minutes so its temperature gets stabilized.

  Run this program.  A "----- done -----" line will indicate that it has done its best.
With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take 
a few minutes to get there.

  Along the way, it will generate a dozen or so lines of output, showing that for each 
of the 6 desired offsets, it is 
      * first, trying to find two estimates, one too low and one too high, and
      * then, closing in until the bracket can't be made smaller.

  The line just above the "done" line will look something like
    [567,567] --> [-1,2]  [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1]  [-25,-24] --> [0,3] [5,6] --> [0,4]
As will have been shown in interspersed header lines, the six groups making up this
line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
X gyro, Y gyro, and Z gyro, respectively.  In the sample shown just above, the trial showed
that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration, 
and so on.

  The need for the delay between readings (usDelay) was brought to my attention by Nikolaus Doppelhammer.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;
      
void ForceHeader()
  { LinesOut = 99; }
    
void GetSmoothed()
  { int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          mySerial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums
//    unsigned long usForN = micros() - Start;
//    mySerial.print(" reading at ");
//    mySerial.print(1000000/((usForN+N/2)/N));
//    mySerial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
      { Smoothed[i] = (Sums[i] + N/2) / N ; }
  } // GetSmoothed

void Initialize()
  {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mySerial.begin(9600);

    // initialize device
    mySerial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    mySerial.println("Testing device connections...");
    mySerial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    mySerial.println("PID tuning Each Dot = 100 readings");
  /*A tidbit on how PID (PI actually) tuning works. 
    When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
    integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
    uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
    to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
    set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
    integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
    noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
    readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
    the fact it reacts to any noise.
  */
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        mySerial.println("\nat 600 Readings");
        accelgyro.PrintActiveOffsets();
        mySerial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        mySerial.println("700 Total Readings");
        accelgyro.PrintActiveOffsets();
        mySerial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        mySerial.println("800 Total Readings");
        accelgyro.PrintActiveOffsets();
        mySerial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        mySerial.println("900 Total Readings");
        accelgyro.PrintActiveOffsets();
        mySerial.println();    
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        mySerial.println("1000 Total Readings");
        accelgyro.PrintActiveOffsets();
     mySerial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
  } // Initialize

void SetOffsets(int TheOffsets[6])
  { accelgyro.setXAccelOffset(TheOffsets [iAx]);
    accelgyro.setYAccelOffset(TheOffsets [iAy]);
    accelgyro.setZAccelOffset(TheOffsets [iAz]);
    accelgyro.setXGyroOffset (TheOffsets [iGx]);
    accelgyro.setYGyroOffset (TheOffsets [iGy]);
    accelgyro.setZGyroOffset (TheOffsets [iGz]);
  } // SetOffsets

void ShowProgress()
  { if (LinesOut >= LinesBetweenHeaders)
      { // show header
        mySerial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
      } // show header
    mySerial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
      { mySerial.print(LBRACKET);
        mySerial.print(LowOffset[i]),
        mySerial.print(COMMA);
        mySerial.print(HighOffset[i]);
        mySerial.print("] --> [");
        mySerial.print(LowValue[i]);
        mySerial.print(COMMA);
        mySerial.print(HighValue[i]);
        if (i == iGz)
          { mySerial.println(RBRACKET); }
        else
          { mySerial.print("]\t"); }
      }
    LinesOut++;
  } // ShowProgress

// Forward declaration to allow use before definition
void SetAveraging(int NewN);

void PullBracketsIn()
  { boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    mySerial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking) 
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working
   
  } // PullBracketsIn

void PullBracketsOut()
  { boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    mySerial.println("expanding:");
    ForceHeader();
 
    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values
      
        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut

void SetAveraging(int NewN)
  { N = NewN;
    mySerial.print("averaging ");
    mySerial.print(N);
    mySerial.println(" readings each time");
   } // SetAveraging

void setup()
  { Initialize();
    for (int i = iAx; i <= iGz; i++)
      { // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel 
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);
    
    PullBracketsOut();
    PullBracketsIn();
    
    mySerial.println("-------------- done --------------");
  } // setup
 
void loop()
  {
  } // loop
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(100);
}