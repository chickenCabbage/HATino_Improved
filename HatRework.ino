const char versionStr[] = "HAT V 1.10";
//#define GY521
#define LSM303

/* Arduino head tracker for FaceTrackNoIR and OpenTrack
 * with the MPU6050 "GY-521" module, using DMP MotionApps v4.1.
 * 
 * HAT 14/04/2013 by FuraX49.
 * 
 * Some fixes done by Juan M. Casillas <juan.casillas@gmail.com>
 * Code cleanup, serial fixes and porting to Arduino Micro.
 * Additional refactoring by Alon Shiboleth <alon.shiboleth@gmail.com>
 */

/* PINS:
 * 
 * 	NANO/UNO | LEO/MICRO | 6050	| 9DOF
 * ==========|===========|======|=====
 * 	   A4	 |    D2	 | 	   SDA
*	   A5	 |    D3	 | 	   SCL
* 		 D2 (INT0)		 | INT	|  -
 * 	  		 5V  		 | VCC	| VIN
 * 	  		 GND		 |	   GND
 *
 * To set the I2C address to 0x68 pull the GY-521 AD0 pin
 * ADO low or leave floating. To set address 0x69, pull AD0 high.
 * View the #define AD0_HIGH
 */

/* HATIRE FRAME STRUCTURE:
 * {
 *	int16		- START (0xAAAA)
 *	int16		- Code
 *	float[3]	- Rotation YPR
 *	float[3]	- Translation XYZ
 *	int16		- STOP (0x5555)
 * }
 * 
 * HATIRE FRAME CODES:
 * 0000 - 0999:	Valid frame with floats Y, P, R, and reserved room for 3 more (translation).
 * 1000 - 1999:	Reserved for gyro calibration
 * 2000 - 2999:	Reserved for accelerometer calibration
 * 3000 - 3999:	Reserved for magnometer calibratiom
 * 4000 - 4999: Reserved for drift calibration
 * 5000 - 5999: 24-byte arr for info messages
 * 9000 - 9999: 24-byte arr for error messages
 */

/* HANDY TERMS:
 * Y, P, R = Yaw, pitch, roll.
 * D.M.P. = digital motion processor
 */

#define INT_PIN 2	//where INT is attached. wire INT to an interrupt-capable pin.
#define USE_EEPROM	//define to save and load values to/from EEPROM
//#define BT_SERIAL	//define to use a bluetooth HC-05 serial device.
#ifdef GY521
	//#define AD0_HIGH	//define if the MPU6050 I2C address is 0x69 rather than 0x68
#endif

#include <Wire.h>

typedef struct {
	int16_t start;		//2	bytes - start
	uint16_t frameNum;	//2	bytes - frame code
	float gyro[3];		//12 bytes - gyro [y, p, r]
	float acc[3];		//12 bytes - translation [x, y, z]
	int16_t stop;		//2 bytes - stop
} HatireStruct;			//total 30 bytes
HatireStruct hatireStruct;

typedef struct {
	int16_t start;		//2 bytes - start
	uint16_t code;		//2 bytes - code info
	char msg[24];		//24 bytes - message
	int16_t stop;		//2 bytes - stop
} MsgStruct;			//total 30 bytes
MsgStruct msgStruct;

typedef struct {
	byte rate;
	double gyro_offset[3];
	double acc_offset[3];
} CalibrateStruct;
CalibrateStruct calValuesStruct;

bool pendingCal = false;	//set true when calibrating is asked
int calFrameCounter = 0;
#define calFrameAmnt 5

#ifdef BT_SERIAL
	#define BTKeyPin	10	//where the BT's key/enable pin is attached.
	#define BTSerTxPin	9	//softwareSerial TX for BT
	#define BTSerRxPin	8	//softwareSerial RX for BT
	#include <SoftwareSerial.h>
	SoftwareSerial BTSerial(BTSerRxPin, BTSerTxPin);
#endif

#ifdef USE_EEPROM
	#include <avr/eeprom.h>
#else
	#ifdef GY521
	//gyro offsets, as given by the Arduino IMU_Zero example.
	//IMU_Zero by Robert R. Fenichel (bob@fenichel.net)
	#define accOffsetX	-2600
	#define accOffsetY	-633
	#define accOffsetZ	797
	#define gyroOffsetX	115
	#define gyroOffsetY	-16
	#define gyroOffsetZ	6
	#endif
#endif

#ifdef GY521
	#include <I2Cdev.h>
	#include <MPU6050_6Axis_MotionApps20.h>
	#ifndef AD0_HIGH
		#define MPU_ADDR 0x68	//the default address.
	#else
		#define MPU_ADDR 0x69	//if the AD0 pin is pulled HIGH, the I2C address changes to 0x69
	#endif
	
	MPU6050 mpu(MPU_ADDR);
	
	//MPU control/status vars:
	bool dmpLoaded = false;	//set true if DMP communicated and loaded successfuly
	bool dmpReady = false;	//set true if DMP init was successful and ready to give an interrupt
	
	uint8_t mpuStatus;		//holds actual interrupt status byte from MPU
	//aditionally, holds during init the return status after each device operation (0 = success, !0 = error)
	
	uint16_t packetSize;	//expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;		//amount of all bytes currently in FIFO
	uint8_t fifoBuffer[64];	//FIFO contents storage buffer
	
	//orientation/motion vars:
	#define RAD_2_DEG (180 / M_PI)
	Quaternion quaternion;	//[w, x, y, z]	quaternion container
	VectorFloat gravity;	//[x, y, z]		gravity vector
	
	/* INTERRUPT HANDLING:
	 * When an interrupt occurs, dmpDataReady() is called. See: attachInterrupt().
	 * dmpDataReady() raises a flag that allows handling of the interrupt later.
	 */
	volatile bool mpuInterrupt = false;
	void dmpDataReady() {
		mpuInterrupt = true;
	}
#endif

#ifdef LSM303
	#include <Adafruit_Sensor.h>
	#include <Adafruit_LSM303_U.h>
	#include <Adafruit_LSM303_U.h>
	#include <Adafruit_9DOF.h>
	
	Adafruit_9DOF dof = Adafruit_9DOF();
	Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
	Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

	bool active = false;
#endif

//SEND CODE OVER SERIAL TO PC/BT:
void sendCodeSerial(uint16_t code, const char msg[24], bool EOL) {
	msgStruct.code = code;
	//copy passed msg to msgStruct.msg:
	memset(msgStruct.msg, 0x00, 24);	//clear msgStruct.msg (set 0x00 to 24 bytes)
	strcpy(msgStruct.msg, msg);			//copy msg to msgStruct.msg
		msgStruct.msg[23] = '\n';		//if EOL, add \n at the last character

	//send HATIRE message to PC:
	//pass write() a pointer to msgStruct, for 30 bytes (msgStruct size)
	#ifndef BT_SERIAL
	Serial.write((byte*) &msgStruct, 30);
	#else
	BTSerial.write((byte*) &msgStruct, 30);
	#endif
}

//INITIALIZE:
void setup() {
	//initialize I2C:
	Wire.begin(); //join I2C bus
	TWBR = 24;
	//put the i2c bus at 200kHz to prevent buffer overflow - doesn't work so much.
	//the problems are related to the serial port (to low).
	//try to write less packets instead.

	//initialize serial communication with PC:
	#ifndef BT_SERIAL
	Serial.begin(115200);	//max speed supported by OpenTrack
	while (!Serial);		//wait for USB enumeration
	#else
	//initialize serial with BT:
	pinMode(BTKeyPin, OUTPUT);		//BTKeyPin will pull the BT key pin to switch module to/from AT mode
	digitalWrite(BTKeyPin, LOW);	//do not transmit (yet)
	BTSerial.begin(115200);			//HC-05 default speed in AT command mode
	#endif

	sendCodeSerial(2000, versionStr, true);

	//set initial codes:
	hatireStruct.start = 0xAAAA;
	hatireStruct.frameNum = 0;
	hatireStruct.stop = 0x5555;

	msgStruct.start = 0xAAAA;
	msgStruct.code = 0;
	msgStruct.stop = 0x5555;

	#ifdef GY521
		sendCodeSerial(5001, "Initializing I2C", true);
		mpu.initialize();
	#endif
	
	//verify connection:
	sendCodeSerial(5002, "Testing connections", true);
	#ifdef GY521
		if(mpu.testConnection())
			sendCodeSerial(5003, "MPU6050 connection OK", true);
		else
			sendCodeSerial(9007, "MPU6050 ERROR CNX", true);
	#endif
	#ifdef LSM303
		if(accel.begin() && mag.begin()) {
			sendCodeSerial(5003, "LSM303 connection OK", true);
			active = true;
		}
		else
			sendCodeSerial(9007, "LSM303 ERROR CNX", true);
	#endif

	while(Serial.available() && Serial.read()); //clear the buffer of any responses

	#ifdef GY521
		//init the gyro/accelerometer:
		mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
		mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
		//load and configure the DMP
		sendCodeSerial(5004, "Initializing DMP...", true);
		mpuStatus = mpu.dmpInitialize();
	
		//make sure initialization successful. mpu!=0 if error.
		if(mpuStatus == 0) {
			#ifdef USE_EEPROM
				//calibrate:
				sendCodeSerial(5005, "Reading saved params...", true); //read EEPROM saved params
				readFromEEPROM();
			#else
				mpu.setXGyroOffset(gyroOffsetX);
				mpu.setYGyroOffset(gyroOffsetY);
				mpu.setZGyroOffset(gyroOffsetZ);
		
				mpu.setXAccelOffset(accOffsetX);
				mpu.setYAccelOffset(accOffsetY);
				mpu.setZAccelOffset(accOffsetZ);
			#endif
	
			//turn on the DMP, now that it's ready
			sendCodeSerial(5006, "Enabling DMP...", true);
			mpu.setDMPEnabled(true);
			dmpLoaded = true; //set global flag
	
			//enable Arduino interrupt detection
			sendCodeSerial(5007, "Enabling interrupt", true);
			attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
			mpuStatus = mpu.getIntStatus();
	
			//set the DMP Ready flag so the main loop() function knows it's okay to use it
			sendCodeSerial(5000, "HAT BEGIN", true);
			dmpReady = true;
			//get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
		}
		else {
			//error! mpuStatus was not 0
			//1 = initial memory load failed
			//2 = DMP configuration updates failed
			dmpLoaded = false;
			sendCodeSerial(9000 + mpuStatus, "DMP initialization failed", true);
		}
	#endif //end of #ifdef GY521
} //end of setup()

//set the current offsets to 0
void flushOffset() {
	calValuesStruct.gyro_offset[0] = 0;
	calValuesStruct.gyro_offset[1] = 0;
	calValuesStruct.gyro_offset[2] = 0;
	calValuesStruct.acc_offset[0] = 0;
	calValuesStruct.acc_offset[1] = 0;
	calValuesStruct.acc_offset[2] = 0;
}

#ifdef USE_EEPROM
//casting to void tells the compiler that calValuesStruct is a complete object in and of itself.
//or, in this case, a pointer to a complete object.
void saveToEEPROM() {
	eeprom_write_block((const void*) &calValuesStruct, (void*) 0, sizeof(calValuesStruct));
}
void readFromEEPROM() {
	eeprom_read_block((void*) &calValuesStruct, (void*) 0, sizeof(calValuesStruct));
}
#endif

//TODO: adapt to LSM303 inclusion
//handle serial input from FTNIR or OpenTrack:
void serialEvent() {
	char serialCommand = (char) Serial.read();
	switch(serialCommand) {
		case 'S': //START
			sendCodeSerial(5001, "HAT START", true);
			hatireStruct.frameNum = 0;
			#ifdef GY521
				if(dmpLoaded) {
					mpu.resetFIFO();
					attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
					//when INT pin rises, call dmpDataReady().
					mpu.setDMPEnabled(true);
					dmpReady = true; //DMP has been started
				}
				else { //DMP is not yet loaded. must be loaded first - init in setup()
					//this will run only if setup() has not finished properly and mpuStatus != 0
					sendCodeSerial(9011, "Error DMP not loaded", true);
				}
			#endif
			#ifdef LSM303
				active = true;
			#endif
			break;

		case 's': //STOP
			sendCodeSerial(5002, "HAT STOP", true);
			#ifdef GY521
				if(dmpReady) {
					mpu.setDMPEnabled(false);
					detachInterrupt(digitalPinToInterrupt(INT_PIN));
					dmpReady = false;
				}
			#endif
			#ifdef LSM303
				active = false;
			#endif
			break;

		case 'R': //RESET
			sendCodeSerial(5003, "HAT RESET", true);
			#ifdef GY521
				if(!dmpLoaded)
					sendCodeSerial(9011, "Error DMP not loaded", true);
					
				//absolutely re-init everything:
				mpu.setDMPEnabled(false);
				detachInterrupt(0);
				mpu.resetFIFO();
				hatireStruct.frameNum = 0;
				dmpReady = false;
			#endif
			#ifdef LSM303
				active = false;
			#endif

			setup();
			
			flushOffset();
			pendingCal = true;
			break;

		case 'C': //CALIBRATE
			calFrameCounter = 0;
			flushOffset();
			pendingCal = true;
			break;

		case 'V': //SHOW VERSION
			sendCodeSerial(2000, versionStr, true);
			break;

		#ifdef BT_SERIAL
		case 'I': //INFO
			BTSerial.println();
			BTSerial.print("Version : \t");
			BTSerial.println(versionStr);

			BTSerial.println("Gyroscopes offsets");
			for (int i = 0; i <= 2; i++) {
				BTSerial.print(i);
				BTSerial.print(" : ");
				BTSerial.print(calValuesStruct.gyro_offset[i]);
				BTSerial.println();
			}

			BTSerial.println("Accelerometers offsets");
			for (int i = 0; i <= 2; i++) {
				BTSerial.print(i);
				BTSerial.print(" : ");
				BTSerial.print(calValuesStruct.acc_offset[i]);
				BTSerial.println();
			}
			break;
		#endif

		default:
			break;
	} //end of switch(serialCommand)
} //end of serialEvent()

//after init, loop forever:
void loop() {
	if(Serial.available() > 0) serialEvent();
	//write to hatireStruct, depending on the chosen sensor. later, hatireStruct will be sent to the PC.
	#ifdef GY521
		//if programming failed, don't try to do anything
		if(dmpReady) { //if the MPU6050 has started measuring
			//wait until an interrupt occurs and enough data has been passed:
			while (!mpuInterrupt && (fifoCount < packetSize));
	
			//reset interrupt flag and get INT_STATUS byte
			mpuInterrupt = false;
			mpuStatus = mpu.getIntStatus();
			fifoCount = mpu.getFIFOCount(); //get current FIFO count
			//check for overflow (this should never happen unless our code is too inefficient):
			//bit no. 4 is an overflow flag. see "MPU-6000 and MPU-6050 Register Map and Descriptions"
			//at https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
			if((mpuStatus & 0b10000) || (fifoCount == 1024)) {
				//reset the FIFO and send a message:
				mpu.resetFIFO();
				sendCodeSerial(9010, "Overflow FIFO DMP", true);
				hatireStruct.frameNum = 0;
			}
			//datasheet says bit 0 is the ready flag. originally 0b10, changed to 0b01, but print shows = to 0b11 so both work.
			//can be changed to just 1 for clarity if needed.
			else if(mpuStatus & 0b01) { //check for DMP data ready interrupt (this should happen frequently):
				//wait for correct available data length, should be a VERY short wait as the FIFO gets the data
				while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
				mpu.getFIFOBytes(fifoBuffer, packetSize); //read a packet from FIFO
				//in case there is > 1 packet available, track FIFO count here:
				//(this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize;
	
				//get Euler angles in degrees:
				mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &quaternion);
				mpu.dmpGetYawPitchRoll(hatireStruct.gyro, &quaternion, &gravity);
	
				//TODO: add acceleration/head location that works with OpenTrack.
				/*uint16_t accelMeas[3];
				mpu.getAcceleration(&accelMeas[0], &accelMeas[1], &accelMeas[2]);
				hatireStruct.acc[0] = accelMeas[0]/16384 - calValuesStruct.acc_offset[0];
				hatireStruct.acc[1] = accelMeas[1]/16384 - calValuesStruct.acc_offset[1];
				hatireStruct.acc[2] = accelMeas[2]/16384 - calValuesStruct.acc_offset[2];*/
	
				//convert euler angles to +/- 180 degrees from gyro readings to offset-ed hatire data:
				for(int i=0; i<=2; i++) {
					hatireStruct.gyro[i] = (hatireStruct.gyro[i] - calValuesStruct.gyro_offset[i]) * RAD_2_DEG;
					if(hatireStruct.gyro[i] > 180) hatireStruct.gyro[i] -= 360; //example 190 -> -170
				}
			} //end of check if DMP data is ready
		} //end of check if DMP ready
		else {
			sendCodeSerial(9011, "Error DMP not loaded", true);
		}
	#endif //end of #ifdef GY521

	#ifdef LSM303
		if(active) {
			sensors_event_t	accel_event;
			sensors_event_t	mag_event;
			sensors_vec_t	orientation;
			
			//read the accelerometer and magnetometer:
			accel.getEvent(&accel_event);
			mag.getEvent(&mag_event);
	
			//fuse sensor readings:
			if(dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
				//put the updated data into the hatire packet:
				hatireStruct.gyro[0] = orientation.heading	- calValuesStruct.gyro_offset[0]; //yaw
				hatireStruct.gyro[1] = orientation.pitch	- calValuesStruct.gyro_offset[1]; //pitch
				hatireStruct.gyro[2] = orientation.roll		- calValuesStruct.gyro_offset[2]; //roll
			}
			else {
				sendCodeSerial(9011, "Error can't fuse sensors", true);
			}
		}
		else {
			sendCodeSerial(9011, "Error not active", true);
		}
	#endif

	#ifdef GY521
		if(dmpReady) {
	#endif
	#ifdef LSM303
		if(active) {
	#endif
	
		//once pendingCal has been raised, measure and average calFrameAmnt measurments.
		if(pendingCal) {
			if(calFrameCounter >= calFrameAmnt) {
				calFrameCounter = 0;
				calValuesStruct.gyro_offset[0] = calValuesStruct.gyro_offset[0] / calFrameAmnt;
				calValuesStruct.gyro_offset[1] = calValuesStruct.gyro_offset[1] / calFrameAmnt;
				calValuesStruct.gyro_offset[2] = calValuesStruct.gyro_offset[2] / calFrameAmnt;
				calValuesStruct.acc_offset[0] = calValuesStruct.acc_offset[0] / calFrameAmnt;
				calValuesStruct.acc_offset[1] = calValuesStruct.acc_offset[1] / calFrameAmnt;
				calValuesStruct.acc_offset[2] = calValuesStruct.acc_offset[2] / calFrameAmnt;
				pendingCal = false;
				calFrameCounter = 0;
				#ifdef USE_EEPROM
					saveToEEPROM();
				#endif
			}
			else {
				calValuesStruct.gyro_offset[0] += (float) hatireStruct.gyro[0];
				calValuesStruct.gyro_offset[1] += (float) hatireStruct.gyro[1];
				calValuesStruct.gyro_offset[2] += (float) hatireStruct.gyro[2];
				calValuesStruct.acc_offset[0] += (float) hatireStruct.acc[0];
				calValuesStruct.acc_offset[1] += (float) hatireStruct.acc[1];
				calValuesStruct.acc_offset[2] += (float) hatireStruct.acc[2];
	
				calFrameCounter++;
			}
		}
	
		//send frame to the PC via serial or BT
		//pass write() a pointer to the hatireStruct struct that we've been filling
		#ifndef BT_SERIAL
			Serial.write((byte*) &hatireStruct, 30);
		#else
			BTSerial.write((byte*) &hatireStruct, 30);
		#endif
		
		hatireStruct.frameNum++; //increment the frame counter and prevent rollover
		if(hatireStruct.frameNum > 999) {
			hatireStruct.frameNum = 0;
		}
	}
	
	delay(1);
} //end of loop()
