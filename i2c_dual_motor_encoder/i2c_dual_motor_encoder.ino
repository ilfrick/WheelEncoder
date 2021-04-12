#include <frickSoftwareSerial.h>
const byte rxPin = 14;
const byte txPin = 15;
SoftwareSerial nss(rxPin, txPin);

#include <Wire.h>

#define SLAVE_ADDRESS      		0x29 //slave address,any number from 0x01 to 0x7F
#define DEVICE_ID      			122

//#define MAX_SENT_BYTES     		3
//#define MODE_REG				0x09	// remove?
//#define CONFIG_REG				0x0A	//

//static uint8_t receivedCommands[MAX_SENT_BYTES];

//Incoming cmds
char rycmd;

// where we store the values
static uint8_t	mode_register;
static uint8_t	config_register;

// incoming data buffer
static uint8_t 	mode_data;
static uint8_t 	config_data;

// new data flag
static bool 	mode_available;
static bool 	config_available;
static float speed_1, speed_2;

// s    1       2       1s 	   2s	   m   c  id
// 0    1   2   3   4   5  6   7  8    9  10  11
// 8,   8   8   8   8   8  8   8  8    8,  8,  8
// 1    22      33      44     55      66  77  88 // proof
// 1,    0, 0,  6,  0,  0, 0,  90, 0,  66, 77, 122

struct reg_map {
  uint8_t status;			// I2C status
  int16_t wheel_1_delta;	// My Data
  int16_t wheel_2_delta;
  int16_t wheel_1_speed;  // My Data
  int16_t wheel_2_speed;
  uint8_t	mode;			// register values
  uint8_t	config;			// register values
  uint8_t	id;				// never changes
  int16_t rollcmd; //Roll command
  int16_t yawcmd;  //Yaw command
};

int16_t rcmdprev = 1500;
int16_t ycmdprev = 1500;

const int union_size = sizeof(reg_map);

// buffer
static union {
  reg_map map;
  uint8_t bytes[union_size];
} _buffer;



volatile int16_t wheel_1, wheel_2;

static uint32_t loopTimer;


void setup()
{
  Serial.begin(9600);
  Serial.println("Begin");
  //Serial.printf("buff: %d\n", sizeof(_buffer));

  // setup pins to read the encoder
  pin_setup();

  // setup the I2C slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  //  nss.handle_interrupt();
  nss.begin(9600);

  // for debugging union
  //test_values();

  // init our device ID
  _buffer.map.id 			= DEVICE_ID;

  //init commands
  _buffer.map.rollcmd = rcmdprev;
  _buffer.map.yawcmd = ycmdprev;
}

void loop()
{
  //  if (Serial.available() > 0) {
  //    getcmd();
  //  }
}

void changeModeConfig()
{
  if (mode_available) {
    mode_register 		= mode_data;
    mode_available		= false;		// always make sure to reset the flags before returning from the function
    mode_data 			= 0;
  }

  if (config_available) {
    config_register 	= config_data;
    config_available 	= false;		// always make sure to reset the flags before returning from the function
    config_data 		= 0;
  }
}

void getcmd()
{
  //  Serial.println("getcmd() called");
  //  nss.println("getcmd() called");
  if (nss.available() > 0) {
    rycmd = nss.read();
    Serial.println("I received: ");
    Serial.println(rycmd);

    switch (rycmd) {
      case '0':
        _buffer.map.rollcmd = 1500;
        _buffer.map.yawcmd = 1500;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case 'o':
        _buffer.map.rollcmd = 1500;
        _buffer.map.yawcmd = 1500;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case '1':
        _buffer.map.rollcmd = 1900;
        _buffer.map.yawcmd = 1500;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case '2':
        _buffer.map.rollcmd = 1100;
        _buffer.map.yawcmd = 1500;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case '3':
        _buffer.map.rollcmd = 1900;
        _buffer.map.yawcmd = 1100;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case '4':
        _buffer.map.rollcmd = 1900;
        _buffer.map.yawcmd = 1900;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case '5':
        _buffer.map.rollcmd = 1100;
        _buffer.map.yawcmd = 1100;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      case '6':
        _buffer.map.rollcmd = 1100;
        _buffer.map.yawcmd = 1900;
        rcmdprev = _buffer.map.rollcmd;
        ycmdprev = _buffer.map.yawcmd;
        break;
      default:
        _buffer.map.rollcmd = rcmdprev;
        _buffer.map.yawcmd = ycmdprev;
        break;
    }
    givefeedback();
  }
}

void givefeedback()
{
  Serial.println("Roll command received: ");
  Serial.println(_buffer.map.rollcmd);
  Serial.println("Yaw command received: ");
  Serial.println(_buffer.map.yawcmd);
  nss.println("Roll command received: ");
  nss.println(_buffer.map.rollcmd);
  nss.println("Yaw command received: ");
  nss.println(_buffer.map.yawcmd);
  nss.println("Wheel 1 speed: ");
  nss.println(_buffer.map.wheel_1_speed);
  nss.println("Wheel 2 speed: ");
  nss.println(_buffer.map.wheel_2_speed);
}

void requestEvent()
{
  uint32_t timer = micros();

  // copy over and clear wheel encoder values
  _buffer.map.wheel_1_delta = wheel_1;
  wheel_1 = 0;

  _buffer.map.wheel_2_delta = wheel_2;
  wheel_2 = 0;

  // find elapsed time since last read
  float dt  = (float)(timer - loopTimer) / 1000000.0;

  // prevent a big buildup, read at a least 10hz
  dt = min(dt, .15);

  loopTimer 	= timer;

  // Find velocity of rotation
  speed_1 += ((float)_buffer.map.wheel_1_delta / dt);
  speed_2 += ((float)_buffer.map.wheel_2_delta / dt);

  // small averaging filter
  speed_1 *= .5;
  speed_2 *= .5;

  _buffer.map.wheel_1_speed = speed_1;
  _buffer.map.wheel_2_speed = speed_2;

  //Serial.printf("%1.4f, %d, %d\n", dt, _buffer.map.wheel_1_speed, _buffer.map.wheel_2_speed);

  /*
  	debug values
    _buffer.map.wheel_1_delta = 55;
    _buffer.map.wheel_2_delta = 66;
    _buffer.map.wheel_1_speed = 99;
    _buffer.map.wheel_2_speed = 88;
  */

  //Serial.printf("%1.4f, %d, %d\n", //dt, _buffer.map.wheel_1_speed, _buffer.map.wheel_2_speed);
  //11, 0, -2, 0, -59, 66, 77, 122

  /*
    Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
  			_buffer.map.status,
  			_buffer.map.wheel_1_delta,
  			_buffer.map.wheel_2_delta,
  			_buffer.map.wheel_1_speed,
  			_buffer.map.wheel_2_speed,
  			_buffer.map.mode,
  			_buffer.map.config,
  			_buffer.map.id);
    //*/

  /*
    Serial.printf("%d : %d \t|\t %d: %d\n",
  		_buffer.map.wheel_1_delta,
  		_buffer.map.wheel_1_speed,
  		_buffer.map.wheel_2_delta,
  		_buffer.map.wheel_2_speed);
    //*/

  getcmd();

  //Set the buffer to send all bytes
  Wire.write(_buffer.bytes, sizeof(_buffer));
}

void receiveEvent(int bytesReceived)
{
  /*
  	for (int a = 0; a < bytesReceived; a++){
  		if(a < MAX_SENT_BYTES){
  			receivedCommands[a] = Wire.read();
  		}else{
  			Wire.read();	// if we receive more data then allowed just throw it away
  		}
  	}

  	switch(receivedCommands[0]){
  		case MODE_REG:
  			mode_available = true; // this variable is a status flag to let us know we have new data in register MODE_REG
  			mode_data = receivedCommands[1]; // save the data to a separate variable
  			break;

  		case CONFIG_REG:
  			config_available = true;
  			config_data = receivedCommands[1];
  			break;

  		default:
  			return; // ignore the commands and return
  	}
  */
}

void test_values()
{
  _buffer.map.status 			= 11;
  _buffer.map.wheel_1_delta 	= 22;
  _buffer.map.wheel_2_delta 	= 33;
  _buffer.map.wheel_1_speed 	= 44;
  _buffer.map.wheel_2_speed 	= 55;
  _buffer.map.mode 			= 66;
  _buffer.map.config 			= 77;
  _buffer.map.id 				= 88;
  ///*
  //	Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
  //				_buffer.map.status,
  //				_buffer.map.wheel_1_delta,
  //				_buffer.map.wheel_2_delta,
  //				_buffer.map.wheel_1_speed,
  //				_buffer.map.wheel_2_speed,
  //				_buffer.map.mode,
  //				_buffer.map.config,
  //				_buffer.map.id);
  //*/
  //	for (uint8_t i = 0; i < sizeof(_buffer); i++){
  //		Serial.printf("%d, ", _buffer.bytes[i]);
  //	}

  Serial.println();
}



void pin_setup()
{
  // ATMEGA ADC
  // PORTC - PCMSK1
  // PC0 - ADC0 		– PCINT8
  // PC1 - ADC1 		– PCINT9
  // PC2 - ADC2 		– PCINT10
  // PC3 - ADC3 		– PCINT11
  // PC4 - ADC4 		– PCINT12	// SDA
  // PC5 - ADC5 		– PCINT13	// SLC

  // ATMEGA
  // PORTD - PCMSK2
  // p0				// PD0 - RX D  		- PCINT16
  // p1				// PD1 - TX D  		- PCINT17
  pinMode(2, INPUT);	// PD2 - INT0 		- PCINT18
  pinMode(3, INPUT);	// PD3 - INT1 		- PCINT19
  pinMode(4, INPUT);	// PD4 - XCK / T0 	- PCINT20
  pinMode(5, INPUT);	// PD5 - T0			- PCINT21
  pinMode(6, INPUT);	// PD6 - T1			- PCINT22
  pinMode(7, INPUT);	// PD7 - AIN0		- PCINT23

  // PORTB- PCMSK0
  pinMode(8, INPUT); 	// PB0 - AIN1		- PCINT0
  pinMode(9, INPUT);	// PB1 - OC1A		- PCINT1
  pinMode(10, INPUT);	// PB2 - OC1B		- PCINT2
  pinMode(11, INPUT); 	// PB3 - MOSI / OC2	- PCINT3
  pinMode(12, INPUT); 	// PB4 - MISO		- PCINT4
  pinMode(13, INPUT); 	// PB5 - SCK		- PCINT5
  //					// PB6 -    		- PCINT6
  //					// PB7 -    		- PCINT7


  // PCINT0_vect = interrupt vector for external interrupt on pin PCINT  0..7
  // PCINT1_vect = interrupt vector for external interrupt on pin PCINT  8..13
  // PCINT2_vect = interrupt vector for external interrupt on pin PCINT 16..23

  PCMSK2 	= _BV(PCINT18);	// | _BV(PCINT19);
  PCMSK0 	= _BV(PCINT0);	// | _BV(PCINT19);

  PCICR	= _BV(PCIE2) | _BV(PCIE0);
}

ISR(PCINT2_vect) {

  if ((PIND & B00000100) == 0 && (PIND & B00001000) == 0) {
    wheel_1++;
  } else if ((PIND & B00000100) > 1 && (PIND & B00001000) > 1) {
    wheel_1++;
  } else {
    wheel_1--;
  }


}

ISR(PCINT0_vect) {

  if ((PINB & B00000001) == 0 && (PINB & B00000010) == 0) {
    wheel_2++;
  } else if ((PINB & B00000001) == 1 && (PINB & B00000010) > 1) {
    wheel_2++;
  } else {
    wheel_2--;
  }

}
