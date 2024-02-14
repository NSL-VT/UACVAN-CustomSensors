/**
 * In trying to modify this code, most of the modifications that need to be made should be made in the MX_falling code. This code is where the falling edge of th e
 * PWM signal is captured and is where the necessary messages get published. 
*/
#include "uavcan.h"

/* Define the pin numbers */
#define M1_PIN 1 
#define M2_PIN 2
#define M3_PIN 3
#define M4_PIN 4
#define M5_PIN 5
#define M6_PIN 6

/* Interrupt functions and variables */
void M1_rising(); void M2_rising(); void M3_rising();
void M4_rising(); void M5_rising(); void M6_rising();
void M1_falling(); void M2_falling(); void M3_falling();
void M4_falling(); void M5_falling(); void M6_falling();

int counter = 0;




/* Different motor PWM signals */
volatile uint32_t M1_PWM = 0, M2_PWM = 0, M3_PWM = 0, 
M4_PWM = 0, M5_PWM = 0, M6_PWM = 0;
/* Different motor PWM times for debugging */
volatile uint32_t M1_t0 = 0, M2_t0 = 0, M3_t0 = 0,
M4_t0 = 0, M5_t0 = 0, M6_t0 = 0, M1_t1 = 0, M2_t1 = 0, 
M3_t1 = 0, M4_t1 = 0, M5_t1 = 0, M6_t1 = 0;

/* Different motor PWM times */
uint32_t M1_prev_time, M2_prev_time, M3_prev_time, 
M4_prev_time, M5_prev_time, M6_prev_time ;

/* Node constants */
static constexpr uint32_t NODE_ID = 2;
static constexpr uint8_t SW_VER = 1;
static constexpr uint8_t HW_VER = 1;
static const char* NODE_NAME = "TEST";
static const uint32_t NODE_MEM = 8192;  // size of node memory
uavcan::CanDriver<1> *can;
uavcan::Node<NODE_MEM> *node;
uavcan::CanIface<CAN1> *can1;
uavcan::Publisher<uavcan::equipment::esc::Status> *pub;

/* Data message */
uavcan::equipment::esc::Status msg;

/* Fake airspeed data */
float airspeed_ms = 0;

int NUM_POLES = 12;



/**
 * @section Interrupt functions for alpha and beta vanes
 * @link https://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
 */
void M1_rising() 
{
	attachInterrupt(M1_PIN, M1_falling, FALLING);
	M1_prev_time = micros();
}
void M2_rising() 
{
	attachInterrupt(M2_PIN, M2_falling, FALLING);
	M2_prev_time = micros();
}
void M3_rising() 
{
	attachInterrupt(M3_PIN, M3_falling, FALLING);
	M3_prev_time = micros();
}
void M4_rising() 
{
	attachInterrupt(M4_PIN, M4_falling, FALLING);
	M4_prev_time = micros();
}
void M5_rising() 
{
	attachInterrupt(M5_PIN, M5_falling, FALLING);
	M5_prev_time = micros();
}
void M6_rising() 
{
	attachInterrupt(M6_PIN, M6_falling, FALLING);
	M6_prev_time = micros();
}
void M1_falling() 
{
	attachInterrupt(M1_PIN, M1_rising, RISING);
	M1_PWM = micros()-M1_prev_time;


  msg.rpm = M1_PWM;
  msg.esc_index = 0;

  if (pub->broadcast(msg) < 0)
  {
    Serial.println("WARNING issue publishing rpm message");
  }else{

    Serial.print("P,");
    Serial.print(M1_PWM);
    Serial.print(",");
    Serial.println(micros());


    
  }

}
void M2_falling() 
{
	attachInterrupt(M2_PIN, M2_rising, RISING);
	M2_PWM = micros()-M2_prev_time;

  msg.rpm = M2_PWM;
  msg.esc_index = 1;

  if (pub->broadcast(msg) < 0)
  {
    Serial.println("WARNING issue publishing rpm message");
  }
    Serial.print("Y,");
    Serial.print(M1_PWM);
    Serial.print(",");
    Serial.println(micros());

    

}
void M3_falling() 
{
	attachInterrupt(M3_PIN, M3_rising, RISING);
	M3_PWM = micros()-M3_prev_time;

  msg.rpm = M3_PWM;
  msg.esc_index = 2;

  if (pub->broadcast(msg) < 0)
  {
    Serial.println("WARNING issue publishing rpm message");
  }
    Serial.print("W,");
    Serial.print(M1_PWM);
    Serial.print(",");
    Serial.println(micros());
}
void M4_falling() 
{
	attachInterrupt(M4_PIN, M4_rising, RISING);
	M4_PWM = micros()-M4_prev_time;

  msg.rpm = M4_PWM;
  msg.esc_index = 3;

  if (pub->broadcast(msg) < 0)
  {
    Serial.println("WARNING issue publishing rpm message");
  }
}
void M5_falling() 
{
	attachInterrupt(M5_PIN, M5_rising, RISING);
	M5_PWM = micros()-M5_prev_time;

    msg.rpm = M5_PWM;
  msg.esc_index = 4;

  if (pub->broadcast(msg) < 0)
  {
    Serial.println("WARNING issue publishing rpm message");
  }
}
void M6_falling() 
{
	attachInterrupt(M6_PIN, M6_rising, RISING);
	M6_PWM = micros()-M6_prev_time;

    msg.rpm = M6_PWM;
  msg.esc_index = 5;

  if (pub->broadcast(msg) < 0)
  {
    Serial.println("WARNING issue publishing rpm message");
  }
}




void setup() {
  Serial.begin(115200);
  /* Init CAN interface */
  can1 = new uavcan::CanIface<CAN1>;
  can1->begin();
  can1->setBaudRate(1000000);
  /* Init CAN driver */
  can = new uavcan::CanDriver<1>({can1});
  /* Init Node */
  node = new uavcan::Node<NODE_MEM>(*can, uavcan::clock);
  uavcan::protocol::SoftwareVersion sw_ver;
  uavcan::protocol::HardwareVersion hw_ver;
  sw_ver.major = SW_VER;
  sw_ver.minor = 0;
  hw_ver.major = HW_VER;
  hw_ver.minor = 0;

  node->setNodeID(NODE_ID);
  node->setName(NODE_NAME);
  node->setSoftwareVersion(sw_ver);
  node->setHardwareVersion(hw_ver);
  if (node->start() < 0) {
    Serial.println("ERROR starting node");
    while (1) {}
  }
  Serial.println("Node initialized");

  /* Init publisher */
  pub = new uavcan::Publisher<uavcan::equipment::esc::Status>(*node);
  if (pub->init() < 0) {
    Serial.println("ERROR initializing publisher");
    while (1) {}
  }
  Serial.println("Publisher initialized");
  /* CAN acceptance filters */
  uavcan::configureCanAcceptanceFilters(*node);
  
    	/* Set the PWM interrupt pins */
	pinMode(M1_PIN, INPUT); pinMode(M2_PIN, INPUT); pinMode(M3_PIN, INPUT); 
	pinMode(M4_PIN, INPUT); pinMode(M5_PIN, INPUT); pinMode(M6_PIN, INPUT);


    /* When pin goes high, call the rising function */
	attachInterrupt(M1_PIN, M1_rising, RISING); attachInterrupt(M2_PIN, M2_rising, RISING);
	attachInterrupt(M3_PIN, M3_rising, RISING); attachInterrupt(M4_PIN, M4_rising, RISING);
	attachInterrupt(M5_PIN, M5_rising, RISING); attachInterrupt(M6_PIN, M6_rising, RISING); 

  
  /* Set Node mode to operational */
  node->setModeOperational();
  Serial.println("Setup complete");
  digitalWrite(LED_BUILTIN, 1);
  Serial.println(micros());


  
}

void loop() {
  delay(2);
  
}




