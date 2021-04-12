#include <Arduino_FreeRTOS.h>
#include "math.h"
#include <semphr.h>
#include <timers.h>
#include <task.h>

#define TIME_MS                       ( xTaskGetTickCount() * portTICK_PERIOD_MS)

#define MAX_SPEED                      5.0
#define PWM_MAX_SPEED                  255

#define MAX_ROLLING_AVERAGE_PERCENT    88
#define MAX_ROLLING_AVERAGE            (MAX_SPEED * MAX_ROLLING_AVERAGE_PERCENT / 100)
#define NUMBER_MEASUREMENTS_AVERAGE    6

#define DECREASE_RATE   20
#define INCREASE_RATE   20

#define SPEED_INCREASE  MAX_SPEED * INCREASE_RATE / 100
#define SPEED_DECREASE  MAX_SPEED * DECREASE_RATE / 100

#define MONITOR_FREQ        2
#define FLASH_FREQ          4 
#define FLASH_DURATION      3
#define NUM_FLASHES         (FLASH_DURATION * FLASH_FREQ)

#define DISTANCE_BETWEEN_WHEEL_CENTRES  2
#define ODOMETRY_UPDATE_TIME            1

#define TO_PWM(x)       map((x), -MAX_SPEED, MAX_SPEED, -PWM_MAX_SPEED, PWM_MAX_SPEED)
#define TO_RPM(x)       map((x), -PWM_MAX_SPEED, PWM_MAX_SPEED, -MAX_SPEED, MAX_SPEED)

#define DEBOUNCE_DELAY   500

#define DIAGNOSTIC_PERIOD         2000UL
#define SENSOR_PROCESSING_PERIOD  20UL
#define CPU_BUSY_PERIOD           5UL

// buttons on Arduino 
#define INCREASE_LEFT_BUTTON    10
#define DECREASE_LEFT_BUTTON    16
#define INCREASE_RIGHT_BUTTON   5
#define DECREASE_RIGHT_BUTTON   7
#define BRAKE_BUTTON            2  // can only be 2 or 3
#define DIAGNOSTIC_MODE_BUTTON  3  // can only be 2 or 3

// I/O pins on Arduino 
#define PWM_LEFT_OUTPUT    6
#define DIR_LEFT_OUTPUT    12
#define PWM_RIGHT_OUTPUT   11
#define DIR_RIGHT_OUTPUT   13
#define BRAKE_LEFT         8
#define BRAKE_RIGHT        9 
#define SENSE_LEFT         14
#define SENSE_RIGHT        15
#define MONITOR_LED        13

unsigned stack_hwm = 0;

// left and right speed
float left = 0;
float right = 0;

// arrays storing the left and right DC motor speeds 
int measurementsLeft[NUMBER_MEASUREMENTS_AVERAGE];
int measurementsRight[NUMBER_MEASUREMENTS_AVERAGE];

// strings for logging data 
const char *outputs[5]; 

typedef struct Pose{
  float x;
  float y;
  float theta;
  float linearVel;
  float rotVel;  
}Pose;
Pose pose;

// global variable for state of breaks (engaged or not) and diagnostic 
int brakeState = 0;
int diagnosticState = 0;

// processes 
static void TaskUpdateMotorSpeeds( void *pvParameters );
static void TaskDiagnostic( void *pvParameters );
static void TaskProcessing ( void *pvParameters);
static void TaskMonitor (void *pvParameters );
static void TaskDeadReckoning (void *pvParameters );

// interrupt handler function
void breakInterruptHandler();
void diagnosticInterruptHandler();
volatile unsigned long lastDebounceDiagnostic = 0;

void setup() {
  outputs[0] = "v: "; outputs[1] = "omega: "; outputs[2] = "x: ";
  outputs[3] = "y: "; outputs[4] = "theta: ";                          
    
  // Set input buttons
   pinMode(INCREASE_LEFT_BUTTON, INPUT);
   pinMode(DECREASE_LEFT_BUTTON, INPUT);
   pinMode(INCREASE_RIGHT_BUTTON, INPUT);
   pinMode(DECREASE_RIGHT_BUTTON, INPUT);
   
   // Set outputs 
   pinMode(PWM_LEFT_OUTPUT, OUTPUT);
   pinMode(DIR_LEFT_OUTPUT, OUTPUT);
   pinMode(PWM_RIGHT_OUTPUT, OUTPUT);
   pinMode(DIR_RIGHT_OUTPUT, OUTPUT);
   pinMode(MONITOR_LED, OUTPUT);
  
  // initialize serial communications at 9600 bps
  Serial.begin(9600); 

  // attach interrupt handler to break button and diagnostic button
  attachInterrupt(digitalPinToInterrupt(BRAKE_BUTTON), breakInterruptHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(DIAGNOSTIC_MODE_BUTTON), diagnosticInterruptHandler, LOW);

  // task for updating motor speeeds 
  xTaskCreate(
    TaskUpdateMotorSpeeds
    ,  "Updated Left Motor Speed"   // A name
    ,  100  // Stack size 78
    ,  NULL
    ,  1  // priority
    ,  NULL );

  // task for diagnostic
    xTaskCreate(
    TaskDiagnostic
    ,  "Diagnostics"   // A name 
    ,  82  // Stack size 82
    ,  NULL
    ,  1  // priority
    ,  NULL );

  // task for velocity monitoring
   xTaskCreate(
    TaskDeadReckoning
    ,  "Dead Reckoning"   // A name 
    ,  150  // Stack size 142
    ,  NULL
    ,  1  // priority
    ,  NULL );  

  // task for sensor processing 
  /*xTaskCreate(
    TaskProcessing
    ,  "Sensor Processing"   // A name 
    ,  256  // Stack size
    ,  NULL
    ,  2  // priority
    ,  NULL );*/

  // task for velocity monitoring 
  xTaskCreate(
   TaskMonitor
    ,  "Velocity Monitoring"   // A name 
    ,  130  // Stack size 113
    ,  NULL
    ,  1  // priority
    ,  NULL );

  // start timer     
  //xSensorProcessingTimerStarted = xTimerStart (xSensorProcessingTimer, 0 );
  
  //if(xSensorProcessingTimerStarted == pdPASS)
  //{
    // start scheduler so created task start executing 
    vTaskStartScheduler();
  //}
//}
}

void loop() {  
}

void TaskUpdateMotorSpeeds( void *pvParameters) {
  (void) pvParameters;

  // last time buttons were triggered
  long lastDebounceDecreaseLeft = 0;
  long lastDebounceIncreaseLeft = 0;
  long lastDebounceDecreaseRight = 0;
  long lastDebounceIncreaseRight = 0;
  
  for (;;)
  {  
    if (brakeState == 0) {

      // check button inputs 
      int increaseLeft = digitalRead(INCREASE_LEFT_BUTTON);
      int decreaseLeft = digitalRead(DECREASE_LEFT_BUTTON);
      int increaseRight = digitalRead(INCREASE_RIGHT_BUTTON);
      int decreaseRight = digitalRead(DECREASE_RIGHT_BUTTON);
      
      // if last debounce time is more than a threshold, enter loop
      if ( ((TIME_MS - lastDebounceIncreaseLeft) > DEBOUNCE_DELAY) && (increaseLeft == LOW) ) {
        // if button is pressed and break is not activated increase speed
        left += SPEED_INCREASE;   
        if (left > MAX_SPEED) 
          left = MAX_SPEED;
        
        lastDebounceIncreaseLeft = TIME_MS;
      }
      
      // if last debounce time is more than a threshold, enter loop        
      if ( (millis() - lastDebounceDecreaseLeft > DEBOUNCE_DELAY) && (decreaseLeft == LOW) ) {
        // if button is pressed and break is not activated decrease speed
        left -= SPEED_DECREASE;
        if (left < -MAX_SPEED) 
          left = -MAX_SPEED;
        
        lastDebounceDecreaseLeft = TIME_MS;
      }
      
      // if last debounce time is more than a threshold, enter loop
      if ( ((TIME_MS - lastDebounceIncreaseRight) > DEBOUNCE_DELAY) && (increaseRight == LOW) ) {
        
        // if button is pressed and break is not activated increase speed
        right += SPEED_INCREASE;      
        if (right > MAX_SPEED) 
          right = MAX_SPEED;
        
        lastDebounceIncreaseRight = TIME_MS;
      }
      
      // if last debounce time is more than a threshold, enter loop
      if ( ((TIME_MS - lastDebounceDecreaseRight) > DEBOUNCE_DELAY) && (decreaseRight == LOW) ) {
        
        // if button is pressed and break is not activated decrease speed
        right -= SPEED_DECREASE;      
        if (right < -MAX_SPEED) 
          right = -MAX_SPEED;
        
        lastDebounceDecreaseRight = TIME_MS;
      }
      
      if ((increaseLeft == LOW) || (decreaseLeft == LOW) || (increaseRight == LOW) || (decreaseRight == LOW))
        logOdometry(); 
/*
      unsigned temp;
   temp = uxTaskGetStackHighWaterMark(NULL);
    
    if (!stack_hwm || temp < stack_hwm) {
        stack_hwm = temp;
        Serial.print(F(", High Watermark from function1: "));
        Serial.println(stack_hwm); // https://www.freertos.org/uxTaskGetStackHighWaterMark.html
    }*/
    
    changeMotorSpeeds(left, right, 0);
    }
    // delay task 
    taskYIELD();
  } 
}

static void TaskDeadReckoning( void * pvParameters ) {
  (void) pvParameters;

  long lastUpdateTime = millis();
  long lastPrintTime = millis();

  long dt = 0;
  float R = 0; float omega = 0;
  float ICCx = 0; float ICCy = 0;

  for (;;) {

    // print odometry every second 
    if ( (millis() - lastPrintTime) > ODOMETRY_UPDATE_TIME * 1000 ) {
      logOdometry();
      lastPrintTime = millis();
/*
      unsigned temp;
      temp = uxTaskGetStackHighWaterMark(NULL);
    
      if (!stack_hwm || temp < stack_hwm) {
        stack_hwm = temp;
        Serial.print(F(", High Watermark from function1: "));
        Serial.println(stack_hwm); // https://www.freertos.org/uxTaskGetStackHighWaterMark.html
      }*/
    }
    
    // no rotation, movement in straight line -> limiting case, else R -> infinity 
    dt = (millis() - lastUpdateTime) / 1000;
    if (left == right) {      
      lastUpdateTime = millis();
      
      pose.x += right * cos(pose.theta) * dt;
      pose.y += right * sin(pose.theta) * dt;

      pose.linearVel = right;
      pose.rotVel = 0.0;
    }
    else {           
      lastUpdateTime = millis();
      R = (DISTANCE_BETWEEN_WHEEL_CENTRES / 2) * ( (left + right) / (right - left) );
      omega = (right - left) / DISTANCE_BETWEEN_WHEEL_CENTRES;
  
      ICCx = pose.x - R * sin(pose.theta);
      ICCy = pose.y + R * cos(pose.theta);
        
      pose.x = ( cos(omega * dt) * (pose.x - ICCx) ) + ( -sin(omega * dt) * (pose.y - ICCy) ) + ICCx; 
      pose.y = ( sin(omega * dt) * (pose.x - ICCx) ) + ( cos(omega * dt) * (pose.y - ICCy) ) + ICCy;

      pose.theta += omega * dt;    

      pose.linearVel = (right + left) / 2;
      pose.rotVel = omega;
   }
    // delay task 
    taskYIELD();
  }
}

static void TaskDiagnostic( void *pvParameters ) {
    (void) pvParameters;

    uint16_t currentLeft = 0; uint16_t currentRight = 0;
    uint16_t absoluteLeft = 0; uint16_t absoluteRight = 0;   

    long start = millis();
    
    for (;;) {
      if (diagnosticState == 1) {

        if ( (millis() - start) < DIAGNOSTIC_PERIOD) {

          // check currents in DC motors
          int senseLeft = analogRead(SENSE_LEFT);
          int senseRight = analogRead(SENSE_RIGHT);
        
          // update accordingly 
          if (senseLeft > currentLeft )
            currentLeft = senseLeft;
        
          if (senseRight > currentRight )
            currentRight = senseRight;
  
        } else {
        
          // update maximums         
          if (currentLeft > absoluteLeft) 
            absoluteLeft = currentLeft;
          
          if (currentRight > absoluteRight) 
            absoluteRight = currentRight;      

          // print maxed sensed currents
          Serial.print(F("curr left: "));
          Serial.println(currentLeft);
          Serial.print(F("curr right: ")); 
          Serial.println(currentRight);
          Serial.print(F("abs left: ")); 
          Serial.println(absoluteLeft);
          Serial.print(F("abs right: ")); 
          Serial.println(absoluteRight);

          // reset current values 
          currentRight = 0; currentLeft = 0; 
          
          start = millis();
/*
      unsigned temp;
      temp = uxTaskGetStackHighWaterMark(NULL);
    
      if (!stack_hwm || temp < stack_hwm) {
        stack_hwm = temp;
        Serial.print(F(", High Watermark from function1: "));
        Serial.println(stack_hwm); // https://www.freertos.org/uxTaskGetStackHighWaterMark.html
      }*/
      }
    } 
       
    // delay task 
    taskYIELD();
  }
}

static void TaskProcessing ( void *pvParameters) {
  (void) pvParameters;

  //const TickType_t xDelay = pdMS_TO_TICKS( SENSOR_PROCESSING_PERIOD - CPU_BUSY_PERIOD);
  const TickType_t xDelay = pdMS_TO_TICKS( 30UL );

  for (;;) {
    long initialTime = millis();
    while ((millis() - initialTime) < CPU_BUSY_PERIOD) {
      ;
    }
    Serial.println("check");
    vTaskDelay(xDelay);
  }  
}

static void TaskMonitor( void *pvParameters) {
  (void) pvParameters;

  // initialize sums, pointer, averages and arrays
  float sumLeft = 0;
  float sumRight = 0;
  uint8_t curr = 0;
  
  
    // initialize 
  for (int i = 0; i < NUMBER_MEASUREMENTS_AVERAGE; i++) {
    measurementsLeft[i] = 0;
    measurementsRight[i] = 0;
  }
  
  long lastAverageUpdate = millis();
  long lastFlash = millis();
  int flashCount = 0;
  
  for (;;) {

    if ( ((millis() - lastAverageUpdate) > ((1 / MONITOR_FREQ) * 1000)) && diagnosticState == 0) {
      
      // remove pointer element from sum
      sumLeft -= TO_RPM(measurementsLeft[curr]);
      sumRight -= TO_RPM(measurementsRight[curr]);
  
      // add measurements 
      measurementsLeft[curr] = TO_PWM(left);
      measurementsRight[curr] = TO_PWM(right);
      
      // rolling sum 
      sumLeft += left;
      sumRight += right;
      
      // increase counter
      curr++; 
      if (curr >= NUMBER_MEASUREMENTS_AVERAGE) {
        curr = 0;
      }
      // check if rolling average is higher than maximum
      if ((sumLeft / NUMBER_MEASUREMENTS_AVERAGE) > MAX_ROLLING_AVERAGE && ((sumRight / NUMBER_MEASUREMENTS_AVERAGE) > MAX_ROLLING_AVERAGE)) {
          
        if ( ((millis() - lastFlash) > ((1 / FLASH_FREQ) * 1000)) && (flashCount < NUM_FLASHES)) {
           // blink LED
          digitalWrite(MONITOR_LED, HIGH);
          digitalWrite(MONITOR_LED, LOW);
    
          // increase counter 
          flashCount++;
          if (flashCount == NUM_FLASHES)
            flashCount = 0;
    
          // update time of last blink 
          lastFlash = millis();
    
          unsigned temp;
         temp = uxTaskGetStackHighWaterMark(NULL);
         
         if (!stack_hwm || temp < stack_hwm) {
            stack_hwm = temp;
            Serial.print(", High Watermark from function1: ");
            Serial.println(stack_hwm); // https://www.freertos.org/uxTaskGetStackHighWaterMark.html
         }
       }
     } else {
      flashCount = 0;
     }      
      lastAverageUpdate = millis();   
    }
    taskYIELD();
  }
}

void breakInterruptHandler() { 

  if (brakeState == 0) {

    // update state 
    left = 0;
    right = 0;
    changeMotorSpeeds(left, right, 1);
    
    brakeState = 1;

    // print message on terminal 
    Serial.println("BRAKES ON");
    
  // disengage breaks 
  } else if (brakeState == 1) {
    
    // update state 
    brakeState = 0;
    
    // print message on terminal 
    Serial.println("BRAKES OFF");  
  }
}

void diagnosticInterruptHandler() {
  // if last debounce time is more than a threshold, modify diagnostic state 
  if ( (millis() - lastDebounceDiagnostic) > DEBOUNCE_DELAY ) {
    diagnosticState = !diagnosticState;
    
    lastDebounceDiagnostic = millis();     
  }
}

void logOdometry()
{
  Serial.print(outputs[0]);
  Serial.println(pose.linearVel);
  Serial.print(outputs[1]);
  Serial.println(pose.rotVel);

  Serial.print(outputs[2]);
  Serial.println(pose.x);
  Serial.print(outputs[3]);
  Serial.println(pose.y);
  Serial.print(outputs[4]);
  Serial.println(pose.theta);
}

void changeMotorSpeeds(float left, float right, int brake)
{   
  // change right motor
  analogWrite(PWM_RIGHT_OUTPUT, abs(TO_PWM(right))); // speed
  digitalWrite(DIR_RIGHT_OUTPUT, (right > 0) ? HIGH : LOW); // direction
  digitalWrite(BRAKE_RIGHT, brake); // don't brake 
  
  // change left motor
  analogWrite(PWM_LEFT_OUTPUT, abs(TO_PWM(left))); // speed
  digitalWrite(DIR_LEFT_OUTPUT, (left > 0) ? HIGH : LOW); // direction
  digitalWrite(BRAKE_LEFT, brake); // don't brake

}
