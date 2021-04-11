#include <Arduino_FreeRTOS.h>
#include "math.h"
#include <semphr.h>
#include <timers.h>
#include <task.h>

#define MAX_SPEED                      5.0
#define PWM_MAX_SPEED                  255

#define DECREASE_RATE   20
#define INCREASE_RATE   20

#define SPEED_INCREASE  MAX_SPEED * INCREASE_RATE / 100
#define SPEED_DECREASE  MAX_SPEED * DECREASE_RATE / 100

#define DISTANCE_BETWEEN_WHEEL_CENTRES  2
#define ODOMETRY_UPDATE_TIME            1
#define TO_DEGREES(x)                   ( (x) * M_PI / 180.0 )

#define TO_PWM(x)       map((x), -MAX_SPEED, MAX_SPEED, -PWM_MAX_SPEED, PWM_MAX_SPEED)

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

//unsigned stack_hwm = 0;

// enum for the state of the motors
enum state_t { DECREASING, INCREASING };

// typedef for speed and state of motors 
typedef struct MotorSpeed{
  uint8_t left;
  uint8_t right;
  state_t leftDir;
  state_t rightDir;
}MotorSpeed;

// left and right speed
float left = 0;
float right = 0;

typedef struct Diagnostic{
  uint16_t currentLeft;
  uint16_t currentRight;
  uint16_t absoluteLeft;
  uint16_t absoluteRight;
}Diagnostic;
Diagnostic dgn;

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

// semaphores
SemaphoreHandle_t xBreakSemaphore;
SemaphoreHandle_t xResourceSemaphore;

// task handle
TaskHandle_t xTaskBreakHandle;

// timers
static TimerHandle_t xDiagnosticTimer;
//static TimerHandle_t xSensorProcessingTimer;

// processes 
static void TaskUpdateMotorSpeeds( void *pvParameters );
static void TaskBrake( void *pvParameters );
static void TaskDiagnostic( void *pvParameters );
static void TaskProcessing ( void *pvParameters);
static void TaskMonitor (void *pvParameters );
static void TaskDeadReckoning (void *pvParameters );

// interrupt handler function
void breakInterruptHandler();
void diagnosticInterruptHandler();
volatile unsigned long lastDebounceDiagnostic = 0;

void setup() {
  BaseType_t xSensorProcessingTimerStarted;
  
  // create semaphore and mutex
  xBreakSemaphore = xSemaphoreCreateBinary();
  xResourceSemaphore = xSemaphoreCreateBinary();

  // timers
  const TickType_t xDiagnosticPeriod = pdMS_TO_TICKS( DIAGNOSTIC_PERIOD );
  xDiagnosticTimer = xTimerCreate( "Diagnostic Timer",        // name
                                  xDiagnosticPeriod,          // period in ticks 
                                  pdFALSE,                    // false for one-shot timer 
                                  0,                          // The timer ID is initialised to 0
                                  diagnosticTimerCallback );         // callback
                                  
  /*const TickType_t xSensorProcessingPeriod = pdMS_TO_TICKS( DIAGNOSTIC_PERIOD );
  xSensorProcessingTimer = xTimerCreate( "Sensor Processing Timer",        // name
                                  xSensorProcessingPeriod,          // period in ticks 
                                  pdTRUE,                    // false for one-shot timer 
                                  0,                          // The timer ID is initialised to 0
                                  sensorProcessingTimerCallback );         // callback        */                      

  // if semaphore and mutex created successfully 
  if ( xBreakSemaphore != NULL && xResourceSemaphore != NULL && xDiagnosticTimer != NULL ) {
    
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
    
    // initialize serial communications at 9600 bps
    Serial.begin(9600); 

    // attach interrupt handler to break button and diagnostic button
    attachInterrupt(digitalPinToInterrupt(BRAKE_BUTTON), breakInterruptHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIAGNOSTIC_MODE_BUTTON), diagnosticInterruptHandler, LOW);

    // task for updating motor speeeds 
    xTaskCreate(
      TaskUpdateMotorSpeeds
      ,  "Updated Left Motor Speed"   // A name
      ,  79  // Stack size 79
      ,  NULL
      ,  1  // priority
      ,  NULL );

    // task for breaking
    xTaskCreate(
      TaskBrake
      ,  "Brake Vehicle"   // A name 
      ,  71  // Stack size
      ,  NULL
      ,  1  // priority
      ,  &xTaskBreakHandle );
     
    // task for diagnostic
    /*xTaskCreate(
      TaskDiagnostic
      ,  "Diagnostics"   // A name 
      ,  100  // Stack size
      ,  NULL
      ,  1  // priority
      ,  NULL );

    // task for velocity monitoring
    xTaskCreate(
      TaskDeadReckoning
      ,  "Dead Reckoning"   // A name 
      ,  155  // Stack size
      ,  NULL
      ,  1  // priority
      ,  NULL );  */

    // task for sensor processing 
    xTaskCreate(
      TaskProcessing
      ,  "Sensor Processing"   // A name 
      ,  256  // Stack size
      ,  NULL
      ,  2  // priority
      ,  NULL );

    // task for velocity monitoring 
    /*xTaskCreate(
      TaskProcessing
      ,  "Velocity Monitoring"   // A name 
      ,  128  // Stack size
      ,  NULL
      ,  1  // priority
      ,  NULL );*/

    // start timer     
    //xSensorProcessingTimerStarted = xTimerStart (xSensorProcessingTimer, 0 );
    
    //if(xSensorProcessingTimerStarted == pdPASS)
    //{
      // start scheduler so created task start executing 
      vTaskStartScheduler();
    //}
  }
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

  const TickType_t xDelay = pdMS_TO_TICKS( 50UL );
  
  for (;;)
  {
    // mutex for resource sharing with brake task 
    if (xSemaphoreTake( xResourceSemaphore, (TickType_t) 1 ) == pdPASS) {
    
      if (brakeState == 0) {
  
        // check button inputs 
        int increaseLeft = digitalRead(INCREASE_LEFT_BUTTON);
        int decreaseLeft = digitalRead(DECREASE_LEFT_BUTTON);
        int increaseRight = digitalRead(INCREASE_RIGHT_BUTTON);
        int decreaseRight = digitalRead(DECREASE_RIGHT_BUTTON);
        
        // if last debounce time is more than a threshold, enter loop
        if ( ((millis() - lastDebounceIncreaseLeft) > DEBOUNCE_DELAY) && (increaseLeft == LOW) ) {
          // if button is pressed and break is not activated increase speed
          left += SPEED_INCREASE;   
          if (left > MAX_SPEED) 
            left = MAX_SPEED;
          
          lastDebounceIncreaseLeft = millis();
        }
        
        // if last debounce time is more than a threshold, enter loop
        
        if ( (millis() - lastDebounceDecreaseLeft > DEBOUNCE_DELAY) && (decreaseLeft == LOW) ) {
          // if button is pressed and break is not activated decrease speed
          left -= SPEED_DECREASE;
          if (left < -MAX_SPEED) 
            left = -MAX_SPEED;
          
          lastDebounceDecreaseLeft = millis();
        }
        
        // if last debounce time is more than a threshold, enter loop
        if ( ((millis() - lastDebounceIncreaseRight) > DEBOUNCE_DELAY) && (increaseRight == LOW) ) {
          
          // if button is pressed and break is not activated increase speed
          right += SPEED_INCREASE;      
          if (right > MAX_SPEED) 
            right = MAX_SPEED;
          
          lastDebounceIncreaseRight = millis();
        }
        
        // if last debounce time is more than a threshold, enter loop
        if ( ((millis() - lastDebounceDecreaseRight) > DEBOUNCE_DELAY) && (decreaseRight == LOW) ) {
          
          // if button is pressed and break is not activated decrease speed
          right -= SPEED_DECREASE;      
          if (right < -MAX_SPEED) 
            right = -MAX_SPEED;
          
          lastDebounceDecreaseRight = millis();
        }
      }
    }  
    // set final speed and direction 
    MotorSpeed speed_t;
    speed_t.left = abs(TO_PWM(left));
    speed_t.right = abs(TO_PWM(right));   
    speed_t.leftDir = (left > 0) ? INCREASING : DECREASING;
    speed_t.rightDir = (right > 0) ? INCREASING : DECREASING;

    changeMotorSpeeds(speed_t, 0);
    //logOdometry(); // log inside changeMotorSpeeds ?? 
    

    // delay task 
    vTaskDelay( xDelay );
  } 
}

void TaskBrake ( void *pvParameters) {
  (void) pvParameters;

  const TickType_t xDelay = pdMS_TO_TICKS( 50UL );
  
  for (;;)
  {
    // take interrupt semaphore 
    if (xSemaphoreTake( xBreakSemaphore, (TickType_t) 1 ) == pdPASS) {
        // if breaks not engaged 
        if (brakeState == 0)
        {
          // engage breaks 
          digitalWrite(BRAKE_LEFT, HIGH);
          digitalWrite(BRAKE_RIGHT, HIGH);
                 
          //logOdometry();
    
          // update state 
          left = 0;
          right = 0;
          MotorSpeed speed_t;
          speed_t.left = 0; speed_t.right = 0;
          changeMotorSpeeds(speed_t, 1);
          
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
      
      // return to normal priority 
      vTaskPrioritySet( xTaskBreakHandle, 1);
    }
    // give semaphore for resource sharing, after resource used by task 
    xSemaphoreGive(xResourceSemaphore);
    
    // delay task 
    vTaskDelay( xDelay );
  }
}

static void TaskDeadReckoning( void * pvParameters ) {
  (void) pvParameters;

  long lastUpdateTime = millis();
  long lastPrintTime = millis();

  const TickType_t xDelay = pdMS_TO_TICKS(15UL);
  for (;;) {

    // print odometry every second 
    if ( (diagnosticState == 0) && (millis() - lastPrintTime) > ODOMETRY_UPDATE_TIME * 1000 ) {
      logOdometry();
      lastPrintTime = millis();
    }
      /*
      unsigned temp;
   temp = uxTaskGetStackHighWaterMark(NULL);
    
    if (!stack_hwm || temp < stack_hwm) {
        stack_hwm = temp;
        Serial.print(", High Watermark from function1: ");
        Serial.println(stack_hwm); // https://www.freertos.org/uxTaskGetStackHighWaterMark.html
    }
    */
    
    float vL = left;
    float vR = right;

    // no rotation, movement in straight line -> limiting case, else R -> infinity 
    if (vL == vR) {
      long dt = (millis() - lastUpdateTime) / 1000;
      lastUpdateTime = millis();
      
      pose.x += vR * cos(pose.theta) * dt;
      pose.y += vR * sin(pose.theta) * dt;

      pose.linearVel = vR;
      pose.rotVel = 0.0;
    }
    else {      
      float R = (DISTANCE_BETWEEN_WHEEL_CENTRES / 2) * ( (vL + vR) / (vR - vL) );
      float omega = (vR - vL) / DISTANCE_BETWEEN_WHEEL_CENTRES;
  
      float ICCx = pose.x - R * sin(pose.theta);
      float ICCy = pose.y + R * cos(pose.theta);
      
      long dt = (millis() - lastUpdateTime) / 1000;
      lastUpdateTime = millis();
  
      pose.x = ( cos(omega * dt) * (pose.x - ICCx) ) + ( -sin(omega * dt) * (pose.y - ICCy) ) + ICCx; 
      pose.y = ( sin(omega * dt) * (pose.x - ICCx) ) + ( cos(omega * dt) * (pose.y - ICCy) ) + ICCy;
      pose.theta += omega * dt;    

      pose.linearVel = (vR + vL) / 2;
      pose.rotVel = omega;
   }
    // delay task 
    taskYIELD();
  }
  
}

static void TaskDiagnostic( void *pvParameters ) {
    (void) pvParameters;
    
    for (;;) {
      if (diagnosticState == 1) {
        if (xTimerIsTimerActive( xDiagnosticTimer ) == pdFALSE) {

          // update maximums         
          if (dgn.currentLeft > dgn.absoluteLeft) 
            dgn.absoluteLeft = dgn.currentLeft;
          
          if (dgn.currentRight > dgn.absoluteRight) 
            dgn.absoluteRight = dgn.currentRight;      

          // print maxed sensed currents
          Serial.print("Current max current in left motor: ");
          Serial.println(dgn.currentLeft);
          Serial.print("Current max current in right motor: ");
          Serial.println(dgn.currentRight);
          Serial.print("Absolute max current in left motor: ");
          Serial.println(dgn.absoluteLeft);
          Serial.print("Absolute max current in right motor: ");
          Serial.println(dgn.absoluteRight);

          // reset current values and counter 
          dgn.currentRight = 0; dgn.currentLeft = 0;            
          xTimerReset( xDiagnosticTimer, 0 );
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

  for (;;) {
    
  }
}

void breakInterruptHandler() {
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  
  xSemaphoreGiveFromISR(xBreakSemaphore, NULL);  

  // set higher priority to break
  vTaskPrioritySet( xTaskBreakHandle, 2); 
}

static void diagnosticTimerCallback( TimerHandle_t xTimer ) {
  // check currents in DC motors
  int senseLeft = analogRead(SENSE_LEFT);
  int senseRight = analogRead(SENSE_RIGHT);

  // update accordingly 
  if (senseLeft > dgn.currentLeft )
    dgn.currentLeft = senseLeft;

  if (senseRight > dgn.currentRight )
    dgn.currentRight = senseRight;
}

static void sensorProcessingTimerCallback (TimerHandle_t xTimer) {

}

void diagnosticInterruptHandler() {
  // if last debounce time is more than a threshold, modify diagnostic state 
  if ( (millis() - lastDebounceDiagnostic) > DEBOUNCE_DELAY ) {
    diagnosticState = !diagnosticState;
    
    lastDebounceDiagnostic = millis();
    
    if (diagnosticState == 1) {
      Serial.println("DIAGNOSTIC MODE ON");
      // reset current values and counter 
      dgn.currentRight = 0; dgn.currentLeft = 0;            
      xTimerStart( xDiagnosticTimer, 0 );
    }
    else {
      Serial.println("DIAGNOSTIC MODE OFF");
      xTimerStop ( xDiagnosticTimer, 0 );
    }      
  }
}

void logOdometry()
{
  Serial.print("Linear velocity: ");
  Serial.print(pose.linearVel);
  Serial.println(" m/s");

  Serial.print("Angular velocity: ");
  Serial.print(pose.rotVel);
  Serial.println(" rad/s");

  Serial.print("Position: (");
  Serial.print(pose.x);
  Serial.print(", ");
  Serial.print(pose.y);
  Serial.println(")");

  Serial.print("Orientation: ");
  Serial.print(TO_DEGREES(pose.theta));
  Serial.println(" degrees");
}

void changeMotorSpeeds(MotorSpeed speeds, int brake)
{   
  // change right motor
  analogWrite(PWM_RIGHT_OUTPUT, speeds.right); // speed
  digitalWrite(DIR_RIGHT_OUTPUT, speeds.rightDir); // direction
  digitalWrite(BRAKE_RIGHT, brake); // don't brake 
  
  // change left motor
  analogWrite(PWM_LEFT_OUTPUT, speeds.left); // speed
  digitalWrite(DIR_LEFT_OUTPUT, speeds.leftDir); // direction
  digitalWrite(BRAKE_LEFT, brake); // don't brake

}
