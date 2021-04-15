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

#define DECREASE_RATE       20
#define INCREASE_RATE       20

#define SPEED_INCREASE      MAX_SPEED * INCREASE_RATE / 100
#define SPEED_DECREASE      MAX_SPEED * DECREASE_RATE / 100

#define MONITOR_FREQ        2
#define FLASH_FREQ          4 
#define FLASH_DURATION      3
#define NUM_FLASHES         (FLASH_DURATION * FLASH_FREQ)

#define DISTANCE_BETWEEN_WHEEL_CENTRES  2
#define WHEEL_RADIUS                    0.5
#define ODOMETRY_UPDATE_TIME            1

#define TO_PWM(x)         map((x), -MAX_SPEED, MAX_SPEED, -PWM_MAX_SPEED, PWM_MAX_SPEED)
#define TO_RPM(x)         map((x), -PWM_MAX_SPEED, PWM_MAX_SPEED, -MAX_SPEED, MAX_SPEED)

#define DEBOUNCE_DELAY    450 //500

#define DIAGNOSTIC_PERIOD           2000UL
#define SENSOR_PROCESSING_PERIOD    20UL
#define CPU_BUSY_PERIOD             5UL

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

/* left and right DC motor speeds */
float left = 0;
float right = 0;

/* Arrays storing the left and right DC motor speeds */
int measurementsLeft[NUMBER_MEASUREMENTS_AVERAGE];
int measurementsRight[NUMBER_MEASUREMENTS_AVERAGE];

/* Strings for logging data */
const char *outputs[5]; 

/* Struct for storing the pose of the vehicle */ 
typedef struct Pose{
  float x;
  float y;
  float theta;
  float linearVel;
  float rotVel;  
}Pose;
Pose pose;

/* Global variables for the state of breaks (engaged or not) and diagnostic */
int brakeState = 0;
int diagnosticState = 0;

/* FreeRTOS Tasks */
static void TaskUpdateMotorSpeeds( void *pvParameters );
static void TaskDiagnostic( void *pvParameters );
static void TaskProcessing ( void *pvParameters);
static void TaskMonitor (void *pvParameters );
static void TaskDeadReckoning (void *pvParameters );

/* Sensor processing task handle, timer handle and timer callback */
static TaskHandle_t xTaskHandle;
static TimerHandle_t xProcessingTimer;
static void vProcessingTimerCallback( TimerHandle_t xTimer );

/* Interrupt handler functions */
void breakInterruptHandler();
void diagnosticInterruptHandler();

/* Mutex for resource sharing */
SemaphoreHandle_t xSemaphore;

/* Debounce time for diagnostic */
volatile unsigned long lastDebounceDiagnostic = 0;

void setup() {    
  // create mutex 
  xSemaphore = xSemaphoreCreateBinary();

  // check that mutex was created correctly
  if (xSemaphore != NULL) {
    
    // set strings for logging 
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
  
    while (!Serial)
      ; // wait for serial port to connect
  
    // attach interrupt handler to break button and diagnostic button
    attachInterrupt(digitalPinToInterrupt(BRAKE_BUTTON), breakInterruptHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIAGNOSTIC_MODE_BUTTON), diagnosticInterruptHandler, LOW);    
    
    // create timer 
    xProcessingTimer = xTimerCreate(   "Processing timer",          /* Timer name */
                                        pdMS_TO_TICKS(SENSOR_PROCESSING_PERIOD),   /* Timer period */
                                        pdFALSE,                    /* One-shot timer */
                                        ( void * ) 0,               /* ID not used */
                                        vProcessingTimerCallback    /* Callback function that switches priority of task */
                                    );
  
    // task for updating motor speeeds 
    xTaskCreate(
      vMotorUpdateTask
      ,  "Updated Motor Speeds"   /* Task name */
      ,  100                      /* Stack size */
      ,  NULL                     /* No parameters to pass */
      ,  1                        /* Low priority */
      ,  NULL );                  /* No task handler */ 
  
    // task for diagnostic
      xTaskCreate(
      vDiagnosticsTask                
      ,  "Diagnostics"            /* Task name */
      ,  87                       /* Stack size */
      ,  NULL                     /* No parameters to pass */
      ,  1                        /* Low priority */
      ,  NULL );                  /* No task handler */ 
  
    // task for velocity monitoring
     xTaskCreate(
      vDeadReckoningTask
      ,  "Dead Reckoning"         /* Task name */
      ,  142                      /* Stack size */
      ,  NULL                     /* No parameters to pass */
      ,  1                        /* Low priority */
      ,  NULL );                  /* No task handler */ 
  
    // task for sensor processing 
    xTaskCreate(
      vSensorProcessingTask
      ,  "Sensor Processing"      /* Task name */
      ,  68                       /* Stack size */
      ,  NULL                     /* No parameters to pass */
      ,  2                        /* High priority */
      ,  &xTaskHandle );          /* Task handler for priority switching */
  
    // task for velocity monitoring 
    xTaskCreate(
     vVelocityMonitoringTask
      ,  "Velocity Monitoring"      /* Task name */
      ,  84                        /* Stack size 100*/
      ,  NULL                       /* No parameters to pass */
      ,  1                          /* Low priority */
      ,  NULL );                    /* Task handler for priority switching */
  
    // start scheduler so created task start executing 
    vTaskStartScheduler();
  }
}

void vMotorUpdateTask( void *pvParameters) {
  (void) pvParameters;

  // last time buttons were triggered
  long lastDebounceDecreaseLeft = 0;
  long lastDebounceIncreaseLeft = 0;
  long lastDebounceDecreaseRight = 0;
  long lastDebounceIncreaseRight = 0;

  bool speedChanged = false;
  
  for (;;) {   
    // if breaks are engaged 
    if (brakeState == 0) {
      
      // check button inputs 
      int increaseLeft = digitalRead(INCREASE_LEFT_BUTTON);
      int decreaseLeft = digitalRead(DECREASE_LEFT_BUTTON);
      int increaseRight = digitalRead(INCREASE_RIGHT_BUTTON);
      int decreaseRight = digitalRead(DECREASE_RIGHT_BUTTON);

        // if last debounce time is more than a threshold, enter loop
        if ( ((TIME_MS - lastDebounceIncreaseLeft) > DEBOUNCE_DELAY) && (increaseLeft == LOW) ) {
          
          // if button is pressed and break is not activated increase speed
          speedChanged = true;
          left += SPEED_INCREASE;   
          if (left > MAX_SPEED) 
            left = MAX_SPEED;
          
          lastDebounceIncreaseLeft = TIME_MS;
        }
        
        // if last debounce time is more than a threshold, enter loop        
        if ( (TIME_MS - lastDebounceDecreaseLeft > DEBOUNCE_DELAY) && (decreaseLeft == LOW) ) {
          
          // if button is pressed and break is not activated decrease speed
          speedChanged = true;
          left -= SPEED_DECREASE;
          if (left < -MAX_SPEED) 
            left = -MAX_SPEED;
          
          lastDebounceDecreaseLeft = TIME_MS;
        }
        
        // if last debounce time is more than a threshold, enter loop
        if ( ((TIME_MS - lastDebounceIncreaseRight) > DEBOUNCE_DELAY) && (increaseRight == LOW) ) {
          
          // if button is pressed and break is not activated increase speed
          speedChanged = true;
          right += SPEED_INCREASE;      
          if (right > MAX_SPEED) 
            right = MAX_SPEED;
          
          lastDebounceIncreaseRight = TIME_MS;
        }
        
        // if last debounce time is more than a threshold, enter loop
        if ( ((TIME_MS - lastDebounceDecreaseRight) > DEBOUNCE_DELAY) && (decreaseRight == LOW) ) {
          
          // if button is pressed and break is not activated decrease speed
          speedChanged = true;
          right -= SPEED_DECREASE;      
          if (right < -MAX_SPEED) 
            right = -MAX_SPEED;
          
          lastDebounceDecreaseRight = TIME_MS;
        }

        // if any button has been pressed and speed has changed 
        if (speedChanged == true) {

            // change speed without braking, reset speed changed and print pose and velocities to terminal 
            changeMotorSpeeds(0);
            speedChanged = false;
            logOdometry();                 
      }    
    }
    // give semaphore 
    xSemaphoreGive( xSemaphore );
    
    // yield task 
    taskYIELD();
  }    
}

static void vDeadReckoningTask( void * pvParameters ) {
  (void) pvParameters;

  // timestamps for pose update and last print to terminal
  long lastUpdateTime = TIME_MS;
  long lastPrintTime = TIME_MS;

  // variables for pose estimation 
  float dt = 0;
  float R = 0; float omega = 0;
  float ICCx = 0; float ICCy = 0;
  float left_m = 0; float right_m = 0;

  for (;;) {

    // try to take the semaphore for 1 tick 
    if (xSemaphoreTake( xSemaphore, (TickType_t) 1 ) == pdPASS) {
    
      // print odometry every second and update timestamp 
      if ((TIME_MS - lastPrintTime) > ODOMETRY_UPDATE_TIME * 1000) {           
        logOdometry();
        lastPrintTime = TIME_MS;
      }         
        
      // velocities in m/s
      left_m = left * WHEEL_RADIUS;
      right_m = right * WHEEL_RADIUS;

    }

    // update change in time 
    dt = (TIME_MS - lastUpdateTime) / 1000.0;

    // update time stamp 
    lastUpdateTime = TIME_MS;
    
    // no rotation, movement in straight line -> limiting case, else R -> infinity 
    if (left_m == right_m) {      
      // calculate new position, and velocities (theta stays the same) 
      pose.x += right_m * cos(pose.theta) * dt;
      pose.y += right_m * sin(pose.theta) * dt;

      pose.linearVel = right;
      pose.rotVel = 0.0;
    } else {     
      // calculate new position, orientation and velocities 
      R = (DISTANCE_BETWEEN_WHEEL_CENTRES / 2) * ( (left_m + right_m) / (right_m - left_m) );
      omega = (right_m - left_m) / DISTANCE_BETWEEN_WHEEL_CENTRES;
  
      ICCx = pose.x - R * sin(pose.theta);
      ICCy = pose.y + R * cos(pose.theta);
        
      pose.x = ( cos(omega * dt) * (pose.x - ICCx) ) + ( -sin(omega * dt) * (pose.y - ICCy) ) + ICCx; 
      pose.y = ( sin(omega * dt) * (pose.x - ICCx) ) + ( cos(omega * dt) * (pose.y - ICCy) ) + ICCy;

      pose.theta += omega * dt;    

      pose.linearVel = (right_m + left_m) / 2;
      pose.rotVel = omega;
    }

    // yield task 
    vTaskDelay( (TickType_t) pdMS_TO_TICKS(100));
  }
}

static void vDiagnosticsTask( void *pvParameters ) {
    (void) pvParameters;

    // storage for current and absolute maximums 
    uint16_t currentLeft = 0; uint16_t currentRight = 0;
    uint16_t absoluteLeft = 0; uint16_t absoluteRight = 0;   

    // time stamp for last update 
    long start = TIME_MS;
    
    for (;;) {
      // if diagnostic mode is on 
      if (diagnosticState == 1) {

        // if less than 2 seconds have passed 
        if ( (TIME_MS - start) < DIAGNOSTIC_PERIOD) {

          // check currents in DC motors
          int senseLeft = analogRead(SENSE_LEFT);
          int senseRight = analogRead(SENSE_RIGHT);
        
          // update accordingly 
          if (senseLeft > currentLeft )
            currentLeft = senseLeft;
        
          if (senseRight > currentRight )
            currentRight = senseRight;

        // if 2 seconds have passed 
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
          currentRight = 0; 
          currentLeft = 0; 
          
          // update time stamp 
          start = TIME_MS;      
      }
    } 
       
    // yield task 
    taskYIELD();
  }
}

static void vSensorProcessingTask ( void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {

    // keep CPU busy for 5ms 
    long initialTime = TIME_MS;
    while ((TIME_MS - initialTime) < CPU_BUSY_PERIOD) 
     ;
    
    // start timer 
    if (xTimerIsTimerActive( xProcessingTimer ) != pdTRUE) {
      xTimerStart( xProcessingTimer, 0);
    }
    
    // set priority to 1 
    vTaskPrioritySet(NULL, 1);
  }  
}

static void vProcessingTimerCallback( TimerHandle_t xTimer ) {
  // reset priority of sensor processing to 2 after 15ms 
  static long start = TIME_MS;
  if ( (TIME_MS - start) > (SENSOR_PROCESSING_PERIOD - CPU_BUSY_PERIOD)) {
    vTaskPrioritySet(xTaskHandle, 2);  
    start = TIME_MS;
  }
}

static void vVelocityMonitoringTask( void *pvParameters) {
  (void) pvParameters;

  // initialize sums, pointer, averages and arrays
  float sumLeft = 0;
  float sumRight = 0;
  uint8_t curr = 0;

  // time stamp for last update
  long start = 0; 
  
  // initialize velocity arrays 
  for (int i = 0; i < NUMBER_MEASUREMENTS_AVERAGE; i++) {
    measurementsLeft[i] = 0;
    measurementsRight[i] = 0;
  }

  // time stamp for last average update 
  long lastAverageUpdate = TIME_MS;
  long lastFlash = 0;

  // number of flashes and boolean for flashing enabled/disabled 
  int flashCount = 0;
  bool flashing = 0;
  
  for (;;) {
    // if diagnostic mode is off
    if (diagnosticState == 0) {

      // at 2Hz frequency update the rolling average 
      if ( (TIME_MS - lastAverageUpdate) > ((1.0 / MONITOR_FREQ) * 1000)) {
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
        
        // update time stamp
        lastAverageUpdate = TIME_MS; 
      }
      
      // check if rolling average is higher than maximum for both wheels 
      if ((sumLeft / NUMBER_MEASUREMENTS_AVERAGE) > MAX_ROLLING_AVERAGE && ((sumRight / NUMBER_MEASUREMENTS_AVERAGE) > MAX_ROLLING_AVERAGE)
        && ((TIME_MS - lastFlash) > (((1.0 / (FLASH_FREQ)) * 1000) - 50)) ) {
        
        // blink LED
        digitalWrite(MONITOR_LED, HIGH);
        vTaskDelay(  pdMS_TO_TICKS( 50 ) );
        digitalWrite(MONITOR_LED, LOW);
        
        lastFlash = TIME_MS;
         
        // update boolean 
        flashing = 1;
        
     } else {
        // if not above average but was flashing, continue flashing for 3 seconds, otherwise do nothing 
        if ( (flashing == 1) && (flashCount < NUM_FLASHES) && ((TIME_MS - lastFlash) > (((1.0 / (FLASH_FREQ)) * 1000) - 50))) { 
          
          // blink LED         
          digitalWrite(MONITOR_LED, HIGH);
          vTaskDelay(  pdMS_TO_TICKS( 50 ) );
          digitalWrite(MONITOR_LED, LOW);
  
          // update time stamp
          lastFlash = TIME_MS;
    
          // increase counter 
          flashCount++;
  
          // if flashes are over, reset variables 
          if (flashCount == NUM_FLASHES) {
            flashCount = 0;
            flashing = 0;
          }
        }  
      }
    }
    // yield task 
    taskYIELD();
  }
}

void breakInterruptHandler() { 

  // if breaks are not engaged 
  if (brakeState == 0) {

    // update left and right global variables  
    left = 0;
    right = 0;

    // change motor velocity and brake 
    changeMotorSpeeds(1);

    // print to terminal pose and velocities 
    //logOdometry();

    // update brake state 
    brakeState = 1;

    // print message on terminal 
    Serial.println("BRAKES ON");    
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
    
  // if breaks engaged  
  } else if (brakeState == 1) {
    
    // update state 
    brakeState = 0;
    
    // print message on terminal 
    Serial.println("BRAKES OFF");  
  }
}

void diagnosticInterruptHandler() {
  // if last debounce time is more than a threshold, modify diagnostic state 
  if ( (TIME_MS - lastDebounceDiagnostic) > DEBOUNCE_DELAY ) {
    diagnosticState = !diagnosticState;

    // update debouncing time stamp 
    lastDebounceDiagnostic = TIME_MS;     
  }
}

void logOdometry()
{
  // print velocities, position and orientation 
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

void changeMotorSpeeds(int brake)
{   
  // change right motor
  analogWrite(PWM_RIGHT_OUTPUT, abs(TO_PWM(right))); // speed
  digitalWrite(DIR_RIGHT_OUTPUT, (right > 0) ? HIGH : LOW); // direction
  digitalWrite(BRAKE_RIGHT, brake); // brake
  
  // change left motor
  analogWrite(PWM_LEFT_OUTPUT, abs(TO_PWM(left))); // speed
  digitalWrite(DIR_LEFT_OUTPUT, (left > 0) ? HIGH : LOW); // direction
  digitalWrite(BRAKE_LEFT, brake); // brake 

}

void loop() {  
// Empty. Things are done in Tasks.
}
