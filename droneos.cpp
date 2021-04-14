/*
*James Kuban, Kevin Kasper, Mitchell Russell, Samson Weisbrod                    
*/

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

/* Local includes. */
#include "console.h"

//Defines
//TODO: Defines

static void proxSens(void *pvParameters);
static void imu(void *pvParameters);
static void gps(void *pvParameters);
//static void gpsMon(void *pvParameters);
static void videoFeed(void *pvParameters);
static void videoForward(void *pvParameters);
//static void videoMon(void *pvParameters)
//static void proxMon(void *pvParameters);//TODO: Something
static void control(void *pvParameters);
//static void imuMon(void *pvParameters);//TODO: Something
static void motor(void *pvParameters);
//static void motorMon(void *pvParameters);
static void monitor(void *pvParameters);

//Queues Handles
static QueueHandle_t proxRx     = NULL;
static QueueHandle_t imuRx      = NULL;
static QueueHandle_t gpsRx      = NULL;
static QueueHandle_t gpsTx      = NULL;
static QueueHandle_t videoRx    = NULL;
static QueueHandle_t videoTx    = NULL;
static QueueHandle_t commandRx  = NULL;
static QueueHandle_t commandTx  = NULL;
static QueueHandle_t motorTx    = NULL;

//Task Handles
static TaskHandle_t h_proxSens      = NULL;
static TaskHandle_t h_imu           = NULL;
static TaskHandle_t h_gps           = NULL;
static TaskHandle_t h_videoFeed     = NULL;
static TaskHandle_t h_videoForward  = NULL;
static TaskHandle_t h_control       = NULL;
static TaskHandle_t h_motor         = NULL;
static TaskHandle_t h_monitor       = NULL;

void main_drone(void)
{
    //TODO: Queues
    proxRx      = xQueueCreate(, );
    imuRx       = xQueueCreate(, );
    gpsRx       = xQueueCreate(, );
    gpsTx       = xQueueCreate(, );
    videoRx     = xQueueCreate(, );
    videoTx     = xQueueCreate(, );
    commandRx   = xQueueCreate(, );
    commandTx   = xQueueCreate(, );
    motorTx     = xQueueCreate(, );
    
    if(){
        //TODO: Tasks
        xTaskCreate(proxSens,"PRXSN", , , ,&h_proxSens);
        xTaskCreate(imu,"IMU", , NULL, , &h_imu);
        xTaskCreate(gps,"GPS", , , , &h_gps);
        //xTaskCreate(gpsTx, , , , , );
        xTaskCreate(videoFeed,"VDFE", , , , &h_videoFeed);
        xTaskCreate(videoForward,"VDFWD", , , , &h_videoForward);
        //xTaskCreate(commandRx, , , , , );
        //xTaskCreate(commandTx, , , , , );
        xTaskCreate(motor,"MOT", , , , &h_motor);
        xTaskCreate(monitor, "MON", configMINIMAL_STACK_SIZE, NULL, 
        	MONITOR_TASK_PRIORITY, &h_monitor);
        
        vTaskStartScheduler();
    }
    
    for( ;; ); // <-- We go beyond this, the code breaks.
}
/** @fn static void proxSens()
*   @brief Depends on how our obstacles behave. TBD!
*   TODO
*   @param Task parameters are current gps coordinates?
*   @return void
**/
static void proxSens(void *pvParameters)
{
    double distance[6];
    //+x, -x, +y, -y, +z, -z
    for( ;; ){
        
    }
}

/** @fn static void imu()
*   @brief ???
*   @param ???
*   @return void
**/
static void imu()
{
    //maybe dummy data? (no actual feedback just rand)
    double inertia[];
    //rotation x3, 
    for( ;; ){
        
    }
}

/** @fn static void gps()
*   @brief Creates the video feed pixels randomly
*   @param Task parameters for cartesian coor
*   @return void
**/
static void gps(void *pvParameters)
{
    double location[3];
    //x, y, z
    for( ;; ){
        
    }
}

//static void gpsMon(void *pvParameters){}

/** @fn static void videoFeed()
*   @brief Creates the video feed pixels randomly
*   @param no parameters because the input is randomly generated by the task.
*   @return void
**/
static void videoFeed()
{
    char vid[][][];
    //[Frame][x pixels][y pixels]
    for( ;; ){
        
    }
}

/** @fn static void videoForward()
*   @brief Forwards video pixel data to the monitor task
*   @param Task parameters to return video image to user??
*   @return void
**/
static void videoForward(void *pvParameters)
{
    char frame[][];
    //[x pixel][y pixel]
    
}

//static void videoMon(void *pvParameters){}

//static void proxMon(void *pvParameters){}

/** @fn static void control()
*   @brief Control is very complex and does a lot of things that I don't know
*          yet.
*   @param no parameters
*   @return void
**/
static void control(void *pvParameters)
{
    
    for( ;; ){
        
    }
}

//static void imuMon(void *pvParameters){}

/** @fn static void motor()
*   @brief Outputs the motor information for each of the four rotors in the
*          in the quadcopter.
*   @param Task parameters, maybe increase speed or decrease speed of specific
*          motor for particular rotorcraft directional movement.
*   @return void
**/
static void motor(void *pvParameters)
{
    uint8_t pwm[4];
    //each motor 0 to 255 (rpm? Top speed is 255?)
    for( ;; ){
        
    }
}

//static void motorMon(void *pvParameters){}

/** @fn static void monitor()
*   @brief monitors something I really don't know.
*   @param no parameters
*   @return void
**/
static void monitor()
{
    
    for( ;; ){
        
    }
}
