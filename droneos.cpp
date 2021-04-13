/*
*James Kuban, Kevin Kasper, Mitchell Russell, Samson Weisbrod
*No CONFIG modifications
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

//Deines
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
        xTaskCreate(proxSens, , , , , );
        xTaskCreate(imuRx, , , , , );
        xTaskCreate(gpsRx, , , , , );
        xTaskCreate(gpsTx, , , , , );
        xTaskCreate(videoRx, , , , , );
        xTaskCreate(commandRx, , , , , );
        xTaskCreate(commandTx, , , , , );
        xTaskCreate(motorTx, , , , , );
        xTaskCreate(monitor, "MON", configMINIMAL_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, NULL);
        
        vTaskStartScheduler();
    }
    
    for( ;; );
}

static void proxSens(void *pvParameters)
{
    double distance[6];
    //+x, -x, +y, -y, +z, -z
    for( ;; ){
        
    }
}

static void imu(void *pvParameters)
{
    //maybe dummy data? (no actual feedback just rand)
    double inertia[];
    //rotation x3, 
    for( ;; ){
        
    }
}

static void gps(void *pvParameters)
{
    double location[3];
    //x, y, z
    for( ;; ){
        
    }
}

//static void gpsMon(void *pvParameters){}

static void videoFeed(void *pvParameters)
{
    char vid[][][];
    //[Frame][x pixels][y pixels]
    for( ;; ){
        
    }
}

static void videoForward(void *pvParameters)
{
    char frame[][];
    //[x pixel][y pixel]
    
}

//static void videoMon(void *pvParameters){}

//static void proxMon(void *pvParameters){}

static void control(void *pvParameters)
{
    
    for( ;; ){
        
    }
}

//static void imuMon(void *pvParameters){}

static void motor(void *pvParameters)
{
    uint8_t pwm[4];
    //each motor 0 to 255
    for( ;; ){
        
    }
}

//static void motorMon(void *pvParameters){}

static void monitor(void *pvParameters)
{
    
    for( ;; ){
        
    }
}
