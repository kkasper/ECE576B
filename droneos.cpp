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

//TODO: Add Tasks
static void proximitySensors(void *pvParameters);
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

//TODO: Add Queues
static QueueHandle_t proxRx;
static QueueHandle_t imuRx;
static QueueHandle_t gpsRx;
static QueueHandle_t gpsTx;
static QueueHandle_t videoRx;
static QueueHandle_t videoTx;
static QueueHandle_t commandRx;
static QueueHandle_t commandTx;
static QueueHandle_t motorTx;

void main_drone(void)
{
    
    
    
    if(){
        
        xTaskCreate(monitor, "MON", configMINIMAL_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, NULL);
    }
}