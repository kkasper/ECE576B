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
#define H_PROXSENS_PRIORITY          x
#define H_IMU_PRIORITY               x
#define H_GPS_PRIORITY               x
#define H_VIDEOFEED_PRIORITY         x
#define H_VIDEOFORWARD_PRIORITY      x
#define H_CONTROL_PRIORITY           x
#define H_MOTOR_PRIORITY             x
#define H_MONITOR_PRIORITY           x
#define H_BIGCHUNGUS_PRIORITY        x //MAX

#define PROX_SYNC_BITS  
#define GPS_SYNC_BITS   
#define COM_SYNC_BITS   

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
static TaskHandle_t h_bigChungus    = NULL;

//Event Group Handle
static EventGroupHandle_t theBois = NULL;

void main_drone(void)
{
    //TODO: xQueueCreate
    proxRx      = xQueueCreate(6, sizeof(double));//
    imuRx       = xQueueCreate(6, sizeof(double));
    //gpsRx       = xQueueCreate(3, sizeof(double));//
    gpsTx       = xQueueCreate(3, sizeof(double));//
    videoRx     = xQueueCreate(, sizeof(char));
    videoTx     = xQueueCreate(, sizeof(char));
    commandRx   = xQueueCreate(3, sizeof(int8_t));
    commandTx   = xQueueCreate(3, sizeof(int8_t));//
    motorTx     = xQueueCreate(4, sizeof(uint8_t));
    
    //TODO: initialize Queues gps, 
    double gpsInit[] = {0, 0, 0};
    for(int i = 0; i < 3; i++){
        xQueueSend(gpsTx, &gpsInit[i], portMAX_DELAY);
    }
    
    theBois = xEventGroupCreate();
    
    xTaskCreate(proxSens, "PRXSN", configMINIMAL_STACK_SIZE, NULL, H_PROXSENS_PRIORITY, &h_proxSens);
    xTaskCreate(imu, "IMU", configMINIMAL_STACK_SIZE, NULL, H_IMU_PRIORITY, &h_imu);
    xTaskCreate(gps, "GPS", configMINIMAL_STACK_SIZE, NULL, H_GPS_PRIORITY, &h_gps);
    xTaskCreate(videoFeed, "VDFE", configMINIMAL_STACK_SIZE, NULL, H_VIDEOFEED_PRIORITY, &h_videoFeed);
    xTaskCreate(videoForward, "VDFWD", configMINIMAL_STACK_SIZE, NULL, H_VIDEOFORWARD_PRIORITY, &h_videoForward);
    xTaskCreate(control, "CON", configMINIMAL_STACK_SIZE, NULL, H_CONTROL_PRIORITY, &h_control);
    xTaskCreate(motor, "MOT", configMINIMAL_STACK_SIZE, NULL, H_MOTOR_PRIORITY, &h_motor);
    xTaskCreate(monitor, "MON", configMINIMAL_STACK_SIZE, NULL, H_MONITOR_PRIORITY, &h_monitor);
    xTaskCreate(bigChungus, "BC", configMINIMAL_STACK_SIZE, NULL, , &h_bigChungus);
    
    vTaskStartScheduler();
    
    for( ;; ); // <-- We go beyond this, the code breaks.
}

static void bigChungus(void *pvParameters)
{
    double trashD;
    int8_t trashI;
    EventBits_t uxReturn;
    EventBits_t prox_start_bits = ;
    EventBits_t gps_start_bits = ;
    EventBits_t com_start_bits = ;
    //TODO: think about deadlock
    for(;;){
        if(uxQueueMessagesWaiting(proxRx) > 0){
            uxReturn = xEventGroupSync(theBois, prox_start_bits, PROX_SYNC_BITS, DELAY);
            if(uxReturn == PROX_SYNC_BITS){
                prox_start_bits = ;
                xQueueReceive(proxRx, &trashD, portMAX_DELAY);
            }
            else{
                prox_start_bits = uxReturn;
            }
        }
        if(uxQueueMessagesWaiting(gpsTx) > 0){
            uxReturn = xEventGroupSync(theBois, gps_start_bits, GPS_SYNC_BITS, DELAY);
            if(uxReturn == GPS_SYNC_BITS){
                gps_start_bits = ;
                xQueueReceive(proxRx, &trashD, portMAX_DELAY);
            }
            else{
                gps_start_bits = uxReturn;
            }
        }
        if(uxQueueMessagesWaiting(commandTx) > 0){
            uxReturn = xEventGroupSync(theBois, com_start_bits, COM_SYNC_BITS, DELAY);
            if(uxReturn == COM_SYNC_BITS){
                com_start_bits = ;
                xQueueReceive(proxRx, &trashI, portMAX_DELAY);
            }
            else{
                com_start_bits = uxReturn;
            }
        }
    }
}

/** @fn static void proxSens()
*   @brief Depends on how our obstacles behave. TBD!
*   TODO
*   @param Task parameters are current gps coordinates?
*   @return void
**/
static void proxSens(void *pvParameters)
{
    double distance[6]; //+x, -x, +y, -y, +z, -z
    double gps;
    double bound_cond = 3.14;
    double bounds[] = {10, 10, 10};
    
    for( ;; ){
        for(int i = 0; i < 3; i++){
            xEventGroupWaitBits(theBois, , pdFALSE, portMAX_DELAY);
            xQueuePeek(gpsTx, *gps, portMAX_DELAY);
            xEventGroupSync(theBois, , GPS_SYNC_BITS, portMAX_DELAY);
            distance[2*i] = (bounds[i] - gps < bound_cond) ? (bounds[i] - gps) : -1;
            distance[2*i + 1] = (gps - bounds[i] < bound_cond) ? (gps - bounds[i]) : -1;
        }
        for(int i = 0; i < 6; i++){
            xQueueSend(proxRx, &distance[i], portMAX_DELAY);
        }
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
    double inertia[6]; //rotation x3, acceleration x3
    
    for( ;; ){
        for(int i = 0, i < 3; i++){
            inertia[i] = rand();
            inertia[2*i] = rand();
        }
        for(int i = 0; i < 6; i++){
            xQueueSend(imuRx, &inertia[i], portMAX_DELAY);
        }
    }
}

/** @fn static void gps()
*   @brief Creates the video feed pixels randomly
*   @param Task parameters for cartesian coor
*   @return void
**/
static void gps(void *pvParameters)
{
    double location[3] = {, , };//x, y, z
    int8_t control[3]; //x, y, z: -127 full reverse, 128 full forward
    
    for( ;; ){
        for(int i = 0; i < 3; i++){
            //TODO: peek
            xEventGroupWaitBits();
            xQueuePeek(commandTx, *control[i], portMAX_DELAY);
            xEventGroupSync(theBois, , , portMAX_DELAY);
        }
        for(int i = 0; i < 3; i++){
            location[i] += ;//TODO: math
            xQueueSend(gpsTx, location[i], portMAX_DELAY);
        }
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
    //TODO: ASCI image array thing
    char vid[][][]; //[Frame][x pixels][y pixels]
    
    for( ;; ){
        for(int i = 0; i < frame; i++){
            for(int j = 0; j < x; j++){
                for(int k = 0; k < y; k++){
                    xQueueSend(videoRx, &vid[i][j][k], portMAX_DELAY);
                }
            }
        }
    }
}

/** @fn static void videoForward()
*   @brief Forwards video pixel data to the monitor task
*   @param Task parameters to return video image to user??
*   @return void
**/
static void videoForward(void *pvParameters)
{
    char frame[][]; //[x pixel][y pixel]
    
    //Do we want to direct forawrd each "pixel"
    for( ;; ){
        while(0);
        for(int i = 0; i < x; i++){
            for(int j = 0; j < y; j++){
                xQueueReceive(videoRx, *frame[i][j], portMAX_DELAY);
            }
        }
        for(int i = 0; i < x; i++){
            for(int j = 0; j < y; j++){
                xQueueSend(videoTx, &frame[i][j], portMAX_DELAY);
            }
        }
        
    }
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
    int8_t commandRAW[3];
    int8_t commandWRAPPED[3];
    
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
    uint8_t pwm[4]; //each motor 0 to 255 (rpm? Top speed is 255?)
    int8_t control[3]; //x, y, z: -127 full reverse, 128 full forward
    for( ;; ){
        for(int i = 0; i < 3; i++){
            //TODO: xQueuePeek
            xEventGroupWaitBits();
            xQueuePeek(commandTx, *control[i], portMAX_DELAY);
            xEventGroupSync(theBois, , , portMAX_DELAY);
        }
        //TODO: MATH
        for(int i = 0; i < 4; i++){
            xQueueSend(motorTx, &pwm[i], portMAX_DELAY);
        }
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
    double  prox[6];
    double  imu[6];
    double  gps[3];
    char    video[][];
    int8_t  commandRAW[3];
    int8_t  commandWRAPPED[3];
    uint8_t motor[4];
    
    //TODO: Mucho xQueuePeek
    //TODO: xQueue
    for( ;; ){
        for(int i = 0; i < 3; i++){
            xEventGroupWaitBits();
            xQueuePeek(, *gps[i], portMAX_DELAY);
            xEventGroupSync(theBois, , , portMAX_DELAY);
            xEventGroupWaitBits();
            xQueuePeek(commandRx, *commandRAW[i], portMAX_DELAY);
            xEventGroupSync(theBois, , , portMAX_DELAY);
            xEventGroupWaitBits();
            xQueuePeek(commandTx, *commandWRAPPED[i], portMAX_DELAY);
            xEventGroupSync(theBois, , , portMAX_DELAY);
        }
        for(int i = 0; i < 4; i++){
            motor[i] = xQueueReceive(, , );
        }
        for(int i = 0; i < 6; i++){
            xEventGroupWaitBits();
            xQueuePeek(, *prox[i], portMAX_DELAY);
            xEventGroupSync(theBois, , , portMAX_DELAY);
            imu[i] = xQueueReceive(, , );
        }
        if(1){
            for(int i = 0; i < ; i++){  //TODO: Bounds
                for(int j = 0; i < ; i++){
                    video[i][j] = xQueueReceive(, , );
                }
            }
        }
        //TODO: Write that shit
    }
}
