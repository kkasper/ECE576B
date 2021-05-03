/*
*James Kuban, Kevin Kasper, Mitchell Russell, Samson Weisbrod                    
*/

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Local includes. */
#include "console.h"

//Defines
#define FRAME   278
#define X       86
#define Y       52

/*#define FRAME   278
#define X       66
#define Y       40*/

#define H_PROXSENS_PRIORITY          30
#define H_IMU_PRIORITY               30
#define H_GPS_PRIORITY               30
#define H_VIDEOFEED_PRIORITY         30
#define H_VIDEOFORWARD_PRIORITY      30
#define H_CONTROL_PRIORITY           30
#define H_MOTOR_PRIORITY             30
#define H_MONITOR_PRIORITY           30


#define DELAY 1000
#define BUFFER_SCALE 10

static void proxSens(void *pvParameters);
static void imu(void *pvParameters);
static void gps(void *pvParameters);
static void videoFeed(void *pvParameters);
static void videoForward(void *pvParameters);
static void control(void *pvParameters);
static void motor(void *pvParameters);
static void monitor(void *pvParameters);

//Queues Handles
static QueueHandle_t proxRxCon      = NULL;
static QueueHandle_t proxRxMon      = NULL;
static QueueHandle_t imuRx          = NULL;
static QueueHandle_t gpsTxProx      = NULL;
static QueueHandle_t gpsTxMon       = NULL;
static QueueHandle_t videoRx        = NULL;
static QueueHandle_t videoTx        = NULL;
static QueueHandle_t commandRx      = NULL;
static QueueHandle_t commandTxGPS   = NULL;
static QueueHandle_t commandTxMot   = NULL;
static QueueHandle_t commandTxMon   = NULL;
static QueueHandle_t motorTx        = NULL;

//Task Handles
static TaskHandle_t h_proxSens      = NULL;
static TaskHandle_t h_imu           = NULL;
static TaskHandle_t h_gps           = NULL;
static TaskHandle_t h_videoFeed     = NULL;
static TaskHandle_t h_videoForward  = NULL;
static TaskHandle_t h_control       = NULL;
static TaskHandle_t h_motor         = NULL;
static TaskHandle_t h_monitor       = NULL;

char vid[FRAME][Y][X];

extern "C" void main_blinky(void)
{
    proxRxCon       = xQueueCreate(6 * BUFFER_SCALE, sizeof(double));
    proxRxMon       = xQueueCreate(6 * BUFFER_SCALE, sizeof(double));
    imuRx           = xQueueCreate(6 * BUFFER_SCALE, sizeof(double));
    gpsTxProx       = xQueueCreate(3 * BUFFER_SCALE, sizeof(double));
    gpsTxMon        = xQueueCreate(3 * BUFFER_SCALE, sizeof(double));
    videoRx         = xQueueCreate(X * Y * BUFFER_SCALE, sizeof(char));
    videoTx         = xQueueCreate(X * Y * BUFFER_SCALE, sizeof(char));
    commandRx       = xQueueCreate(3 * BUFFER_SCALE, sizeof(int8_t));
    commandTxGPS    = xQueueCreate(3 * BUFFER_SCALE, sizeof(int8_t));
    commandTxMot    = xQueueCreate(3 * BUFFER_SCALE, sizeof(int8_t));
    commandTxMon    = xQueueCreate(3 * BUFFER_SCALE, sizeof(int8_t));
    motorTx         = xQueueCreate(4 * BUFFER_SCALE, sizeof(uint8_t));
    
    /*double gpsInit[] = {0, 0, 0};
    for(int i = 0; i < 3; i++){
        xQueueSend(gpsTx, &gpsInit[i], portMAX_DELAY);
    }*/
    
    char vidLineTemp[X];
    std::ifstream vidFile;
    vidFile.open("movie2.csv");
    for(int i = 0; i < FRAME; i++){
        for(int j = 0; j < Y; j++){
            vidFile.getline(vidLineTemp, X);
            for(int k = 0; k < X; k++){
                vid[i][j][k] = vidLineTemp[k];
            }
            vidFile.clear();
        }
    }
    /*for(int i = 0; i < Y; i++){
        for(int j = 0; j < X; j++){
            std::cout << vid[0][i][j];
        }
        std::cout << std::endl;
        //std::cout << "\n";
    }*/
    //std::cout << std::endl;
    
        
    vidFile.close();
    
    xTaskCreate(proxSens, "PRXSN", configMINIMAL_STACK_SIZE, NULL, H_PROXSENS_PRIORITY, &h_proxSens);
    xTaskCreate(imu, "IMU", configMINIMAL_STACK_SIZE, NULL, H_IMU_PRIORITY, &h_imu);
    xTaskCreate(gps, "GPS", configMINIMAL_STACK_SIZE, NULL, H_GPS_PRIORITY, &h_gps);
    xTaskCreate(videoFeed, "VDFE", configMINIMAL_STACK_SIZE, NULL, H_VIDEOFEED_PRIORITY, &h_videoFeed);
    xTaskCreate(videoForward, "VDFWD", configMINIMAL_STACK_SIZE, NULL, H_VIDEOFORWARD_PRIORITY, &h_videoForward);
    xTaskCreate(control, "CON", configMINIMAL_STACK_SIZE, NULL, H_CONTROL_PRIORITY, &h_control);
    xTaskCreate(motor, "MOT", configMINIMAL_STACK_SIZE, NULL, H_MOTOR_PRIORITY, &h_motor);
    xTaskCreate(monitor, "MON", configMINIMAL_STACK_SIZE, NULL, H_MONITOR_PRIORITY, &h_monitor);
    
    vTaskStartScheduler();
    
    for( ;; );
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
    double bound_cond = 10;
    double bounds[] = {100, -100, 100, -100, 100, -100};
    
    for( ;; ){
        if(uxQueueMessagesWaiting(gpsTxProx) >= 3){
            for(int i = 0; i < 3; i++){
                xQueueReceive(gpsTxProx, &gps, portMAX_DELAY);

                distance[2*i] = (bounds[2*i] - gps < bound_cond) ? (bounds[2*i] - gps) : -1;
                distance[2*i + 1] = (gps - bounds[2*i + 1] < bound_cond) ? (gps - bounds[2*i + 1]) : -1;
            }
            for(int i = 0; i < 6; i++){
                xQueueSend(proxRxCon, &distance[i], portMAX_DELAY);
                xQueueSend(proxRxMon, &distance[i], portMAX_DELAY);
            }
        }
    }
}

/** @fn static void imu()
*   @brief ???
*   @param ???
*   @return void
**/
static void imu(void *pvParameters)
{
    //maybe dummy data? (no actual feedback just rand)
    double inertia[6]; //rotation x3, acceleration x3
    
    for( ;; ){
        for(int i = 0; i < 3; i++){
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
    double location[3] = {0, 0, 0};//x, y, z
    int8_t control[3]; //x, y, z: -127 full reverse, 128 full forward
    
    for(int i = 0; i < 3; i++){
        xQueueSend(gpsTxProx, &location[i], portMAX_DELAY);
        xQueueSend(gpsTxMon, &location[i], portMAX_DELAY);
    }
    
    for( ;; ){
        if(uxQueueMessagesWaiting(commandTxGPS) >= 3){
            for(int i = 0; i < 3; i++){
                xQueueReceive(commandTxGPS, &control[i], portMAX_DELAY);
            }
            for(int i = 0; i < 3; i++){
                location[i] += (double)control[i] / 128.0;
            }
            for(int i = 0; i < 3; i++){
                xQueueSend(gpsTxProx, &location[i], portMAX_DELAY);
                xQueueSend(gpsTxMon, &location[i], portMAX_DELAY);
            }
        }
    }
}

//static void gpsMon(void *pvParameters){}

/** @fn static void videoFeed()
*   @brief Creates the video feed pixels randomly
*   @param no parameters because the input is randomly generated by the task.
*   @return void
**/
static void videoFeed(void *pvParameters)
{
    //char vid[FRAME][Y][X]; //[Frame][Y pixels][X pixels]
    
    for( ;; ){
        for(int i = 0; i < FRAME; i++){
            for(int j = 0; j < Y; j++){
                for(int k = 0; k < X; k++){
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
    char frame[Y][X]; //[Y pixel][X pixel]
    
    //Do we want to direct forawrd each "pixel"
    for( ;; ){
        if(uxQueueMessagesWaiting(videoRx) >= X*Y){
            for(int i = 0; i < Y; i++){
                for(int j = 0; j < X; j++){
                    xQueueReceive(videoRx, &frame[i][j], portMAX_DELAY);
                }
            }
            for(int i = 0; i < Y; i++){
                for(int j = 0; j < X; j++){
                    xQueueSend(videoTx, &frame[i][j], portMAX_DELAY);
                }
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
    double prox[6];
    int8_t commandRAW[3];
    int8_t commandWRAPPED[3];
    int8_t COMMAND[12][3] = {
                        {1, 3, 6},
                        {20, 30, 40},
                        {8, 12, 15},
                        {67, 89, 47},
                        {21, 31, 41},
                        {88, 99, 103},
                        {27, 46, 89},
                        {99, 120, 80},
                        {100, 90, 80},
                        {20, 48, 56},
                        {77, 99, 67},
                        {77, 82, 92}
                    };
    int j = 0;
    
    for( ;; ){
        if(uxQueueMessagesWaiting(proxRxCon) >= 6){
            for(int i = 0; i < 6; i++){
                xQueueReceive(proxRxCon, &prox[i], portMAX_DELAY);
            }
        }
        for(int i = 0; i < 3; i++){
            commandRAW[i] = COMMAND[j][i];
            if((prox[2*i] == -1 || prox[2*i] > 7) && (prox[2*i + 1] == -1 || prox[2*i + 1] > 7)){
                commandWRAPPED[i] = commandRAW[i];
            }
            /*else if(prox[i] > 2.5){
                commandWRAPPED[i] = commandRAW[i] / 2;
            }*/
            /*else if(prox[i] < 1){
                commandWRAPPED[i] = -64;
            }*/
            else{
                commandWRAPPED[i] = 0;
            }
        }
        for(int i = 0; i < 3; i++){
            xQueueSend(commandRx, &commandRAW[i], portMAX_DELAY);
            xQueueSend(commandTxGPS, &commandWRAPPED[i], portMAX_DELAY);
            xQueueSend(commandTxMot, &commandWRAPPED[i], portMAX_DELAY);
            xQueueSend(commandTxMon, &commandWRAPPED[i], portMAX_DELAY);
        }
        j = (j < 11) ? j + 1 : 0;
    }
}

/** @fn static void motor()
*   @brief Outputs the motor information for each of the four rotors in the
*          in the quadcopter.
*   @param Task parameters, maybe increase speed or decrease speed of specific
*          motor for particular rotorcraft directional movement.
*   @return void
**/
static void motor(void *pvParameters)
{
    double pwm[3]; //each motor 0 to 255 (rpm? Top speed is 255?)
    int8_t control[3]; //x, y, z: -127 full reverse, 128 full forward
    
    for( ;; ){
        if(uxQueueMessagesWaiting(commandTxMot) >= 3){
            for(int i = 0; i < 3; i++){
                xQueueReceive(commandTxMot, &control[i], portMAX_DELAY);
            }
            //std::cout << "motor" << std::endl;
            pwm[0] = sqrt(pow((double)(int)control[0], 2.0) + pow((double)(int)control[1], 2.0)); //rho
            pwm[1] = atan((double)(int)control[0] / (double)(int)control[1]); //phi
            /*std::cout << "Raw: " << control[2] << std::endl;
            std::cout << "Int: " << (int)control[2] << std::endl;
            std::cout << "Double: " << (double)control[2] << std::endl;*/
            pwm[2] = (double)control[2]; //z
            for(int i = 0; i < 3; i++){
                xQueueSend(motorTx, &pwm[i], portMAX_DELAY);
            }
        }
    }
}

//static void motorMon(void *pvParameters){}

/** @fn static void monitor()
*   @brief monitors something I really don't know.
*   @param no parameters
*   @return void
**/
static void monitor(void *pvParameters)
{
    double  prox[6];
    double  imu[6];
    double  gps[3];
    char    video[Y][X];
    int8_t  commandRAW[3];
    int8_t  commandWRAPPED[3];
    double motor[3];
    
    for( ;; ){
        if(uxQueueMessagesWaiting(gpsTxMon) >= 3){
            for(int i = 0; i < 3; i++){
                xQueueReceive(gpsTxMon, &gps[i], portMAX_DELAY);
            }
        }
        if(uxQueueMessagesWaiting(commandTxMon) >= 3){
            for(int i = 0; i < 3; i++){
                xQueueReceive(commandTxMon, &commandWRAPPED[i], portMAX_DELAY);
            }
        }
        if(uxQueueMessagesWaiting(motorTx) >= 3){
            for(int i = 0; i < 3; i++){               
                xQueueReceive(motorTx, &motor[i], portMAX_DELAY);
            }
        }
        if(uxQueueMessagesWaiting(commandRx) >= 3){
            for(int i = 0; i < 3; i++){
                xQueueReceive(commandRx, &commandRAW[i], portMAX_DELAY);
            }
        }
        if(uxQueueMessagesWaiting(proxRxMon) >= 6){
            for(int i = 0; i < 6; i++){
                xQueueReceive(proxRxMon, &prox[i], portMAX_DELAY);
            }
        }
        if(uxQueueMessagesWaiting(imuRx) >= 6){
            for(int i = 0; i < 6; i++){
                xQueueReceive(imuRx, &imu[i], portMAX_DELAY);
            }
        }
        if(uxQueueMessagesWaiting(videoRx) >= X*Y){
            for(int i = 0; i < Y; i++){
                for(int j = 0; j < X; j++){
                    xQueueReceive(videoRx, &video[i][j], portMAX_DELAY);
                }
            }
        }
        
        vTaskDelay((TickType_t) 50);
        if (system("CLS")) system("clear");
        std::cout << "\n\n\n";
        std::cout << "Video Feed:\n";
        std::cout << "    \x1B[01;95m";
        for(int j = 0; j < X; j++){
                std::cout << "-";
        }
        std::cout << "\n";
        for(int i = 0; i < Y; i += 2){
            std::cout << "    |";
            for(int j = 0; j < X; j++){
                std::cout << video[i][j];
            }
            if(i%2 != 0) std::cout << " ";
            std::cout << "|\n";
        }
        std::cout << "    ";
        for(int j = 0; j < X; j++){
                std::cout << "-";
        }
        std::cout << "\x1B[0m\n\n\n";
        
        std::cout << "GPS: ";
        for(int i = 0; i < 3; i++){
            std::cout << gps[i];
            if(i < 2) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "Proximity: ";
        for(int i = 0; i < 6; i++){
            std::cout << prox[i];
            if(i < 5) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "IMU: ";
        for(int i = 0; i < 6; i++){
            std::cout << imu[i];
            if(i < 5) std::cout << ", ";
        }
        
        std::cout << "\n\n\n";
        std::cout << "Raw Command Data: ";
        for(int i = 0; i < 3; i++){
            std::cout << (int)commandRAW[i];
            if(i < 2) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "Proccessed Command Data: ";
        for(int i = 0; i < 3; i++){
            std::cout << (int)commandWRAPPED[i];
            if(i < 2) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "Motor Output: ";
        for(int i = 0; i < 3; i++){
            std::cout << motor[i];
            if(i < 2) std::cout << ", ";
        }
        std::cout << std::endl;/**/
    }
}
