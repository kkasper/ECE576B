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