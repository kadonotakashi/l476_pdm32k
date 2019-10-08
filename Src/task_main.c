/*
 * task_main.c
 *
 *  Created on: 2017/09/10
 *      Author: Takashi
 */
#include "main.h"
#include "cmsis_os.h"

extern osMessageQId QueGLCDHandle;//main.c
void tk_main(void const * argument)
{

	for(;;)
	{
		osDelay(1);
	}
}

