#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4_discovery.h"
#include "utils.h"
#include "iprintf.h"
#include "adcbsp.h"
#include "usart1bsp.h"
#include "stream.h"
#include "dds.h"
#include "debug.h"
#include "scheduler.h"
#include "meassurements.h"
#include "GFDProtocol.h"
#include "GFDMonitor.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
//
//TTaskHandler *testTaskHandler;
//
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

int main(void) {

	int x = 0;
//	iprintf("x = %d", x + 1);

	SCB ->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));

//	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb,
//			&USR_cb);

	SysTick_Config(SystemCoreClock / 1000);

	usart1bsp_Init(9600);


	meassurements_Init();
	meassurements_Start();

	initCommunicationModule();

//	setPortStream(COM1,DEBUG_STREAM,9600);
	setPortStream(COM2, DATA_STREAM, 9600);
//	setPortStream(USB, DATA_STREAM, 9600);
//	setPortStream(USB, DEBUG_STREAM, 9600);

	schedulerInit(false);

	GFDProtocol_Init();

	GFDMonitor_Init();

	schedulerRun();

	while (1)
		;

	return 0;
}
