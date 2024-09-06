/**
 * Copyright (C) 2013, 2014 Johannes Taelman
 * Edited 2023 - 2024 by Ksoloti
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 */

#include "axoloti_defines.h"

#include "sdram.h"
#include "stm32f4xx_fmc.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "string.h"
#include <stdio.h>

#include "codec.h"
#include "ui.h"
#include "midi.h"
#include "sdcard.h"
#include "patch.h"
#include "pconnection.h"
#include "axoloti_control.h"
#include "axoloti_math.h"
#include "axoloti_board.h"
#include "exceptions.h"
#include "watchdog.h"

#include "chprintf.h"
#include "usbcfg.h"
#include "sysmon.h"
#ifdef FW_SPILINK
#include "spilink.h"
#endif

#include "sdram.c"
#include "stm32f4xx_fmc.c"

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

#define TEST_RAM_SPEED 0
#if TEST_RAM_SPEED
    #define TEST_RAM_SPEED_ITTERATIONS 20000
    #define TEST_RAM_SPEED_DATA_SIZE 1024
    uint8_t ccData[TEST_RAM_SPEED_DATA_SIZE] __attribute__ ((section (".ccmramend")));
    uint8_t s1Data[TEST_RAM_SPEED_DATA_SIZE] __attribute__ ((section (".sram1")));
    uint8_t s3Data[TEST_RAM_SPEED_DATA_SIZE] __attribute__ ((section (".sram3")));
    uint16_t __attribute__((optimize("O3"))) TestRamSpeed(uint8_t *pData)
//    uint16_t TestRamSpeed(uint8_t *pData)
    {
        uint16_t i;
        uint16_t m;
        uint16_t t = 0;

        for(i = 0; i < TEST_RAM_SPEED_ITTERATIONS; i++)
        {
            for(m = 0; m < TEST_RAM_SPEED_DATA_SIZE; m++)
                t += pData[m];
        }

        return t;
    }
#endif

#define ENABLE_SERIAL_DEBUG 1
#define ANDY_GPIO_DEBUG 1
extern void MY_USBH_Init(void);

#if TEST_SRAM3
#define SRAM3_BYTE_SIZE (1024*64)

uint8_t sram3Buffer[SRAM3_BYTE_SIZE] __attribute__ ((section (".sram3")));
#endif


int main(void) {
    /* copy vector table to SRAM1! */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
    memcpy((char *)0x20000000, (const char)0x00000000, 0x200);
#pragma GCC diagnostic pop

    /* remap SRAM1 to 0x00000000 */
    SYSCFG->MEMRMP |= 0x03;

    halInit();
    chSysInit();

#ifdef FW_SPILINK
    pThreadSpilink = 0;
#endif

    sdcard_init();
    sysmon_init();

#if ENABLE_SERIAL_DEBUG
    /* SD2 for serial debug output */
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7) | PAL_MODE_INPUT); /* RX */
    palSetPadMode(GPIOA, 2, PAL_MODE_OUTPUT_PUSHPULL); /* TX */
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); /* TX */

    /* 115200 baud */
    static const SerialConfig sd2Cfg = {115200, 0, 0, 0};
    sdStart(&SD2, &sd2Cfg);
    chprintf((BaseSequentialStream * )&SD2,"Hello %u world!\r\n", 17);
#endif

#if ANDY_GPIO_DEBUG
    palSetPadMode(GPIOG, 11, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOG, 10, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  3, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  4, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  5, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  6, PAL_MODE_OUTPUT_PUSHPULL); 

    palWritePad(GPIOG, 11, 0);
    palWritePad(GPIOG, 10, 0);
    palWritePad(GPIOD,  3, 0);
    palWritePad(GPIOD,  4, 0);
    palWritePad(GPIOD,  5, 0);
    palWritePad(GPIOD,  6, 0);
#endif
    exception_init();

    InitPatch0();

    InitUsbAudio();

    InitPConnection();


    chThdSleepMilliseconds(10);

    /* Pull up SPILINK detector (HIGH means MASTER i.e. regular operation) */
    palSetPadMode(SPILINK_JUMPER_PORT, SPILINK_JUMPER_PIN, PAL_MODE_INPUT_PULLUP);

    axoloti_board_init();
    adc_init();
    adc_convert();
    axoloti_math_init();
    midi_init();
    start_dsp_thread();
    ui_init();
    configSDRAM();
    // memTest();


#if TEST_SRAM3
    // set pattern in sram3
    uint8_t uVal = 0;
    uint32_t u;
    for(u = 0; u < SRAM3_BYTE_SIZE; u++)
        sram3Buffer[u] = uVal++;
#endif

    bool_t is_master = palReadPad(SPILINK_JUMPER_PORT, SPILINK_JUMPER_PIN);

    codec_init(is_master);
#ifdef FW_SPILINK
    spilink_init(is_master);
#endif

    if (!palReadPad(SW2_PORT, SW2_PIN)) {
        /* button S2 not pressed */
        // watchdog_init();
        chThdSleepMilliseconds(1);
    }

#ifdef AXOLOTI_CONTROL
    axoloti_control_init();
#endif

    MY_USBH_Init();

    if (!exception_check()) {
        /* Only try mounting SD and booting a patch when no exception is reported */

        sdcard_attemptMountIfUnmounted();

        /* Patch start can be skipped by holding S2 during boot */
        if (!palReadPad(SW2_PORT, SW2_PIN)) {

            if (fs_ready) {
                LoadPatchStartSD();
                chThdSleepMilliseconds(100);
            }

            /* If no patch booting or running yet try loading from flash */
            // if (patchStatus == STOPPED) {
            if (patchStatus != RUNNING) {
                LoadPatchStartFlash();
            }
        }
    }

#if TEST_RAM_SPEED
    chprintf((BaseSequentialStream * )&SD2,"Testing ccram read speed\r\n");
    DWT->CYCCNT = 0;
    volatile uint16_t u1 = TestRamSpeed(ccData);
    chprintf((BaseSequentialStream * )&SD2," took %lu cycles\r\n", DWT->CYCCNT);

    chprintf((BaseSequentialStream * )&SD2,"Testing sram1 read speed\r\n");
    DWT->CYCCNT = 0;
    volatile uint16_t u2 = TestRamSpeed(s1Data);
    chprintf((BaseSequentialStream * )&SD2," took %lu cycles\r\n", DWT->CYCCNT);

    chprintf((BaseSequentialStream * )&SD2,"Testing sram3 read speed\r\n");
    DWT->CYCCNT = 0;
    volatile uint16_t u3 = TestRamSpeed(s3Data);
    chprintf((BaseSequentialStream * )&SD2," took %lu cycles\r\n", DWT->CYCCNT);

#endif

#if TEST_SRAM3
    // every 2 secs check sram3
    uint32_t uCheckSec = 2;

    while (1) {
        chThdSleepMilliseconds(1000);
        if(uCheckSec == 0)
        {
            uVal = 0;
            bool bOk = true;
            for(u = 0; bOk && (u < SRAM3_BYTE_SIZE); u++)
                bOk == (sram3Buffer[u] != uVal++);

            if(bOk)
                chprintf((BaseSequentialStream * )&SD2,"SRAM3 is OK\r\n");
            else
                chprintf((BaseSequentialStream * )&SD2,"SRAM3 is BAD\r\n");

            uCheckSec = 2;
        }
        else
            uCheckSec--;
    }
#else
    EventListener audioEventListener;
    chEvtRegisterMask(&ADU1.event, &audioEventListener, AUDIO_EVENT);

    while (1) 
    {
        chEvtWaitOne(AUDIO_EVENT);
        uint32_t  evt = chEvtGetAndClearFlags(&audioEventListener);

        if(evt & AUDIO_EVENT_USB_STATE)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB State changed.\r\n");
        else if(evt & AUDIO_EVENT_MUTE)
            chprintf((BaseSequentialStream * )&SD2,"Audio mute changed.\r\n");
        else if(evt & AUDIO_EVENT_VOLUME)
            chprintf((BaseSequentialStream * )&SD2,"Audio volume changed.\r\n");
        else if(evt & AUDIO_EVENT_INPUT)
            chprintf((BaseSequentialStream * )&SD2,"Audio input state changed = %u\r\n", aduState.isInputActive);
        else if(evt & AUDIO_EVENT_OUTPUT)
            chprintf((BaseSequentialStream * )&SD2,"Audio output state changed = %u\r\n", aduState.isOutputActive);
        else if(evt & AUDIO_EVENT_FORMAT)
            chprintf((BaseSequentialStream * )&SD2,"Audio Format type changed = %u\r\n", aduState.currentSampleRate);

        //chThdSleepMilliseconds(1000);
    }
#endif
}


void HAL_Delay(unsigned int n) {
    chThdSleepMilliseconds(n);
}


void _sbrk(void) {
    while (1);
}
