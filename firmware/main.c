/*
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
#include "axoloti_math.h"
#include "axoloti_board.h"
#include "exceptions.h"
#include "watchdog.h"

#include "usbcfg.h"
#include "sysmon.h"
#ifdef FW_SPILINK
#include "spilink.h"
#endif

#include "sdram.c"
#include "stm32f4xx_fmc.c"
#include "analyse.h"

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

#define ENABLE_SERIAL_DEBUG 1


extern void MY_USBH_Init(void);

volatile int32buffer AudioInputLeft, AudioInputRight, AudioOutputLeft, AudioOutputRight, UsbInputLeft, UsbInputRight, UsbOutputLeft, UsbOutputRight;

// o1 - 11.15us, 1.35us, 2.05us, 1.1us, 0.50us
// o2 - 11.14us, 1.29us, 2.64us, 1.1us, 0.42us 
// o3 - 11.07us, 0.43us, 0.46us, 11.06us, 0.45us

void __attribute__((optimize("O3"))) TestMemset(void)
{
    uint_fast8_t u;

    Analyse(GPIOG, 10, 1);
    memset(AudioOutputLeft, 0, sizeof(AudioOutputLeft));
    memset(AudioOutputRight, 0, sizeof(AudioOutputRight));
    memset(UsbOutputLeft, 0, sizeof(UsbOutputLeft));
    memset(UsbOutputRight, 0, sizeof(UsbOutputRight));
    Analyse(GPIOG, 10, 0);

    Analyse(GPIOG, 10, 1);
    for(u=0; u < BUFSIZE; u++)
    {
        AudioOutputLeft[u] = 0;
        AudioOutputRight[u] = 0;
        UsbOutputLeft[u] = 0;
        UsbOutputRight[u] = 0;
    }
    Analyse(GPIOG, 10, 0);

    Analyse(GPIOG, 10, 1);
    for(u=0; u < BUFSIZE; u++)
        AudioOutputLeft[u] = 0;

    for(u=0; u < BUFSIZE; u++)
        AudioOutputRight[u] = 0;

    for(u=0; u < BUFSIZE; u++)
        UsbOutputLeft[u] = 0;

    for(u=0; u < BUFSIZE; u++)
        UsbOutputRight[u] = 0;
    Analyse(GPIOG, 10, 0);

    Analyse(GPIOG, 10, 1);
    uint64_t *p1 = (uint32_t *)AudioOutputLeft;
    uint64_t *p2 = (uint32_t *)AudioOutputRight;
    uint64_t *p3 = (uint32_t *)UsbOutputLeft;
    uint64_t *p4 = (uint32_t *)UsbOutputRight;
    uint_fast8_t uLen = BUFSIZE >> 1;
    for(u=0;u <(BUFSIZE>>1);u++)
        *p1++ = *p2++ = *p3++ = *p4++ = 0;
    Analyse(GPIOG, 10, 0);

    Analyse(GPIOG, 10, 1);
    AudioOutputLeft[0] = 0;
    AudioOutputLeft[1] = 0;
    AudioOutputLeft[2] = 0;
    AudioOutputLeft[3] = 0;
    AudioOutputLeft[4] = 0;
    AudioOutputLeft[5] = 0;
    AudioOutputLeft[6] = 0;
    AudioOutputLeft[7] = 0;
    AudioOutputLeft[8] = 0;
    AudioOutputLeft[9] = 0;
    AudioOutputLeft[10] = 0;
    AudioOutputLeft[11] = 0;
    AudioOutputLeft[12] = 0;
    AudioOutputLeft[13] = 0;
    AudioOutputLeft[14] = 0;
    AudioOutputLeft[15] = 0;

    AudioOutputRight[0] = 0;
    AudioOutputRight[1] = 0;
    AudioOutputRight[2] = 0;
    AudioOutputRight[3] = 0;
    AudioOutputRight[4] = 0;
    AudioOutputRight[5] = 0;
    AudioOutputRight[6] = 0;
    AudioOutputRight[7] = 0;
    AudioOutputRight[8] = 0;
    AudioOutputRight[9] = 0;
    AudioOutputRight[10] = 0;
    AudioOutputRight[11] = 0;
    AudioOutputRight[12] = 0;
    AudioOutputRight[13] = 0;
    AudioOutputRight[14] = 0;
    AudioOutputRight[15] = 0;

    UsbOutputLeft[0] = 0;
    UsbOutputLeft[1] = 0;
    UsbOutputLeft[2] = 0;
    UsbOutputLeft[3] = 0;
    UsbOutputLeft[4] = 0;
    UsbOutputLeft[5] = 0;
    UsbOutputLeft[6] = 0;
    UsbOutputLeft[7] = 0;
    UsbOutputLeft[8] = 0;
    UsbOutputLeft[9] = 0;
    UsbOutputLeft[10] = 0;
    UsbOutputLeft[11] = 0;
    UsbOutputLeft[12] = 0;
    UsbOutputLeft[13] = 0;
    UsbOutputLeft[14] = 0;
    UsbOutputLeft[15] = 0;

    UsbOutputRight[0] = 0;
    UsbOutputRight[1] = 0;
    UsbOutputRight[2] = 0;
    UsbOutputRight[3] = 0;
    UsbOutputRight[4] = 0;
    UsbOutputRight[5] = 0;
    UsbOutputRight[6] = 0;
    UsbOutputRight[7] = 0;
    UsbOutputRight[8] = 0;
    UsbOutputRight[9] = 0;
    UsbOutputRight[10] = 0;
    UsbOutputRight[11] = 0;
    UsbOutputRight[12] = 0;
    UsbOutputRight[13] = 0;
    UsbOutputRight[14] = 0;
    UsbOutputRight[15] = 0;

    Analyse(GPIOG, 10, 0);

}

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
    chprintf((BaseSequentialStream * )&SD2,"Hello world!\r\n");
#endif

#if ANALYSE_ENABLE
    palSetPadMode(GPIOG, 11, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOG, 10, PAL_MODE_OUTPUT_PUSHPULL); 

    palSetPadMode(GPIOD,  3, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  4, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  5, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOD,  6, PAL_MODE_OUTPUT_PUSHPULL); 

    palSetPadMode(GPIOA,  9, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  9, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  8, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  7, PAL_MODE_OUTPUT_PUSHPULL); 

    palSetPadMode(GPIOB,  6, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  4, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  3, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOC,  7, PAL_MODE_OUTPUT_PUSHPULL); 

    Analyse(GPIOG, 11, 0);
    Analyse(GPIOG, 10, 0);

    Analyse(GPIOD, 3, 0);
    Analyse(GPIOD, 4, 0);
    Analyse(GPIOD, 5, 0);
    Analyse(GPIOD, 6, 0);

    Analyse(GPIOA, 9, 0); 
    Analyse(GPIOB, 9, 0); 
    Analyse(GPIOB, 8, 0); 
    Analyse(GPIOB, 7, 0); 

    Analyse(GPIOB, 6, 0); 
    Analyse(GPIOB, 4, 0); 
    Analyse(GPIOB, 3, 0); 
    Analyse(GPIOC, 7, 0); 

#endif
    exception_init();

    InitPatch0();

#if ENABLE_USB_AUDIO
    InitUsbAudio();
#endif

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

    //TestMemset();

#if ENABLE_USB_AUDIO
    EventListener audioEventListener;
    chEvtRegisterMask(&ADU1.event, &audioEventListener, AUDIO_EVENT);

    while (1) 
    {
        chEvtWaitOne(AUDIO_EVENT);
        uint32_t  evt = chEvtGetAndClearFlags(&audioEventListener);

        if(evt & AUDIO_EVENT_USB_CONGIGURED)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB Configured.\r\n");
        else if(evt & AUDIO_EVENT_USB_SUSPEND)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB Suspend\r\n");
        else if(evt & AUDIO_EVENT_USB_WAKEUP)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB Wakeup.\r\n");
        else if(evt & AUDIO_EVENT_USB_STALLED)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB Stalled.\r\n");
        else if(evt & AUDIO_EVENT_USB_RESET)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB Reset.\r\n");
        else if(evt & AUDIO_EVENT_USB_ENABLE)
            chprintf((BaseSequentialStream * )&SD2,"Audio USB Enable.\r\n");
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


    }
#else
    while (1) {
        chThdSleepMilliseconds(1000);
    }
#endif
}


void HAL_Delay(unsigned int n) {
    chThdSleepMilliseconds(n);
}


void _sbrk(void) {
    while (1);
}
