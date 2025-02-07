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

// #include "sdram.h"
// #include "stm32f4xx_fmc.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "string.h"
#include <stdio.h>

#define TEST_FLASH 0

#if TEST_FLASH
#include "stm32h7xx_hal.h"
#endif

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

//#include "sdram.c"
//#include "stm32f4xx_fmc.c"
#include "analyse.h"

#include "board.h"

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/
#if TEST_FLASH

  #define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
  #define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

  /* Base address of the Flash sectors Bank 1 */
  #define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

  /* Base address of the Flash sectors Bank 2 */
  #define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

  #define FLASH_BANK2_START_ADDR   ADDR_FLASH_SECTOR_0_BANK2        /* Start @ of user Flash area Bank2 */
  #define FLASH_BANK2_END_ADDR     (ADDR_FLASH_SECTOR_1_BANK2 - 1)  /* End @ of user Flash area Bank2*/

  uint32_t GetSector(uint32_t Address)
  {
    uint32_t sector = 0;
    
    if(((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
      ((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2)))    
    {
      sector = FLASH_SECTOR_0;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
            ((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2)))    
    {
      sector = FLASH_SECTOR_1;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
            ((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2)))    
    {
      sector = FLASH_SECTOR_2;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
            ((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2)))    
    {
      sector = FLASH_SECTOR_3;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
            ((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2)))    
    {
      sector = FLASH_SECTOR_4;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
            ((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2)))    
    {
      sector = FLASH_SECTOR_5;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
            ((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2)))    
    {
      sector = FLASH_SECTOR_6;  
    }
    else if(((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
            ((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2)))
    {
      sector = FLASH_SECTOR_7;  
    }
    else
    {
      sector = FLASH_SECTOR_7;
    }

    return sector;
  }
#endif

#define AHB1_EN_MASK    STM32_GPIO_EN_MASK
#define AHB1_LPEN_MASK  AHB1_EN_MASK

/**
 * @brief   GPIO port setup info.
 */
typedef struct {
  /** Initial value for MODER register.*/
  uint32_t              moder;
  /** Initial value for OTYPER register.*/
  uint32_t              otyper;
  /** Initial value for OSPEEDR register.*/
  uint32_t              ospeedr;
  /** Initial value for PUPDR register.*/
  uint32_t              pupdr;
  /** Initial value for ODR register.*/
  uint32_t              odr;
  /** Initial value for AFRL register.*/
  uint32_t              afrl;
  /** Initial value for AFRH register.*/
  uint32_t              afrh;
} stm32_gpio_setup_t;

/**
 * @brief   STM32 GPIO static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialize the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 */
typedef struct {
#if STM32_HAS_GPIOA || defined(__DOXYGEN__)
  /** @brief Port A setup data.*/
  stm32_gpio_setup_t    PAData;
#endif
#if STM32_HAS_GPIOB || defined(__DOXYGEN__)
  /** @brief Port B setup data.*/
  stm32_gpio_setup_t    PBData;
#endif
#if STM32_HAS_GPIOC || defined(__DOXYGEN__)
  /** @brief Port C setup data.*/
  stm32_gpio_setup_t    PCData;
#endif
#if STM32_HAS_GPIOD || defined(__DOXYGEN__)
  /** @brief Port D setup data.*/
  stm32_gpio_setup_t    PDData;
#endif
#if STM32_HAS_GPIOE || defined(__DOXYGEN__)
  /** @brief Port E setup data.*/
  stm32_gpio_setup_t    PEData;
#endif
#if STM32_HAS_GPIOF || defined(__DOXYGEN__)
  /** @brief Port F setup data.*/
  stm32_gpio_setup_t    PFData;
#endif
#if STM32_HAS_GPIOG || defined(__DOXYGEN__)
  /** @brief Port G setup data.*/
  stm32_gpio_setup_t    PGData;
#endif
#if STM32_HAS_GPIOH || defined(__DOXYGEN__)
  /** @brief Port H setup data.*/
  stm32_gpio_setup_t    PHData;
#endif
#if STM32_HAS_GPIOI || defined(__DOXYGEN__)
  /** @brief Port I setup data.*/
  stm32_gpio_setup_t    PIData;
#endif
#if STM32_HAS_GPIOJ || defined(__DOXYGEN__)
  /** @brief Port I setup data.*/
  stm32_gpio_setup_t    PJData;
#endif
#if STM32_HAS_GPIOK || defined(__DOXYGEN__)
  /** @brief Port I setup data.*/
  stm32_gpio_setup_t    PKData;
#endif
} PALConfig;

#if TESTME
//shit
#endif

const PALConfig pal_default_config = {
    {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
     VAL_GPIOA_ODR, VAL_GPIOA_AFRL, VAL_GPIOA_AFRH},
    {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
     VAL_GPIOB_ODR, VAL_GPIOB_AFRL, VAL_GPIOB_AFRH},
    {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
     VAL_GPIOC_ODR, VAL_GPIOC_AFRL, VAL_GPIOC_AFRH},
    {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
     VAL_GPIOD_ODR, VAL_GPIOD_AFRL, VAL_GPIOD_AFRH},
    {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
     VAL_GPIOE_ODR, VAL_GPIOE_AFRL, VAL_GPIOE_AFRH},
    {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
     VAL_GPIOF_ODR, VAL_GPIOF_AFRL, VAL_GPIOF_AFRH},
    {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
     VAL_GPIOG_ODR, VAL_GPIOG_AFRL, VAL_GPIOG_AFRH},
    {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
     VAL_GPIOH_ODR, VAL_GPIOH_AFRL, VAL_GPIOH_AFRH}};
    // {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
    //  VAL_GPIOI_ODR, VAL_GPIOI_AFRL, VAL_GPIOI_AFRH}};


static void initgpio(stm32_gpio_t *gpiop, const stm32_gpio_setup_t *config) {

  gpiop->OTYPER  = config->otyper;
  gpiop->OSPEEDR = config->ospeedr;
  gpiop->PUPDR   = config->pupdr;
  gpiop->ODR     = config->odr;
  gpiop->AFRL    = config->afrl;
  gpiop->AFRH    = config->afrh;
  gpiop->MODER   = config->moder;
}

void myPalInit(const PALConfig *config) {

  /*
   * Enables the GPIO related clocks.
   */
#if defined(STM32L0XX)
  RCC->IOPENR |= AHB_EN_MASK;
  RCC->IOPSMENR |= AHB_LPEN_MASK;
#elif defined(STM32L1XX)
  rccEnableAHB(AHB_EN_MASK, TRUE);
  RCC->AHBLPENR |= AHB_LPEN_MASK;
#elif defined(STM32F0XX)
  rccEnableAHB(AHB_EN_MASK, TRUE);
#elif defined(STM32F3XX) || defined(STM32F37X)
  rccEnableAHB(AHB_EN_MASK, TRUE);
#elif defined(STM32F2XX) || defined(STM32F4XX) || defined(STM32F7XX)
  RCC->AHB1ENR   |= AHB1_EN_MASK;
  RCC->AHB1LPENR |= AHB1_LPEN_MASK;
#endif

  /*
   * Initial GPIO setup.
   */
#if STM32_HAS_GPIOA
  initgpio(GPIOA, &config->PAData);
#endif
#if STM32_HAS_GPIOB
  initgpio(GPIOB, &config->PBData);
#endif
#if STM32_HAS_GPIOC
  initgpio(GPIOC, &config->PCData);
#endif
#if STM32_HAS_GPIOD
  initgpio(GPIOD, &config->PDData);
#endif
#if STM32_HAS_GPIOE
  initgpio(GPIOE, &config->PEData);
#endif
#if STM32_HAS_GPIOF
  initgpio(GPIOF, &config->PFData);
#endif
#if STM32_HAS_GPIOG
  initgpio(GPIOG, &config->PGData);
#endif
#if STM32_HAS_GPIOH
  initgpio(GPIOH, &config->PHData);
#endif
#if STM32_HAS_GPIOI
  initgpio(GPIOI, &config->PIData);
#endif
#if STM32_HAS_GPIOJ
  initgpio(GPIOJ, &config->PJData);
#endif
#if STM32_HAS_GPIOK
  initgpio(GPIOK, &config->PKData);
#endif
}

extern void MY_USBH_Init(void);

int32buffer AudioInputLeft, AudioInputRight, AudioOutputLeft, AudioOutputRight, 
            UsbInputLeft, UsbInputRight, UsbOutputLeft, UsbOutputRight,
            UsbInput2Left, UsbInput2Right, UsbOutput2Left, UsbOutput2Right;

/*
 * GPT4 callback.
 */
static void gpt4cb(GPTDriver *gptp)
{
  (void)gptp;
  Analyse(GPIOB, 8, 1); 
  static bool b = 0;

  if (b)
  {
    computebufI(rbuf2, buf);
  }
  else
  {
    computebufI(rbuf, buf2);
  }
  b = !b;
  Analyse(GPIOB, 8, 0); 

  // chSysLockFromISR();
  // computebufI(rbuf, buf2);
  // chSysUnlockFromISR();
}

// 120Mhz timer clock
#define GPT4_FREQ 120000000
// need 3.33 khz clock
#define GPT4_TICKS 40000
/*
 * GPT4 configuration.
 */
static const GPTConfig gpt4cfg = 
{
  GPT4_FREQ, 
  gpt4cb, /* Timer callback.*/
  0,
  0
};


uint32_t HAL_GetTick(void) {
    return RTT2MS(hal_lld_get_counter_value());
}


uint32_t TestFlash(void)
{
  volatile uint32_t uResult = 0;

  #if TEST_FLASH
    volatile bool bTestFlash = false;

    if(bTestFlash)
    {
      // test flash
      HAL_FLASH_Unlock();
      SCB_DisableICache();

      /* Get the 1st sector to erase */
      uint32_t uFirstSector = GetSector(FLASH_BANK2_START_ADDR);
      /* Get the number of sector to erase from 1st sector*/
      uint32_t uNbOfSectors = GetSector(FLASH_BANK2_END_ADDR) - uFirstSector + 1;

      FLASH_EraseInitTypeDef EraseInitStruct;

      /* Fill EraseInit structure*/
      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
      EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
      EraseInitStruct.Banks         = FLASH_BANK_2;
      EraseInitStruct.Sector        = uFirstSector;
      EraseInitStruct.NbSectors     = uNbOfSectors;
      
      uint32_t uSectorError;
      volatile bool bErased = false;
      volatile bool bError = false;

      if (HAL_FLASHEx_Erase(&EraseInitStruct, &uSectorError) == HAL_OK)
      {
        bErased = true;

        uint64_t FlashWord[4] = { 0x0102030405060708,
                            0x1112131415161718,
                            0x2122232425262728,    
                            0x3132333435363738
                          };

        uint32_t uAddress = FLASH_BANK2_START_ADDR;

        while (!bError && (uAddress < FLASH_BANK2_END_ADDR))
        {
          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, uAddress, ((uint32_t)FlashWord)) == HAL_OK)
          {
            uAddress = uAddress + 32; /* increment for the next Flash word*/
          }
          else
          {
            bError = true;
          }
        }
      }

      HAL_FLASH_Lock();
      SCB_EnableICache();
      uResult = !bError;
    }
    else
    {
      // Test HAL_tick
      volatile uint32_t uTickStart = HAL_GetTick();
      chThdSleepMilliseconds(1000);
      volatile uint32_t uTickEnd = HAL_GetTick();
      uResult =  uTickEnd - uTickStart;
    }
#endif
    return uResult;
}

int main(void) {
    /* copy vector table to SRAM1! */
    // vectors are at 0x0000000008000000 and size 0x2a0
    // why does this code remap SRAM1 to 0?
    // itcmram is at 0 on h7
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
//    memcpy((char *)0x20000000, (const char)0x00000000, 0x200);
#pragma GCC diagnostic pop

    /* remap SRAM1 to 0x00000000 */
//TODO??    SYSCFG->MEMRMP |= 0x03;

    halInit();
    myPalInit(&pal_default_config);
    chSysInit();

    volatile uint32_t uResult = TestFlash();

    gptStart(&GPTD4, &gpt4cfg);
    gptPolledDelay(&GPTD4, 10); /* Small delay.*/
   
      // test code at 0x30000000/1
    // patchMeta.fptr_patch_init = (fptr_patch_init_t)(PATCHMAINLOC);
    // (patchMeta.fptr_patch_init)(1234);

    // load  /Users/andrewcapon/ksoloti/build/xpatch.elf 0
    // load /Users/andrewcapon/ksoloti/build/xpatch.bin 0x30000000
//     typedef void func(uint32_t);
//     func* f1 = shit;
//     func* f = (func*)0x30000001; // add 1 for thumb
// chSysLock();    
//     f1(1234);
//     f(1234);
// chSysUnlock();

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
#endif

#if ANALYSE_ENABLE
    // 755 values
    palSetPadMode(GPIOB,  8, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  9, PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode(GPIOB,  7, PAL_MODE_OUTPUT_PUSHPULL); 
    Analyse(GPIOB, 8, 1); 
    Analyse(GPIOB, 9, 1); 
    Analyse(GPIOB, 7, 1); 
    Analyse(GPIOB, 8, 0); 
    Analyse(GPIOB, 9, 0); 
    Analyse(GPIOB, 7, 0); 

    // palSetPadMode(GPIOG, 11, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOG, 10, PAL_MODE_OUTPUT_PUSHPULL); 

    // palSetPadMode(GPIOD,  3, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOD,  4, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOD,  5, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOD,  6, PAL_MODE_OUTPUT_PUSHPULL); 

    // palSetPadMode(GPIOA,  9, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOB,  9, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOB,  8, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOB,  7, PAL_MODE_OUTPUT_PUSHPULL); 

    // palSetPadMode(GPIOB,  6, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOB,  4, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOB,  3, PAL_MODE_OUTPUT_PUSHPULL); 
    // palSetPadMode(GPIOC,  7, PAL_MODE_OUTPUT_PUSHPULL); 

    // Analyse(GPIOG, 11, 0);
    // Analyse(GPIOG, 10, 0);

    // Analyse(GPIOD, 3, 0);
    // Analyse(GPIOD, 4, 0);
    // Analyse(GPIOD, 5, 0);
    // Analyse(GPIOD, 6, 0);

    // Analyse(GPIOA, 9, 0); 
    // Analyse(GPIOB, 9, 0); 
    // Analyse(GPIOB, 8, 0); 
    // Analyse(GPIOB, 7, 0); 

    // Analyse(GPIOB, 6, 0); 
    // Analyse(GPIOB, 4, 0); 
    // Analyse(GPIOB, 3, 0); 
    // Analyse(GPIOC, 7, 0); 
#endif
    //exception_init();

    InitPatch0();

#if FW_USBAUDIO
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
    // configSDRAM();
    // memTest();

    bool_t is_master = palReadPad(SPILINK_JUMPER_PORT, SPILINK_JUMPER_PIN);

    codec_init(is_master);
    gptStartContinuous(&GPTD4, GPT4_TICKS);
    chThdSleepMilliseconds(10);
    uint32_t uCount = gptGetCounterX(&GPTD4);

#ifdef FW_SPILINK
    spilink_init(is_master);
#endif

    if (!palReadPad(SW2_PORT, SW2_PIN)) {
        /* button S2 not pressed */
        // watchdog_init();
        chThdSleepMilliseconds(1);
    }

    // Test patch load from flash
    if (patchStatus != RUNNING) 
      LoadPatchStartFlash();
  
    //MY_USBH_Init(); 


    // if (!exception_check()) {
    //     /* Only try mounting SD and booting a patch when no exception is reported */

    //     sdcard_attemptMountIfUnmounted();

    //     /* Patch start can be skipped by holding S2 during boot */
    //     if (!palReadPad(SW2_PORT, SW2_PIN)) {

    //         if (fs_ready) {
    //             LoadPatchStartSD();
    //             chThdSleepMilliseconds(100);
    //         }

    //         /* If no patch booting or running yet try loading from flash */
    //         // if (patchStatus == STOPPED) {
    //         if (patchStatus != RUNNING) {
    //             LoadPatchStartFlash();
    //         }
    //     }
    // }

    //TestMemset();


#if FW_USBAUDIO
    EventListener audioEventListener;
    chEvtRegisterMask(&ADU1.event, &audioEventListener, AUDIO_EVENT);

    while (1) 
    {
        chEvtWaitOne(AUDIO_EVENT);
        uint32_t  evt = chEvtGetAndClearFlags(&audioEventListener);

        if(evt & AUDIO_EVENT_USB_CONGIGURED)
            LogTextMessage("Audio USB Configured.\r\n");
        else if(evt & AUDIO_EVENT_USB_SUSPEND)
            LogTextMessage("Audio USB Suspend\r\n");
        else if(evt & AUDIO_EVENT_USB_WAKEUP)
            LogTextMessage("Audio USB Wakeup.\r\n");
        else if(evt & AUDIO_EVENT_USB_STALLED)
            LogTextMessage("Audio USB Stalled.\r\n");
        else if(evt & AUDIO_EVENT_USB_RESET)
            LogTextMessage("Audio USB Reset.\r\n");
        else if(evt & AUDIO_EVENT_USB_ENABLE)
            LogTextMessage("Audio USB Enable.\r\n");
        else if(evt & AUDIO_EVENT_MUTE)
            LogTextMessage("Audio mute changed.\r\n");
        else if(evt & AUDIO_EVENT_VOLUME)
            LogTextMessage("Audio volume changed.\r\n");
        else if(evt & AUDIO_EVENT_INPUT)
            LogTextMessage("Audio input state changed = %u\r\n", aduState.isInputActive);
        else if(evt & AUDIO_EVENT_OUTPUT)
            LogTextMessage("Audio output state changed = %u\r\n", aduState.isOutputActive);
        else if(evt & AUDIO_EVENT_FORMAT)
            LogTextMessage("Audio Format type changed = %u\r\n", aduState.currentSampleRate);

        connectionFlags.usbActive = aduIsUsbInUse();
        LogTextMessage("connectionFlags.usbActive = %u\r\n", connectionFlags.usbActive );
    }
#else
    while (1) {
        chThdSleepMilliseconds(1000);
    }
#endif
}


void HAL_Delay(uint32_t n) {
    chThdSleepMilliseconds(n);
}


void _sbrk(void) {
    while (1);
}
