/*
 * Copyright (C) 2013, 2014, 2015 Johannes Taelman
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
#include "ch.h"
#include "hal.h"
#include "exceptions.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
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
     VAL_GPIOH_ODR, VAL_GPIOH_AFRL, VAL_GPIOH_AFRH},
    {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
     VAL_GPIOI_ODR, VAL_GPIOI_AFRL, VAL_GPIOI_AFRH}};
#endif

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void)
{
    exception_check_DFU();
    stm32_clock_init();
}

void __late_init(void)
{
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool_t sdc_lld_is_card_inserted(SDCDriver *sdcp)
{
    (void)sdcp;
    /* TODO: Fill the implementation.*/
    return TRUE;
}

/**
 * @brief   SDC card write protection detection.
 */
bool_t sdc_lld_is_write_protected(SDCDriver *sdcp)
{
    (void)sdcp;
    /* TODO: Fill the implementation.*/
    return FALSE;
}
#endif /* HAL_USE_SDC */

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void)
{
}

int _getpid(void)
{
    return -1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    return -1;
}

int _write(void)
{
    return -1;
}

void _exit(int i)
{
    (void)i;
    while (1);
}
