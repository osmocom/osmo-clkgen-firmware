/*
 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdbool.h>
#include "si5351c.h"

extern bool i2c_tx_start(uint32_t addr);
extern bool i2c_tx_byte(uint8_t byte);
extern bool i2c_rx_byte(uint8_t *byte, bool last);
extern bool i2c_stop(void);

enum pll_sources active_clock_source;

/* write to single register */
void si5351c_write_single(uint8_t reg, uint8_t val)
{
  i2c_tx_start(SI5351C_I2C_ADDR);
  i2c_tx_byte(reg);
  i2c_tx_byte(val);
  i2c_stop();
}

/* read single register */
uint8_t si5351c_read_single(uint8_t reg)
{
  uint8_t val;

  /* set register address with write */
  i2c_tx_start(SI5351C_I2C_ADDR);
  i2c_tx_byte(reg);

  /* read the value */
  i2c_tx_start(SI5351C_I2C_ADDR | 1);
  i2c_rx_byte(&val, true);
  i2c_stop();

  return val;
}

/*
 * Write to one or more contiguous registers. data[0] should be the first
 * register number, one or more values follow.
 */
void si5351c_write(uint8_t* const data, const uint_fast8_t data_count)
{
  uint_fast8_t i;

  i2c_tx_start(SI5351C_I2C_ADDR);

  for (i = 0; i < data_count; i++)
    i2c_tx_byte(data[i]);
  i2c_stop();
}

/* Turn off OEB pin control for all CLKx */
void si5351c_disable_oeb_pin_control()
{
  uint8_t data[] = { 9, 0xFF };
  si5351c_write(data, sizeof(data));
}

/* Power down all CLKx */
void si5351c_power_down_all_clocks()
{
  uint8_t data[] = {
    16
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
    , SI5351C_CLK_POWERDOWN
  };
  si5351c_write(data, sizeof(data));
}

/*
 * Register 183: Crystal Internal Load Capacitance
 * Reads as 0xE4 on power-up
 * Set to 8pF based on crystal specs and HackRF One testing
 */
void si5351c_set_crystal_configuration()
{
  uint8_t data[] = { 183, 0x80 };
  si5351c_write(data, sizeof(data));
}

/*
 * Register 187: Fanout Enable
 * Turn on XO and MultiSynth fanout only.
 */
void si5351c_enable_xo_and_ms_fanout()
{
  uint8_t data[] = { 187, 0xD0 };
  si5351c_write(data, sizeof(data));
}

/*
 * Register 15: PLL Input Source
 * CLKIN_DIV=0 (Divide by 1)
 * PLLA_SRC=0 (XTAL)
 * PLLB_SRC=1 (CLKIN)
 */
void si5351c_configure_pll_sources(void)
{
  uint8_t data[] = { 15, 0x08 };
  si5351c_write(data, sizeof(data));
}

/* MultiSynth NA (PLLA) and NB (PLLB) */
void si5351c_configure_pll_multisynth(void)
{
  /* init plla to (0x0e00+512)/128*25MHz xtal = 800MHz -> int mode */
  uint8_t data[] = { 26, 0x00, 0x01, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00 };
  si5351c_write(data, sizeof(data));

  /* 10 MHz input on CLKIN for PLLB */
  data[0] = 34;
  data[4] = 0x26;
  si5351c_write(data, sizeof(data));
}

void si5351c_reset_pll(void)
{
  /* reset PLLA and PLLB */
  uint8_t data[] = { 177, 0xAC };
  si5351c_write(data, sizeof(data));
}

void si5351c_configure_multisynth(const uint_fast8_t ms_number,
                                  const uint32_t p1, const uint32_t p2, const uint32_t p3,
                                  const uint_fast8_t r_div)
{
  /*
   * TODO: Check for p3 > 0? 0 has no meaning in fractional mode?
   * And it makes for more jitter in integer mode.
   */
  /*
   * r_div is the R divider value encoded:
   *   0 means divide by 1
   *   1 means divide by 2
   *   2 means divide by 4
   *   ...
   *   7 means divide by 128
   */
  const uint_fast8_t register_number = 42 + (ms_number * 8);
  uint8_t data[] = {
    register_number,
    (p3 >> 8) & 0xFF,
    (p3 >> 0) & 0xFF,
    (r_div << 4) | (0 << 2) | ((p1 >> 16) & 0x3),
    (p1 >> 8) & 0xFF,
    (p1 >> 0) & 0xFF,
    (((p3 >> 16) & 0xF) << 4) | (((p2 >> 16) & 0xF) << 0),
    (p2 >> 8) & 0xFF,
    (p2 >> 0) & 0xFF
  };

  si5351c_write(data, sizeof(data));
}

void si5351c_configure_clock_control(const enum pll_sources source)
{
  uint8_t pll;

  if (source == PLL_SOURCE_CLKIN) {
    /* PLLB on CLKIN */
    pll = SI5351C_CLK_PLL_SRC_B;
  } else {
    /* PLLA on XTAL */
    pll = SI5351C_CLK_PLL_SRC_A;
  }

  uint8_t data[] = {
    16
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
    , SI5351C_CLK_INT_MODE  | SI5351C_CLK_PLL_SRC(pll) | SI5351C_CLK_SRC(SI5351C_CLK_SRC_MULTISYNTH_SELF) | SI5351C_CLK_IDRV(SI5351C_CLK_IDRV_2MA)
  };
  si5351c_write(data, sizeof(data));
}

void si5351c_enable_clock_outputs(const uint_fast8_t mask)
{
  uint8_t data[] = { 3, ~mask };
  si5351c_write(data, sizeof(data));
}
#if 0
void si5351c_set_int_mode(const uint_fast8_t ms_number, const uint_fast8_t on)
{
  uint8_t data[] = {16, 0};

  if(ms_number < 8){
    data[0] = 16 + ms_number;
    data[1] = si5351c_read_single(data[0]);

    if(on)
      data[1] |= SI5351C_CLK_INT_MODE;
    else
      data[1] &= ~(SI5351C_CLK_INT_MODE);

    si5351c_write(data, 2);
  }
}
#endif
void si5351c_set_clock_source(const enum pll_sources source)
{
  si5351c_configure_clock_control(source);
  active_clock_source = source;
}

enum pll_sources si5351c_activate_best_clock_source(void)
{
  uint8_t device_status = si5351c_read_single(0);

  if (device_status & SI5351C_LOS) {
    /* CLKIN not detected */
    if (active_clock_source == PLL_SOURCE_CLKIN) {
      si5351c_set_clock_source(PLL_SOURCE_XTAL);
    }
  } else {
    /* CLKIN detected */
    if (active_clock_source == PLL_SOURCE_XTAL) {
      si5351c_set_clock_source(PLL_SOURCE_CLKIN);
    }
  }

  return active_clock_source;
}
