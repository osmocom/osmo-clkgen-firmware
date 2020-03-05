#include <stdbool.h>
#include "si5351c.h"

void clkgen_init(void)
{
  si5351c_enable_clock_outputs(0x00); /* Disable Outputs */

  si5351c_disable_oeb_pin_control();
  si5351c_power_down_all_clocks(); /* Powerdown all output drivers */
  si5351c_set_crystal_configuration();
  si5351c_enable_xo_and_ms_fanout();
  si5351c_configure_pll_sources();
  si5351c_configure_pll_multisynth();

  /* MS0/CLK0 */
  si5351c_configure_multisynth(0, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS1/CLK1 */
  si5351c_configure_multisynth(1, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS2/CLK2 */
  si5351c_configure_multisynth(2, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS3/CLK3 */
  si5351c_configure_multisynth(3, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS4/CLK4 */
  si5351c_configure_multisynth(4, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS5/CLK5 */
  si5351c_configure_multisynth(5, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS6/CLK6 */
  si5351c_configure_multisynth(6, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */
  /* MS7/CLK7 */
  si5351c_configure_multisynth(7, 80*128-512, 0, 1, 0); /* 800/80 = 10MHz */

  si5351c_set_clock_source(PLL_SOURCE_XTAL);
  si5351c_activate_best_clock_source();

  si5351c_reset_pll(); /* Apply PLLA and PLLB soft reset */

  si5351c_enable_clock_outputs(0xff); /* Enable desired outputs */
}

bool clkgen_run(void)
{
  bool externally_locked = false;

  enum pll_sources clk_src = si5351c_activate_best_clock_source();

  if ( clk_src == PLL_SOURCE_CLKIN )
    externally_locked = true;

  return externally_locked;
}
