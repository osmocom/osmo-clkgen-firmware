#!/bin/sh

openocd -f openocd.cfg -c "program ../make/build/osmo-clkgen.bin 0 verify; reset; exit"

