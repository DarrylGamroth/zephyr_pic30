#
#  Copyright (c) 2018 Analog Devices Inc.
#
#  SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADXL355 adxl355.c)
zephyr_library_sources_ifdef(CONFIG_ADXL355_TRIGGER adxl355_trigger.c)
