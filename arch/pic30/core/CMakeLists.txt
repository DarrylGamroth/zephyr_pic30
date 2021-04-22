# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  fatal.c
  irq_manage.c
  prep_c.c
  reset.S
  reboot.c
  switch.S
  thread.c
  vector_table.S
  cpu_idle.S
)

zephyr_library_sources_ifdef(CONFIG_PIC30_EDS data_init_eds.S)
zephyr_library_sources_ifndef(CONFIG_PIC30_EDS data_init.S)

zephyr_library_sources_ifdef(CONFIG_GEN_SW_ISR_TABLE isr_wrapper.S)
zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)

zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors vector_table.ld)
zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0x0icd icd.ld)
