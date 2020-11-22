# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  fatal.c
  irq_init.c
  irq_manage.c
  prep_c.c
  reset.S
  switch.S
  thread.c
  vector_table.S
  cpu_idle.S
  data_init_extendedch.S
)

zephyr_library_sources_ifdef(CONFIG_GEN_SW_ISR_TABLE isr_wrapper.S)
zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)
