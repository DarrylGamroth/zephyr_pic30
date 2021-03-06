/* SPDX-License-Identifier: Apache-2.0 */

	SECTION_RODATA_PROLOGUE(initlevel,,)
	{
		/*
		 * link in initialization objects for all objects that are
		 * automatically initialized by the kernel; the objects are
		 * sorted in the order they will be initialized (i.e. ordered
		 * by level, sorted by priority within a level)
		 */
		SYMBOL(__init_start) = .;
		CREATE_OBJ_LEVEL(init, PRE_KERNEL_1)
		CREATE_OBJ_LEVEL(init, PRE_KERNEL_2)
		CREATE_OBJ_LEVEL(init, POST_KERNEL)
		CREATE_OBJ_LEVEL(init, APPLICATION)
		CREATE_OBJ_LEVEL(init, SMP)
		SYMBOL(__init_end) = .;
	} GROUP_LINK_IN(ROMABLE_REGION)

#if defined(CONFIG_GEN_SW_ISR_TABLE) && !defined(CONFIG_DYNAMIC_INTERRUPTS)
	SECTION_RODATA_PROLOGUE(sw_isr_table,,)
	{
		/*
		 * Some arch requires an entry to be aligned to arch
		 * specific boundary for using double word load
		 * instruction.  See include/sw_isr_table.h.
		 */
		. = ALIGN(CONFIG_ARCH_SW_ISR_TABLE_ALIGN);
		*(_SW_ISR_TABLE_SECTION_NAME*)
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif

	/* verify we don't have rogue .init_<something> initlevel sections */
	SECTION_RODATA_PROLOGUE(initlevel_error,,)
	{
		KEEP(*(SORT(.init_[_A-Z0-9]*)))
	}
	ASSERT(SIZEOF(initlevel_error) == 0, "Undefined initialization levels used.")

#ifdef CONFIG_CPLUSPLUS
	SECTION_RODATA_PROLOGUE(_CTOR_SECTION_NAME,,)
	{
		/*
		 * The compiler fills the constructor pointers table below,
		 * hence symbol __CTOR_LIST__ must be aligned on word
		 * boundary.  To align with the C++ standard, the first elment
		 * of the array contains the number of actual constructors. The
		 * last element is NULL.
		 */
#ifdef CONFIG_64BIT
		. = ALIGN(8);
		__CTOR_LIST__ = .;
		QUAD((__CTOR_END__ - __CTOR_LIST__) / 8 - 2)
		KEEP(*(SORT_BY_NAME(".ctors*")))
		QUAD(0)
		__CTOR_END__ = .;
#else
		. = ALIGN(4);
		__CTOR_LIST__ = .;
		LONG((__CTOR_END__ - __CTOR_LIST__) / 4 - 2)
		KEEP(*(SORT_BY_NAME(".ctors*")))
		LONG(0)
		__CTOR_END__ = .;
#endif
	} GROUP_LINK_IN(ROMABLE_REGION)

	SECTION_RODATA_PROLOGUE(init_array,,)
	{
		. = ALIGN(4);
		SYMBOL(__init_array_start) = .;
		KEEP(*(SORT_BY_NAME(".init_array*")))
		SYMBOL(__init_array_end) = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif
#ifdef CONFIG_USERSPACE
	/* Build-time assignment of permissions to kernel objects to
	 * threads declared with K_THREAD_DEFINE()
	 */
	Z_ITERABLE_SECTION_ROM(z_object_assignment, 4)
#endif

	SECTION_RODATA_PROLOGUE(app_shmem_regions,,)
	{
		SYMBOL(__app_shmem_regions_start) = .;
		KEEP(*(SORT(.app_regions.*)));
		SYMBOL(__app_shmem_regions_end) = .;
	} GROUP_LINK_IN(ROMABLE_REGION)

#if defined(CONFIG_NET_SOCKETS)
	Z_ITERABLE_SECTION_ROM(net_socket_register, 4)
#endif

#if defined(CONFIG_NET_L2_PPP)
	Z_ITERABLE_SECTION_ROM(ppp_protocol_handler, 4)
#endif

	Z_ITERABLE_SECTION_ROM(bt_l2cap_fixed_chan, 4)

#if defined(CONFIG_BT_BREDR)
	Z_ITERABLE_SECTION_ROM(bt_l2cap_br_fixed_chan, 4)
#endif

	Z_ITERABLE_SECTION_ROM(bt_gatt_service_static, 4)

#if defined(CONFIG_BT_MESH)
	Z_ITERABLE_SECTION_ROM(bt_mesh_subnet_cb, 4)
	Z_ITERABLE_SECTION_ROM(bt_mesh_app_key_cb, 4)

	Z_ITERABLE_SECTION_ROM(bt_mesh_hb_cb, 4)
#endif

#if defined(CONFIG_BT_MESH_FRIEND)
	Z_ITERABLE_SECTION_ROM(bt_mesh_friend_cb, 4)
#endif

#if defined(CONFIG_BT_MESH_LOW_POWER)
	Z_ITERABLE_SECTION_ROM(bt_mesh_lpn_cb, 4)
#endif

#if defined(CONFIG_EC_HOST_CMD)
	Z_ITERABLE_SECTION_ROM(ec_host_cmd_handler, 4)
#endif

#if defined(CONFIG_SETTINGS)
	Z_ITERABLE_SECTION_ROM(settings_handler_static, 4)
#endif

#if defined(CONFIG_EMUL)
	SECTION_RODATA_PROLOGUE(emulators_section,,)
	{
		SYMBOL(__emul_list_start) = .;
		KEEP(*(SORT_BY_NAME(".emulators")));
		SYMBOL(__emul_list_end) = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif /* CONFIG_EMUL */

#if defined(CONFIG_DNS_SD)
	Z_ITERABLE_SECTION_ROM(dns_sd_rec, 4)
#endif

#if defined(CONFIG_PCIE)
	SECTION_RODATA_PROLOGUE(irq_alloc,,)
	{
		__irq_alloc_start = .;
		KEEP(*(SORT_BY_NAME("._irq_alloc*")));
		__irq_alloc_end = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif /* CONFIG_PCIE */

	Z_ITERABLE_SECTION_ROM(log_const, 4)
	Z_ITERABLE_SECTION_ROM(log_backends, 4)

	Z_ITERABLE_SECTION_ROM(shell, 4)
	Z_ITERABLE_SECTION_ROM(shell_root_cmds, 4)

	Z_ITERABLE_SECTION_ROM(font_entry, 4)

	Z_ITERABLE_SECTION_ROM(tracing_backend, 4)
