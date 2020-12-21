/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <ztest.h>
#include <device.h>
#include <soc.h>
#include <power/power.h>
#include <logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(pwrmgmt_test);

#define SLP_STATES_SUPPORTED      2ul

/* Thread properties */
#undef TASK_STACK_SIZE
#define TASK_STACK_SIZE           1024ul
#define PRIORITY                  K_PRIO_COOP(5)

/* Sleep time should be lower than CONFIG_PM_MIN_RESIDENCY_SLEEP_1 */
#define THREAD_A_SLEEP_TIME       100ul
#define THREAD_B_SLEEP_TIME       1000ul

/* Maximum latency should be less than 300 ms */
#define MAX_EXPECTED_MS_LATENCY   500ul

/* Sleep some extra time than minimum residency */
#define DP_EXTRA_SLP_TIME         1100ul
#define LT_EXTRA_SLP_TIME         500ul

#define SEC_TO_MSEC               1000ul

K_THREAD_STACK_DEFINE(stack_a, TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_b, TASK_STACK_SIZE);

static struct k_thread thread_a_id;
static struct k_thread thread_b_id;

struct pm_counter {
	uint8_t entry_cnt;
	uint8_t exit_cnt;
};

/* Track time elapsed */
static int64_t trigger_time;
static bool checks_enabled;
/* Track entry/exit to sleep */
struct pm_counter pm_counters[SLP_STATES_SUPPORTED];

static void pm_latency_check(void)
{
	int64_t latency;
	int secs;
	int msecs;

	latency = k_uptime_delta(&trigger_time);
	secs = (int)(latency / SEC_TO_MSEC);
	msecs = (int)(latency % SEC_TO_MSEC);
	LOG_INF("PM sleep entry latency %d.%03d seconds", secs, msecs);

	zassert_false(secs > 0, "Sleep entry latency is too high");

	zassert_false(msecs > MAX_EXPECTED_MS_LATENCY,
			"Sleep entry latency is higher than expected");
}

/* Hooks to count entry/exit */
void pm_notify_power_state_entry(enum power_states state)
{
	if (!checks_enabled) {
		return;
	}

	if (pm_is_sleep_state(state)) {
		pm_counters[0].entry_cnt++;
		pm_latency_check();
	} else if (pm_is_deep_sleep_state(state)) {
		pm_counters[1].entry_cnt++;
		pm_latency_check();
	}
}

void pm_notify_power_state_exit(enum power_states state)
{
	if (!checks_enabled) {
		return;
	}

	if (pm_is_sleep_state(state)) {
		pm_counters[0].exit_cnt++;
	} else if (pm_is_deep_sleep_state(state)) {
		pm_counters[1].exit_cnt++;
	}
}

static void pm_check_counters(uint8_t cycles)
{
	for (int i = 0; i < SLP_STATES_SUPPORTED; i++) {

		LOG_INF("PM state[%d] entry counter %d\n", i,
			pm_counters[i].entry_cnt);
		LOG_INF("PM state[%d] exit counter %d\n", i,
			pm_counters[i].exit_cnt);

		zassert_equal(pm_counters[i].entry_cnt,
				pm_counters[i].exit_cnt,
				"PM counters entry/exit mismatch");

		pm_counters[i].entry_cnt = 0;
		pm_counters[i].exit_cnt = 0;
	}
}

static void pm_reset_counters(void)
{
	for (int i = 0; i < SLP_STATES_SUPPORTED; i++) {
		pm_counters[i].entry_cnt = 0;
		pm_counters[i].exit_cnt = 0;
	}

	checks_enabled = false;
}

static void pm_trigger_marker(void)
{
	trigger_time = k_uptime_get();

	printk("PM >\n");
}

static void pm_exit_marker(void)
{
	int64_t residency_delta;
	int secs;
	int msecs;

	printk("PM <\n");

	if (trigger_time > 0) {
		residency_delta = k_uptime_delta(&trigger_time);
		secs = (int)(residency_delta / SEC_TO_MSEC);
		msecs = (int)(residency_delta % SEC_TO_MSEC);
		LOG_INF("PM sleep residency %d.%03d seconds", secs, msecs);
	}
}

static int task_a_init(void)
{
	LOG_INF("Thread task A init");

	return 0;
}

static int task_b_init(void)
{
	LOG_INF("Thread task B init");

	return 0;
}

void task_a_thread(void *p1, void *p2, void *p3)
{
	while (true) {
		k_msleep(THREAD_A_SLEEP_TIME);
		printk("A");
	}
}

static void task_b_thread(void *p1, void *p2, void *p3)
{
	while (true) {
		k_msleep(THREAD_B_SLEEP_TIME);
		printk("B");
	}
}

static void create_tasks(void)
{
	task_a_init();
	task_b_init();

	k_thread_create(&thread_a_id, stack_a, TASK_STACK_SIZE, task_a_thread,
		NULL, NULL, NULL, PRIORITY,  K_INHERIT_PERMS, K_FOREVER);
	k_thread_create(&thread_b_id, stack_b, TASK_STACK_SIZE, task_b_thread,
		NULL, NULL, NULL, PRIORITY,  K_INHERIT_PERMS, K_FOREVER);

	k_thread_start(&thread_a_id);
	k_thread_start(&thread_b_id);

}

static void destroy_tasks(void)
{
	k_thread_abort(&thread_a_id);
	k_thread_abort(&thread_b_id);

	k_thread_join(&thread_a_id, K_FOREVER);
	k_thread_join(&thread_b_id, K_FOREVER);
}

static void suspend_all_tasks(void)
{
	k_thread_suspend(&thread_a_id);
	k_thread_suspend(&thread_b_id);
}

static void resume_all_tasks(void)
{
	k_thread_resume(&thread_a_id);
	k_thread_resume(&thread_b_id);
}

int test_pwr_mgmt_multithread(uint8_t cycles)
{
	uint8_t iterations = cycles;

	create_tasks();

	LOG_INF("PM multi-thread test started for cycles: %d", cycles);

	checks_enabled = true;
	while (iterations-- > 0) {

		/* Light sleep cycle */
		LOG_INF("Suspend...");
		suspend_all_tasks();
		LOG_INF("About to enter light sleep");
		pm_trigger_marker();
		k_msleep(CONFIG_PM_MIN_RESIDENCY_SLEEP_1 +
			 LT_EXTRA_SLP_TIME);

		LOG_INF("Wake from Light Sleep");
		pm_exit_marker();
		LOG_INF("Resume");
		resume_all_tasks();

		/* Deep sleep cycle */
		/* Platforms that do not automatically enter deep sleep */
		/* states in its residency policy will simply enter light */
		/* sleep states instead */
		LOG_INF("Suspend...");
		suspend_all_tasks();
		LOG_INF("About to enter deep sleep");

		pm_trigger_marker();
		k_msleep(CONFIG_PM_MIN_RESIDENCY_DEEP_SLEEP_1 +
			 DP_EXTRA_SLP_TIME);

		LOG_INF("Wake from Deep Sleep");
		pm_exit_marker();
		LOG_INF("Resume");
		resume_all_tasks();
	}

	destroy_tasks();

	LOG_INF("PM multi-thread completed");
	pm_check_counters(cycles);
	pm_reset_counters();

	return 0;
}

int test_pwr_mgmt_singlethread(uint8_t cycles)
{
	uint8_t iterations = cycles;

	LOG_INF("PM single-thread test started for cycles: %d", cycles);

	checks_enabled = true;
	while (iterations-- > 0) {

		/* Trigger Light Sleep 1 state. 48MHz PLL stays on */
		LOG_INF("About to enter light sleep");
		pm_trigger_marker();
		k_msleep(CONFIG_PM_MIN_RESIDENCY_SLEEP_1 +
			 LT_EXTRA_SLP_TIME);
		LOG_INF("Wake from Light Sleep");
		pm_exit_marker();

		/* Trigger Deep Sleep 1 state. 48MHz PLL off */
		/* Platforms that do not automatically enter deep sleep */
		/* states in its residency policy will simply enter light */
		/* sleep states instead */
		LOG_INF("About to enter deep Sleep");

		pm_trigger_marker();
		k_msleep(CONFIG_PM_MIN_RESIDENCY_DEEP_SLEEP_1 +
			 DP_EXTRA_SLP_TIME);

		LOG_INF("Wake from Deep Sleep");
		pm_exit_marker();
	}

	LOG_INF("PM single-thread completed");
	pm_check_counters(cycles);
	pm_reset_counters();

	return 0;
}

int test_dummy_init(void)
{
	uint8_t iterations = 1;

	LOG_INF("PM dummy single-thread test started for one cycle");

	checks_enabled = true;
	while (iterations-- > 0) {
		LOG_INF("About to enter light sleep");
		pm_trigger_marker();
		k_msleep(CONFIG_PM_MIN_RESIDENCY_SLEEP_1 +
			 LT_EXTRA_SLP_TIME);
		LOG_INF("Wake from Light Sleep");
		pm_exit_marker();

		LOG_INF("About to enter deep Sleep");
		pm_trigger_marker();
		k_msleep(CONFIG_PM_MIN_RESIDENCY_DEEP_SLEEP_1 +
			 DP_EXTRA_SLP_TIME);
		LOG_INF("Wake from Deep Sleep");
		pm_exit_marker();
	}

	LOG_INF("PM dummy single-thread completed");
	pm_reset_counters();
	return 0;
}
