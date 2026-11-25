/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 22:05:27
 * @LastEditors: kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime: 2024-07-12 03:34:38
 * @FilePath     : /you_see_arm/ys_business/ys_state_machine.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_business_network.h"
#include "ys_consumer.h"
#include "ys_errno.h"
#include "ys_msg_distribute.h"
#include "ys_producer.h"
#include "ys_state_machine.h"

#include "../core/ys_base.h"
#include "../core/ys_conf.h"
#include "../core/ys_pthread.h"

#include "../common/common.h"
#include "../common/ys_protocal.h"

#if MF150_FL6031 == 1 || MF150_FL6532 == 1
static inline int sys_workmode_get(void)
{
	return 0;
}
static inline int sys_workmode_set(int mode)
{
	(void)mode;
	return 0;
}
#else
#include "yousee_lidar_api_public.h"
#endif

static ys_state_machine_t ys_state; // 程序状态机
thpool_t data_thpool;
sys_state_t sys_state = { 0 };

const char state_buf[][8] = {
	"INIT", // 初始化状态
	"IDLE", // 空闲状态
	"SCAN", // 扫描状态
	"FAULT", // 故障状态
	"DEBUG", // 调试状态
};

static const p_handler_t scan_pthread_handler[] = {
#if USE_SOCKET == 0
	{ 0, NO_SET_PRIORITY, pthread_cent_udp_client }, // UDP 实例（核0）
#endif
	{ 2, SET_HIGHEST_PRIORITY, phtread_consumer }, // 消费者线程（核2）
	{ 1, SET_HIGHEST_PRIORITY, phtread_producer }, // 生产者线程（核1）
};

int get_now_run_state()
{
	int state;
	pthread_mutex_lock(&(ys_state.lock));
	state = ys_state.state;
	pthread_mutex_unlock(&(ys_state.lock));
	return state;
}

int stop_capture_for_shutdown(void)
{
	int ret = RET_OK;
#if LOCAL_TEST == 0
#if CS96_BETA == 1 || CS96_GAMMA == 1
	/* 当前 am6204 驱动的 sys_workmode_set() 为空实现，不能作为硬件停采确认。 */
	log_error("硬件停采尚未实现：回收应用线程后必须由平台适配器执行 recover");
	ret = RET_FAIL;
#else
	ret = sys_workmode_set(PL_IDLE);
#endif
#endif
	exit_thread_to_get_in_idle_state();
	return ret;
}

int exit_thread_to_get_in_idle_state()
{
	set_memory_pool_run_flag(OFF);
	thpool_exit(&data_thpool);
	return RET_OK;
}

/**
 * @description  : check state is valid
 * @param         {int} state
 * @return        {uint8_t} result
 */
static uint8_t check_state_valid(int state)
{
	if (state >= INIT && state <= DEBUG) {
		return RET_OK;
	} else {
		return RET_FAIL;
	}
}

/**
 * @description  : get now running data thread count
 * @param         {*}
 * @return        {int} count
 */
static int get_now_running_data_thread_count()
{
	int task_num = 0;
	pthread_mutex_lock(&(data_thpool.lock));
	task_num = data_thpool.thpool_count;
	pthread_mutex_unlock(&(data_thpool.lock));

	return task_num;
}

/**
 * @description  : set state to scan mode via add work to thread pool
 * @param         {*}
 * @return        {*}
 */
static void set_scan_with_add_work(void)
{
	int ret = RET_FAIL;
	ret = init_producer_parm(cal_param.lines_v);
	if (ret != RET_OK) {
		log_fatal("init producer paramter fail");
		return;
	}

	ret = init_consumer_parm(cal_param.lines_v, config.echo_mode);
	if (ret != RET_OK) {
		log_fatal("init consumer paramter fail");
		return;
	}

	// 使能内存池的分配标识，否则内存池的返回值永远是 NULL
	set_memory_pool_run_flag(ON);

	int count = (int)(sizeof(scan_pthread_handler) / sizeof(scan_pthread_handler[0]));
	for (int i = 0; i < count; i++) {
		thread_t th_arg;

		th_arg.thread_arg = (void *)(&udp_data_instance);
		th_arg.signal_enable = ON;
		data_thpool.signal_num[i] = th_arg.signum = MY_SIGNAL + i;
		data_thpool.tid[i] = create_thread(scan_pthread_handler[i].handler, &th_arg, NULL,
						   scan_pthread_handler[i].core_id, scan_pthread_handler[i].priority);
	}

	pthread_mutex_lock(&(data_thpool.lock));
	data_thpool.thpool_count = count;
	pthread_mutex_unlock(&(data_thpool.lock));

	return;
}

/**
 * @description  : 从初始状态->空闲状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_init_to_idle(void)
{
	int task_num = get_now_running_data_thread_count();
	int ret = RET_FAIL;

	if (task_num != NO_TASK_TO_RUN) {
		exit_thread_to_get_in_idle_state();
		log_info("thread pool pause %d task", task_num);
	}

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_IDLE) {
		ret = sys_workmode_set(PL_IDLE);
	}
#endif
	return ret;
}

/**
 * @description  : 从初始状态->扫描状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_init_to_scan(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_SCAN) {
		ret = sys_workmode_set(PL_SCAN);
	}
#endif

	set_scan_with_add_work();
	return ret;
}

/**
 * @description  : 从初始状态->故障状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_init_to_fault(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_FAULT) {
		ret = sys_workmode_set(PL_FAULT);
	}
#endif

	return ret;
}

/**
 * @description  : 从初始状态->调试状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_init_to_debug(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_DEBUG) {
		ret = sys_workmode_set(PL_DEBUG);
	}
#endif
	return ret;
}

/**
 * @description  : 从空闲状态->扫描状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_idle_to_scan(void)
{
	int task_num = get_now_running_data_thread_count();
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_SCAN) {
		ret = sys_workmode_set(PL_SCAN);
	}
#endif

	if (task_num != NO_TASK_TO_RUN) {
		exit_thread_to_get_in_idle_state();
	}
	set_scan_with_add_work();

	return ret;
}

/**
 * @description  : 从空闲状态->故障状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_idle_to_fault(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_FAULT) {
		ret = sys_workmode_set(PL_FAULT);
	}
#endif

	return ret;
}

/**
 * @description  : 从空闲状态->调试状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_idle_to_debug(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_DEBUG) {
		ret = sys_workmode_set(PL_DEBUG);
	}
#endif
	return ret;
}

/**
 * @description  : 从扫描状态->空闲状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_scan_to_idle(void)
{
	int task_num = get_now_running_data_thread_count();
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_IDLE) {
		ret = sys_workmode_set(PL_IDLE);
	}
#endif

	if (task_num != NO_TASK_TO_RUN) {
		exit_thread_to_get_in_idle_state();
	}
	return ret;
}

/**
 * @description  : 从扫描状态->故障状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_scan_to_fault(void)
{
	int task_num = get_now_running_data_thread_count();
	int ret = RET_FAIL;

	if (task_num != NO_TASK_TO_RUN) {
		exit_thread_to_get_in_idle_state();
		log_info("thread pool pause %d task", task_num);
	}

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_FAULT) {
		ret = sys_workmode_set(PL_FAULT);
	}
#endif

	return ret;
}

/**
 * @description  : 从故障状态->调试状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_fault_to_debug(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_DEBUG) {
		ret = sys_workmode_set(PL_DEBUG);
	}
#endif
	return ret;
}

/**
 * @description  : 从调试状态->空闲状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_debug_to_idle(void)
{
	int task_num = get_now_running_data_thread_count();
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_IDLE) {
		ret = sys_workmode_set(PL_IDLE);
	}
#endif

	if (task_num != NO_TASK_TO_RUN) {
		exit_thread_to_get_in_idle_state();
	}
	return ret;
}

/**
 * @description  : 从调试状态->故障状态
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int set_state_from_debug_to_fault(void)
{
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	if (sys_workmode_get() != PL_FAULT) {
		ret = sys_workmode_set(PL_FAULT);
	}
#endif

	return ret;
}

void get_run_state(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	ps_state_info_t ack_data = { 0 };
	int ret = RET_FAIL;
	int state = get_now_run_state();

	ret = check_state_valid(state);

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = ret;
	ack_data.state = state;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void set_run_state(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = run_fail;
	int switch_to_state = msg->buf[2];
	int state;

	state = get_now_run_state();

	if (state == switch_to_state) {
		ret = run_succ;
		goto ack;
	} else {
		log_info("Switch state %s to %s", state_buf[state + 1], state_buf[switch_to_state + 1]);
	}

	if (switch_to_state == DEBUG && state != INIT) {
		uint32_t input_passwd = combine_uint8_to_uint32(msg->buf[3], msg->buf[4], msg->buf[5], msg->buf[6]);
		if (input_passwd != DEBUG_PASSWORD) {
			ret = pwd_error;
			goto ack;
		}
	}

	switch (state) {
	case INIT:
		switch (switch_to_state) {
		case IDLE:
			ret = set_state_from_init_to_idle();
			break;
		case SCAN:
			ret = set_state_from_init_to_scan();
			break;
		case FAULT:
			ret = set_state_from_init_to_fault();
			break;
		case DEBUG:
			ret = set_state_from_init_to_debug();
			break;
		default:
			ret = illegal_cmd;
			goto ack;
			break;
		}
		break;
	case IDLE:
		switch (switch_to_state) {
		case SCAN:
			ret = set_state_from_idle_to_scan();
			break;
		case FAULT:
			ret = set_state_from_idle_to_fault();
			break;
		case DEBUG:
			ret = set_state_from_idle_to_debug();
			break;
		default:
			ret = illegal_cmd;
			goto ack;
			break;
		}
		break;
	case SCAN:
		switch (switch_to_state) {
		case IDLE:
			ret = set_state_from_scan_to_idle();
			break;
		case FAULT:
			ret = set_state_from_scan_to_fault();
			break;
		default:
			ret = illegal_cmd;
			goto ack;
			break;
		}
		break;
	case FAULT:
		switch (switch_to_state) {
		case DEBUG:
			ret = set_state_from_fault_to_debug();
			break;
		default:
			ret = illegal_cmd;
			goto ack;
			break;
		}
		break;
	case DEBUG:
		switch (switch_to_state) {
		case IDLE:
			ret = set_state_from_debug_to_idle();
			break;
		case FAULT:
			ret = set_state_from_debug_to_fault();
			break;
		default:
			ret = illegal_cmd;
			goto ack;
			break;
		}
		break;
	default:
		ret = illegal_cmd;
		goto ack;
		break;
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pthread_mutex_lock(&(ys_state.lock));
	ys_state.state = switch_to_state;
	log_info("Switch state %s to %s success", state_buf[state + 1], state_buf[switch_to_state + 1]);
	pthread_mutex_unlock(&(ys_state.lock));

#if LOCAL_TEST == 0
	int now_state = get_now_run_state();
	log_info("Now PS state is %s, PL state is %d", state_buf[now_state + 1], sys_workmode_get());
#endif

ack:
	if (msg->silent) {
		return;
	}

	set_run_state_t ack_info;
	ack_info.ret_code = ret;
	ack_info.now_state = (uint8_t)get_now_run_state();
	pack(msg, (uint8_t *)&ack_info, sizeof(ack_info), ACK);
	return;
}

/**
 * @description  : initialization state machine mutex lock
 * @param         {*}
 * @return        {*}
 */
static void init_state_machine_mutex()
{
	pthread_mutex_init(&(sys_state.lock), NULL);
	pthread_mutex_init(&(ys_state.lock), NULL);
	pthread_mutex_init(&(data_thpool.lock), NULL);
	return;
}

/**
 * @description  : destroy state machine mutex lock
 * @param         {*}
 * @return        {*}
 */
static void destroy_state_machine_mutex()
{
	pthread_mutex_destroy(&(sys_state.lock));
	pthread_mutex_destroy(&(ys_state.lock));
	pthread_mutex_destroy(&(data_thpool.lock));
	return;
}

void init_state_machine(int init_state)
{
	uint8_t buf[] = { CMD_SET_STATE, CMD_ID_SET_STATE, IDLE };
	management_id_t msg;
	msg.code_set = CMD_SET_STATE;
	msg.code_id = CMD_ID_SET_STATE;
	msg.silent = SILENT;
	msg.buf = buf;

	if (check_state_valid(init_state) == run_succ) {
		buf[2] = init_state;
	}

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	sys_state.fpga_state = (struct lidar_fpga_selftest *)Malloc(sizeof(struct lidar_fpga_selftest));
	memset(sys_state.fpga_state, 0, sizeof(struct lidar_fpga_selftest));
#endif
	init_state_machine_mutex();

	pthread_mutex_lock(&(ys_state.lock));
	ys_state.state = INIT;
	pthread_mutex_unlock(&(ys_state.lock));

	set_run_state((void *)&msg);

	return;
}

void uninit_state_machine()
{
	destroy_state_machine_mutex();
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	Free((void **)&(sys_state.fpga_state));
#endif
}

void get_self_state(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	device_state_info_t ack_data = { 0 };
	int ret = RET_FAIL;

	pthread_mutex_lock(&(sys_state.lock));
	memcpy(&(ack_data.pl_state), sys_state.fpga_state, sizeof(struct lidar_fpga_selftest));
	pthread_mutex_unlock(&(sys_state.lock));

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	ret = ld_temp_get(&(ack_data.ld_temp));
	ack_data.ld_temp = uint16_to_float_positive_number(ack_data.ld_temp);

	ret += spad_temp_get(&(ack_data.spad_temp));
	ack_data.spad_temp = uint16_to_float_positive_number(ack_data.spad_temp);

	ret += fpga_temp_get(&(ack_data.fpga_temp));
	ack_data.fpga_temp = uint16_to_float_positive_number(ack_data.fpga_temp);

	ack_data.motor_cycles = motor_period_get();
	if (ack_data.motor_cycles != ERR_FPGA_REG32) {
		ret += RET_OK;
	} else {
		ret += RET_FAIL;
	}

	ack_data.vop_voltage = vop_sample_get();
	if (ack_data.vop_voltage != ERR_FPGA_REG32) {
		ret += RET_OK;
	} else {
		ret += RET_FAIL;
	}
#endif // LOCAL_TEST

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ps_state = (uint8_t)get_now_run_state();
	ack_data.ret_code = (uint8_t)ret;

	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
#endif

	return;
}
