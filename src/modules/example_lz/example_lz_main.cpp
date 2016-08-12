/**
 * @file example_lz_main.cpp
 * For UAVGP2016 --- LiuZhong
 * 2016/7/31
 */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <uORB/uORB.h>
#include <nuttx/config.h>
#include <stdlib.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

// 6个自动定义的Mavlink消息
#include <uORB/topics/task_status_change.h>
#include <uORB/topics/task_status_monitor.h>
#include <uORB/topics/fixed_target_position.h>
#include <uORB/topics/fixed_target_return.h>
#include <uORB/topics/vision_num_scan.h>
#include <uORB/topics/vision_one_num_get.h>

static bool thread_should_exit = false;		/**< example_lz exit flag */
static bool thread_running = false;			/**< example_lz status flag */
static int  example_lz_task;				/**< Handle of example_lz task / thread */

// 线程管理程序
extern "C" __EXPORT int example_lz_main(int argc, char *argv[]);
// 用户线程, 执行用户代码
int example_lz_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
static void
usage(const char *reason)
{

}

// 线程管理程序
int example_lz_main(int argc, char *argv[])
{
	if (argc < 2) {
			warnx("usage: example_lz {start|stop|status}");
			return 1;
		}

	if (!strcmp(argv[1], "start")) {   //shell启动命令
		if (thread_running) {		   // 如果线程已经启动了
			warnx("example_lz already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;		// 将线程状态位设置为false
		example_lz_task = px4_task_spawn_cmd("example_lz",				    // 线程名
										SCHED_DEFAULT,					// 调度模式
										SCHED_PRIORITY_DEFAULT,			// 优先级
										1200,							// 堆栈大小
										example_lz_thread_main,			// 线程入口
										nullptr);
		if (example_lz_task < 0) {
				warn("task start failed");
				return -errno;
			}
		exit(0);						// 正常退出
	}
	if (!strcmp(argv[1], "stop")) {		// shell停止命令
		thread_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {	// shell查询命令, 用于查询线程的状态.
		if (thread_running) {
			warnx("\t running\n");
		} else {
			warnx("\t not started\n");
		}
		exit(0);
	}
	usage("unrecognized command");
	exit(1);
}
// 线程主体
int example_lz_thread_main(int argc, char *argv[])
{
	thread_running=true;

	// 订阅位置信息
	int _fixed_target_position_sub;
	_fixed_target_position_sub = orb_subscribe(ORB_ID(fixed_target_position));
	struct fixed_target_position_s position_sub;
	bool updated_pos;
	// 发布位置信息
	orb_advert_t _fixed_target_return_pub;
	struct fixed_target_return_s position_pub;
	memset(&position_pub, 0, sizeof(position_pub));

	// 订阅任务状态修改信息
	int _task_status_change_sub;
	_task_status_change_sub  = orb_subscribe(ORB_ID(task_status_change));
	struct task_status_change_s task_status_sub;
	bool updated_task;
	// 发布实时任务状态信息
	orb_advert_t _task_status_monitor_pub;
	struct task_status_monitor_s task_status_pub;
	memset(&task_status_pub, 0, sizeof(task_status_pub));

	// 发布数字扫描信息
	orb_advert_t _vision_num_scan_pub;
	struct vision_num_scan_s num_scan_pub;
	memset(&num_scan_pub, 0, sizeof(num_scan_pub));
	// 发布数据采集信息
	orb_advert_t _vision_one_num_get_pub;
	struct vision_one_num_get_s one_num_pub;
	memset(&one_num_pub, 0, sizeof(one_num_pub));

	// 如果线程没有被停止
	while(!thread_should_exit)
	{
		// 位置信息
		orb_check(_fixed_target_position_sub, &updated_pos);
		if (updated_pos)
		{
			// 读取主题
			orb_copy(ORB_ID(fixed_target_position), _fixed_target_position_sub, &position_sub);
			// 变量赋值
			position_pub.timestamp=hrt_absolute_time();
			position_pub.home_lon=position_sub.home_lon;
			position_pub.home_lat=position_sub.home_lat;
			position_pub.home_alt=position_sub.home_alt;
			position_pub.observe_lon=position_sub.observe_lon;
			position_pub.observe_lat=position_sub.observe_lat;
			position_pub.observe_alt=position_sub.observe_alt;
			position_pub.spray_left_lon=position_sub.spray_left_lon;
			position_pub.spray_left_lat=position_sub.spray_left_lat;
			position_pub.spray_left_alt=position_sub.spray_left_alt;
			position_pub.spray_right_lon=position_sub.spray_right_lon;
			position_pub.spray_right_lat=position_sub.spray_right_lat;
			position_pub.spray_right_alt=position_sub.spray_right_alt;
			// 发布主题
			_fixed_target_return_pub = orb_advertise(ORB_ID(fixed_target_return), &position_pub);
			orb_publish(ORB_ID(fixed_target_return), _fixed_target_return_pub, &position_pub);
		}

		// 任务状态信息
		orb_check(_task_status_change_sub, &updated_task);
		if (updated_task)
		{
			// 读取主题
			orb_copy(ORB_ID(task_status_change), _task_status_change_sub, &task_status_sub);
			// 变量赋值
			task_status_pub.timestamp=hrt_absolute_time();
			task_status_pub.num_odd_even=task_status_sub.num_odd_even;
			task_status_pub.task_status_1=task_status_sub.task_status_1;
			task_status_pub.task_status_2=task_status_sub.task_status_2;
			task_status_pub.target_lon=position_sub.home_lon;
			task_status_pub.target_lat=position_sub.home_lat;
			task_status_pub.target_alt=position_sub.home_alt;
			// 发布主题
			_task_status_monitor_pub = orb_advertise(ORB_ID(task_status_monitor), &task_status_pub);
			orb_publish(ORB_ID(task_status_monitor), _task_status_monitor_pub, &task_status_pub);

			if(task_status_pub.task_status_1==2) //扫描
			{
				num_scan_pub.timestamp=hrt_absolute_time();
				num_scan_pub.serial_num=10;
				num_scan_pub.cur_num=9;
				num_scan_pub.cur_num_lon=123;
				num_scan_pub.cur_num_lat=60;
				num_scan_pub.cur_num_alt=120;
				_vision_num_scan_pub = orb_advertise(ORB_ID(vision_num_scan), &num_scan_pub);
				orb_publish(ORB_ID(vision_num_scan), _vision_num_scan_pub, &num_scan_pub);
			}
			if(task_status_pub.task_status_1==3) //Num1 - Num5
			{
				one_num_pub.timestamp=hrt_absolute_time();
				one_num_pub.serial_num=1;
				one_num_pub.num=11;
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get), &one_num_pub);
				orb_publish(ORB_ID(vision_one_num_get), _vision_one_num_get_pub, &one_num_pub);
			}
			if(task_status_pub.task_status_1==4)
			{
				one_num_pub.timestamp=hrt_absolute_time();
				one_num_pub.serial_num=2;
				one_num_pub.num=12;
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get), &one_num_pub);
				orb_publish(ORB_ID(vision_one_num_get), _vision_one_num_get_pub, &one_num_pub);
			}
			if(task_status_pub.task_status_1==5)
			{
				one_num_pub.timestamp=hrt_absolute_time();
				one_num_pub.serial_num=3;
				one_num_pub.num=13;
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get), &one_num_pub);
				orb_publish(ORB_ID(vision_one_num_get), _vision_one_num_get_pub, &one_num_pub);
			}
			if(task_status_pub.task_status_1==6)
			{
				one_num_pub.timestamp=hrt_absolute_time();
				one_num_pub.serial_num=4;
				one_num_pub.num=14;
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get), &one_num_pub);
				orb_publish(ORB_ID(vision_one_num_get), _vision_one_num_get_pub, &one_num_pub);
			}
			if(task_status_pub.task_status_1==7)
			{
				one_num_pub.timestamp=hrt_absolute_time();
				one_num_pub.serial_num=5;
				one_num_pub.num=15;
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get), &one_num_pub);
				orb_publish(ORB_ID(vision_one_num_get), _vision_one_num_get_pub, &one_num_pub);
			}
		}

		usleep(100000); // 100ms
	}

	thread_running=false;
	return 0;
}
