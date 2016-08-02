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
#include <uORB/topics/task_status_monitor.h>
#include <uORB/topics/fixed_target_position.h>
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
	int _vision_one_num_get_sub;
	_vision_one_num_get_sub = orb_subscribe(ORB_ID(vision_one_num_get));
	struct vision_one_num_get_s for_sub;
	memset(&for_sub, 0, sizeof(for_sub));
	bool updated;
	orb_advert_t _vision_one_num_get_pub;
	struct vision_one_num_get_s for_pub;
	memset(&for_pub, 0, sizeof(for_pub));
	while(!thread_should_exit)				// 如果线程没有被停止
	{
		orb_check(_vision_one_num_get_sub, &updated);
		if (updated)
		{
			// 读取线程
			orb_copy(ORB_ID(vision_one_num_get), _vision_one_num_get_sub, &for_sub);
			// 在原来线程的基础上发布一个Mavlink消息
			for_pub.timestamp=hrt_absolute_time();
			for_pub.num=for_sub.num*2;
			for_pub._padding0[0]=0;
			for_pub._padding0[1]=1;
			for_pub._padding0[2]=2;
			for_pub._padding0[3]=3;
			for_pub._padding0[4]=4;
			for_pub._padding0[5]=5;
			for_pub._padding0[6]=6;
			_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get), &for_pub);
			orb_publish(ORB_ID(vision_one_num_get), _vision_one_num_get_pub, &for_pub);
		}
		usleep(100000); // 100ms
	}
	return 0;
}
