#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/marlin_platform.h>
#include <linux/sdiom_rx_api.h>
#include <linux/vmalloc.h>

#include "mdbg_common.h"
#include "wcn_debug.h"

static struct task_struct *wcn_debug_thread;
struct completion dumpmem_complete;
static struct file *log_file;
static struct file *dump_file;
static loff_t log_pos;

int log_rx_callback(void *addr, unsigned int len, unsigned int fifo_id)
{
	ssize_t ret;

	pr_info("log_rx_callback\n");
	if (IS_ERR(log_file)) {
		mdbg_read_release(fifo_id);
		return 0;
	}
	ret = kernel_write(log_file, addr, len, log_pos);
	if (ret != len) {
		pr_err("wcn log write to file failed: %zd\n", ret);
		return ret < 0 ? ret : -ENODEV;
	}
	log_pos += ret;

	mdbg_read_release(fifo_id);

	return 0;
}

static int wcn_dbg_thread(void *unused)
{
	ssize_t ret;
	char *buffer = NULL;
	long int read_size;
	ssize_t count = 2 * 1024 * 1024;

	while (!kthread_should_stop()) {
		wait_for_completion(&dumpmem_complete);

		buffer = vmalloc(2 * 1024 * 1024);
		if (!buffer)
			return -ENOMEM;

		read_size = mdbg_receive(buffer, count);
		pr_info("mem read size is %ld\n", read_size);
		if (read_size < 0) {
			vfree(buffer);
			return read_size;
		}

		dump_file = filp_open("/data/wcn/cp2mem.mem",
				O_RDWR | O_CREAT | O_APPEND, 0644);
		if (IS_ERR(dump_file)) {
			pr_err("%s open file mem error\n", __func__);
			vfree(buffer);
			return PTR_ERR(dump_file);
		}

		ret = kernel_write(dump_file, buffer, read_size, 0);
		if (ret != read_size) {
			pr_err("wcn mem write to file failed: %zd\n", ret);
			vfree(buffer);
			return ret < 0 ? ret : -ENODEV;
		}

		vfree(buffer);
		filp_close(dump_file, NULL);

	}

	return 0;
}

static int launch_debug(void)
{
	if (wcn_debug_thread)
		return 0;

	wcn_debug_thread = kthread_create(wcn_dbg_thread, NULL,
				"wcn_dbg_thread");
	if (IS_ERR(wcn_debug_thread)) {
		pr_err("cannot start thread wcn_debug_thread");
		wcn_debug_thread = NULL;
		return -ENOMEM;
	}
	wake_up_process(wcn_debug_thread);

	return 0;
}

int wcn_debug_init(void)
{
	int err;

	if (functionmask[7] == 0)
		return 0;
	init_completion(&dumpmem_complete);

	pr_info("%s entry\n", __func__);

	if ((functionmask[7] & CP2_FLAG_YLOG) == 1) {
		log_file = filp_open("/data/wcn/cp2log.txt",
				O_CREAT | O_WRONLY, 0);
		if (IS_ERR(log_file)) {
			pr_err("%s open wcn log file error no. %d\n",
					__func__, IS_ERR(log_file));
			return PTR_ERR(log_file);
		}
	}

	if (unlikely(!wcn_debug_thread)) {
		err = launch_debug();
		if (err)
			goto out;
	}

	return 0;

out:
	return err;
}
