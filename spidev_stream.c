/*
 * @brief    : this is a spi protocol driver . 
 *           : we can use to transfer data via spi focus on streaming data
 *           : in here i just write skeleton code (custom from availble spidev.c). it's depend on your purpose then you can customize it to fix your project. 
 * @author   : giann <ngocgia73@gmail.com>
 * @filename : spidev_stream.c
 */
#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/ioctl.h>
#include <linux/fs.h> 
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>

#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/kfifo.h>
#include <mach/irqs.h>
#include <asm/io.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <linux/mm.h> 
#include <linux/irq.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/spi/spidev_stream.h>

#define STATUS_EXIT_MAIN_THREAD -100
#define SPIDEV_MAJOR                    153     /* assigned */
#define N_SPI_MINORS                    32      /* ... up to 256 */
#define DEV_NAME                "spidev"

#define GPIO_INT_PIN            13 // you need redefine this value
#define AUDIO_QUEUE_SIZE    (4096*2)
#define VIDEO_QUEUE_SIZE    (4096*16)
#define TALKBACK_QUEUE_SIZE (4096*2)
#define U_QUEUE_SIZE        (128) 


// 
#define IO_COMMAND_BYTE_MASK  0xFF00000
#define IO_ADDRESS_BYTE_MASK  0x00FF0000
#define DEVICE_ADDRESS_MASK   0x0000FF00
#define COMMAND_BYTE_MASK     0x000000FF
#define UART_READ_BIT         0x20000000
#define VIDEO_FULL_MASK       0x00000001
#define AUDIO_FULL_MASK       0x00000002
#define TALKBACK_FULL_MASK    0x00000004
#define J_FULL_MASK           0x00000001
#define U_CTS_MASK            0x00000040

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list); 
static unsigned int bufsiz = 4096; 
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message")

static struct task_struct *maintask = NULL;
static DEFINE_MUTEX(device_list_lock);
static struct class *socspi_class;
static DECLARE_COMPLETION(spi_done);

static void socspi_complete(void *arg)
{
	complete(arg);
}

static ssize_t socspi_sync(struct socspi_data *socspi, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = socspi_complete;
	message->context = &done;
	
	spin_lock_irq(&socspi->spi_lock);
	if (socspi->spi == NULL)
	{
		status = -ESHUTDOWN;
	}
	else
	{
		status = spi_async(socspi->spi, message);
	}
	spin_unlock_irq(&socspi->spi_lock);

	if (status == 0)
	{
		wait_for_completion(&done);
		status = message->status;
		if(status == 0)
		{
			status = message->actual_length;
		}
	}
	return status;
}

// =====GPIO======
// We use a GPIO Interrupt (called CVInt Pin)  connect to the Master
// The SPI Master can only start the SPI Transaction once the CVInt Pin is low.
tatic irqreturn_t gpio_isr_handle (int irq, void *devid)
{
	// signal to continue transfer data
	// look into socspi_send_command funtion : we call socspi_wait_interrupt function
	// mean that we are waiting event complete from this one.  
	complete(&spi_done);
	gpio_interrupt_clear(irq);
	return IRQ_HANDLED;
}

static void socspi_gpio_init(void)
{
	int ret ;
	struct gpio_interrupt_mode mode = {
		.trigger_method 	= GPIO_INT_TRIGGER_EDGE,
		.trigger_edge_nr 	= GPIO_INT_SINGLE_EDGE,
		.trigger_rise_neg 	= GPIO_INT_FALLING
	};
	ret = gpio_request(GPIO_INT_PIN, "socspi_irq");
	if(ret < 0)
	{
		printk("irq GPIO request failed: %d\n", ret);
	}
	else
	{
		printk("irq GPIO request: %d\n", ret);
	}
	gpio_direction_input(GPIO_INT_PIN);
	printk("GPIO value %d\n", gpio_get_value(GPIO_INT_PIN)); 
	irq = gpio_to_irq (GPIO_INT_PIN);
	gpio_interrupt_enable(irq); 
	gpio_interrupt_setup(irq, &mode);
	ret = request_irq (irq, gpio_isr_handle, 0, "socspi_irq", NULL);
	if(ret)
	{
		printk("request_irq error(%d) = %d\n",irq,ret);
	}
	else
	{
		printk("irq init success : %d\n",ret);
	}

}

static void socspi_gpio_deinit(void)
{
	int irq;
	irq = gpio_to_irq (GPIO_INT_PIN);
	free_irq (irq, NULL);
	gpio_free(GPIO_INT_PIN);
	printk("irq de-init success\n");
}

inline int socspi_wait_interrupt(int ticks)
{
	long timeout;
	timeout = wait_for_completion_killable_timeout(&spi_done, ticks);
	if(timeout == 0)
	{
		printk("wait for interrupt timeout\n");
		return -1;
	}
	else if(timeout == -ERESTARTSYS)
	{
		printk("SPI interrupted\n");
		return -2;
	}
	return 0;
}

inline int socspi_wait_idle(int ticks) 
{
	if(__gpio_get_value(GPIO_INT_PIN) == 0)
	{
		printk("interrupt pin is at low level\n");
		return 0;
	}
	return socspi_wait_interrupt(ticks);
}

static int socspi_DMA_read(struct socspi_data *socspi,
		struct spi_message *p_msg,
		char *p_tx, int len, void *p_rx, bool is_user_cmd)
{
	struct spi_transfer transfer = {
		.cs_change = 1,
		.bits_per_word = socspi->spi->bits_per_word,
		.delay_usecs = 0,
		.speed_hz = socspi->spi->max_speed_hz,
	};
	int ret = -1;
	int retry = 3;
	// wait for gpio pin is low level
	ret = socspi_wait_idle(MS_TO_TICKS(200));
	if(ret < 0)
	{
		return ret;
	}
__READ_COMMAND_LOOP:
	transfer.len = len;
	transfer.tx_buf = NULL;
	transfer.rx_buf = p_rx;

	spi_message_init(p_msg);
	printk("cmd tx : %08x\n",*p_tx);
	spi_message_add_tail(&transfer, p_msg);
	ret = socspi_sync(socspi, p_msg);
	//socspi_wait_interrupt(MS_TO_TICKS(200));

	printk("sync return: %d\n",ret);

	if((ret < 0) && (retry > 0))
	{
		retry--;
		if(kthread_should_stop())
		{
			ret = STATUS_EXIT_MAIN_THREAD;
		}
		else
		{
			printk("try to send cmd again\n");
			goto __READ_COMMAND_LOOP;
		}
	}
	else if(is_user_cmd)
	{
		mutex_lock(&socspi->command_lock);
		socspi->command = 0; // clear command
		mutex_unlock(&socspi->command_lock);
	}
	return ret;
}

static int socspi_send_command(struct socspi_data *socspi,
		struct spi_message *p_msg,
		char *p_tx, int len, void *p_rx, bool is_user_cmd)
{
	struct spi_transfer transfer = {
		.cs_change = 1,
		.bits_per_word = socspi->spi->bits_per_word,
		.delay_usecs = 0,
		.speed_hz = socspi->spi->max_speed_hz,
	};
	int ret = -1;
	int retry = 3;
	// wait for gpio pin is low level
	ret = socspi_wait_idle(MS_TO_TICK(200));
	if(ret < 0)
	{
		return ret;
	}
__SEND_COMMAND_LOOP:

	transfer.len = len;
	transfer.tx_buf = p_tx;
	transfer.rx_buf = p_rx;

	spi_message_init(p_msg);
	spi_message_add_tail(&transfer, p_msg);
	ret = socspi_sync(socspi, p_msg);
	//socspi_wait_interrupt(MS_TO_TICKS(200));

	printk("command tx : %08x\n",*p_tx);
	if((ret < 0) && (retry > 0))
	{
		retry--;
		if(kthread_should_stop())
		{
			ret = STATUS_EXIT_MAIN_THREAD;
		}
		else
		{
			printk("go to send command loop\n");
			goto __SEND_COMMAND_LOOP;
		}
	}
	else if(is_user_cmd)
	{
		mutex_lock(&socspi->command_lock);
		socspi->command = 0; //clear command
		mutex_unlock(&socspi->command_lock);
	}
	return ret;
}

static int socspi_check_command_status(unsigned int  rx_status)
{
	// TODO: do something
	// it's depend on your purpose
	return 1;       	
}

static int socspi_maintask(void *data)
{
	printk("runing socspi maintask\n");
	int ret = 0;
	unsigned int command;
	struct spi_message msg;
	unsigned int rx_status = 0;
	unsigned int audio_busy_jiff = jiffies;
	struct socspi_data *socspi = (struct socspi_data*)data;

	do
	{
		// send command from user space via ioctl 
		if(socspi->command != 0)
		{
			mutex_lock(&socspi->command_lock);
			command = socspi->command;
			mutex_unlock(&socspi->command_lock);
			ret = socspi_send_command(socspi, &msg, (char *)&command, 4, &rx_status, 1);
			if(ret < 0)
			{
				goto __CONTINUE_LOOP;
			}
			if((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK)
			{
				if(socspi_check_command_status(rx_status) == 1)
				{
					socspi->command_status = rx_status;
					wake_up_interruptible(&socspi->command_wq_list);
				}
				if(rx_status & TALKBACK_FULL_MASK)
				{
					// inform that talkback data is coming
					socspi->is_talkback = 1;
					goto __TALKBACK_READING;
				}
			}
			else
			{
				goto __CONTINUE_LOOP;
			}	
		}
__TALKBACK_READING:
		// if talkback is availble
		if(socspi_data->is_talkback == 1)
		{
			// get status of talkback from spi_soc module is true
			// ret : length of that data receive from spi_soc via DMA
			ret = socspi_DMA_read(socspi, &msg, &socspi->buffer,
					TALKBACK_TRANSFER_SIZE, socspi->talkback_buffer, 0);
			if(ret < 0)
				goto __CONTINUE_LOOP;
			if(kfifo_avail(&socspi->talkback_fifo) < ret)
			{
				// current size of tb fifo less than received data
				printk("talkback fifo is overrun. resetting...\n");
				mutex_lock(&socspi->talkback_lock);
				kfifo_reset(&socspi->talkback_fifo);
				mutex_unlock(&socspi->talkback_lock);
			}
			// push data into tb fifo
			mutex_lock(&socspi->talkback_lock);
			kfifo_in(&socspi->talkback_fifo, socspi->talkback_buffer, ret);
			mutex_unlock(&socspi->talkback_lock);
			// if talkback data is waiting in some where
			// wake it up
			if (socspi->talkback_blocked)
			{
				socspi->talkback_blocked = 0;
				wake_up_interruptible(&socspi->talkback_wq_list);
			}
			socspi->is_talkback = 0;

		}
		// if audio buffer is available and the last audio send is more than 40ms
		else if(kfifo_len(&socspi->audio_fifo) >= AUDIO_TRANSFER_SIZE &&
				(jiffies - audio_busy_jiff) > MS_TO_TICKS(20))
		{
			command = SPI_CMD_AUDIO(AUDIO_TRANSFER_SIZE);
			printk("audio send cmd %08x\n",command);
			ret = socspi_send_command(socspi, &msg, (char *)&command, 4, &rx_status, 0);
			if (ret < 0)
			{
				// send cmd fail
				goto __CONTINUE_LOOP;
			}	
			printk("audio status = %08x\n",command);
			if((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK)
			{
				if(socspi_check_command_status(rx_status) == 1)
				{
					socspi->command_status = rx_status;
					wake_up_interruptible(&socspi->command_wq_list);
				}
				if(rx_status & TALKBACK_TRANSFER_SIZE)
				{
					socspi->is_talkback = 1;
					goto __TALKBACK_READING;
				}
			}
			else
				goto __CONTINUE_LOOP;
			// available to send 128byte audio data
			if(!(rx_status & AUDIO_FULL_MASK))
			{
				mutex_lock(&socspi->audio_lock);
				ret = kfifo_out_peek(&socspi->audio_fifo, socspi->audio_buffer, AUDIO_TRANSFER_SIZE);
				mutex_unlock(&socspi->audio_lock);
				if(ret > 0)
				{
					printk("AUDIO_FULL_MASK\n");
					// copy from kfifo to buffer
					// socspi->audio_buffer is that buffer was sent
					ret = socspi_send_command(socspi, &msg, socspi->audio_buffer,AUDIO_FULL_MASK, socspi->buffer, 0);
					if(ret < 0)
					{
						if(ret == STATUS_EXIT_MAIN_THREAD)
							goto __CONTINUE_LOOP;
						else
							goto __CONTINUE_LOOP;
					}
					if(ret != AUDIO_TRANSFER_SIZE)
					{
						// has trouble during send audio data
						audio_busy_jiff = jiffies;
					}
					else
					{
						// send audio data success . remove it from kfifo
						mutex_lock(&socspi->audio_lock);
						ret = kfifo_out(&socspi->audio_fifo, socspi->audio_buffer, AUDIO_TRANSFER_SIZE);
						mutex_unlock(&socspi->audio_lock);
						if((socspi->audio_blocked) && (kfifo_avail(&socspi->audio_fifo) >= socspi->audio_count));
						{
							// permit push data into kfifo audio
							wake_up_interruptible(&socspi->audio_wq_list);
						}
					}
				}
			}
			else
			{
				audio_busy_jiff = jiffies;
				printk("audio buffer is busy \n");
			}
		}
		// if video buffer is available
		if((kfifo_len(&socspi->video_fifo) >= VIDEO_TRANSFER_SIZE))
		{
			command = SPI_CMD_VIDEO(VIDEO_TRANSFER_SIZE);
			ret = socspi_send_command(socspi, &msg, (char *)&command, 4, &rx_status, 0);
			if(ret < 0)
			{
				goto __CONTINUE_LOOP;
			}
			if((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK)
			{
				if(socspi_check_command_status(rx_status) == 1)
				{
					// rx_status is returned data from soc
					socspi->command_status = rx_status;
					wake_up_interruptible(&socspi->command_wq_list);
				}
				if (rx_status & TALKBACK_FULL_MASK)
				{
					// talkback detected
					goto __TALKBACK_READING;
				}
			}
			else
			{
				goto __CONTINUE_LOOP;
			}
			if(!(rx_status & VIDEO_FULL_MASK))
			{
				// it's time to tranfer 512 byte video
				mutex_lock(&socspi->video_lock);
				// get video data from kfifo to video buff
				ret = kfifo_out_peek(&socspi->video_fifo, socspi->video_buffer, VIDEO_TRANSFER_SIZE);
				mutex_unlock(&socspi->video_lock);
			}
			if(ret > 0)
			{
				// every step is 128 byte
				int len = 0;
				while(len < VIDEO_TRANSFER_SIZE)
				{
					// ...,tx, len, rx, ...
					ret = socspi_send_command(socspi, &msg, socspi->video_buffer + len, BULK_TRANSFER_SIZE, socspi->buffer, 0);
					if(ret < 0)
					{
						goto __CONTINUE_LOOP;
					}
					if(ret != BULK_TRANSFER_SIZE)
					{
						// have trouble during transfer video data
					}
					len += BLE_TRANSFER_SIZE;
				}
				if (len == VIDEO_TRANSFER_SIZE)
				{
					mutex_lock(&socspi->video_lock);
					// remove 512 byte video data from kfifo video
					ret = kfifo_out(&socspi->video_fifo, socspi->video_buffer, VIDEO_TRANSFER_SIZE);
					mutex_unlock(&socspi->video_lock);
				}
				if((socspi->video_blocked) && kfifo_avail(&socspi->video_fifo))
				{
					// permit push data into video kfifo
					wake_up_interruptible(&socspi->video_wq_list);
				}
			}
		}
		else
		{
			printk("video buffer is busy\n");
		}
	}
__CONTINUE_LOOP:
	while(!kthread_should_stop());
}	

static ssize_t socspi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	// TODO: use to read audio talkback from spi_soc module
	//
	ssize_t status = 0;
	unsigned int tb_len = 0; // length og tb data
	unsigned int copied = 0; // status of copying tb data
	struct socspi_data *socspi = filp->private_data;
	if(filp->f_flags & O_NONBLOCK)
	{
		if(kfifo_len(&socspi->talkback_fifo) == 0)
			return -EAGAIN;
	}
	else if(kfifo_len(&socspi->talkback_fifo) == 0)
	{
		socspi->talkback_blocked = 1;
		printk("tb wait here : until availible data\n");
		status = wait_event_interruptible(socspi->talkback_wq_list, kfifo_len(&socspi->talkback_fifo) > 0);
		if(status != 0)
			return -ERESTARTSYS;
	}
	tb_len = kfifo_len(&socspi->talkback_fifo);
	tb_len = (tb_len < count)? tb_len : count;
	// put tb data to user space
	if(!kfifo_to_user(&socspi->talkback_fifo, (char __user*)buf, tb_len, &copied))
	{
		status = copied;
	}
	else
	{
		status = -EFAULT;
	}
	return status;
}

static ssize_t socspi_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	// TODO: use to send audio data to module spi_soc
	struct socspi_data *socspi = filp->private_data;
	ssize_t status = 0;
	int copied = 0;
	// chipselect only toggles at start or end of operation
	if(count > bufsiz)
		return -EMSGSIZE;
	// send data is nonblock : from user
	if(filp->f_flags & O_NONBLOCK)
	{
		if(kfifo_avail(&socspi->audio_fifo) < count)
		{
			return -EAGAIN;
		}
	}
	else if (socspi->is_talkback)
	{
		socspi->audio_blocked = 1;
		status = wait_event_interruptible(socspi->audio_wq_list, socspi->is_talkback == 0);
		if(status != 0)
		{
			return -ERESTARTSYS;
		}
	}
	else if(kfifo_avail(socspi->audio_fifo) < count)
	{
		socspi->audio_blocked = 1;
		status = wait_event_interruptible(socspi->audio_wq_list, kfifo_avail(socspi->audio_fifo) >= count);
		if(status != 0)
		{
			return -ERESTARTSYS;
		}
	}
	// put audio data from user to kernel
	mutex_lock(&socspi->buf_lock);
	status = kfifo_from_user(&socspi->audio_fifo, (void __user *) buf, count, &copied);
	mutex_unlock(&socspi->buf_lock);
	if(status != 0)
	{
		return -EIO;
	}
	else if(count != copied)
	{
		return -EFAULT;
	}
	return copied;	
}

static unsigned int socspi_poll(struct file *filp, struct poll_table_struct *pwait)
{
	// TODO : poll tb data
	unsigned int mask = 0 ;
	struct socspi_data *socspi = (struct socspi_data*)filp->private_data;
	socspi->talkback_blocked = 1;
	poll_wait(filp, &socspi->talkback_wq_list, pwait);

	if(kfifo_len(&socspi->talkback_fifo) > 0)
	{
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

static long socspi_ioctl(struct file *filp, unsigned int cmd, unsgned long arg)
{
	// TODO: use to send video data and command io
	struct socspi_data *socspi;
	struct spi_device *spi;
	struct socspi_transfer req;
	int ret = 0;

	socspi = filp->private_data;
	spin_lock_irq(&socspi->spi_lock);
	spi = spi_dev_get(socspi->spi);
	spin_unlock_irq(&socspi->spi_lock);
	if(spi == NULL)
	{
		return -ESHUTDOWN;
	}
	switch(cmd){
		// we have a lot of COMMAND here. but i just focus on these command below
		case SOCSPI_VIDEO_TRANSFER:
			ret = copy_from_user(&req, (void __user *)arg, sizeof(struct socspi_transfer));
			if(ret != 0)
			{
				ret = -EFAULT;
				break;
			}
			if(kfifo_avail(&socspi->video_fifo) < 0 || kfifo_avail(&socspi->video_fifo) < req.len)
			{
				printk("buffer video already full or available buff video less than that data requested from user\n");
				socspi->video_blocked = 1;
				ret = wait_event_interruptible_timeout(socspi->video_wq_list, kfifo_avail(&socspi->video_fifo) >= req.len);
				socspi->video_blocked = 0;
			}
			else
			{
				printk("kfifo video not full yet. continue push data to buffer\n");
				ret = kfifo_from_user(&socspi->video_fifo, (void __user *) req.len, &copied);
				if (ret != 0)
				{
					// push data fail
					ret = -EIO;
				}
				else if(copied != req.len)
				{
					printk("copy video data fail %d != %d\n",creq.len, copied);
					ret -EFAULT;
				}
				else
				{
					ret = req.len;
				}
			}	
			break;
		case SOCSPI_COMMAND_TRANSFER:
			if (copy_from_user(&req, (void __user *)arg, sizeof(struct socspi_transfer)))
			{
				ret = -EFAULT;
				break;
			}
			// wait until socspi->command = 0 or timeout
			// for first time: socspi->command = 0
			ret = wait_event_interruptible_timeout(socspi->command_wq_list,
					socspi->command == 0, HZ);
			if(ret > 0)
			{
				mutex_lock(&socspi->command_lock);
				socspi->command = (unsigned int)req.data;
				mutex_unlock(&socspi->command_lock);
				ret = req.len;
			}
			else
			{
				ret = -ETIMEDOUT;
			}
			break;
	 	case SOCSPI_STATUS_TRANSFER:
			ret = wait_event_interruptible_timeout(socspi->command_wq_list,
					socspi->command_status != 0, HZ);
			if(ret > 0)
			{
				mutex_lock(&socspi->command_lock);
				req.data = (void*)socspi->command_status;
				socspi->command_status = 0;
				mutex_unlock(&socspi->command_lock);
				// push data status to user space
				if(copy_to_user((void __user*)arg, &req, sizeof(struct socspi_transfer)))
				{
					ret = -EFAULT;
				}
				else
				{
					ret = 0;
				}
			}
			else
			{
				printk("update status from spi_soc timeout\n");
				ret = -ETIMEDOUT;
			}
			break;
		default:
			// TODO: do something 
			break;		
	}
	spi_dev_put(spi);
	return ret;
}

static int socspi_open(struct inode *inode, struct file *filp)
{
	// TODO:
	//  need reuse exits code of spidev.c
	//  need get pointer of socspi 
	//  do it later
	struct socspi_data *socspi ;
	unsigned int status = -ENXIO;
	printk("open socspi\n");
	mutex_lock(&device_list_lock);
	// add source code here
	if(status == 0)
	{
		filp->private_data = socspi;
		nonseekable_open(inode, filp);

		socspi->video_blocked = 0;
		socspi->audio_blocked = 0;
		socspi->talkback_blocked = 0;

		maintask = kthread_run(socspi_maintask, (void*)socspi, "socspi");

	}	

	mutex_unlock(&device_list_lock);
}

static int socspi_release(struct inoede *inode, struct file *filp)
{
	// TODO: handle when user call function close
	struct socspi_data *socspi;
	int 	status = 0;
	mutex_lock(&device_list_lock);
	socspi = filp->private_data;
	filp->private_data = NULL;

	if (socspi->users > 0)
		socspi->users--;
	/* last close? */
	if (!socspi->users) 
	{
		int 	dofree;
		if(maintask != NULL) 
		{
			// stop main thread
			kthread_stop(maintask);
			maintask = NULL;
			printk("stopped maintask\n");
		}
		kfree(socspi->buffer);
		socspi->buffer = NULL; 
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&socspi->spi_lock);
		dofree = (socspi->spi == NULL);
		spin_unlock_irq(&socspi->spi_lock);
		if (dofree)
		{
			kfifo_free(&socspi->video_fifo);
			kfifo_free(&socspi->audio_fifo);
			kfifo_free(&socspi->talkback_fifo);
			kfifo_free(&socspi->u_read_fifo);
			kfree(socspi);
		}
	}
	mutex_unlock(&device_list_lock);
	printk("release socspi\n");
	return status;
}

static const struct file_operations socspi_fops = {
	.owner = 		THIS_MODULE,
	.write = 		socspi_write,
	.read = 		socspi_read,
	.unlocked_ioctl = 	socspi_ioctl,
	.compat_ioctl = 	NULL,
	.open = 		socspi_open,
	.release = 		socspi_release,
	.llseek = 		no_llseek,
	.poll = 		socspi_poll,
}

static int __devinit socspi_probe(struct spi_device *spi)
{
	// TODO :
	struct socspi_data *socspi;
	int status;
	unsigned long minor;

	// allocate driver data
	socspi = kzalloc(sizeof(*socspi), GFP_KERNEL);
	if (!socspi)
		return -ENOMEM;
	// initialize the driver data
	socspi->spi = spi;
	// hard code spi setting
	spi->bits_per_word = 8;
	spi->max_speed_hz = 1500000;
	spi->chip_select = 0;
	spi->mode = SPI_MODE_3;

	status = kfifo_alloc(&socspi->video_fifo, VIDEO_QUEUE_SIZE, GFP_KERNEL);
	if(status)
	{
		printk("video fifo allocate fail\n");
		return status;
	}

	status = kfifo_alloc(&socspi->audio_fifo, AUDIO_QUEUE_SIZE, GFP_KERNEL);
	if(status)
	{
		printk("audio fifo allocate fail\n");
		return status;
	}
	
	status = kfifo_alloc(&socspi->talkback_fifo, TALKBACK_QUEUE_SIZE, GFP_KERNEL);
	if(status)
	{
		printk("talkback fifo allocate fail\n");
		return status;
	}

	status = kfifo_alloc(&socspi->u_read_fifo, U_QUEUE_SIZE, GFP_KERNEL);
	if(status)
	{
		printk("u_read fifo allocate fail\n");
		return status;
	}

	printk("initial waitqueue head\n");

	init_waitqueue_head(&socspi->command_wq_list);
	init_waitqueue_head(&socspi->talkback_wq_list);
	init_waitqueue_head(&socspi->audio_wq_list);
	init_waitqueue_head(&socspi->video_wq_list);
	init_waitqueue_head(&socspi->u_read_wq_list);
	init_waitqueue_head(&socspi->u_write_wq_list);

	mutex_init(&socspi->audio_lock);
	mutex_init(&socspi->video_lock);
	mutex_init(&socspi->talkback_lock);
	mutex_init(&socspi->command_lock);
	mutex_init(&socspi->u_read_lock);
	mutex_init(&socspi->buf_lock);

	spin_lock_init(&socspi->spi_lock);


	mutex_lock(&device_list_lock);

	// TODO : device_create
	// reuse source code of spidev.c fot this part
	// do it later
	
	mutex_unlock(&device_list_lock);
	
	if (status == 0)
	{
		socspi_gpio_init();
	}
	else
		kfree(socspi);
	return status;	
}

static int __devexit socspi_remove(struct spi_device *spi)
{
	struct socspi_data = *socspi = spi_get_drvdata(spi);
	socspi_gpio_deinit();

	kfifo_free(&socspi->video_fifo);
	kfifo_free(&socspi->audio_fifo);
	kfifo_free(&socspi->talkback_fifo);
	kfifo_free(&socspi->u_read_fifo);

	// reuse exits code of spidev
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_init(&socspi->spi_lock);
	socspi->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&socspi->spi_lock);

	// prevent new opens
	mutex_lock(&device_list_lock);
	list_del(&socspi->device_entry);
	device_destroy(socspi_class, socspi_devt);
	clear_bit(MINOR(socspi->devt), minors);

	if(socspi->user == 0)
		kfree(socspi);
	mutex_unlock(&device_list_lock);
	return 0;
	
}

static struct spi_driver socspi_spi_driver = {
	.driver = {
		.name = 	DEV_NAME,
		.owner = 	THIS_MODULE,
	},
	.probe = 	socspi_probe,
	.remove = 	__devexit_p(socspi_remove),
};

static int __init socspi_init(void)
{
	int status;
	BUILD_BUG_ON (N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &socspi_fops);
	if(status < 0)
		return status;
	socspi_class = class_create(THIS_MODULE, DEV_NAME);
	if(IS_ERR(socspi_class))
	{
		unregister_chrdev(SPIDEV_MAJOR, socspi_spi_driver.driver.name);
		return PTR_ERR(socspi_class);
	}
	status = spi_register_driver(&socspi_spi_driver);
	if (status < 0)
	{
		class_destroy(socspi_class);
		unregister_chrdev(SPIDEV_MAJOR, socspi_spi_driver.driver.name);
	}
	return status;

}

static void __exit socspi_exit(void)
{
	spi_unregister_driver(&socspi_spi_driver);
	class_destroy(socspi_class);
	unregister_chrdev(SPIDEV_MAJOR, socspi_spi_driver.driver.name);
}

module_init(socspi_init);
module_exit(socspi_exit);

MODULE_AUTHOR("Ngoc Gia <ngocgia73@gmail.com>");
MODULE_DESCRIPTION("SPI_SOC device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spi_socspi");
