/*
 * Driver for the OLPC XO-1.75 Embedded Controller.
 *
 * The EC protocol is documented at:
 * http://wiki.laptop.org/go/XO_1.75_HOST_to_EC_Protocol
 *
 * Copyright (C) 2010 One Laptop per Child Foundation.
 * Copyright (C) 2018 Lubomir Rintel <lkundrak@v3.sk>
 *
 * Licensed under the GPL v2 or later.
 */

// echo -n 08:1 00 >/sys/kernel/debug/olpc-ec/cmd; cat /sys/kernel/debug/olpc-ec/cmd
// echo -n 08:1 >/sys/kernel/debug/olpc-ec/cmd; cat /sys/kernel/debug/olpc-ec/cmd

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/olpc-ec.h>
#include <linux/spi/spi.h>
//#include <linux/sched.h>
//#include <linux/power_supply.h>
#include <linux/input.h>
#include <linux/kfifo.h>
#include <linux/module.h>

#define EC_CMD_LEN              8
#define EC_MAX_RESP_LEN         16









#if 0

/*
 * manage and receive EC event notifications
 */
void olpc_ec_disable_wakeup(int event);
void olpc_ec_register_event_callback(void (*f)(int),int);
void olpc_ec_deregister_event_callback(int);

#endif

/*
 * EC commands (from http://dev.laptop.org/git/users/rsmith/ec-1.75/tree/ec_cmd.h)
 */
#define CMD_GET_API_VERSION              0x08  // out: u8 
#define CMD_READ_VOLTAGE                 0x10  // out: u16, *9.76/32,    mV
# define EC_BAT_VOLTAGE CMD_READ_VOLTAGE                 
#define CMD_READ_CURRENT                 0x11  // out: s16, *15.625/120, mA
# define EC_BAT_CURRENT CMD_READ_CURRENT                 
#define CMD_READ_ACR                     0x12  // out: s16, *6250/15,    µAh
# define EC_BAT_ACR CMD_READ_ACR                     
#define CMD_READ_BATT_TEMPERATURE        0x13  // out: u16, *100/256,   deg C
# define EC_BAT_TEMP CMD_READ_BATT_TEMPERATURE        
#define CMD_READ_AMBIENT_TEMPERATURE     0x14  // unimplemented, no hardware
# define EC_AMB_TEMP CMD_READ_AMBIENT_TEMPERATURE     
#define CMD_READ_BATTERY_STATUS          0x15  // out: u8, bitmask
# define EC_BAT_STATUS CMD_READ_BATTERY_STATUS          
#define CMD_READ_SOC                     0x16  // out: u8, percentage
# define EC_BAT_SOC CMD_READ_SOC                     
#define CMD_READ_GAUGE_ID                0x17  // out: u8 * 8    
# define EC_BAT_SERIAL CMD_READ_GAUGE_ID                
#define CMD_READ_GAUGE_DATA              0x18  // in: u8 addr, out: u8 data 
# define EC_BAT_EEPROM CMD_READ_GAUGE_DATA              
#define CMD_READ_BOARD_ID                0x19  // out: u16 (platform id) 
#define CMD_READ_BATT_ERR_CODE           0x1f  // out: u8, error bitmask
# define EC_BAT_ERRCODE CMD_READ_BATT_ERR_CODE           
#define CMD_SET_DCON_POWER               0x26  // in: u8 
#define CMD_RESET_EC                     0x28  // none 
#define CMD_READ_BATTERY_TYPE            0x2c  // out: u8  
#define CMD_SET_AUTOWAK                  0x33  // out: u8 
#define CMD_SET_EC_WAKEUP_TIMER          0x36  // in: u32, out: ? 
#define CMD_READ_EXT_SCI_MASK            0x37  // ? 
#define CMD_WRITE_EXT_SCI_MASK           0x38  // ? 
#define CMD_CLEAR_EC_WAKEUP_TIMER        0x39  // none 
#define CMD_ENABLE_RUNIN_DISCHARGE       0x3B  // none 
#define CMD_DISABLE_RUNIN_DISCHARGE      0x3C  // none 
#define CMD_READ_MPPT_ACTIVE             0x3d  // out: u8 
#define CMD_READ_MPPT_LIMIT              0x3e  // out: u8 
#define CMD_SET_MPPT_LIMIT               0x3f  // in: u8 
#define CMD_DISABLE_MPPT                 0x40  // none 
#define CMD_ENABLE_MPPT                  0x41  // none 
#define CMD_READ_VIN                     0x42  // out: u16 
#define CMD_EXT_SCI_QUERY                0x43  // ? 
#define RSP_KEYBOARD_DATA                0x48  // ? 
#define RSP_TOUCHPAD_DATA                0x49  // ? 
#define CMD_GET_FW_VERSION               0x4a  // out: u8 * 16 
#define CMD_POWER_CYCLE                  0x4b  // none 
#define CMD_POWER_OFF                    0x4c  // none 
#define CMD_RESET_EC_SOFT                0x4d  // none 
#define CMD_READ_GAUGE_U16               0x4e  // ? 
#define CMD_ENABLE_MOUSE                 0x4f  // ? 
#define CMD_JUMP_TO_UPDATE               0x50  // (A1 only)  out: u8 
#define CMD_UPDATE_FLASH                 0x51  // (A1 only) 
#define CMD_ECHO                         0x52  // in: u8 * 5, out: u8 * 5 
#define CMD_GET_FW_DATE                  0x53  // out: u8 * 16 
#define CMD_GET_FW_USER                  0x54  // out: u8 * 16 
#define CMD_TURN_OFF_POWER               0x55  // none (same as 0x4c)
#define CMD_READ_OLS                     0x56  // out: u16 
#define CMD_OLS_SMT_LEDON                0x57  // none 
#define CMD_OLS_SMT_LEDOFF               0x58  // none 
#define CMD_START_OLS_ASSY               0x59  // none 
#define CMD_STOP_OLS_ASSY                0x5a  // none 
#define CMD_OLS_SMTTEST_STOP             0x5b  // none 
#define CMD_READ_VIN_SCALED              0x5c  // out: u16
#define CMD_READ_BAT_MIN_W               0x5d  // out: u16
#define CMD_READ_BAR_MAX_W               0x5e  // out: u16
#define CMD_RESET_BAT_MINMAX_W           0x5f  // none

#define CMD_READ_LOCATION                0x60  // in: u16 addr, out: u8 data 
#define CMD_WRITE_LOCATION               0x61  // in: u16 addr, u8 data 
#define CMD_KEYBOARD_CMD                 0x62  // in: u8, out: ? 
#define CMD_TOUCHPAD_CMD                 0x63  // in: u8, out: ? 
#define CMD_GET_FW_HASH                  0x64  // out: u8 * 16 
#define CMD_SUSPEND_HINT                 0x65  // in: u8
#define CMD_ENABLE_WAKE_TIMER            0x66  // in: u8
#define CMD_SET_WAKE_TIMER               0x67  // in: 32
#define CMD_ENABLE_WAKE_AUTORESET        0x68  // in: u8
#define CMD_OLS_SET_LIMITS               0x69  // in: u16, u16
#define CMD_OLS_GET_LIMITS               0x6a  // out: u16, u16
#define CMD_OLS_SET_CEILING              0x6b  // in: u16
#define CMD_OLS_GET_CEILING              0x6c  // out: u16



/*
 * EC events
 */
#define EVENT_AC_CHANGE            1    // AC plugged/unplugged 
#define EVENT_BATTERY_STATUS       2    // Battery low/full/error/gone 
#define EVENT_BATTERY_CRITICAL     3    // Battery critical voltage 
#define EVENT_BATTERY_SOC_CHANGE   4    // 1% SOC Change 
#define EVENT_BATTERY_ERROR        5    // "Abnormal" error, query for cause 
#define EVENT_POWER_PRESSED        6    // power button was pressed 
#define EVENT_POWER_PRESS_WAKE     7    // power button pressed during sleep
#define EVENT_TIMED_HOST_WAKE      8    // Host wake timer
#define EVENT_OLS_HIGH_LIMIT       9    // OLS crossed dark threshold
#define EVENT_OLS_LOW_LIMIT       10    // OLS crossed light threshold










static bool ec_debug;
module_param(ec_debug, bool, 0644);

enum ec_chan_t {
	CHAN_NONE = 0,
	CHAN_SWITCH,
	CHAN_CMD_RESP,
	CHAN_KEYBOARD,
	CHAN_TOUCHPAD,
	CHAN_EVENT,
	CHAN_DEBUG,
	CHAN_CMD_ERROR,
};

#define EC_CMD_LEN		8
#define EC_MAX_RESP_LEN		16

#define LOG_BUF_SIZE		127

#define OLPC_MAX_EC_EVENT	16  // limited by size of EC mask

enum ec_state_t {
	CMD_STATE_IDLE = 0,
	CMD_STATE_WAITING_FOR_SWITCH,
	CMD_STATE_CMD_IN_TX_FIFO,
	CMD_STATE_CMD_SENT,
	CMD_STATE_RESP_RECEIVED,
	CMD_STATE_ERROR_RECEIVED,
};


static struct platform_device *olpc_ec;



struct olpc_xo175_ec {
	struct spi_device *spi;

	struct completion finished;
	struct spi_transfer xfer;
	struct spi_message msg;

	u8 tx_buf[EC_CMD_LEN];
	u8 rx_buf[EC_MAX_RESP_LEN];
	//u8 cmd_buf[EC_CMD_LEN];
	//int cmd_len;



	bool flummoxed;
	bool suspended;

	/*
	 * GPIOs for ACK and CMD signals.
	 */
	struct gpio_desc *gpio_ack;
	struct gpio_desc *gpio_cmd;

	/*
	 * GPIO for the EC's wakeup signal, and a place
	 * to save pin config when enabling wakeups.
	 */
	//int gpio_ecirq;
	//unsigned long save_ecirq_mfpr;

	/*
	 * Receive state machine state.
	 */
	int pkts_to_ignore;

	/*
	 * Command handling related state.
	 */
	spinlock_t cmd_state_lock;
	int cmd_state;
	bool cmd_running;
	struct completion cmd_done;
	u8 cmd[EC_CMD_LEN];
	int expected_resp_len;
	u8 resp[EC_MAX_RESP_LEN];
	int resp_len;

	/*
	 * Power button.
	 */
	struct input_dev *pwrbtn;

	/*
	 * Wakeup mask.  (not an event mask -- just wakeups)
	 */
	unsigned int wakeup_mask;

	/*
	 * Other drivers register callbacks for event notifications.
	 * Those callbacks run on a workqueue.
	 */
	void (*olpc_ec_event_callback[OLPC_MAX_EC_EVENT])(int);
	struct work_struct event_queue_work;
	struct kfifo event_queue;  // for passing events to the workqueue

	/*
	 * Debug handling.
	 */
	char logbuf[LOG_BUF_SIZE + 1];
	int logbuf_len;
};


DEFINE_SPINLOCK(olpc_ec_event_callback_lock);


static void invoke_olpc_ec_event_callback(struct olpc_xo175_ec *ec, unsigned int event)
{
	void (*cbk)(int);

	if (event >= OLPC_MAX_EC_EVENT)
		return;
	
	spin_lock(&olpc_ec_event_callback_lock);
	cbk = ec->olpc_ec_event_callback[event];
	spin_unlock(&olpc_ec_event_callback_lock);

	if (cbk)
		cbk(event);
}

#if 1
/*
 * a driver interested in hearing specific EC events
 * should register for each of those events.
 */
void olpc_ec_register_event_callback(void (*f)(int), int event)
{
	if (event >= OLPC_MAX_EC_EVENT)
		return;
	
	spin_lock(&olpc_ec_event_callback_lock);
	//olpc_ec->olpc_ec_event_callback[event] = f;
	spin_unlock(&olpc_ec_event_callback_lock);
}
EXPORT_SYMBOL_GPL(olpc_ec_register_event_callback);

void olpc_ec_deregister_event_callback(int event)
{
	if (event >= OLPC_MAX_EC_EVENT)
		return;
	
	spin_lock(&olpc_ec_event_callback_lock);
	//olpc_ec->olpc_ec_event_callback[event] = NULL;
	spin_unlock(&olpc_ec_event_callback_lock);
}
EXPORT_SYMBOL_GPL(olpc_ec_deregister_event_callback);

/*
 * this should be called from a driver's suspend routine, to
 * keep the given event from causing a wakeup.  the default is
 * for all events to cause wakeups, and that condition is
 * reestablished automatically during resume.
 */
void olpc_ec_disable_wakeup(int event)
{
	if (event >= OLPC_MAX_EC_EVENT)
		return;
	
	// FIXME -- the per-platform event numbering won't be the
	// same as the global OLPC event numbering, so we'll need a
	// mapping.  also, not all events may exist on all platforms.
	//olpc_ec->wakeup_mask &= ~(1 << event);
}
EXPORT_SYMBOL_GPL(olpc_ec_disable_wakeup);
#endif


static void olpc_xo175_ec_flush_logbuf(struct olpc_xo175_ec *ec)
{
	ec->logbuf[ec->logbuf_len] = 0;
	ec->logbuf_len = 0;

	dev_dbg(&ec->spi->dev, "got debug string [%s]\n", ec->logbuf);
}

static void olpc_xo175_ec_strobe_ack(struct olpc_xo175_ec *ec)
{
#if 0
	u8 txlen = (readl(ec->base + SSSR) >> 8) & 0xf;
	/* before we ACK, we should always have either 2 or 8 bytes in TxFIFO */
	WARN_ON(txlen != 2 && txlen != 8);
#endif

	gpiod_set_value_cansleep(ec->gpio_ack, 1);
	udelay(1);
	gpiod_set_value_cansleep(ec->gpio_ack, 0);
}

static void olpc_xo175_ec_complete(void *arg);

static void olpc_xo175_ec_send_command(struct olpc_xo175_ec *ec, u8 *cmd, size_t cmdlen)
{
	int ret;

	{
		int i;

		printk ("XXX SEND COMMAND!!! len=%d", cmdlen);
		for (i = 0; i < cmdlen; i++)
			printk (" [%02x]", cmd[i]);
		printk ("\n");
	}

	memcpy (ec->tx_buf, cmd, cmdlen);
	ec->xfer.len = cmdlen;

	spi_message_init_with_transfers(&ec->msg, &ec->xfer, 1);

	ec->msg.complete = olpc_xo175_ec_complete;
	ec->msg.context = ec;

	ret = spi_async(ec->spi, &ec->msg);
	if (ret)
		dev_err(&ec->spi->dev, "spi_async() failed %d\n", ret);

	olpc_xo175_ec_strobe_ack(ec);
//	return ret;


#if 0
	int i;

 	/*
	 * The falling edge on ACK (below) tells the EC to transfer exactly 8
	 * bytes over the SPI bus.  This routine puts exactly 2 or 8 bytes into
	 * the transmit FIFO, and the FIFO must be empty before we do so, otherwise
	 * the EC will get the wrong bytes.  If the SoC and the EC remain in sync,
	 * the following clause should not be invoked.  If the EC should time out
	 * while CMD remains high - which could happen if the kernel ignores interrupts
	 * for a long time - the EC will re-send a SWITCH packet, probably causing
	 * a TX underrun and making the FIFO wrap around to not-empty.  In that
	 * case, the correct recovery is to force-empty the FIFO so we put our
	 * data in the right place.
	 */
	if ((readl(ec->base + SSSR) & (SSSR_TFL_MASK | SSSR_TNF)) != SSSR_TNF) {
		dev_warn(&ec->spi->dev, "transmit FIFO not empty\n");

		/* clear the fifo by disabling/enabling SSE */
		olpc_xo175_ec_reset_sse(ec);
	}

	for (i = 0; i < cmdlen; i++)
		writel(cmd[i], ec->base + SSDR);

	olpc_xo175_ec_strobe_ack(ec);
#endif
}

static void olpc_xo175_ec_prime_fifo(struct olpc_xo175_ec *ec)
{
	u8 nonce[] = {0xAB,0xBA};

	olpc_xo175_ec_send_command(ec, nonce, sizeof(nonce));
}

static void event_queue_worker(struct work_struct *work)
{
	unsigned char ev;
	int ret;
	struct olpc_xo175_ec *ec = container_of(work, struct olpc_xo175_ec,
			event_queue_work);

	// FIXME -- if we don't call out to deliver events, then
	// registrants that might call pm_wakeup_event() because
	// of this event won't be called, and a suspend might not
	// be aborted which should be.  i think we need to ignore
	// ec-suspended here, invoke the callback, and rely on
	// the -EBUSY the caller will get if they try for
	// olpc_ec_cmd() to prevent mischief.
	while (!ec->suspended) {

		ret = kfifo_out(&ec->event_queue, &ev, sizeof(ev));
		if (!ret)
			break;

		invoke_olpc_ec_event_callback(ec, ev);
	}

}

static void olpc_xo175_ec_received_pkt(struct olpc_xo175_ec *ec, u8 channel, u8 byte)
{
	unsigned long flags;

	printk ("XXX [%02x] [%02x]\n", channel, byte);

	switch (channel) {

	case CHAN_NONE:
		printk (" * CHAN NONE\n");
		spin_lock_irqsave(&ec->cmd_state_lock, flags);

		if (!ec->cmd_running || !ec->pkts_to_ignore) {
			/* we can safely ignore these */
			dev_err(&ec->spi->dev, "spurious FIFO read packet?\n");
			spin_unlock_irqrestore(&ec->cmd_state_lock, flags);
			return;
		}


#if 0
		// the EC zero-fills the fifo when expecting our command,
		// and we toss them as we read them.
		ec->pkts_to_ignore--;
		if (ec->pkts_to_ignore <= 0) {
			ec->cmd_state = CMD_STATE_CMD_SENT;
			if (!ec->expected_resp_len)
				complete(&ec->cmd_done);
			olpc_xo175_ec_prime_fifo(ec);
			ec->pkts_to_ignore = 0;
		}
#endif
printk (">> pkts_to_ignore=%d xfer.len=%d\n", ec->pkts_to_ignore, ec->xfer.len);
			ec->cmd_state = CMD_STATE_CMD_SENT;
			if (!ec->expected_resp_len)
				complete(&ec->cmd_done);
			olpc_xo175_ec_prime_fifo(ec);
			ec->pkts_to_ignore = 0;

		spin_unlock_irqrestore(&ec->cmd_state_lock, flags);
		return;

	case CHAN_SWITCH:
		printk (" * CHAN SWITCH\n");
		spin_lock_irqsave(&ec->cmd_state_lock, flags);

		if (!ec->cmd_running) {
			/* just go with the flow */
			dev_err(&ec->spi->dev, "spurious SWITCH packet?\n");
			memset(ec->cmd, 0, sizeof(ec->cmd));
			ec->cmd[0] = CMD_ECHO;
		}

		ec->cmd_state = CMD_STATE_CMD_IN_TX_FIFO;
		ec->pkts_to_ignore = 4;

		/* throw command into TxFIFO */
		gpiod_set_value_cansleep(ec->gpio_cmd, 0);
		olpc_xo175_ec_send_command(ec, ec->cmd, sizeof(ec->cmd));

		spin_unlock_irqrestore(&ec->cmd_state_lock, flags);
		return;

	case CHAN_CMD_RESP:
		printk (" * CHAN CMD_RESP\n");
		spin_lock_irqsave(&ec->cmd_state_lock, flags);

		if (!ec->cmd_running) {
			dev_err(&ec->spi->dev, "spurious response packet?\n");
		} else if (ec->resp_len >= ec->expected_resp_len) {
			dev_err(&ec->spi->dev, "too many response packets from EC?\n");
		} else {
			ec->resp[ec->resp_len++] = byte;

			if (ec->resp_len == ec->expected_resp_len) {
				ec->cmd_state = CMD_STATE_RESP_RECEIVED;
				complete(&ec->cmd_done);
			}
		}

		spin_unlock_irqrestore(&ec->cmd_state_lock, flags);
		break;

	case CHAN_CMD_ERROR:
		printk (" * CHAN CMD_ERR\n");
		spin_lock_irqsave(&ec->cmd_state_lock, flags);

		if (!ec->cmd_running) {
			dev_err(&ec->spi->dev, "spurious cmd error packet?\n");
		} else {
			ec->resp[0] = byte;
			ec->resp_len = 1;
			ec->cmd_state = CMD_STATE_ERROR_RECEIVED;
			complete(&ec->cmd_done);
		}
		spin_unlock_irqrestore(&ec->cmd_state_lock, flags);
		break;

	case CHAN_KEYBOARD:
	case CHAN_TOUCHPAD:
		dev_warn(&ec->spi->dev, "EC-connected kbd/tpad not supported\n");
		break;

	case CHAN_EVENT:
		if (ec_debug)
			dev_dbg(&ec->spi->dev, "got event %.2x\n", byte);
		switch (byte) {

		case EVENT_POWER_PRESSED:
			input_report_key(ec->pwrbtn, KEY_POWER, 1);
			input_sync(ec->pwrbtn);
			input_report_key(ec->pwrbtn, KEY_POWER, 0);
			input_sync(ec->pwrbtn);
			// fall through

		case EVENT_POWER_PRESS_WAKE:
		case EVENT_TIMED_HOST_WAKE:
			pm_wakeup_event(ec->pwrbtn->dev.parent, 1000);
		    	break;

		default:
			kfifo_in(&ec->event_queue, &byte, sizeof(byte));
			schedule_work(&ec->event_queue_work);
			break;
		}
		break;

	case CHAN_DEBUG:
		if (byte == '\n') {
			olpc_xo175_ec_flush_logbuf(ec);
		} else if (isprint(byte)) {
			ec->logbuf[ec->logbuf_len++] = byte;
			if (ec->logbuf_len == LOG_BUF_SIZE)
				olpc_xo175_ec_flush_logbuf(ec);
		}
		break;

	default:
		dev_warn(&ec->spi->dev, "got packet for unknown channel: %d, %.2x\n", channel, byte);
		break;
	}

	/* most non-command packets get the TxFIFO refilled and an ACK.. */
	olpc_xo175_ec_prime_fifo(ec);
}


static void quiesce_ec(struct olpc_xo175_ec *ec)
{
	unsigned long flags;

	spin_lock_irqsave(&ec->cmd_state_lock, flags);

	/* leave the command line in a known state */
	gpiod_set_value_cansleep(ec->gpio_cmd, 0);

	/* allow the EC to move on, if it was waiting on an ACK */
	olpc_xo175_ec_strobe_ack(ec);

	/*
	 * Leaving ACK high will tell the EC not to send us anything (so we
	 * shouldn't get new interrupts).  Any outstanding EC commands will
	 * time out.
	 */
	gpiod_set_value_cansleep(ec->gpio_ack, 1);

	/* clear the fifo by disabling/enabling SSE */
	//olpc_xo175_ec_reset_sse(ec);

	spin_unlock_irqrestore(&ec->cmd_state_lock, flags);
}

static void olpc_xo175_ec_complete(void *arg)
{
	struct olpc_xo175_ec *ec = arg;
	int ret;

	ret = ec->msg.status;
	if (ret)
		goto terminate;



#if 0
	ec->xfer.len = 2;
	ret = olpc_xo175_ec_submit(ec);
	if (ret)
		goto terminate;

	/* We're ready to get another command. */
	gpiod_set_value_cansleep(ec->gpio_ack, 1);
	udelay(1);
	gpiod_set_value_cansleep(ec->gpio_ack, 0);

	gpiod_set_value_cansleep(ec->gpio_cmd, 0);

	return;

terminate:
	dev_info(&ec->spi->dev, "Terminating\n");
	complete(&ec->finished);
#endif

	/*
	 * If we're confused, ignore all packets, and keep trying to
	 * quiesce the GPIO lines.  The EC will get the idea in short
	 * order and return to a (non-broken) UPSTREAM mode.  Meanwhile,
	 * any outstanding commands will simply time out.
	 */
	if (ec->flummoxed) {
		quiesce_ec(ec);
		return;
	}

	/* the fifo threshold is set to two, so there are (at least)
	 * two bytes available.  don't read more.
	 */
	olpc_xo175_ec_received_pkt (ec, ec->rx_buf[0], ec->rx_buf[1]);

	return;

terminate:
	dev_info(&ec->spi->dev, "Terminating\n");
	complete(&ec->finished);
}

















struct ec_cmd_t {
	u8 cmd;
	u8 bytes_returned;
};

/*
 * Accepted EC commands, and how many bytes they return.  There are plenty
 * of EC commands that are no longer implemented, or are implemented only on
 * certain older boards.
 */
static const struct ec_cmd_t olpc_xo175_ec_cmds[] = {
	{ CMD_GET_API_VERSION, 1 },
	{ CMD_READ_VOLTAGE, 2 },
	{ CMD_READ_CURRENT, 2 },
	{ CMD_READ_ACR, 2 },
	{ CMD_READ_BATT_TEMPERATURE, 2 },
	{ CMD_READ_BATTERY_STATUS, 1 },
	{ CMD_READ_SOC, 1 },
	{ CMD_READ_GAUGE_ID, 8 },
	{ CMD_READ_GAUGE_DATA, 1 },
	{ CMD_READ_BOARD_ID, 2 },
	{ CMD_READ_BATT_ERR_CODE, 1 },
	{ CMD_SET_DCON_POWER, 0 },
	{ CMD_RESET_EC, 0 },
	{ CMD_READ_BATTERY_TYPE, 1 },
	{ CMD_ENABLE_RUNIN_DISCHARGE, 0 },
	{ CMD_DISABLE_RUNIN_DISCHARGE, 0 },
	{ CMD_READ_MPPT_ACTIVE, 1 },
	{ CMD_READ_MPPT_LIMIT, 1 },
	{ CMD_SET_MPPT_LIMIT, 0 },
	{ CMD_DISABLE_MPPT, 0 },
	{ CMD_ENABLE_MPPT, 0 },
	{ CMD_READ_VIN, 2 },
	{ CMD_GET_FW_VERSION, 16 },
	{ CMD_POWER_CYCLE, 0 },
	{ CMD_POWER_OFF, 0 },
	{ CMD_RESET_EC_SOFT, 0 },
#ifdef A1_ONLY
	{ CMD_JUMP_TO_UPDATE, 1 },
	{ CMD_UPDATE_FLASH, 0 },
#endif
	{ CMD_ECHO, 5 },
	{ CMD_GET_FW_DATE, 16 },
	{ CMD_GET_FW_USER, 16 },
	{ CMD_TURN_OFF_POWER, 0 },
	{ CMD_READ_OLS, 2 },
	{ CMD_OLS_SMT_LEDON, 0 },
	{ CMD_OLS_SMT_LEDOFF, 0 },
	{ CMD_START_OLS_ASSY, 0 },
	{ CMD_STOP_OLS_ASSY, 0 },
	{ CMD_OLS_SMTTEST_STOP, 0 },
	{ CMD_READ_VIN_SCALED, 2 },
	{ CMD_READ_BAT_MIN_W, 2 },
	{ CMD_READ_BAR_MAX_W, 2 },
	{ CMD_RESET_BAT_MINMAX_W, 0 },

	{ CMD_READ_LOCATION, 1 },
	{ CMD_WRITE_LOCATION, 0 },
	{ CMD_GET_FW_HASH, 16 },
	{ CMD_SUSPEND_HINT, 0 },
	{ CMD_ENABLE_WAKE_TIMER, 0 },
	{ CMD_SET_WAKE_TIMER, 0 },
	{ CMD_ENABLE_WAKE_AUTORESET, 0 },
	{ CMD_OLS_SET_LIMITS, 0 },
	{ CMD_OLS_GET_LIMITS, 4 },
	{ CMD_OLS_SET_CEILING, 0 },
	{ CMD_OLS_GET_CEILING, 2 },
	{ CMD_READ_EXT_SCI_MASK, 2 },
	{ CMD_WRITE_EXT_SCI_MASK, 0 },

	{ 0 },
};

static int olpc_xo175_ec_is_valid_cmd(u8 cmd)
{
	const struct ec_cmd_t *p;

	for (p = olpc_xo175_ec_cmds; p->cmd; p++) {
		if (p->cmd == cmd)
			return p->bytes_returned;
	}

	return -1;
}

#if 0
static int olpc_xo175_ec_cmd(u8 cmd, u8 *inbuf, size_t inlen, u8 *resp, size_t resp_len, void *ec_cb_arg)
{
	struct olpc_xo175_ec *ec = ec_cb_arg;

	gpiod_set_value_cansleep(ec->gpio_cmd, 1);
	dev_info(&ec->spi->dev, "XXX EC CMD: inlen=%d resp_len=%d\n", inlen, resp_len);

	return 0;
}
#endif


/*
 * This function is protected with a mutex.  We can safely assume that
 * there will be only one instance of this function running at a time.
 * One of the ways in which we enforce this is by waiting until we get
 * all response bytes back from the EC, rather than just the number that
 * the caller requests (otherwise, we might start a new command while an
 * old command's response bytes are still incoming).
 */
static int olpc_xo175_ec_cmd(u8 cmd, u8 *inbuf, size_t inlen, u8 *resp, size_t resp_len, void *ec_cb_arg)
{
	struct olpc_xo175_ec *ec = ec_cb_arg;
	unsigned long flags;
	int nr_bytes;
	int ret = 0;

	if (ec_debug)
		dev_info(&ec->spi->dev, "CMD %x, %d bytes expected\n", cmd, resp_len);

	if (inlen >= EC_CMD_LEN - 1) {
		dev_err(&ec->spi->dev, "command len %d too big!\n", resp_len);
		return -EOVERFLOW;
	}

	/* suspending in the middle of an EC command hoses things badly! */
	WARN_ON(ec->suspended);
	if (ec->suspended)
		return -EBUSY;

	memset(resp, 0, resp_len);

	/* ensure a valid command and return bytes */
	nr_bytes = olpc_xo175_ec_is_valid_cmd(cmd);
	if (nr_bytes < 0) {
		if (printk_ratelimit())
			dev_err(&ec->spi->dev, "Unknown EC command 0x%x\n", cmd);

		/*
		 * Assume the best in our callers, and allow unknown commands
		 * through.  I'm not the charitable type, but it was beaten
		 * into me.  Just maintain a minimum standard of sanity..
		 */

		if (resp_len > sizeof(ec->resp)) {
			dev_err(&ec->spi->dev, "response len %d too big!\n", resp_len);
			return -EOVERFLOW;
		}
		nr_bytes = resp_len;
	}
	if (resp_len > nr_bytes)
		resp_len = nr_bytes;

	spin_lock_irqsave(&ec->cmd_state_lock, flags);

	if (ec->flummoxed) {
		ec->flummoxed = false;

#if 0
		/* the FIFO should have two bytes in it prior to lowering ACK */
		writel(0, ec->base + SSDR);
		writel(0, ec->base + SSDR);
#endif

		gpiod_set_value_cansleep(ec->gpio_ack, 0);
	}

	/* initialize the state machine */
	init_completion(&ec->cmd_done);
	ec->cmd_running = true;
	ec->cmd_state = CMD_STATE_WAITING_FOR_SWITCH;
//	memcpy(ec->cmd, cmd, EC_CMD_LEN);
	ec->cmd[0] = cmd;
	memcpy(&ec->cmd[1], inbuf, inlen);
	ec->expected_resp_len = nr_bytes;
	ec->resp_len = 0;
	ec->pkts_to_ignore = 0;
	ec->flummoxed = false;

	/* tickle the cmd gpio to get things started */
	gpiod_set_value_cansleep(ec->gpio_cmd, 1);

	spin_unlock_irqrestore(&ec->cmd_state_lock, flags);

	/* the irq handler should do the rest */
	if (!wait_for_completion_timeout(&ec->cmd_done,
			msecs_to_jiffies(4000))) {
		dev_err(&ec->spi->dev, "EC cmd error: timeout in STATE %d\n",
				ec->cmd_state);
		gpiod_set_value_cansleep(ec->gpio_cmd, 0);
		return -ETIMEDOUT;
	}

	/*
	 * Work around a protocol bug; if we're not getting data back from
	 * the EC, then we'll wake up immediately after the EC gets the
	 * command packet.  However, the EC won't yet have determined the
	 * validity of the command packet.  So, wait some additional mS for
	 * error reports from the EC, just in case.
	 */
	if (!nr_bytes)
		msleep(5);

	spin_lock_irqsave(&ec->cmd_state_lock, flags);

	/* deal with the results.. */
	if (ec->cmd_state == CMD_STATE_ERROR_RECEIVED) {
		/* EC-provided error is in the single response byte */
		dev_err(&ec->spi->dev, "command 0x%x returned error 0x%x\n", cmd, ec->resp[0]);
		ret = -EREMOTEIO;
	} else if (ec->resp_len != nr_bytes) {
		dev_err(&ec->spi->dev, "command 0x%x returned %d bytes, expected %d bytes\n", cmd, ec->resp_len, nr_bytes);
		ret = -ETIMEDOUT;
	} else {
		/*
		 * We may have 8 bytes in ec->resp, but we only care about
		 * what we've been asked for.  If the caller asked for only 2
		 * bytes, give them that.  We've guaranteed that
		 * resp_len <= ec->resp_len and ec->resp_len == nr_bytes.
		 */
		memcpy(resp, ec->resp, resp_len);
	}

	/* this should already be low, but just in case.. */
	gpiod_set_value_cansleep(ec->gpio_cmd, 0);
	ec->cmd_running = false;

	spin_unlock_irqrestore(&ec->cmd_state_lock, flags);

	return ret;
}

static int olpc_xo175_ec_set_event_mask(unsigned int mask)
{
	unsigned char args[2];

	printk ("XXX EVENT MASK\n");
	return 0;

	args[0] = mask & 0xff;
	args[1] = (mask >> 8) & 0xff;
	return olpc_ec_cmd(CMD_WRITE_EXT_SCI_MASK, args, 2, NULL, 0);
}

static int olpc_xo175_ec_remove(struct spi_device *spi)
{
	struct olpc_xo175_ec *ec = spi_get_drvdata(spi);

	platform_device_unregister(olpc_ec);
	olpc_ec = NULL;

	// XXX order
	input_unregister_device(ec->pwrbtn);
	input_free_device(ec->pwrbtn);
	spi_slave_abort(spi);
	wait_for_completion(&ec->finished);

	return 0;
}

#ifdef CONFIG_PM
static int olpc_xo175_ec_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct olpc_xo175_ec *ec = platform_get_drvdata(pdev);
	unsigned char hintargs[5];
	static unsigned int suspend_count;
	
	olpc_xo175_ec_set_event_mask(ec->wakeup_mask);

	suspend_count++;
	if (ec_debug)
		dev_err(dev, "%s: suspend sync %08x\n", __func__, suspend_count);

	// first byte is 1 to indicate suspend, the rest is an
	// integer counter.
	hintargs[0] = 1;
	memcpy(&hintargs[1], &suspend_count, 4);
	olpc_ec_cmd(CMD_SUSPEND_HINT, hintargs, 5, NULL, 0);

	/* it's not clear that being able to disable EC wakeups is
	 * such a great idea, since that will disable even the use
	 * of the power button to wake us up.  if we really need
	 * to suppress the battery and AC events, we'll need to
	 * create an EC command to do so.
	 */

#ifdef NENY
	if (device_may_wakeup(&ec->spi->dev)) {
		ec->save_ecirq_mfpr = mfp_setedge(ec->gpio_ecirq, MFP_SETEDGE_FALLING);
	}
#endif

	/*
	 * After we've sent the suspend hint, don't allow further EC commands
	 * to be run until we've resumed.  Userspace tasks should be frozen,
	 * but kernel threads and interrupts could still schedule EC commands.
	 */
	ec->suspended = true;

	return 0;
}

static int olpc_xo175_ec_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct olpc_xo175_ec *ec = platform_get_drvdata(pdev);

	ec->suspended = false;

	return 0;
}
	
static int olpc_xo175_ec_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct olpc_xo175_ec *ec = platform_get_drvdata(pdev);
	
	ec->suspended = false;	/* just in case.. */

#if 0	
	unsigned char x;
	/* the resume hint is only needed if no other commands are
	 * being sent during resume.  all it does is tell the EC
	 * the SoC is definitely awake.
	 */
	olpc_ec_cmd(CMD_SUSPEND_HINT, (x = 0, &x), 1, NULL, 0);
#endif

	// enable all EC events while we're awake
	olpc_xo175_ec_set_event_mask(0xffff);

	/* always start the wakeup mask with a clean slate. 
	 * drivers will express their disinterest in wakeups
	 * by calling olpc_ec_disable_wakeup() from their suspend
	 * hooks.
	 */
	ec->wakeup_mask = 0xffff;

#ifdef NENY
	if (device_may_wakeup(&ec->spi->dev)) {
		mfp_write(ec->gpio_ecirq, ec->save_ecirq_mfpr);
	}
#endif

	return 0;
}
#endif

static struct olpc_ec_driver ec_driver = {
        .ec_cmd = olpc_xo175_ec_cmd,
};


static int olpc_xo175_ec_probe(struct spi_device *spi)
{
	struct olpc_xo175_ec *ec;
	int ret;


	if (olpc_ec) {
		dev_err(&spi->dev, "OLPC EC already registered.\n");
		return -EBUSY;
	}


	ec = devm_kzalloc(&spi->dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->gpio_ack = devm_gpiod_get(&spi->dev, "ack", GPIOD_OUT_LOW);
	if (IS_ERR(ec->gpio_ack)) {
		dev_err(&spi->dev, "failed to get ack gpio: %ld\n", PTR_ERR(ec->gpio_ack));
		return PTR_ERR(ec->gpio_ack);
	}

	ec->gpio_cmd = devm_gpiod_get(&spi->dev, "cmd", GPIOD_OUT_LOW);
	if (IS_ERR(ec->gpio_cmd)) {
		dev_err(&spi->dev, "failed to get cmd gpio: %ld\n", PTR_ERR(ec->gpio_cmd));
		return PTR_ERR(ec->gpio_cmd);
	}

	ec->spi = spi;
	init_completion(&ec->finished);










	ec->pkts_to_ignore = 0;

	spin_lock_init(&ec->cmd_state_lock);
	ec->cmd_state = CMD_STATE_IDLE;
	init_completion(&ec->cmd_done);

	ec->logbuf_len = 0;

	// shut down the hardware before we start
	//writel(0, ec->base + SSCR0);
	//writel(0, ec->base + SSCR1);




	/* set up power button input device */
	ec->pwrbtn = devm_input_allocate_device(&ec->spi->dev);
	if (!ec->pwrbtn)
		return -ENOMEM;

	ec->pwrbtn->name       = "Power Button";
	ec->pwrbtn->dev.parent = &ec->spi->dev;

	input_set_capability(ec->pwrbtn, EV_KEY, KEY_POWER);

	ret = input_register_device(ec->pwrbtn);
	if (ret) {
		dev_err(&ec->spi->dev, "failed to register power button: %d\n", ret);
		return ret;
	}

	ret = kfifo_alloc(&ec->event_queue, 32, GFP_KERNEL);
	if (ret) {
		dev_err(&ec->spi->dev, "failed to allocate an event queue: %d\n", ret);
		return ret;
	}

	dev_info(&spi->dev, "OLPC XO-1.75 Embedded Controller driver\n");

	INIT_WORK(&ec->event_queue_work, event_queue_worker);

	//device_init_wakeup(&ec->spi->dev, 1);








	spi_set_drvdata(spi, ec);

	ec->xfer.rx_buf = ec->rx_buf;
	ec->xfer.tx_buf = ec->tx_buf;
#if 0
	ec->xfer.len = 2;
	ret = olpc_xo175_ec_submit(ec);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(ec->gpio_cmd, 0);
	gpiod_set_value_cansleep(ec->gpio_ack, 0);
#endif



	olpc_ec_driver_register(&ec_driver, ec);
	olpc_ec = platform_device_register_simple("olpc-ec", -1, NULL, 0);



	olpc_xo175_ec_prime_fifo(ec);












	/*
	 * prime the initial wakeup mask.  all events cause
	 * wakeups, unless someone (usually a driver that's
	 * registered to receive them) tells us otherwise.
	 */
	ec->wakeup_mask = 0xffff;

	/* enable all EC events while we're awake */
	olpc_xo175_ec_set_event_mask(0xffff);




	return 0;
}


#ifdef CONFIG_PM
 const struct dev_pm_ops olpc_xo175_ec_pm_ops = {
	.suspend	= olpc_xo175_ec_suspend,
	.resume_noirq	= olpc_xo175_ec_resume_noirq,
	.resume		= olpc_xo175_ec_resume,
};
#endif

static const struct of_device_id olpc_xo175_ec_of_match[] = {
        { .compatible = "olpc,xo1.75-ec" },
        { },
};
MODULE_DEVICE_TABLE(of, olpc_xo175_ec_of_match);

static struct spi_driver olpc_xo175_ec_driver = {
	.driver = {
		.name	= "olpc-xo175-ec",
		.of_match_table = olpc_xo175_ec_of_match,
#ifdef CONFIG_PM
		.pm = &olpc_xo175_ec_pm_ops,
#endif
	},
	.probe		= olpc_xo175_ec_probe,
	.remove		= olpc_xo175_ec_remove,
};
module_spi_driver(olpc_xo175_ec_driver);


MODULE_DESCRIPTION("OLPC XO-1.75 Embedded Controller driver");
MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_AUTHOR("Lennert Buytenhek <buytenh@wantstofly.org>");
MODULE_LICENSE("GPL");
