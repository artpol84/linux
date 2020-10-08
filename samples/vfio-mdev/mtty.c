// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mediated virtual PCI serial host device driver
 *
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *     Author: Neo Jia <cjia@nvidia.com>
 *             Kirti Wankhede <kwankhede@nvidia.com>
 *
 * Sample driver that creates mdev device that simulates serial port over PCI
 * card.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uuid.h>
#include <linux/vfio.h>
#include <linux/iommu.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/file.h>
#include <linux/mdev.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <uapi/linux/serial_reg.h>
#include <linux/eventfd.h>
/*
 * #defines
 */

#define VERSION_STRING  "0.1"
#define DRIVER_AUTHOR   "NVIDIA Corporation"

#define MTTY_CLASS_NAME "mtty"

#define MTTY_NAME       "mtty"

#define MTTY_STRING_LEN		16

#define MTTY_CONFIG_SPACE_SIZE		0xff
#define MTTY_IO_BAR_SIZE		0x8
#define MTTY_MMIO_BAR_SIZE		0x100000
#define MTTY_MIGRATION_REGION_SIZE	0x1000000	// 16M

#define MTTY_MIGRATION_REGION_INDEX	VFIO_PCI_NUM_REGIONS
#define MTTY_REGIONS_MAX		(MTTY_MIGRATION_REGION_INDEX + 1)

/* Data section start from page aligned offset */
#define MTTY_MIGRATION_REGION_DATA_OFFSET	(0x1000)

/* First page is used for struct vfio_device_migration_info */
#define MTTY_MIGRATION_REGION_SIZE_MMAP     \
	(MTTY_MIGRATION_REGION_SIZE - MTTY_MIGRATION_REGION_DATA_OFFSET)

#define MIGRATION_INFO_OFFSET(MEMBER)	\
		offsetof(struct vfio_device_migration_info, MEMBER)

#define STORE_LE16(addr, val)   (*(u16 *)addr = val)
#define STORE_LE32(addr, val)   (*(u32 *)addr = val)

#define MAX_FIFO_SIZE   16

#define CIRCULAR_BUF_INC_IDX(idx)    (idx = (idx + 1) & (MAX_FIFO_SIZE - 1))

#define MTTY_VFIO_PCI_OFFSET_SHIFT   40

#define MTTY_VFIO_PCI_OFFSET_TO_INDEX(off)   (off >> MTTY_VFIO_PCI_OFFSET_SHIFT)
#define MTTY_VFIO_PCI_INDEX_TO_OFFSET(index) \
				((u64)(index) << MTTY_VFIO_PCI_OFFSET_SHIFT)
#define MTTY_VFIO_PCI_OFFSET_MASK    \
				(((u64)(1) << MTTY_VFIO_PCI_OFFSET_SHIFT) - 1)
#define MAX_MTTYS	24

/* Maximum pages of 4K for upto 2G RAM can be pinned */
#define MAX_GPFN_COUNT	(512 * 1024)
#define PFN_NULL	(~0UL)

/*
 * Global Structures
 */

static struct mtty_dev {
	dev_t		vd_devt;
	struct class	*vd_class;
	struct cdev	vd_cdev;
	struct idr	vd_idr;
	struct device	dev;
} mtty_dev;

struct mdev_region_info {
	u64 start;
	u64 phys_start;
	u32 size;
	u64 vfio_offset;
};

#if defined(DEBUG_REGS)
static const char *wr_reg[] = {
	"TX",
	"IER",
	"FCR",
	"LCR",
	"MCR",
	"LSR",
	"MSR",
	"SCR"
};

static const char *rd_reg[] = {
	"RX",
	"IER",
	"IIR",
	"LCR",
	"MCR",
	"LSR",
	"MSR",
	"SCR"
};
#endif

/* loop back buffer */
struct rxtx {
	u8 fifo[MAX_FIFO_SIZE];
	u8 head, tail;
	u8 count;
};

struct serial_port {
	u8 uart_reg[8];         /* 8 registers */
	struct rxtx rxtx;       /* loop back buffer */
	bool dlab;
	bool overrun;
	u16 divisor;
	u8 fcr;                 /* FIFO control register */
	u8 max_fifo_size;
	u8 intr_trigger_level;  /* interrupt trigger level */
};

/* Migration packet */
#define PACKET_ID		(u16)(0xfeedbaba)

#define PACKET_FLAGS_ACTUAL_DATA	(1 << 0)
#define PACKET_FLAGS_DUMMY_DATA		(1 << 1)

#define PACKET_DATA_SIZE_MAX		(8 * 1024 * 1024)

struct packet {
	u16 id;
	u16 flags;
	u32 data_size;
	u8 data[];
};

enum {
	PACKET_STATE_NONE = 0,
	PACKET_STATE_PREPARED,
	PACKET_STATE_COPIED,
	PACKET_STATE_LAST,
};

/* State of each mdev device */
struct mdev_state {
	int irq_fd;
	struct eventfd_ctx *intx_evtfd;
	struct eventfd_ctx *msi_evtfd;
	int irq_index;
	u8 *vconfig;
	struct mutex ops_lock;
	struct mdev_device *mdev;
	struct mdev_region_info region_info[MTTY_REGIONS_MAX];
	u32 bar_mask[MTTY_REGIONS_MAX];
	struct list_head next;
	struct serial_port s[2];
	struct mutex rxtx_lock;
	struct vfio_device_info dev_info;
	u32 nr_ports;

	/* List of pinned gpfns, gpfn as index and content is translated hpfn */
	unsigned long *gpfn_to_hpfn;
	struct notifier_block nb;

	u32 device_state;
	u64 saved_size;
	void *mig_region_base;
	bool is_actual_data_sent;
	struct packet *pkt;
	u32 packet_state;
	u64 dummy_data_size;
};

static struct mutex mdev_list_lock;
static struct list_head mdev_devices_list;

/*
 * Default dummy data size set to 100 MB. To change value of dummy data size at
 * runtime but before migration write size in MB to sysfs file
 * dummy_data_size_MB
 */
static unsigned long user_dummy_data_size = (100 * 1024 * 1024);

static const struct file_operations vd_fops = {
	.owner          = THIS_MODULE,
};

/* function prototypes */

static int mtty_trigger_interrupt(struct mdev_state *mdev_state);

/* Helper functions */

static void dump_buffer(u8 *buf, uint32_t count)
{
#if defined(DEBUG)
	int i;

	pr_info("Buffer:\n");
	for (i = 0; i < count; i++) {
		pr_info("%2x ", *(buf + i));
		if ((i + 1) % 16 == 0)
			pr_info("\n");
	}
#endif
}

static void mtty_create_config_space(struct mdev_state *mdev_state)
{
	/* PCI dev ID */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x0], 0x32534348);

	/* Control: I/O+, Mem-, BusMaster- */
	STORE_LE16((u16 *) &mdev_state->vconfig[0x4], 0x0001);

	/* Status: capabilities list absent */
	STORE_LE16((u16 *) &mdev_state->vconfig[0x6], 0x0200);

	/* Rev ID */
	mdev_state->vconfig[0x8] =  0x10;

	/* programming interface class : 16550-compatible serial controller */
	mdev_state->vconfig[0x9] =  0x02;

	/* Sub class : 00 */
	mdev_state->vconfig[0xa] =  0x00;

	/* Base class : Simple Communication controllers */
	mdev_state->vconfig[0xb] =  0x07;

	/* base address registers */
	/* BAR0: IO space */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x10], 0x000001);
	mdev_state->bar_mask[0] = ~(MTTY_IO_BAR_SIZE) + 1;

	if (mdev_state->nr_ports == 2) {
		/* BAR1: IO space */
		STORE_LE32((u32 *) &mdev_state->vconfig[0x14], 0x000001);
		mdev_state->bar_mask[1] = ~(MTTY_IO_BAR_SIZE) + 1;
	}

	/* Subsystem ID */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x2c], 0x32534348);

	mdev_state->vconfig[0x34] =  0x00;   /* Cap Ptr */
	mdev_state->vconfig[0x3d] =  0x01;   /* interrupt pin (INTA#) */

	/* Vendor specific data */
	mdev_state->vconfig[0x40] =  0x23;
	mdev_state->vconfig[0x43] =  0x80;
	mdev_state->vconfig[0x44] =  0x23;
	mdev_state->vconfig[0x48] =  0x23;
	mdev_state->vconfig[0x4c] =  0x23;

	mdev_state->vconfig[0x60] =  0x50;
	mdev_state->vconfig[0x61] =  0x43;
	mdev_state->vconfig[0x62] =  0x49;
	mdev_state->vconfig[0x63] =  0x20;
	mdev_state->vconfig[0x64] =  0x53;
	mdev_state->vconfig[0x65] =  0x65;
	mdev_state->vconfig[0x66] =  0x72;
	mdev_state->vconfig[0x67] =  0x69;
	mdev_state->vconfig[0x68] =  0x61;
	mdev_state->vconfig[0x69] =  0x6c;
	mdev_state->vconfig[0x6a] =  0x2f;
	mdev_state->vconfig[0x6b] =  0x55;
	mdev_state->vconfig[0x6c] =  0x41;
	mdev_state->vconfig[0x6d] =  0x52;
	mdev_state->vconfig[0x6e] =  0x54;
}

static void handle_pci_cfg_write(struct mdev_state *mdev_state, u16 offset,
				 u8 *buf, u32 count)
{
	u32 cfg_addr, bar_mask, bar_index = 0;

	switch (offset) {
	case 0x04: /* device control */
	case 0x06: /* device status */
		/* do nothing */
		break;
	case 0x3c:  /* interrupt line */
		mdev_state->vconfig[0x3c] = buf[0];
		break;
	case 0x3d:
		/*
		 * Interrupt Pin is hardwired to INTA.
		 * This field is write protected by hardware
		 */
		break;
	case 0x10:  /* BAR0 */
	case 0x14:  /* BAR1 */
		if (offset == 0x10)
			bar_index = 0;
		else if (offset == 0x14)
			bar_index = 1;

		if ((mdev_state->nr_ports == 1) && (bar_index == 1)) {
			STORE_LE32(&mdev_state->vconfig[offset], 0);
			break;
		}

		cfg_addr = *(u32 *)buf;
		pr_info("BAR%d addr 0x%x\n", bar_index, cfg_addr);

		if (cfg_addr == 0xffffffff) {
			bar_mask = mdev_state->bar_mask[bar_index];
			cfg_addr = (cfg_addr & bar_mask);
		}

		cfg_addr |= (mdev_state->vconfig[offset] & 0x3ul);
		STORE_LE32(&mdev_state->vconfig[offset], cfg_addr);
		break;
	case 0x18:  /* BAR2 */
	case 0x1c:  /* BAR3 */
	case 0x20:  /* BAR4 */
		STORE_LE32(&mdev_state->vconfig[offset], 0);
		break;
	default:
		pr_info("PCI config write @0x%x of %d bytes not handled\n",
			offset, count);
		break;
	}
}

static void handle_bar_write(unsigned int index, struct mdev_state *mdev_state,
				u16 offset, u8 *buf, u32 count)
{
	u8 data = *buf;

	/* Handle data written by guest */
	switch (offset) {
	case UART_TX:
		/* if DLAB set, data is LSB of divisor */
		if (mdev_state->s[index].dlab) {
			mdev_state->s[index].divisor |= data;
			break;
		}

		mutex_lock(&mdev_state->rxtx_lock);

		/* save in TX buffer */
		if (mdev_state->s[index].rxtx.count <
				mdev_state->s[index].max_fifo_size) {
			mdev_state->s[index].rxtx.fifo[
					mdev_state->s[index].rxtx.head] = data;
			mdev_state->s[index].rxtx.count++;
			CIRCULAR_BUF_INC_IDX(mdev_state->s[index].rxtx.head);
			mdev_state->s[index].overrun = false;

			/*
			 * Trigger interrupt if receive data interrupt is
			 * enabled and fifo reached trigger level
			 */
			if ((mdev_state->s[index].uart_reg[UART_IER] &
						UART_IER_RDI) &&
			   (mdev_state->s[index].rxtx.count ==
				    mdev_state->s[index].intr_trigger_level)) {
				/* trigger interrupt */
#if defined(DEBUG_INTR)
				pr_err("Serial port %d: Fifo level trigger\n",
					index);
#endif
				mtty_trigger_interrupt(mdev_state);
			}
		} else {
#if defined(DEBUG_INTR)
			pr_err("Serial port %d: Buffer Overflow\n", index);
#endif
			mdev_state->s[index].overrun = true;

			/*
			 * Trigger interrupt if receiver line status interrupt
			 * is enabled
			 */
			if (mdev_state->s[index].uart_reg[UART_IER] &
								UART_IER_RLSI)
				mtty_trigger_interrupt(mdev_state);
		}
		mutex_unlock(&mdev_state->rxtx_lock);
		break;

	case UART_IER:
		/* if DLAB set, data is MSB of divisor */
		if (mdev_state->s[index].dlab)
			mdev_state->s[index].divisor |= (u16)data << 8;
		else {
			mdev_state->s[index].uart_reg[offset] = data;
			mutex_lock(&mdev_state->rxtx_lock);
			if ((data & UART_IER_THRI) &&
			    (mdev_state->s[index].rxtx.head ==
					mdev_state->s[index].rxtx.tail)) {
#if defined(DEBUG_INTR)
				pr_err("Serial port %d: IER_THRI write\n",
					index);
#endif
				mtty_trigger_interrupt(mdev_state);
			}

			mutex_unlock(&mdev_state->rxtx_lock);
		}

		break;

	case UART_FCR:
		mdev_state->s[index].fcr = data;

		mutex_lock(&mdev_state->rxtx_lock);
		if (data & (UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT)) {
			/* clear loop back FIFO */
			mdev_state->s[index].rxtx.count = 0;
			mdev_state->s[index].rxtx.head = 0;
			mdev_state->s[index].rxtx.tail = 0;
		}
		mutex_unlock(&mdev_state->rxtx_lock);

		switch (data & UART_FCR_TRIGGER_MASK) {
		case UART_FCR_TRIGGER_1:
			mdev_state->s[index].intr_trigger_level = 1;
			break;

		case UART_FCR_TRIGGER_4:
			mdev_state->s[index].intr_trigger_level = 4;
			break;

		case UART_FCR_TRIGGER_8:
			mdev_state->s[index].intr_trigger_level = 8;
			break;

		case UART_FCR_TRIGGER_14:
			mdev_state->s[index].intr_trigger_level = 14;
			break;
		}

		/*
		 * Set trigger level to 1 otherwise or  implement timer with
		 * timeout of 4 characters and on expiring that timer set
		 * Recevice data timeout in IIR register
		 */
		mdev_state->s[index].intr_trigger_level = 1;
		if (data & UART_FCR_ENABLE_FIFO)
			mdev_state->s[index].max_fifo_size = MAX_FIFO_SIZE;
		else {
			mdev_state->s[index].max_fifo_size = 1;
			mdev_state->s[index].intr_trigger_level = 1;
		}

		break;

	case UART_LCR:
		if (data & UART_LCR_DLAB) {
			mdev_state->s[index].dlab = true;
			mdev_state->s[index].divisor = 0;
		} else
			mdev_state->s[index].dlab = false;

		mdev_state->s[index].uart_reg[offset] = data;
		break;

	case UART_MCR:
		mdev_state->s[index].uart_reg[offset] = data;

		if ((mdev_state->s[index].uart_reg[UART_IER] & UART_IER_MSI) &&
				(data & UART_MCR_OUT2)) {
#if defined(DEBUG_INTR)
			pr_err("Serial port %d: MCR_OUT2 write\n", index);
#endif
			mtty_trigger_interrupt(mdev_state);
		}

		if ((mdev_state->s[index].uart_reg[UART_IER] & UART_IER_MSI) &&
				(data & (UART_MCR_RTS | UART_MCR_DTR))) {
#if defined(DEBUG_INTR)
			pr_err("Serial port %d: MCR RTS/DTR write\n", index);
#endif
			mtty_trigger_interrupt(mdev_state);
		}
		break;

	case UART_LSR:
	case UART_MSR:
		/* do nothing */
		break;

	case UART_SCR:
		mdev_state->s[index].uart_reg[offset] = data;
		break;

	default:
		break;
	}
}

static void handle_bar_read(unsigned int index, struct mdev_state *mdev_state,
			    u16 offset, u8 *buf, u32 count)
{
	/* Handle read requests by guest */
	switch (offset) {
	case UART_RX:
		/* if DLAB set, data is LSB of divisor */
		if (mdev_state->s[index].dlab) {
			*buf  = (u8)mdev_state->s[index].divisor;
			break;
		}

		mutex_lock(&mdev_state->rxtx_lock);
		/* return data in tx buffer */
		if (mdev_state->s[index].rxtx.head !=
				 mdev_state->s[index].rxtx.tail) {
			*buf = mdev_state->s[index].rxtx.fifo[
						mdev_state->s[index].rxtx.tail];
			mdev_state->s[index].rxtx.count--;
			CIRCULAR_BUF_INC_IDX(mdev_state->s[index].rxtx.tail);
		}

		if (mdev_state->s[index].rxtx.head ==
				mdev_state->s[index].rxtx.tail) {
		/*
		 *  Trigger interrupt if tx buffer empty interrupt is
		 *  enabled and fifo is empty
		 */
#if defined(DEBUG_INTR)
			pr_err("Serial port %d: Buffer Empty\n", index);
#endif
			if (mdev_state->s[index].uart_reg[UART_IER] &
							 UART_IER_THRI)
				mtty_trigger_interrupt(mdev_state);
		}
		mutex_unlock(&mdev_state->rxtx_lock);

		break;

	case UART_IER:
		if (mdev_state->s[index].dlab) {
			*buf = (u8)(mdev_state->s[index].divisor >> 8);
			break;
		}
		*buf = mdev_state->s[index].uart_reg[offset] & 0x0f;
		break;

	case UART_IIR:
	{
		u8 ier = mdev_state->s[index].uart_reg[UART_IER];
		*buf = 0;

		mutex_lock(&mdev_state->rxtx_lock);
		/* Interrupt priority 1: Parity, overrun, framing or break */
		if ((ier & UART_IER_RLSI) && mdev_state->s[index].overrun)
			*buf |= UART_IIR_RLSI;

		/* Interrupt priority 2: Fifo trigger level reached */
		if ((ier & UART_IER_RDI) &&
		    (mdev_state->s[index].rxtx.count >=
		      mdev_state->s[index].intr_trigger_level))
			*buf |= UART_IIR_RDI;

		/* Interrupt priotiry 3: transmitter holding register empty */
		if ((ier & UART_IER_THRI) &&
		    (mdev_state->s[index].rxtx.head ==
				mdev_state->s[index].rxtx.tail))
			*buf |= UART_IIR_THRI;

		/* Interrupt priotiry 4: Modem status: CTS, DSR, RI or DCD  */
		if ((ier & UART_IER_MSI) &&
		    (mdev_state->s[index].uart_reg[UART_MCR] &
				 (UART_MCR_RTS | UART_MCR_DTR)))
			*buf |= UART_IIR_MSI;

		/* bit0: 0=> interrupt pending, 1=> no interrupt is pending */
		if (*buf == 0)
			*buf = UART_IIR_NO_INT;

		/* set bit 6 & 7 to be 16550 compatible */
		*buf |= 0xC0;
		mutex_unlock(&mdev_state->rxtx_lock);
	}
	break;

	case UART_LCR:
	case UART_MCR:
		*buf = mdev_state->s[index].uart_reg[offset];
		break;

	case UART_LSR:
	{
		u8 lsr = 0;

		mutex_lock(&mdev_state->rxtx_lock);
		/* atleast one char in FIFO */
		if (mdev_state->s[index].rxtx.head !=
				 mdev_state->s[index].rxtx.tail)
			lsr |= UART_LSR_DR;

		/* if FIFO overrun */
		if (mdev_state->s[index].overrun)
			lsr |= UART_LSR_OE;

		/* transmit FIFO empty and tramsitter empty */
		if (mdev_state->s[index].rxtx.head ==
				 mdev_state->s[index].rxtx.tail)
			lsr |= UART_LSR_TEMT | UART_LSR_THRE;

		mutex_unlock(&mdev_state->rxtx_lock);
		*buf = lsr;
		break;
	}
	case UART_MSR:
		*buf = UART_MSR_DSR | UART_MSR_DDSR | UART_MSR_DCD;

		mutex_lock(&mdev_state->rxtx_lock);
		/* if AFE is 1 and FIFO have space, set CTS bit */
		if (mdev_state->s[index].uart_reg[UART_MCR] &
						 UART_MCR_AFE) {
			if (mdev_state->s[index].rxtx.count <
					mdev_state->s[index].max_fifo_size)
				*buf |= UART_MSR_CTS | UART_MSR_DCTS;
		} else
			*buf |= UART_MSR_CTS | UART_MSR_DCTS;
		mutex_unlock(&mdev_state->rxtx_lock);

		break;

	case UART_SCR:
		*buf = mdev_state->s[index].uart_reg[offset];
		break;

	default:
		break;
	}
}

static void mdev_read_base(struct mdev_state *mdev_state)
{
	int index, pos;
	u32 start_lo, start_hi;
	u32 mem_type;

	pos = PCI_BASE_ADDRESS_0;

	for (index = 0; index <= VFIO_PCI_BAR5_REGION_INDEX; index++) {

		if (!mdev_state->region_info[index].size)
			continue;

		start_lo = (*(u32 *)(mdev_state->vconfig + pos)) &
			PCI_BASE_ADDRESS_MEM_MASK;
		mem_type = (*(u32 *)(mdev_state->vconfig + pos)) &
			PCI_BASE_ADDRESS_MEM_TYPE_MASK;

		switch (mem_type) {
		case PCI_BASE_ADDRESS_MEM_TYPE_64:
			start_hi = (*(u32 *)(mdev_state->vconfig + pos + 4));
			pos += 4;
			break;
		case PCI_BASE_ADDRESS_MEM_TYPE_32:
		case PCI_BASE_ADDRESS_MEM_TYPE_1M:
			/* 1M mem BAR treated as 32-bit BAR */
		default:
			/* mem unknown type treated as 32-bit BAR */
			start_hi = 0;
			break;
		}
		pos += 4;
		mdev_state->region_info[index].start = ((u64)start_hi << 32) |
							start_lo;
	}
}

static int save_setup(struct mdev_state *mdev_state)
{
	mdev_state->is_actual_data_sent = false;

	memset(mdev_state->pkt, 0, sizeof(struct packet) +
				   PACKET_DATA_SIZE_MAX);

	return 0;
}

static int set_device_state(struct mdev_state *mdev_state, u32 device_state)
{
	int ret = 0;

	if (mdev_state->device_state == device_state)
		return 0;

	if (device_state & VFIO_DEVICE_STATE_RUNNING) {
#if defined(DEBUG)
		if (device_state & VFIO_DEVICE_STATE_SAVING) {
			pr_info("%s: %s Pre-copy\n", __func__,
				dev_name(mdev_dev(mdev_state->mdev)));
		} else
			pr_info("%s: %s Running\n", __func__,
				dev_name(mdev_dev(mdev_state->mdev)));
#endif
	} else {
		if (device_state & VFIO_DEVICE_STATE_SAVING) {
#if defined(DEBUG)
			pr_info("%s: %s Stop-n-copy\n", __func__,
				dev_name(mdev_dev(mdev_state->mdev)));
#endif
			ret = save_setup(mdev_state);

		} else if (device_state & VFIO_DEVICE_STATE_RESUMING) {
#if defined(DEBUG)
			pr_info("%s: %s Resuming\n", __func__,
				dev_name(mdev_dev(mdev_state->mdev)));
		} else {
			pr_info("%s: %s Stopped\n", __func__,
				dev_name(mdev_dev(mdev_state->mdev)));
#endif
		}
	}

	mdev_state->device_state = device_state;

	return ret;
}

static u32 get_device_state(struct mdev_state *mdev_state)
{
	return mdev_state->device_state;
}

static void write_to_packet(struct packet *pkt, u8 *data, size_t size)
{
	if ((pkt->data_size + size) > PACKET_DATA_SIZE_MAX) {
		pr_err("%s: packet data overflow\n", __func__);
		return;
	}
	memcpy((void *)&pkt->data[pkt->data_size], (void *)data, size);
	pkt->data_size += size;
}

static void read_from_packet(struct packet *pkt, u8 *data,
			     int index, size_t size)
{
	if ((index + size) > PACKET_DATA_SIZE_MAX) {
		pr_err("%s: packet data overflow\n", __func__);
		return;
	}

	memcpy((void *)data, (void *)&pkt->data[index], size);
}

static int save_device_data(struct mdev_state *mdev_state, u64 *pending)
{
	/* Save device data only during stop-and-copy phase */
	if (mdev_state->device_state != VFIO_DEVICE_STATE_SAVING) {
		*pending = 0;
		return 0;
	}

	if (mdev_state->packet_state == PACKET_STATE_PREPARED) {
		*pending = sizeof(struct packet) + mdev_state->pkt->data_size;
		return 0;
	}

	if (!mdev_state->is_actual_data_sent) {

		/* create actual data packet */
		write_to_packet(mdev_state->pkt, (u8 *)&mdev_state->nr_ports,
				sizeof(mdev_state->nr_ports));
		write_to_packet(mdev_state->pkt, (u8 *)&mdev_state->s,
				sizeof(struct serial_port) * 2);

		write_to_packet(mdev_state->pkt, mdev_state->vconfig,
				MTTY_CONFIG_SPACE_SIZE);

		write_to_packet(mdev_state->pkt, (u8 *)mdev_state->gpfn_to_hpfn,
				sizeof(unsigned long) * MAX_GPFN_COUNT);

		mdev_state->pkt->id = PACKET_ID;
		mdev_state->pkt->flags = PACKET_FLAGS_ACTUAL_DATA;

		mdev_state->is_actual_data_sent = true;
	} else {
		/* create dummy data packet */
		if (mdev_state->dummy_data_size > user_dummy_data_size) {
			*pending = 0;
			mdev_state->packet_state = PACKET_STATE_NONE;
			return 0;
		}

		memset(mdev_state->pkt->data, 0xa5, PACKET_DATA_SIZE_MAX);

		mdev_state->pkt->id = PACKET_ID;
		mdev_state->pkt->flags = PACKET_FLAGS_DUMMY_DATA;
		mdev_state->pkt->data_size = PACKET_DATA_SIZE_MAX;
		mdev_state->dummy_data_size += PACKET_DATA_SIZE_MAX;
	}

	*pending = sizeof(struct packet) + mdev_state->pkt->data_size;
	mdev_state->packet_state = PACKET_STATE_PREPARED;
	mdev_state->saved_size = 0;

	return 0;
}

static int copy_device_data(struct mdev_state *mdev_state)
{
	u64 size;

	if (!mdev_state->pkt || !mdev_state->mig_region_base)
		return -EINVAL;

	if (mdev_state->packet_state == PACKET_STATE_COPIED)
		return 0;

	if (!mdev_state->pkt->data_size)
		return 0;

	size = sizeof(struct packet) + mdev_state->pkt->data_size;

	memcpy(mdev_state->mig_region_base, mdev_state->pkt, size);

	mdev_state->saved_size = size;
	mdev_state->packet_state = PACKET_STATE_COPIED;
	memset(mdev_state->pkt, 0, sizeof(struct packet));
	return 0;
}

static int resume_device_data(struct mdev_state *mdev_state, u64 data_size)
{
	unsigned long i;

	if (mdev_state->device_state != VFIO_DEVICE_STATE_RESUMING)
		return -EINVAL;

	if (!mdev_state->pkt || !mdev_state->mig_region_base)
		return -EINVAL;

	memcpy(mdev_state->pkt, mdev_state->mig_region_base, data_size);

	if (mdev_state->pkt->flags & PACKET_FLAGS_ACTUAL_DATA) {
		int index = 0;
		/* restore device data */
		read_from_packet(mdev_state->pkt, (u8 *)&mdev_state->nr_ports,
				 index, sizeof(mdev_state->nr_ports));
		index += sizeof(mdev_state->nr_ports);

		read_from_packet(mdev_state->pkt, (u8 *)&mdev_state->s,
				index, sizeof(struct serial_port) * 2);
		index += sizeof(struct serial_port) * 2;

		read_from_packet(mdev_state->pkt, mdev_state->vconfig,
				 index, MTTY_CONFIG_SPACE_SIZE);
		index += MTTY_CONFIG_SPACE_SIZE;

		read_from_packet(mdev_state->pkt,
				(u8 *)mdev_state->gpfn_to_hpfn,
				index, sizeof(unsigned long) * MAX_GPFN_COUNT);
		index += sizeof(unsigned long) * MAX_GPFN_COUNT;

		for (i = 0; i < MAX_GPFN_COUNT; i++) {
			if (mdev_state->gpfn_to_hpfn[i] != PFN_NULL) {
				int ret;
				unsigned long hpfn;

				ret = vfio_pin_pages(mdev_dev(mdev_state->mdev),
				       &i, 1, IOMMU_READ | IOMMU_WRITE, &hpfn);
				if (ret <= 0) {
					pr_err("%s: 0x%lx unpin error %d\n",
							__func__, i, ret);
					continue;
				}
				mdev_state->gpfn_to_hpfn[i] = hpfn;
			}
		}
	} else {
#if defined(DEBUG)
		pr_info("%s: %s discard data 0x%llx\n",
			 __func__, dev_name(mdev_dev(mdev_state->mdev)),
			data_size);
#endif
	}

	return 0;
}

static int handle_mig_read(unsigned int index, struct mdev_state *mdev_state,
			   loff_t offset, u8 *buf, u32 count)
{
	int ret = 0;
	u64 pending = 0;

	switch (offset) {
	case MIGRATION_INFO_OFFSET(device_state):	// 0x00
		*(u32 *)buf = get_device_state(mdev_state);
        pr_err("handle_mig_read: device_state = 0x%x", *(u32 *)buf); 
		break;

	case MIGRATION_INFO_OFFSET(pending_bytes):	// 0x08
		ret = save_device_data(mdev_state, &pending);
		if (ret)
			break;
		*(u64 *)buf = pending;
        pr_err("handle_mig_read: pending_bytes = %lld", *(u64 *)buf); 
		break;

	case MIGRATION_INFO_OFFSET(data_offset):	// 0x10
		if (mdev_state->device_state & VFIO_DEVICE_STATE_SAVING) {
			ret = copy_device_data(mdev_state);
			if (ret)
				break;
		}
		*(u64 *)buf = MTTY_MIGRATION_REGION_DATA_OFFSET;
        pr_err("handle_mig_read: data_offset = %lld", *(u64 *)buf); 
		break;

	case MIGRATION_INFO_OFFSET(data_size):		// 0x18
		*(u64 *)buf = mdev_state->saved_size;
        pr_err("handle_mig_read: data_size = %lld", *(u64 *)buf); 
		break;

	default:
		ret = -EINVAL;
	}

#if defined(DEBUG)
	pr_info("%s: %s MIG  RD @0x%llx bytes: %d data: 0x%x\n",
			__func__, dev_name(mdev_dev(mdev_state->mdev)),
			offset, count, *(u32 *)buf);
#endif
	return ret;
}

static int handle_mig_write(unsigned int index, struct mdev_state *mdev_state,
				loff_t offset, u8 *buf, u32 count)
{
	int ret = 0;

#if defined(DEBUG)
	pr_info("%s: %s MIG  WR @0x%llx bytes: %d data: 0x%x\n",
			__func__, dev_name(mdev_dev(mdev_state->mdev)),
			offset, count, *(u32 *)buf);
#endif
	switch (offset) {
	case MIGRATION_INFO_OFFSET(device_state):	// 0x00
		ret = set_device_state(mdev_state, *(u32 *)buf);
        pr_err("handle_mig_write: device_state = 0x%x", *(u32 *)buf); 
		break;

	case MIGRATION_INFO_OFFSET(data_size):		// 0x18
		ret = resume_device_data(mdev_state, *(u64 *)buf);
        pr_err("handle_mig_write: data_size = %lld", *(u64 *)buf); 
		break;

	case MIGRATION_INFO_OFFSET(pending_bytes):	// 0x08
	case MIGRATION_INFO_OFFSET(data_offset):	// 0x10
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mdev_access(struct mdev_device *mdev, u8 *buf, size_t count,
			   loff_t pos, bool is_write)
{
	struct mdev_state *mdev_state;
	unsigned int index;
	loff_t offset;
	int ret = 0;

	if (!mdev || !buf)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state) {
		pr_err("%s mdev_state not found\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mdev_state->ops_lock);

	index = MTTY_VFIO_PCI_OFFSET_TO_INDEX(pos);
	offset = pos & MTTY_VFIO_PCI_OFFSET_MASK;
	switch (index) {
	case VFIO_PCI_CONFIG_REGION_INDEX:

#if defined(DEBUG)
		pr_info("%s: PCI config space %s at offset 0x%llx\n",
			 __func__, is_write ? "write" : "read", offset);
#endif
		if (is_write) {
			dump_buffer(buf, count);
			handle_pci_cfg_write(mdev_state, offset, buf, count);
		} else {
			memcpy(buf, (mdev_state->vconfig + offset), count);
			dump_buffer(buf, count);
		}

		break;

	case VFIO_PCI_BAR0_REGION_INDEX ... VFIO_PCI_BAR5_REGION_INDEX:
		if (!mdev_state->region_info[index].start)
			mdev_read_base(mdev_state);

		if (is_write) {
			dump_buffer(buf, count);

#if defined(DEBUG_REGS)
			pr_info("%s: BAR%d  WR @0x%llx %s val:0x%02x dlab:%d\n",
				__func__, index, offset, wr_reg[offset],
				*buf, mdev_state->s[index].dlab);
#endif
			handle_bar_write(index, mdev_state, offset, buf, count);
		} else {
			handle_bar_read(index, mdev_state, offset, buf, count);
			dump_buffer(buf, count);

#if defined(DEBUG_REGS)
			pr_info("%s: BAR%d  RD @0x%llx %s val:0x%02x dlab:%d\n",
				__func__, index, offset, rd_reg[offset],
				*buf, mdev_state->s[index].dlab);
#endif
		}
		break;

	case MTTY_MIGRATION_REGION_INDEX:
		if (is_write) {
			ret = handle_mig_write(index, mdev_state, offset, buf,
					      count);
		} else {
			ret = handle_mig_read(index, mdev_state, offset, buf,
					      count);
		}
		if (ret)
			goto accessfailed;
		break;

	default:
		ret = -1;
		goto accessfailed;
	}

	ret = count;

accessfailed:
	mutex_unlock(&mdev_state->ops_lock);

	return ret;
}

static int mtty_create(struct kobject *kobj, struct mdev_device *mdev)
{
	struct mdev_state *mdev_state;
	char name[MTTY_STRING_LEN];
	int nr_ports = 0, i;

	if (!mdev)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		snprintf(name, MTTY_STRING_LEN, "%s-%d",
			dev_driver_string(mdev_parent_dev(mdev)), i + 1);
		if (!strcmp(kobj->name, name)) {
			nr_ports = i + 1;
			break;
		}
	}

	if (!nr_ports)
		return -EINVAL;

	mdev_state = kzalloc(sizeof(struct mdev_state), GFP_KERNEL);
	if (mdev_state == NULL)
		return -ENOMEM;

	mdev_state->nr_ports = nr_ports;
	mdev_state->irq_index = -1;
	mdev_state->s[0].max_fifo_size = MAX_FIFO_SIZE;
	mdev_state->s[1].max_fifo_size = MAX_FIFO_SIZE;
	mutex_init(&mdev_state->rxtx_lock);
	mdev_state->vconfig = kzalloc(MTTY_CONFIG_SPACE_SIZE, GFP_KERNEL);

	if (mdev_state->vconfig == NULL) {
		kfree(mdev_state);
		return -ENOMEM;
	}

	mdev_state->gpfn_to_hpfn =
		 kzalloc(sizeof(unsigned long) * MAX_GPFN_COUNT, GFP_KERNEL);
	if (mdev_state->gpfn_to_hpfn == NULL) {
		kfree(mdev_state->vconfig);
		kfree(mdev_state);
		return -ENOMEM;
	}

	memset(mdev_state->gpfn_to_hpfn, ~0,
	       sizeof(unsigned long) * MAX_GPFN_COUNT);

	mutex_init(&mdev_state->ops_lock);
	mdev_state->mdev = mdev;
	mdev_set_drvdata(mdev, mdev_state);

	mtty_create_config_space(mdev_state);

	mutex_lock(&mdev_list_lock);
	list_add(&mdev_state->next, &mdev_devices_list);
	mutex_unlock(&mdev_list_lock);

	return 0;
}

static int mtty_remove(struct mdev_device *mdev)
{
	struct mdev_state *mds, *tmp_mds;
	struct mdev_state *mdev_state = mdev_get_drvdata(mdev);
	int ret = -EINVAL;

	mutex_lock(&mdev_list_lock);
	list_for_each_entry_safe(mds, tmp_mds, &mdev_devices_list, next) {
		if (mdev_state == mds) {
			list_del(&mdev_state->next);
			mdev_set_drvdata(mdev, NULL);
			kfree(mdev_state->gpfn_to_hpfn);
			kfree(mdev_state->vconfig);
			kfree(mdev_state);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&mdev_list_lock);

	return ret;
}

static int mtty_reset(struct mdev_device *mdev)
{
	struct mdev_state *mdev_state;

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -EINVAL;

	pr_info("%s: called\n", __func__);

	return 0;
}

static ssize_t mtty_read(struct mdev_device *mdev, char __user *buf,
			 size_t count, loff_t *ppos)
{
	unsigned int done = 0, index;
	int ret;

	index = MTTY_VFIO_PCI_OFFSET_TO_INDEX(*ppos);

	while (count) {
		size_t filled;

		if ((index == MTTY_MIGRATION_REGION_INDEX) &&
		    (count >= 8 && !(*ppos % 8))) {
			u64 val;

			ret =  mdev_access(mdev, (u8 *)&val, sizeof(val),
					   *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 8;

		} else if (count >= 4 && !(*ppos % 4)) {
			u32 val;

			ret =  mdev_access(mdev, (u8 *)&val, sizeof(val),
					   *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 4;
		} else if (count >= 2 && !(*ppos % 2)) {
			u16 val;

			ret = mdev_access(mdev, (u8 *)&val, sizeof(val),
					  *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 2;
		} else {
			u8 val;

			ret = mdev_access(mdev, (u8 *)&val, sizeof(val),
					  *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 1;
		}

		count -= filled;
		done += filled;
		*ppos += filled;
		buf += filled;
	}

	return done;

read_err:
	return -EFAULT;
}

static ssize_t mtty_write(struct mdev_device *mdev, const char __user *buf,
		   size_t count, loff_t *ppos)
{
	unsigned int done = 0, index;
	int ret;

	index = MTTY_VFIO_PCI_OFFSET_TO_INDEX(*ppos);
	while (count) {
		size_t filled;

		if ((index == MTTY_MIGRATION_REGION_INDEX) &&
		    (count >= 8 && !(*ppos % 8))) {
			u64 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 8;
		} else if (count >= 4 && !(*ppos % 4)) {
			u32 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 4;
		} else if (count >= 2 && !(*ppos % 2)) {
			u16 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 2;
		} else {
			u8 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 1;
		}
		count -= filled;
		done += filled;
		*ppos += filled;
		buf += filled;
	}

	return done;
write_err:
	return -EFAULT;
}

static int mtty_set_irqs(struct mdev_device *mdev, uint32_t flags,
			 unsigned int index, unsigned int start,
			 unsigned int count, void *data)
{
	int ret = 0;
	struct mdev_state *mdev_state;

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -EINVAL;

	mutex_lock(&mdev_state->ops_lock);
	switch (index) {
	case VFIO_PCI_INTX_IRQ_INDEX:
		switch (flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
		case VFIO_IRQ_SET_ACTION_MASK:
		case VFIO_IRQ_SET_ACTION_UNMASK:
			break;
		case VFIO_IRQ_SET_ACTION_TRIGGER:
		{
			if (flags & VFIO_IRQ_SET_DATA_NONE) {
				pr_info("%s: disable INTx\n", __func__);
				if (mdev_state->intx_evtfd)
					eventfd_ctx_put(mdev_state->intx_evtfd);
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_EVENTFD) {
				int fd = *(int *)data;

				if (fd > 0) {
					struct eventfd_ctx *evt;

					evt = eventfd_ctx_fdget(fd);
					if (IS_ERR(evt)) {
						ret = PTR_ERR(evt);
						break;
					}
					mdev_state->intx_evtfd = evt;
					mdev_state->irq_fd = fd;
					mdev_state->irq_index = index;
					break;
				}
			}
			break;
		}
		}
		break;
	case VFIO_PCI_MSI_IRQ_INDEX:
		switch (flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
		case VFIO_IRQ_SET_ACTION_MASK:
		case VFIO_IRQ_SET_ACTION_UNMASK:
			break;
		case VFIO_IRQ_SET_ACTION_TRIGGER:
			if (flags & VFIO_IRQ_SET_DATA_NONE) {
				if (mdev_state->msi_evtfd)
					eventfd_ctx_put(mdev_state->msi_evtfd);
				pr_info("%s: disable MSI\n", __func__);
				mdev_state->irq_index = VFIO_PCI_INTX_IRQ_INDEX;
				break;
			}
			if (flags & VFIO_IRQ_SET_DATA_EVENTFD) {
				int fd = *(int *)data;
				struct eventfd_ctx *evt;

				if (fd <= 0)
					break;

				if (mdev_state->msi_evtfd)
					break;

				evt = eventfd_ctx_fdget(fd);
				if (IS_ERR(evt)) {
					ret = PTR_ERR(evt);
					break;
				}
				mdev_state->msi_evtfd = evt;
				mdev_state->irq_fd = fd;
				mdev_state->irq_index = index;
			}
			break;
	}
	break;
	case VFIO_PCI_MSIX_IRQ_INDEX:
		pr_info("%s: MSIX_IRQ\n", __func__);
		break;
	case VFIO_PCI_ERR_IRQ_INDEX:
		pr_info("%s: ERR_IRQ\n", __func__);
		break;
	case VFIO_PCI_REQ_IRQ_INDEX:
		pr_info("%s: REQ_IRQ\n", __func__);
		break;
	}

	mutex_unlock(&mdev_state->ops_lock);
	return ret;
}

static int mtty_trigger_interrupt(struct mdev_state *mdev_state)
{
	int ret = -1;

	if ((mdev_state->irq_index == VFIO_PCI_MSI_IRQ_INDEX) &&
	    (!mdev_state->msi_evtfd))
		return -EINVAL;
	else if ((mdev_state->irq_index == VFIO_PCI_INTX_IRQ_INDEX) &&
		 (!mdev_state->intx_evtfd)) {
		pr_info("%s: Intr eventfd not found\n", __func__);
		return -EINVAL;
	}

	if (mdev_state->irq_index == VFIO_PCI_MSI_IRQ_INDEX)
		ret = eventfd_signal(mdev_state->msi_evtfd, 1);
	else
		ret = eventfd_signal(mdev_state->intx_evtfd, 1);

#if defined(DEBUG_INTR)
	pr_info("Intx triggered\n");
#endif
	if (ret != 1)
		pr_err("%s: eventfd signal failed (%d)\n", __func__, ret);

	return ret;
}

static int mtty_get_region_info(struct mdev_device *mdev,
				struct vfio_region_info *region_info,
				struct vfio_info_cap *caps)
{
	unsigned int size = 0;
	struct mdev_state *mdev_state;
	u32 index;
	int ret = 0;

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -EINVAL;

	index = region_info->index;
	if (index >= MTTY_REGIONS_MAX)
		return -EINVAL;

	mutex_lock(&mdev_state->ops_lock);

	switch (index) {
	case VFIO_PCI_CONFIG_REGION_INDEX:
		size = MTTY_CONFIG_SPACE_SIZE;
		break;
	case VFIO_PCI_BAR0_REGION_INDEX:
		size = MTTY_IO_BAR_SIZE;
		break;
	case VFIO_PCI_BAR1_REGION_INDEX:
		if (mdev_state->nr_ports == 2)
			size = MTTY_IO_BAR_SIZE;
		break;
	case MTTY_MIGRATION_REGION_INDEX:
		size = MTTY_MIGRATION_REGION_SIZE;
		break;
	default:
		size = 0;
		break;
	}

	mdev_state->region_info[index].size = size;
	mdev_state->region_info[index].vfio_offset =
					MTTY_VFIO_PCI_INDEX_TO_OFFSET(index);

	region_info->size = size;
	region_info->offset = MTTY_VFIO_PCI_INDEX_TO_OFFSET(index);
	region_info->flags = VFIO_REGION_INFO_FLAG_READ |
			     VFIO_REGION_INFO_FLAG_WRITE;

	if (index == MTTY_MIGRATION_REGION_INDEX) {
		struct vfio_region_info_cap_sparse {
			struct vfio_region_info_cap_sparse_mmap sparse;
			struct vfio_region_sparse_mmap_area area;
		};

		struct vfio_region_info_cap_sparse mig_region;

		struct vfio_region_info_cap_type cap_type = {
			.header.id = VFIO_REGION_INFO_CAP_TYPE,
			.header.version = 1,
			.type = VFIO_REGION_TYPE_MIGRATION,
			.subtype = VFIO_REGION_SUBTYPE_MIGRATION
		};

		/* Add REGION CAP type */
		ret = vfio_info_add_capability(caps, &cap_type.header,
						sizeof(cap_type));
		if (ret)
			goto exit;

		/* Add sparse mmap cap type */
		mig_region.sparse.nr_areas = 1;
		mig_region.sparse.header.id = VFIO_REGION_INFO_CAP_SPARSE_MMAP;
		mig_region.sparse.header.version = 1;

		mig_region.area.offset = MTTY_MIGRATION_REGION_DATA_OFFSET;
		mig_region.area.size = MTTY_MIGRATION_REGION_SIZE_MMAP;

		region_info->flags |= VFIO_REGION_INFO_FLAG_CAPS;

		if (region_info->argsz > sizeof(*region_info))
			region_info->flags |= VFIO_REGION_INFO_FLAG_MMAP;

		ret = vfio_info_add_capability(caps, &mig_region.sparse.header,
						sizeof(mig_region));
	}
exit:
	mutex_unlock(&mdev_state->ops_lock);
	return ret;
}

static int mtty_get_irq_info(struct mdev_device *mdev,
			     struct vfio_irq_info *irq_info)
{
	switch (irq_info->index) {
	case VFIO_PCI_INTX_IRQ_INDEX:
	case VFIO_PCI_MSI_IRQ_INDEX:
	case VFIO_PCI_REQ_IRQ_INDEX:
		break;

	default:
		return -EINVAL;
	}

	irq_info->flags = VFIO_IRQ_INFO_EVENTFD;
	irq_info->count = 1;

	if (irq_info->index == VFIO_PCI_INTX_IRQ_INDEX)
		irq_info->flags |= (VFIO_IRQ_INFO_MASKABLE |
				VFIO_IRQ_INFO_AUTOMASKED);
	else
		irq_info->flags |= VFIO_IRQ_INFO_NORESIZE;

	return 0;
}

static int mtty_get_device_info(struct mdev_device *mdev,
			 struct vfio_device_info *dev_info)
{
	dev_info->flags = VFIO_DEVICE_FLAGS_PCI;
	dev_info->num_regions = MTTY_REGIONS_MAX;
	dev_info->num_irqs = VFIO_PCI_NUM_IRQS;

	return 0;
}

static long mtty_ioctl(struct mdev_device *mdev, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	unsigned long minsz;
	struct mdev_state *mdev_state;
	struct vfio_info_cap caps = { .buf = NULL, .size = 0 };

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -ENODEV;

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
	{
		struct vfio_device_info info;

		minsz = offsetofend(struct vfio_device_info, num_irqs);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = mtty_get_device_info(mdev, &info);
		if (ret)
			return ret;

		memcpy(&mdev_state->dev_info, &info, sizeof(info));

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_GET_REGION_INFO:
	{
		struct vfio_region_info info;

		minsz = offsetofend(struct vfio_region_info, offset);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = mtty_get_region_info(mdev, &info, &caps);
		if (ret)
			return ret;

		if (caps.size) {
			info.flags |= VFIO_REGION_INFO_FLAG_CAPS;
			if (info.argsz < sizeof(info) + caps.size) {
				info.argsz = sizeof(info) + caps.size;
				info.cap_offset = 0;
			} else {
				vfio_info_cap_shift(&caps, sizeof(info));
				if (copy_to_user((void __user *)arg +
							sizeof(info), caps.buf,
							caps.size)) {
					kfree(caps.buf);
					ret = -EFAULT;
					break;
				}
				info.cap_offset = sizeof(info);
			}
			kfree(caps.buf);
		}

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}

	case VFIO_DEVICE_GET_IRQ_INFO:
	{
		struct vfio_irq_info info;

		minsz = offsetofend(struct vfio_irq_info, count);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if ((info.argsz < minsz) ||
		    (info.index >= mdev_state->dev_info.num_irqs))
			return -EINVAL;

		ret = mtty_get_irq_info(mdev, &info);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_SET_IRQS:
	{
		struct vfio_irq_set hdr;
		u8 *data = NULL, *ptr = NULL;
		size_t data_size = 0;

		minsz = offsetofend(struct vfio_irq_set, count);

		if (copy_from_user(&hdr, (void __user *)arg, minsz))
			return -EFAULT;

		ret = vfio_set_irqs_validate_and_prepare(&hdr,
						mdev_state->dev_info.num_irqs,
						VFIO_PCI_NUM_IRQS,
						&data_size);
		if (ret)
			return ret;

		if (data_size) {
			ptr = data = memdup_user((void __user *)(arg + minsz),
						 data_size);
			if (IS_ERR(data))
				return PTR_ERR(data);
		}

		ret = mtty_set_irqs(mdev, hdr.flags, hdr.index, hdr.start,
				    hdr.count, data);

		kfree(ptr);
		return ret;
	}
	case VFIO_DEVICE_RESET:
		return mtty_reset(mdev);
	}
	return -ENOTTY;
}

void mmap_close(struct vm_area_struct *vma)
{
	struct mdev_device *mdev = vma->vm_private_data;
	struct mdev_state *mdev_state;
	uint32_t index = 0;

	if (!mdev)
		return;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return;

	mutex_lock(&mdev_state->ops_lock);
	index = MTTY_VFIO_PCI_OFFSET_TO_INDEX(vma->vm_pgoff << PAGE_SHIFT);
	if (index == MTTY_MIGRATION_REGION_INDEX) {
		if (mdev_state->mig_region_base != NULL) {
			vfree(mdev_state->mig_region_base);
			mdev_state->mig_region_base = NULL;
		}

		if (mdev_state->pkt != NULL) {
			vfree(mdev_state->pkt);
			mdev_state->pkt = NULL;
		}
	}
	mutex_unlock(&mdev_state->ops_lock);
}

static const struct vm_operations_struct mdev_vm_ops = {
	.close = mmap_close,
};

static int mtty_mmap(struct mdev_device *mdev, struct vm_area_struct *vma)
{
	struct mdev_state *mdev_state;
	unsigned int index;
	int ret = 0;

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -ENODEV;

	mutex_lock(&mdev_state->ops_lock);

	index = MTTY_VFIO_PCI_OFFSET_TO_INDEX(vma->vm_pgoff << PAGE_SHIFT);
	if (index == MTTY_MIGRATION_REGION_INDEX) {
		mdev_state->mig_region_base =
				 vmalloc_user(MTTY_MIGRATION_REGION_SIZE_MMAP);
		if (mdev_state->mig_region_base == NULL) {
			ret = -ENOMEM;
			goto mmap_exit;
		}

		mdev_state->pkt = vzalloc(sizeof(struct packet) +
					  PACKET_DATA_SIZE_MAX);
		if (mdev_state->pkt == NULL) {
			vfree(mdev_state->mig_region_base);
			mdev_state->mig_region_base = NULL;
			ret = -ENOMEM;
			goto mmap_exit;
		}

		vma->vm_ops = &mdev_vm_ops;

		ret = remap_vmalloc_range(vma, mdev_state->mig_region_base, 0);
		if (ret != 0) {
			pr_err("remap_vmalloc_range failed, ret= %d\n", ret);
			vfree(mdev_state->mig_region_base);
			mdev_state->mig_region_base = NULL;
			vfree(mdev_state->pkt);
			mdev_state->pkt = NULL;
			goto mmap_exit;
		}
	}
mmap_exit:
	mutex_unlock(&mdev_state->ops_lock);
	return ret;
}

static void unpin_pages_all(struct mdev_state *mdev_state)
{
	struct mdev_device *mdev = mdev_state->mdev;
	unsigned long i;

	mutex_lock(&mdev_state->ops_lock);
	for (i = 0; i < MAX_GPFN_COUNT; i++) {
		if (mdev_state->gpfn_to_hpfn[i] != PFN_NULL) {
			int ret;

			ret = vfio_unpin_pages(mdev_dev(mdev), &i, 1);
			if (ret <= 0) {
				pr_err("%s: 0x%lx unpin error %d\n",
					 __func__, i, ret);
				continue;
			}
			mdev_state->gpfn_to_hpfn[i] = PFN_NULL;
		}
	}
	mutex_unlock(&mdev_state->ops_lock);
}

static int unmap_notifier(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	if (action == VFIO_IOMMU_NOTIFY_DMA_UNMAP) {
		struct mdev_state *mdev_state = container_of(nb,
							 struct mdev_state, nb);
		struct mdev_device *mdev = mdev_state->mdev;
		struct vfio_iommu_type1_dma_unmap *unmap = data;
		unsigned long start = unmap->iova >> PAGE_SHIFT;
		unsigned long end = (unmap->iova + unmap->size) >> PAGE_SHIFT;
		unsigned long i;

		mutex_lock(&mdev_state->ops_lock);
		for (i = start; i < end; i++) {
			if (mdev_state->gpfn_to_hpfn[i] != PFN_NULL) {
				int ret;

				ret = vfio_unpin_pages(mdev_dev(mdev), &i, 1);
				if (ret <= 0) {
					pr_err("%s: 0x%lx unpin error %d\n",
							__func__, i, ret);
					continue;
				}
				mdev_state->gpfn_to_hpfn[i] = PFN_NULL;
			}
		}
		mutex_unlock(&mdev_state->ops_lock);

	}
	return 0;
}

static int mtty_open(struct mdev_device *mdev)
{
	unsigned long events = VFIO_IOMMU_NOTIFY_DMA_UNMAP;
	struct mdev_state *mdev_state;
	int ret;

	pr_info("%s\n", __func__);

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -ENODEV;

	mdev_state->nb.notifier_call = unmap_notifier;

	ret = vfio_register_notifier(mdev_dev(mdev), VFIO_IOMMU_NOTIFY, &events,
				     &mdev_state->nb);
	mdev_state->dummy_data_size = 0;
	mdev_state->mig_region_base = NULL;
	return ret;
}

static void mtty_close(struct mdev_device *mdev)
{
	struct mdev_state *mdev_state;

	pr_info("%s\n", __func__);

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return;

	unpin_pages_all(mdev_state);
	vfio_unregister_notifier(mdev_dev(mdev), VFIO_IOMMU_NOTIFY,
				 &mdev_state->nb);
	if (mdev_state->pkt != NULL) {
		vfree(mdev_state->pkt);
		mdev_state->pkt = NULL;
	}

	if (mdev_state->mig_region_base != NULL) {
		vfree(mdev_state->mig_region_base);
		mdev_state->mig_region_base = NULL;
	}
}

static ssize_t
sample_mtty_dev_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	return sprintf(buf, "This is phy device\n");
}

static DEVICE_ATTR_RO(sample_mtty_dev);

static struct attribute *mtty_dev_attrs[] = {
	&dev_attr_sample_mtty_dev.attr,
	NULL,
};

static const struct attribute_group mtty_dev_group = {
	.name  = "mtty_dev",
	.attrs = mtty_dev_attrs,
};

static const struct attribute_group *mtty_dev_groups[] = {
	&mtty_dev_group,
	NULL,
};

static ssize_t
sample_mdev_dev_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	if (mdev_from_dev(dev))
		return sprintf(buf, "This is MDEV %s\n", dev_name(dev));

	return sprintf(buf, "\n");
}

static DEVICE_ATTR_RO(sample_mdev_dev);

static ssize_t
pin_pages_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mdev_device *mdev = mdev_from_dev(dev);
	struct mdev_state *mdev_state;
	int i, count = 0;

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -EINVAL;

	mutex_lock(&mdev_state->ops_lock);
	for (i = 0; i < MAX_GPFN_COUNT; i++) {
		if (mdev_state->gpfn_to_hpfn[i] != PFN_NULL)
			count++;
	}
	mutex_unlock(&mdev_state->ops_lock);
	return sprintf(buf, "Pinned 0x%x pages\n", count);
}

static ssize_t
pin_pages_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mdev_device *mdev = mdev_from_dev(dev);
	struct mdev_state *mdev_state;
	unsigned long gpfn, hpfn;
	int ret;

	if (!mdev)
		return -EINVAL;

	mdev_state = mdev_get_drvdata(mdev);
	if (!mdev_state)
		return -EINVAL;

	ret = kstrtoul(buf, 0, &gpfn);
	if (ret)
		return ret;

	if (gpfn >= MAX_GPFN_COUNT) {
		pr_err("Error 0x%lx > 0x%lx\n",
		       gpfn, (unsigned long)MAX_GPFN_COUNT);
		return -EINVAL;
	}

	mutex_lock(&mdev_state->ops_lock);

	if (mdev_state->gpfn_to_hpfn[gpfn] != PFN_NULL) {
		ret = -EEXIST;
		goto out;
	}

	ret = vfio_pin_pages(mdev_dev(mdev), &gpfn, 1,
			     IOMMU_READ | IOMMU_WRITE, &hpfn);

	if (ret <= 0) {
		pr_err("Failed to pin, ret %d\n", ret);
		goto out;
	}

	mdev_state->gpfn_to_hpfn[gpfn] = hpfn;
	ret = count;
out:
	mutex_unlock(&mdev_state->ops_lock);
	return ret;
}

static DEVICE_ATTR_RW(pin_pages);

static ssize_t
dummy_data_size_MB_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;

	ret = kstrtoul(buf, 0, &user_dummy_data_size);
	if (ret)
		return ret;

	user_dummy_data_size = user_dummy_data_size << 20;
	return count;
}

static DEVICE_ATTR_WO(dummy_data_size_MB);

static struct attribute *mdev_dev_attrs[] = {
	&dev_attr_sample_mdev_dev.attr,
	&dev_attr_pin_pages.attr,
	&dev_attr_dummy_data_size_MB.attr,
	NULL,
};

static const struct attribute_group mdev_dev_group = {
	.name  = "vendor",
	.attrs = mdev_dev_attrs,
};

static const struct attribute_group *mdev_dev_groups[] = {
	&mdev_dev_group,
	NULL,
};

static ssize_t
name_show(struct kobject *kobj, struct device *dev, char *buf)
{
	char name[MTTY_STRING_LEN];
	int i;
	const char *name_str[2] = {"Single port serial", "Dual port serial"};

	for (i = 0; i < 2; i++) {
		snprintf(name, MTTY_STRING_LEN, "%s-%d",
			 dev_driver_string(dev), i + 1);
		if (!strcmp(kobj->name, name))
			return sprintf(buf, "%s\n", name_str[i]);
	}

	return -EINVAL;
}

static MDEV_TYPE_ATTR_RO(name);

static ssize_t
available_instances_show(struct kobject *kobj, struct device *dev, char *buf)
{
	char name[MTTY_STRING_LEN];
	int i;
	struct mdev_state *mds;
	int ports = 0, used = 0;

	for (i = 0; i < 2; i++) {
		snprintf(name, MTTY_STRING_LEN, "%s-%d",
			 dev_driver_string(dev), i + 1);
		if (!strcmp(kobj->name, name)) {
			ports = i + 1;
			break;
		}
	}

	if (!ports)
		return -EINVAL;

	list_for_each_entry(mds, &mdev_devices_list, next)
		used += mds->nr_ports;

	return sprintf(buf, "%d\n", (MAX_MTTYS - used)/ports);
}

static MDEV_TYPE_ATTR_RO(available_instances);


static ssize_t device_api_show(struct kobject *kobj, struct device *dev,
			       char *buf)
{
	return sprintf(buf, "%s\n", VFIO_DEVICE_API_PCI_STRING);
}

static MDEV_TYPE_ATTR_RO(device_api);

static struct attribute *mdev_types_attrs[] = {
	&mdev_type_attr_name.attr,
	&mdev_type_attr_device_api.attr,
	&mdev_type_attr_available_instances.attr,
	NULL,
};

static struct attribute_group mdev_type_group1 = {
	.name  = "1",
	.attrs = mdev_types_attrs,
};

static struct attribute_group mdev_type_group2 = {
	.name  = "2",
	.attrs = mdev_types_attrs,
};

static struct attribute_group *mdev_type_groups[] = {
	&mdev_type_group1,
	&mdev_type_group2,
	NULL,
};

static const struct mdev_parent_ops mdev_fops = {
	.owner                  = THIS_MODULE,
	.dev_attr_groups        = mtty_dev_groups,
	.mdev_attr_groups       = mdev_dev_groups,
	.supported_type_groups  = mdev_type_groups,
	.create                 = mtty_create,
	.remove			= mtty_remove,
	.open                   = mtty_open,
	.release                = mtty_close,
	.read                   = mtty_read,
	.write                  = mtty_write,
	.ioctl		        = mtty_ioctl,
	.mmap			= mtty_mmap,
};

static void mtty_device_release(struct device *dev)
{
	dev_dbg(dev, "mtty: released\n");
}

static int __init mtty_dev_init(void)
{
	int ret = 0;

	pr_info("mtty_dev: %s\n", __func__);

	memset(&mtty_dev, 0, sizeof(mtty_dev));

	idr_init(&mtty_dev.vd_idr);

	ret = alloc_chrdev_region(&mtty_dev.vd_devt, 0, MINORMASK + 1,
				  MTTY_NAME);

	if (ret < 0) {
		pr_err("Error: failed to register mtty_dev, err:%d\n", ret);
		return ret;
	}

	cdev_init(&mtty_dev.vd_cdev, &vd_fops);
	cdev_add(&mtty_dev.vd_cdev, mtty_dev.vd_devt, MINORMASK + 1);

	pr_info("major_number:%d\n", MAJOR(mtty_dev.vd_devt));

	mtty_dev.vd_class = class_create(THIS_MODULE, MTTY_CLASS_NAME);

	if (IS_ERR(mtty_dev.vd_class)) {
		pr_err("Error: failed to register mtty_dev class\n");
		ret = PTR_ERR(mtty_dev.vd_class);
		goto failed1;
	}

	mtty_dev.dev.class = mtty_dev.vd_class;
	mtty_dev.dev.release = mtty_device_release;
	dev_set_name(&mtty_dev.dev, "%s", MTTY_NAME);

	ret = device_register(&mtty_dev.dev);
	if (ret)
		goto failed2;

	ret = mdev_register_device(&mtty_dev.dev, &mdev_fops);
	if (ret)
		goto failed3;

	mutex_init(&mdev_list_lock);
	INIT_LIST_HEAD(&mdev_devices_list);

	goto all_done;

failed3:

	device_unregister(&mtty_dev.dev);
failed2:
	class_destroy(mtty_dev.vd_class);

failed1:
	cdev_del(&mtty_dev.vd_cdev);
	unregister_chrdev_region(mtty_dev.vd_devt, MINORMASK + 1);

all_done:
	return ret;
}

static void __exit mtty_dev_exit(void)
{
	mtty_dev.dev.bus = NULL;
	mdev_unregister_device(&mtty_dev.dev);

	device_unregister(&mtty_dev.dev);
	idr_destroy(&mtty_dev.vd_idr);
	cdev_del(&mtty_dev.vd_cdev);
	unregister_chrdev_region(mtty_dev.vd_devt, MINORMASK + 1);
	class_destroy(mtty_dev.vd_class);
	mtty_dev.vd_class = NULL;
	pr_info("mtty_dev: Unloaded!\n");
}

module_init(mtty_dev_init)
module_exit(mtty_dev_exit)

MODULE_LICENSE("GPL v2");
MODULE_INFO(supported, "Test driver that simulate serial port over PCI");
MODULE_VERSION(VERSION_STRING);
MODULE_AUTHOR(DRIVER_AUTHOR);
