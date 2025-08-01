// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  libata-scsi.c - helper library for ATA
 *
 *  Copyright 2003-2004 Red Hat, Inc.  All rights reserved.
 *  Copyright 2003-2004 Jeff Garzik
 *
 *  libata documentation is available via 'make {ps|pdf}docs',
 *  as Documentation/driver-api/libata.rst
 *
 *  Hardware documentation available from
 *  - http://www.t10.org/
 *  - http://www.t13.org/
 */

#include <linux/compat.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/spinlock.h>
#include <linux/export.h>
#include <scsi/scsi.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsi_transport.h>
#include <linux/libata.h>
#include <linux/hdreg.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>
#include <linux/ioprio.h>
#include <linux/of.h>

#include "libata.h"
#include "libata-transport.h"

#define ATA_SCSI_RBUF_SIZE	2048

static DEFINE_SPINLOCK(ata_scsi_rbuf_lock);
static u8 ata_scsi_rbuf[ATA_SCSI_RBUF_SIZE];

typedef unsigned int (*ata_xlat_func_t)(struct ata_queued_cmd *qc);

static struct ata_device *__ata_scsi_find_dev(struct ata_port *ap,
					const struct scsi_device *scsidev);

#define RW_RECOVERY_MPAGE		0x1
#define RW_RECOVERY_MPAGE_LEN		12
#define CACHE_MPAGE			0x8
#define CACHE_MPAGE_LEN			20
#define CONTROL_MPAGE			0xa
#define CONTROL_MPAGE_LEN		12
#define ALL_MPAGES			0x3f
#define ALL_SUB_MPAGES			0xff
#define CDL_T2A_SUB_MPAGE		0x07
#define CDL_T2B_SUB_MPAGE		0x08
#define CDL_T2_SUB_MPAGE_LEN		232
#define ATA_FEATURE_SUB_MPAGE		0xf2
#define ATA_FEATURE_SUB_MPAGE_LEN	16

static const u8 def_rw_recovery_mpage[RW_RECOVERY_MPAGE_LEN] = {
	RW_RECOVERY_MPAGE,
	RW_RECOVERY_MPAGE_LEN - 2,
	(1 << 7),	/* AWRE */
	0,		/* read retry count */
	0, 0, 0, 0,
	0,		/* write retry count */
	0, 0, 0
};

static const u8 def_cache_mpage[CACHE_MPAGE_LEN] = {
	CACHE_MPAGE,
	CACHE_MPAGE_LEN - 2,
	0,		/* contains WCE, needs to be 0 for logic */
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	0,		/* contains DRA, needs to be 0 for logic */
	0, 0, 0, 0, 0, 0, 0
};

static const u8 def_control_mpage[CONTROL_MPAGE_LEN] = {
	CONTROL_MPAGE,
	CONTROL_MPAGE_LEN - 2,
	2,	/* DSENSE=0, GLTSD=1 */
	0,	/* [QAM+QERR may be 1, see 05-359r1] */
	0, 0, 0, 0, 0xff, 0xff,
	0, 30	/* extended self test time, see 05-359r1 */
};

static ssize_t ata_scsi_park_show(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	struct scsi_device *sdev = to_scsi_device(device);
	struct ata_port *ap;
	struct ata_link *link;
	struct ata_device *dev;
	unsigned long now;
	unsigned int msecs;
	int rc = 0;

	ap = ata_shost_to_port(sdev->host);

	spin_lock_irq(ap->lock);
	dev = ata_scsi_find_dev(ap, sdev);
	if (!dev) {
		rc = -ENODEV;
		goto unlock;
	}
	if (dev->flags & ATA_DFLAG_NO_UNLOAD) {
		rc = -EOPNOTSUPP;
		goto unlock;
	}

	link = dev->link;
	now = jiffies;
	if (ap->pflags & ATA_PFLAG_EH_IN_PROGRESS &&
	    link->eh_context.unloaded_mask & (1 << dev->devno) &&
	    time_after(dev->unpark_deadline, now))
		msecs = jiffies_to_msecs(dev->unpark_deadline - now);
	else
		msecs = 0;

unlock:
	spin_unlock_irq(ap->lock);

	return rc ? rc : sysfs_emit(buf, "%u\n", msecs);
}

static ssize_t ata_scsi_park_store(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct scsi_device *sdev = to_scsi_device(device);
	struct ata_port *ap;
	struct ata_device *dev;
	int input;
	unsigned long flags;
	int rc;

	rc = kstrtoint(buf, 10, &input);
	if (rc)
		return rc;
	if (input < -2)
		return -EINVAL;
	if (input > ATA_TMOUT_MAX_PARK) {
		rc = -EOVERFLOW;
		input = ATA_TMOUT_MAX_PARK;
	}

	ap = ata_shost_to_port(sdev->host);

	spin_lock_irqsave(ap->lock, flags);
	dev = ata_scsi_find_dev(ap, sdev);
	if (unlikely(!dev)) {
		rc = -ENODEV;
		goto unlock;
	}
	if (dev->class != ATA_DEV_ATA &&
	    dev->class != ATA_DEV_ZAC) {
		rc = -EOPNOTSUPP;
		goto unlock;
	}

	if (input >= 0) {
		if (dev->flags & ATA_DFLAG_NO_UNLOAD) {
			rc = -EOPNOTSUPP;
			goto unlock;
		}

		dev->unpark_deadline = ata_deadline(jiffies, input);
		dev->link->eh_info.dev_action[dev->devno] |= ATA_EH_PARK;
		ata_port_schedule_eh(ap);
		complete(&ap->park_req_pending);
	} else {
		switch (input) {
		case -1:
			dev->flags &= ~ATA_DFLAG_NO_UNLOAD;
			break;
		case -2:
			dev->flags |= ATA_DFLAG_NO_UNLOAD;
			break;
		}
	}
unlock:
	spin_unlock_irqrestore(ap->lock, flags);

	return rc ? rc : len;
}
DEVICE_ATTR(unload_heads, S_IRUGO | S_IWUSR,
	    ata_scsi_park_show, ata_scsi_park_store);
EXPORT_SYMBOL_GPL(dev_attr_unload_heads);

bool ata_scsi_sense_is_valid(u8 sk, u8 asc, u8 ascq)
{
	/*
	 * If sk == NO_SENSE, and asc + ascq == NO ADDITIONAL SENSE INFORMATION,
	 * then there is no sense data to add.
	 */
	if (sk == 0 && asc == 0 && ascq == 0)
		return false;

	/* If sk > COMPLETED, sense data is bogus. */
	if (sk > COMPLETED)
		return false;

	return true;
}

void ata_scsi_set_sense(struct ata_device *dev, struct scsi_cmnd *cmd,
			u8 sk, u8 asc, u8 ascq)
{
	bool d_sense = (dev->flags & ATA_DFLAG_D_SENSE);

	scsi_build_sense(cmd, d_sense, sk, asc, ascq);
}

static void ata_scsi_set_sense_information(struct ata_queued_cmd *qc)
{
	u64 information;

	if (!(qc->flags & ATA_QCFLAG_RTF_FILLED)) {
		ata_dev_dbg(qc->dev,
			    "missing result TF: can't set INFORMATION sense field\n");
		return;
	}

	information = ata_tf_read_block(&qc->result_tf, qc->dev);
	if (information == U64_MAX)
		return;

	scsi_set_sense_information(qc->scsicmd->sense_buffer,
				   SCSI_SENSE_BUFFERSIZE, information);
}

/**
 *	ata_scsi_set_passthru_sense_fields - Set ATA fields in sense buffer
 *	@qc: ATA PASS-THROUGH command.
 *
 *	Populates "ATA Status Return sense data descriptor" / "Fixed format
 *	sense data" with ATA taskfile fields.
 *
 *	LOCKING:
 *	None.
 */
static void ata_scsi_set_passthru_sense_fields(struct ata_queued_cmd *qc)
{
	struct ata_device *dev = qc->dev;
	struct scsi_cmnd *cmd = qc->scsicmd;
	struct ata_taskfile *tf = &qc->result_tf;
	unsigned char *sb = cmd->sense_buffer;

	if (!(qc->flags & ATA_QCFLAG_RTF_FILLED)) {
		ata_dev_dbg(dev,
			    "missing result TF: can't set ATA PT sense fields\n");
		return;
	}

	if ((sb[0] & 0x7f) >= 0x72) {
		unsigned char *desc;
		u8 len;

		/* descriptor format */
		len = sb[7];
		desc = (char *)scsi_sense_desc_find(sb, len + 8, 9);
		if (!desc) {
			if (SCSI_SENSE_BUFFERSIZE < len + 14)
				return;
			sb[7] = len + 14;
			desc = sb + 8 + len;
		}
		desc[0] = 9;
		desc[1] = 12;
		/*
		 * Copy registers into sense buffer.
		 */
		desc[2] = 0x00;
		desc[3] = tf->error;
		desc[5] = tf->nsect;
		desc[7] = tf->lbal;
		desc[9] = tf->lbam;
		desc[11] = tf->lbah;
		desc[12] = tf->device;
		desc[13] = tf->status;

		/*
		 * Fill in Extend bit, and the high order bytes
		 * if applicable.
		 */
		if (tf->flags & ATA_TFLAG_LBA48) {
			desc[2] |= 0x01;
			desc[4] = tf->hob_nsect;
			desc[6] = tf->hob_lbal;
			desc[8] = tf->hob_lbam;
			desc[10] = tf->hob_lbah;
		}
	} else {
		/* Fixed sense format */
		sb[0] |= 0x80;
		sb[3] = tf->error;
		sb[4] = tf->status;
		sb[5] = tf->device;
		sb[6] = tf->nsect;
		if (tf->flags & ATA_TFLAG_LBA48)  {
			sb[8] |= 0x80;
			if (tf->hob_nsect)
				sb[8] |= 0x40;
			if (tf->hob_lbal || tf->hob_lbam || tf->hob_lbah)
				sb[8] |= 0x20;
		}
		sb[9] = tf->lbal;
		sb[10] = tf->lbam;
		sb[11] = tf->lbah;
	}
}

static void ata_scsi_set_invalid_field(struct ata_device *dev,
				       struct scsi_cmnd *cmd, u16 field, u8 bit)
{
	ata_scsi_set_sense(dev, cmd, ILLEGAL_REQUEST, 0x24, 0x0);
	/* "Invalid field in CDB" */
	scsi_set_sense_field_pointer(cmd->sense_buffer, SCSI_SENSE_BUFFERSIZE,
				     field, bit, 1);
}

static void ata_scsi_set_invalid_parameter(struct ata_device *dev,
					   struct scsi_cmnd *cmd, u16 field)
{
	/* "Invalid field in parameter list" */
	ata_scsi_set_sense(dev, cmd, ILLEGAL_REQUEST, 0x26, 0x0);
	scsi_set_sense_field_pointer(cmd->sense_buffer, SCSI_SENSE_BUFFERSIZE,
				     field, 0xff, 0);
}

static struct attribute *ata_common_sdev_attrs[] = {
	&dev_attr_unload_heads.attr,
	NULL
};

static const struct attribute_group ata_common_sdev_attr_group = {
	.attrs = ata_common_sdev_attrs
};

const struct attribute_group *ata_common_sdev_groups[] = {
	&ata_common_sdev_attr_group,
	NULL
};
EXPORT_SYMBOL_GPL(ata_common_sdev_groups);

/**
 *	ata_std_bios_param - generic bios head/sector/cylinder calculator used by sd.
 *	@sdev: SCSI device for which BIOS geometry is to be determined
 *	@bdev: block device associated with @sdev
 *	@capacity: capacity of SCSI device
 *	@geom: location to which geometry will be output
 *
 *	Generic bios head/sector/cylinder calculator
 *	used by sd. Most BIOSes nowadays expect a XXX/255/16  (CHS)
 *	mapping. Some situations may arise where the disk is not
 *	bootable if this is not used.
 *
 *	LOCKING:
 *	Defined by the SCSI layer.  We don't really care.
 *
 *	RETURNS:
 *	Zero.
 */
int ata_std_bios_param(struct scsi_device *sdev, struct block_device *bdev,
		       sector_t capacity, int geom[])
{
	geom[0] = 255;
	geom[1] = 63;
	sector_div(capacity, 255*63);
	geom[2] = capacity;

	return 0;
}
EXPORT_SYMBOL_GPL(ata_std_bios_param);

/**
 *	ata_scsi_unlock_native_capacity - unlock native capacity
 *	@sdev: SCSI device to adjust device capacity for
 *
 *	This function is called if a partition on @sdev extends beyond
 *	the end of the device.  It requests EH to unlock HPA.
 *
 *	LOCKING:
 *	Defined by the SCSI layer.  Might sleep.
 */
void ata_scsi_unlock_native_capacity(struct scsi_device *sdev)
{
	struct ata_port *ap = ata_shost_to_port(sdev->host);
	struct ata_device *dev;
	unsigned long flags;

	spin_lock_irqsave(ap->lock, flags);

	dev = ata_scsi_find_dev(ap, sdev);
	if (dev && dev->n_sectors < dev->n_native_sectors) {
		dev->flags |= ATA_DFLAG_UNLOCK_HPA;
		dev->link->eh_info.action |= ATA_EH_RESET;
		ata_port_schedule_eh(ap);
	}

	spin_unlock_irqrestore(ap->lock, flags);
	ata_port_wait_eh(ap);
}
EXPORT_SYMBOL_GPL(ata_scsi_unlock_native_capacity);

/**
 *	ata_get_identity - Handler for HDIO_GET_IDENTITY ioctl
 *	@ap: target port
 *	@sdev: SCSI device to get identify data for
 *	@arg: User buffer area for identify data
 *
 *	LOCKING:
 *	Defined by the SCSI layer.  We don't really care.
 *
 *	RETURNS:
 *	Zero on success, negative errno on error.
 */
static int ata_get_identity(struct ata_port *ap, struct scsi_device *sdev,
			    void __user *arg)
{
	struct ata_device *dev = ata_scsi_find_dev(ap, sdev);
	u16 __user *dst = arg;
	char buf[40];

	if (!dev)
		return -ENOMSG;

	if (copy_to_user(dst, dev->id, ATA_ID_WORDS * sizeof(u16)))
		return -EFAULT;

	ata_id_string(dev->id, buf, ATA_ID_PROD, ATA_ID_PROD_LEN);
	if (copy_to_user(dst + ATA_ID_PROD, buf, ATA_ID_PROD_LEN))
		return -EFAULT;

	ata_id_string(dev->id, buf, ATA_ID_FW_REV, ATA_ID_FW_REV_LEN);
	if (copy_to_user(dst + ATA_ID_FW_REV, buf, ATA_ID_FW_REV_LEN))
		return -EFAULT;

	ata_id_string(dev->id, buf, ATA_ID_SERNO, ATA_ID_SERNO_LEN);
	if (copy_to_user(dst + ATA_ID_SERNO, buf, ATA_ID_SERNO_LEN))
		return -EFAULT;

	return 0;
}

/**
 *	ata_cmd_ioctl - Handler for HDIO_DRIVE_CMD ioctl
 *	@scsidev: Device to which we are issuing command
 *	@arg: User provided data for issuing command
 *
 *	LOCKING:
 *	Defined by the SCSI layer.  We don't really care.
 *
 *	RETURNS:
 *	Zero on success, negative errno on error.
 */
int ata_cmd_ioctl(struct scsi_device *scsidev, void __user *arg)
{
	int rc = 0;
	u8 sensebuf[SCSI_SENSE_BUFFERSIZE];
	u8 scsi_cmd[MAX_COMMAND_SIZE];
	u8 args[4], *argbuf = NULL;
	int argsize = 0;
	struct scsi_sense_hdr sshdr;
	const struct scsi_exec_args exec_args = {
		.sshdr = &sshdr,
		.sense = sensebuf,
		.sense_len = sizeof(sensebuf),
	};
	int cmd_result;

	if (arg == NULL)
		return -EINVAL;

	if (copy_from_user(args, arg, sizeof(args)))
		return -EFAULT;

	memset(sensebuf, 0, sizeof(sensebuf));
	memset(scsi_cmd, 0, sizeof(scsi_cmd));

	if (args[3]) {
		argsize = ATA_SECT_SIZE * args[3];
		argbuf = kmalloc(argsize, GFP_KERNEL);
		if (argbuf == NULL) {
			rc = -ENOMEM;
			goto error;
		}

		scsi_cmd[1]  = (4 << 1); /* PIO Data-in */
		scsi_cmd[2]  = 0x0e;     /* no off.line or cc, read from dev,
					    block count in sector count field */
	} else {
		scsi_cmd[1]  = (3 << 1); /* Non-data */
		scsi_cmd[2]  = 0x20;     /* cc but no off.line or data xfer */
	}

	scsi_cmd[0] = ATA_16;

	scsi_cmd[4] = args[2];
	if (args[0] == ATA_CMD_SMART) { /* hack -- ide driver does this too */
		scsi_cmd[6]  = args[3];
		scsi_cmd[8]  = args[1];
		scsi_cmd[10] = ATA_SMART_LBAM_PASS;
		scsi_cmd[12] = ATA_SMART_LBAH_PASS;
	} else {
		scsi_cmd[6]  = args[1];
	}
	scsi_cmd[14] = args[0];

	/* Good values for timeout and retries?  Values below
	   from scsi_ioctl_send_command() for default case... */
	cmd_result = scsi_execute_cmd(scsidev, scsi_cmd, REQ_OP_DRV_IN, argbuf,
				      argsize, 10 * HZ, 5, &exec_args);
	if (cmd_result < 0) {
		rc = cmd_result;
		goto error;
	}
	if (scsi_sense_valid(&sshdr)) {/* sense data available */
		u8 *desc = sensebuf + 8;

		/* If we set cc then ATA pass-through will cause a
		 * check condition even if no error. Filter that. */
		if (scsi_status_is_check_condition(cmd_result)) {
			if (sshdr.sense_key == RECOVERED_ERROR &&
			    sshdr.asc == 0 && sshdr.ascq == 0x1d)
				cmd_result &= ~SAM_STAT_CHECK_CONDITION;
		}

		/* Send userspace a few ATA registers (same as drivers/ide) */
		if (sensebuf[0] == 0x72 &&	/* format is "descriptor" */
		    desc[0] == 0x09) {		/* code is "ATA Descriptor" */
			args[0] = desc[13];	/* status */
			args[1] = desc[3];	/* error */
			args[2] = desc[5];	/* sector count (0:7) */
			if (copy_to_user(arg, args, sizeof(args)))
				rc = -EFAULT;
		}
	}


	if (cmd_result) {
		rc = -EIO;
		goto error;
	}

	if ((argbuf)
	 && copy_to_user(arg + sizeof(args), argbuf, argsize))
		rc = -EFAULT;
error:
	kfree(argbuf);
	return rc;
}

/**
 *	ata_task_ioctl - Handler for HDIO_DRIVE_TASK ioctl
 *	@scsidev: Device to which we are issuing command
 *	@arg: User provided data for issuing command
 *
 *	LOCKING:
 *	Defined by the SCSI layer.  We don't really care.
 *
 *	RETURNS:
 *	Zero on success, negative errno on error.
 */
int ata_task_ioctl(struct scsi_device *scsidev, void __user *arg)
{
	int rc = 0;
	u8 sensebuf[SCSI_SENSE_BUFFERSIZE];
	u8 scsi_cmd[MAX_COMMAND_SIZE];
	u8 args[7];
	struct scsi_sense_hdr sshdr;
	int cmd_result;
	const struct scsi_exec_args exec_args = {
		.sshdr = &sshdr,
		.sense = sensebuf,
		.sense_len = sizeof(sensebuf),
	};

	if (arg == NULL)
		return -EINVAL;

	if (copy_from_user(args, arg, sizeof(args)))
		return -EFAULT;

	memset(sensebuf, 0, sizeof(sensebuf));
	memset(scsi_cmd, 0, sizeof(scsi_cmd));
	scsi_cmd[0]  = ATA_16;
	scsi_cmd[1]  = (3 << 1); /* Non-data */
	scsi_cmd[2]  = 0x20;     /* cc but no off.line or data xfer */
	scsi_cmd[4]  = args[1];
	scsi_cmd[6]  = args[2];
	scsi_cmd[8]  = args[3];
	scsi_cmd[10] = args[4];
	scsi_cmd[12] = args[5];
	scsi_cmd[13] = args[6] & 0x4f;
	scsi_cmd[14] = args[0];

	/* Good values for timeout and retries?  Values below
	   from scsi_ioctl_send_command() for default case... */
	cmd_result = scsi_execute_cmd(scsidev, scsi_cmd, REQ_OP_DRV_IN, NULL,
				      0, 10 * HZ, 5, &exec_args);
	if (cmd_result < 0) {
		rc = cmd_result;
		goto error;
	}
	if (scsi_sense_valid(&sshdr)) {/* sense data available */
		u8 *desc = sensebuf + 8;

		/* If we set cc then ATA pass-through will cause a
		 * check condition even if no error. Filter that. */
		if (cmd_result & SAM_STAT_CHECK_CONDITION) {
			if (sshdr.sense_key == RECOVERED_ERROR &&
			    sshdr.asc == 0 && sshdr.ascq == 0x1d)
				cmd_result &= ~SAM_STAT_CHECK_CONDITION;
		}

		/* Send userspace ATA registers */
		if (sensebuf[0] == 0x72 &&	/* format is "descriptor" */
				desc[0] == 0x09) {/* code is "ATA Descriptor" */
			args[0] = desc[13];	/* status */
			args[1] = desc[3];	/* error */
			args[2] = desc[5];	/* sector count (0:7) */
			args[3] = desc[7];	/* lbal */
			args[4] = desc[9];	/* lbam */
			args[5] = desc[11];	/* lbah */
			args[6] = desc[12];	/* select */
			if (copy_to_user(arg, args, sizeof(args)))
				rc = -EFAULT;
		}
	}

	if (cmd_result) {
		rc = -EIO;
		goto error;
	}

 error:
	return rc;
}

static bool ata_ioc32(struct ata_port *ap)
{
	if (ap->flags & ATA_FLAG_PIO_DMA)
		return true;
	if (ap->pflags & ATA_PFLAG_PIO32)
		return true;
	return false;
}

/*
 * This handles both native and compat commands, so anything added
 * here must have a compatible argument, or check in_compat_syscall()
 */
int ata_sas_scsi_ioctl(struct ata_port *ap, struct scsi_device *scsidev,
		     unsigned int cmd, void __user *arg)
{
	unsigned long val;
	int rc = -EINVAL;
	unsigned long flags;

	switch (cmd) {
	case HDIO_GET_32BIT:
		spin_lock_irqsave(ap->lock, flags);
		val = ata_ioc32(ap);
		spin_unlock_irqrestore(ap->lock, flags);
#ifdef CONFIG_COMPAT
		if (in_compat_syscall())
			return put_user(val, (compat_ulong_t __user *)arg);
#endif
		return put_user(val, (unsigned long __user *)arg);

	case HDIO_SET_32BIT:
		val = (unsigned long) arg;
		rc = 0;
		spin_lock_irqsave(ap->lock, flags);
		if (ap->pflags & ATA_PFLAG_PIO32CHANGE) {
			if (val)
				ap->pflags |= ATA_PFLAG_PIO32;
			else
				ap->pflags &= ~ATA_PFLAG_PIO32;
		} else {
			if (val != ata_ioc32(ap))
				rc = -EINVAL;
		}
		spin_unlock_irqrestore(ap->lock, flags);
		return rc;

	case HDIO_GET_IDENTITY:
		return ata_get_identity(ap, scsidev, arg);

	case HDIO_DRIVE_CMD:
		if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
			return -EACCES;
		return ata_cmd_ioctl(scsidev, arg);

	case HDIO_DRIVE_TASK:
		if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
			return -EACCES;
		return ata_task_ioctl(scsidev, arg);

	default:
		rc = -ENOTTY;
		break;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(ata_sas_scsi_ioctl);

int ata_scsi_ioctl(struct scsi_device *scsidev, unsigned int cmd,
		   void __user *arg)
{
	return ata_sas_scsi_ioctl(ata_shost_to_port(scsidev->host),
				scsidev, cmd, arg);
}
EXPORT_SYMBOL_GPL(ata_scsi_ioctl);

/**
 *	ata_scsi_qc_new - acquire new ata_queued_cmd reference
 *	@dev: ATA device to which the new command is attached
 *	@cmd: SCSI command that originated this ATA command
 *
 *	Obtain a reference to an unused ata_queued_cmd structure,
 *	which is the basic libata structure representing a single
 *	ATA command sent to the hardware.
 *
 *	If a command was available, fill in the SCSI-specific
 *	portions of the structure with information on the
 *	current command.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Command allocated, or %NULL if none available.
 */
static struct ata_queued_cmd *ata_scsi_qc_new(struct ata_device *dev,
					      struct scsi_cmnd *cmd)
{
	struct ata_port *ap = dev->link->ap;
	struct ata_queued_cmd *qc;
	int tag;

	if (unlikely(ata_port_is_frozen(ap)))
		goto fail;

	if (ap->flags & ATA_FLAG_SAS_HOST) {
		/*
		 * SAS hosts may queue > ATA_MAX_QUEUE commands so use
		 * unique per-device budget token as a tag.
		 */
		if (WARN_ON_ONCE(cmd->budget_token >= ATA_MAX_QUEUE))
			goto fail;
		tag = cmd->budget_token;
	} else {
		tag = scsi_cmd_to_rq(cmd)->tag;
	}

	qc = __ata_qc_from_tag(ap, tag);
	qc->tag = qc->hw_tag = tag;
	qc->ap = ap;
	qc->dev = dev;

	ata_qc_reinit(qc);

	qc->scsicmd = cmd;
	qc->scsidone = scsi_done;

	qc->sg = scsi_sglist(cmd);
	qc->n_elem = scsi_sg_count(cmd);

	if (scsi_cmd_to_rq(cmd)->rq_flags & RQF_QUIET)
		qc->flags |= ATA_QCFLAG_QUIET;

	return qc;

fail:
	set_host_byte(cmd, DID_OK);
	set_status_byte(cmd, SAM_STAT_TASK_SET_FULL);
	scsi_done(cmd);
	return NULL;
}

static void ata_qc_set_pc_nbytes(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;

	qc->extrabytes = scmd->extra_len;
	qc->nbytes = scsi_bufflen(scmd) + qc->extrabytes;
}

/**
 *	ata_to_sense_error - convert ATA error to SCSI error
 *	@drv_stat: value contained in ATA status register
 *	@drv_err: value contained in ATA error register
 *	@sk: the sense key we'll fill out
 *	@asc: the additional sense code we'll fill out
 *	@ascq: the additional sense code qualifier we'll fill out
 *
 *	Converts an ATA error into a SCSI error.  Fill out pointers to
 *	SK, ASC, and ASCQ bytes for later use in fixed or descriptor
 *	format sense blocks.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static void ata_to_sense_error(u8 drv_stat, u8 drv_err, u8 *sk, u8 *asc,
			       u8 *ascq)
{
	int i;

	/* Based on the 3ware driver translation table */
	static const unsigned char sense_table[][4] = {
		/* BBD|ECC|ID|MAR */
		{0xd1,		ABORTED_COMMAND, 0x00, 0x00},
			// Device busy                  Aborted command
		/* BBD|ECC|ID */
		{0xd0,		ABORTED_COMMAND, 0x00, 0x00},
			// Device busy                  Aborted command
		/* ECC|MC|MARK */
		{0x61,		HARDWARE_ERROR, 0x00, 0x00},
			// Device fault                 Hardware error
		/* ICRC|ABRT */		/* NB: ICRC & !ABRT is BBD */
		{0x84,		ABORTED_COMMAND, 0x47, 0x00},
			// Data CRC error               SCSI parity error
		/* MC|ID|ABRT|TRK0|MARK */
		{0x37,		NOT_READY, 0x04, 0x00},
			// Unit offline                 Not ready
		/* MCR|MARK */
		{0x09,		NOT_READY, 0x04, 0x00},
			// Unrecovered disk error       Not ready
		/*  Bad address mark */
		{0x01,		MEDIUM_ERROR, 0x13, 0x00},
			// Address mark not found for data field
		/* TRK0 - Track 0 not found */
		{0x02,		HARDWARE_ERROR, 0x00, 0x00},
			// Hardware error
		/* Abort: 0x04 is not translated here, see below */
		/* Media change request */
		{0x08,		NOT_READY, 0x04, 0x00},
			// FIXME: faking offline
		/* SRV/IDNF - ID not found */
		{0x10,		ILLEGAL_REQUEST, 0x21, 0x00},
			// Logical address out of range
		/* MC - Media Changed */
		{0x20,		UNIT_ATTENTION, 0x28, 0x00},
			// Not ready to ready change, medium may have changed
		/* ECC - Uncorrectable ECC error */
		{0x40,		MEDIUM_ERROR, 0x11, 0x04},
			// Unrecovered read error
		/* BBD - block marked bad */
		{0x80,		MEDIUM_ERROR, 0x11, 0x04},
			// Block marked bad	Medium error, unrecovered read error
		{0xFF, 0xFF, 0xFF, 0xFF}, // END mark
	};
	static const unsigned char stat_table[][4] = {
		/* Must be first because BUSY means no other bits valid */
		{0x80,		ABORTED_COMMAND, 0x47, 0x00},
		// Busy, fake parity for now
		{0x40,		ILLEGAL_REQUEST, 0x21, 0x04},
		// Device ready, unaligned write command
		{0x20,		HARDWARE_ERROR,  0x44, 0x00},
		// Device fault, internal target failure
		{0x08,		ABORTED_COMMAND, 0x47, 0x00},
		// Timed out in xfer, fake parity for now
		{0x04,		RECOVERED_ERROR, 0x11, 0x00},
		// Recovered ECC error	  Medium error, recovered
		{0xFF, 0xFF, 0xFF, 0xFF}, // END mark
	};

	/*
	 *	Is this an error we can process/parse
	 */
	if (drv_stat & ATA_BUSY) {
		drv_err = 0;	/* Ignore the err bits, they're invalid */
	}

	if (drv_err) {
		/* Look for drv_err */
		for (i = 0; sense_table[i][0] != 0xFF; i++) {
			/* Look for best matches first */
			if ((sense_table[i][0] & drv_err) ==
			    sense_table[i][0]) {
				*sk = sense_table[i][1];
				*asc = sense_table[i][2];
				*ascq = sense_table[i][3];
				return;
			}
		}
	}

	/*
	 * Fall back to interpreting status bits.  Note that if the drv_err
	 * has only the ABRT bit set, we decode drv_stat.  ABRT by itself
	 * is not descriptive enough.
	 */
	for (i = 0; stat_table[i][0] != 0xFF; i++) {
		if (stat_table[i][0] & drv_stat) {
			*sk = stat_table[i][1];
			*asc = stat_table[i][2];
			*ascq = stat_table[i][3];
			return;
		}
	}

	/*
	 * We need a sensible error return here, which is tricky, and one
	 * that won't cause people to do things like return a disk wrongly.
	 */
	*sk = ABORTED_COMMAND;
	*asc = 0x00;
	*ascq = 0x00;
}

/*
 *	ata_gen_passthru_sense - Generate check condition sense block.
 *	@qc: Command that completed.
 *
 *	This function is specific to the ATA pass through commands.
 *	Regardless of whether the command errored or not, return a sense
 *	block. If there was no error, we get the request from an ATA
 *	passthrough command, so we use the following sense data:
 *	sk = RECOVERED ERROR
 *	asc,ascq = ATA PASS-THROUGH INFORMATION AVAILABLE
 *      
 *
 *	LOCKING:
 *	None.
 */
static void ata_gen_passthru_sense(struct ata_queued_cmd *qc)
{
	struct ata_device *dev = qc->dev;
	struct scsi_cmnd *cmd = qc->scsicmd;
	struct ata_taskfile *tf = &qc->result_tf;
	u8 sense_key, asc, ascq;

	if (!(qc->flags & ATA_QCFLAG_RTF_FILLED)) {
		ata_dev_dbg(dev,
			    "missing result TF: can't generate ATA PT sense data\n");
		return;
	}

	/*
	 * Use ata_to_sense_error() to map status register bits
	 * onto sense key, asc & ascq.
	 */
	if (qc->err_mask ||
	    tf->status & (ATA_BUSY | ATA_DF | ATA_ERR | ATA_DRQ)) {
		ata_to_sense_error(tf->status, tf->error,
				   &sense_key, &asc, &ascq);
		ata_scsi_set_sense(qc->dev, cmd, sense_key, asc, ascq);
	} else {
		/*
		 * ATA PASS-THROUGH INFORMATION AVAILABLE
		 *
		 * Note: we are supposed to call ata_scsi_set_sense(), which
		 * respects the D_SENSE bit, instead of unconditionally
		 * generating the sense data in descriptor format. However,
		 * because hdparm, hddtemp, and udisks incorrectly assume sense
		 * data in descriptor format, without even looking at the
		 * RESPONSE CODE field in the returned sense data (to see which
		 * format the returned sense data is in), we are stuck with
		 * being bug compatible with older kernels.
		 */
		scsi_build_sense(cmd, 1, RECOVERED_ERROR, 0, 0x1D);
	}
}

/**
 *	ata_gen_ata_sense - generate a SCSI fixed sense block
 *	@qc: Command that we are erroring out
 *
 *	Generate sense block for a failed ATA command @qc.
 *
 *	LOCKING:
 *	None.
 */
static void ata_gen_ata_sense(struct ata_queued_cmd *qc)
{
	struct ata_device *dev = qc->dev;
	struct scsi_cmnd *cmd = qc->scsicmd;
	struct ata_taskfile *tf = &qc->result_tf;
	u8 sense_key, asc, ascq;

	if (ata_dev_disabled(dev)) {
		/* Device disabled after error recovery */
		/* LOGICAL UNIT NOT READY, HARD RESET REQUIRED */
		ata_scsi_set_sense(dev, cmd, NOT_READY, 0x04, 0x21);
		return;
	}

	if (!(qc->flags & ATA_QCFLAG_RTF_FILLED)) {
		ata_dev_dbg(dev,
			    "missing result TF: can't generate sense data\n");
		return;
	}

	/* Use ata_to_sense_error() to map status register bits
	 * onto sense key, asc & ascq.
	 */
	if (qc->err_mask ||
	    tf->status & (ATA_BUSY | ATA_DF | ATA_ERR | ATA_DRQ)) {
		ata_to_sense_error(tf->status, tf->error,
				   &sense_key, &asc, &ascq);
		ata_scsi_set_sense(dev, cmd, sense_key, asc, ascq);
	} else {
		/* Could not decode error */
		ata_dev_warn(dev, "could not decode error status 0x%x err_mask 0x%x\n",
			     tf->status, qc->err_mask);
		ata_scsi_set_sense(dev, cmd, ABORTED_COMMAND, 0, 0);
		return;
	}
}

void ata_scsi_sdev_config(struct scsi_device *sdev)
{
	sdev->use_10_for_rw = 1;
	sdev->use_10_for_ms = 1;
	sdev->no_write_same = 1;

	/* Schedule policy is determined by ->qc_defer() callback and
	 * it needs to see every deferred qc.  Set dev_blocked to 1 to
	 * prevent SCSI midlayer from automatically deferring
	 * requests.
	 */
	sdev->max_device_blocked = 1;
}

/**
 *	ata_scsi_dma_need_drain - Check whether data transfer may overflow
 *	@rq: request to be checked
 *
 *	ATAPI commands which transfer variable length data to host
 *	might overflow due to application error or hardware bug.  This
 *	function checks whether overflow should be drained and ignored
 *	for @request.
 *
 *	LOCKING:
 *	None.
 *
 *	RETURNS:
 *	1 if ; otherwise, 0.
 */
bool ata_scsi_dma_need_drain(struct request *rq)
{
	struct scsi_cmnd *scmd = blk_mq_rq_to_pdu(rq);

	return atapi_cmd_type(scmd->cmnd[0]) == ATAPI_MISC;
}
EXPORT_SYMBOL_GPL(ata_scsi_dma_need_drain);

int ata_scsi_dev_config(struct scsi_device *sdev, struct queue_limits *lim,
		struct ata_device *dev)
{
	int depth = 1;

	if (!ata_id_has_unload(dev->id))
		dev->flags |= ATA_DFLAG_NO_UNLOAD;

	/* configure max sectors */
	dev->max_sectors = min(dev->max_sectors, sdev->host->max_sectors);
	lim->max_hw_sectors = dev->max_sectors;

	if (dev->class == ATA_DEV_ATAPI) {
		sdev->sector_size = ATA_SECT_SIZE;

		/* set DMA padding */
		lim->dma_pad_mask = ATA_DMA_PAD_SZ - 1;

		/* make room for appending the drain */
		lim->max_segments--;

		sdev->dma_drain_len = ATAPI_MAX_DRAIN;
		sdev->dma_drain_buf = kmalloc(sdev->dma_drain_len, GFP_NOIO);
		if (!sdev->dma_drain_buf) {
			ata_dev_err(dev, "drain buffer allocation failed\n");
			return -ENOMEM;
		}
	} else {
		sdev->sector_size = ata_id_logical_sector_size(dev->id);

		/*
		 * Ask the sd driver to issue START STOP UNIT on runtime suspend
		 * and resume and shutdown only. For system level suspend/resume,
		 * devices power state is handled directly by libata EH.
		 * Given that disks are always spun up on system resume, also
		 * make sure that the sd driver forces runtime suspended disks
		 * to be resumed to correctly reflect the power state of the
		 * device.
		 */
		sdev->manage_runtime_start_stop = 1;
		sdev->manage_shutdown = 1;
		sdev->force_runtime_start_on_system_start = 1;
	}

	/*
	 * ata_pio_sectors() expects buffer for each sector to not cross
	 * page boundary.  Enforce it by requiring buffers to be sector
	 * aligned, which works iff sector_size is not larger than
	 * PAGE_SIZE.  ATAPI devices also need the alignment as
	 * IDENTIFY_PACKET is executed as ATA_PROT_PIO.
	 */
	if (sdev->sector_size > PAGE_SIZE)
		ata_dev_warn(dev,
			"sector_size=%u > PAGE_SIZE, PIO may malfunction\n",
			sdev->sector_size);

	lim->dma_alignment = sdev->sector_size - 1;

	if (dev->flags & ATA_DFLAG_AN)
		set_bit(SDEV_EVT_MEDIA_CHANGE, sdev->supported_events);

	if (ata_ncq_supported(dev))
		depth = min(sdev->host->can_queue, ata_id_queue_depth(dev->id));
	depth = min(ATA_MAX_QUEUE, depth);
	scsi_change_queue_depth(sdev, depth);

	if (dev->flags & ATA_DFLAG_TRUSTED)
		sdev->security_supported = 1;

	dev->sdev = sdev;
	return 0;
}

/**
 *	ata_scsi_sdev_init - Early setup of SCSI device
 *	@sdev: SCSI device to examine
 *
 *	This is called from scsi_alloc_sdev() when the scsi device
 *	associated with an ATA device is scanned on a port.
 *
 *	LOCKING:
 *	Defined by SCSI layer.  We don't really care.
 */

int ata_scsi_sdev_init(struct scsi_device *sdev)
{
	struct ata_port *ap = ata_shost_to_port(sdev->host);
	struct device_link *link;

	ata_scsi_sdev_config(sdev);

	/*
	 * Create a link from the ata_port device to the scsi device to ensure
	 * that PM does suspend/resume in the correct order: the scsi device is
	 * consumer (child) and the ata port the supplier (parent).
	 */
	link = device_link_add(&sdev->sdev_gendev, &ap->tdev,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (!link) {
		ata_port_err(ap, "Failed to create link to scsi device %s\n",
			     dev_name(&sdev->sdev_gendev));
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ata_scsi_sdev_init);

/**
 *	ata_scsi_sdev_configure - Set SCSI device attributes
 *	@sdev: SCSI device to examine
 *	@lim: queue limits
 *
 *	This is called before we actually start reading
 *	and writing to the device, to configure certain
 *	SCSI mid-layer behaviors.
 *
 *	LOCKING:
 *	Defined by SCSI layer.  We don't really care.
 */

int ata_scsi_sdev_configure(struct scsi_device *sdev, struct queue_limits *lim)
{
	struct ata_port *ap = ata_shost_to_port(sdev->host);
	struct ata_device *dev = __ata_scsi_find_dev(ap, sdev);

	if (dev)
		return ata_scsi_dev_config(sdev, lim, dev);

	return 0;
}
EXPORT_SYMBOL_GPL(ata_scsi_sdev_configure);

/**
 *	ata_scsi_sdev_destroy - SCSI device is about to be destroyed
 *	@sdev: SCSI device to be destroyed
 *
 *	@sdev is about to be destroyed for hot/warm unplugging.  If
 *	this unplugging was initiated by libata as indicated by NULL
 *	dev->sdev, this function doesn't have to do anything.
 *	Otherwise, SCSI layer initiated warm-unplug is in progress.
 *	Clear dev->sdev, schedule the device for ATA detach and invoke
 *	EH.
 *
 *	LOCKING:
 *	Defined by SCSI layer.  We don't really care.
 */
void ata_scsi_sdev_destroy(struct scsi_device *sdev)
{
	struct ata_port *ap = ata_shost_to_port(sdev->host);
	unsigned long flags;
	struct ata_device *dev;

	device_link_remove(&sdev->sdev_gendev, &ap->tdev);

	spin_lock_irqsave(ap->lock, flags);
	dev = __ata_scsi_find_dev(ap, sdev);
	if (dev && dev->sdev) {
		/* SCSI device already in CANCEL state, no need to offline it */
		dev->sdev = NULL;
		dev->flags |= ATA_DFLAG_DETACH;
		ata_port_schedule_eh(ap);
	}
	spin_unlock_irqrestore(ap->lock, flags);

	kfree(sdev->dma_drain_buf);
}
EXPORT_SYMBOL_GPL(ata_scsi_sdev_destroy);

/**
 *	ata_scsi_start_stop_xlat - Translate SCSI START STOP UNIT command
 *	@qc: Storage for translated ATA taskfile
 *
 *	Sets up an ATA taskfile to issue STANDBY (to stop) or READ VERIFY
 *	(to start). Perhaps these commands should be preceded by
 *	CHECK POWER MODE to see what power mode the device is already in.
 *	[See SAT revision 5 at www.t10.org]
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Zero on success, non-zero on error.
 */
static unsigned int ata_scsi_start_stop_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	const u8 *cdb = scmd->cmnd;
	u16 fp;
	u8 bp = 0xff;

	if (scmd->cmd_len < 5) {
		fp = 4;
		goto invalid_fld;
	}

	/* LOEJ bit set not supported */
	if (cdb[4] & 0x2) {
		fp = 4;
		bp = 1;
		goto invalid_fld;
	}

	/* Power conditions not supported */
	if (((cdb[4] >> 4) & 0xf) != 0) {
		fp = 4;
		bp = 3;
		goto invalid_fld;
	}

	/* Ignore IMMED bit (cdb[1] & 0x1), violates sat-r05 */
	if (!ata_dev_power_init_tf(qc->dev, &qc->tf, cdb[4] & 0x1)) {
		ata_scsi_set_sense(qc->dev, scmd, ABORTED_COMMAND, 0, 0);
		return 1;
	}

	/*
	 * Standby and Idle condition timers could be implemented but that
	 * would require libata to implement the Power condition mode page
	 * and allow the user to change it. Changing mode pages requires
	 * MODE SELECT to be implemented.
	 */

	return 0;

 invalid_fld:
	ata_scsi_set_invalid_field(qc->dev, scmd, fp, bp);
	return 1;
}

/**
 *	ata_scsi_flush_xlat - Translate SCSI SYNCHRONIZE CACHE command
 *	@qc: Storage for translated ATA taskfile
 *
 *	Sets up an ATA taskfile to issue FLUSH CACHE or
 *	FLUSH CACHE EXT.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Zero on success, non-zero on error.
 */
static unsigned int ata_scsi_flush_xlat(struct ata_queued_cmd *qc)
{
	struct ata_taskfile *tf = &qc->tf;

	tf->flags |= ATA_TFLAG_DEVICE;
	tf->protocol = ATA_PROT_NODATA;

	if (qc->dev->flags & ATA_DFLAG_FLUSH_EXT)
		tf->command = ATA_CMD_FLUSH_EXT;
	else
		tf->command = ATA_CMD_FLUSH;

	/* flush is critical for IO integrity, consider it an IO command */
	qc->flags |= ATA_QCFLAG_IO;

	return 0;
}

/**
 *	scsi_6_lba_len - Get LBA and transfer length
 *	@cdb: SCSI command to translate
 *
 *	Calculate LBA and transfer length for 6-byte commands.
 *
 *	RETURNS:
 *	@plba: the LBA
 *	@plen: the transfer length
 */
static void scsi_6_lba_len(const u8 *cdb, u64 *plba, u32 *plen)
{
	*plba = get_unaligned_be24(&cdb[1]) & 0x1fffff;
	*plen = cdb[4];
}

/**
 *	scsi_10_lba_len - Get LBA and transfer length
 *	@cdb: SCSI command to translate
 *
 *	Calculate LBA and transfer length for 10-byte commands.
 *
 *	RETURNS:
 *	@plba: the LBA
 *	@plen: the transfer length
 */
static inline void scsi_10_lba_len(const u8 *cdb, u64 *plba, u32 *plen)
{
	*plba = get_unaligned_be32(&cdb[2]);
	*plen = get_unaligned_be16(&cdb[7]);
}

/**
 *	scsi_16_lba_len - Get LBA and transfer length
 *	@cdb: SCSI command to translate
 *
 *	Calculate LBA and transfer length for 16-byte commands.
 *
 *	RETURNS:
 *	@plba: the LBA
 *	@plen: the transfer length
 */
static inline void scsi_16_lba_len(const u8 *cdb, u64 *plba, u32 *plen)
{
	*plba = get_unaligned_be64(&cdb[2]);
	*plen = get_unaligned_be32(&cdb[10]);
}

/**
 *	scsi_dld - Get duration limit descriptor index
 *	@cdb: SCSI command to translate
 *
 *	Returns the dld bits indicating the index of a command duration limit
 *	descriptor.
 */
static inline int scsi_dld(const u8 *cdb)
{
	return ((cdb[1] & 0x01) << 2) | ((cdb[14] >> 6) & 0x03);
}

/**
 *	ata_scsi_verify_xlat - Translate SCSI VERIFY command into an ATA one
 *	@qc: Storage for translated ATA taskfile
 *
 *	Converts SCSI VERIFY command to an ATA READ VERIFY command.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Zero on success, non-zero on error.
 */
static unsigned int ata_scsi_verify_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	struct ata_taskfile *tf = &qc->tf;
	struct ata_device *dev = qc->dev;
	u64 dev_sectors = qc->dev->n_sectors;
	const u8 *cdb = scmd->cmnd;
	u64 block;
	u32 n_block;
	u16 fp;

	tf->flags |= ATA_TFLAG_ISADDR | ATA_TFLAG_DEVICE;
	tf->protocol = ATA_PROT_NODATA;

	switch (cdb[0]) {
	case VERIFY:
		if (scmd->cmd_len < 10) {
			fp = 9;
			goto invalid_fld;
		}
		scsi_10_lba_len(cdb, &block, &n_block);
		break;
	case VERIFY_16:
		if (scmd->cmd_len < 16) {
			fp = 15;
			goto invalid_fld;
		}
		scsi_16_lba_len(cdb, &block, &n_block);
		break;
	default:
		fp = 0;
		goto invalid_fld;
	}

	if (!n_block)
		goto nothing_to_do;
	if (block >= dev_sectors)
		goto out_of_range;
	if ((block + n_block) > dev_sectors)
		goto out_of_range;

	if (dev->flags & ATA_DFLAG_LBA) {
		tf->flags |= ATA_TFLAG_LBA;

		if (lba_28_ok(block, n_block)) {
			/* use LBA28 */
			tf->command = ATA_CMD_VERIFY;
			tf->device |= (block >> 24) & 0xf;
		} else if (lba_48_ok(block, n_block)) {
			if (!(dev->flags & ATA_DFLAG_LBA48))
				goto out_of_range;

			/* use LBA48 */
			tf->flags |= ATA_TFLAG_LBA48;
			tf->command = ATA_CMD_VERIFY_EXT;

			tf->hob_nsect = (n_block >> 8) & 0xff;

			tf->hob_lbah = (block >> 40) & 0xff;
			tf->hob_lbam = (block >> 32) & 0xff;
			tf->hob_lbal = (block >> 24) & 0xff;
		} else
			/* request too large even for LBA48 */
			goto out_of_range;

		tf->nsect = n_block & 0xff;

		tf->lbah = (block >> 16) & 0xff;
		tf->lbam = (block >> 8) & 0xff;
		tf->lbal = block & 0xff;

		tf->device |= ATA_LBA;
	} else {
		/* CHS */
		u32 sect, head, cyl, track;

		if (!lba_28_ok(block, n_block))
			goto out_of_range;

		/* Convert LBA to CHS */
		track = (u32)block / dev->sectors;
		cyl   = track / dev->heads;
		head  = track % dev->heads;
		sect  = (u32)block % dev->sectors + 1;

		/* Check whether the converted CHS can fit.
		   Cylinder: 0-65535
		   Head: 0-15
		   Sector: 1-255*/
		if ((cyl >> 16) || (head >> 4) || (sect >> 8) || (!sect))
			goto out_of_range;

		tf->command = ATA_CMD_VERIFY;
		tf->nsect = n_block & 0xff; /* Sector count 0 means 256 sectors */
		tf->lbal = sect;
		tf->lbam = cyl;
		tf->lbah = cyl >> 8;
		tf->device |= head;
	}

	return 0;

invalid_fld:
	ata_scsi_set_invalid_field(qc->dev, scmd, fp, 0xff);
	return 1;

out_of_range:
	ata_scsi_set_sense(qc->dev, scmd, ILLEGAL_REQUEST, 0x21, 0x0);
	/* "Logical Block Address out of range" */
	return 1;

nothing_to_do:
	scmd->result = SAM_STAT_GOOD;
	return 1;
}

static bool ata_check_nblocks(struct scsi_cmnd *scmd, u32 n_blocks)
{
	struct request *rq = scsi_cmd_to_rq(scmd);
	u32 req_blocks;

	if (!blk_rq_is_passthrough(rq))
		return true;

	req_blocks = blk_rq_bytes(rq) / scmd->device->sector_size;
	if (n_blocks > req_blocks)
		return false;

	return true;
}

/**
 *	ata_scsi_rw_xlat - Translate SCSI r/w command into an ATA one
 *	@qc: Storage for translated ATA taskfile
 *
 *	Converts any of six SCSI read/write commands into the
 *	ATA counterpart, including starting sector (LBA),
 *	sector count, and taking into account the device's LBA48
 *	support.
 *
 *	Commands %READ_6, %READ_10, %READ_16, %WRITE_6, %WRITE_10, and
 *	%WRITE_16 are currently supported.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Zero on success, non-zero on error.
 */
static unsigned int ata_scsi_rw_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	const u8 *cdb = scmd->cmnd;
	struct request *rq = scsi_cmd_to_rq(scmd);
	int class = IOPRIO_PRIO_CLASS(req_get_ioprio(rq));
	unsigned int tf_flags = 0;
	int dld = 0;
	u64 block;
	u32 n_block;
	int rc;
	u16 fp = 0;

	switch (cdb[0]) {
	case WRITE_6:
	case WRITE_10:
	case WRITE_16:
		tf_flags |= ATA_TFLAG_WRITE;
		break;
	}

	/* Calculate the SCSI LBA, transfer length and FUA. */
	switch (cdb[0]) {
	case READ_10:
	case WRITE_10:
		if (unlikely(scmd->cmd_len < 10)) {
			fp = 9;
			goto invalid_fld;
		}
		scsi_10_lba_len(cdb, &block, &n_block);
		if (cdb[1] & (1 << 3))
			tf_flags |= ATA_TFLAG_FUA;
		if (!ata_check_nblocks(scmd, n_block))
			goto invalid_fld;
		break;
	case READ_6:
	case WRITE_6:
		if (unlikely(scmd->cmd_len < 6)) {
			fp = 5;
			goto invalid_fld;
		}
		scsi_6_lba_len(cdb, &block, &n_block);

		/* for 6-byte r/w commands, transfer length 0
		 * means 256 blocks of data, not 0 block.
		 */
		if (!n_block)
			n_block = 256;
		if (!ata_check_nblocks(scmd, n_block))
			goto invalid_fld;
		break;
	case READ_16:
	case WRITE_16:
		if (unlikely(scmd->cmd_len < 16)) {
			fp = 15;
			goto invalid_fld;
		}
		scsi_16_lba_len(cdb, &block, &n_block);
		dld = scsi_dld(cdb);
		if (cdb[1] & (1 << 3))
			tf_flags |= ATA_TFLAG_FUA;
		if (!ata_check_nblocks(scmd, n_block))
			goto invalid_fld;
		break;
	default:
		fp = 0;
		goto invalid_fld;
	}

	/* Check and compose ATA command */
	if (!n_block)
		/* For 10-byte and 16-byte SCSI R/W commands, transfer
		 * length 0 means transfer 0 block of data.
		 * However, for ATA R/W commands, sector count 0 means
		 * 256 or 65536 sectors, not 0 sectors as in SCSI.
		 *
		 * WARNING: one or two older ATA drives treat 0 as 0...
		 */
		goto nothing_to_do;

	qc->flags |= ATA_QCFLAG_IO;
	qc->nbytes = n_block * scmd->device->sector_size;

	rc = ata_build_rw_tf(qc, block, n_block, tf_flags, dld, class);
	if (likely(rc == 0))
		return 0;

	if (rc == -ERANGE)
		goto out_of_range;
	/* treat all other errors as -EINVAL, fall through */
invalid_fld:
	ata_scsi_set_invalid_field(qc->dev, scmd, fp, 0xff);
	return 1;

out_of_range:
	ata_scsi_set_sense(qc->dev, scmd, ILLEGAL_REQUEST, 0x21, 0x0);
	/* "Logical Block Address out of range" */
	return 1;

nothing_to_do:
	scmd->result = SAM_STAT_GOOD;
	return 1;
}

static void ata_qc_done(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *cmd = qc->scsicmd;
	void (*done)(struct scsi_cmnd *) = qc->scsidone;

	ata_qc_free(qc);
	done(cmd);
}

static void ata_scsi_qc_complete(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *cmd = qc->scsicmd;
	u8 *cdb = cmd->cmnd;
	bool have_sense = qc->flags & ATA_QCFLAG_SENSE_VALID;
	bool is_ata_passthru = cdb[0] == ATA_16 || cdb[0] == ATA_12;
	bool is_ck_cond_request = cdb[2] & 0x20;
	bool is_error = qc->err_mask != 0;

	/* For ATA pass thru (SAT) commands, generate a sense block if
	 * user mandated it or if there's an error.  Note that if we
	 * generate because the user forced us to [CK_COND=1], a check
	 * condition is generated and the ATA register values are returned
	 * whether the command completed successfully or not. If there
	 * was no error, and CK_COND=1, we use the following sense data:
	 * sk = RECOVERED ERROR
	 * asc,ascq = ATA PASS-THROUGH INFORMATION AVAILABLE
	 */
	if (is_ata_passthru && (is_ck_cond_request || is_error || have_sense)) {
		if (!have_sense)
			ata_gen_passthru_sense(qc);
		ata_scsi_set_passthru_sense_fields(qc);
		if (is_ck_cond_request)
			set_status_byte(qc->scsicmd, SAM_STAT_CHECK_CONDITION);
	} else if (is_error) {
		if (!have_sense)
			ata_gen_ata_sense(qc);
		ata_scsi_set_sense_information(qc);
	}

	ata_qc_done(qc);
}

/**
 *	ata_scsi_translate - Translate then issue SCSI command to ATA device
 *	@dev: ATA device to which the command is addressed
 *	@cmd: SCSI command to execute
 *	@xlat_func: Actor which translates @cmd to an ATA taskfile
 *
 *	Our ->queuecommand() function has decided that the SCSI
 *	command issued can be directly translated into an ATA
 *	command, rather than handled internally.
 *
 *	This function sets up an ata_queued_cmd structure for the
 *	SCSI command, and sends that ata_queued_cmd to the hardware.
 *
 *	The xlat_func argument (actor) returns 0 if ready to execute
 *	ATA command, else 1 to finish translation. If 1 is returned
 *	then cmd->result (and possibly cmd->sense_buffer) are assumed
 *	to be set reflecting an error condition or clean (early)
 *	termination.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	0 on success, SCSI_ML_QUEUE_DEVICE_BUSY if the command
 *	needs to be deferred.
 */
static int ata_scsi_translate(struct ata_device *dev, struct scsi_cmnd *cmd,
			      ata_xlat_func_t xlat_func)
{
	struct ata_port *ap = dev->link->ap;
	struct ata_queued_cmd *qc;
	int rc;

	qc = ata_scsi_qc_new(dev, cmd);
	if (!qc)
		goto err_mem;

	/* data is present; dma-map it */
	if (cmd->sc_data_direction == DMA_FROM_DEVICE ||
	    cmd->sc_data_direction == DMA_TO_DEVICE) {
		if (unlikely(scsi_bufflen(cmd) < 1)) {
			ata_dev_warn(dev, "WARNING: zero len r/w req\n");
			goto err_did;
		}

		ata_sg_init(qc, scsi_sglist(cmd), scsi_sg_count(cmd));

		qc->dma_dir = cmd->sc_data_direction;
	}

	qc->complete_fn = ata_scsi_qc_complete;

	if (xlat_func(qc))
		goto early_finish;

	if (ap->ops->qc_defer) {
		if ((rc = ap->ops->qc_defer(qc)))
			goto defer;
	}

	/* select device, send command to hardware */
	ata_qc_issue(qc);

	return 0;

early_finish:
	ata_qc_free(qc);
	scsi_done(cmd);
	return 0;

err_did:
	ata_qc_free(qc);
	cmd->result = (DID_ERROR << 16);
	scsi_done(cmd);
err_mem:
	return 0;

defer:
	ata_qc_free(qc);
	if (rc == ATA_DEFER_LINK)
		return SCSI_MLQUEUE_DEVICE_BUSY;
	else
		return SCSI_MLQUEUE_HOST_BUSY;
}

/**
 *	ata_scsi_rbuf_fill - wrapper for SCSI command simulators
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@actor: Callback hook for desired SCSI command simulator
 *
 *	Takes care of the hard work of simulating a SCSI command...
 *	Mapping the response buffer, calling the command's handler,
 *	and handling the handler's return value.  This return value
 *	indicates whether the handler wishes the SCSI command to be
 *	completed successfully (0), or not (in which case cmd->result
 *	and sense buffer are assumed to be set).
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static void ata_scsi_rbuf_fill(struct ata_device *dev, struct scsi_cmnd *cmd,
		unsigned int (*actor)(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf))
{
	unsigned long flags;
	unsigned int len;

	spin_lock_irqsave(&ata_scsi_rbuf_lock, flags);

	memset(ata_scsi_rbuf, 0, ATA_SCSI_RBUF_SIZE);
	len = actor(dev, cmd, ata_scsi_rbuf);
	if (len) {
		sg_copy_from_buffer(scsi_sglist(cmd), scsi_sg_count(cmd),
				    ata_scsi_rbuf, ATA_SCSI_RBUF_SIZE);
		cmd->result = SAM_STAT_GOOD;
		if (scsi_bufflen(cmd) > len)
			scsi_set_resid(cmd, scsi_bufflen(cmd) - len);
	}

	spin_unlock_irqrestore(&ata_scsi_rbuf_lock, flags);
}

/**
 *	ata_scsiop_inq_std - Simulate standard INQUIRY command
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Returns standard device identification data associated
 *	with non-VPD INQUIRY command output.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_std(struct ata_device *dev,
				       struct scsi_cmnd *cmd, u8 *rbuf)
{
	static const u8 versions[] = {
		0x00,
		0x60,	/* SAM-3 (no version claimed) */

		0x03,
		0x20,	/* SBC-2 (no version claimed) */

		0x03,
		0x00	/* SPC-3 (no version claimed) */
	};
	static const u8 versions_zbc[] = {
		0x00,
		0xA0,	/* SAM-5 (no version claimed) */

		0x06,
		0x00,	/* SBC-4 (no version claimed) */

		0x05,
		0xC0,	/* SPC-5 (no version claimed) */

		0x60,
		0x24,   /* ZBC r05 */
	};

	u8 hdr[] = {
		TYPE_DISK,
		0,
		0x5,	/* claim SPC-3 version compatibility */
		2,
		95 - 4,
		0,
		0,
		2
	};

	/*
	 * Set the SCSI Removable Media Bit (RMB) if the ATA removable media
	 * device bit (obsolete since ATA-8 ACS) is set.
	 */
	if (ata_id_removable(dev->id))
		hdr[1] |= (1 << 7);

	if (dev->class == ATA_DEV_ZAC) {
		hdr[0] = TYPE_ZBC;
		hdr[2] = 0x7; /* claim SPC-5 version compatibility */
	}

	if (dev->flags & ATA_DFLAG_CDL)
		hdr[2] = 0xd; /* claim SPC-6 version compatibility */

	memcpy(rbuf, hdr, sizeof(hdr));
	memcpy(&rbuf[8], "ATA     ", 8);
	ata_id_string(dev->id, &rbuf[16], ATA_ID_PROD, 16);

	/* From SAT, use last 2 words from fw rev unless they are spaces */
	ata_id_string(dev->id, &rbuf[32], ATA_ID_FW_REV + 2, 4);
	if (strncmp(&rbuf[32], "    ", 4) == 0)
		ata_id_string(dev->id, &rbuf[32], ATA_ID_FW_REV, 4);

	if (rbuf[32] == 0 || rbuf[32] == ' ')
		memcpy(&rbuf[32], "n/a ", 4);

	if (ata_id_zoned_cap(dev->id) || dev->class == ATA_DEV_ZAC)
		memcpy(rbuf + 58, versions_zbc, sizeof(versions_zbc));
	else
		memcpy(rbuf + 58, versions, sizeof(versions));

	/*
	 * Include all 8 possible version descriptors, even if not all of
	 * them are popoulated.
	 */
	return 96;
}

/**
 *	ata_scsiop_inq_00 - Simulate INQUIRY VPD page 0, list of pages
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Returns list of inquiry VPD pages available.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_00(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	int i, num_pages = 0;
	static const u8 pages[] = {
		0x00,	/* page 0x00, this page */
		0x80,	/* page 0x80, unit serial no page */
		0x83,	/* page 0x83, device ident page */
		0x89,	/* page 0x89, ata info page */
		0xb0,	/* page 0xb0, block limits page */
		0xb1,	/* page 0xb1, block device characteristics page */
		0xb2,	/* page 0xb2, thin provisioning page */
		0xb6,	/* page 0xb6, zoned block device characteristics */
		0xb9,	/* page 0xb9, concurrent positioning ranges */
	};

	for (i = 0; i < sizeof(pages); i++) {
		if (pages[i] == 0xb6 && !ata_dev_is_zac(dev))
			continue;
		rbuf[num_pages + 4] = pages[i];
		num_pages++;
	}
	rbuf[3] = num_pages;	/* number of supported VPD pages */

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_80 - Simulate INQUIRY VPD page 80, device serial number
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Returns ATA device serial number.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_80(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	static const u8 hdr[] = {
		0,
		0x80,			/* this page code */
		0,
		ATA_ID_SERNO_LEN,	/* page len */
	};

	memcpy(rbuf, hdr, sizeof(hdr));
	ata_id_string(dev->id, (unsigned char *) &rbuf[4],
		      ATA_ID_SERNO, ATA_ID_SERNO_LEN);

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_83 - Simulate INQUIRY VPD page 83, device identity
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Yields two logical unit device identification designators:
 *	 - vendor specific ASCII containing the ATA serial number
 *	 - SAT defined "t10 vendor id based" containing ASCII vendor
 *	   name ("ATA     "), model and serial numbers.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_83(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	const int sat_model_serial_desc_len = 68;
	int num;

	rbuf[1] = 0x83;			/* this page code */
	num = 4;

	/* piv=0, assoc=lu, code_set=ACSII, designator=vendor */
	rbuf[num + 0] = 2;
	rbuf[num + 3] = ATA_ID_SERNO_LEN;
	num += 4;
	ata_id_string(dev->id, (unsigned char *) rbuf + num,
		      ATA_ID_SERNO, ATA_ID_SERNO_LEN);
	num += ATA_ID_SERNO_LEN;

	/* SAT defined lu model and serial numbers descriptor */
	/* piv=0, assoc=lu, code_set=ACSII, designator=t10 vendor id */
	rbuf[num + 0] = 2;
	rbuf[num + 1] = 1;
	rbuf[num + 3] = sat_model_serial_desc_len;
	num += 4;
	memcpy(rbuf + num, "ATA     ", 8);
	num += 8;
	ata_id_string(dev->id, (unsigned char *) rbuf + num, ATA_ID_PROD,
		      ATA_ID_PROD_LEN);
	num += ATA_ID_PROD_LEN;
	ata_id_string(dev->id, (unsigned char *) rbuf + num, ATA_ID_SERNO,
		      ATA_ID_SERNO_LEN);
	num += ATA_ID_SERNO_LEN;

	if (ata_id_has_wwn(dev->id)) {
		/* SAT defined lu world wide name */
		/* piv=0, assoc=lu, code_set=binary, designator=NAA */
		rbuf[num + 0] = 1;
		rbuf[num + 1] = 3;
		rbuf[num + 3] = ATA_ID_WWN_LEN;
		num += 4;
		ata_id_string(dev->id, (unsigned char *) rbuf + num,
			      ATA_ID_WWN, ATA_ID_WWN_LEN);
		num += ATA_ID_WWN_LEN;
	}
	rbuf[3] = num - 4;    /* page len (assume less than 256 bytes) */

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_89 - Simulate INQUIRY VPD page 89, ATA info
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Yields SAT-specified ATA VPD page.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_89(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	rbuf[1] = 0x89;			/* our page code */
	rbuf[2] = (0x238 >> 8);		/* page size fixed at 238h */
	rbuf[3] = (0x238 & 0xff);

	memcpy(&rbuf[8], "linux   ", 8);
	memcpy(&rbuf[16], "libata          ", 16);
	memcpy(&rbuf[32], DRV_VERSION, 4);

	rbuf[36] = 0x34;		/* force D2H Reg FIS (34h) */
	rbuf[37] = (1 << 7);		/* bit 7 indicates Command FIS */
					/* TODO: PMP? */

	/* we don't store the ATA device signature, so we fake it */
	rbuf[38] = ATA_DRDY;		/* really, this is Status reg */
	rbuf[40] = 0x1;
	rbuf[48] = 0x1;

	rbuf[56] = ATA_CMD_ID_ATA;

	memcpy(&rbuf[60], &dev->id[0], 512);

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_b0 - Simulate INQUIRY VPD page B0, Block Limits
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Return data for the VPD page B0h (Block Limits).
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_b0(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	u16 min_io_sectors;

	rbuf[1] = 0xb0;
	rbuf[3] = 0x3c;		/* required VPD size with unmap support */

	/*
	 * Optimal transfer length granularity.
	 *
	 * This is always one physical block, but for disks with a smaller
	 * logical than physical sector size we need to figure out what the
	 * latter is.
	 */
	min_io_sectors = 1 << ata_id_log2_per_physical_sector(dev->id);
	put_unaligned_be16(min_io_sectors, &rbuf[6]);

	/*
	 * Optimal unmap granularity.
	 *
	 * The ATA spec doesn't even know about a granularity or alignment
	 * for the TRIM command.  We can leave away most of the unmap related
	 * VPD page entries, but we have specifify a granularity to signal
	 * that we support some form of unmap - in thise case via WRITE SAME
	 * with the unmap bit set.
	 */
	if (ata_id_has_trim(dev->id)) {
		u64 max_blocks = 65535 * ATA_MAX_TRIM_RNUM;

		if (dev->quirks & ATA_QUIRK_MAX_TRIM_128M)
			max_blocks = 128 << (20 - SECTOR_SHIFT);

		put_unaligned_be64(max_blocks, &rbuf[36]);
		put_unaligned_be32(1, &rbuf[28]);
	}

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_b1 - Simulate INQUIRY VPD page B1, Block Device
 *			    Characteristics
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Return data for the VPD page B1h (Block Device Characteristics).
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_b1(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	int form_factor = ata_id_form_factor(dev->id);
	int media_rotation_rate = ata_id_rotation_rate(dev->id);
	u8 zoned = ata_id_zoned_cap(dev->id);

	rbuf[1] = 0xb1;
	rbuf[3] = 0x3c;
	rbuf[4] = media_rotation_rate >> 8;
	rbuf[5] = media_rotation_rate;
	rbuf[7] = form_factor;
	if (zoned)
		rbuf[8] = (zoned << 4);

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_b2 - Simulate INQUIRY VPD page B2, Logical Block
 *			    Provisioning
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Return data for the VPD page B2h (Logical Block Provisioning).
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_b2(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	/* SCSI Thin Provisioning VPD page: SBC-3 rev 22 or later */
	rbuf[1] = 0xb2;
	rbuf[3] = 0x4;
	rbuf[5] = 1 << 6;	/* TPWS */

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_b6 - Simulate INQUIRY VPD page B6, Zoned Block Device
 *			    Characteristics
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Return data for the VPD page B2h (Zoned Block Device Characteristics).
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_b6(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	if (!ata_dev_is_zac(dev)) {
		ata_scsi_set_invalid_field(dev, cmd, 2, 0xff);
		return 0;
	}

	/*
	 * zbc-r05 SCSI Zoned Block device characteristics VPD page
	 */
	rbuf[1] = 0xb6;
	rbuf[3] = 0x3C;

	/*
	 * URSWRZ bit is only meaningful for host-managed ZAC drives
	 */
	if (dev->zac_zoned_cap & 1)
		rbuf[4] |= 1;
	put_unaligned_be32(dev->zac_zones_optimal_open, &rbuf[8]);
	put_unaligned_be32(dev->zac_zones_optimal_nonseq, &rbuf[12]);
	put_unaligned_be32(dev->zac_zones_max_open, &rbuf[16]);

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inq_b9 - Simulate INQUIRY VPD page B9, Concurrent Positioning
 *			    Ranges
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Return data for the VPD page B9h (Concurrent Positioning Ranges).
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inq_b9(struct ata_device *dev,
				      struct scsi_cmnd *cmd, u8 *rbuf)
{
	struct ata_cpr_log *cpr_log = dev->cpr_log;
	u8 *desc = &rbuf[64];
	int i;

	if (!cpr_log) {
		ata_scsi_set_invalid_field(dev, cmd, 2, 0xff);
		return 0;
	}

	/* SCSI Concurrent Positioning Ranges VPD page: SBC-5 rev 1 or later */
	rbuf[1] = 0xb9;
	put_unaligned_be16(64 + (int)cpr_log->nr_cpr * 32 - 4, &rbuf[2]);

	for (i = 0; i < cpr_log->nr_cpr; i++, desc += 32) {
		desc[0] = cpr_log->cpr[i].num;
		desc[1] = cpr_log->cpr[i].num_storage_elements;
		put_unaligned_be64(cpr_log->cpr[i].start_lba, &desc[8]);
		put_unaligned_be64(cpr_log->cpr[i].num_lbas, &desc[16]);
	}

	return get_unaligned_be16(&rbuf[2]) + 4;
}

/**
 *	ata_scsiop_inquiry - Simulate INQUIRY command
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Returns data associated with an INQUIRY command output.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_inquiry(struct ata_device *dev,
				       struct scsi_cmnd *cmd, u8 *rbuf)
{
	const u8 *scsicmd = cmd->cmnd;

	/* is CmdDt set?  */
	if (scsicmd[1] & 2) {
		ata_scsi_set_invalid_field(dev, cmd, 1, 0xff);
		return 0;
	}

	/* Is EVPD clear? */
	if ((scsicmd[1] & 1) == 0)
		return ata_scsiop_inq_std(dev, cmd, rbuf);

	switch (scsicmd[2]) {
	case 0x00:
		return ata_scsiop_inq_00(dev, cmd, rbuf);
	case 0x80:
		return ata_scsiop_inq_80(dev, cmd, rbuf);
	case 0x83:
		return ata_scsiop_inq_83(dev, cmd, rbuf);
	case 0x89:
		return ata_scsiop_inq_89(dev, cmd, rbuf);
	case 0xb0:
		return ata_scsiop_inq_b0(dev, cmd, rbuf);
	case 0xb1:
		return ata_scsiop_inq_b1(dev, cmd, rbuf);
	case 0xb2:
		return ata_scsiop_inq_b2(dev, cmd, rbuf);
	case 0xb6:
		return ata_scsiop_inq_b6(dev, cmd, rbuf);
	case 0xb9:
		return ata_scsiop_inq_b9(dev, cmd, rbuf);
	default:
		ata_scsi_set_invalid_field(dev, cmd, 2, 0xff);
		return 0;
	}
}

/**
 *	modecpy - Prepare response for MODE SENSE
 *	@dest: output buffer
 *	@src: data being copied
 *	@n: length of mode page
 *	@changeable: whether changeable parameters are requested
 *
 *	Generate a generic MODE SENSE page for either current or changeable
 *	parameters.
 *
 *	LOCKING:
 *	None.
 */
static void modecpy(u8 *dest, const u8 *src, int n, bool changeable)
{
	if (changeable) {
		memcpy(dest, src, 2);
		memset(dest + 2, 0, n - 2);
	} else {
		memcpy(dest, src, n);
	}
}

/**
 *	ata_msense_caching - Simulate MODE SENSE caching info page
 *	@id: device IDENTIFY data
 *	@buf: output buffer
 *	@changeable: whether changeable parameters are requested
 *
 *	Generate a caching info page, which conditionally indicates
 *	write caching to the SCSI layer, depending on device
 *	capabilities.
 *
 *	LOCKING:
 *	None.
 */
static unsigned int ata_msense_caching(u16 *id, u8 *buf, bool changeable)
{
	modecpy(buf, def_cache_mpage, sizeof(def_cache_mpage), changeable);
	if (changeable) {
		buf[2] |= (1 << 2);	/* ata_mselect_caching() */
	} else {
		buf[2] |= (ata_id_wcache_enabled(id) << 2);	/* write cache enable */
		buf[12] |= (!ata_id_rahead_enabled(id) << 5);	/* disable read ahead */
	}
	return sizeof(def_cache_mpage);
}

/*
 * Simulate MODE SENSE control mode page, sub-page 0.
 */
static unsigned int ata_msense_control_spg0(struct ata_device *dev, u8 *buf,
					    bool changeable)
{
	modecpy(buf, def_control_mpage,
		sizeof(def_control_mpage), changeable);
	if (changeable) {
		/* ata_mselect_control() */
		buf[2] |= (1 << 2);
	} else {
		bool d_sense = (dev->flags & ATA_DFLAG_D_SENSE);

		/* descriptor format sense data */
		buf[2] |= (d_sense << 2);
	}

	return sizeof(def_control_mpage);
}

/*
 * Translate an ATA duration limit in microseconds to a SCSI duration limit
 * using the t2cdlunits 0xa (10ms). Since the SCSI duration limits are 2-bytes
 * only, take care of overflows.
 */
static inline u16 ata_xlat_cdl_limit(u8 *buf)
{
	u32 limit = get_unaligned_le32(buf);

	return min_t(u32, limit / 10000, 65535);
}

/*
 * Simulate MODE SENSE control mode page, sub-pages 07h and 08h
 * (command duration limits T2A and T2B mode pages).
 */
static unsigned int ata_msense_control_spgt2(struct ata_device *dev, u8 *buf,
					     u8 spg)
{
	u8 *b, *cdl, *desc;
	u32 policy;
	int i;

	if (!(dev->flags & ATA_DFLAG_CDL) || !dev->cdl)
		return 0;

	cdl = dev->cdl->desc_log_buf;

	/*
	 * Fill the subpage. The first four bytes of the T2A/T2B mode pages
	 * are a header. The PAGE LENGTH field is the size of the page
	 * excluding the header.
	 */
	buf[0] = CONTROL_MPAGE;
	buf[1] = spg;
	put_unaligned_be16(CDL_T2_SUB_MPAGE_LEN - 4, &buf[2]);
	if (spg == CDL_T2A_SUB_MPAGE) {
		/*
		 * Read descriptors map to the T2A page:
		 * set perf_vs_duration_guidleine.
		 */
		buf[7] = (cdl[0] & 0x03) << 4;
		desc = cdl + 64;
	} else {
		/* Write descriptors map to the T2B page */
		desc = cdl + 288;
	}

	/* Fill the T2 page descriptors */
	b = &buf[8];
	policy = get_unaligned_le32(&cdl[0]);
	for (i = 0; i < 7; i++, b += 32, desc += 32) {
		/* t2cdlunits: fixed to 10ms */
		b[0] = 0x0a;

		/* Max inactive time and its policy */
		put_unaligned_be16(ata_xlat_cdl_limit(&desc[8]), &b[2]);
		b[6] = ((policy >> 8) & 0x0f) << 4;

		/* Max active time and its policy */
		put_unaligned_be16(ata_xlat_cdl_limit(&desc[4]), &b[4]);
		b[6] |= (policy >> 4) & 0x0f;

		/* Command duration guideline and its policy */
		put_unaligned_be16(ata_xlat_cdl_limit(&desc[16]), &b[10]);
		b[14] = policy & 0x0f;
	}

	return CDL_T2_SUB_MPAGE_LEN;
}

/*
 * Simulate MODE SENSE control mode page, sub-page f2h
 * (ATA feature control mode page).
 */
static unsigned int ata_msense_control_ata_feature(struct ata_device *dev,
						   u8 *buf)
{
	/* PS=0, SPF=1 */
	buf[0] = CONTROL_MPAGE | (1 << 6);
	buf[1] = ATA_FEATURE_SUB_MPAGE;

	/*
	 * The first four bytes of ATA Feature Control mode page are a header.
	 * The PAGE LENGTH field is the size of the page excluding the header.
	 */
	put_unaligned_be16(ATA_FEATURE_SUB_MPAGE_LEN - 4, &buf[2]);

	if (dev->flags & ATA_DFLAG_CDL_ENABLED)
		buf[4] = 0x02; /* T2A and T2B pages enabled */
	else
		buf[4] = 0;

	return ATA_FEATURE_SUB_MPAGE_LEN;
}

/**
 *	ata_msense_control - Simulate MODE SENSE control mode page
 *	@dev: ATA device of interest
 *	@buf: output buffer
 *	@spg: sub-page code
 *	@changeable: whether changeable parameters are requested
 *
 *	Generate a generic MODE SENSE control mode page.
 *
 *	LOCKING:
 *	None.
 */
static unsigned int ata_msense_control(struct ata_device *dev, u8 *buf,
				       u8 spg, bool changeable)
{
	unsigned int n;

	switch (spg) {
	case 0:
		return ata_msense_control_spg0(dev, buf, changeable);
	case CDL_T2A_SUB_MPAGE:
	case CDL_T2B_SUB_MPAGE:
		return ata_msense_control_spgt2(dev, buf, spg);
	case ATA_FEATURE_SUB_MPAGE:
		return ata_msense_control_ata_feature(dev, buf);
	case ALL_SUB_MPAGES:
		n = ata_msense_control_spg0(dev, buf, changeable);
		n += ata_msense_control_spgt2(dev, buf + n, CDL_T2A_SUB_MPAGE);
		n += ata_msense_control_spgt2(dev, buf + n, CDL_T2B_SUB_MPAGE);
		n += ata_msense_control_ata_feature(dev, buf + n);
		return n;
	default:
		return 0;
	}
}

/**
 *	ata_msense_rw_recovery - Simulate MODE SENSE r/w error recovery page
 *	@buf: output buffer
 *	@changeable: whether changeable parameters are requested
 *
 *	Generate a generic MODE SENSE r/w error recovery page.
 *
 *	LOCKING:
 *	None.
 */
static unsigned int ata_msense_rw_recovery(u8 *buf, bool changeable)
{
	modecpy(buf, def_rw_recovery_mpage, sizeof(def_rw_recovery_mpage),
		changeable);
	return sizeof(def_rw_recovery_mpage);
}

/**
 *	ata_scsiop_mode_sense - Simulate MODE SENSE 6, 10 commands
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Simulate MODE SENSE commands. Assume this is invoked for direct
 *	access devices (e.g. disks) only. There should be no block
 *	descriptor for other device types.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_mode_sense(struct ata_device *dev,
					  struct scsi_cmnd *cmd, u8 *rbuf)
{
	u8 *scsicmd = cmd->cmnd, *p = rbuf;
	static const u8 sat_blk_desc[] = {
		0, 0, 0, 0,	/* number of blocks: sat unspecified */
		0,
		0, 0x2, 0x0	/* block length: 512 bytes */
	};
	u8 pg, spg;
	unsigned int ebd, page_control, six_byte;
	u8 dpofua = 0, bp = 0xff;
	u16 fp;

	six_byte = (scsicmd[0] == MODE_SENSE);
	ebd = !(scsicmd[1] & 0x8);      /* dbd bit inverted == edb */
	/*
	 * LLBA bit in msense(10) ignored (compliant)
	 */

	page_control = scsicmd[2] >> 6;
	switch (page_control) {
	case 0: /* current */
	case 1: /* changeable */
	case 2: /* defaults */
		break;  /* supported */
	case 3: /* saved */
		goto saving_not_supp;
	default:
		fp = 2;
		bp = 6;
		goto invalid_fld;
	}

	if (six_byte)
		p += 4 + (ebd ? 8 : 0);
	else
		p += 8 + (ebd ? 8 : 0);

	pg = scsicmd[2] & 0x3f;
	spg = scsicmd[3];

	/*
	 * Supported subpages: all subpages and sub-pages 07h, 08h and f2h of
	 * the control page.
	 */
	if (spg) {
		switch (spg) {
		case ALL_SUB_MPAGES:
			break;
		case CDL_T2A_SUB_MPAGE:
		case CDL_T2B_SUB_MPAGE:
		case ATA_FEATURE_SUB_MPAGE:
			if (dev->flags & ATA_DFLAG_CDL && pg == CONTROL_MPAGE)
				break;
			fallthrough;
		default:
			fp = 3;
			goto invalid_fld;
		}
	}

	switch(pg) {
	case RW_RECOVERY_MPAGE:
		p += ata_msense_rw_recovery(p, page_control == 1);
		break;

	case CACHE_MPAGE:
		p += ata_msense_caching(dev->id, p, page_control == 1);
		break;

	case CONTROL_MPAGE:
		p += ata_msense_control(dev, p, spg, page_control == 1);
		break;

	case ALL_MPAGES:
		p += ata_msense_rw_recovery(p, page_control == 1);
		p += ata_msense_caching(dev->id, p, page_control == 1);
		p += ata_msense_control(dev, p, spg, page_control == 1);
		break;

	default:		/* invalid page code */
		fp = 2;
		goto invalid_fld;
	}

	if (dev->flags & ATA_DFLAG_FUA)
		dpofua = 1 << 4;

	if (six_byte) {
		rbuf[0] = p - rbuf - 1;
		rbuf[2] |= dpofua;
		if (ebd) {
			rbuf[3] = sizeof(sat_blk_desc);
			memcpy(rbuf + 4, sat_blk_desc, sizeof(sat_blk_desc));
		}

		return rbuf[0] + 1;
	}

	put_unaligned_be16(p - rbuf - 2, &rbuf[0]);
	rbuf[3] |= dpofua;
	if (ebd) {
		rbuf[7] = sizeof(sat_blk_desc);
		memcpy(rbuf + 8, sat_blk_desc, sizeof(sat_blk_desc));
	}

	return get_unaligned_be16(&rbuf[0]) + 2;

invalid_fld:
	ata_scsi_set_invalid_field(dev, cmd, fp, bp);
	return 0;

saving_not_supp:
	ata_scsi_set_sense(dev, cmd, ILLEGAL_REQUEST, 0x39, 0x0);
	 /* "Saving parameters not supported" */
	return 0;
}

/**
 *	ata_scsiop_read_cap - Simulate READ CAPACITY[ 16] commands
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Simulate READ CAPACITY commands.
 *
 *	LOCKING:
 *	None.
 */
static unsigned int ata_scsiop_read_cap(struct ata_device *dev,
					struct scsi_cmnd *cmd, u8 *rbuf)
{
	u8 *scsicmd = cmd->cmnd;
	u64 last_lba = dev->n_sectors - 1; /* LBA of the last block */
	u32 sector_size; /* physical sector size in bytes */
	u8 log2_per_phys;
	u16 lowest_aligned;

	sector_size = ata_id_logical_sector_size(dev->id);
	log2_per_phys = ata_id_log2_per_physical_sector(dev->id);
	lowest_aligned = ata_id_logical_sector_offset(dev->id, log2_per_phys);

	if (scsicmd[0] == READ_CAPACITY) {
		if (last_lba >= 0xffffffffULL)
			last_lba = 0xffffffff;

		/* sector count, 32-bit */
		rbuf[0] = last_lba >> (8 * 3);
		rbuf[1] = last_lba >> (8 * 2);
		rbuf[2] = last_lba >> (8 * 1);
		rbuf[3] = last_lba;

		/* sector size */
		rbuf[4] = sector_size >> (8 * 3);
		rbuf[5] = sector_size >> (8 * 2);
		rbuf[6] = sector_size >> (8 * 1);
		rbuf[7] = sector_size;

		return 8;
	}

	/*
	 * READ CAPACITY 16 command is defined as a service action
	 * (SERVICE_ACTION_IN_16 command).
	 */
	if (scsicmd[0] != SERVICE_ACTION_IN_16 ||
	    (scsicmd[1] & 0x1f) != SAI_READ_CAPACITY_16) {
		ata_scsi_set_invalid_field(dev, cmd, 1, 0xff);
		return 0;
	}

	/* sector count, 64-bit */
	rbuf[0] = last_lba >> (8 * 7);
	rbuf[1] = last_lba >> (8 * 6);
	rbuf[2] = last_lba >> (8 * 5);
	rbuf[3] = last_lba >> (8 * 4);
	rbuf[4] = last_lba >> (8 * 3);
	rbuf[5] = last_lba >> (8 * 2);
	rbuf[6] = last_lba >> (8 * 1);
	rbuf[7] = last_lba;

	/* sector size */
	rbuf[ 8] = sector_size >> (8 * 3);
	rbuf[ 9] = sector_size >> (8 * 2);
	rbuf[10] = sector_size >> (8 * 1);
	rbuf[11] = sector_size;

	if (ata_id_zoned_cap(dev->id) || dev->class == ATA_DEV_ZAC)
		rbuf[12] = (1 << 4); /* RC_BASIS */
	rbuf[13] = log2_per_phys;
	rbuf[14] = (lowest_aligned >> 8) & 0x3f;
	rbuf[15] = lowest_aligned;

	if (ata_id_has_trim(dev->id) && !(dev->quirks & ATA_QUIRK_NOTRIM)) {
		rbuf[14] |= 0x80; /* LBPME */

		if (ata_id_has_zero_after_trim(dev->id) &&
		    dev->quirks & ATA_QUIRK_ZERO_AFTER_TRIM) {
			ata_dev_info(dev, "Enabling discard_zeroes_data\n");
			rbuf[14] |= 0x40; /* LBPRZ */
		}
	}

	return 16;
}

/**
 *	ata_scsiop_report_luns - Simulate REPORT LUNS command
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Simulate REPORT LUNS command.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_report_luns(struct ata_device *dev,
					   struct scsi_cmnd *cmd, u8 *rbuf)
{
	rbuf[3] = 8;	/* just one lun, LUN 0, size 8 bytes */

	return 16;
}

/*
 * ATAPI devices typically report zero for their SCSI version, and sometimes
 * deviate from the spec WRT response data format.  If SCSI version is
 * reported as zero like normal, then we make the following fixups:
 *   1) Fake MMC-5 version, to indicate to the Linux scsi midlayer this is a
 *	modern device.
 *   2) Ensure response data format / ATAPI information are always correct.
 */
static void atapi_fixup_inquiry(struct scsi_cmnd *cmd)
{
	u8 buf[4];

	sg_copy_to_buffer(scsi_sglist(cmd), scsi_sg_count(cmd), buf, 4);
	if (buf[2] == 0) {
		buf[2] = 0x5;
		buf[3] = 0x32;
	}
	sg_copy_from_buffer(scsi_sglist(cmd), scsi_sg_count(cmd), buf, 4);
}

static void atapi_qc_complete(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *cmd = qc->scsicmd;
	unsigned int err_mask = qc->err_mask;

	/* handle completion from EH */
	if (unlikely(err_mask || qc->flags & ATA_QCFLAG_SENSE_VALID)) {

		if (!(qc->flags & ATA_QCFLAG_SENSE_VALID))
			ata_gen_passthru_sense(qc);

		/* SCSI EH automatically locks door if sdev->locked is
		 * set.  Sometimes door lock request continues to
		 * fail, for example, when no media is present.  This
		 * creates a loop - SCSI EH issues door lock which
		 * fails and gets invoked again to acquire sense data
		 * for the failed command.
		 *
		 * If door lock fails, always clear sdev->locked to
		 * avoid this infinite loop.
		 *
		 * This may happen before SCSI scan is complete.  Make
		 * sure qc->dev->sdev isn't NULL before dereferencing.
		 */
		if (qc->cdb[0] == ALLOW_MEDIUM_REMOVAL && qc->dev->sdev)
			qc->dev->sdev->locked = 0;

		qc->scsicmd->result = SAM_STAT_CHECK_CONDITION;
		ata_qc_done(qc);
		return;
	}

	/* successful completion path */
	if (cmd->cmnd[0] == INQUIRY && (cmd->cmnd[1] & 0x03) == 0)
		atapi_fixup_inquiry(cmd);
	cmd->result = SAM_STAT_GOOD;

	ata_qc_done(qc);
}
/**
 *	atapi_xlat - Initialize PACKET taskfile
 *	@qc: command structure to be initialized
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Zero on success, non-zero on failure.
 */
static unsigned int atapi_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	struct ata_device *dev = qc->dev;
	int nodata = (scmd->sc_data_direction == DMA_NONE);
	int using_pio = !nodata && (dev->flags & ATA_DFLAG_PIO);
	unsigned int nbytes;

	memset(qc->cdb, 0, dev->cdb_len);
	memcpy(qc->cdb, scmd->cmnd, scmd->cmd_len);

	qc->complete_fn = atapi_qc_complete;

	qc->tf.flags |= ATA_TFLAG_ISADDR | ATA_TFLAG_DEVICE;
	if (scmd->sc_data_direction == DMA_TO_DEVICE) {
		qc->tf.flags |= ATA_TFLAG_WRITE;
	}

	qc->tf.command = ATA_CMD_PACKET;
	ata_qc_set_pc_nbytes(qc);

	/* check whether ATAPI DMA is safe */
	if (!nodata && !using_pio && atapi_check_dma(qc))
		using_pio = 1;

	/* Some controller variants snoop this value for Packet
	 * transfers to do state machine and FIFO management.  Thus we
	 * want to set it properly, and for DMA where it is
	 * effectively meaningless.
	 */
	nbytes = min(ata_qc_raw_nbytes(qc), (unsigned int)63 * 1024);

	/* Most ATAPI devices which honor transfer chunk size don't
	 * behave according to the spec when odd chunk size which
	 * matches the transfer length is specified.  If the number of
	 * bytes to transfer is 2n+1.  According to the spec, what
	 * should happen is to indicate that 2n+1 is going to be
	 * transferred and transfer 2n+2 bytes where the last byte is
	 * padding.
	 *
	 * In practice, this doesn't happen.  ATAPI devices first
	 * indicate and transfer 2n bytes and then indicate and
	 * transfer 2 bytes where the last byte is padding.
	 *
	 * This inconsistency confuses several controllers which
	 * perform PIO using DMA such as Intel AHCIs and sil3124/32.
	 * These controllers use actual number of transferred bytes to
	 * update DMA pointer and transfer of 4n+2 bytes make those
	 * controller push DMA pointer by 4n+4 bytes because SATA data
	 * FISes are aligned to 4 bytes.  This causes data corruption
	 * and buffer overrun.
	 *
	 * Always setting nbytes to even number solves this problem
	 * because then ATAPI devices don't have to split data at 2n
	 * boundaries.
	 */
	if (nbytes & 0x1)
		nbytes++;

	qc->tf.lbam = (nbytes & 0xFF);
	qc->tf.lbah = (nbytes >> 8);

	if (nodata)
		qc->tf.protocol = ATAPI_PROT_NODATA;
	else if (using_pio)
		qc->tf.protocol = ATAPI_PROT_PIO;
	else {
		/* DMA data xfer */
		qc->tf.protocol = ATAPI_PROT_DMA;
		qc->tf.feature |= ATAPI_PKT_DMA;

		if ((dev->flags & ATA_DFLAG_DMADIR) &&
		    (scmd->sc_data_direction != DMA_TO_DEVICE))
			/* some SATA bridges need us to indicate data xfer direction */
			qc->tf.feature |= ATAPI_DMADIR;
	}


	/* FIXME: We need to translate 0x05 READ_BLOCK_LIMITS to a MODE_SENSE
	   as ATAPI tape drives don't get this right otherwise */
	return 0;
}

static struct ata_device *ata_find_dev(struct ata_port *ap, unsigned int devno)
{
	/*
	 * For the non-PMP case, ata_link_max_devices() returns 1 (SATA case),
	 * or 2 (IDE master + slave case). However, the former case includes
	 * libsas hosted devices which are numbered per scsi host, leading
	 * to devno potentially being larger than 0 but with each struct
	 * ata_device having its own struct ata_port and struct ata_link.
	 * To accommodate these, ignore devno and always use device number 0.
	 */
	if (likely(!sata_pmp_attached(ap))) {
		int link_max_devices = ata_link_max_devices(&ap->link);

		if (link_max_devices == 1)
			return &ap->link.device[0];

		if (devno < link_max_devices)
			return &ap->link.device[devno];

		return NULL;
	}

	/*
	 * For PMP-attached devices, the device number corresponds to C
	 * (channel) of SCSI [H:C:I:L], indicating the port pmp link
	 * for the device.
	 */
	if (devno < ap->nr_pmp_links)
		return &ap->pmp_link[devno].device[0];

	return NULL;
}

static struct ata_device *__ata_scsi_find_dev(struct ata_port *ap,
					      const struct scsi_device *scsidev)
{
	int devno;

	/* skip commands not addressed to targets we simulate */
	if (!sata_pmp_attached(ap)) {
		if (unlikely(scsidev->channel || scsidev->lun))
			return NULL;
		devno = scsidev->id;
	} else {
		if (unlikely(scsidev->id || scsidev->lun))
			return NULL;
		devno = scsidev->channel;
	}

	return ata_find_dev(ap, devno);
}

/**
 *	ata_scsi_find_dev - lookup ata_device from scsi_cmnd
 *	@ap: ATA port to which the device is attached
 *	@scsidev: SCSI device from which we derive the ATA device
 *
 *	Given various information provided in struct scsi_cmnd,
 *	map that onto an ATA bus, and using that mapping
 *	determine which ata_device is associated with the
 *	SCSI command to be sent.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	Associated ATA device, or %NULL if not found.
 */
struct ata_device *
ata_scsi_find_dev(struct ata_port *ap, const struct scsi_device *scsidev)
{
	struct ata_device *dev = __ata_scsi_find_dev(ap, scsidev);

	if (unlikely(!dev || !ata_dev_enabled(dev)))
		return NULL;

	return dev;
}

/*
 *	ata_scsi_map_proto - Map pass-thru protocol value to taskfile value.
 *	@byte1: Byte 1 from pass-thru CDB.
 *
 *	RETURNS:
 *	ATA_PROT_UNKNOWN if mapping failed/unimplemented, protocol otherwise.
 */
static u8
ata_scsi_map_proto(u8 byte1)
{
	switch((byte1 & 0x1e) >> 1) {
	case 3:		/* Non-data */
		return ATA_PROT_NODATA;

	case 6:		/* DMA */
	case 10:	/* UDMA Data-in */
	case 11:	/* UDMA Data-Out */
		return ATA_PROT_DMA;

	case 4:		/* PIO Data-in */
	case 5:		/* PIO Data-out */
		return ATA_PROT_PIO;

	case 12:	/* FPDMA */
		return ATA_PROT_NCQ;

	case 0:		/* Hard Reset */
	case 1:		/* SRST */
	case 8:		/* Device Diagnostic */
	case 9:		/* Device Reset */
	case 7:		/* DMA Queued */
	case 15:	/* Return Response Info */
	default:	/* Reserved */
		break;
	}

	return ATA_PROT_UNKNOWN;
}

/**
 *	ata_scsi_pass_thru - convert ATA pass-thru CDB to taskfile
 *	@qc: command structure to be initialized
 *
 *	Handles either 12, 16, or 32-byte versions of the CDB.
 *
 *	RETURNS:
 *	Zero on success, non-zero on failure.
 */
static unsigned int ata_scsi_pass_thru(struct ata_queued_cmd *qc)
{
	struct ata_taskfile *tf = &(qc->tf);
	struct scsi_cmnd *scmd = qc->scsicmd;
	struct ata_device *dev = qc->dev;
	const u8 *cdb = scmd->cmnd;
	u16 fp;
	u16 cdb_offset = 0;

	/* 7Fh variable length cmd means a ata pass-thru(32) */
	if (cdb[0] == VARIABLE_LENGTH_CMD)
		cdb_offset = 9;

	tf->protocol = ata_scsi_map_proto(cdb[1 + cdb_offset]);
	if (tf->protocol == ATA_PROT_UNKNOWN) {
		fp = 1;
		goto invalid_fld;
	}

	if ((cdb[2 + cdb_offset] & 0x3) == 0) {
		/*
		 * When T_LENGTH is zero (No data is transferred), dir should
		 * be DMA_NONE.
		 */
		if (scmd->sc_data_direction != DMA_NONE) {
			fp = 2 + cdb_offset;
			goto invalid_fld;
		}

		if (ata_is_ncq(tf->protocol))
			tf->protocol = ATA_PROT_NCQ_NODATA;
	}

	/* enable LBA */
	tf->flags |= ATA_TFLAG_LBA;

	/*
	 * 12 and 16 byte CDBs use different offsets to
	 * provide the various register values.
	 */
	switch (cdb[0]) {
	case ATA_16:
		/*
		 * 16-byte CDB - may contain extended commands.
		 *
		 * If that is the case, copy the upper byte register values.
		 */
		if (cdb[1] & 0x01) {
			tf->hob_feature = cdb[3];
			tf->hob_nsect = cdb[5];
			tf->hob_lbal = cdb[7];
			tf->hob_lbam = cdb[9];
			tf->hob_lbah = cdb[11];
			tf->flags |= ATA_TFLAG_LBA48;
		} else
			tf->flags &= ~ATA_TFLAG_LBA48;

		/*
		 * Always copy low byte, device and command registers.
		 */
		tf->feature = cdb[4];
		tf->nsect = cdb[6];
		tf->lbal = cdb[8];
		tf->lbam = cdb[10];
		tf->lbah = cdb[12];
		tf->device = cdb[13];
		tf->command = cdb[14];
		break;
	case ATA_12:
		/*
		 * 12-byte CDB - incapable of extended commands.
		 */
		tf->flags &= ~ATA_TFLAG_LBA48;

		tf->feature = cdb[3];
		tf->nsect = cdb[4];
		tf->lbal = cdb[5];
		tf->lbam = cdb[6];
		tf->lbah = cdb[7];
		tf->device = cdb[8];
		tf->command = cdb[9];
		break;
	default:
		/*
		 * 32-byte CDB - may contain extended command fields.
		 *
		 * If that is the case, copy the upper byte register values.
		 */
		if (cdb[10] & 0x01) {
			tf->hob_feature = cdb[20];
			tf->hob_nsect = cdb[22];
			tf->hob_lbal = cdb[16];
			tf->hob_lbam = cdb[15];
			tf->hob_lbah = cdb[14];
			tf->flags |= ATA_TFLAG_LBA48;
		} else
			tf->flags &= ~ATA_TFLAG_LBA48;

		tf->feature = cdb[21];
		tf->nsect = cdb[23];
		tf->lbal = cdb[19];
		tf->lbam = cdb[18];
		tf->lbah = cdb[17];
		tf->device = cdb[24];
		tf->command = cdb[25];
		tf->auxiliary = get_unaligned_be32(&cdb[28]);
		break;
	}

	/* For NCQ commands copy the tag value */
	if (ata_is_ncq(tf->protocol))
		tf->nsect = qc->hw_tag << 3;

	/* enforce correct master/slave bit */
	tf->device = dev->devno ?
		tf->device | ATA_DEV1 : tf->device & ~ATA_DEV1;

	switch (tf->command) {
	/* READ/WRITE LONG use a non-standard sect_size */
	case ATA_CMD_READ_LONG:
	case ATA_CMD_READ_LONG_ONCE:
	case ATA_CMD_WRITE_LONG:
	case ATA_CMD_WRITE_LONG_ONCE:
		if (tf->protocol != ATA_PROT_PIO || tf->nsect != 1) {
			fp = 1;
			goto invalid_fld;
		}
		qc->sect_size = scsi_bufflen(scmd);
		break;

	/* commands using reported Logical Block size (e.g. 512 or 4K) */
	case ATA_CMD_CFA_WRITE_NE:
	case ATA_CMD_CFA_TRANS_SECT:
	case ATA_CMD_CFA_WRITE_MULT_NE:
	/* XXX: case ATA_CMD_CFA_WRITE_SECTORS_WITHOUT_ERASE: */
	case ATA_CMD_READ:
	case ATA_CMD_READ_EXT:
	case ATA_CMD_READ_QUEUED:
	/* XXX: case ATA_CMD_READ_QUEUED_EXT: */
	case ATA_CMD_FPDMA_READ:
	case ATA_CMD_READ_MULTI:
	case ATA_CMD_READ_MULTI_EXT:
	case ATA_CMD_PIO_READ:
	case ATA_CMD_PIO_READ_EXT:
	case ATA_CMD_READ_STREAM_DMA_EXT:
	case ATA_CMD_READ_STREAM_EXT:
	case ATA_CMD_VERIFY:
	case ATA_CMD_VERIFY_EXT:
	case ATA_CMD_WRITE:
	case ATA_CMD_WRITE_EXT:
	case ATA_CMD_WRITE_FUA_EXT:
	case ATA_CMD_WRITE_QUEUED:
	case ATA_CMD_WRITE_QUEUED_FUA_EXT:
	case ATA_CMD_FPDMA_WRITE:
	case ATA_CMD_WRITE_MULTI:
	case ATA_CMD_WRITE_MULTI_EXT:
	case ATA_CMD_WRITE_MULTI_FUA_EXT:
	case ATA_CMD_PIO_WRITE:
	case ATA_CMD_PIO_WRITE_EXT:
	case ATA_CMD_WRITE_STREAM_DMA_EXT:
	case ATA_CMD_WRITE_STREAM_EXT:
		qc->sect_size = scmd->device->sector_size;
		break;

	/* Everything else uses 512 byte "sectors" */
	default:
		qc->sect_size = ATA_SECT_SIZE;
	}

	/*
	 * Set flags so that all registers will be written, pass on
	 * write indication (used for PIO/DMA setup), result TF is
	 * copied back and we don't whine too much about its failure.
	 */
	tf->flags |= ATA_TFLAG_ISADDR | ATA_TFLAG_DEVICE;
	if (scmd->sc_data_direction == DMA_TO_DEVICE)
		tf->flags |= ATA_TFLAG_WRITE;

	qc->flags |= ATA_QCFLAG_RESULT_TF | ATA_QCFLAG_QUIET;

	/*
	 * Set transfer length.
	 *
	 * TODO: find out if we need to do more here to
	 *       cover scatter/gather case.
	 */
	ata_qc_set_pc_nbytes(qc);

	/* We may not issue DMA commands if no DMA mode is set */
	if (tf->protocol == ATA_PROT_DMA && !ata_dma_enabled(dev)) {
		fp = 1;
		goto invalid_fld;
	}

	/* We may not issue NCQ commands to devices not supporting NCQ */
	if (ata_is_ncq(tf->protocol) && !ata_ncq_enabled(dev)) {
		fp = 1;
		goto invalid_fld;
	}

	/* sanity check for pio multi commands */
	if ((cdb[1] & 0xe0) && !is_multi_taskfile(tf)) {
		fp = 1;
		goto invalid_fld;
	}

	if (is_multi_taskfile(tf)) {
		unsigned int multi_count = 1 << (cdb[1] >> 5);

		/* compare the passed through multi_count
		 * with the cached multi_count of libata
		 */
		if (multi_count != dev->multi_count)
			ata_dev_warn(dev, "invalid multi_count %u ignored\n",
				     multi_count);
	}

	/*
	 * Filter SET_FEATURES - XFER MODE command -- otherwise,
	 * SET_FEATURES - XFER MODE must be preceded/succeeded
	 * by an update to hardware-specific registers for each
	 * controller (i.e. the reason for ->set_piomode(),
	 * ->set_dmamode(), and ->post_set_mode() hooks).
	 */
	if (tf->command == ATA_CMD_SET_FEATURES &&
	    tf->feature == SETFEATURES_XFER) {
		fp = (cdb[0] == ATA_16) ? 4 : 3;
		goto invalid_fld;
	}

	/*
	 * Filter TPM commands by default. These provide an
	 * essentially uncontrolled encrypted "back door" between
	 * applications and the disk. Set libata.allow_tpm=1 if you
	 * have a real reason for wanting to use them. This ensures
	 * that installed software cannot easily mess stuff up without
	 * user intent. DVR type users will probably ship with this enabled
	 * for movie content management.
	 *
	 * Note that for ATA8 we can issue a DCS change and DCS freeze lock
	 * for this and should do in future but that it is not sufficient as
	 * DCS is an optional feature set. Thus we also do the software filter
	 * so that we comply with the TC consortium stated goal that the user
	 * can turn off TC features of their system.
	 */
	if (tf->command >= 0x5C && tf->command <= 0x5F && !libata_allow_tpm) {
		fp = (cdb[0] == ATA_16) ? 14 : 9;
		goto invalid_fld;
	}

	return 0;

 invalid_fld:
	ata_scsi_set_invalid_field(dev, scmd, fp, 0xff);
	return 1;
}

/**
 * ata_format_dsm_trim_descr() - SATL Write Same to DSM Trim
 * @cmd: SCSI command being translated
 * @trmax: Maximum number of entries that will fit in sector_size bytes.
 * @sector: Starting sector
 * @count: Total Range of request in logical sectors
 *
 * Rewrite the WRITE SAME descriptor to be a DSM TRIM little-endian formatted
 * descriptor.
 *
 * Upto 64 entries of the format:
 *   63:48 Range Length
 *   47:0  LBA
 *
 *  Range Length of 0 is ignored.
 *  LBA's should be sorted order and not overlap.
 *
 * NOTE: this is the same format as ADD LBA(S) TO NV CACHE PINNED SET
 *
 * Return: Number of bytes copied into sglist.
 */
static size_t ata_format_dsm_trim_descr(struct scsi_cmnd *cmd, u32 trmax,
					u64 sector, u32 count)
{
	struct scsi_device *sdp = cmd->device;
	size_t len = sdp->sector_size;
	size_t r;
	__le64 *buf;
	u32 i = 0;
	unsigned long flags;

	WARN_ON(len > ATA_SCSI_RBUF_SIZE);

	if (len > ATA_SCSI_RBUF_SIZE)
		len = ATA_SCSI_RBUF_SIZE;

	spin_lock_irqsave(&ata_scsi_rbuf_lock, flags);
	buf = ((void *)ata_scsi_rbuf);
	memset(buf, 0, len);
	while (i < trmax) {
		u64 entry = sector |
			((u64)(count > 0xffff ? 0xffff : count) << 48);
		buf[i++] = __cpu_to_le64(entry);
		if (count <= 0xffff)
			break;
		count -= 0xffff;
		sector += 0xffff;
	}
	r = sg_copy_from_buffer(scsi_sglist(cmd), scsi_sg_count(cmd), buf, len);
	spin_unlock_irqrestore(&ata_scsi_rbuf_lock, flags);

	return r;
}

/**
 * ata_scsi_write_same_xlat() - SATL Write Same to ATA SCT Write Same
 * @qc: Command to be translated
 *
 * Translate a SCSI WRITE SAME command to be either a DSM TRIM command or
 * an SCT Write Same command.
 * Based on WRITE SAME has the UNMAP flag:
 *
 *   - When set translate to DSM TRIM
 *   - When clear translate to SCT Write Same
 */
static unsigned int ata_scsi_write_same_xlat(struct ata_queued_cmd *qc)
{
	struct ata_taskfile *tf = &qc->tf;
	struct scsi_cmnd *scmd = qc->scsicmd;
	struct scsi_device *sdp = scmd->device;
	size_t len = sdp->sector_size;
	struct ata_device *dev = qc->dev;
	const u8 *cdb = scmd->cmnd;
	u64 block;
	u32 n_block;
	const u32 trmax = len >> 3;
	u32 size;
	u16 fp;
	u8 bp = 0xff;
	u8 unmap = cdb[1] & 0x8;

	/* we may not issue DMA commands if no DMA mode is set */
	if (unlikely(!ata_dma_enabled(dev)))
		goto invalid_opcode;

	/*
	 * We only allow sending this command through the block layer,
	 * as it modifies the DATA OUT buffer, which would corrupt user
	 * memory for SG_IO commands.
	 */
	if (unlikely(blk_rq_is_passthrough(scsi_cmd_to_rq(scmd))))
		goto invalid_opcode;

	if (unlikely(scmd->cmd_len < 16)) {
		fp = 15;
		goto invalid_fld;
	}
	scsi_16_lba_len(cdb, &block, &n_block);

	if (!unmap || (dev->quirks & ATA_QUIRK_NOTRIM) ||
	    !ata_id_has_trim(dev->id)) {
		fp = 1;
		bp = 3;
		goto invalid_fld;
	}
	/* If the request is too large the cmd is invalid */
	if (n_block > 0xffff * trmax) {
		fp = 2;
		goto invalid_fld;
	}

	/*
	 * WRITE SAME always has a sector sized buffer as payload, this
	 * should never be a multiple entry S/G list.
	 */
	if (!scsi_sg_count(scmd))
		goto invalid_param_len;

	/*
	 * size must match sector size in bytes
	 * For DATA SET MANAGEMENT TRIM in ACS-2 nsect (aka count)
	 * is defined as number of 512 byte blocks to be transferred.
	 */

	size = ata_format_dsm_trim_descr(scmd, trmax, block, n_block);
	if (size != len)
		goto invalid_param_len;

	if (ata_ncq_enabled(dev) && ata_fpdma_dsm_supported(dev)) {
		/* Newer devices support queued TRIM commands */
		tf->protocol = ATA_PROT_NCQ;
		tf->command = ATA_CMD_FPDMA_SEND;
		tf->hob_nsect = ATA_SUBCMD_FPDMA_SEND_DSM & 0x1f;
		tf->nsect = qc->hw_tag << 3;
		tf->hob_feature = (size / 512) >> 8;
		tf->feature = size / 512;

		tf->auxiliary = 1;
	} else {
		tf->protocol = ATA_PROT_DMA;
		tf->hob_feature = 0;
		tf->feature = ATA_DSM_TRIM;
		tf->hob_nsect = (size / 512) >> 8;
		tf->nsect = size / 512;
		tf->command = ATA_CMD_DSM;
	}

	tf->flags |= ATA_TFLAG_ISADDR | ATA_TFLAG_DEVICE | ATA_TFLAG_LBA48 |
		     ATA_TFLAG_WRITE;

	ata_qc_set_pc_nbytes(qc);

	return 0;

invalid_fld:
	ata_scsi_set_invalid_field(dev, scmd, fp, bp);
	return 1;
invalid_param_len:
	/* "Parameter list length error" */
	ata_scsi_set_sense(dev, scmd, ILLEGAL_REQUEST, 0x1a, 0x0);
	return 1;
invalid_opcode:
	/* "Invalid command operation code" */
	ata_scsi_set_sense(dev, scmd, ILLEGAL_REQUEST, 0x20, 0x0);
	return 1;
}

/**
 *	ata_scsiop_maint_in - Simulate a subset of MAINTENANCE_IN
 *	@dev: Target device.
 *	@cmd: SCSI command of interest.
 *	@rbuf: Response buffer, to which simulated SCSI cmd output is sent.
 *
 *	Yields a subset to satisfy scsi_report_opcode()
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsiop_maint_in(struct ata_device *dev,
					struct scsi_cmnd *cmd, u8 *rbuf)
{
	u8 *cdb = cmd->cmnd;
	u8 supported = 0, cdlp = 0, rwcdlp = 0;

	if ((cdb[1] & 0x1f) != MI_REPORT_SUPPORTED_OPERATION_CODES) {
		ata_scsi_set_invalid_field(dev, cmd, 1, 0xff);
		return 0;
	}

	if (cdb[2] != 1 && cdb[2] != 3) {
		ata_dev_warn(dev, "invalid command format %d\n", cdb[2]);
		ata_scsi_set_invalid_field(dev, cmd, 1, 0xff);
		return 0;
	}

	switch (cdb[3]) {
	case INQUIRY:
	case MODE_SENSE:
	case MODE_SENSE_10:
	case READ_CAPACITY:
	case SERVICE_ACTION_IN_16:
	case REPORT_LUNS:
	case REQUEST_SENSE:
	case SYNCHRONIZE_CACHE:
	case SYNCHRONIZE_CACHE_16:
	case REZERO_UNIT:
	case SEEK_6:
	case SEEK_10:
	case TEST_UNIT_READY:
	case SEND_DIAGNOSTIC:
	case MAINTENANCE_IN:
	case READ_6:
	case READ_10:
	case WRITE_6:
	case WRITE_10:
	case ATA_12:
	case ATA_16:
	case VERIFY:
	case VERIFY_16:
	case MODE_SELECT:
	case MODE_SELECT_10:
	case START_STOP:
		supported = 3;
		break;
	case READ_16:
		supported = 3;
		if (dev->flags & ATA_DFLAG_CDL) {
			/*
			 * CDL read descriptors map to the T2A page, that is,
			 * rwcdlp = 0x01 and cdlp = 0x01
			 */
			rwcdlp = 0x01;
			cdlp = 0x01 << 3;
		}
		break;
	case WRITE_16:
		supported = 3;
		if (dev->flags & ATA_DFLAG_CDL) {
			/*
			 * CDL write descriptors map to the T2B page, that is,
			 * rwcdlp = 0x01 and cdlp = 0x02
			 */
			rwcdlp = 0x01;
			cdlp = 0x02 << 3;
		}
		break;
	case ZBC_IN:
	case ZBC_OUT:
		if (ata_id_zoned_cap(dev->id) ||
		    dev->class == ATA_DEV_ZAC)
			supported = 3;
		break;
	case SECURITY_PROTOCOL_IN:
	case SECURITY_PROTOCOL_OUT:
		if (dev->flags & ATA_DFLAG_TRUSTED)
			supported = 3;
		break;
	default:
		break;
	}

	/* One command format */
	rbuf[0] = rwcdlp;
	rbuf[1] = cdlp | supported;

	return 4;
}

/**
 *	ata_scsi_report_zones_complete - convert ATA output
 *	@qc: command structure returning the data
 *
 *	Convert T-13 little-endian field representation into
 *	T-10 big-endian field representation.
 *	What a mess.
 */
static void ata_scsi_report_zones_complete(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	struct sg_mapping_iter miter;
	unsigned long flags;
	unsigned int bytes = 0;

	sg_miter_start(&miter, scsi_sglist(scmd), scsi_sg_count(scmd),
		       SG_MITER_TO_SG | SG_MITER_ATOMIC);

	local_irq_save(flags);
	while (sg_miter_next(&miter)) {
		unsigned int offset = 0;

		if (bytes == 0) {
			char *hdr;
			u32 list_length;
			u64 max_lba, opt_lba;
			u16 same;

			/* Swizzle header */
			hdr = miter.addr;
			list_length = get_unaligned_le32(&hdr[0]);
			same = get_unaligned_le16(&hdr[4]);
			max_lba = get_unaligned_le64(&hdr[8]);
			opt_lba = get_unaligned_le64(&hdr[16]);
			put_unaligned_be32(list_length, &hdr[0]);
			hdr[4] = same & 0xf;
			put_unaligned_be64(max_lba, &hdr[8]);
			put_unaligned_be64(opt_lba, &hdr[16]);
			offset += 64;
			bytes += 64;
		}
		while (offset < miter.length) {
			char *rec;
			u8 cond, type, non_seq, reset;
			u64 size, start, wp;

			/* Swizzle zone descriptor */
			rec = miter.addr + offset;
			type = rec[0] & 0xf;
			cond = (rec[1] >> 4) & 0xf;
			non_seq = (rec[1] & 2);
			reset = (rec[1] & 1);
			size = get_unaligned_le64(&rec[8]);
			start = get_unaligned_le64(&rec[16]);
			wp = get_unaligned_le64(&rec[24]);
			rec[0] = type;
			rec[1] = (cond << 4) | non_seq | reset;
			put_unaligned_be64(size, &rec[8]);
			put_unaligned_be64(start, &rec[16]);
			put_unaligned_be64(wp, &rec[24]);
			WARN_ON(offset + 64 > miter.length);
			offset += 64;
			bytes += 64;
		}
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);

	ata_scsi_qc_complete(qc);
}

static unsigned int ata_scsi_zbc_in_xlat(struct ata_queued_cmd *qc)
{
	struct ata_taskfile *tf = &qc->tf;
	struct scsi_cmnd *scmd = qc->scsicmd;
	const u8 *cdb = scmd->cmnd;
	u16 sect, fp = (u16)-1;
	u8 sa, options, bp = 0xff;
	u64 block;
	u32 n_block;

	if (unlikely(scmd->cmd_len < 16)) {
		ata_dev_warn(qc->dev, "invalid cdb length %d\n",
			     scmd->cmd_len);
		fp = 15;
		goto invalid_fld;
	}
	scsi_16_lba_len(cdb, &block, &n_block);
	if (n_block != scsi_bufflen(scmd)) {
		ata_dev_warn(qc->dev, "non-matching transfer count (%d/%d)\n",
			     n_block, scsi_bufflen(scmd));
		goto invalid_param_len;
	}
	sa = cdb[1] & 0x1f;
	if (sa != ZI_REPORT_ZONES) {
		ata_dev_warn(qc->dev, "invalid service action %d\n", sa);
		fp = 1;
		goto invalid_fld;
	}
	/*
	 * ZAC allows only for transfers in 512 byte blocks,
	 * and uses a 16 bit value for the transfer count.
	 */
	if ((n_block / 512) > 0xffff || n_block < 512 || (n_block % 512)) {
		ata_dev_warn(qc->dev, "invalid transfer count %d\n", n_block);
		goto invalid_param_len;
	}
	sect = n_block / 512;
	options = cdb[14] & 0xbf;

	if (ata_ncq_enabled(qc->dev) &&
	    ata_fpdma_zac_mgmt_in_supported(qc->dev)) {
		tf->protocol = ATA_PROT_NCQ;
		tf->command = ATA_CMD_FPDMA_RECV;
		tf->hob_nsect = ATA_SUBCMD_FPDMA_RECV_ZAC_MGMT_IN & 0x1f;
		tf->nsect = qc->hw_tag << 3;
		tf->feature = sect & 0xff;
		tf->hob_feature = (sect >> 8) & 0xff;
		tf->auxiliary = ATA_SUBCMD_ZAC_MGMT_IN_REPORT_ZONES | (options << 8);
	} else {
		tf->command = ATA_CMD_ZAC_MGMT_IN;
		tf->feature = ATA_SUBCMD_ZAC_MGMT_IN_REPORT_ZONES;
		tf->protocol = ATA_PROT_DMA;
		tf->hob_feature = options;
		tf->hob_nsect = (sect >> 8) & 0xff;
		tf->nsect = sect & 0xff;
	}
	tf->device = ATA_LBA;
	tf->lbah = (block >> 16) & 0xff;
	tf->lbam = (block >> 8) & 0xff;
	tf->lbal = block & 0xff;
	tf->hob_lbah = (block >> 40) & 0xff;
	tf->hob_lbam = (block >> 32) & 0xff;
	tf->hob_lbal = (block >> 24) & 0xff;

	tf->flags |= ATA_TFLAG_ISADDR | ATA_TFLAG_DEVICE | ATA_TFLAG_LBA48;
	qc->flags |= ATA_QCFLAG_RESULT_TF;

	ata_qc_set_pc_nbytes(qc);

	qc->complete_fn = ata_scsi_report_zones_complete;

	return 0;

invalid_fld:
	ata_scsi_set_invalid_field(qc->dev, scmd, fp, bp);
	return 1;

invalid_param_len:
	/* "Parameter list length error" */
	ata_scsi_set_sense(qc->dev, scmd, ILLEGAL_REQUEST, 0x1a, 0x0);
	return 1;
}

static unsigned int ata_scsi_zbc_out_xlat(struct ata_queued_cmd *qc)
{
	struct ata_taskfile *tf = &qc->tf;
	struct scsi_cmnd *scmd = qc->scsicmd;
	struct ata_device *dev = qc->dev;
	const u8 *cdb = scmd->cmnd;
	u8 all, sa;
	u64 block;
	u32 n_block;
	u16 fp = (u16)-1;

	if (unlikely(scmd->cmd_len < 16)) {
		fp = 15;
		goto invalid_fld;
	}

	sa = cdb[1] & 0x1f;
	if ((sa != ZO_CLOSE_ZONE) && (sa != ZO_FINISH_ZONE) &&
	    (sa != ZO_OPEN_ZONE) && (sa != ZO_RESET_WRITE_POINTER)) {
		fp = 1;
		goto invalid_fld;
	}

	scsi_16_lba_len(cdb, &block, &n_block);
	if (n_block) {
		/*
		 * ZAC MANAGEMENT OUT doesn't define any length
		 */
		goto invalid_param_len;
	}

	all = cdb[14] & 0x1;
	if (all) {
		/*
		 * Ignore the block address (zone ID) as defined by ZBC.
		 */
		block = 0;
	} else if (block >= dev->n_sectors) {
		/*
		 * Block must be a valid zone ID (a zone start LBA).
		 */
		fp = 2;
		goto invalid_fld;
	}

	if (ata_ncq_enabled(qc->dev) &&
	    ata_fpdma_zac_mgmt_out_supported(qc->dev)) {
		tf->protocol = ATA_PROT_NCQ_NODATA;
		tf->command = ATA_CMD_NCQ_NON_DATA;
		tf->feature = ATA_SUBCMD_NCQ_NON_DATA_ZAC_MGMT_OUT;
		tf->nsect = qc->hw_tag << 3;
		tf->auxiliary = sa | ((u16)all << 8);
	} else {
		tf->protocol = ATA_PROT_NODATA;
		tf->command = ATA_CMD_ZAC_MGMT_OUT;
		tf->feature = sa;
		tf->hob_feature = all;
	}
	tf->lbah = (block >> 16) & 0xff;
	tf->lbam = (block >> 8) & 0xff;
	tf->lbal = block & 0xff;
	tf->hob_lbah = (block >> 40) & 0xff;
	tf->hob_lbam = (block >> 32) & 0xff;
	tf->hob_lbal = (block >> 24) & 0xff;
	tf->device = ATA_LBA;
	tf->flags |= ATA_TFLAG_ISADDR | ATA_TFLAG_DEVICE | ATA_TFLAG_LBA48;

	return 0;

 invalid_fld:
	ata_scsi_set_invalid_field(qc->dev, scmd, fp, 0xff);
	return 1;
invalid_param_len:
	/* "Parameter list length error" */
	ata_scsi_set_sense(qc->dev, scmd, ILLEGAL_REQUEST, 0x1a, 0x0);
	return 1;
}

/**
 *	ata_mselect_caching - Simulate MODE SELECT for caching info page
 *	@qc: Storage for translated ATA taskfile
 *	@buf: input buffer
 *	@len: number of valid bytes in the input buffer
 *	@fp: out parameter for the failed field on error
 *
 *	Prepare a taskfile to modify caching information for the device.
 *
 *	LOCKING:
 *	None.
 */
static int ata_mselect_caching(struct ata_queued_cmd *qc,
			       const u8 *buf, int len, u16 *fp)
{
	struct ata_taskfile *tf = &qc->tf;
	struct ata_device *dev = qc->dev;
	u8 mpage[CACHE_MPAGE_LEN];
	u8 wce;
	int i;

	/*
	 * The first two bytes of def_cache_mpage are a header, so offsets
	 * in mpage are off by 2 compared to buf.  Same for len.
	 */

	if (len != CACHE_MPAGE_LEN - 2) {
		*fp = min(len, CACHE_MPAGE_LEN - 2);
		return -EINVAL;
	}

	wce = buf[0] & (1 << 2);

	/*
	 * Check that read-only bits are not modified.
	 */
	ata_msense_caching(dev->id, mpage, false);
	for (i = 0; i < CACHE_MPAGE_LEN - 2; i++) {
		if (i == 0)
			continue;
		if (mpage[i + 2] != buf[i]) {
			*fp = i;
			return -EINVAL;
		}
	}

	tf->flags |= ATA_TFLAG_DEVICE | ATA_TFLAG_ISADDR;
	tf->protocol = ATA_PROT_NODATA;
	tf->nsect = 0;
	tf->command = ATA_CMD_SET_FEATURES;
	tf->feature = wce ? SETFEATURES_WC_ON : SETFEATURES_WC_OFF;
	return 0;
}

/*
 * Simulate MODE SELECT control mode page, sub-page 0.
 */
static int ata_mselect_control_spg0(struct ata_queued_cmd *qc,
				    const u8 *buf, int len, u16 *fp)
{
	struct ata_device *dev = qc->dev;
	u8 mpage[CONTROL_MPAGE_LEN];
	u8 d_sense;
	int i;

	/*
	 * The first two bytes of def_control_mpage are a header, so offsets
	 * in mpage are off by 2 compared to buf.  Same for len.
	 */

	if (len != CONTROL_MPAGE_LEN - 2) {
		*fp = min(len, CONTROL_MPAGE_LEN - 2);
		return -EINVAL;
	}

	d_sense = buf[0] & (1 << 2);

	/*
	 * Check that read-only bits are not modified.
	 */
	ata_msense_control_spg0(dev, mpage, false);
	for (i = 0; i < CONTROL_MPAGE_LEN - 2; i++) {
		if (i == 0)
			continue;
		if (mpage[2 + i] != buf[i]) {
			*fp = i;
			return -EINVAL;
		}
	}
	if (d_sense & (1 << 2))
		dev->flags |= ATA_DFLAG_D_SENSE;
	else
		dev->flags &= ~ATA_DFLAG_D_SENSE;
	return 0;
}

/*
 * Translate MODE SELECT control mode page, sub-page f2h (ATA feature mode
 * page) into a SET FEATURES command.
 */
static int ata_mselect_control_ata_feature(struct ata_queued_cmd *qc,
					   const u8 *buf, int len, u16 *fp)
{
	struct ata_device *dev = qc->dev;
	struct ata_taskfile *tf = &qc->tf;
	u8 cdl_action;

	/*
	 * The first four bytes of ATA Feature Control mode page are a header,
	 * so offsets in mpage are off by 4 compared to buf.  Same for len.
	 */
	if (len != ATA_FEATURE_SUB_MPAGE_LEN - 4) {
		*fp = min(len, ATA_FEATURE_SUB_MPAGE_LEN - 4);
		return -EINVAL;
	}

	/* Check cdl_ctrl */
	switch (buf[0] & 0x03) {
	case 0:
		/* Disable CDL if it is enabled */
		if (!(dev->flags & ATA_DFLAG_CDL_ENABLED))
			return 0;
		ata_dev_dbg(dev, "Disabling CDL\n");
		cdl_action = 0;
		dev->flags &= ~ATA_DFLAG_CDL_ENABLED;
		break;
	case 0x02:
		/*
		 * Enable CDL if not already enabled. Since this is mutually
		 * exclusive with NCQ priority, allow this only if NCQ priority
		 * is disabled.
		 */
		if (dev->flags & ATA_DFLAG_CDL_ENABLED)
			return 0;
		if (dev->flags & ATA_DFLAG_NCQ_PRIO_ENABLED) {
			ata_dev_err(dev,
				"NCQ priority must be disabled to enable CDL\n");
			return -EINVAL;
		}
		ata_dev_dbg(dev, "Enabling CDL\n");
		cdl_action = 1;
		dev->flags |= ATA_DFLAG_CDL_ENABLED;
		break;
	default:
		*fp = 0;
		return -EINVAL;
	}

	tf->flags |= ATA_TFLAG_DEVICE | ATA_TFLAG_ISADDR;
	tf->protocol = ATA_PROT_NODATA;
	tf->command = ATA_CMD_SET_FEATURES;
	tf->feature = SETFEATURES_CDL;
	tf->nsect = cdl_action;

	return 1;
}

/**
 *	ata_mselect_control - Simulate MODE SELECT for control page
 *	@qc: Storage for translated ATA taskfile
 *	@spg: target sub-page of the control page
 *	@buf: input buffer
 *	@len: number of valid bytes in the input buffer
 *	@fp: out parameter for the failed field on error
 *
 *	Prepare a taskfile to modify caching information for the device.
 *
 *	LOCKING:
 *	None.
 */
static int ata_mselect_control(struct ata_queued_cmd *qc, u8 spg,
			       const u8 *buf, int len, u16 *fp)
{
	switch (spg) {
	case 0:
		return ata_mselect_control_spg0(qc, buf, len, fp);
	case ATA_FEATURE_SUB_MPAGE:
		return ata_mselect_control_ata_feature(qc, buf, len, fp);
	default:
		return -EINVAL;
	}
}

/**
 *	ata_scsi_mode_select_xlat - Simulate MODE SELECT 6, 10 commands
 *	@qc: Storage for translated ATA taskfile
 *
 *	Converts a MODE SELECT command to an ATA SET FEATURES taskfile.
 *	Assume this is invoked for direct access devices (e.g. disks) only.
 *	There should be no block descriptor for other device types.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */
static unsigned int ata_scsi_mode_select_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	const u8 *cdb = scmd->cmnd;
	u8 pg, spg;
	unsigned six_byte, pg_len, hdr_len, bd_len;
	int len, ret;
	u16 fp = (u16)-1;
	u8 bp = 0xff;
	u8 buffer[64];
	const u8 *p = buffer;

	six_byte = (cdb[0] == MODE_SELECT);
	if (six_byte) {
		if (scmd->cmd_len < 5) {
			fp = 4;
			goto invalid_fld;
		}

		len = cdb[4];
		hdr_len = 4;
	} else {
		if (scmd->cmd_len < 9) {
			fp = 8;
			goto invalid_fld;
		}

		len = get_unaligned_be16(&cdb[7]);
		hdr_len = 8;
	}

	/* We only support PF=1, SP=0.  */
	if ((cdb[1] & 0x11) != 0x10) {
		fp = 1;
		bp = (cdb[1] & 0x01) ? 1 : 5;
		goto invalid_fld;
	}

	/* Test early for possible overrun.  */
	if (!scsi_sg_count(scmd) || scsi_sglist(scmd)->length < len)
		goto invalid_param_len;

	/* Move past header and block descriptors.  */
	if (len < hdr_len)
		goto invalid_param_len;

	if (!sg_copy_to_buffer(scsi_sglist(scmd), scsi_sg_count(scmd),
			       buffer, sizeof(buffer)))
		goto invalid_param_len;

	if (six_byte)
		bd_len = p[3];
	else
		bd_len = get_unaligned_be16(&p[6]);

	len -= hdr_len;
	p += hdr_len;
	if (len < bd_len)
		goto invalid_param_len;
	if (bd_len != 0 && bd_len != 8) {
		fp = (six_byte) ? 3 : 6;
		fp += bd_len + hdr_len;
		goto invalid_param;
	}

	len -= bd_len;
	p += bd_len;
	if (len == 0)
		goto skip;

	/* Parse both possible formats for the mode page headers.  */
	pg = p[0] & 0x3f;
	if (p[0] & 0x40) {
		if (len < 4)
			goto invalid_param_len;

		spg = p[1];
		pg_len = get_unaligned_be16(&p[2]);
		p += 4;
		len -= 4;
	} else {
		if (len < 2)
			goto invalid_param_len;

		spg = 0;
		pg_len = p[1];
		p += 2;
		len -= 2;
	}

	/*
	 * Supported subpages: all subpages and ATA feature sub-page f2h of
	 * the control page.
	 */
	if (spg) {
		switch (spg) {
		case ALL_SUB_MPAGES:
			/* All subpages is not supported for the control page */
			if (pg == CONTROL_MPAGE) {
				fp = (p[0] & 0x40) ? 1 : 0;
				fp += hdr_len + bd_len;
				goto invalid_param;
			}
			break;
		case ATA_FEATURE_SUB_MPAGE:
			if (qc->dev->flags & ATA_DFLAG_CDL &&
			    pg == CONTROL_MPAGE)
				break;
			fallthrough;
		default:
			fp = (p[0] & 0x40) ? 1 : 0;
			fp += hdr_len + bd_len;
			goto invalid_param;
		}
	}
	if (pg_len > len)
		goto invalid_param_len;

	switch (pg) {
	case CACHE_MPAGE:
		if (ata_mselect_caching(qc, p, pg_len, &fp) < 0) {
			fp += hdr_len + bd_len;
			goto invalid_param;
		}
		break;
	case CONTROL_MPAGE:
		ret = ata_mselect_control(qc, spg, p, pg_len, &fp);
		if (ret < 0) {
			fp += hdr_len + bd_len;
			goto invalid_param;
		}
		if (!ret)
			goto skip; /* No ATA command to send */
		break;
	default:
		/* Invalid page code */
		fp = bd_len + hdr_len;
		goto invalid_param;
	}

	/*
	 * Only one page has changeable data, so we only support setting one
	 * page at a time.
	 */
	if (len > pg_len)
		goto invalid_param;

	return 0;

 invalid_fld:
	ata_scsi_set_invalid_field(qc->dev, scmd, fp, bp);
	return 1;

 invalid_param:
	ata_scsi_set_invalid_parameter(qc->dev, scmd, fp);
	return 1;

 invalid_param_len:
	/* "Parameter list length error" */
	ata_scsi_set_sense(qc->dev, scmd, ILLEGAL_REQUEST, 0x1a, 0x0);
	return 1;

 skip:
	scmd->result = SAM_STAT_GOOD;
	return 1;
}

static u8 ata_scsi_trusted_op(u32 len, bool send, bool dma)
{
	if (len == 0)
		return ATA_CMD_TRUSTED_NONDATA;
	else if (send)
		return dma ? ATA_CMD_TRUSTED_SND_DMA : ATA_CMD_TRUSTED_SND;
	else
		return dma ? ATA_CMD_TRUSTED_RCV_DMA : ATA_CMD_TRUSTED_RCV;
}

static unsigned int ata_scsi_security_inout_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	const u8 *cdb = scmd->cmnd;
	struct ata_taskfile *tf = &qc->tf;
	u8 secp = cdb[1];
	bool send = (cdb[0] == SECURITY_PROTOCOL_OUT);
	u16 spsp = get_unaligned_be16(&cdb[2]);
	u32 len = get_unaligned_be32(&cdb[6]);
	bool dma = !(qc->dev->flags & ATA_DFLAG_PIO);

	/*
	 * We don't support the ATA "security" protocol.
	 */
	if (secp == 0xef) {
		ata_scsi_set_invalid_field(qc->dev, scmd, 1, 0);
		return 1;
	}

	if (cdb[4] & 7) { /* INC_512 */
		if (len > 0xffff) {
			ata_scsi_set_invalid_field(qc->dev, scmd, 6, 0);
			return 1;
		}
	} else {
		if (len > 0x01fffe00) {
			ata_scsi_set_invalid_field(qc->dev, scmd, 6, 0);
			return 1;
		}

		/* convert to the sector-based ATA addressing */
		len = (len + 511) / 512;
	}

	tf->protocol = dma ? ATA_PROT_DMA : ATA_PROT_PIO;
	tf->flags |= ATA_TFLAG_DEVICE | ATA_TFLAG_ISADDR | ATA_TFLAG_LBA;
	if (send)
		tf->flags |= ATA_TFLAG_WRITE;
	tf->command = ata_scsi_trusted_op(len, send, dma);
	tf->feature = secp;
	tf->lbam = spsp & 0xff;
	tf->lbah = spsp >> 8;

	if (len) {
		tf->nsect = len & 0xff;
		tf->lbal = len >> 8;
	} else {
		if (!send)
			tf->lbah = (1 << 7);
	}

	ata_qc_set_pc_nbytes(qc);
	return 0;
}

/**
 *	ata_scsi_var_len_cdb_xlat - SATL variable length CDB to Handler
 *	@qc: Command to be translated
 *
 *	Translate a SCSI variable length CDB to specified commands.
 *	It checks a service action value in CDB to call corresponding handler.
 *
 *	RETURNS:
 *	Zero on success, non-zero on failure
 *
 */
static unsigned int ata_scsi_var_len_cdb_xlat(struct ata_queued_cmd *qc)
{
	struct scsi_cmnd *scmd = qc->scsicmd;
	const u8 *cdb = scmd->cmnd;
	const u16 sa = get_unaligned_be16(&cdb[8]);

	/*
	 * if service action represents a ata pass-thru(32) command,
	 * then pass it to ata_scsi_pass_thru handler.
	 */
	if (sa == ATA_32)
		return ata_scsi_pass_thru(qc);

	/* unsupported service action */
	return 1;
}

/**
 *	ata_get_xlat_func - check if SCSI to ATA translation is possible
 *	@dev: ATA device
 *	@cmd: SCSI command opcode to consider
 *
 *	Look up the SCSI command given, and determine whether the
 *	SCSI command is to be translated or simulated.
 *
 *	RETURNS:
 *	Pointer to translation function if possible, %NULL if not.
 */

static inline ata_xlat_func_t ata_get_xlat_func(struct ata_device *dev, u8 cmd)
{
	switch (cmd) {
	case READ_6:
	case READ_10:
	case READ_16:

	case WRITE_6:
	case WRITE_10:
	case WRITE_16:
		return ata_scsi_rw_xlat;

	case WRITE_SAME_16:
		return ata_scsi_write_same_xlat;

	case SYNCHRONIZE_CACHE:
	case SYNCHRONIZE_CACHE_16:
		if (ata_try_flush_cache(dev))
			return ata_scsi_flush_xlat;
		break;

	case VERIFY:
	case VERIFY_16:
		return ata_scsi_verify_xlat;

	case ATA_12:
	case ATA_16:
		return ata_scsi_pass_thru;

	case VARIABLE_LENGTH_CMD:
		return ata_scsi_var_len_cdb_xlat;

	case MODE_SELECT:
	case MODE_SELECT_10:
		return ata_scsi_mode_select_xlat;

	case ZBC_IN:
		return ata_scsi_zbc_in_xlat;

	case ZBC_OUT:
		return ata_scsi_zbc_out_xlat;

	case SECURITY_PROTOCOL_IN:
	case SECURITY_PROTOCOL_OUT:
		if (!(dev->flags & ATA_DFLAG_TRUSTED))
			break;
		return ata_scsi_security_inout_xlat;

	case START_STOP:
		return ata_scsi_start_stop_xlat;
	}

	return NULL;
}

int __ata_scsi_queuecmd(struct scsi_cmnd *scmd, struct ata_device *dev)
{
	struct ata_port *ap = dev->link->ap;
	u8 scsi_op = scmd->cmnd[0];
	ata_xlat_func_t xlat_func;

	/*
	 * scsi_queue_rq() will defer commands if scsi_host_in_recovery().
	 * However, this check is done without holding the ap->lock (a libata
	 * specific lock), so we can have received an error irq since then,
	 * therefore we must check if EH is pending or running, while holding
	 * ap->lock.
	 */
	if (ata_port_eh_scheduled(ap))
		return SCSI_MLQUEUE_DEVICE_BUSY;

	if (unlikely(!scmd->cmd_len))
		goto bad_cdb_len;

	if (dev->class == ATA_DEV_ATA || dev->class == ATA_DEV_ZAC) {
		if (unlikely(scmd->cmd_len > dev->cdb_len))
			goto bad_cdb_len;

		xlat_func = ata_get_xlat_func(dev, scsi_op);
	} else if (likely((scsi_op != ATA_16) || !atapi_passthru16)) {
		/* relay SCSI command to ATAPI device */
		int len = COMMAND_SIZE(scsi_op);

		if (unlikely(len > scmd->cmd_len ||
			     len > dev->cdb_len ||
			     scmd->cmd_len > ATAPI_CDB_LEN))
			goto bad_cdb_len;

		xlat_func = atapi_xlat;
	} else {
		/* ATA_16 passthru, treat as an ATA command */
		if (unlikely(scmd->cmd_len > 16))
			goto bad_cdb_len;

		xlat_func = ata_get_xlat_func(dev, scsi_op);
	}

	if (xlat_func)
		return ata_scsi_translate(dev, scmd, xlat_func);

	ata_scsi_simulate(dev, scmd);

	return 0;

 bad_cdb_len:
	scmd->result = DID_ERROR << 16;
	scsi_done(scmd);
	return 0;
}

/**
 *	ata_scsi_queuecmd - Issue SCSI cdb to libata-managed device
 *	@shost: SCSI host of command to be sent
 *	@cmd: SCSI command to be sent
 *
 *	In some cases, this function translates SCSI commands into
 *	ATA taskfiles, and queues the taskfiles to be sent to
 *	hardware.  In other cases, this function simulates a
 *	SCSI device by evaluating and responding to certain
 *	SCSI commands.  This creates the overall effect of
 *	ATA and ATAPI devices appearing as SCSI devices.
 *
 *	LOCKING:
 *	ATA host lock
 *
 *	RETURNS:
 *	Return value from __ata_scsi_queuecmd() if @cmd can be queued,
 *	0 otherwise.
 */
int ata_scsi_queuecmd(struct Scsi_Host *shost, struct scsi_cmnd *cmd)
{
	struct ata_port *ap;
	struct ata_device *dev;
	struct scsi_device *scsidev = cmd->device;
	int rc = 0;
	unsigned long irq_flags;

	ap = ata_shost_to_port(shost);

	spin_lock_irqsave(ap->lock, irq_flags);

	dev = ata_scsi_find_dev(ap, scsidev);
	if (likely(dev))
		rc = __ata_scsi_queuecmd(cmd, dev);
	else {
		cmd->result = (DID_BAD_TARGET << 16);
		scsi_done(cmd);
	}

	spin_unlock_irqrestore(ap->lock, irq_flags);

	return rc;
}
EXPORT_SYMBOL_GPL(ata_scsi_queuecmd);

/**
 *	ata_scsi_simulate - simulate SCSI command on ATA device
 *	@dev: the target device
 *	@cmd: SCSI command being sent to device.
 *
 *	Interprets and directly executes a select list of SCSI commands
 *	that can be handled internally.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 */

void ata_scsi_simulate(struct ata_device *dev, struct scsi_cmnd *cmd)
{
	const u8 *scsicmd = cmd->cmnd;
	u8 tmp8;

	switch(scsicmd[0]) {
	case INQUIRY:
		ata_scsi_rbuf_fill(dev, cmd, ata_scsiop_inquiry);
		break;

	case MODE_SENSE:
	case MODE_SENSE_10:
		ata_scsi_rbuf_fill(dev, cmd, ata_scsiop_mode_sense);
		break;

	case READ_CAPACITY:
	case SERVICE_ACTION_IN_16:
		ata_scsi_rbuf_fill(dev, cmd, ata_scsiop_read_cap);
		break;

	case REPORT_LUNS:
		ata_scsi_rbuf_fill(dev, cmd, ata_scsiop_report_luns);
		break;

	case REQUEST_SENSE:
		ata_scsi_set_sense(dev, cmd, 0, 0, 0);
		break;

	/* if we reach this, then writeback caching is disabled,
	 * turning this into a no-op.
	 */
	case SYNCHRONIZE_CACHE:
	case SYNCHRONIZE_CACHE_16:
		fallthrough;

	/* no-op's, complete with success */
	case REZERO_UNIT:
	case SEEK_6:
	case SEEK_10:
	case TEST_UNIT_READY:
		break;

	case SEND_DIAGNOSTIC:
		tmp8 = scsicmd[1] & ~(1 << 3);
		if (tmp8 != 0x4 || scsicmd[3] || scsicmd[4])
			ata_scsi_set_invalid_field(dev, cmd, 1, 0xff);
		break;

	case MAINTENANCE_IN:
		ata_scsi_rbuf_fill(dev, cmd, ata_scsiop_maint_in);
		break;

	/* all other commands */
	default:
		ata_scsi_set_sense(dev, cmd, ILLEGAL_REQUEST, 0x20, 0x0);
		/* "Invalid command operation code" */
		break;
	}

	scsi_done(cmd);
}

int ata_scsi_add_hosts(struct ata_host *host, const struct scsi_host_template *sht)
{
	int i, rc;

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];
		struct Scsi_Host *shost;

		rc = -ENOMEM;
		shost = scsi_host_alloc(sht, sizeof(struct ata_port *));
		if (!shost)
			goto err_alloc;

		shost->eh_noresume = 1;
		*(struct ata_port **)&shost->hostdata[0] = ap;
		ap->scsi_host = shost;

		shost->transportt = ata_scsi_transport_template;
		shost->unique_id = ap->print_id;
		shost->max_id = 16;
		shost->max_lun = 1;
		shost->max_channel = 1;
		shost->max_cmd_len = 32;

		/* Schedule policy is determined by ->qc_defer()
		 * callback and it needs to see every deferred qc.
		 * Set host_blocked to 1 to prevent SCSI midlayer from
		 * automatically deferring requests.
		 */
		shost->max_host_blocked = 1;

		rc = scsi_add_host_with_dma(shost, &ap->tdev, ap->host->dev);
		if (rc)
			goto err_alloc;
	}

	return 0;

 err_alloc:
	while (--i >= 0) {
		struct Scsi_Host *shost = host->ports[i]->scsi_host;

		/* scsi_host_put() is in ata_devres_release() */
		scsi_remove_host(shost);
	}
	return rc;
}

#ifdef CONFIG_OF
static void ata_scsi_assign_ofnode(struct ata_device *dev, struct ata_port *ap)
{
	struct scsi_device *sdev = dev->sdev;
	struct device *d = ap->host->dev;
	struct device_node *np = d->of_node;
	struct device_node *child;

	for_each_available_child_of_node(np, child) {
		int ret;
		u32 val;

		ret = of_property_read_u32(child, "reg", &val);
		if (ret)
			continue;
		if (val == dev->devno) {
			dev_dbg(d, "found matching device node\n");
			sdev->sdev_gendev.of_node = child;
			return;
		}
	}
}
#else
static void ata_scsi_assign_ofnode(struct ata_device *dev, struct ata_port *ap)
{
}
#endif

void ata_scsi_scan_host(struct ata_port *ap, int sync)
{
	int tries = 5;
	struct ata_device *last_failed_dev = NULL;
	struct ata_link *link;
	struct ata_device *dev;

 repeat:
	ata_for_each_link(link, ap, EDGE) {
		ata_for_each_dev(dev, link, ENABLED) {
			struct scsi_device *sdev;
			int channel = 0, id = 0;

			if (dev->sdev)
				continue;

			if (ata_is_host_link(link))
				id = dev->devno;
			else
				channel = link->pmp;

			sdev = __scsi_add_device(ap->scsi_host, channel, id, 0,
						 NULL);
			if (!IS_ERR(sdev)) {
				dev->sdev = sdev;
				ata_scsi_assign_ofnode(dev, ap);
				scsi_device_put(sdev);
			} else {
				dev->sdev = NULL;
			}
		}
	}

	/* If we scanned while EH was in progress or allocation
	 * failure occurred, scan would have failed silently.  Check
	 * whether all devices are attached.
	 */
	ata_for_each_link(link, ap, EDGE) {
		ata_for_each_dev(dev, link, ENABLED) {
			if (!dev->sdev)
				goto exit_loop;
		}
	}
 exit_loop:
	if (!link)
		return;

	/* we're missing some SCSI devices */
	if (sync) {
		/* If caller requested synchrnous scan && we've made
		 * any progress, sleep briefly and repeat.
		 */
		if (dev != last_failed_dev) {
			msleep(100);
			last_failed_dev = dev;
			goto repeat;
		}

		/* We might be failing to detect boot device, give it
		 * a few more chances.
		 */
		if (--tries) {
			msleep(100);
			goto repeat;
		}

		ata_port_err(ap,
			     "WARNING: synchronous SCSI scan failed without making any progress, switching to async\n");
	}

	queue_delayed_work(system_long_wq, &ap->hotplug_task,
			   round_jiffies_relative(HZ));
}

/**
 *	ata_scsi_offline_dev - offline attached SCSI device
 *	@dev: ATA device to offline attached SCSI device for
 *
 *	This function is called from ata_eh_detach_dev() and is responsible for
 *	taking the SCSI device attached to @dev offline.  This function is
 *	called with host lock which protects dev->sdev against clearing.
 *
 *	LOCKING:
 *	spin_lock_irqsave(host lock)
 *
 *	RETURNS:
 *	true if attached SCSI device exists, false otherwise.
 */
bool ata_scsi_offline_dev(struct ata_device *dev)
{
	if (dev->sdev) {
		scsi_device_set_state(dev->sdev, SDEV_OFFLINE);
		return true;
	}
	return false;
}

/**
 *	ata_scsi_remove_dev - remove attached SCSI device
 *	@dev: ATA device to remove attached SCSI device for
 *
 *	This function is called from ata_eh_scsi_hotplug() and
 *	responsible for removing the SCSI device attached to @dev.
 *
 *	LOCKING:
 *	Kernel thread context (may sleep).
 */
static void ata_scsi_remove_dev(struct ata_device *dev)
{
	struct ata_port *ap = dev->link->ap;
	struct scsi_device *sdev;
	unsigned long flags;

	/* Alas, we need to grab scan_mutex to ensure SCSI device
	 * state doesn't change underneath us and thus
	 * scsi_device_get() always succeeds.  The mutex locking can
	 * be removed if there is __scsi_device_get() interface which
	 * increments reference counts regardless of device state.
	 */
	mutex_lock(&ap->scsi_host->scan_mutex);
	spin_lock_irqsave(ap->lock, flags);

	/* clearing dev->sdev is protected by host lock */
	sdev = dev->sdev;
	dev->sdev = NULL;

	if (sdev) {
		/* If user initiated unplug races with us, sdev can go
		 * away underneath us after the host lock and
		 * scan_mutex are released.  Hold onto it.
		 */
		if (scsi_device_get(sdev) == 0) {
			/* The following ensures the attached sdev is
			 * offline on return from ata_scsi_offline_dev()
			 * regardless it wins or loses the race
			 * against this function.
			 */
			scsi_device_set_state(sdev, SDEV_OFFLINE);
		} else {
			WARN_ON(1);
			sdev = NULL;
		}
	}

	spin_unlock_irqrestore(ap->lock, flags);
	mutex_unlock(&ap->scsi_host->scan_mutex);

	if (sdev) {
		ata_dev_info(dev, "detaching (SCSI %s)\n",
			     dev_name(&sdev->sdev_gendev));

		scsi_remove_device(sdev);
		scsi_device_put(sdev);
	}
}

static void ata_scsi_handle_link_detach(struct ata_link *link)
{
	struct ata_port *ap = link->ap;
	struct ata_device *dev;

	ata_for_each_dev(dev, link, ALL) {
		unsigned long flags;

		spin_lock_irqsave(ap->lock, flags);
		if (!(dev->flags & ATA_DFLAG_DETACHED)) {
			spin_unlock_irqrestore(ap->lock, flags);
			continue;
		}

		dev->flags &= ~ATA_DFLAG_DETACHED;
		spin_unlock_irqrestore(ap->lock, flags);

		ata_scsi_remove_dev(dev);
	}
}

/**
 *	ata_scsi_media_change_notify - send media change event
 *	@dev: Pointer to the disk device with media change event
 *
 *	Tell the block layer to send a media change notification
 *	event.
 *
 * 	LOCKING:
 * 	spin_lock_irqsave(host lock)
 */
void ata_scsi_media_change_notify(struct ata_device *dev)
{
	if (dev->sdev)
		sdev_evt_send_simple(dev->sdev, SDEV_EVT_MEDIA_CHANGE,
				     GFP_ATOMIC);
}

/**
 *	ata_scsi_hotplug - SCSI part of hotplug
 *	@work: Pointer to ATA port to perform SCSI hotplug on
 *
 *	Perform SCSI part of hotplug.  It's executed from a separate
 *	workqueue after EH completes.  This is necessary because SCSI
 *	hot plugging requires working EH and hot unplugging is
 *	synchronized with hot plugging with a mutex.
 *
 *	LOCKING:
 *	Kernel thread context (may sleep).
 */
void ata_scsi_hotplug(struct work_struct *work)
{
	struct ata_port *ap =
		container_of(work, struct ata_port, hotplug_task.work);
	int i;

	if (ap->pflags & ATA_PFLAG_UNLOADING)
		return;

	mutex_lock(&ap->scsi_scan_mutex);

	/* Unplug detached devices.  We cannot use link iterator here
	 * because PMP links have to be scanned even if PMP is
	 * currently not attached.  Iterate manually.
	 */
	ata_scsi_handle_link_detach(&ap->link);
	if (ap->pmp_link)
		for (i = 0; i < SATA_PMP_MAX_PORTS; i++)
			ata_scsi_handle_link_detach(&ap->pmp_link[i]);

	/* scan for new ones */
	ata_scsi_scan_host(ap, 0);

	mutex_unlock(&ap->scsi_scan_mutex);
}

/**
 *	ata_scsi_user_scan - indication for user-initiated bus scan
 *	@shost: SCSI host to scan
 *	@channel: Channel to scan
 *	@id: ID to scan
 *	@lun: LUN to scan
 *
 *	This function is called when user explicitly requests bus
 *	scan.  Set probe pending flag and invoke EH.
 *
 *	LOCKING:
 *	SCSI layer (we don't care)
 *
 *	RETURNS:
 *	Zero.
 */
int ata_scsi_user_scan(struct Scsi_Host *shost, unsigned int channel,
		       unsigned int id, u64 lun)
{
	struct ata_port *ap = ata_shost_to_port(shost);
	unsigned long flags;
	int devno, rc = 0;

	if (lun != SCAN_WILD_CARD && lun)
		return -EINVAL;

	if (!sata_pmp_attached(ap)) {
		if (channel != SCAN_WILD_CARD && channel)
			return -EINVAL;
		devno = id;
	} else {
		if (id != SCAN_WILD_CARD && id)
			return -EINVAL;
		devno = channel;
	}

	spin_lock_irqsave(ap->lock, flags);

	if (devno == SCAN_WILD_CARD) {
		struct ata_link *link;

		ata_for_each_link(link, ap, EDGE) {
			struct ata_eh_info *ehi = &link->eh_info;
			ehi->probe_mask |= ATA_ALL_DEVICES;
			ehi->action |= ATA_EH_RESET;
		}
	} else {
		struct ata_device *dev = ata_find_dev(ap, devno);

		if (dev) {
			struct ata_eh_info *ehi = &dev->link->eh_info;
			ehi->probe_mask |= 1 << dev->devno;
			ehi->action |= ATA_EH_RESET;
		} else
			rc = -EINVAL;
	}

	if (rc == 0) {
		ata_port_schedule_eh(ap);
		spin_unlock_irqrestore(ap->lock, flags);
		ata_port_wait_eh(ap);
	} else
		spin_unlock_irqrestore(ap->lock, flags);

	return rc;
}

/**
 *	ata_scsi_dev_rescan - initiate scsi_rescan_device()
 *	@work: Pointer to ATA port to perform scsi_rescan_device()
 *
 *	After ATA pass thru (SAT) commands are executed successfully,
 *	libata need to propagate the changes to SCSI layer.
 *
 *	LOCKING:
 *	Kernel thread context (may sleep).
 */
void ata_scsi_dev_rescan(struct work_struct *work)
{
	struct ata_port *ap =
		container_of(work, struct ata_port, scsi_rescan_task.work);
	struct ata_link *link;
	struct ata_device *dev;
	unsigned long flags;
	bool do_resume;
	int ret = 0;

	mutex_lock(&ap->scsi_scan_mutex);
	spin_lock_irqsave(ap->lock, flags);

	ata_for_each_link(link, ap, EDGE) {
		ata_for_each_dev(dev, link, ENABLED) {
			struct scsi_device *sdev = dev->sdev;

			/*
			 * If the port was suspended before this was scheduled,
			 * bail out.
			 */
			if (ap->pflags & ATA_PFLAG_SUSPENDED)
				goto unlock_ap;

			if (!sdev)
				continue;
			if (scsi_device_get(sdev))
				continue;

			do_resume = dev->flags & ATA_DFLAG_RESUMING;

			spin_unlock_irqrestore(ap->lock, flags);
			if (do_resume) {
				ret = scsi_resume_device(sdev);
				if (ret == -EWOULDBLOCK)
					goto unlock_scan;
				dev->flags &= ~ATA_DFLAG_RESUMING;
			}
			ret = scsi_rescan_device(sdev);
			scsi_device_put(sdev);
			spin_lock_irqsave(ap->lock, flags);

			if (ret)
				goto unlock_ap;
		}
	}

unlock_ap:
	spin_unlock_irqrestore(ap->lock, flags);
unlock_scan:
	mutex_unlock(&ap->scsi_scan_mutex);

	/* Reschedule with a delay if scsi_rescan_device() returned an error */
	if (ret)
		schedule_delayed_work(&ap->scsi_rescan_task,
				      msecs_to_jiffies(5));
}
