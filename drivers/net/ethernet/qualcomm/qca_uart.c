/*
 *   Copyright (c) 2011, 2012, Qualcomm Atheros Communications Inc.
 *   Copyright (c) 2016, I2SE GmbH
 *
 *   Permission to use, copy, modify, and/or distribute this software
 *   for any purpose with or without fee is hereby granted, provided
 *   that the above copyright notice and this permission notice appear
 *   in all copies.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 *   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 *   WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL
 *   THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 *   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 *   LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 *   NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 *   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*   This module implements the Qualcomm Atheros UART protocol for
 *   kernel-based UART device; it is essentially an Ethernet-to-UART
 *   serial converter;
 */

#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/tty.h>
#include <linux/types.h>

#include "qca_common.h"

#define QCAUART_DRV_VERSION "0.0.6"
#define QCAUART_DRV_NAME "qcauart"
#define QCAUART_TX_TIMEOUT (1 * HZ)

struct qcauart {
	struct net_device *net_dev;
	spinlock_t lock;
	struct work_struct tx_work;		/* Flushes transmit buffer   */

	struct tty_struct *tty;

	unsigned char xbuff[QCAFRM_ETHMAXMTU];	/* transmitter buffer        */
	unsigned char *xhead;			/* pointer to next XMIT byte */
	int xleft;				/* bytes left in XMIT queue  */

	struct qcafrm_handle frm_handle;

	struct sk_buff *rx_skb;
};

static struct net_device *qcauart_dev;

void
qca_tty_receive(struct tty_struct *tty, const unsigned char *cp, char *fp,
		int count)
{
	struct qcauart *qca = tty->disc_data;
	struct net_device_stats *n_stats = &qca->net_dev->stats;
	int dropped = 0;

	if (!qca->rx_skb) {
		qca->rx_skb = netdev_alloc_skb(qca->net_dev, qca->net_dev->mtu +
					       VLAN_ETH_HLEN);
		if (!qca->rx_skb) {
			n_stats->rx_errors++;
			n_stats->rx_dropped++;
			return;
		}
	}

	while (count--) {
		s32 retcode;

		if (fp && *fp++) {
			n_stats->rx_errors++;
			cp++;
			dropped++;
			continue;
		}

		retcode = qcafrm_fsm_decode(&qca->frm_handle,
					    qca->rx_skb->data,
					    skb_tailroom(qca->rx_skb),
					    *cp);

		cp++;
		switch (retcode) {
		case QCAFRM_GATHER:
		case QCAFRM_NOHEAD:
			break;
		case QCAFRM_NOTAIL:
			netdev_dbg(qca->net_dev, "recv: no RX tail\n");
			n_stats->rx_errors++;
			n_stats->rx_dropped++;
			break;
		case QCAFRM_INVLEN:
			netdev_dbg(qca->net_dev, "recv: invalid RX length\n");
			n_stats->rx_errors++;
			n_stats->rx_dropped++;
			break;
		default:
			qca->rx_skb->dev = qca->net_dev;
			n_stats->rx_packets++;
			n_stats->rx_bytes += retcode;
			skb_put(qca->rx_skb, retcode);
			qca->rx_skb->protocol = eth_type_trans(
						qca->rx_skb, qca->rx_skb->dev);
			qca->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
			netif_rx_ni(qca->rx_skb);
			qca->rx_skb = netdev_alloc_skb(qca->net_dev,
						       qca->net_dev->mtu +
						       VLAN_ETH_HLEN);
			if (!qca->rx_skb) {
				netdev_dbg(qca->net_dev, "recv: out of RX resources\n");
				n_stats->rx_errors++;
				break;
			}
		}
	}

	if (dropped)
		dev_dbg_ratelimited(&qca->net_dev->dev, "recv: invalid %d bytes\n",
				    dropped);
}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void qcauart_transmit(struct work_struct *work)
{
	struct qcauart *qca = container_of(work, struct qcauart, tx_work);
	struct net_device_stats *n_stats = &qca->net_dev->stats;
	int written;

	spin_lock_bh(&qca->lock);
	/* First make sure we're connected. */
	if (!qca->tty || !netif_running(qca->net_dev)) {
		spin_unlock_bh(&qca->lock);
		return;
	}

	if (qca->xleft <= 0)  {
		/* Now serial buffer is almost free & we can start
		 * transmission of another packet
		 */
		n_stats->tx_packets++;
		clear_bit(TTY_DO_WRITE_WAKEUP, &qca->tty->flags);
		spin_unlock_bh(&qca->lock);
		netif_wake_queue(qca->net_dev);
		return;
	}

	written = qca->tty->ops->write(qca->tty, qca->xhead, qca->xleft);
	qca->xleft -= written;
	qca->xhead += written;
	spin_unlock_bh(&qca->lock);
}

/* Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void qca_tty_wakeup(struct tty_struct *tty)
{
	struct qcauart *qca = tty->disc_data;

	schedule_work(&qca->tx_work);
}

int
qca_tty_open(struct tty_struct *tty)
{
	struct qcauart *qca;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!tty->ops->write)
		return -EOPNOTSUPP;

	if (tty->disc_data)
		return -EEXIST;

	qca = netdev_priv(qcauart_dev);
	qca->tty = tty;
	tty->disc_data = qca;
	tty->receive_room = 4096;
	netif_carrier_on(qca->net_dev);

	return 0;
}

void
qca_tty_close(struct tty_struct *tty)
{
	struct qcauart *qca = (void *)tty->disc_data;

	if (!qca)
		return;

	netif_carrier_off(qca->net_dev);
	qca->tty = NULL;

	/* Detach from the tty */
	tty->disc_data = NULL;
}

static struct tty_ldisc_ops qca_ldisc = {
	.owner  = THIS_MODULE,
	.magic	= TTY_LDISC_MAGIC,
	.name	= "qca",
	.open	= qca_tty_open,
	.close	= qca_tty_close,
	.ioctl	= n_tty_ioctl_helper,
	.receive_buf = qca_tty_receive,
	.write_wakeup = qca_tty_wakeup,
};

int
qcauart_netdev_open(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	qcafrm_fsm_init_uart(&qca->frm_handle);
	netif_start_queue(qca->net_dev);

	return 0;
}

int
qcauart_netdev_close(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	spin_lock_bh(&qca->lock);
	if (qca->tty) {
		/* TTY discipline is running. */
		clear_bit(TTY_DO_WRITE_WAKEUP, &qca->tty->flags);
	}
	netif_stop_queue(dev);
	qca->xleft = 0;
	spin_unlock_bh(&qca->lock);

	return 0;
}

netdev_tx_t
qcauart_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);
	struct net_device_stats *n_stats = &dev->stats;
	u8 *pos;
	u8 pad_len = 0;
	int written;

	spin_lock(&qca->lock);

	if (!netif_running(dev))  {
		spin_unlock(&qca->lock);
		netdev_warn(qca->net_dev, "xmit: iface is down\n");
		goto out;
	}
	if (!qca->tty) {
		spin_unlock(&qca->lock);
		goto out;
	}

	pos = qca->xbuff;

	if (skb->len < QCAFRM_ETHMINLEN)
		pad_len = QCAFRM_ETHMINLEN - skb->len;

	pos += qcafrm_create_header(pos, skb->len + pad_len);

	memcpy(pos, skb->data, skb->len);
	pos += skb->len;

	if (pad_len) {
		memset(pos, 0, pad_len);
		pos += pad_len;
	}

	pos += qcafrm_create_footer(pos);

	netif_stop_queue(qca->net_dev);

	set_bit(TTY_DO_WRITE_WAKEUP, &qca->tty->flags);
	written = qca->tty->ops->write(qca->tty, qca->xbuff, pos - qca->xbuff);
	qca->xleft = (pos - qca->xbuff) - written;
	qca->xhead = qca->xbuff + written;
	n_stats->tx_bytes += written;
	spin_unlock(&qca->lock);

	dev->trans_start = jiffies;
out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

void
qcauart_netdev_tx_timeout(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	netdev_info(qca->net_dev, "Transmit timeout at %ld, latency %ld\n",
		    jiffies, jiffies - dev->trans_start);
	dev->stats.tx_errors++;
	dev->stats.tx_dropped++;

	clear_bit(TTY_DO_WRITE_WAKEUP, &qca->tty->flags);
}

static int
qcauart_netdev_init(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	/* Finish setting up the device info. */
	dev->mtu = QCAFRM_ETHMAXMTU;
	dev->type = ARPHRD_ETHER;

	qca->rx_skb = netdev_alloc_skb(qca->net_dev,
				       qca->net_dev->mtu + VLAN_ETH_HLEN);
	if (!qca->rx_skb)
		return -ENOMEM;

	return 0;
}

static void
qcauart_netdev_uninit(struct net_device *dev)
{
	struct qcauart *qca = netdev_priv(dev);

	if (qca->rx_skb)
		dev_kfree_skb(qca->rx_skb);
}

static const struct net_device_ops qcauart_netdev_ops = {
	.ndo_init = qcauart_netdev_init,
	.ndo_uninit = qcauart_netdev_uninit,
	.ndo_open = qcauart_netdev_open,
	.ndo_stop = qcauart_netdev_close,
	.ndo_start_xmit = qcauart_netdev_xmit,
	.ndo_change_mtu = qcacmn_netdev_change_mtu,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_tx_timeout = qcauart_netdev_tx_timeout,
	.ndo_validate_addr = eth_validate_addr,
};

static void
qcauart_netdev_setup(struct net_device *dev)
{
	struct qcauart *qca;

	dev->netdev_ops = &qcauart_netdev_ops;
	dev->watchdog_timeo = QCAUART_TX_TIMEOUT;
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->tx_queue_len = 100;

	qca = netdev_priv(dev);
	memset(qca, 0, sizeof(struct qcauart));
}

static int __init qca_uart_mod_init(void)
{
	struct qcauart *qca;
	int ret;

	ret = tty_register_ldisc(N_QCA7K, &qca_ldisc);
	if (ret) {
		pr_err("qca_uart: Can't register line discipline (ret %d)\n",
		       ret);
		return ret;
	}

	qcauart_dev = alloc_etherdev(sizeof(struct qcauart));
	if (!qcauart_dev)
		return -ENOMEM;

	qcauart_netdev_setup(qcauart_dev);

	qca = netdev_priv(qcauart_dev);
	if (!qca) {
		pr_err("qca_uart: Fail to retrieve private structure\n");
		free_netdev(qcauart_dev);
		return -ENOMEM;
	}
	qca->net_dev = qcauart_dev;
	spin_lock_init(&qca->lock);
	INIT_WORK(&qca->tx_work, qcauart_transmit);

	eth_hw_addr_random(qca->net_dev);
	pr_info("qca_uart: ver=%s, using random MAC address: %pM\n",
		QCAUART_DRV_VERSION, qca->net_dev->dev_addr);

	netif_carrier_off(qca->net_dev);

	if (register_netdev(qcauart_dev)) {
		pr_err("qca_uart: Unable to register net device %s\n",
		       qcauart_dev->name);
		free_netdev(qcauart_dev);
		return -EFAULT;
	}

	return 0;
}

static void __exit qca_uart_mod_exit(void)
{
	struct qcauart *qca;
	int ret;

	qca = netdev_priv(qcauart_dev);
	spin_lock_bh(&qca->lock);
	if (qca->tty)
		tty_hangup(qca->tty);
	spin_unlock_bh(&qca->lock);

	unregister_netdev(qcauart_dev);

	free_netdev(qcauart_dev);
	qcauart_dev = NULL;

	ret = tty_unregister_ldisc(N_QCA7K);
	if (ret)
		pr_err("qca_uart: can't unregister line discipline (ret %d)\n",
		       ret);
}

module_init(qca_uart_mod_init);
module_exit(qca_uart_mod_exit);

MODULE_DESCRIPTION("Qualcomm Atheros UART Driver");
MODULE_AUTHOR("Qualcomm Atheros Communications");
MODULE_AUTHOR("Stefan Wahren <stefan.wahren@i2se.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(QCAUART_DRV_VERSION);
MODULE_ALIAS_LDISC(N_QCA7K);
