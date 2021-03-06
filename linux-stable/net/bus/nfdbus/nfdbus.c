/*
 *  nfdbus.c - Netfilter module for AF_BUS/BUS_PROTO_DBUS.
 */

#define DRIVER_AUTHOR "Alban Crequy"
#define DRIVER_DESC   "Netfilter module for AF_BUS/BUS_PROTO_DBUS."

#include "nfdbus.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netfilter.h>
#include <linux/connector.h>
#include <net/af_bus.h>

#include "message.h"
#include "matchrule.h"

static struct nf_hook_ops nfho_dbus;

static struct cb_id cn_cmd_id = { CN_IDX_NFDBUS, CN_VAL_NFDBUS };

static unsigned int hash;

/* Scoped by AF_BUS address */
struct hlist_head matchrules_table[BUS_HASH_SIZE];
DEFINE_SPINLOCK(matchrules_lock);

static struct bus_match_maker *find_match_maker(struct sockaddr_bus *addr,
		bool create, bool delete)
{
	u64 hash;
	struct hlist_node *node;
	struct bus_match_maker *matchmaker;
	int path_len = strlen(addr->sbus_path);

	hash = csum_partial(addr->sbus_path,
			    strlen(addr->sbus_path), 0);
	hash ^= addr->sbus_addr.s_addr;
	hash ^= hash >> 32;
	hash ^= hash >> 16;
	hash ^= hash >> 8;
	hash &= 0xff;

	spin_lock(&matchrules_lock);
	hlist_for_each_entry(matchmaker, node, &matchrules_table[hash],
			     table_node) {
		if (addr->sbus_family == matchmaker->addr.sbus_family &&
		    addr->sbus_addr.s_addr == matchmaker->addr.sbus_addr.s_addr &&
		    !memcmp(addr->sbus_path, matchmaker->addr.sbus_path,
			   path_len)) {
			kref_get(&matchmaker->kref);
			if (delete)
				hlist_del(&matchmaker->table_node);
			spin_unlock(&matchrules_lock);
			pr_debug("Found matchmaker for hash %llu", hash);
			return matchmaker;
		}
	}
	spin_unlock(&matchrules_lock);

	if (!create) {
		pr_debug("Matchmaker for hash %llu not found", hash);
		return NULL;
	}

	matchmaker = bus_matchmaker_new(GFP_ATOMIC);
	matchmaker->addr.sbus_family = addr->sbus_family;
	matchmaker->addr.sbus_addr.s_addr = addr->sbus_addr.s_addr;
	memcpy(matchmaker->addr.sbus_path, addr->sbus_path, BUS_PATH_MAX);

	pr_debug("Create new matchmaker for hash %llu\n", hash);
	spin_lock(&matchrules_lock);
	hlist_add_head(&matchmaker->table_node, &matchrules_table[hash]);
	kref_get(&matchmaker->kref);
	spin_unlock(&matchrules_lock);
	return matchmaker;
}

static unsigned int dbus_filter(unsigned int hooknum,
				struct sk_buff *skb,
				const struct net_device *in,
				const struct net_device *out,
				int (*okfn)(struct sk_buff *))
{
	struct bus_send_context	*sendctx;
	struct bus_match_maker *matchmaker = NULL;
	struct bus_match_maker *sender = NULL;
	struct dbus_message msg = {0,};
	unsigned char *data;
	size_t len;
	int err;
	int ret;

	if (!skb->sk || skb->sk->sk_family != PF_BUS) {
		WARN(1, "netfilter_dbus received an invalid skb");
		return NF_DROP;
	}

	data = skb->data;
	sendctx = BUSCB(skb).sendctx;
	if (!sendctx || !sendctx->sender || !sendctx->sender_socket) {
		WARN(1, "netfilter_dbus received an AF_BUS packet" \
		     " without context. This is a bug. Dropping the"
			" packet.");
		return NF_DROP;
	}

	if (sendctx->sender_socket->sk->sk_protocol != BUS_PROTO_DBUS) {
		/* This kernel module is for D-Bus. It must not
		 * interfere with other users of AF_BUS. */
		return NF_ACCEPT;
	}
	if (sendctx->recipient)
		matchmaker = find_match_maker(sendctx->recipient, false, false);

	len =  skb_tail_pointer(skb) - data;

	if (sendctx->to_master && sendctx->main_recipient) {
		pr_debug("AF_BUS packet to the bus master. ACCEPT.\n");
		ret = NF_ACCEPT;
		goto out;
	}

	if (sendctx->main_recipient && !sendctx->bus_master_side) {
		pr_debug("AF_BUS packet from a peer to a peer (unicast). ACCEPT.\n");
		ret = NF_ACCEPT;
		goto out;
	}

	err = dbus_message_parse(data, len, &msg);
	if (err) {
		if (!sendctx->main_recipient) {
			pr_debug("AF_BUS packet for an eavesdropper or " \
				 "multicast is not parsable. DROP.\n");
			ret = NF_DROP;
			goto out;
		} else if (sendctx->bus_master_side) {
			pr_debug("AF_BUS packet from bus master is not parsable. ACCEPT.\n");
			ret = NF_ACCEPT;
			goto out;
		} else {
			pr_debug("AF_BUS packet from peer is not parsable. DROP.\n");
			ret = NF_DROP;
			goto out;
		}
	}

	if (sendctx->bus_master_side && !sendctx->main_recipient) {
		pr_debug("AF_BUS packet '%s' from the bus master is for an " \
			 "eavesdropper. DROP.\n",
		       msg.member ? msg.member : "");
		ret = NF_DROP;
		goto out;
	}
	if (sendctx->bus_master_side) {
		if (msg.name_acquired) {
			pr_debug("New name: %s [%p %p].\n",
				 msg.name_acquired, sendctx->sender,
				 sendctx->recipient);

			sender = find_match_maker(sendctx->sender, true, false);
			bus_matchmaker_add_name(sender, msg.name_acquired,
						GFP_ATOMIC);
		}
		if (msg.name_lost) {
			pr_debug("Lost name: %s [%p %p].\n",
				 msg.name_lost, sendctx->sender,
				 sendctx->recipient);

			sender = find_match_maker(sendctx->sender, true, false);
			bus_matchmaker_remove_name(sender, msg.name_acquired);
		}

		pr_debug("AF_BUS packet '%s' from the bus master. ACCEPT.\n",
			 msg.member ? msg.member : "");
		ret = NF_ACCEPT;
		goto out;
	}

	pr_debug("Multicast AF_BUS packet, %ld bytes, " \
		 "considering recipient %lld...\n", len,
		 sendctx->recipient ? sendctx->recipient->sbus_addr.s_addr : 0);

	pr_debug("Message type %d %s->%s [iface: %s][member: %s][matchmaker=%p]...\n",
		 msg.type,
		 msg.sender ? msg.sender : "",
		 msg.destination ? msg.destination : "",
		 msg.interface ? msg.interface : "",
		 msg.member ? msg.member : "",
		 matchmaker);

	if (!matchmaker) {
		pr_debug("No match rules for this recipient. DROP.\n");
		ret = NF_DROP;
		goto out;
	}

	sender = find_match_maker(sendctx->sender, true, false);
	err = bus_matchmaker_filter(matchmaker, sender, sendctx->eavesdropper,
				    &msg);
	if (err) {
		pr_debug("Matchmaker: ACCEPT.\n");
		ret = NF_ACCEPT;
		goto out;
	} else {
		pr_debug("Matchmaker: DROP.\n");
		ret = NF_DROP;
		goto out;
	}

out:
	if (matchmaker)
		kref_put(&matchmaker->kref, bus_matchmaker_free);
	if (sender)
		kref_put(&sender->kref, bus_matchmaker_free);
	return ret;
}

/* Taken from drbd_nl_send_reply() */
static void nfdbus_nl_send_reply(struct cn_msg *msg, int ret_code)
{
	char buffer[sizeof(struct cn_msg)+sizeof(struct nfdbus_nl_cfg_reply)];
	struct cn_msg *cn_reply = (struct cn_msg *) buffer;
	struct nfdbus_nl_cfg_reply *reply =
		(struct nfdbus_nl_cfg_reply *)cn_reply->data;
	int rr;

	memset(buffer, 0, sizeof(buffer));
	cn_reply->id = msg->id;

	cn_reply->seq = msg->seq;
	cn_reply->ack = msg->ack  + 1;
	cn_reply->len = sizeof(struct nfdbus_nl_cfg_reply);
	cn_reply->flags = 0;

	reply->ret_code = ret_code;

	rr = cn_netlink_send(cn_reply, 0, GFP_NOIO);
	if (rr && rr != -ESRCH)
		pr_debug("nfdbus: cn_netlink_send()=%d\n", rr);
}

/**
 * nfdbus_check_perm - check if a pid is allowed to update match rules
 * @sockaddr_bus: the socket address of the bus
 * @pid: the process id that wants to update the match rules set
 *
 * Test if a given process id is allowed to update the match rules set
 * for this bus. Only the process that owns the bus master listen socket
 * is allowed to update the match rules set for the bus.
 */
static bool nfdbus_check_perm(struct sockaddr_bus *sbusname, pid_t pid)
{
	struct net *net = get_net_ns_by_pid(pid);
	struct sock *s;
	struct bus_address *addr;
	struct hlist_node *node;
	int offset = (sbusname->sbus_path[0] == '\0');
	int path_len = strnlen(sbusname->sbus_path + offset, BUS_PATH_MAX);
	int len;
	if (!net)
		return false;

	len = path_len + 1 + sizeof(__kernel_sa_family_t) +
	      sizeof(struct bus_addr);

	spin_lock(&bus_address_lock);

	hlist_for_each_entry(addr, node, &bus_address_table[hash],
			     table_node) {
		s = addr->sock;

		if (s->sk_protocol != BUS_PROTO_DBUS)
			continue;

		if (!net_eq(sock_net(s), net))
			continue;

		if (addr->len == len &&
		    addr->name->sbus_family == sbusname->sbus_family &&
		    addr->name->sbus_addr.s_addr == BUS_MASTER_ADDR &&
		    bus_same_bus(addr->name, sbusname) &&
		    pid_nr(s->sk_peer_pid) == pid) {
			spin_unlock(&bus_address_lock);
			return true;
		}
	}

	spin_unlock(&bus_address_lock);

	return false;
}

static void cn_cmd_cb(struct cn_msg *msg, struct netlink_skb_parms *nsp)
{
	struct nfdbus_nl_cfg_req *nlp = (struct nfdbus_nl_cfg_req *)msg->data;
	struct cn_msg *cn_reply;
	struct nfdbus_nl_cfg_reply *reply;
	int retcode, rr;
	pid_t pid = task_tgid_vnr(current);
	int reply_size = sizeof(struct cn_msg)
		+ sizeof(struct nfdbus_nl_cfg_reply);

	pr_debug("nfdbus: %s nsp->pid=%d pid=%d\n", __func__, nsp->pid, pid);

	if (!nfdbus_check_perm(&nlp->addr, pid)) {
		pr_debug(KERN_ERR "nfdbus: pid=%d is not allowed!\n", pid);
		retcode = EPERM;
		goto fail;
	}

	cn_reply = kzalloc(reply_size, GFP_KERNEL);
	if (!cn_reply) {
		retcode = ENOMEM;
		goto fail;
	}
	reply = (struct nfdbus_nl_cfg_reply *) cn_reply->data;

	if (msg->len < sizeof(struct nfdbus_nl_cfg_req)) {
		reply->ret_code = EINVAL;
	} else if (nlp->cmd == NFDBUS_CMD_ADDMATCH) {
		struct bus_match_rule *rule;
		struct bus_match_maker *matchmaker;
		reply->ret_code = 0;

		if (msg->len == 0)
			reply->ret_code = EINVAL;

		rule = bus_match_rule_parse(nlp->data, GFP_ATOMIC);
		if (rule) {
			matchmaker = find_match_maker(&nlp->addr, true, false);
			pr_debug("Add match rule for matchmaker %p\n",
				 matchmaker);
			bus_matchmaker_add_rule(matchmaker, rule);
			kref_put(&matchmaker->kref, bus_matchmaker_free);
		} else {
			reply->ret_code = EINVAL;
		}
	} else if (nlp->cmd == NFDBUS_CMD_REMOVEMATCH) {
		struct bus_match_rule *rule;
		struct bus_match_maker *matchmaker;

		rule = bus_match_rule_parse(nlp->data, GFP_ATOMIC);
		if (rule) {
			matchmaker = find_match_maker(&nlp->addr, false, false);
			if (!matchmaker) {
				reply->ret_code = EINVAL;
			} else {
				pr_debug("Remove match rule for matchmaker %p\n",
					 matchmaker);
				bus_matchmaker_remove_rule_by_value(matchmaker, rule);
				kref_put(&matchmaker->kref, bus_matchmaker_free);
				reply->ret_code = 0;
			}
			bus_match_rule_free(rule);
		}

	} else if (nlp->cmd == NFDBUS_CMD_REMOVEALLMATCH) {
		struct bus_match_maker *matchmaker;

		matchmaker = find_match_maker(&nlp->addr, false, true);
		if (!matchmaker) {
			reply->ret_code = EINVAL;
		} else {
			pr_debug("Remove matchmaker %p\n", matchmaker);
			kref_put(&matchmaker->kref, bus_matchmaker_free);
			kref_put(&matchmaker->kref, bus_matchmaker_free);
			reply->ret_code = 0;
		}

	} else {
		reply->ret_code = EINVAL;
	}

	cn_reply->id = msg->id;
	cn_reply->seq = msg->seq;
	cn_reply->ack = msg->ack + 1;
	cn_reply->len = sizeof(struct nfdbus_nl_cfg_reply);
	cn_reply->flags = 0;

	rr = cn_netlink_reply(cn_reply, nsp->pid, GFP_KERNEL);
	if (rr && rr != -ESRCH)
		pr_debug("nfdbus: cn_netlink_send()=%d\n", rr);
	pr_debug("nfdbus: cn_netlink_reply(pid=%d)=%d\n", nsp->pid, rr);

	kfree(cn_reply);
	return;
fail:
	nfdbus_nl_send_reply(msg, retcode);
}

static int __init nfdbus_init(void)
{
	int err;
	struct bus_addr master_addr;

	master_addr.s_addr = BUS_MASTER_ADDR;
	hash = bus_compute_hash(master_addr);

	pr_debug("Loading netfilter_dbus\n");

	/* Install D-Bus netfilter hook */
	nfho_dbus.hook     = dbus_filter;
	nfho_dbus.hooknum  = NF_BUS_SENDING;
	nfho_dbus.pf       = NFPROTO_BUS; /* Do not use PF_BUS, you fool! */
	nfho_dbus.priority = 0;
	nfho_dbus.owner = THIS_MODULE;
	err = nf_register_hook(&nfho_dbus);
	if (err)
		return err;
	pr_debug("Netfilter hook for D-Bus: installed.\n");

	/* Install connector hook */
	err = cn_add_callback(&cn_cmd_id, "nfdbus", cn_cmd_cb);
	if (err)
		goto err_cn_cmd_out;
	pr_debug("Connector hook: installed.\n");

	return 0;

err_cn_cmd_out:
	nf_unregister_hook(&nfho_dbus);

	return err;
}

static void __exit nfdbus_cleanup(void)
{
	int i;
	struct hlist_node *node, *tmp;
	struct bus_match_maker *matchmaker;
	nf_unregister_hook(&nfho_dbus);

	cn_del_callback(&cn_cmd_id);

	spin_lock(&matchrules_lock);
	for (i = 0; i < BUS_HASH_SIZE; i++) {
		hlist_for_each_entry_safe(matchmaker, node, tmp,
					  &matchrules_table[i], table_node) {
			hlist_del(&matchmaker->table_node);
			kref_put(&matchmaker->kref, bus_matchmaker_free);
		}
	}
	spin_unlock(&matchrules_lock);

	pr_debug("Unloading netfilter_dbus\n");
}

module_init(nfdbus_init);
module_exit(nfdbus_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS_NET_PF_PROTO(PF_BUS, BUS_PROTO_DBUS);
