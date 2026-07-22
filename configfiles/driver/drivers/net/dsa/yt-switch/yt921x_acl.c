// SPDX-License-Identifier: GPL-2.0-or-later
/* Stub ACL/mirror for kernel 6.1 compatibility */
#include "yt921x_internal.h"

int yt921x_mirror_add(struct yt921x_priv *priv, int port, bool ingress,
		      int to_local_port, struct netlink_ext_ack *extack)
{
	NL_SET_ERR_MSG_MOD(extack, "Port mirroring not supported on this build");
	return -EOPNOTSUPP;
}

int yt921x_mirror_del(struct yt921x_priv *priv, int port, bool ingress)
{
	return 0;
}
