/*
 * dbg-phy-tps6598x driver
 *
 * Author:      Dan Murphy <dmurphy@ti.com>
 * Copyright (C) 2017 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBG_PHY_TPS6598X_H
#define _DBG_PHY_TPS6598X_H

#include "tps6598x.h"

struct dbg_tps6598x_reg {
	const char *name;
	uint8_t reg;
} tps6598x_regs[] = {
	{ "VID", TPS6598X_VID },
	{ "DID", TPS6598X_DID },
	{ "MODE", TPS6598X_MODE },
	{ "TYPE", TPS6598X_TYPE },
	{ "PROTO_VER", TPS6598X_PROTO_VER },
	{ "UID", TPS6598X_UID },
	{ "VER", TPS6598X_VERSION },
	{ "EVENT1", TPS6598X_INT_EVENT_1 },
	{ "EVENT2", TPS6598X_INT_EVENT_2 },
	{ "MASK1", TPS6598X_INT_MASK_1 },
	{ "MASK2", TPS6598X_INT_MASK_2 },
	{ "INT_CLEAR_1", TPS6598X_INT_CLEAR_1 },
	{ "INT_CLEAR_2", TPS6598X_INT_CLEAR_2 },
	{ "STATUS", TPS6598X_STATUS },
	{ "PWR_STATUS", TPS6598X_PWR_STATUS },
};


#endif
