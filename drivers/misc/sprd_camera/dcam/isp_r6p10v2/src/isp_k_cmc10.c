/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/uaccess.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_drv.h"

static int isp_k_cmc10_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_cmc10_info cmc10_info;

	memset(&cmc10_info, 0x00, sizeof(cmc10_info));

	ret = copy_from_user((void *)&cmc10_info, param->property_param,
			sizeof(cmc10_info));
	if (ret != 0) {
		pr_err("cmc10: copy error, ret=0x%x\n", (unsigned int)ret);
		return -EPERM;
	}

	ISP_REG_MWR(idx, ISP_CMC10_PARAM, BIT_0, cmc10_info.bypass);
	if (cmc10_info.bypass)
		return 0;

	val = ((cmc10_info.matrix.val[1] & 0x3FFF) << 14) |
		(cmc10_info.matrix.val[0] & 0x3FFF);
	ISP_REG_WR(idx, ISP_CMC10_MATRIX0, val);

	val = ((cmc10_info.matrix.val[3] & 0x3FFF) << 14) |
		(cmc10_info.matrix.val[2] & 0x3FFF);
	ISP_REG_WR(idx, (ISP_CMC10_MATRIX0 + 4), val);

	val = ((cmc10_info.matrix.val[5] & 0x3FFF) << 14) |
		(cmc10_info.matrix.val[4] & 0x3FFF);
	ISP_REG_WR(idx, (ISP_CMC10_MATRIX0 + 8), val);

	val = ((cmc10_info.matrix.val[7] & 0x3FFF) << 14) |
		(cmc10_info.matrix.val[6] & 0x3FFF);
	ISP_REG_WR(idx, (ISP_CMC10_MATRIX0 + 12), val);

	val = cmc10_info.matrix.val[8] & 0x3FFF;
	ISP_REG_WR(idx, (ISP_CMC10_MATRIX0 + 16), val);

	return ret;
}

int isp_k_cfg_cmc10(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_cmc: param is null error.\n");
		return -EPERM;
	}

	if (param->property_param == NULL) {
		pr_err("isp_k_cfg_cmc: property_param is null error.\n");
		return -EPERM;
	}

	switch (param->property) {
	case ISP_PRO_CMC_BLOCK:
		ret = isp_k_cmc10_block(param, idx);
		break;
	default:
		pr_err("cmc:cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}

