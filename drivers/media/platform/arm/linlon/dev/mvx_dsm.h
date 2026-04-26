/*
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Cix Technology Group Co., Ltd.
 *
 *            (C) Copyright 2025 Cix Technology Group Co., Ltd.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from Cix Technology Group Co., Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */
#ifndef __MVX_DSM_H
#define __MVX_DSM_H

#ifdef CONFIG_PLAT_DSM_SYSEVENT

#include <linux/soc/cix/dsm_pub.h>

struct dsm_client *mvx_dsm_init(void);
int mvx_dsm_remove(struct dsm_client *dsm_client);
void dsm_send_event(struct dsm_client *dsm_client, char *msg);
#else
static inline struct dsm_client *mvx_dsm_init(void)
{
    return NULL;
}

static inline int mvx_dsm_remove(struct dsm_client *dsm_client)
{
    return 0;
}
static void dsm_send_event(struct dsm_client *dsm_client, char *msg)
{
    return;
}
#endif // !CONFIG_PLAT_DSM_SYSEVENT

#endif
