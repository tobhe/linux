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
#include "mvx_dsm.h"

#define EVENT_ERROR  0

#undef DSM_ERROR_ID_DEF
#define DSM_ERROR_ID_DEF(name, subname, value) \
        { .error_id = DSM_##name##_##subname, .error_name = #subname }

static dsm_error_word mvx_error_words[] = { DSM_VPU_ERR_LIST };

static dsm_error_map mvx_error_map = {
        .num = ARRAY_SIZE(mvx_error_words),
        .words = mvx_error_words,
};

struct dsm_dev mvx_dsm_dev = {
        .name = "MVX",
        .device_name = NULL,
        .ic_name = NULL,
        .module_name = NULL,
        .fops = NULL,
        .error_map = &mvx_error_map,
        .buff_size = 1000,
};

struct dsm_client *mvx_dsm_init(void)
{
    struct dsm_client *dc = dsm_register_client(&mvx_dsm_dev);
    if (!dc)
        pr_err("Failed to register MVX dsm device");
    return dc;
}

int mvx_dsm_remove(struct dsm_client *dsm_client)
{
    if (dsm_client) {
        dsm_unregister_client(dsm_client, &mvx_dsm_dev);
    }

    return 0;
}

void dsm_send_event(struct dsm_client *dsm_client, char *msg)
{
    if (!dsm_client)
        return;

    if (!dsm_client_ocuppy(dsm_client)) {
        dsm_client_record(dsm_client, msg);
        dsm_client_notify(dsm_client, mvx_error_words[EVENT_ERROR].error_id);
    }
}