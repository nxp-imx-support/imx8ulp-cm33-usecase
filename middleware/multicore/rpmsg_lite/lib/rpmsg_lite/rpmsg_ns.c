/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * Copyright (c) 2015 Xilinx, Inc.
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rpmsg_lite.h"
#include "rpmsg_ns.h"
#include <stdint.h>

#define RL_NS_NAME_SIZE (64)

/*!
 * struct rpmsg_ns_msg - dynamic name service announcement message
 * @name: name of remote service that is published
 * @addr: address of remote service that is published
 * @flags: indicates whether service is created or destroyed
 *
 * This message is sent across to publish a new service, or announce
 * about its removal. When we receive these messages, an appropriate
 * rpmsg channel (i.e device) is created/destroyed. In turn, the ->probe()
 * or ->remove() handler of the appropriate rpmsg driver will be invoked
 * (if/as-soon-as one is registered).
 */
RL_PACKED_BEGIN
struct rpmsg_ns_msg
{
    char name[RL_NS_NAME_SIZE];
    uint32_t addr;
    uint32_t flags;
} RL_PACKED_END;

// Encode 8-bit data into 12-bit Hamming code (stored in a short)
short hamming_encode_12_8(char data) {
    short codeword = 0;
    int d[8], p1, p2, p4, p8;

    // Extract individual bits from the input data
    for (int i = 0; i < 8; i++) {
        d[i] = (data >> i) & 1;
    }

    // Place data bits into codeword at positions 3,5,6,7,9,10,11,12
    if (d[0]) codeword |= (1 << 2);
    if (d[1]) codeword |= (1 << 4);
    if (d[2]) codeword |= (1 << 5);
    if (d[3]) codeword |= (1 << 6);
    if (d[4]) codeword |= (1 << 8);
    if (d[5]) codeword |= (1 << 9);
    if (d[6]) codeword |= (1 << 10);
    if (d[7]) codeword |= (1 << 11);

    // Calculate parity bits for positions 1,2,4,8
    p1 = ((codeword >> 2) & 1) ^ ((codeword >> 4) & 1) ^ ((codeword >> 6) & 1) ^ ((codeword >> 8) & 1) ^ ((codeword >> 10) & 1);
    p2 = ((codeword >> 2) & 1) ^ ((codeword >> 5) & 1) ^ ((codeword >> 6) & 1) ^ ((codeword >> 9) & 1) ^ ((codeword >> 10) & 1);
    p4 = ((codeword >> 4) & 1) ^ ((codeword >> 5) & 1) ^ ((codeword >> 6) & 1) ^ ((codeword >> 11) & 1);
    p8 = ((codeword >> 8) & 1) ^ ((codeword >> 9) & 1) ^ ((codeword >> 10) & 1) ^ ((codeword >> 11) & 1);

    // Set parity bits in codeword
    if (p1) codeword |= (1 << 0);
    if (p2) codeword |= (1 << 1);
    if (p4) codeword |= (1 << 3);
    if (p8) codeword |= (1 << 7);

    return codeword;
}

// Decode 12-bit Hamming codeword into original 8-bit data
char hamming_decode_12_8(short codeword, int *error_detected) {
    // Calculate syndrome bits to detect error position
    int s1 = ((codeword >> 0) & 1) ^ ((codeword >> 2) & 1) ^ ((codeword >> 4) & 1) ^ ((codeword >> 6) & 1) ^ ((codeword >> 8) & 1) ^ ((codeword >> 10) & 1);
    int s2 = ((codeword >> 1) & 1) ^ ((codeword >> 2) & 1) ^ ((codeword >> 5) & 1) ^ ((codeword >> 6) & 1) ^ ((codeword >> 9) & 1) ^ ((codeword >> 10) & 1);
    int s4 = ((codeword >> 3) & 1) ^ ((codeword >> 4) & 1) ^ ((codeword >> 5) & 1) ^ ((codeword >> 6) & 1) ^ ((codeword >> 11) & 1);
    int s8 = ((codeword >> 7) & 1) ^ ((codeword >> 8) & 1) ^ ((codeword >> 9) & 1) ^ ((codeword >> 10) & 1) ^ ((codeword >> 11) & 1);

    int error_pos = s8 * 8 + s4 * 4 + s2 * 2 + s1;

	char data = 0;

    // If error detected, correct the bit
    if (error_pos != 0 && error_pos <= 12) {
        if (error_detected != NULL) {
			*error_detected = error_pos;
		}
        codeword ^= (1 << (error_pos - 1));
    } else {
		if (error_detected != NULL) {
			*error_detected = 0;
		}
	}

    // Extract original data bits from codeword
    if ((codeword >> 2) & 1) data |= (1 << 0);
    if ((codeword >> 4) & 1) data |= (1 << 1);
    if ((codeword >> 5) & 1) data |= (1 << 2);
    if ((codeword >> 6) & 1) data |= (1 << 3);
    if ((codeword >> 8) & 1) data |= (1 << 4);
    if ((codeword >> 9) & 1) data |= (1 << 5);
    if ((codeword >> 10) & 1) data |= (1 << 6);
    if ((codeword >> 11) & 1) data |= (1 << 7);

    return data;
}

/*!
 * @brief
 * Nameservice callback, called in interrupt context
 *
 * @param payload     Pointer to the buffer containing received data
 * @param payload_len Size of data received, in bytes
 * @param src         Pointer to address of the endpoint from which data is received
 * @param priv        Private data provided during endpoint creation
 *
 * @return  RL_RELEASE, message is always freed
 *
 */
static int32_t rpmsg_ns_rx_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    struct rpmsg_ns_msg *ns_msg_ptr        = payload;
    struct rpmsg_ns_callback_data *cb_ctxt = priv;
    RL_ASSERT(priv != RL_NULL);
    RL_ASSERT(cb_ctxt->cb != RL_NULL);

    /* Drop likely bad messages received at nameservice address */
    if (payload_len == sizeof(struct rpmsg_ns_msg))
    {
        cb_ctxt->cb(ns_msg_ptr->addr, ns_msg_ptr->name, ns_msg_ptr->flags, cb_ctxt->user_data);
    }

    return RL_RELEASE;
}

#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
rpmsg_ns_handle rpmsg_ns_bind(struct rpmsg_lite_instance *rpmsg_lite_dev,
                              rpmsg_ns_new_ept_cb app_cb,
                              void *user_data,
                              rpmsg_ns_static_context *ns_ept_ctxt)
#else
rpmsg_ns_handle rpmsg_ns_bind(struct rpmsg_lite_instance *rpmsg_lite_dev, rpmsg_ns_new_ept_cb app_cb, void *user_data)
#endif /* RL_USE_STATIC_API */
{
    struct rpmsg_ns_context *ns_ctxt;

    if (app_cb == RL_NULL)
    {
        return RL_NULL;
    }

#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    if (ns_ept_ctxt == RL_NULL)
    {
        return RL_NULL;
    }

    ns_ctxt = &ns_ept_ctxt->ns_ctxt;

    /* Set-up the nameservice callback context */
    ns_ept_ctxt->cb_ctxt.user_data = user_data;
    ns_ept_ctxt->cb_ctxt.cb        = app_cb;

    ns_ctxt->cb_ctxt = &ns_ept_ctxt->cb_ctxt;

    ns_ctxt->ept = rpmsg_lite_create_ept(rpmsg_lite_dev, RL_NS_EPT_ADDR, rpmsg_ns_rx_cb, (void *)ns_ctxt->cb_ctxt,
                                         &ns_ept_ctxt->ept_ctxt);
#else
    {
        struct rpmsg_ns_callback_data *cb_ctxt;

        cb_ctxt = env_allocate_memory(sizeof(struct rpmsg_ns_callback_data));
        if (cb_ctxt == RL_NULL)
        {
            return RL_NULL;
        }
        ns_ctxt = env_allocate_memory(sizeof(struct rpmsg_ns_context));
        if (ns_ctxt == RL_NULL)
        {
            env_free_memory(cb_ctxt);
            return RL_NULL;
        }

        /* Set-up the nameservice callback context */
        cb_ctxt->user_data = user_data;
        cb_ctxt->cb        = app_cb;

        ns_ctxt->cb_ctxt = cb_ctxt;

        ns_ctxt->ept = rpmsg_lite_create_ept(rpmsg_lite_dev, RL_NS_EPT_ADDR, rpmsg_ns_rx_cb, (void *)ns_ctxt->cb_ctxt);
    }
#endif /* RL_USE_STATIC_API */

    return (rpmsg_ns_handle)ns_ctxt;
}

int32_t rpmsg_ns_unbind(struct rpmsg_lite_instance *rpmsg_lite_dev, rpmsg_ns_handle handle)
{
    struct rpmsg_ns_context *ns_ctxt = (struct rpmsg_ns_context *)handle;

#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    return rpmsg_lite_destroy_ept(rpmsg_lite_dev, ns_ctxt->ept);
#else
    {
        int32_t retval;

        retval = rpmsg_lite_destroy_ept(rpmsg_lite_dev, ns_ctxt->ept);
        env_free_memory(ns_ctxt->cb_ctxt);
        env_free_memory(ns_ctxt);
        return retval;
    }
#endif
}

int32_t rpmsg_ns_announce(struct rpmsg_lite_instance *rpmsg_lite_dev,
                          struct rpmsg_lite_endpoint *new_ept,
                          const char *ept_name,
                          uint32_t flags)
{
    struct rpmsg_ns_msg ns_msg;
    int16_t  *p_encoded_name = ns_msg.name;

    if (ept_name == RL_NULL)
    {
        return RL_ERR_PARAM;
    }

    if (new_ept == RL_NULL)
    {
        return RL_ERR_PARAM;
    }

    for (int32_t i = 0; i < RL_NS_NAME_SIZE / 2; i++) {
        *p_encoded_name = hamming_encode_12_8(ept_name[i]);
        p_encoded_name++;
    }

    // env_strncpy(ns_msg.name, ept_name, RL_NS_NAME_SIZE);
    ns_msg.flags = flags;
    ns_msg.addr  = new_ept->addr;

    return rpmsg_lite_send(rpmsg_lite_dev, new_ept, RL_NS_EPT_ADDR, (char *)&ns_msg, sizeof(struct rpmsg_ns_msg),
                           RL_BLOCK);
}
