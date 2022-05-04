/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Konstantin Ushakov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef COSIM_INTERNAL_H
#define COSIM_INTERNAL_H

#include <stdarg.h>
#include "cosim_common.h"
#include "sfc_cosim_comms.h"

typedef struct cosim_hub cosim_hub;

extern void cosim_hub_set_tracing(struct cosim_hub *hub, int val);

extern void ch_vtrace(const struct cosim_hub *hub, const char *fmt,
                      va_list args);

extern void cosim_hub_dispatch_packet(cosim_hub *hub,
                                      cosim_buffer_head *packet);

extern cosim_hub *cosim_socket_get_hub(cosim_socket *socket);

extern cosim_hub *cosim_hub_new(void);
extern void cosim_hub_delete(cosim_hub *hub);

#endif
