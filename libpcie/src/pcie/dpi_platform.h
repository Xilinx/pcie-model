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

#ifndef DPI_PLATFORM_H
#define DPI_PLATFORM_H

#include "cosim_common.h"

extern int pcie_cleanup(void);

extern void cosim_pcie_accept_buffer(void *opaque,
                                     cosim_buffer_head *buffer);

extern void cosim_pci_reset_accept_buffer(void *opaque,
                                     cosim_buffer_head *buffer);

extern cosim_client *cosim_get_tlp_client(cosim_platform_state_t *state);

extern cosim_client *cosim_get_pci_reset_client(cosim_platform_state_t *state);

/* Exit the simulation with the given code. */
extern void cosim_exit(uint32_t code);
/* Notify that an instance has closed. If that was the
   last one, exit the simulation with the given code. */
extern void cosim_finish(cosim_platform_state_t *state, uint32_t code);

#define DPI_STRGETTIME_BUFLEN 32

extern unsigned long long dpi_get_time_rate(void);
extern int dpi_get_time_rate_log10(void);
extern const char* dpi_strgettime(char *buf);

extern void cosim_dpi_init(void *context, int instance, cosim_platform_state_t **outstate,
                           void(*post_poll_hook)(void*), void *hook_context);
#endif /* DPI_PLATFORM_H */


