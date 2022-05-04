/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Boleslav Stankevich,
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

#ifndef PCIE_CFGUTIL_H
#define PCIE_CFGUTIL_H

#include <stdint.h>
#include "cosim_utils.h"

#define PCI_CFG_SPACE_EXP_SIZE 4096

/* Returns the offset into config space of the requested cap */
extern uint8_t pcie_find_cap(unsigned cap, const uint8_t *cfgspc);

/* Returns the offset into config space of the requested xcap */
extern uint16_t pcie_find_xcap(unsigned cap, const uint8_t *cfgspc);

/* Hides the specified capability by editing the cap chain. Asserts
 * if the cap is not found. */
extern void pcie_hide_cap(unsigned cap, uint8_t *cfgspc);

extern void pcie_hide_xcap(unsigned cap, uint8_t *cfgspc);

/* Prepend a new extended capability to the list. The caller passes in
 * cfgspc: the config space array to patch
 * capid:  the ID of the new cap
 * data, len: an opaque blob of data to follow the cap header
 */
extern void pcie_prepend_cap(uint8_t *cfgspc, uint8_t capid, const uint8_t *data, unsigned len);

/* Append a new extended capability to the list. The caller passes in
 * cfgspc: the config space array to patch
 * capid:  the ID of the new cap
 * ver:    the version of the new cap
 * data, len: an opaque blob of data to follow the xcap header
 * NOTE: this will typically use up more of config space than required
 * since it uses an upper bound for the size of the current last
 * xcap in the chain.
 */
extern void pcie_append_xcap(uint8_t *cfgspc, uint16_t capid, uint8_t ver, const uint8_t *data, unsigned len);

/* Pack the fields and return the header */
extern uint32_t pcie_make_xcap_hdr(uint16_t capid, uint8_t ver, uint16_t next);

typedef void (*pci_cap_cb)(uint8_t *cfgspc, uint16_t offset, void *context);

extern void pcie_walk_caps(uint8_t *cfgspc, pci_cap_cb cb, void *cb_context);

extern void pcie_walk_xcaps(uint8_t *cfgspc, pci_cap_cb cb, void *cb_context);

/* Save everyone writing this bit of boilerplate */
static inline uint8_t pcie_get_cfg_8(const uint8_t *config_space, unsigned int addr)
{
  return config_space[addr];
}

static inline uint32_t pcie_get_cfg_32(const uint8_t *config_space, unsigned int addr)
{
  return config_space[addr] |
         (config_space[addr+1] << 8) |
         (config_space[addr+2] << 16) |
         (config_space[addr+3] << 24);
}

static inline void pcie_put_cfg_8(uint8_t *config_space, unsigned int addr,
                      uint8_t val)
{
  config_space[addr] = val;
}

static inline void pcie_put_cfg_16(uint8_t *config_space,
                                   unsigned int addr,
                                   uint16_t val)
{
  config_space[addr] = val;
  config_space[addr + 1] = val >> 8;
}

static inline void pcie_put_cfg_32(uint8_t *config_space, unsigned int addr, uint32_t val)
{
  config_space[addr] = val & 0xff;
  config_space[addr+1] = val >> 8;
  config_space[addr+2] = val >> 16;
  config_space[addr+3] = val >> 24;
}

void pcie_cfgspc_fixup_write_mask(uint8_t *config_space, uint32_t *write_mask);

bool pcie_cfgspc_verify_write_access(uint32_t *write_mask,
                                     unsigned int addr,
                                     unsigned int bits,
                                     bool is_vf,
                                     bool from_mc);

void pcie_cfgspc_fill_ari_next(uint8_t *config_space, uint8_t next_devfn);
#endif
