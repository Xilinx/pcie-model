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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <linux/pci_regs.h>
#include "pcie_cfgutil.h"
#include "pcie_msix.h"
#include "pcie_api.h"
#include "pcie_internal.h"

static uint32_t get_cfg_16(const uint8_t *config_space, unsigned int addr)
{
  return config_space[addr] |
         (config_space[addr+1] << 8);
}

static inline bool offset_is_tbl(pcie_state_t *s,
                                 pcie_func_t *func, unsigned int offset)
{
  pcie_core_settings_t *c_settings = pcie_core_cb_get_pcie_core_settings(s);
  int tbl_off = c_settings->msix_tbl_off[func->rid];
  int pba_off = c_settings->msix_pba_off[func->rid];

  if (offset < tbl_off) return false;
  if (offset >= tbl_off + sizeof(func->msix.vectors)) return false;
  if (tbl_off < pba_off && offset >= pba_off) return false;
  return true;
}

static inline bool offset_is_pba(pcie_state_t *s,
                                 pcie_func_t *func, unsigned int offset)
{
  pcie_core_settings_t *c_settings = pcie_core_cb_get_pcie_core_settings(s);
  int tbl_off = c_settings->msix_tbl_off[func->rid];
  int pba_off = c_settings->msix_pba_off[func->rid];

  if (offset < pba_off) return false;
  if (offset >= pba_off + sizeof(func->msix.pba)) return false;
  if (pba_off < tbl_off && offset >= tbl_off) return false;
  return true;
}

/* Come out of reset with all vectors masked */
void pcie_hw_msix_init(pcie_func_t *func)
{
  const uint8_t msix = pcie_find_cap(PCI_CAP_ID_MSIX, func->config_space);
  uint16_t ctrl = get_cfg_16(func->config_space, msix + PCI_MSIX_FLAGS);
  int i;
  func->msix.num_vecs = 1 + (ctrl & PCI_MSIX_FLAGS_QSIZE);
  Xassert_le(func->msix.num_vecs, ARRAY_SIZE(func->msix.vectors));
  for (i = 0; i < func->msix.num_vecs; i++) {
    func->msix.vectors[i].vmask = 1;
  }
  func->msix.masked = false;
}

/* Lowest level of MSI-X dispatch - checks for masking have already been done.*/
static void emit_msix(pcie_state_t *state, pcie_func_t *func, unsigned int vecnum)
{
  pcie_msix_vec_t *vec = &func->msix.vectors[vecnum];
  uint64_t host_addr = ((uint64_t)vec->addr_hi << 32) | vec->addr_lo;
  fs_pcie_host_write(state, func->rid, host_addr, &vec->data, sizeof(vec->data));
}

static unsigned get_pba_val(pcie_func_t *func, unsigned int vecnum)
{
  unsigned entry = (vecnum >> 5);
  unsigned bit = (vecnum & 31);
  uint32_t dword = func->msix.pba[entry];
  return (dword >> bit) & 1;
}

static void set_pba_val(pcie_func_t *func, unsigned int vecnum, unsigned int val)
{
  unsigned entry = (vecnum >> 5);
  unsigned bit = (vecnum & 31);
  if (val)
    func->msix.pba[entry] |= (1 << bit);
  else
    func->msix.pba[entry] &= ~(1 << bit);
}


/* The MSI-X table behaves as normal R/W memory as far as host accesses are concerned. */
void pcie_hw_msix_w32(pcie_state_t *state, pcie_func_t *func, unsigned int offset, uint32_t data)
{
  if (offset_is_tbl(state, func, offset)) {
    pcie_core_settings_t *c_settings = pcie_core_cb_get_pcie_core_settings(state);
    unsigned int dword = (offset - c_settings->msix_tbl_off[func->rid]) >> 2;

    /* If it's a write to the vector control we have to handle unmask */
    if ((dword & 3) == 3) {
      data &= 1; /* Only 1 bit in the vector control word */
      if (!data) {
        unsigned int vector = dword >> 2;
        bool pending = get_pba_val(func, vector);
        if (pending) {
          emit_msix(state, func, vector);
          set_pba_val(func, vector, 0);
        }
      }
    }
    func->msix.vec_dwords[dword] = data;
  } else {
    fs_log("FN %d.%d: Dropping misguided write to offset 0x%x in MSI-X region\n", func->pf_num, func->vf_num, offset);
  }
}

uint32_t pcie_hw_msix_r32(pcie_state_t *state, pcie_func_t *func, unsigned int offset)
{
  uint32_t val;
  pcie_core_settings_t *c_settings = pcie_core_cb_get_pcie_core_settings(state);

  if (offset_is_tbl(state, func, offset)) {
    val = func->msix.vec_dwords[(offset - c_settings->msix_tbl_off[func->rid]) >> 2];
  } else if (offset_is_pba(state, func, offset)) {
    val = func->msix.pba[(offset - c_settings->msix_pba_off[func->rid]) >> 2];
  } else {
    fs_log("FN %d.%d: Faking misguided read from offset 0x%x in MSI-X region\n", func->pf_num, func->vf_num, offset);
    val = 0xdeadf00d;
  }
  return val;
}


void pcie_hw_msix_irq(pcie_state_t *state, pcie_func_t *func, unsigned int vecnum)
{
  //fs_log("***MSI-X: Raise IRQ %d for %d:%d\n", vecnum,  func->pf_num, func->vf_num);
  if (func->msix.masked || func->msix.vectors[vecnum].vmask) {
    set_pba_val(func, vecnum, 1);
  } else {
    emit_msix(state, func, vecnum);
  }
}

void pcie_msix_set_func_mask(pcie_state_t *pcistate, pcie_func_t *func, bool state)
{
  bool old = func->msix.masked;
  func->msix.masked = state;
  /* Just cleared. All the pending interrupts are sent. */
  if (old && !state) {
    unsigned int vec;
    for (vec = 0; vec < func->msix.num_vecs; vec++) {
      bool pending = get_pba_val(func, vec);
      if (pending) {
        emit_msix(pcistate, func, vec);
        set_pba_val(func, vec, 0);
      }
    }
  }
}


