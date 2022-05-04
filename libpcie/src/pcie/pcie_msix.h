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

#ifndef PCIE_MSIX_H
#define PCIE_MSIX_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_MSIX_VECS (2048)

typedef struct pcie_msix_vec_s {
  uint32_t addr_lo;
  uint32_t addr_hi;
  uint32_t data;
  uint32_t vmask;
} pcie_msix_vec_t;

typedef struct pcie_msix_state_s {
  union {
    pcie_msix_vec_t vectors[MAX_MSIX_VECS];
    uint32_t vec_dwords[4 * MAX_MSIX_VECS];
  };
  uint32_t pba[64];
  uint16_t num_vecs;
  bool masked;
} pcie_msix_state_t;

struct pcie_func_s;
struct pcie_state_s;

extern void pcie_msix_set_func_mask(struct pcie_state_s *pcistate,  struct pcie_func_s *func, bool state);

extern void pcie_hw_msix_init(struct pcie_func_s *fs_func);

extern void pcie_hw_msix_w32(struct pcie_state_s *state,  struct pcie_func_s *func, unsigned int offset, uint32_t data);

extern uint32_t pcie_hw_msix_r32(struct pcie_state_s *state, struct pcie_func_s *func, unsigned int offset);


extern void pcie_hw_msix_irq(struct pcie_state_s *state, struct pcie_func_s *func, unsigned int vecnum);

#endif
