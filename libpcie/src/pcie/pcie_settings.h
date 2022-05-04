/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Pavan Prerepa,
 *            Charlie Palmer,
 *            Jackson Rigby,
 *            Konstantin Ushakov,
 *            Paul Burton
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

#ifndef PCIE_SETTINGS_H
#define PCIE_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include <pcie/pcie_api.h>

/* Stores a single location in config space to fix up, and a set of
 * PFs to which it applies. The location is either absolute (cap == 0)
 * or relative to the start of a capability. */
typedef struct {
  uint16_t cap;
  uint16_t addr;
  uint32_t mask;
  uint32_t val;
  uint32_t pfs;
} cfgspc_fixup_t;

#define ALL_PFS ((uint32_t)-1)

#define CFGSPC_FIXUP_PFS(_addr, _val, _size, _pfs) \
{ \
  .cap = 0, \
  .addr = (_addr & 0xfffc), \
  .val = (_val << (8 * (_addr & 3))), \
  .mask = (~(((1 << _size) - 1) << (8 * (_addr & 3)))), \
  .pfs = _pfs \
}

#define CFGSPC_FIXUP(_addr, _val, _size) CFGSPC_FIXUP_PFS(_addr, _val, _size, ALL_PFS)

#define CFGSPC_FIXUP_CAP_PFS(_cap, _offs, _val, _size, _pfs) \
{ \
  .cap = (_cap | 0x8000), \
  .addr = (_offs & 0xfffc), \
  .val = (_val << (8 * (_offs & 3))), \
  .mask = ((_size == 32) ? 0 : ((~(((1U << _size) - 1) << (8 * (_offs & 3)))))), \
  .pfs = _pfs \
}

#define CFGSPC_FIXUP_XCAP_PFS(_cap, _offs, _val, _size, _pfs) \
{ \
  .cap = _cap, \
  .addr = (_offs & 0xfffc), \
  .val = (_val << (8 * (_offs & 3))), \
  .mask = ((_size == 32) ? 0 : ((~(((1U << _size) - 1) << (8 * (_offs & 3)))))), \
  .pfs = _pfs \
}

#define CFGSPC_FIXUP_SPARE { .addr = 0xfffe }
#define CFGSPC_FIXUP_IS_SPARE(f_) (f_->addr == 0xfffe)
#define CFGSPC_FIXUPS_END { .addr = 0xffff }
#define CFGSPC_FIXUP_IS_END(f_) (f_->addr == 0xffff)

#define CFGSPC_MAX_FIXUPS (32)

typedef struct {
  int bar;
  uint32_t mask;
  uint32_t flags;
  uint32_t pf_mask;
} barmask_fixup_t;

#define BARMASK_FIXUPS_END { .bar = -1 }
#define BARMASK_FIXUP_IS_END(f_) (f_->bar == -1)


#define BARMASK_MAX_FIXUPS (10)

/* Extended capabilities to add to config space */
typedef struct {
  uint16_t id;
  uint8_t ver;
  uint8_t *data;
  int datalen;
} xcap_addition_t;

#define XCAP_ADDITIONS_END { .id = 0 }
#define XCAP_ADDITION_IS_END(x_) (x_->id == 0)

#define XCAP_MAX_ADDITIONS (4)

/* Capabilities to add to config space */
typedef struct {
  uint8_t *data;
  int datalen;
  uint8_t id;
} cap_addition_t;

#define CAP_ADDITIONS_END { .id = 0 }
#define CAP_ADDITION_IS_END(x_) (x_->id == 0)

typedef struct pcie_cfgspc_callbacks_s {
  /*
   * Configuration callbacks
   */
  void (*fill_reset_values)(pcie_state_t *s,
                            unsigned int pf, unsigned int vf,
                            uint8_t *config_space);

  /*
   * Called when the host writes the address register of the VPD cap. We get
   * given the function the write was sent to, the position of the cap in
   * config space, and the value written to the address register. When the
   * function returns, it has updated the address register and the caller will
   * not perform a normal write to config space.
   */
  void (*trigger_vpd)(pcie_func_t *func, uint16_t cap_pos, uint16_t addr);
  void (*trigger_flr)(struct mc_s *mc_in, pcie_func_t *func);

  /* The chip-level model knows which PF BAR maps what functionality */
  pci_bar_type_t (*bar_num_to_type)(pcie_state_t *s, int fn, pci_bar_num_t bar);

  void (*exprom_init)(pcie_func_t *fs_func);
  void (*dpa_ctrl_notification)(struct fs_hw_s *hw, int pf);
  void (*set_pf_bme)(struct fs_hw_s *hw, uint32_t pf, uint32_t enabled);
  void (*set_vf_bme)(struct fs_hw_s *hw, uint32_t vf, uint32_t enabled);

  pcie_func_t *(*new_pcie_func)(void *hw);
  void (*free_pcie_func)(pcie_func_t *pfunc);

  /* This gets called when the VF is enabled */
  void (*pcie_apply_app_vf_hacks)(pcie_state_t *state, int parent_pf, int vf,
                                  uint8_t *config_space);

} pcie_cfgspc_callbacks_t;

typedef struct pcie_settings_s {
  int max_pfs;
  int max_vfs;
  int max_total_funcs;
  bool initial_cfg_retry;
  bool process_tlp_once;
  bool pseudo_rc_enabled;
  bool no_pcie_cap;
  tag_type_t tag_type;
  cfgspc_fixup_t pf_cfgspc_fixups[CFGSPC_MAX_FIXUPS];
  barmask_fixup_t pf_barmask_fixups[BARMASK_MAX_FIXUPS];
  cfgspc_fixup_t vf_cfgspc_fixups[CFGSPC_MAX_FIXUPS];
  barmask_fixup_t vf_barmask_fixups[BARMASK_MAX_FIXUPS];
  xcap_addition_t pf_new_xcaps[XCAP_MAX_ADDITIONS];
  xcap_addition_t vf_new_xcaps[XCAP_MAX_ADDITIONS];
  cap_addition_t *pf_new_caps;
  cap_addition_t *vf_new_caps;
  pcie_cfgspc_callbacks_t *cfg_cbs;
  pcie_core_callbacks_t *core_cbs;
} pcie_settings_t;

#endif
