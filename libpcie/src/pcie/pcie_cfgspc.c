/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Charlie Palmer,
 *            Pavan Prerepa,
 *            Boleslav Stankevich,
 *            Ilya Repko,
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

/* PCI
 * ~~~
 *
 * This model pretends to provide up to the maximum number of PFs and VFs on a
 * 256-function ARI bus.
 *
 * It is presented to snapper by providing our own implementations of the
 * libpci pci_XXX() functions, and the driver/ul/linux_bar.c BAR mapping
 * library efx_bar_XXX() and efx_dev_XXX() functions, and intercepting
 * readl()/writel() (see fs_build.h); and to mcsim by providing hooks for PCIe
 * management (formerly known as DBI) config space read/write.
 *
 * The two main data structures are fs_pci_core_resource_t, which represents
 * the state held internally by the PCI core (some configuration space, and
 * a PF/VF assignment), and fs_pci_lies_t, which represents the hardware
 * state exposed by a function on the bus to the external world.
 *
 * The core resources for PFs are a simple array.  The core resources for VFs
 * are held in a linked list, and are allocated to or freed from a parent PF
 * as the SRIOV capability in the PF is programmed.  For convenience, a mapping
 * from (PF,VF) to core resource is maintained using the same indexing scheme
 * as the PFVF2VI table in the BIU.
 *
 * The external lies for PFs are instantiated at the point where the MC
 * releases the PCIe configuration retry GPIO.  The external lies for VFs are
 * instantiated, and may be destroyed again later, as VFs are enabled and 
 * disabled via the VF Enable flag in the SRIOV capability.
 *
 * Configuration space is constructed mainly from the reset values provided by
 * the lib/efhwdef library -- although unfortunately this is a bit broken for
 * VFs and needs fixing up.
 */

/* NOTE that although the source code is used in more than one tool (frankington, rhsim, ...),
 * it needs to be built once per tool because knowledge of what 
 * capabilities exist and where they are in config space gets baked in at 
 * compile-time. Given the current build system for the tools this isn't
 * a problem, but any attempt to change that should bear this in mind */ 

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include <linux/pci_regs.h>
#ifndef PCI_EXT_CAP_ID_DPA
#define PCI_EXT_CAP_ID_DPA 0x16
#endif
#ifdef CONFIG_TLM
  typedef struct fs_hw_s fs_hw_t;
  typedef struct mc_s mc_t;
#else
  #include <mc/dbi_hw_defines.h>
#endif
#include "cosim_utils.h"
#include "pcie/pcie_settings.h"
#include "pcie/pcie_cfgutil.h"
#include "pcie/pcie_msix.h"
#include "pcie/pcie_internal.h"

#ifndef PCI_MSIX_TABLE_BIR
#define PCI_MSIX_TABLE_BIR 0x00000007
#define PCI_MSIX_TABLE_OFFSET 0xfffffff8
#define PCI_MSIX_PBA_BIR   0x00000007
#define PCI_MSIX_PBA_OFFSET 0xfffffff8
#endif

/* Debugging control */
#define FS_PCI_DEBUGGING_ENABLED 0

#if FS_PCI_DEBUGGING_ENABLED
#define FS_PCI_DEBUG(args...) fs_log(args)
#else
#define FS_PCI_DEBUG(args...) \
  do {                        \
    if (0)                    \
      fs_log(args);           \
  } while (0)
#endif

#define SRIOV_CAP_OFFSET_BAR(x) (PCI_SRIOV_BAR + x * 4)

/* Magic values for various fields of fs_core_resource_t:
 */
#define PF_NONE  ((unsigned int)-1)
#define RID_NONE ((unsigned int)-1)

/* This structure represents the information about a device that we present
 * to snapper.  None of this stuff is available until the layout of the bus
 * is known.
 */
typedef struct {
  uint64_t start;
  uint64_t end;
  struct fs_pci_lies_s *owner;
  pci_bar_type_t type;
  pci_bar_num_t num;
} pci_bar_t;

/* If bar->start is set to this it will never match any host access (using
 * 0 is convenient because it automatically means that BARs part way through
 * setup do not get matched
 */
#define BAR_NEVER_MATCH (0)

/* This structure represents the internal PCI core resources owned by a PF or
 * VF (but not any associated hardware outside the core).  This information is
 * accessible via the PCIe management interface before the bus structures are
 * set up.
 */
typedef struct fs_pci_core_resource_s {
  struct fs_pci_bus_resource_s *br;
  bool visible;
  unsigned int pf;
  unsigned int vf;
  unsigned int rid;
  struct fs_pci_core_resource_s *next_vf;
  uint8_t config_space[0x1000];
  uint32_t write_mask[PCI_CFG_SPACE_EXP_SIZE/sizeof(uint32_t)];
  uint32_t bar_mask[PCI_NUM_BARS];      /* [6] and [7] unused, [8] is expansion ROM BAR, 9 - 14 are SRIOV BARS */
  uint32_t region_flags[PCI_NUM_BARS];  /* As for bar_mask */
} fs_pci_core_resource_t;

struct pciedev {
  uint8_t bus;
  uint8_t dev;
  uint8_t func;
};

typedef struct fs_pci_lies_s {
  pcie_func_t *func;
  struct pciedev dev;
  pci_bar_t bars[PCI_NUM_BARS];
  fs_pci_core_resource_t *core;
} fs_pci_lies_t;

/* This structure represents a location on the PCI bus.
 */
typedef struct fs_pci_bus_resource_s {
  fs_pci_core_resource_t *core;
  fs_pci_lies_t *lies;
} fs_pci_bus_resource_t;

/* Types of configuration space access.
 */
typedef enum fs_pci_cfg_access_type_e {
  CFG_ACCESS_HOST,
  CFG_ACCESS_MC,
  CFG_ACCESS_MC_CS2
} fs_pci_cfg_access_type_t;



static void create_pfs(pcie_state_t *state);

/* A fs_pci_addr_space_t contains an array of
 * BARs sorted by address. Memory and I/O BARs
 * must be placed in separate addr_spaces.
 * 'configured_bars' tracks the number of populated
 * entries in 'bars', all of which are at the start
 * (so 'configured_bars' is also the index of the
 * first unused slot).
 */
typedef struct fs_pci_addr_space_s {
  pci_bar_t **bar;
  int configured_bars;
} fs_pci_addr_space_t;

#define MAX_APERTURES_PER_FUNC (6)


/* Register offsets from dpa_cap */
#define DPA_STATUS_CNTRL_REG (0xc)
#define DPA_SUBSTATE_CONTROL_LBN 16
#define DPA_SUBSTATE_CONTROL_WIDTH 5


typedef struct pcie_cfgspc_state_s {
  const pcie_settings_t *settings;
  /* Resources */
  fs_pci_core_resource_t *pf_resources;
  fs_pci_core_resource_t *vf_resources;
  /* Unallocated VF resources live on this linked list.*/
  fs_pci_core_resource_t *vf_resource_free_list;
  /* This maps an encoding of a (PF, VF) tuple to a core resource structure.  The
  * encoding is the same one used inside the BIU for the PFVF2VI table; see
  * map_pfvf_to_core_resource() below.
  */
  fs_pci_core_resource_t *pfvf_to_core_resource_map[1 << 11];
  /* This represents the state (the associated internal core resource, and
  * external lies) of each possible location on the bus.
  */
  fs_pci_bus_resource_t *bus_state;

  fs_pci_addr_space_t mem_addr_space;
  fs_pci_addr_space_t io_addr_space;
  /* These are just the offsets into config space of their selected caps,
   * cached for convenience, or 0 if not present. */
  uint16_t dpa_cap;
  uint16_t vpd_cap;
  uint16_t * msix_cap; /* Gets an array, one for each RID */
  uint16_t vsec_cap;
} pcie_cfgspc_state_t;

/* ----------------------------------------------------------------------------
 * PCI core management model
 * ----------------------------------------------------------------------------
 */

/* Set or clear the mapping from (PF, VF) to a core resource.
 */
static void map_pfvf_to_core_resource(pcie_cfgspc_state_t *state, 
                                      unsigned int pf, unsigned int vf,
                                      fs_pci_core_resource_t *resource)
{
  unsigned int row;

  row = (vf == VF_NONE) ? pf : (pf + ((vf + 1) << 4));

  if (resource)
    assert(state->pfvf_to_core_resource_map[row] == NULL);
  else
    assert(state->pfvf_to_core_resource_map[row] != NULL);

  state->pfvf_to_core_resource_map[row] = resource;
}


/* Find a core resource by (PF, VF).
 */
static fs_pci_core_resource_t *find_pfvf_resource(pcie_cfgspc_state_t *state, 
                                                  unsigned int pf,
                                                  unsigned int vf)
{
  unsigned int row;

  row = (vf == VF_NONE) ? pf : (pf + ((vf + 1) << 4));

  return state->pfvf_to_core_resource_map[row];
}

pcie_func_t *fs_pci_pfvf_to_func(pcie_state_t *state,
                                 unsigned int pf, unsigned int vf, bool vf_active)
{
  pcie_cfgspc_state_t *cs = state->cfgspc_state;
  fs_pci_core_resource_t *resource = find_pfvf_resource(cs, pf, vf_active ? vf : VF_NONE);
  int rid;

  if (!resource)
    return NULL;

  rid = resource->rid;

  return cs->bus_state[rid].lies->func;
}

static int vf_count(fs_pci_core_resource_t *pf)
{
  fs_pci_core_resource_t *func = pf;
  int vf_count = 0;
  while(func->next_vf != 0) {
    vf_count++;
    func = func->next_vf;
  }
  return vf_count;
}

static int vf_to_vf_hw_idx(pcie_cfgspc_state_t *state, fs_pci_core_resource_t *resource)
{
  unsigned vf_hw_idx = 0;
  int i;
  for (i = 0; i < resource->pf; i++) {
    if (!state->pf_resources[i].visible)
      continue;
    vf_hw_idx += vf_count(&state->pf_resources[i]);
  }
  return vf_hw_idx + resource->vf;
}


/* Read an 8-bit configuration space value from a core resource (without any
 * side effects).
 */
static uint8_t get_cfg_8(fs_pci_core_resource_t *resource, unsigned int addr)
{
  return resource->config_space[addr];
}

/* Read a 16-bit configuration space value from a core resource (without any
 * side effects).
 */
static uint16_t get_cfg_16(fs_pci_core_resource_t *resource, unsigned int addr)
{
  return resource->config_space[addr] |
         (resource->config_space[addr+1] << 8);
}

/* Read a 32-bit configuration space value from a core resource (without any
 * side effects).
 */
static uint32_t get_cfg_32(fs_pci_core_resource_t *resource, unsigned int addr)
{
  return resource->config_space[addr] |
         (resource->config_space[addr+1] << 8) |
         (resource->config_space[addr+2] << 16) |
         (resource->config_space[addr+3] << 24);
}

/* Write an 8-bit configuration space value to a core resource (without any
 * side effects).
 */
static void put_cfg_8(fs_pci_core_resource_t *resource, unsigned int addr,
                      uint8_t val)
{
  resource->config_space[addr] = val;
}

/* Write a 16-bit configuration space value to a core resource (without any
 * side effects).
 */
static void put_cfg_16(fs_pci_core_resource_t *resource, unsigned int addr,
                       uint16_t val)
{
  resource->config_space[addr] = val & 0xff;
  resource->config_space[addr+1] = (val >> 8) & 0xff;
}

/* Write a 32-bit configuration space value to a core resource (without any
 * side effects).
 */
static void put_cfg_32(fs_pci_core_resource_t *resource, unsigned int addr,
                       uint32_t val)
{
  resource->config_space[addr] = val & 0xff;
  resource->config_space[addr+1] = (val >> 8) & 0xff;
  resource->config_space[addr+2] = (val >> 16) & 0xff;
  resource->config_space[addr+3] = (val >> 24) & 0xff;
}

/* Helper for applying a set of app-supplied fixups to a function's 
 * config space and BAR masks */
static void apply_config_space_fixups(fs_pci_core_resource_t *resource,
                                      const cfgspc_fixup_t *cfgspc_fixups,
                                      const barmask_fixup_t *mask_fixups)
{
  while(!CFGSPC_FIXUP_IS_END(cfgspc_fixups)) {
    if ((cfgspc_fixups->pfs & (1 << resource->pf)) != 0) {
      uint32_t addr, val;
      if (cfgspc_fixups->cap == 0) /* addr is address in config space */
        addr = cfgspc_fixups->addr;
      else if ((cfgspc_fixups->cap & 0x8000) != 0)
        addr = pcie_find_cap(cfgspc_fixups->cap & 0xff, resource->config_space) + cfgspc_fixups->addr;
      else                 /* addr is offset from start of cap */
        addr = pcie_find_xcap(cfgspc_fixups->cap, resource->config_space) + cfgspc_fixups->addr;
      val = get_cfg_32(resource, addr);
      val &= cfgspc_fixups->mask;
      val |= cfgspc_fixups->val;
      put_cfg_32(resource, addr, val);
    }
    cfgspc_fixups++;
  }

  while(!BARMASK_FIXUP_IS_END(mask_fixups)) {
    if ((mask_fixups->pf_mask & (1 << resource->pf)) == 0) {
      mask_fixups++;
      continue;
    }
    resource->bar_mask[mask_fixups->bar] = mask_fixups->mask;
    resource->region_flags[mask_fixups->bar] = mask_fixups->flags;
    mask_fixups++;
  }

  /*
   * Config space reads come straight from our backing store.  Ensure
   * that the region flags read from there are accurate.
   */
#define APPLY_REGION_FLAGS(addr_)                                       \
  do {                                                                  \
    unsigned bar_num__;                                                 \
    unsigned val__;                                                     \
    switch (addr_) {                                                    \
    case PCI_BASE_ADDRESS_0:                                            \
    case PCI_BASE_ADDRESS_1:                                            \
    case PCI_BASE_ADDRESS_2:                                            \
    case PCI_BASE_ADDRESS_3:                                            \
    case PCI_BASE_ADDRESS_4:                                            \
    case PCI_BASE_ADDRESS_5:                                            \
    case PCI_ROM_ADDRESS:                                               \
      bar_num__ = ((addr_) - PCI_BASE_ADDRESS_0)/4;                     \
      val__ = get_cfg_32(resource, (addr_));                            \
      val__ &= 0xffffffff - resource->bar_mask[bar_num__];              \
      val__ |= resource->region_flags[bar_num__];                       \
      put_cfg_32(resource, (addr_), val__);                             \
                                                                        \
      break;                                                            \
    default:                                                            \
      assert(0);                                                        \
    }                                                                   \
  } while (0)

  APPLY_REGION_FLAGS(PCI_BASE_ADDRESS_0);
  APPLY_REGION_FLAGS(PCI_BASE_ADDRESS_1);
  APPLY_REGION_FLAGS(PCI_BASE_ADDRESS_2);
  APPLY_REGION_FLAGS(PCI_BASE_ADDRESS_3);
  APPLY_REGION_FLAGS(PCI_BASE_ADDRESS_4);
  APPLY_REGION_FLAGS(PCI_BASE_ADDRESS_5);
  APPLY_REGION_FLAGS(PCI_ROM_ADDRESS);

#undef APPLY_REGION_FLAGS

  /* Setup SR-IOV BAR0/2 for PFs */
  if (resource->vf == VF_NONE) {
    uint16_t sriov_xcap = pcie_find_xcap(PCI_EXT_CAP_ID_SRIOV,
                                         resource->config_space);
    if (sriov_xcap != 0) {
      for (uint16_t bar_offset = SRIOV_CAP_OFFSET_BAR(0);
           bar_offset <= SRIOV_CAP_OFFSET_BAR(2); bar_offset += 8) {
        unsigned int bar_num;
        uint32_t addr;
        uint32_t val;

        bar_num = PCI_SRIOV_BAR0 + (bar_offset - SRIOV_CAP_OFFSET_BAR(0)) / 4;
        addr = sriov_xcap + bar_offset;

        val = get_cfg_32(resource, addr);
        val &= 0xffffffff - resource->bar_mask[bar_num];
        val |= resource->region_flags[bar_num];
        put_cfg_32(resource, addr, val);
      }
    }
  }
}

static pcie_cfgspc_callbacks_t *get_cfgspc_callbacks(pcie_cfgspc_state_t *s)
{
  return s->settings->cfg_cbs;
}

static void pcie_cfgspc_cb_fill_reset_values(pcie_state_t *s,
                                             unsigned int pf, unsigned int vf,
                                             uint8_t *config_space)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s->cfgspc_state);

  cbs->fill_reset_values(s, pf, vf, config_space);
}

static void pcie_cfgspc_cb_trigger_vpd(pcie_cfgspc_state_t *s,
                                       pcie_func_t *func,
                                       uint16_t cap_pos, uint16_t addr)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->trigger_vpd(func, cap_pos, addr);
}

static void pcie_cfgspc_cb_trigger_flr(pcie_cfgspc_state_t *s,
                                       void *mc_in, pcie_func_t *func)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->trigger_flr(mc_in, func);
}

static pci_bar_type_t pcie_cfgspc_cb_bar_num_to_type(pcie_state_t *s,
                                                     int fn,
                                                     pci_bar_num_t bar)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s->cfgspc_state);

  return cbs->bar_num_to_type(s, fn, bar);
}

static void pcie_cfgspc_cb_exprom_init(pcie_cfgspc_state_t *s,
                                       pcie_func_t *func)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->exprom_init(func);
}

static void pcie_cfgspc_cb_dpa_ctrl_notification(pcie_cfgspc_state_t *s,
                                                 struct fs_hw_s *hw,
                                                 int pf)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->dpa_ctrl_notification(hw, pf);
}

static void pcie_cfgspc_cb_set_pf_bme(pcie_cfgspc_state_t *s,
                                      struct fs_hw_s *hw, uint32_t pf, uint32_t enabled)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->set_pf_bme(hw, pf, enabled);
}

static void pcie_cfgspc_cb_set_vf_bme(pcie_cfgspc_state_t *s,
                                      struct fs_hw_s *hw, uint32_t vf, uint32_t enabled)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->set_vf_bme(hw, vf, enabled);
}

static pcie_func_t *pcie_cfgspc_cb_new_pcie_func(pcie_cfgspc_state_t *s,
                                                 void *hw)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  return cbs->new_pcie_func(hw);
}

static void pcie_cfgspc_cb_free_pcie_func(pcie_cfgspc_state_t *s,
                                          pcie_func_t *pfunc)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s);

  cbs->free_pcie_func(pfunc);
}

static void pcie_cfgspc_cb_pcie_apply_app_vf_hacks(pcie_state_t *s,
                                                   int parent_pf, int vf,
                                                   uint8_t *config_space)
{
  pcie_cfgspc_callbacks_t *cbs = get_cfgspc_callbacks(s->cfgspc_state);

  cbs->pcie_apply_app_vf_hacks(s, parent_pf, vf, config_space);
}

/* Apply fixups (i.e. differences from the reset values extracted via the
 * efhwdef library) to PF config space.
 */
static void apply_pf_config_space_fixups(pcie_cfgspc_state_t *state,
                                         fs_pci_core_resource_t *resource, bool hide_pcie)
{
  const cfgspc_fixup_t *cfgfix = state->settings->pf_cfgspc_fixups;
  const barmask_fixup_t *maskfix = state->settings->pf_barmask_fixups;
  const xcap_addition_t *xcap = state->settings->pf_new_xcaps;
  cap_addition_t *cap = state->settings->pf_new_caps;

  assert(resource->pf != PF_NONE);
  assert(resource->vf == VF_NONE);

  if (hide_pcie)
    pcie_hide_cap(PCI_CAP_ID_EXP, resource->config_space);

  apply_config_space_fixups(resource, cfgfix, maskfix);

  put_cfg_8(resource, PCI_INTERRUPT_PIN, (resource->pf % 4) + 1);

  while(!XCAP_ADDITION_IS_END(xcap)) {
    pcie_append_xcap(resource->config_space, xcap->id, xcap->ver, xcap->data, xcap->datalen);
    xcap++;
  }

  if (cap != NULL)
    while(!CAP_ADDITION_IS_END(cap)) {
      pcie_prepend_cap(resource->config_space, cap->id,  cap->data, cap->datalen);
      cap++;
    }
}


/* Apply fixups (i.e. differences from the reset values extracted via the
 * efhwdef library) to VF config space.
 */
static void apply_vf_config_space_fixups(pcie_state_t *state, fs_pci_core_resource_t *resource)
{
  pcie_cfgspc_state_t *cfg = state->cfgspc_state;
  const cfgspc_fixup_t *cfgfix = cfg->settings->vf_cfgspc_fixups;
  const barmask_fixup_t *maskfix = cfg->settings->vf_barmask_fixups;
  const xcap_addition_t *xcap = cfg->settings->vf_new_xcaps;

  assert(resource->pf != PF_NONE);
  assert(resource->vf != VF_NONE);

  apply_config_space_fixups(resource, cfgfix, maskfix);
  while(!XCAP_ADDITION_IS_END(xcap)) {
    pcie_append_xcap(resource->config_space, xcap->id, xcap->ver, xcap->data, xcap->datalen);
    xcap++;
  }
  pcie_cfgspc_cb_pcie_apply_app_vf_hacks(state, resource->pf, resource->vf,
                                         resource->config_space);
}

/* Interrupt state management */
bool fs_pci_intx_asserted(pcie_state_t *state,  pcie_func_t *func)
{
  fs_pci_core_resource_t *resource = state->cfgspc_state->bus_state[func->rid].core;  
  uint16_t status = get_cfg_16(resource, PCI_STATUS);
  return  (status & PCI_STATUS_INTERRUPT) != 0;
}

static void func_irq_setting(pcie_state_t *state, 
                             fs_pci_core_resource_t *res, pcie_irqmode_t mode,  bool on)
{
  pcie_func_t *func;

  /* This happens during start up: the MC prods cfg space from the DBI side. */
  if (res->rid == RID_NONE)
    return;

  func = fs_pci_pfvf_to_func(state, res->pf, res->vf, res->vf != VF_NONE);
  if (on) {
    if (((func->irqmode == FS_IRQMODE_MSI) && (mode == FS_IRQMODE_MSIX)) ||
        ((func->irqmode == FS_IRQMODE_MSIX) && (mode == FS_IRQMODE_MSI)))
      fs_log("Ignoring attempt to enable IRQ mode %d while in IRQ mode %d. Possible badness.\n",
              mode, func->irqmode);
    else
      func->irqmode = mode;
  } else {
    /* If legacy interrupts have just been disabled we should check whether we need to deassert the
     * the interrupt. NOTE: in theory we should be checking whether any other functions are asserting
     * the same legacy IRQ, but skip that for now. */
    if (mode == FS_IRQMODE_LEGACY && fs_pci_intx_asserted(state, func)) {
      fs_pci_assert_intx(state, func, false);
    }
    if (func->irqmode == mode)
      func->irqmode = FS_IRQMODE_NONE;
  }
}

static void update_cmd_enables(pcie_state_t *state, fs_pci_core_resource_t *res, unsigned val)
{
  pcie_func_t *func;
  bool bme, is_vf;

  if (res->rid == RID_NONE)
    return;

  is_vf = res->vf != VF_NONE;
  func = fs_pci_pfvf_to_func(state, res->pf, res->vf, is_vf);
  bme = func->master_enabled;
  func->mem_enabled = !!(val & PCI_COMMAND_MEMORY);
  func->master_enabled = !!(val & PCI_COMMAND_MASTER);
  func->io_enabled =  !!(val & PCI_COMMAND_IO);
  if (bme != func->master_enabled) {
    fs_log("%d.%d: BME state: %d -> %d\n",  res->pf, res->vf, bme, func->master_enabled);
    if (is_vf) {
      int vf = vf_to_vf_hw_idx(state->cfgspc_state, res);
      pcie_cfgspc_cb_set_vf_bme(state->cfgspc_state,
                                state->hw, vf, func->master_enabled);
    } else {
      pcie_cfgspc_cb_set_pf_bme(state->cfgspc_state,
                                state->hw, res->pf, func->master_enabled);
    }
  }
}

static void func_mask_msix(pcie_state_t *state, fs_pci_core_resource_t *res, bool on)
{
  pcie_func_t *func;

  /* This happens during start up: the MC prods cfg space from the DBI side. */
  if (res->rid == RID_NONE)
    return;

  func = fs_pci_pfvf_to_func(state, res->pf, res->vf, res->vf != VF_NONE);
  pcie_msix_set_func_mask(state, func, on);
}

/* Allocate some VF resources for a PF.
 */
static void alloc_vfs(pcie_cfgspc_state_t *state,  unsigned int pf, unsigned int num_vfs)
{
  unsigned int vf;
  fs_pci_core_resource_t *pf_resource;
  fs_pci_core_resource_t *vf_resource;
  fs_pci_core_resource_t *prev_resource;

  assert(pf < state->settings->max_pfs);

  pf_resource = find_pfvf_resource(state, pf, VF_NONE);
  assert(pf_resource != NULL);
  assert(pf_resource->next_vf == NULL);
  prev_resource = pf_resource;

  for (vf = 0; vf < num_vfs; vf++) {
    /* Take a VF resource from the free list */
    assert(state->vf_resource_free_list != NULL);
    vf_resource = state->vf_resource_free_list;
    state->vf_resource_free_list = state->vf_resource_free_list->next_vf;
    /* Initialise it */
    vf_resource->visible = false;
    vf_resource->pf = pf;
    vf_resource->vf = vf;
    vf_resource->rid = RID_NONE;
    vf_resource->next_vf = NULL;
    map_pfvf_to_core_resource(state, pf, vf, vf_resource);
    /* Append to PF's list */
    prev_resource->next_vf = vf_resource;
    prev_resource = vf_resource;
  }
}


/* Free all of the VF resources currently allocated to a PF.
 */
static void free_vfs(pcie_cfgspc_state_t *state, unsigned int pf)
{
  fs_pci_core_resource_t *pf_resource;
  fs_pci_core_resource_t *vf_resource;
  fs_pci_core_resource_t *next_vf_resource;

  assert(pf < state->settings->max_pfs);

  pf_resource = find_pfvf_resource(state, pf, VF_NONE);
  assert(pf_resource != NULL);
  vf_resource = pf_resource->next_vf;

  while (vf_resource != NULL) {
    /* Sanity check */
    assert(vf_resource->pf == pf);
    /* This isn't a valid thing to do after enumeration */
    assert(vf_resource->visible == false);
    assert(vf_resource->rid == RID_NONE);
    /* Remove the mapping */
    map_pfvf_to_core_resource(state, pf, vf_resource->vf, NULL);
    /* Zap some fields */
    vf_resource->pf = PF_NONE;
    vf_resource->vf = VF_NONE;
    /* Move to free list */
    next_vf_resource = vf_resource->next_vf;
    vf_resource->next_vf = state->vf_resource_free_list;
    state->vf_resource_free_list = vf_resource;
    vf_resource = next_vf_resource;
  }

  pf_resource->next_vf = NULL;
}

static void pcie_do_flr(pcie_state_t *state, fs_pci_core_resource_t *fn)
{
  uint16_t cmd = get_cfg_16(fn, PCI_COMMAND);

  cmd &= ~(PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY | PCI_COMMAND_IO);
  put_cfg_16(fn, PCI_COMMAND, cmd);

  update_cmd_enables(state, fn, cmd);
  /* Reenable DPA */
  if (state->cfgspc_state->dpa_cap)
    put_cfg_32(fn, state->cfgspc_state->dpa_cap + DPA_STATUS_CNTRL_REG, 0x100);
  
  /* Let the MC know */
  pcie_cfgspc_cb_trigger_flr(state->cfgspc_state, NULL, fn->br->lies->func);
}

static uint16_t dev_ctl_write(pcie_state_t *state, fs_pci_core_resource_t *fn, uint16_t val)
{
  bool flr = !!(val & PCI_EXP_DEVCTL_BCR_FLR);
  val &= ~PCI_EXP_DEVCTL_BCR_FLR;
  if (flr)
    pcie_do_flr(state, fn); 
  return val;
}

static void update_msix_settings(pcie_state_t *state, fs_pci_core_resource_t *res)
{
  pcie_core_settings_t *c_settings = pcie_core_cb_get_pcie_core_settings(state);
  uint32_t tbl_info, pba_info;
  const uint8_t msix = pcie_find_cap(PCI_CAP_ID_MSIX, res->config_space);
  tbl_info = get_cfg_32(res, msix + PCI_MSIX_TABLE);
  pba_info = get_cfg_32(res, msix + PCI_MSIX_PBA);
  /* The chip model has compiled-in knowledge of which BAR maps the MSI-X structures,
   * but we check this for consistency against what the MSI-X capability says. */
  Xassert_eq(
      pcie_cfgspc_cb_bar_num_to_type(state, 0, tbl_info & PCI_MSIX_TABLE_BIR),
      PCI_BAR_TYPE_MSIX);
  Xassert_eq(
      pcie_cfgspc_cb_bar_num_to_type(state, 0, pba_info & PCI_MSIX_PBA_BIR),
      PCI_BAR_TYPE_MSIX);

  c_settings->msix_tbl_off[res->rid] = tbl_info & PCI_MSIX_TABLE_OFFSET;
  c_settings->msix_pba_off[res->rid] = pba_info & PCI_MSIX_PBA_OFFSET;
}

static void init_master_core(pcie_state_t *state)
{
  pcie_core_settings_t *c_settings = pcie_core_cb_get_pcie_core_settings(state);
  state->retry_en = state->cfgspc_state->settings->initial_cfg_retry;
  c_settings->highest_visible_pf = state->cfgspc_state->settings->max_pfs;
}

/* Default bar_mask should be 0xffffffff for each bar. Otherwise we're allowing
 * to write any value to the BARs (even the ones we don't use), which confuses
 * some hosts
 */
static void init_bar_masks(fs_pci_core_resource_t *core)
{
  int i;

  for (i = 0; i < PCI_NUM_BARS; i++)
    core->bar_mask[i] = 0xffffffff;
}

void pcie_settings_validate(const pcie_settings_t *s)
{
  pcie_cfgspc_callbacks_t *cfg_cbs = s->cfg_cbs;
  pcie_core_callbacks_t *core_cbs = s->core_cbs;

  if(!cfg_cbs) {
    ERR("PCIe settings: no config space callbacks provided.\n");
  }
  if(!cfg_cbs->fill_reset_values) {
    ERR("PCIe settings: "
        "no 'fill_reset_values' config space callback provided.\n");
  }
  if(!cfg_cbs->trigger_vpd) {
    ERR("PCIe settings: "
        "no 'trigger_vpd' config space callback provided.\n");
  }
  if(!cfg_cbs->trigger_flr) {
    ERR("PCIe settings: "
        "no 'trigger_flr' config space callback provided.\n");
  }
  if(!cfg_cbs->bar_num_to_type) {
    ERR("PCIe settings: "
        "no 'bar_num_to_type' config space callback provided.\n");
  }
  if(!cfg_cbs->exprom_init) {
    ERR("PCIe settings error:"
        "no 'exprom_init' core callback provided.\n");
  }
  if(!cfg_cbs->dpa_ctrl_notification) {
    ERR("PCIe settings error:"
        "no 'dpa_ctrl_notification' config space callback provided.\n");
  }
  if(!cfg_cbs->set_pf_bme) {
    ERR("PCIe settings error:"
        "no 'set_pf_bme' config space callback provided.\n");
  }
  if(!cfg_cbs->set_vf_bme) {
    ERR("PCIe settings error:"
        "no 'set_vf_bme' config space callback provided.\n");
  }
  if(!cfg_cbs->new_pcie_func) {
    ERR("PCIe settings error:"
        "no 'new_pcie_func' config space callback provided.\n");
  }
  if(!cfg_cbs->free_pcie_func) {
    ERR("PCIe settings error:"
        "no 'free_pcie_func' config space callback provided.\n");
  }
  if(!cfg_cbs->pcie_apply_app_vf_hacks) {
    ERR("PCIe settings error:"
        "no 'pcie_apply_app_vf_hacks' config space callback provided.\n");
  }

  if(!core_cbs) {
    ERR("PCIe settings: no core callbacks provided.\n");
  }

#ifdef CONFIG_TLM
  if(!core_cbs->pcie_tx_tlp) {
    ERR("PCIe settings error:"
        "no 'pcie_tx_tlp' core callback provided.\n");
  }
#endif
  if(!core_cbs->pcie_vdm) {
    ERR("PCIe settings error:"
        "no 'pcie_vdm' core callback provided.\n");
  }
  if(!core_cbs->get_pcie_core_settings) {
    ERR("PCIe settings error:"
        "no 'get_pcie_core_settings' core callback provided.\n");
  }
  if(!core_cbs->host_r32) {
    ERR("PCIe settings error: no 'host_r32' core callback provided.\n");
  }
  if(!core_cbs->host_w32) {
    ERR("PCIe settings error: no 'host_w32' core callback provided.\n");
  }
  if(!core_cbs->biu_dma_read_complete) {
    ERR("PCIe settings error:"
        " no 'biu_dma_read_complete' core callback provided.\n");
  }
  if(!core_cbs->pcie_exit_hook) {
    ERR("PCIe settings error:"
        " no 'pcie_exit_hook' core callback provided.\n");
  }
  if(!core_cbs->pcie_validate_access) {
    ERR("PCIe settings error:"
        " no 'pcie_validate_access' core callback provided.\n");
  }
}

/* Initialise the model.
 */
void fs_pci_lies_init(fs_hw_t *hw, const pcie_settings_t *settings, pcie_state_t **state_out, pcie_intf_t instance)
{
  pcie_state_t *state;
  pcie_cfgspc_state_t *cfgspc_state;
  unsigned int pf, vf, next;

  pcie_settings_validate(settings);

  state = checked_calloc(sizeof(pcie_state_t));
  state->hw = hw;
  state->instance = instance;
  state->process_tlp_once = settings->process_tlp_once;
  state->pseudo_rc_enabled = settings->pseudo_rc_enabled;
  state->core_cbs = settings->core_cbs;
  state->tag_type = settings->tag_type;

  *state_out = state;

  state->cfgspc_state = cfgspc_state = checked_calloc(sizeof(pcie_cfgspc_state_t));

  memset(cfgspc_state->pfvf_to_core_resource_map, 0, sizeof(cfgspc_state->pfvf_to_core_resource_map));
  cfgspc_state->settings = settings;

  cfgspc_state->mem_addr_space.bar = checked_calloc(MAX_APERTURES_PER_FUNC * settings->max_total_funcs * sizeof(pci_bar_t));
  cfgspc_state->io_addr_space.bar = checked_calloc(MAX_APERTURES_PER_FUNC * settings->max_total_funcs * sizeof(pci_bar_t));

  cfgspc_state->bus_state = checked_calloc(settings->max_total_funcs * sizeof(cfgspc_state->bus_state[0]));

  cfgspc_state->pf_resources = checked_calloc(settings->max_pfs * sizeof(cfgspc_state->pf_resources[0]));
  cfgspc_state->msix_cap = checked_calloc(settings->max_total_funcs * sizeof(cfgspc_state->msix_cap[0]));

  for (pf = 0; pf < settings->max_pfs; pf++) {
    next = (pf + 1 < settings->max_pfs ? pf + 1 : 0);

    cfgspc_state->pf_resources[pf].visible = true;
    cfgspc_state->pf_resources[pf].pf = pf;
    cfgspc_state->pf_resources[pf].vf = VF_NONE;
    cfgspc_state->pf_resources[pf].rid = RID_NONE;
    cfgspc_state->pf_resources[pf].next_vf = NULL;
    init_bar_masks(&cfgspc_state->pf_resources[pf]);

    pcie_cfgspc_cb_fill_reset_values(state,
                                     pf,
                                     VF_NONE,
                                     cfgspc_state->pf_resources[pf].config_space);

    apply_pf_config_space_fixups(cfgspc_state, &cfgspc_state->pf_resources[pf], settings->no_pcie_cap);
    map_pfvf_to_core_resource(cfgspc_state, pf, VF_NONE, &cfgspc_state->pf_resources[pf]);

    pcie_cfgspc_fixup_write_mask(cfgspc_state->pf_resources[pf].config_space,
                                 cfgspc_state->pf_resources[pf].write_mask);
    pcie_cfgspc_fill_ari_next(cfgspc_state->pf_resources[pf].config_space,
                              next);
  }
  cfgspc_state->dpa_cap = pcie_find_xcap(PCI_EXT_CAP_ID_DPA, cfgspc_state->pf_resources[0].config_space);
  cfgspc_state->vpd_cap = pcie_find_cap(PCI_CAP_ID_VPD, cfgspc_state->pf_resources[0].config_space);
  cfgspc_state->vsec_cap = pcie_find_xcap(
      PCI_EXT_CAP_ID_VNDR, cfgspc_state->pf_resources[0].config_space);

  cfgspc_state->vf_resources = checked_calloc(settings->max_vfs * sizeof(cfgspc_state->vf_resources[0]));
  cfgspc_state->vf_resource_free_list = NULL;

  for (vf = 0; vf < settings->max_vfs; vf++) {
    cfgspc_state->vf_resources[vf].visible = false;
    cfgspc_state->vf_resources[vf].pf = PF_NONE;
    cfgspc_state->vf_resources[vf].vf = VF_NONE;
    cfgspc_state->vf_resources[vf].rid = RID_NONE;
    cfgspc_state->vf_resources[vf].next_vf = cfgspc_state->vf_resource_free_list;
    memset(cfgspc_state->vf_resources[vf].config_space, 0,
           sizeof(cfgspc_state->vf_resources[vf].config_space));
    cfgspc_state->vf_resource_free_list = &cfgspc_state->vf_resources[vf];
  }

  init_master_core(state);
  create_pfs(state);
  init_pcie(state);
}


/* Create the external lies for a given core resource and position on the bus.
 */
static void create_lies_for_core_resource(pcie_state_t *state,
                                          fs_pci_core_resource_t *resource,
                                          unsigned int rid)
{
  pcie_cfgspc_state_t *cfgspc_state = state->cfgspc_state;
  fs_pci_lies_t *lies;
  pcie_func_t *func;
  bool is_vf = (resource->vf != VF_NONE);
  
  Xassert_lt(rid, cfgspc_state->settings->max_total_funcs);

  FS_PCI_DEBUG("@@ Creating lies for resource %p rid %u\n", resource, rid);
  lies = calloc(1, sizeof(*lies));

  func = pcie_cfgspc_cb_new_pcie_func(cfgspc_state, state->hw);
  func->hw = state->hw;

  lies->core = resource;

  for (int i = 0; i < ARRAY_SIZE(lies->bars); i++) {
    lies->bars[i].owner = lies;
    lies->bars[i].num = i;
    /* The SR-IOV apertures are mapped by the PF's SR-IOV BARs. The bars[]
     * of a VF should never be placed or looked up directly. Setting
     * them to invalid will catch any mistakes of that sort. */
    lies->bars[i].type = is_vf ? PCI_BAR_TYPE_INVAL
                               : pcie_cfgspc_cb_bar_num_to_type(state,
                                                                resource->rid, i);

    if (lies->bars[i].type == PCI_BAR_TYPE_IO)
      resource->region_flags[i] = 1; /* I/O */
  }

  lies->dev.bus = 1;
  lies->dev.dev = rid >> 3;
  lies->dev.func = rid & 7;


  func->iface = state->instance;
  func->pf_num = resource->pf;
  func->vf_num = (resource->vf != VF_NONE) ? resource->vf : -1;
  func->bus = lies->dev.bus;
  func->dev = lies->dev.dev;
  func->func = lies->dev.func;
  func->rid = rid;
  func->config_space = resource->config_space;

  pcie_hw_msix_init(func);
  pcie_cfgspc_cb_exprom_init(cfgspc_state, func);

  lies->func = func;
  cfgspc_state->bus_state[rid].core = resource;
  cfgspc_state->bus_state[rid].core->br = &cfgspc_state->bus_state[rid];
  cfgspc_state->bus_state[rid].lies = lies;

  cfgspc_state->msix_cap[rid] =  pcie_find_cap(PCI_CAP_ID_MSIX, resource->config_space);
  update_msix_settings(state, resource);
}

/* Destroy the external lies for the VF at a given position on the bus.
 */
static void destroy_lies_for_vf_on_bus(pcie_cfgspc_state_t *state,
                                       unsigned int rid)
{
  fs_pci_core_resource_t *vf_resource;
  fs_pci_lies_t *lies;

  vf_resource = state->bus_state[rid].core;
  assert(vf_resource != NULL);
  assert(vf_resource->vf != VF_NONE);
  assert((vf_resource->rid == rid) || (vf_resource->rid == RID_NONE));

  FS_PCI_DEBUG("@@ Destroying lies for resource %p rid %u\n", vf_resource, rid);
  lies = state->bus_state[rid].lies;
  if (lies != NULL) {
    pcie_cfgspc_cb_free_pcie_func(state, lies->func);
    memset(lies, 0, sizeof(*lies));
    free(lies);
    state->bus_state[rid].lies = NULL;
  }
}

/* Create the external lies for all of the PFs on the bus.
 */
static void create_pfs(pcie_state_t *state)
{
  unsigned int pf, rid;
  fs_pci_core_resource_t *pf_resource;

  for (pf = 0; pf < state->cfgspc_state->settings->max_pfs; pf++) {
    rid = pf;
    pf_resource = find_pfvf_resource(state->cfgspc_state, pf, VF_NONE);
    if (pf_resource->visible) {
      pf_resource->rid = rid;
      create_lies_for_core_resource(state, pf_resource, rid);
    }
  }
}


/* Reveal the VFs for a PF on the bus, and create the external lies for them.
 */
static void enable_vfs(pcie_state_t *state, unsigned int pf)
{
  fs_pci_core_resource_t *pf_resource;
  fs_pci_core_resource_t *vf_resource;
  unsigned int rid, num_vfs, vf_offset, vf_stride, vf;
  uint16_t sriov_xcap;

  pf_resource = find_pfvf_resource(state->cfgspc_state, pf, VF_NONE);
  assert(pf_resource != NULL);
  assert(pf_resource->visible);
  assert(pf_resource->pf == pf);
  assert(pf_resource->vf == VF_NONE);
  assert(pf_resource->rid != RID_NONE);
  
  sriov_xcap = pcie_find_xcap(PCI_EXT_CAP_ID_SRIOV, pf_resource->config_space);
  num_vfs = get_cfg_16(pf_resource, sriov_xcap + PCI_SRIOV_NUM_VF);
  vf_offset = get_cfg_16(pf_resource, sriov_xcap + PCI_SRIOV_VF_OFFSET);
  vf_stride = get_cfg_16(pf_resource, sriov_xcap + PCI_SRIOV_VF_STRIDE);

  /* Release any existing VFs and re-allocate the correct number */
  free_vfs(state->cfgspc_state, pf);
  alloc_vfs(state->cfgspc_state, pf, num_vfs);
  
  FS_PCI_DEBUG("@@ Enabling %u VF(s) on PF%u\n", num_vfs, pf);

  rid = pf_resource->rid + vf_offset;

  for (vf = 0; vf < num_vfs; vf++) {
    vf_resource = find_pfvf_resource(state->cfgspc_state, pf, vf);
    assert(vf_resource != NULL);
    assert(vf_resource->pf == pf);
    assert(vf_resource->vf == vf);
    assert(vf_resource->rid == RID_NONE);
    vf_resource->visible = true;
    vf_resource->rid = rid;

    pcie_cfgspc_cb_fill_reset_values(state, pf, vf,
                                     vf_resource->config_space);
    apply_vf_config_space_fixups(state, vf_resource);
    /* This expects to extract useful information from config space,
     * so gets called after it has been filled in. */
    create_lies_for_core_resource(state, vf_resource, rid);

    pcie_cfgspc_fixup_write_mask(vf_resource->config_space,
                                 vf_resource->write_mask);

    rid += vf_stride;
  }
}


/* Disable the VFs for a PF on the bus, and release resources for their
 * external lies.
 */
static void disable_vfs(pcie_state_t *state, unsigned int pf)
{
  fs_pci_core_resource_t *pf_resource;
  fs_pci_core_resource_t *vf_resource;
  unsigned int rid, num_vfs, vf_offset, vf_stride, vf;
  uint16_t sriov_xcap;

  pf_resource = find_pfvf_resource(state->cfgspc_state, pf, VF_NONE);
  assert(pf_resource != NULL);
  assert(pf_resource->visible);
  assert(pf_resource->pf == pf);
  assert(pf_resource->vf == VF_NONE);
  assert(pf_resource->rid != RID_NONE);

  sriov_xcap = pcie_find_xcap(PCI_EXT_CAP_ID_SRIOV, pf_resource->config_space);
  num_vfs = get_cfg_16(pf_resource, sriov_xcap + PCI_SRIOV_NUM_VF);
  vf_offset = get_cfg_16(pf_resource, sriov_xcap + PCI_SRIOV_VF_OFFSET);
  vf_stride = get_cfg_16(pf_resource, sriov_xcap + PCI_SRIOV_VF_STRIDE);

  FS_PCI_DEBUG("@@ Disabling %u VF(s) on PF%u\n", num_vfs, pf);

  rid = pf_resource->rid + vf_offset;

  for (vf = 0; vf < num_vfs; vf++) {
    vf_resource = find_pfvf_resource(state->cfgspc_state, pf, vf);
    assert(vf_resource != NULL);
    assert(vf_resource->pf == pf);
    assert(vf_resource->vf == vf);
    assert(vf_resource->rid == rid);
    destroy_lies_for_vf_on_bus(state->cfgspc_state, rid);
    vf_resource->visible = false;
    vf_resource->rid = RID_NONE;
    rid += vf_stride;
  }
}


/* Memory-space aperture handling */

/* This is passed to qsort to sort the BARs by start address */
static int bar_sorter(const void *a, const void *b)
{
  const pci_bar_t *b1 = *(pci_bar_t **)a;
  const pci_bar_t *b2 = *(pci_bar_t **)b;

  if (b1->start < b2->start)
    return -1;
  
  if (b2->start == b1->start)
    return 0;
  
  return 1;
}

/* This is passed to bsearch to check whether the address matches the BAR.
 * There is a slight wrinkle: BARs that have been sized but not placed by
 * the host will appear to run from 0 to (size - 1). We avoid ever matching
 * them through the "|| b->start == 0" in the second test (since they
 * are all at the start of the array, returning 1 will move bsearch past them).
 */
static int bar_finder(const void *key, const void *elem)
{
  uint64_t addr = *(uint64_t *)key;
  const pci_bar_t *b = *(pci_bar_t **)elem;
  if (addr < b->start)
   return -1;
  if (addr >= b->end || b->start == BAR_NEVER_MATCH)
   return 1;
  return 0;
}

static void dump_addr_space(fs_pci_addr_space_t *addr_space)
{
#if 0
  int i;
  fs_log("+++++++++++++++++++\n");
  for (i = 0; i < addr_space->configured_bars; i++) {
    pci_bar_t *bar = addr_space->bar[i];
    if (bar)
      fs_log("+++ RID: %d at 0x%x - 0x%x\n", bar->owner->core->rid,  bar->start, bar->end);
    else
      fs_log("+++ NULL\n");
  }
#endif
}

/* The amount of address space mapped by a SRIOV BAR changes depending on
 * NUMVFS (it's up to the host to have reserved enough space for the largest
 * number of VFs that will ever be enabled. */
static void set_vfbar_sizes(fs_pci_core_resource_t *resource, uint32_t val)
{
  fs_pci_bus_resource_t *br = resource->br;
  pci_bar_t *bar;
  bar = &br->lies->bars[PCI_SRIOV_BAR0];
  bar->end = bar->start + (resource->bar_mask[PCI_SRIOV_BAR0] + 1) * val - 1;
  bar = &br->lies->bars[PCI_SRIOV_BAR2];
  bar->end = bar->start + (resource->bar_mask[PCI_SRIOV_BAR2] + 1) * val - 1;
}

static void place_aperture(pcie_cfgspc_state_t *state,
                           fs_pci_core_resource_t *resource, int barno,
                           uint64_t val, uint64_t mask)
{
  fs_pci_bus_resource_t *br = resource->br;
  fs_pci_addr_space_t *addr_space = NULL;
  pci_bar_t *bar = &br->lies->bars[barno];
  int i, scale;

  if (barno >= PCI_SRIOV_BAR0 && barno <= PCI_SRIOV_BAR5) {
    uint16_t sriov_xcap = pcie_find_xcap(PCI_EXT_CAP_ID_SRIOV,
                                         resource->config_space);
    scale = get_cfg_16(resource, sriov_xcap + PCI_SRIOV_NUM_VF);
  } else {
    scale = 1;
  }
  
  switch(bar->type) {
    case PCI_BAR_TYPE_IO:
      addr_space = &state->io_addr_space;
      break;
    case PCI_BAR_TYPE_EXPROM:
      addr_space = &state->mem_addr_space;
      /* The exprom BAR has an enable bit in position 0. If this
       * is not set the BAR should never match a host request (note that
       * changing val does not change what the host reads back as the
       * BAR value. This allows a host to disable the exprom and place
       * another aperture at the same address without collision. */
      if ((val & 1) == 0)
        val = BAR_NEVER_MATCH;
      break;
    case PCI_BAR_TYPE_INVAL:
      ERR("Attempt to map BAR %d which is not used on this platform.\n", barno);
      break;
    default:
      addr_space = &state->mem_addr_space;
      break;
  }
  
  if (bar == NULL)
    return;

  for (i = 0; i < addr_space->configured_bars; i++)  {
    /* Someone may already have placed this BAR */
    if (addr_space->bar[i] == bar) {
      break;
    }
  }
  if (i == addr_space->configured_bars) {
    /* We didn't find it. New entry */
    addr_space->bar[i] = bar;
    addr_space->configured_bars++;
  }

  bar->start = val & mask;
  bar->end = bar->start + (~mask + 1) * scale - 1;
  fs_log("%d : BAR %d position at 0x%lx : 0x%lx\n", resource->rid, barno, bar->start, bar->end);
  qsort(addr_space->bar, addr_space->configured_bars, sizeof(bar), bar_sorter);
}

static void dpa_ctl_stat_write(pcie_state_t *state,
                                  fs_pci_core_resource_t *resource,
                                  uint32_t value,
                                  bool ctl_written,
                                  bool stat_written,
                                  bool by_mc)
{
  uint16_t ctl, stat, old_stat, dpa_cap;
  dpa_cap = state->cfgspc_state->dpa_cap;
  old_stat = get_cfg_16(resource, dpa_cap  + DPA_STATUS_CNTRL_REG);
  if (ctl_written && stat_written) {
    ctl = BITFIELD_GET(value, DPA_SUBSTATE_CONTROL);
    stat = value & 0xffff;
  } else if (ctl_written) {
    ctl = value;
    stat = 0; /* gcc moronicity */
  } else {
    stat = value;
    ctl = 0; /* gcc moronicity */
  }

  if (stat_written) {
    if (by_mc) {
      put_cfg_16(resource, dpa_cap + DPA_STATUS_CNTRL_REG, stat);
    } else {
      /* The only thing the host can influence is bit 8 which is W1C */
      if ((stat & 0x100)) {
        old_stat &= ~0x100;
        put_cfg_16(resource, dpa_cap + DPA_STATUS_CNTRL_REG, old_stat);
      }
    }
  }
  /* Bit 8 of stat control whether writing ctrl does anything */
  if (ctl_written) {
    put_cfg_16(resource, dpa_cap + DPA_STATUS_CNTRL_REG + 2, ctl);
    /* Notify MC of host write */
    if (!by_mc && (old_stat & 0x100) != 0) 
      pcie_cfgspc_cb_dpa_ctrl_notification(state->cfgspc_state,
                                           state->hw, resource->pf);
  }
}

/* Handle a 32-bit configuration space read, with any associated side effects.
 */
static uint32_t handle_cfg_read_32(fs_pci_core_resource_t *resource,
                                   unsigned int addr,
                                   fs_pci_cfg_access_type_t type)
{
  /* We don't implement any read side effects at the moment */
  return resource->visible ? get_cfg_32(resource, addr) : 0xffffffff;
}

/* Handle an 8-bit configuration space write, with any associated side effects.
 */
static void handle_cfg_write_8(pcie_state_t *state, fs_pci_core_resource_t *resource,
                               unsigned int addr, uint8_t val,
                               fs_pci_cfg_access_type_t type)
{
  bool is_vf, from_mc, write_ok;

  is_vf = (resource->vf != VF_NONE);
  from_mc = (type != CFG_ACCESS_HOST);
  write_ok = pcie_cfgspc_verify_write_access(resource->write_mask, addr, 8, is_vf, from_mc);
  if (!write_ok) {
    fs_log("Warning: config write to 0x%03x rejected\n", addr);
    return;
  }

  if (type == CFG_ACCESS_MC_CS2) {
    /* No 8-bit CS2 registers are implemented */
    return;
  }
  else {
    /* No side effects are implemented for 8-bit writes at the moment */
    fs_log("MAYBE IMPLEMENT ME 0x%03x (8)\n", addr);
    put_cfg_8(resource, addr, val);
  }
}

/* The YML is confused about the available access widths for the VPD_CAP_CTL_REG */
static bool misc_vpd_workaround(pcie_state_t *state, unsigned int addr, bool is_vf)
{
  uint16_t vpd_cap = state->cfgspc_state->vpd_cap;
  return  (!is_vf && vpd_cap != 0 && ((addr & 0xfffc) == vpd_cap));
}

static bool in_cap_range(uint32_t addr, uint32_t cap_base, uint32_t cap_len)
{
  return cap_base != 0 && addr >= cap_base && addr < (cap_base + cap_len);
}

/* Handle a 16-bit configuration space write, with any associated side effects.
 */
static void handle_cfg_write_16(pcie_state_t *state, fs_pci_core_resource_t *resource,
                                unsigned int addr, uint16_t val,
                                fs_pci_cfg_access_type_t type)
{
  bool is_vf, from_mc, write_ok;
  uint16_t dpa_cap = state->cfgspc_state->dpa_cap;
  uint16_t msix_cap = state->cfgspc_state->msix_cap[resource->rid];
  uint16_t pcie_cap = pcie_find_cap(PCI_CAP_ID_EXP, resource->config_space);
  uint16_t sriov_xcap = 0;

  is_vf = (resource->vf != VF_NONE);
  from_mc = (type != CFG_ACCESS_HOST);
  write_ok = pcie_cfgspc_verify_write_access(resource->write_mask, addr, 16,
                                             is_vf, from_mc);
  if (!write_ok && !misc_vpd_workaround(state, addr, is_vf)) {
    fs_log("Warning: config write to 0x%03x rejected\n", addr);
    return;
  }

  /* SR-IOV Extended capability (only allowed on PFs). */
  if (!is_vf) {
    sriov_xcap = pcie_find_xcap(PCI_EXT_CAP_ID_SRIOV, resource->config_space);
  }

  if (type == CFG_ACCESS_MC_CS2) {

    if (in_cap_range(addr, sriov_xcap, PCI_EXT_CAP_SRIOV_SIZEOF)) {
      unsigned int sriov_offset = addr - sriov_xcap;

      switch (sriov_offset) {
      case PCI_SRIOV_INITIAL_VF:
        assert(resource->pf != PF_NONE);
        /* The synopsys manual has TOTALVFS just tracking INITIALVFS */
        put_cfg_16(resource, sriov_xcap + PCI_SRIOV_TOTAL_VF, val);
        break;
      case PCI_SRIOV_VF_OFFSET:
      case PCI_SRIOV_VF_STRIDE:
        /* We let these through instead of the non-CS2 variants */
        break;
      default:
        fs_log("IMPLEMENT ME 0x%03x (16, cs2)\n", addr);
        return;
      }
      put_cfg_16(resource, addr, val);

    } else if (msix_cap != 0 && addr == (msix_cap + PCI_MSIX_FLAGS)) {
      if (is_vf) {
        ERR("Not expecting CS2 access to MSIX_CTL_REG on a VF\n");
      } else {
        /* CS2 accesses to a PF PCMCR_MSIX_CTL_REG set it for all the child VFs */
        fs_pci_core_resource_t *vf_resource;
        int num_vfs = sriov_xcap ? get_cfg_16(resource,
                                             sriov_xcap + PCI_SRIOV_TOTAL_VF) : 0;
        int pf, vf;
        pf = resource->pf;
        for (vf = 0; vf < num_vfs; vf++) {
          vf_resource = find_pfvf_resource(state->cfgspc_state, pf, vf);
          func_irq_setting(state, vf_resource, FS_IRQMODE_MSIX,
                           !!(val & PCI_MSIX_FLAGS_ENABLE));
          func_mask_msix(state, vf_resource,  !!(val & PCI_MSIX_FLAGS_MASKALL));
          put_cfg_16(vf_resource, addr, val);
        }
      }
    } else if (in_cap_range(addr, state->cfgspc_state->vpd_cap, PCI_VPD_DATA)) {
       put_cfg_16(resource, addr, val);
    } else {
      fs_log("IMPLEMENT ME 0x%03x (16, cs2)\n", addr);
    }
  }
  else {
    switch (addr) {
    case PCI_COMMAND:
      func_irq_setting(state, resource, FS_IRQMODE_LEGACY, !(val & PCI_COMMAND_INTX_DISABLE));
      update_cmd_enables(state, resource, val);
      break;
    case PCI_SUBSYSTEM_VENDOR_ID:
    case PCI_SUBSYSTEM_ID:
      break;
#ifdef PCMCR_MSI_CTL_REG
    case PCMCR_MSI_CTL_REG:
      func_irq_setting(state, resource, FS_IRQMODE_MSI, BITFIELD_GET(val, PCMCRF_MSI_EN));
      break;
#endif
    default:
      if (in_cap_range(addr, dpa_cap, PCI_DPA_BASE_SIZEOF)) {
        switch(addr - dpa_cap - DPA_STATUS_CNTRL_REG) {
          case 0:
            dpa_ctl_stat_write(state, resource, val, false, true, from_mc);
            return; /* dpa_ctl_stat_write handles the update */
            break;
          case 2:
            dpa_ctl_stat_write(state, resource, val, true, false, from_mc);
            return;  /* dpa_ctl_stat_write handles the update */
            break;
          default:
            break;
        }

      } else if (in_cap_range(addr, sriov_xcap, PCI_EXT_CAP_SRIOV_SIZEOF)) {
        unsigned int sriov_offset = addr - sriov_xcap;

        switch (sriov_offset) {
        case PCI_SRIOV_CTRL:
          if ((type == CFG_ACCESS_HOST) && !is_vf) {
            uint16_t previous = get_cfg_16(resource,
                                           sriov_xcap + PCI_SRIOV_CTRL);
            bool vf_en_prev = previous & PCI_SRIOV_CTRL_VFE;
            bool vf_en_next = val & PCI_SRIOV_CTRL_VFE;

            if (vf_en_next && !vf_en_prev) {
              enable_vfs(state, resource->pf);
            } else if (!vf_en_next && vf_en_prev) {
              disable_vfs(state, resource->pf);
            }
          }
          break;
        case PCI_SRIOV_NUM_VF:
          set_vfbar_sizes(resource, val);
          break;
        case PCI_SRIOV_VF_DID:
        case PCI_SRIOV_SUP_PGSIZE:
          /* Known registers with no implemented side effects */
          break;
        case PCI_SRIOV_INITIAL_VF:
        case PCI_SRIOV_VF_OFFSET:
        case PCI_SRIOV_VF_STRIDE:
          /* We let the CS2 variants (the ARI version) through instead */
          return;
        default:
          fs_log("SRIOV Ext CAP address: MAYBE IMPLEMENT ME 0x%03x (16)\n",
                 sriov_offset);
          break;
        }

        /* Fall through to below put_cfg_16 */

      } else if (pcie_cap && addr == (pcie_cap + PCI_EXP_DEVCAP)) {
        val = dev_ctl_write(state, resource, val);
      } else if (msix_cap && addr == (msix_cap + PCI_MSIX_FLAGS)) {
        uint16_t table_sz;

         //   case PCMCR_MSIX_CTL_REG:
        func_irq_setting(state, resource, FS_IRQMODE_MSIX,
                         !!(val & PCI_MSIX_FLAGS_ENABLE));
        func_mask_msix(state, resource, !!(val & PCI_MSIX_FLAGS_MASKALL));

        /* The table size is read only and can't be altered by SW */
        table_sz = get_cfg_16(resource, msix_cap + PCI_MSIX_FLAGS) &
                   PCI_MSIX_FLAGS_QSIZE;

        val &= ~PCI_MSIX_FLAGS_QSIZE;
        val |= table_sz;
      } else if (in_cap_range(addr,
                              state->cfgspc_state->vpd_cap, PCI_VPD_DATA)) {
        pcie_cfgspc_cb_trigger_vpd(state->cfgspc_state, resource->br->lies->func,
                                   state->cfgspc_state->vpd_cap, val);
        /* fs_hw_trigger_vpd may synchronously update config space, so we do *not* want the
         * put_cfg_16 below to execute */
        return;
      } else {
       fs_log("MAYBE IMPLEMENT ME 0x%03x (16)\n", addr);
      }
      break;
    }
    put_cfg_16(resource, addr, val);
  }
}

/*
 * Check if it is a 64 bit bar.
 */
static bool pcie_is_64bit_bar(fs_pci_core_resource_t *resource,
                              uint32_t base_addr,
                              uint32_t addr,
                              bool *is_bar_hi)
{
  uint32_t bar_num = (addr - base_addr) / 4;
  uint32_t bar = 0;

  /* Go through the bars, 5:th main BAR or SRIOV BAR can't be 64 bit */
  while (bar < PCI_MAIN_BAR5) {
    bool is_64bit_bar = resource->region_flags[bar] &
                        PCI_BASE_ADDRESS_MEM_TYPE_64;
    uint32_t next_bar = bar + 1;

    if (is_64bit_bar && bar_num == bar) {
      *is_bar_hi = false;
      return true;
    } else if (is_64bit_bar && bar_num == next_bar) {
      *is_bar_hi = true;
      return true;
    }

    /* Go to next main BAR */
    if (is_64bit_bar) {
      /* Skip next bar_num if it's part of a 64 bit BAR */
      bar++;
    }
    bar++;
  }

  return false;
}

/*
 * Updates the BAR mappings and returns the value to write to the BAR register.
 */
static uint32_t pcie_update_bar_mappings(pcie_state_t *state,
                                         fs_pci_core_resource_t *resource,
                                         uint32_t base_addr,
                                         uint32_t addr,
                                         uint32_t val)
{
  uint32_t bar_num;
  uint64_t val64 = 0;
  bool is_64bit_bar;
  bool is_bar_hi;
  uint64_t mask64;
  uint32_t mask32;
  uint64_t cfg_mask;

  bar_num = (addr - base_addr)/4;
  if (base_addr == SRIOV_CAP_OFFSET_BAR(0)) {
    bar_num += PCI_SRIOV_BAR0;
  }

  is_bar_hi = false;
  is_64bit_bar = pcie_is_64bit_bar(resource, base_addr, addr, &is_bar_hi);

  /* Create the 64 bit addr and 32 bit mask */
  if (is_64bit_bar) {
    if (is_bar_hi) {
      /* Fetch addr[31:4] from previous BAR */
      uint32_t addr_lo = get_cfg_32(resource, addr - 4) & ~(0xF);

      val64 = val;
      val64 <<= 32;
      val64 |= addr_lo;

      cfg_mask = resource->bar_mask[bar_num];
      cfg_mask <<= 32;
      cfg_mask |= resource->bar_mask[bar_num - 1];

      mask64 = ~(cfg_mask);
      mask32 = (mask64 >> 32) & 0xFFFFFFFF;
    } else {
      /* Fetch addr[63:32] from next BAR */
      uint32_t addr_hi = get_cfg_32(resource, addr + 4);

      val64 = addr_hi;
      val64 <<= 32;
      val64 |= val;

      cfg_mask = resource->bar_mask[bar_num + 1];
      cfg_mask <<= 32;
      cfg_mask |= resource->bar_mask[bar_num];

      mask64 = ~(cfg_mask);
      mask32 = mask64 & 0xFFFFFFFF;
    }
  } else {
    val64 = val;
    cfg_mask = resource->bar_mask[bar_num];
    mask64 = ~cfg_mask;
    mask32 = mask64;
  }

  /*
   * If we have a BAR mask set up and the host writes a suitably-aligned
   * non-zero BAR value, place the aperture.
   */
  val64 &= mask64;
  if (val64 && cfg_mask) {
    uint32_t bar;
    /*
     * Always configure the the lower bar_num (this means that memory
     * transactions will go through the lower index BAR).
     */
    bar = bar_num;
    if (is_64bit_bar && is_bar_hi) {
      bar--;
    }
    place_aperture(state->cfgspc_state, resource, bar, val64, mask64);
  }

  /* Update the 32 bit value to write according to the mask */
  val &= mask32;

  /*
   * Store the flags if it is a 32 bit BAR or the lower 'bar_num' of a 64 bit
   * BAR is being written.
   */
  if (!is_bar_hi) {
    val |= resource->region_flags[bar_num];
  }

  return val;
}

/* Handle a 32-bit configuration space write, with any associated side effects.
 */
static void handle_cfg_write_32(pcie_state_t *state, fs_pci_core_resource_t *resource,
                                unsigned int addr, uint32_t val,
                                fs_pci_cfg_access_type_t type)
{
  bool is_vf, from_mc, write_ok;
  uint16_t dpa_cap = state->cfgspc_state->dpa_cap;
  uint16_t msix_cap = state->cfgspc_state->msix_cap[resource->rid];
  uint16_t sriov_xcap = 0;

  is_vf = (resource->vf != VF_NONE);
  from_mc = (type != CFG_ACCESS_HOST);
  write_ok = pcie_cfgspc_verify_write_access(resource->write_mask, addr, 32,
                                             is_vf, from_mc);
  if (!write_ok && !misc_vpd_workaround(state, addr, is_vf)) {
    fs_log("Warning: config write to 0x%03x rejected\n", addr);
    return;
  }

  /* SR-IOV Extended capability (only allowed on PFs). */
  if (!is_vf) {
    sriov_xcap = pcie_find_xcap(PCI_EXT_CAP_ID_SRIOV, resource->config_space);
  }

  if (type == CFG_ACCESS_MC_CS2) {
    switch (addr) {
    case PCI_BASE_ADDRESS_0:
    case PCI_BASE_ADDRESS_1:
    case PCI_BASE_ADDRESS_2:
    case PCI_BASE_ADDRESS_3:
    case PCI_BASE_ADDRESS_4:
    case PCI_BASE_ADDRESS_5:
    case PCI_ROM_ADDRESS:
      if (val & 1)
        resource->bar_mask[(addr - PCI_BASE_ADDRESS_0)/4] = val; /* Bottom bit is enable */
      return;
    default:
      if (in_cap_range(addr,
                       state->cfgspc_state->vpd_cap, PCI_CAP_VPD_SIZEOF)) {
        put_cfg_32(resource, addr, val);
      } else if (in_cap_range(addr, sriov_xcap, PCI_EXT_CAP_SRIOV_SIZEOF)) {
        unsigned int sriov_offset = addr - sriov_xcap;

        switch (sriov_offset) {
        case SRIOV_CAP_OFFSET_BAR(0):
        case SRIOV_CAP_OFFSET_BAR(1):
        case SRIOV_CAP_OFFSET_BAR(2):
        case SRIOV_CAP_OFFSET_BAR(3):
          /* Bottom bit is enable */
          if (val & 1) {
            resource->bar_mask[PCI_SRIOV_BAR0 +
                              (sriov_offset - SRIOV_CAP_OFFSET_BAR(0))/4] = val;
          }
          return;
        default:
          fs_log("SRIOV Ext CAP address: MAYBE IMPLEMENT ME 0x%03x (16)\n",
                 sriov_offset);
          break;
        }
      } else {
        fs_log("IMPLEMENT ME 0x%03x (32, cs2)\n", addr);
      }
      return;
    }
  }
  else {
    switch (addr) {
    case PCI_BASE_ADDRESS_0:
    case PCI_BASE_ADDRESS_1:
    case PCI_BASE_ADDRESS_2:
    case PCI_BASE_ADDRESS_3:
    case PCI_BASE_ADDRESS_4:
    case PCI_BASE_ADDRESS_5:
    case PCI_ROM_ADDRESS:
      val = pcie_update_bar_mappings(state, resource,
                                     PCI_BASE_ADDRESS_0,
                                     addr,
                                     val);
      break;
#ifdef PCMCR_TPH_CAP_HDR_REG
    case PCMCR_TPH_CAP_HDR_REG:
#endif
#ifdef PCMCR_SYMBOL_TMR_AND_FILTER_MSK_REG
    case PCMCR_SYMBOL_TMR_AND_FILTER_MSK_REG:
#endif
#ifdef PCMCR_GEN3_CONTROL_REG
    case PCMCR_GEN3_CONTROL_REG:
#endif
#ifdef PCMCR_VPD_CAP_DATA_REG
    case PCMCR_VPD_CAP_DATA_REG:
#endif
      /* Known registers with no implemented side effects */
      break;
#ifdef PCMCR_TIMER_CTRL_MAX_FUNC_NUM_REG
    case PCMCR_TIMER_CTRL_MAX_FUNC_NUM_REG: {
      unsigned int pf;
      state->hw->pcie_core.settings.highest_visible_pf = BITFIELD_GET(val,PCMCRF_CX_NFUNC_MINUS_ONE);
      assert(state->hw->pcie_core.settings.highest_visible_pf < state->cfgspc_state->settings.max_pfs);
      for (pf = state->hw->pcie_core.settings.highest_visible_pf; pf < state->cfgspc_state->settings.max_pfs; pf++) {
        fs_pci_core_resource_t *pf_resource;
        pf_resource = find_pfvf_resource(state->cfgspc_state, pf, VF_NONE);
        assert(pf_resource != NULL);
        pf_resource->visible = false;
      }
    }
    break;
#endif
#ifdef PCMCR_PF_HIDE_CONTROL_REG
    case PCMCR_PF_HIDE_CONTROL_REG: {
        unsigned int pf;
        int prev_vis = 0;
        for (pf = 0; pf <= state->hw->pcie_core.settings.highest_visible_pf; pf++) {
        fs_pci_core_resource_t *pf_resource;
          pf_resource = find_pfvf_resource(state->cfgspc_state, pf, VF_NONE);
          assert(pf_resource != NULL);
          if (((val >> (pf * 2)) & 3) == PCMCFE_PFVISIBLE) {
            fs_pci_core_resource_t *prev_pf = find_pfvf_resource(state->cfgspc_state, prev_vis, VF_NONE);
            pf_resource->visible = true;
            put_cfg_8(prev_pf, PCMCR_ARI_CAP_REG + 1, pf);
            put_cfg_8(pf_resource, PCMCR_ARI_CAP_REG + 1, 0);
            prev_vis = pf;
          }
        }
      }
      break;
#endif
    default:
      if (dpa_cap != 0 && addr == dpa_cap + DPA_STATUS_CNTRL_REG) {
        dpa_ctl_stat_write(state, resource, val, true, true, from_mc);
        return; /* dpa_ctl_stat_write handles the update */
      } else if (in_cap_range(addr, msix_cap, PCI_CAP_MSIX_SIZEOF)) {
        put_cfg_32(resource, addr, val); /* Get config space to its target starte before update_msix_settings */
        update_msix_settings(state, resource);
        return;  
      } else if (in_cap_range(addr, sriov_xcap, PCI_EXT_CAP_SRIOV_SIZEOF)) {
        unsigned int sriov_offset = addr - sriov_xcap;

        switch (sriov_offset) {
        case SRIOV_CAP_OFFSET_BAR(0):
        case SRIOV_CAP_OFFSET_BAR(2):
          val = pcie_update_bar_mappings(state, resource,
                                         SRIOV_CAP_OFFSET_BAR(0),
                                         sriov_offset,
                                         val);
          break;

        case SRIOV_CAP_OFFSET_BAR(1):
        case SRIOV_CAP_OFFSET_BAR(3):
        case SRIOV_CAP_OFFSET_BAR(4):
        case SRIOV_CAP_OFFSET_BAR(5):
          val = 0;
          break;
        default:
          fs_log("SRIOV Ext CAP address: MAYBE IMPLEMENT ME 0x%03x (16)\n",
                 sriov_offset);
          break;
        }
      } else if (in_cap_range(addr,
                              state->cfgspc_state->vpd_cap, PCI_VPD_DATA)) {
        pcie_cfgspc_cb_trigger_vpd(state->cfgspc_state, resource->br->lies->func,
                          state->cfgspc_state->vpd_cap, (val >> 16));
        /* fs_hw_trigger_vpd may synchronously update config space, so we do *not* want the
          * put_cfg_32 below to execute */
        return;
      } else {
        fs_log("MAYBE IMPLEMENT ME 0x%03x (32)\n", addr);
      }
      break;
    }
    put_cfg_32(resource, addr, val);
  }
}


pcie_func_t *fs_rid_to_func(pcie_state_t *state, unsigned rid)
{
  if (rid >= state->cfgspc_state->settings->max_total_funcs || 
      state->cfgspc_state->bus_state[rid].lies == NULL)
    return NULL;
  return state->cfgspc_state->bus_state[rid].lies->func;
}

static pcie_func_t *host_access_decode(pcie_state_t *state,
                                       fs_pci_addr_space_t *addr_space,
                                       uint64_t *addr,
                                       pci_bar_type_t *bar_type,
                                       pci_bar_num_t *bar_num)
{
   pci_bar_t *bar, **result;
   dump_addr_space(addr_space);
   result = bsearch(addr, addr_space->bar, addr_space->configured_bars,
                    sizeof(addr_space->bar[0]), bar_finder);
   Xassert(result);
   
   bar =  *( pci_bar_t **)result;

   FS_PCI_DEBUG("Found BAR %d, position at 0x%lx : 0x%lx\n", bar->type, bar->start, bar->end);
   FS_PCI_DEBUG("RID is %d pf/vf %d/%d %02x:%02x.%x\n",
                bar->owner->func->rid, bar->owner->func->pf_num, bar->owner->func->vf_num,
                bar->owner->func->bus,  bar->owner->func->dev,  bar->owner->func->func);


   if (bar->num >= PCI_SRIOV_BAR0 && bar->num <= PCI_SRIOV_BAR5) {
     fs_pci_core_resource_t *pf = bar->owner->core, *vf;
     unsigned addr_mask, offset, vfno;
     addr_mask =  pf->bar_mask[bar->num] | 0xf; /* Retrieve the per-VF size, not the aperture size, bottom bits are type info. */
     offset = *addr  - bar->start;
     vfno = offset / (addr_mask + 1);
     vf = find_pfvf_resource(state->cfgspc_state, pf->pf, vfno);
     FS_PCI_DEBUG("Need to fix up SRIOV BAR access. Addr = 0x%lx , offset 0x%x VFno = %d VF = %p\n", *addr, offset, vfno, vf);
     *addr = *addr & addr_mask;
     *bar_type = bar->type;
     FS_PCI_DEBUG("RID is %d pf/vf %d/%d %02x:%02x.%x\n", vf->br->lies->func->rid, vf->br->lies->func->pf_num, vf->br->lies->func->vf_num, 
        vf->br->lies->func->bus,  vf->br->lies->func->dev,  vf->br->lies->func->func);
    if (bar_num != NULL)
      *bar_num = bar->num - PCI_SRIOV_BAR0;
     return vf->br->lies->func;
   }      
   *addr = *addr & (bar->end - bar->start);
   *bar_type = bar->type;
   if (bar_num != NULL)
     *bar_num = bar->num;
   return bar->owner->func;
}

pcie_func_t *host_mem_access_decode(pcie_state_t *state,
                                    uint64_t *addr,
                                    pci_bar_type_t *bar_type,
                                    pci_bar_num_t *bar_num)
{
  pcie_func_t *f = host_access_decode(state, &state->cfgspc_state->mem_addr_space, addr, bar_type,
    bar_num);
  return f;
}

pcie_func_t *host_io_access_decode(pcie_state_t *state,
                                   unsigned *addr,
                                   pci_bar_type_t *bar_type,
                                   pci_bar_num_t *bar_num)
{
  uint64_t addr64 = *addr;
  pcie_func_t *f =  host_access_decode(state, &state->cfgspc_state->io_addr_space, &addr64, bar_type,
    bar_num);
  *addr = addr64;
  return f;
}

static const char *pci_cfg_reg_name(unsigned int addr, bool is_vf, bool from_mc)
{
  return "Some reg";
}

/* PCI config space read hook, called by mcsim core for PCIe management
 * accesses.
 */
uint32_t fs_pci_cfg_read(mc_t *mc, pcie_state_t *state, unsigned int pf,
                         unsigned int vf, bool vf_active, unsigned int addr)
{
  fs_pci_core_resource_t *resource;
  uint32_t val;

  resource = find_pfvf_resource(state->cfgspc_state, pf, vf_active ? vf : VF_NONE);
  if (resource == NULL) {
    if (vf_active)
      fs_log("Warning: ignoring config read from PF%u/VF%u 0x%03x %s\n", pf,
             vf, addr, pci_cfg_reg_name(addr, true, true));
    else
      fs_log("Warning: ignoring config read from PF%u 0x%03x %s\n", pf, addr,
             pci_cfg_reg_name(addr, false, true));
    return 0;
  }

#ifdef FMCR_PTM_RQST_LOCAL_CLOCK_LO_REG /* FIXME: once PTM is merged, should become M2 specific instead. */
  if(addr==PCMCR_PTM_CONTROL_REG){
    val = ptm_config_read(mc, addr);
  } else {
      val = handle_cfg_read_32(resource, addr, CFG_ACCESS_MC);
  }
#else
  val = handle_cfg_read_32(resource, addr, CFG_ACCESS_MC);
#endif


  if (vf_active)
    FS_PCI_DEBUG("@@ PCI CFG READ: PF%u/VF%u addr 0x%03x = 0x%08x %s\n", pf,
                 vf, addr, val, pci_cfg_reg_name(addr, true, true));
  else
    FS_PCI_DEBUG("@@ PCI CFG READ: PF%u addr 0x%03x = 0x%08x %s\n", pf, addr,
                 val, pci_cfg_reg_name(addr, false, true));

  return val;
}

/* Called by the PCIe core only */
uint32_t fs_host_pci_cfg_read(pcie_state_t *state, unsigned int pf,
                         unsigned int vf, bool vf_active, unsigned int addr)
{
  return fs_pci_cfg_read(NULL, state, pf, vf, vf_active, addr);
}

static void _fs_pci_cfg_write(mc_t *mc, pcie_state_t *state, unsigned int pf,
                      unsigned int vf, bool vf_active, unsigned int addr,
                      bool cs2, unsigned int be, uint32_t val,
                      fs_pci_cfg_access_type_t access_type)
{
  fs_pci_core_resource_t *resource;
  int bits;
  

  resource = find_pfvf_resource(state->cfgspc_state, pf, vf_active ? vf : VF_NONE);
  if (!resource)
    return;

  /* Turn 32-bit aligned address + byte enables into something more useful: */
  switch (be) {
    /* 8-bit cases: */
  case 0x1:
    val = val & 0x000000ff;
    bits = 8;
    break;
  case 0x2:
    val = (val & 0x0000ff00) >> 8;
    addr += 1;
    bits = 8;
    break;
  case 0x4:
    val = (val & 0x00ff0000) >> 16;
    addr += 2;
    bits = 8;
    break;
  case 0x8:
    val = (val & 0xff000000) >> 24;
    addr += 3;
    bits = 8;
    break;

    /* Aligned 16-bit cases: */
  case 0x3:
    val = val & 0x0000ffff;
    bits = 16;
    break;
  case 0xc:
    val = (val & 0xffff0000) >> 16;
    addr += 2;
    bits = 16;
    break;

    /* The 32-bit case: */
  case 0xf:
    bits = 32;
    break;

    /* We don't handle anything else */
  default:
    bits = 0;
    ERR("*** Can't cope with write to 0x%03x with BE 0x%x\n", addr, be);
  }

  if (resource->rid == RID_NONE) {
    fs_log("Warning: squashing access to function that has never been set up (see below).\n");
    resource = NULL;
  }

  if (resource == NULL) {
    if (vf_active)
      fs_log("Warning: ignoring config write to PF%u/VF%u 0x%03x\n", pf, vf,
             addr);
    else
      fs_log("Warning: ignoring config write to PF%u 0x%03x\n", pf, addr);
    return;
  }

  if (vf_active)
    FS_PCI_DEBUG("@@ PCI CFG WRITE: PF%u/VF%u addr 0x%03x%s = 0x%0*x %s\n", pf,
                 vf, addr, cs2 ? "[CS2]" : "", bits / 4, val,
                 pci_cfg_reg_name(addr, true, true));
  else
    FS_PCI_DEBUG("@@ PCI CFG WRITE: PF%u addr 0x%03x%s = 0x%0*x %s\n", pf,
                 addr, cs2 ? "[CS2]" : "", bits / 4, val,
                 pci_cfg_reg_name(addr, false, true));

  switch (bits) {
  case 8:
    handle_cfg_write_8(state, resource, addr, val, access_type);
    break;
  case 16:
    handle_cfg_write_16(state, resource, addr, val, access_type);
    break;
  case 32:
#ifdef FMCR_PTM_RQST_LOCAL_CLOCK_LO_REG /* FIXME: once PTM is merged, should become M2 specific instead. */
    if(addr==PCMCR_PTM_CONTROL_REG)  {
      ptm_config_write(mc, addr,val);
    } else {
      handle_cfg_write_32(state, resource, addr, val, access_type);
    }
    break;
#else
    handle_cfg_write_32(state, resource, addr, val, access_type);
    break;
#endif
  default:
    ERR("Bad PCI config write (can't happen!)\n");
  }
}
/* PCI config space write hook, called by mcsim core for PCIe management
 * accesses.
 */
void fs_pci_cfg_write(mc_t *mc, pcie_state_t *state, unsigned int pf,
                      unsigned int vf, bool vf_active, unsigned int addr,
                      bool cs2, unsigned int be, uint32_t val)
{
  fs_pci_cfg_access_type_t access_type;
  access_type = cs2 ? CFG_ACCESS_MC_CS2 : CFG_ACCESS_MC;
  _fs_pci_cfg_write(mc, state, pf, vf, vf_active, addr, cs2, be, val, access_type);
}

void fs_host_pci_cfg_write(pcie_state_t *state, unsigned int pf,
                      unsigned int vf, bool vf_active, unsigned int addr,
                      bool cs2, unsigned int be, uint32_t val)
{
  _fs_pci_cfg_write(NULL, state, pf, vf, vf_active, addr, cs2, be, val, CFG_ACCESS_HOST);
}


/* PCI config retry enable control, called when the relevant GPIO register is
 * written.
 */
void fs_pci_cfg_retry_en(pcie_state_t *state, bool en_set, bool en_reset)
{
  if (en_set && en_reset)
    ERR("FW error?  PCI config retry set and reset simultaneously!\n");

  if (en_reset)
    state->retry_en = false;
 
  if (en_set)
    state->retry_en = true;
}


void fs_pci_assert_intx(pcie_state_t *state, pcie_func_t *func, bool onoff)
{
  fs_pci_core_resource_t *resource = state->cfgspc_state->bus_state[func->rid].core;
  uint16_t status;
  uint8_t pin = get_cfg_8(resource, PCI_INTERRUPT_PIN);
  FS_PCI_DEBUG("INT %d -> %d\n", pin, onoff);
  status = get_cfg_16(resource, PCI_STATUS);
  if (onoff)
    status |= PCI_STATUS_INTERRUPT;
  else
    status &= ~PCI_STATUS_INTERRUPT;
  put_cfg_16(resource, PCI_STATUS, status);
  fi_pcie_assert_intx(state, pin - 1, onoff);
}

uint32_t pcie_max_payload(pcie_state_t *state)
{
  fs_pci_core_resource_t *resource = &state->cfgspc_state->pf_resources[0];
  uint16_t pcie_cap = pcie_find_cap(PCI_CAP_ID_EXP, resource->config_space);
  uint32_t dev_ctl;

  assert(pcie_cap);
  dev_ctl = get_cfg_32(resource, pcie_cap + PCI_EXP_DEVCTL);

  switch (dev_ctl & PCI_EXP_DEVCTL_PAYLOAD) {
  case PCI_EXP_DEVCTL_PAYLOAD_128B:
    return 128;
  case PCI_EXP_DEVCTL_PAYLOAD_256B:
    return 256;
  case PCI_EXP_DEVCTL_PAYLOAD_512B:
    return 512;
  case PCI_EXP_DEVCTL_PAYLOAD_1024B:
    return 1024;
  case PCI_EXP_DEVCTL_PAYLOAD_2048B:
    return 2048;
  case PCI_EXP_DEVCTL_PAYLOAD_4096B:
    return 4096;
  default:
    assert(0);
  }
}

uint32_t pcie_max_read_req_size(pcie_state_t *state)
{
  fs_pci_core_resource_t *resource = &state->cfgspc_state->pf_resources[0];
  uint16_t pcie_cap = pcie_find_cap(PCI_CAP_ID_EXP, resource->config_space);
  uint32_t dev_ctl;

  assert(pcie_cap);
  dev_ctl = get_cfg_32(resource, pcie_cap + PCI_EXP_DEVCTL);

  switch (dev_ctl & PCI_EXP_DEVCTL_READRQ) {
  case PCI_EXP_DEVCTL_READRQ_128B:
    return 128;
  case PCI_EXP_DEVCTL_READRQ_256B:
    return 256;
  case PCI_EXP_DEVCTL_READRQ_512B:
    return 512;
  case PCI_EXP_DEVCTL_READRQ_1024B:
    return 1024;
  case PCI_EXP_DEVCTL_READRQ_2048B:
    return 2048;
  case PCI_EXP_DEVCTL_READRQ_4096B:
    return 4096;
  default:
    assert(0);
  }
}

bool pcie_retry_active(pcie_state_t *state)
{
  return state->retry_en;
}

const uint8_t  *fs_pcie_get_cfgspc(pcie_state_t *state, unsigned int pf, unsigned int vf, bool use_vf)
{
  fs_pci_core_resource_t *res = find_pfvf_resource(state->cfgspc_state, pf,
                                                   use_vf ? vf : VF_NONE);
  return res != NULL ? res->config_space : NULL;
}

uint16_t fs_pcie_get_cfgspc_xcap_vsec(pcie_state_t *state)
{
  pcie_cfgspc_state_t *cfgspc_state = state->cfgspc_state;

  return cfgspc_state->vsec_cap;
}
