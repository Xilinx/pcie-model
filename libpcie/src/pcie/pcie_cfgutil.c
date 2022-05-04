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

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <linux/pci_regs.h>
#include "pcie/pcie_cfgutil.h"

#define XCAP_START_OFFS (0x100)

/* Layout of an extended capability header. */
#define      PCMCRF_XCAP_NXT_PTR_LBN 20
#define      PCMCRF_XCAP_NXT_PTR_WIDTH 12
#define      PCMCRF_XCAP_VER_LBN  16
#define      PCMCRF_XCAP_VER_WIDTH 4
#define      PCMCRF_XCAP_ID_LBN  0
#define      PCMCRF_XCAP_ID_WIDTH 16

#ifndef PCI_CFG_SPACE_SIZE
#define PCI_CFG_SPACE_SIZE 256
#endif

static uint32_t default_write_mask[PCI_CFG_SPACE_EXP_SIZE/sizeof(uint32_t)] = {
  /*DEV/VEND  CMD/STAT    CLS/REV     Stuff        BAR0       BAR1        BAR2        BAR3 */
  0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
  /* BAR4      BAR5       Cardbus     SubsysID     Exprom     Caps ptr    Rsvd        int pin/line */
  0xffffffff, 0xffffffff, 0x00000000, 0x00000000, 0xffffffff, 0x00000000, 0x00000000, 0x000000ff,
};

static const uint32_t sriov_mask[] = {
  /* Cap hdr   cap        ctrl/stat   init/tot   num/lnk      1st/stride  rsvd/devid  suppg */
  0x00000000, 0x00000000, 0x0000ffff, 0x00000000, 0x0000ffff, 0x00000000, 0x00000000, 0x00000000,
  /* syspg    BAR0        BAR1        BAR2        BAR3        BAR4        BAR5 */
  0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

static const uint16_t msix_mask[] = {
  /* Cap hdr   msix control reg */
  0x0000,      0xffff,
};

static const uint32_t pcie_mask[] = {
  /* Cap       dev_cap    dev_ctl */
  0x00000000, 0x00000000, 0xffffffff,
  /* Link Capabilities */
  0x00000000,
  /* Link control / Link status */
  0x0000FFFF,
};

static const uint32_t acs_mask[] = {
  /* Cap hdr  cap/ctrl    egress control*/
  0x00000000, 0x00ff0000, 0xffffffff
};

static const uint32_t pm_mask[] = {
  /* Cap      PM ctrl/stat */
  0x00000000, 0x0000ffff,
};

uint8_t pcie_find_cap(unsigned cap, const uint8_t *cfgspc)
{
  uint8_t pos = pcie_get_cfg_8(cfgspc, PCI_CAPABILITY_LIST);
  while(pos != 0) {
    uint8_t id = pcie_get_cfg_8(cfgspc, pos);
    if (id == cap) {
      return pos;
    }
    pos = pcie_get_cfg_8(cfgspc, pos + 1);
  }
  return 0;
}

uint16_t pcie_find_xcap(unsigned cap, const uint8_t *cfgspc)
{
  uint16_t pos = XCAP_START_OFFS;
  while(pos != 0) {
    uint32_t hdr = pcie_get_cfg_32(cfgspc, pos);
    uint16_t id = PCI_EXT_CAP_ID(hdr);
    if (id == cap) {
      return pos;
    }
    pos = PCI_EXT_CAP_NEXT(hdr);
  }
  return 0;
}

void pcie_hide_cap(unsigned cap, uint8_t *cfgspc)
{
  uint8_t prev_next_loc = PCI_CAPABILITY_LIST;
  uint8_t pos;
  while(pcie_get_cfg_8(cfgspc, prev_next_loc) != 0) {
    pos = pcie_get_cfg_8(cfgspc, prev_next_loc);
    uint8_t id = pcie_get_cfg_8(cfgspc, pos);
    if (id == cap) {
      uint8_t next = pcie_get_cfg_8(cfgspc, pos + 1);
      pcie_put_cfg_8(cfgspc, prev_next_loc, next);
      return;
    }
    prev_next_loc = pos + 1;
  }
  Xassert(0);
}

void pcie_hide_xcap(unsigned cap, uint8_t *cfgspc)
{
  uint16_t prev_next_loc = XCAP_START_OFFS;
  uint16_t pos = XCAP_START_OFFS;
  while(pos != 0) {
    uint32_t hdr = pcie_get_cfg_32(cfgspc, pos);
    uint16_t id = PCI_EXT_CAP_ID(hdr);
    if (id == cap) {
      uint32_t prev_hdr = pcie_get_cfg_32(cfgspc, prev_next_loc);
      uint16_t next = PCI_EXT_CAP_NEXT(hdr);
      BITFIELD_SET(prev_hdr, PCMCRF_XCAP_NXT_PTR, next);
      pcie_put_cfg_32(cfgspc, prev_next_loc, prev_hdr);
      return;
    }
    prev_next_loc = pos;
    pos = PCI_EXT_CAP_NEXT(hdr);
  }
  Xassert(0);
}

/* PCI capabilities are 4-byte-aligned */
#define ALIGN_UP_4(x)   (((x) + 7) & ~7)

/* For the purposes of cap manipulation code here, the header
 * is just the ID and next pointer. The 2 bytes that must follow
 * per the standard are passed in at the start of 'data' */
#define CAP_HDR_BYTES (2)

static void write_cap(uint8_t *cfgspc, uint8_t where, uint8_t id,
                      const uint8_t *data, unsigned len, bool last)
{
  cfgspc[where + PCI_CAP_LIST_ID] = id;
  cfgspc[where + PCI_CAP_LIST_NEXT] = last ? 0 : ALIGN_UP_4(where + len + CAP_HDR_BYTES);
  memcpy(cfgspc + where + CAP_HDR_BYTES, data, len);
}

/* The lack of a length of the last cap means that the only way to add
 * a new cap without wasting lots of space (which we don't have in PCI
 * config space) is to add it at the start and relocate  the original
 * cap chain to follow. */
void pcie_prepend_cap(uint8_t *cfgspc, uint8_t capid, const uint8_t *data, unsigned len)
{
  uint8_t backup[256];
  uint8_t new_cap, old_cap, next;
  uint8_t start = pcie_get_cfg_8(cfgspc, PCI_CAPABILITY_LIST);
  bool done = false;
  memcpy(backup, cfgspc, PCI_CFG_SPACE_SIZE);
  new_cap = start;
  old_cap = start;
  write_cap(cfgspc, new_cap, capid, data, len, false);
  new_cap = ALIGN_UP_4(new_cap + len + CAP_HDR_BYTES);
  next = backup[old_cap + PCI_CAP_LIST_NEXT];
  do {
    uint8_t id = pcie_get_cfg_8(backup, old_cap);
    done = next == 0;
    /* We have no way of knowing the length of the last cap (short of being able to parse
     * every known cap type), so we just copy as much of the backed up config space as
     * will fit for the last cap. Either this works or there wasn't room for the new cap
     * in the first place. */
    len = done ? 255 - new_cap - len - CAP_HDR_BYTES : next - old_cap - CAP_HDR_BYTES;
    write_cap(cfgspc, new_cap, id, backup + old_cap + CAP_HDR_BYTES, len, done);
    old_cap = ALIGN_UP_4(old_cap + len + CAP_HDR_BYTES);
    new_cap = ALIGN_UP_4(new_cap + len + CAP_HDR_BYTES);
    next = pcie_get_cfg_8(backup, old_cap + PCI_CAP_LIST_NEXT);
  } while(!done);
}

#define MAX_PLAUSIBLE_XCAP_SIZE (256)

/* Actually return the correct size in some cases, otherwise something
 * known to be big enough. This saves space that matters in a few cases,
 * feel free to extend as needed. */
static int xcap_size(uint8_t id)
{
  int size;
  switch(id) {
    case PCI_EXT_CAP_ID_DSN:
      size = PCI_EXT_CAP_DSN_SIZEOF;
      break;
    case PCI_EXT_CAP_ID_PWR:
      size = PCI_EXT_CAP_PWR_SIZEOF;
      break;
    case PCI_EXT_CAP_ID_ARI:
      size = PCI_EXT_CAP_ARI_SIZEOF;
      break;
    case PCI_EXT_CAP_ID_ATS:
      size = PCI_EXT_CAP_ATS_SIZEOF;
      break;
    case PCI_EXT_CAP_ID_PRI:
      size = PCI_EXT_CAP_PRI_SIZEOF;
      break;
    case PCI_EXT_CAP_ID_ACS:
      size = 8; /* No suitable #define in header */
      break;
    default:
      size = MAX_PLAUSIBLE_XCAP_SIZE;
      break;
  }
  return size;
}

uint32_t pcie_make_xcap_hdr(uint16_t capid, uint8_t ver, uint16_t next)
{
  uint32_t hdr = 0;

  BITFIELD_SET(hdr, PCMCRF_XCAP_ID, capid);
  BITFIELD_SET(hdr, PCMCRF_XCAP_VER, ver);
  BITFIELD_SET(hdr, PCMCRF_XCAP_NXT_PTR, next);

  return hdr;
}

void pcie_append_xcap(uint8_t *cfgspc, uint16_t capid, uint8_t ver, const uint8_t *data, unsigned len)
{
  uint32_t hdr;
  uint16_t last, pos = XCAP_START_OFFS;
  uint8_t last_id;

  /* Find the last xcap in the existing chain */
  while(pos != 0) {
    hdr = pcie_get_cfg_32(cfgspc, pos);
    last = pos;
    pos = PCI_EXT_CAP_NEXT(hdr);
    last_id = PCI_EXT_CAP_ID(hdr);
  }

  /* Rewrite the next pointer of the last xcap. */
  pos = last + xcap_size(last_id); /* This is where the new xcap will go */
  Xassert_lt(pos + len, 0x1000); /* Not off the end of config space? */
  hdr = pcie_get_cfg_32(cfgspc, last);
  BITFIELD_SET(hdr, PCMCRF_XCAP_NXT_PTR, pos);
  pcie_put_cfg_32(cfgspc, last, hdr);

  /* Set up the header for the new cap, setting NXT to 0 */
  pcie_put_cfg_32(cfgspc, pos, pcie_make_xcap_hdr(capid, ver, 0));

  /* Copy in the opaque data */
  memcpy(cfgspc + pos + 4, data, len);
}


void pcie_walk_caps(uint8_t *cfgspc, pci_cap_cb cb, void *cb_context)
{
   uint16_t pos = pcie_get_cfg_8(cfgspc, PCI_CAPABILITY_LIST);
   while(pos) {
     uint8_t next = pcie_get_cfg_8(cfgspc, pos + PCI_CAP_LIST_NEXT);
     cb(cfgspc, pos, cb_context);
     pos = next;
   }
}

void pcie_walk_xcaps(uint8_t *cfgspc, pci_cap_cb cb, void *cb_context)
{
   uint16_t pos = XCAP_START_OFFS;
   while(pos) {
     uint32_t hdr = pcie_get_cfg_32(cfgspc, pos);
     cb(cfgspc, pos, cb_context);
     pos = PCI_EXT_CAP_NEXT(hdr);
   }
}

static void cap_fixer(uint8_t *cfgspc, uint16_t offset, void *context)
{
  uint8_t id = pcie_get_cfg_8(cfgspc, offset);
  switch(id) {
    case PCI_CAP_ID_MSIX:
      memcpy((uint8_t *)context + offset, msix_mask, sizeof(msix_mask));
      break;
    case PCI_CAP_ID_EXP:
      memcpy((uint8_t *)context + offset, pcie_mask, sizeof(pcie_mask));
      break;
    case PCI_CAP_ID_PM:
      memcpy((uint8_t *)context + offset, pm_mask, sizeof(pm_mask));
      break;
  }
}

static void xcap_fixer(uint8_t *cfgspc, uint16_t offset, void *context)
{
  uint16_t id = pcie_get_cfg_32(cfgspc, offset);
  switch(id) {
    case PCI_EXT_CAP_ID_SRIOV:
      memcpy((uint8_t *)context + offset, sriov_mask, sizeof(sriov_mask));
      break;
    case PCI_EXT_CAP_ID_ACS:
      memcpy((uint8_t *)context + offset, acs_mask, sizeof(acs_mask));
      break;
  }
}

void pcie_cfgspc_fixup_write_mask(uint8_t *config_space, uint32_t *write_mask)
{
  memcpy(write_mask, default_write_mask, sizeof(default_write_mask));

  pcie_walk_caps(config_space, cap_fixer, write_mask);
  pcie_walk_xcaps(config_space, xcap_fixer, write_mask);
}

bool pcie_cfgspc_verify_write_access(uint32_t *write_mask,
                                     unsigned int addr,
                                     unsigned int bits,
                                     bool is_vf,
                                     bool from_mc)
{
  if (addr >= PCI_CFG_SPACE_EXP_SIZE)
    return false;
  if (from_mc)
    return true;
  switch(bits) {
    case 32:
      return write_mask[addr/4] == 0xffffffff;
      break;
    case 16:
      return ((uint16_t *)write_mask)[addr/2] == 0xffff;
      break;
    case 8:
      return ((uint8_t *)write_mask)[addr] == 0xff;
      break;
    default:
      assert(0);
      break;
  }
  return false;
}

static void ari_fixer(uint8_t *cfgspc, uint16_t offset, void *context)
{
  uint16_t id = pcie_get_cfg_32(cfgspc, offset);
  uint8_t next_devfn = *(uint8_t *)(context);

  if (id == PCI_EXT_CAP_ID_ARI)
    pcie_put_cfg_8(cfgspc, offset + PCI_ARI_CAP + 1, next_devfn);
}

void pcie_cfgspc_fill_ari_next(uint8_t *config_space, uint8_t next_devfn)
{
  pcie_walk_xcaps(config_space, ari_fixer, &next_devfn);
}
