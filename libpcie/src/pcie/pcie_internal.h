/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Pavan Prerepa,
 *            Charlie Palmer,
 *            Paul Burton,
 *            Jackson Rigby,
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
#ifndef PCIE_INTERNAL_H
#define PCIE_INTERNAL_H

#include <pthread.h>
#include "pcie_api.h"
#include "dpi_platform.h"

/* FIXME: naming should be cleaned up */
#define fs_log printf

typedef struct pcie_tlp_state_s pcie_tlp_state_t;

/* Used by pseudocore.c to track requests */
typedef struct {
  uint32_t buffer[1024];
  void *dut_buffer;
  pthread_cond_t cond;
  pthread_mutex_t mutex;
  int start_offset;
  int end_offset;
  int len;
  void *context;
  bool ok;
} read_req;

typedef struct pcie_state_s {
  struct fs_hw_s *hw;
  pcie_intf_t instance;
  struct pcie_cfgspc_state_s *cfgspc_state;

  bool retry_en;

  bool process_tlp_once;
  tag_type_t tag_type;

  /* Request handling machinery */

  /* The hw and pci threads both need to send TLPs. The lock is needed
   * because the header call for one TLP must be followed by the data for
   * that TLP, without another header being interleaved. This problem
   * goes away if we let pcie.c diverge from the cosim one and change the
   * send API to accept both the header and the optional associated data */
  pthread_mutex_t send_mutex;
  read_req requests_by_tag[PCI_MAX_TAGS_SUPPORTED];

  pcie_tlp_info_t
      pending_host_reads[PCI_MAX_TAGS_SUPPORTED]; /* indexed by tag */

  /* Somewhat fictional, but monotonic, timestamp */
  uint64_t ts;
  /* The bus on which we are an endpoint */
  uint16_t busnum;

  /* We have to deal with the case where we aren't the only devices on the bus
  * (QEMU with --nomagicbus). We assume that enumeration proceeds in a vaguely
  * sane fashion, and that the CID of the very first config. space access we 
  * see gives us the devfn of our lowest-numbered device. */
  unsigned rid_offset;
  bool rid_offset_done;

  /* Low-level TLP machinery */
  pcie_tlp_state_t *tlp;

  /* Platform-specific cosim machinery */
  cosim_platform_state_t *cosim_state;

  /* Configuration space handler */
  const pcie_cfg_handler_t *cfg_handler;
  void *cfg_handler_user;

  /* Whether to enable transactions to target the pseudo-RC */
  bool pseudo_rc_enabled;

  pcie_core_callbacks_t *core_cbs;

} pcie_state_t;

/* Returns the address truncated to an offset */
extern pcie_func_t *host_mem_access_decode(pcie_state_t *state,
                                           uint64_t *addr,
                                           pci_bar_type_t *bar,
                                           pci_bar_num_t *bar_num);
extern pcie_func_t *host_io_access_decode(pcie_state_t *state,
                                          unsigned *addr,
                                          pci_bar_type_t *bar,
                                          pci_bar_num_t *bar_num);


extern uint32_t fs_host_pci_cfg_read(pcie_state_t *state, unsigned int pf,
                                unsigned int vf, bool vf_active,
                                unsigned int addr);

extern void fs_host_pci_cfg_write(pcie_state_t *state, unsigned int pf,
                      unsigned int vf, bool vf_active, unsigned int addr,
                      bool cs2, unsigned int be, uint32_t val);



extern void fi_pcie_assert_intx(pcie_state_t *state, uint8_t pin, bool onoff);


extern void init_pcie(pcie_state_t *state);

extern void pcie_tlp_init(pcie_state_t *state);

extern void pcie_reset(pcie_state_t *state);

/* Low-level TLP calls */

extern void host_to_dut_tlphdr_c_model(pcie_state_t *state,
                                uint64_t time,
                                /* OUTPUT */int *valid,
                                /* OUTPUT */int *eop,
                                /* OUTPUT */int *tlp_fmt,
                                /* OUTPUT */int *tlp_type,
                                /* OUTPUT */int *cpl_status,
                                /* OUTPUT */int *ep,
                                /* OUTPUT */int *addr_hi,
                                /* OUTPUT */int *addr_lo,

                                /* OUTPUT */int *rqstr_id,
                                /* OUTPUT */int *cpltr_id,
                                /* OUTPUT */int *first_dw_be,
                                /* OUTPUT */int *last_dw_be,
                                /* OUTPUT */int *tag,
                                /* OUTPUT */int *num_dwords,
                                /* OUTPUT */int *bcnt,
                                /* OUTPUT */int *attr,
                                /* OUTPUT */int *tc,
                                /* OUTPUT */int *at,

                                /* OUTPUT */int *tph_on,
                                /* OUTPUT */int *tph_type,
                                /* OUTPUT */int *st_tag,
                                /* OUTPUT */int *msg_code,

                                /* OUTPUT */int *obff_code,
                                /* OUTPUT */ uint32_t *hdr,
                                /* OUTPUT */int *pcredits
                                      );

extern void dut_to_host_tlphdr_c_model(pcie_state_t *state,
                                 uint64_t time,
                                int valid,
                                int eop,
                                int tlp_fmt,
                                int tlp_type,
                                int cpl_status,
                                int ep,
                                int addr_hi,
                                int addr_lo,

                                int rqstr_id,
                                int cpltr_id,
                                int first_dw_be,
                                int last_dw_be,
                                int tag,
                                int num_dwords,
                                int bcnt,
                                int attr,
                                int tc,
                                int at,

                                int tph_on,
                                int tph_type,
                                int st_tag,
                                int msg_code,

                                int ltr_snp_lat,
                                int ltr_no_snp_lat);

extern void dut_to_host_tlpdat_c_model(pcie_state_t *state,
                                uint64_t time,
                                int valid,
                                int eop,
                                int data);

extern void dut_to_host_tlpdat_buf_c_model(pcie_state_t *state,
                                           uint64_t time,
                                           int valid,
                                           void *data,
                                           uint32_t word_len);

extern void host_to_dut_tlpdat_c_model(pcie_state_t *state,
                                       uint64_t time,
                                      /* OUTPUT */int *valid,
                                      /* OUTPUT */int *eop,
                                      /* OUTPUT */int *data);
extern void host_to_dut_tlpdat_buf_c_model(pcie_state_t *state,
                                           uint64_t time,
                                           /* OUTPUT */int *valid,
                                           /* OUTPUT */void *data,
                                           uint32_t word_len);

extern void prod_pcie(void *context);

/* Returns true if there is a completion in dest_buffer */
extern bool pcie_tlp_to_host(pcie_state_t *state,
                             uint32_t rid,
                             pcie_tlp_req_t *req,
                             void *src_buffer,
                             void *dest_buffer);


extern pcie_func_t *fs_pci_pfvf_to_func(pcie_state_t *state,
                                        unsigned int pf,
                                        unsigned int vf,
                                        bool vf_active);

extern bool fs_pci_intx_asserted(pcie_state_t *state, pcie_func_t *func);

static inline bool fs_func_is_pf(pcie_func_t *f)
{
  return f->vf_num == (int) VF_NONE;
}

#endif

