/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Pavan Prerepa,
 *            Charlie Palmer,
 *            Paul Burton,
 *            Dmitriy Bulavskiy,
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <linux/pci.h>
#include <stddef.h>
#ifndef CONFIG_TLM
  /*
   * The PCIe model in SystemC / TLM 2.0 simulations forwards TLPs for further
   * processing through fs_host_w32 / fs_host_r32.
   */
  #include "lib/hwfifo.h"
#endif
#include "sfc_cosim_comms.h"
#include "cosim_interface.h"
#include "cosim_utils.h"
#include "pcie_api.h"
#include "pcie_internal.h"

#define FI_VERBOSE 0

#define container_of(c_type, mbr_name, p_mbr)  \
  ( (c_type*) ((char*)(p_mbr) - offsetof(c_type, mbr_name)) )

void fi_log(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);
}


#if FI_VERBOSE
#define SEND_LOCK(state_) do { fi_log("%s: lock\n", __FUNCTION__); pthread_mutex_lock(&state_->send_mutex); } while(0)
#define SEND_UNLOCK(state_) do { fi_log("%s: unlock\n", __FUNCTION__); pthread_mutex_unlock(&state_->send_mutex); } while(0)
#else
#define SEND_LOCK(state_) do {  pthread_mutex_lock(&state_->send_mutex); } while(0)
#define SEND_UNLOCK(state_) do { pthread_mutex_unlock(&state_->send_mutex); } while(0)

#endif

static void cfg_read(const pcie_tlp_info_t *tlp_info, void *context);
static void cfg_write(const pcie_tlp_info_t *tlp_info, unsigned data, void *context);

static void pcie_core_cb_pcie_vdm(pcie_state_t *s,
                                  struct fs_hw_s *hw, uint32_t *hdr,
                                  uint32_t *payload, int payload_dwords)
{
  pcie_core_callbacks_t *cbs = s->core_cbs;

  cbs->pcie_vdm(hw, hdr, payload, payload_dwords);
}

static unsigned pcie_core_cb_host_w32(pcie_state_t *s,
                                      pcie_func_t *func,
                                      pci_bar_type_t bar,
                                      pci_bar_num_t bar_num,
                                      unsigned int offset,
                                      tlp_data_t *data,
                                      int credits,
                                      pcie_tlp_info_t *tlp_info)
{
  pcie_core_callbacks_t *cbs = s->core_cbs;

  return cbs->host_w32(func, bar, bar_num, offset, data[0], credits, tlp_info);
}

static uint32_t pcie_core_cb_host_r32(pcie_state_t *s,
                                      pcie_func_t *func,
                                      pci_bar_type_t bar,
                                      pci_bar_num_t bar_num,
                                      unsigned int offset,
                                      tlp_data_t *rd_data,
                                      int tag,
                                      bool *complete,
                                      pcie_tlp_info_t *tlp_info)
{
  pcie_core_callbacks_t *cbs = s->core_cbs;

  return cbs->host_r32(func, bar, bar_num, offset, rd_data, tag, complete,
                       tlp_info);
}

static void pcie_core_cb_biu_dma_read_complete(pcie_state_t *s,
                                               void *context,
                                               uint16_t tag,
                                               int len,
                                               bool ok,
                                               pcie_tlp_info_t *tlp_info)
{
  pcie_core_callbacks_t *cbs = s->core_cbs;

  cbs->biu_dma_read_complete(context, tag, len, ok, tlp_info);
}

enum pcie_exit_decision pcie_core_cb_pcie_exit_hook(struct cosim_platform_state_s *c_s,
                                                    const int live_instances,
                                                    const int instance_closing)
{
  pcie_state_t *s = container_of(pcie_state_t, cosim_state, c_s);
  pcie_core_callbacks_t *cbs = s->core_cbs;

  return cbs->pcie_exit_hook(live_instances, instance_closing);
}

void pcie_core_cb_pcie_validate_access(pcie_state_t *s,
                                       struct fs_hw_s *hw,
                                       pcie_func_t *func,
                                       int bar,
                                       unsigned offset)
{
  pcie_core_callbacks_t *cbs = s->core_cbs;

  cbs->pcie_validate_access(hw, func, bar, offset);
}

uint64_t to_addr64(uint32_t addr_hi, uint32_t addr_lo)
{
  return ((uint64_t) addr_hi << 32) | addr_lo;
}

/* Calls into the pcie core model pass in a rid that is always 0-based. It's the
 * core's job to note the bus number and any possible offset value during enumeration
 * and to apply the conversion. */

/* Convert an internal rid to one valid on the bus */
static int internal_to_bus_rid(pcie_state_t *state, int rid)
{
  return state->busnum | (rid + state->rid_offset);
}

void init_pcie(pcie_state_t *state)
{
  static const pcie_cfg_handler_t default_handler = { .read = cfg_read, .write = cfg_write};
  int i;
  for (i = 0; i < 64; i++) {
    pthread_mutex_init(&state->requests_by_tag[i].mutex, NULL);
    pthread_cond_init(&state->requests_by_tag[i].cond, NULL);
  }
  pthread_mutex_init(&state->send_mutex, NULL);
  pcie_cfg_handler_set(state, &default_handler, state);
  pcie_tlp_init(state);
}

static void cfg_read(const pcie_tlp_info_t *tlp_info, void *context)
{
  pcie_state_t *state = context;
  if (pcie_retry_active(state)) {
    pcie_cfg_handler_cpl(state, tlp_info, TLP1_COMPL_STATUS_CR);
    usleep(100000); /* Limit the spam rate to keep log size down. The MC is setting things up in another thread */
  } else {
    unsigned rid = tlp_info->cpltr_id & 0xff;
    state->busnum = tlp_info->cpltr_id & 0xff00;
    if (!state->rid_offset_done) {
      state->rid_offset = rid;
      state->rid_offset_done = true;
    }
    pcie_func_t *func = fs_rid_to_func(state, rid - state->rid_offset);
    uint32_t data = (func == NULL) ?  0xffffffff : 
                                      fs_host_pci_cfg_read(state,
                                                           func->pf_num,
                                                           func->vf_num,
                                                           func->vf_num != -1,
                                                           tlp_info->addr_lo);
    pcie_cfg_handler_cpld(state, tlp_info, TLP1_COMPL_STATUS_SC, data);
  }
}

static void cfg_write(const pcie_tlp_info_t *tlp_info, unsigned data, void *context)
{
  pcie_state_t *state = context;
  unsigned rid = tlp_info->cpltr_id & 0xff;
  pcie_func_t *func = fs_rid_to_func(state, rid - state->rid_offset);
  if (func != NULL)
    fs_host_pci_cfg_write(state,
                          func->pf_num,
                          func->vf_num,
                          func->vf_num != -1,
                          tlp_info->addr_lo,
                          0,
                          tlp_info->first_be,
                          data);
  pcie_cfg_handler_cpl(state, tlp_info, TLP1_COMPL_STATUS_SC);
}

/* Returns credits that are to be freed by the caller. Credits that are passed on
 * are not returned here */
static int mem_write(pcie_state_t *state,
                     uint64_t addr,
                     tlp_data_t data,
                     int credits,
                     pcie_tlp_info_t *tlp_info)
{
  pci_bar_type_t bar_type;
  pci_bar_num_t bar_num;
  uint32_t data_word;
  pcie_func_t *fn = host_mem_access_decode(state, &addr, &bar_type, &bar_num);
  if (fn->mem_enabled) {
    /* MSI-X accesses are handled internally to the PCIe core */
    if (bar_type == PCI_BAR_TYPE_MSIX) {
      pcie_core_cb_pcie_validate_access(state, state->hw, fn, bar_num, addr);
      /** TODO: Passing single elem for now even for buf. Need to enhance */
      if (!data.is_buf_data)
        data_word = data.word.elem;
      else
        data_word = *data.buf.ptr;

      pcie_hw_msix_w32(state, fn, addr, data_word);

      /** TODO: Since MSIX is not yet using  tlp_info, free it
       *        Don't free when passing tlp_info to pcie_hw_msix_w32()
       */
      if ((tlp_info) && (!state->process_tlp_once))
        free(tlp_info);
    } else {
      credits = pcie_core_cb_host_w32(state, fn, bar_type, bar_num, addr, &data,
                                      credits, tlp_info);
    }
  }
  return credits;
}

/* complete is true on entry, no need to touch it in paths that always complete */
static int mem_read(pcie_state_t *state,
                    uint64_t addr,
                    tlp_data_t *rd_data,
                    int tag,
                    int *cid,
                    bool *complete,
                    pcie_tlp_info_t *tlp_info)
{
  pci_bar_type_t bar_type;
  pci_bar_num_t bar_num;
  uint32_t data_word;
  pcie_func_t *fn = host_mem_access_decode(state, &addr, &bar_type, &bar_num);
  *cid = fn->rid;
  if (fn->mem_enabled) {
    /* MSI-X accesses are handled internally to the PCIe core */
    if (bar_type == PCI_BAR_TYPE_MSIX) {
      /** TODO: Passing single elem for now even for buf. Need to enhance */
      if (!rd_data->is_buf_data) {
        data_word = pcie_hw_msix_r32(state, fn, addr);
        rd_data->word.elem = data_word;
      }
      else {
        data_word = pcie_hw_msix_r32(state, fn, addr);
        rd_data->buf.ptr = checked_calloc(rd_data->buf.len * sizeof(uint32_t));
        *rd_data->buf.ptr = data_word;
        tlp_info->tlp_type = TLP0_TYPE_CPL;
        tlp_info->tlp_fmt = TLP0_FMT_3DW_DATA;
        tlp_info->cpl_status = TLP1_COMPL_STATUS_SC;
      }

      /** TODO: Since MSIX is not yet using  tlp_info, free it
       *        Don't free when passing tlp_info to pcie_hw_msix_r32()
       */
      if ((tlp_info) && (!state->process_tlp_once))
        free(tlp_info);

      return data_word;
    }
    else {
      return pcie_core_cb_host_r32(state, fn, bar_type, bar_num, addr, rd_data,
                                   tag, complete, tlp_info);
    }
  } else {
    return 0xffffffff;
  }
}

/* as for mem_write */
static int io_write(pcie_state_t *state,
                    unsigned addr,
                    tlp_data_t data,
                    int credits,
                    pcie_tlp_info_t *tlp_info)
{
  pci_bar_type_t bar;
  pci_bar_num_t bar_num;
  pcie_func_t *fn = host_io_access_decode(state, &addr, &bar, &bar_num);
  if (fn->io_enabled) {
    credits = pcie_core_cb_host_w32(state, fn, bar, bar_num, addr, &data,
                                    credits, tlp_info);
  }
  return credits;
}

static int io_read(pcie_state_t *state,
                   unsigned addr,
                   tlp_data_t *rd_data,
                   int tag,
                   int *cid,
                   bool *complete,
                   pcie_tlp_info_t *tlp_info)
{
  pci_bar_type_t bar;
  pci_bar_num_t bar_num;
  pcie_func_t *fn = host_io_access_decode(state, &addr, &bar, &bar_num);
  *cid = fn->rid;
  if (fn->io_enabled)
    return pcie_core_cb_host_r32(state, fn, bar, bar_num, addr, rd_data,
                                 tag, complete, tlp_info);
  else
    return 0xffffffff;
}

static void pcie_read_complete(pcie_state_t *state,
                               uint16_t tag,
                               int eop,
                               int valid,
                               pcie_tlp_info_t *tlp_info)
{
  uint32_t *buffer;
  uint32_t num_dwords;
  int dummy_first, dummy_last;
  unsigned dummy_this_time;
  read_req *req = &state->requests_by_tag[tag];
  //fi_log("Completion for tag 0x%x\n", tag);
  if (req->dut_buffer == NULL) {
        fi_log("No request was outstanding for tag 0x%x!\n", tag);
        exit(1);
  }
  buffer = req->buffer;
  /* calculate_bes_and_len does more work that is really wanted here but it gets the
   * correct result, so call it with some dummy parameters */
  num_dwords = calculate_bes_and_len(req->start_offset, req->len, 4096,
                                     &dummy_first, &dummy_last, &dummy_this_time);

  host_to_dut_tlpdat_buf_c_model(state, state->ts, &valid, buffer, num_dwords);
  memcpy(req->dut_buffer, (uint8_t *)req->buffer + req->start_offset, req->len);

  pthread_mutex_lock(&req->mutex);
  req->dut_buffer = NULL;
  if (req->context == NULL) {
    pthread_cond_signal(&req->cond);
  }
  else {
    if (!state->process_tlp_once) {
      /* Free the packets for process_tlp_once = false */
      free(tlp_info);
      tlp_info = NULL;
    }
    pcie_core_cb_biu_dma_read_complete(state, req->context, tag, req->len,
                                       true /** OK */, tlp_info);
  }
  pthread_mutex_unlock(&req->mutex);
}

static void pcie_read_fail(pcie_state_t *state,
                           uint8_t tag,
                           pcie_tlp_info_t *tlp_info)
{
  read_req *req = &state->requests_by_tag[tag];
  //fi_log("Completion for tag 0x%x\n", tag);
  if (req->dut_buffer == NULL) {
        fi_log("No request was outstanding for tag 0x%x!\n", tag);
        exit(1);
  }
  req->ok = false;
  pthread_mutex_lock(&req->mutex);
  req->dut_buffer = NULL;
  if (req->context == NULL) {
    pthread_cond_signal(&req->cond);
  }
  else {
    if (!state->process_tlp_once) {
      /* Free the packets for process_tlp_once = false */
      free(tlp_info);
      tlp_info = NULL;
    }
    pcie_core_cb_biu_dma_read_complete(state, req->context, tag, req->len,
                                       false /** OK */, tlp_info);
  }
  pthread_mutex_unlock(&req->mutex);
}

static void complete_read_from_host(pcie_state_t *state,
                                    uint32_t *data_in,
                                    uint32_t num_dwords,
                                    pcie_tlp_info_t *tlp_info)
{
  int data[4];
  int eop = (num_dwords == 0) ? 1 : 0;
  int i, is_first;
  tlp_data_t rd_data = {0};
  bool done = true;
  uint64_t addr = to_addr64(tlp_info->addr_hi, tlp_info->addr_lo);

  if (data_in == NULL) {
    rd_data.is_buf_data = (state->process_tlp_once) ? true : false;
    if (rd_data.is_buf_data) {
      rd_data.buf.len = num_dwords;

      data[0] = tlp_info->is_io
                    ? io_read(state, addr, &rd_data, tlp_info->tag,
                              &tlp_info->cpltr_id, &done, tlp_info)
                    : mem_read(state, addr, &rd_data,
                               tlp_info->tag, &tlp_info->cpltr_id, &done,
                               tlp_info);

      if (!done)
        return;

      data_in = (uint32_t *)rd_data.buf.ptr;
    }
    else {
      is_first = 1;
      pcie_tlp_info_t *pend_tlp_info =
          &state->pending_host_reads[tlp_info->tag];
      /* Stash the read for possible later completion by the model. We
       * must do this before calling mem_read/io_read (and possibly discard
       * the result after because the call may hand over the request to
       * another thread, which can attempt to complete it at any point thereafter
       */
      Xassert(!pend_tlp_info->valid);
      *pend_tlp_info = *tlp_info;
      pend_tlp_info->valid = true;
      for (i = 0; i < num_dwords; i++) {
        rd_data.word.be = is_first ? tlp_info->first_be
                                   : (eop ? tlp_info->last_be : 0xf);

        data[i] = tlp_info->is_io
                      ? io_read(state, addr + (4 * i), &rd_data,
                                tlp_info->tag, &tlp_info->cpltr_id, &done, NULL)
                      : mem_read(state, addr + (4 * i), &rd_data,
                                 tlp_info->tag, &tlp_info->cpltr_id, &done,
                                 NULL);

        /* The only case we support is handing off a single-dword read to the
         * model for later completion. */
        Xassert(done || num_dwords == 1);
        if (!done) {
          /* Free the tlp_info as we don't need it further */
          free(tlp_info);
          tlp_info = NULL;
          return;
        }

        is_first = 0;
      }
      /* If the read completed synchonously the stored version should never be used. */
      pend_tlp_info->valid = false;
      data_in = (uint32_t *)data;
    }
  }

  if (!state->process_tlp_once) {
    /* As the API doesn't have enough information about completion status
     * (success or failure), the key used here is the length of the completion
     *
     * If num_dwords == 0, there is no completion data,
     * return cpl_status = TLP1_COMPL_STATUS_UR
     * (the other error types ignored for now)
     *
     * This is not proper way of defining the completion error
     * (as the CFG/IO Write requests has valid completions with zero length,
     * which are not processed as part of this function)
     */
    tlp_info->tlp_type = TLP0_TYPE_CPL;
    if (num_dwords == 0) {
      tlp_info->tlp_fmt = TLP0_FMT_3DW_NO_DATA;
      tlp_info->cpl_status = TLP1_COMPL_STATUS_UR;
    }
    else {
      tlp_info->tlp_fmt = TLP0_FMT_3DW_DATA;
      tlp_info->cpl_status = TLP1_COMPL_STATUS_SC;
    }
  }

  SEND_LOCK(state);
  dut_to_host_tlphdr_c_model(
      state, state->ts, true, eop, tlp_info->tlp_fmt, tlp_info->tlp_type,
      tlp_info->cpl_status, tlp_info->poisoned_req, tlp_info->addr_hi,
      tlp_info->addr_lo, tlp_info->rqstr_id, tlp_info->cpltr_id,
      tlp_info->first_be, tlp_info->last_be, tlp_info->tag, num_dwords,
      tlp_info->bcnt, tlp_info->attr, tlp_info->tc, tlp_info->at,
      tlp_info->tph_present, tlp_info->tph_type, tlp_info->tph_st_tag,
      tlp_info->msg_code, 0, 0);

  if (num_dwords != 0) {
    dut_to_host_tlpdat_buf_c_model(state, state->ts, 1, (void *)&data_in[0],
                                   num_dwords);
  }

  if (tlp_info != NULL) {
    free(tlp_info);
    tlp_info = NULL;
  }

  /* Free the buffer allocated in MSIx Mem reads */
  if (rd_data.is_buf_data)
    free(rd_data.buf.ptr);

  SEND_UNLOCK(state);
}

void prod_pcie(void *context)
{
  pcie_state_t *state = context;
  uint64_t ts;
  uint32_t hdr[4];
  int valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id;
  int first_dw_be, last_dw_be, tag, num_dwords, bcnt, attr, tc, at, tph_on, tph_type;
  int st_tag, msg_code, obff_code, pcredits;
  pcie_tlp_info_t ltlp_info = { 0 };
  pcie_tlp_info_t *tlp_info;
  bool for_cfg_handler;

  host_to_dut_tlphdr_c_model(state, state->ts, &valid, &eop, &tlp_fmt, &tlp_type, &cpl_status, &ep, &addr_hi, &addr_lo, &rqstr_id, &cpltr_id,
                            &first_dw_be, &last_dw_be, &tag, &num_dwords, &bcnt, &attr, &tc, &at, &tph_on, &tph_type,
                            &st_tag, &msg_code,  &obff_code, hdr, &pcredits);
  if (!valid)
    return;

  ltlp_info.valid = true;
  ltlp_info.instance = state->instance;
  ltlp_info.tlp_fmt = tlp_fmt;
  ltlp_info.tlp_type = tlp_type;
  ltlp_info.rqstr_id = rqstr_id;
  // ltlp_info.rqstr_id_en;
  ltlp_info.cpltr_id = cpltr_id;
  ltlp_info.cpl_status = cpl_status;
  ltlp_info.poisoned_req = ep;
  ltlp_info.at = at;
  ltlp_info.tc = tc;
  ltlp_info.bcnt = bcnt;
  ltlp_info.attr = attr;
  ltlp_info.tag = tag;
  // ltlp_info.tph_ind_tag_en;
  ltlp_info.tph_st_tag = st_tag;
  ltlp_info.tph_type = tph_type;
  ltlp_info.tph_present = tph_on;
  // ltlp_info.addr_offset;
  ltlp_info.last_be = last_dw_be;
  ltlp_info.first_be = first_dw_be;
  ltlp_info.msg_code = msg_code;
  ltlp_info.addr_lo = addr_lo;
  ltlp_info.addr_hi = addr_hi;
  ltlp_info.is_io = false;
  if (ltlp_info.tlp_type == TLP0_TYPE_IO) {
    ltlp_info.is_io = true;
  }

  if (cpl_status != TLP1_COMPL_STATUS_SC) {
    tlp_info = checked_calloc(sizeof(*tlp_info));
    memcpy(tlp_info, &ltlp_info, sizeof(ltlp_info));
    Xassert(TLP_FMT_NO_DATA(tlp_fmt));
    pcie_read_fail(state, tag, tlp_info);
    pcie_release_posted_credits(state, pcredits);
    return;
  }

  state->ts += 10;

  ts = state->ts;

  switch (tlp_type) {
    case TLP0_TYPE_CFG0:
      /* When the pseudo-RC is enabled type 0 config transactions are handled
       * for it later. If it's disabled then give them to cfg_handler. */
      for_cfg_handler = !state->pseudo_rc_enabled;
      break;

    case TLP0_TYPE_CFG1:
      for_cfg_handler = true;
      break;

    default:
      for_cfg_handler = false;
      break;
  }

  if (for_cfg_handler && tlp_fmt == TLP0_FMT_3DW_NO_DATA) {
    state->cfg_handler->read(&ltlp_info, state->cfg_handler_user);
  } else if (for_cfg_handler && tlp_fmt == TLP0_FMT_3DW_DATA) {
    int data;
    while(valid && !eop)
      host_to_dut_tlpdat_c_model(state, ts, &valid, &eop, &data);
    state->cfg_handler->write(&ltlp_info, data, state->cfg_handler_user);
  } else if (tlp_type == TLP0_TYPE_MEM && TLP_FMT_NO_DATA(tlp_fmt)) {
    tlp_info = checked_calloc(sizeof(*tlp_info));
    memcpy(tlp_info, &ltlp_info, sizeof(ltlp_info));
    complete_read_from_host(state, NULL /* data_in */, num_dwords, tlp_info);
  } else if (tlp_type == TLP0_TYPE_MEM && !TLP_FMT_NO_DATA(tlp_fmt)) {
    tlp_data_t data;
    uint64_t addr = to_addr64(addr_hi, addr_lo);

    if (!state->process_tlp_once) {
      int be, is_first;
      is_first = 1;
      /** Indicates data is given word by word */
      data.is_buf_data = false;
      while (valid && !eop) {
        host_to_dut_tlpdat_c_model(state, ts, &valid, &eop,
                                   (int *)&data.word.elem);
        be = is_first ? first_dw_be : (eop ? last_dw_be : 0xf);
        data.word.be = be;
        pcredits = mem_write(state, addr, data, pcredits, NULL);
        addr_lo += 4;
        is_first = 0;
      }
    }
    else {
      tlp_info = checked_calloc(sizeof(*tlp_info));
      memcpy(tlp_info, &ltlp_info, sizeof(ltlp_info));
      data.is_buf_data = true;
      data.buf.ptr = (uint32_t *) checked_calloc(num_dwords * sizeof(uint32_t));
      data.buf.len = num_dwords;
      host_to_dut_tlpdat_buf_c_model(state, ts, &valid, data.buf.ptr,
                                     num_dwords);
      pcredits = mem_write(state, addr, data, pcredits, tlp_info);
    }
  } else if (tlp_type == TLP0_TYPE_IO && TLP_FMT_NO_DATA(tlp_fmt)) {
    tlp_info = checked_calloc(sizeof(*tlp_info));
    memcpy(tlp_info, &ltlp_info, sizeof(ltlp_info));
    complete_read_from_host(state, NULL /* data_in */, num_dwords, tlp_info);
  } else if (tlp_type == TLP0_TYPE_IO && !TLP_FMT_NO_DATA(tlp_fmt)) {
    tlp_data_t data;
    if (!state->process_tlp_once) {
      int be, is_first;
      is_first = 1;
      /** Indicates data is given word by word */
      data.is_buf_data = false;
      while (valid && !eop) {
        host_to_dut_tlpdat_c_model(state, ts, &valid, &eop,
                                   (int *)&data.word.elem);
        be = is_first ? first_dw_be : (eop ? last_dw_be : 0xf);
        data.word.be = be;
        pcredits = io_write(state, addr_lo, data, pcredits, NULL);
        addr_lo += 4;
        is_first = 0;
      }
      tlp_type = TLP0_TYPE_CPL;
      tlp_fmt = TLP0_FMT_3DW_NO_DATA;
      cpl_status = TLP1_COMPL_STATUS_SC;
      eop = 1;
      SEND_LOCK(state);
      dut_to_host_tlphdr_c_model(
          state, ts, valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi,
          addr_lo, rqstr_id, cpltr_id, first_dw_be, last_dw_be, tag, num_dwords,
          bcnt, attr, tc, at, tph_on, tph_type, st_tag, msg_code, 0, 0);
      SEND_UNLOCK(state);
    }
    else {
      tlp_info = checked_calloc(sizeof(*tlp_info));
      memcpy(tlp_info, &ltlp_info, sizeof(ltlp_info));
      data.is_buf_data = true;
      data.buf.ptr = (uint32_t *) checked_calloc(num_dwords * sizeof(uint32_t));
      data.buf.len = num_dwords;
      host_to_dut_tlpdat_buf_c_model(state, ts, &valid, data.buf.ptr,
                                     num_dwords);
      pcredits = io_write(state, addr_lo, data, pcredits, tlp_info);
    }
  } else if (tlp_type == TLP0_TYPE_CPL && !TLP_FMT_NO_DATA(tlp_fmt)) {
    tlp_info = checked_calloc(sizeof(*tlp_info));
    memcpy(tlp_info, &ltlp_info, sizeof(ltlp_info));
    pcie_read_complete(state, tag, eop, valid, tlp_info);
  }  else if (tlp_type == TLP0_TYPE_CFG0 && TLP_FMT_NO_DATA(tlp_fmt)) {
          int data;
          /* It's a request to Dan'd pseudo-RC */
          switch(addr_lo) {
                  case COSIM_CONTROL_TIME_LO_REG:
                    data = ts & 0xffffffff;
                    break;
                  case COSIM_CONTROL_TIME_HI_REG:
                    data = ts >> 32;
                    break;
                  default:
                    fi_log("Unhandled CFG0 read from 0x%x!\n", addr_lo);
                    data = 0xdeadcafe;
                    break;
          }
        tlp_type = TLP0_TYPE_CPL;
        tlp_fmt = TLP0_FMT_3DW_DATA;
        cpl_status = TLP1_COMPL_STATUS_SC;
        eop = 0;
        SEND_LOCK(state);
        dut_to_host_tlphdr_c_model(state, ts, valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id,
                                first_dw_be, last_dw_be, tag, num_dwords, bcnt, attr, tc, at, tph_on, tph_type,
                                st_tag, msg_code,  0, 0);
        eop = 1;
        dut_to_host_tlpdat_c_model(state, ts, 1, eop, data);
        SEND_UNLOCK(state);
        /* Note that the magic write for exits is handled by pcie.c and that's the onlt CFG0 write
         * we care about. */
  } else if (tlp_type >= TLP0_TYPE_MSG_LO && tlp_type <= TLP0_TYPE_MSG_HI) {
#ifdef SFC_ARCH_HUNT
          fi_log("Downstream PCIe messages are not handled on hunt\n");
          exit(1);
#else
          uint32_t payload[64];
          int data, idx = 0;
          fi_log("Got a message...\n");
          while(valid && !eop) {
                host_to_dut_tlpdat_c_model(state, ts, &valid, &eop, &data);
                payload[idx++] = data;
          }
          pcie_core_cb_pcie_vdm(state, state->hw, hdr, payload, idx);
#endif
  } else {
          fi_log("Unhandled TLP: type 0x%x fmt 0x%x\n", tlp_type, tlp_fmt);
  }
  pcie_release_posted_credits(state, pcredits);
}

static void dump_buffer(uint8_t *buffer, int len)
{
#if FI_VERBOSE
  int i;
  for (i = 0; i < len; i++) {
    fi_log("%.02x ", buffer[i]);
    if ((i & 15) == 15)
      fi_log("\n");
  }
  fi_log("\n");  
#endif
}

/* In: address and length. Out: fbe, lbe, bytes this tlp. Returns num_dword */
unsigned calculate_bes_and_len(unsigned addr, unsigned len, unsigned max,
                                      int *first, int *last, unsigned *this_time)
{
  static uint8_t end_bes[] = {0xf, 0x1, 0x3, 0x7};
  unsigned offset = addr & 3;
  unsigned end_offset, fits, num_dwords;
  unsigned spill = (len + (addr & 0xfff)) > 0x1000; /* Do we cross a page boundary? */
  unsigned first_dword, last_dword;

  //fs_log("0x%x bytes at address 0x%x. Spill? %d\n", len, addr, spill);

  if (!spill) {
    /* Easy, the last target byte is on the same page as the first. */
    fits = len;
  } else {
    /* How much can we fit in to the end of the page? */
    fits = 0x1000 - (addr & 0xfff);
    /* But we can't actually do a full page. */
    if (fits == 0x1000)
        fits = 0xffc;    
  }
  
  if (fits > max)
    fits = max;

  //fs_log("Fits: 0x%x\n", fits);
  *this_time = fits;
  end_offset = (addr + fits) & 3;
  

  first_dword = addr  & 0xfffffffc;
  last_dword = (addr + fits - 1) & 0xfffffffc;

  //fs_log("Start, end dwords 0x%x 0x%x\n", first_dword, last_dword);
  if (last_dword == first_dword) {
    *first = (((1 << fits) - 1) << offset) & 0xf;
    *last = 0x0;
  }
  else {
    *first = (0xf << offset) & 0xf;
    *last = end_bes[end_offset];
  }
  
  num_dwords = 1 + ((last_dword - first_dword) >> 2);

  //fs_log("Dwords: 0x%x Bes %x : %x\n", num_dwords, *first, *last);
  return num_dwords;
}

/* NOTE: There is a fair bit of duplication between fs_pcie_host_write
 * pcie_host_read and pcie_tlp_to_host. If making fixes either ensure all
 * the the necessary fixes are made or, better yet, factor out the common code.
 */
bool pcie_host_read(pcie_state_t *state, uint32_t rid, void *dest_buffer, uint64_t host_addr,
                    size_t len, uint8_t tag, void *context)
{
  read_req *req;
  int valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id;
  int first_dw_be, last_dw_be, num_dwords, bcnt, attr, tc, at, tph_on, tph_type;
  int st_tag, msg_code;
  unsigned done = 0;
  unsigned maxread = pcie_max_read_req_size(
      state); /* To make this settable we need to loop as we do in
                 fs_pcie_host_write */
  // fs_log("\nHost read: 0x%x bytes from 0x%llx\n", len, (unsigned long
  // long)host_addr);
  tlp_type = TLP0_TYPE_MEM;
  valid = 1;
  eop = 1;
  cpl_status = 0;
  ep = 0;
  rqstr_id = internal_to_bus_rid(state, rid);
  cpltr_id = 0;
  bcnt = 0;
  attr = 0;
  tc = 0;
  at = 0;
  tph_on = 0;
  st_tag = 0;
  tph_type = 0;
  msg_code = 0;

  req = &state->requests_by_tag[tag];
  Xassert(req->dut_buffer == NULL);

  Xassert_eq(((host_addr + len - 1) >> 12), (host_addr >> 12));

  unsigned this_time;

  addr_hi = (host_addr >> 32);
  addr_lo = (host_addr & 0xffffffff);
  tlp_fmt = (addr_hi == 0) ? TLP0_FMT_3DW_NO_DATA : TLP0_FMT_4DW_NO_DATA;

  num_dwords = calculate_bes_and_len(addr_lo, len, maxread, &first_dw_be, &last_dw_be, &this_time);
  req->dut_buffer = (uint8_t *)dest_buffer + done;
  req->start_offset = addr_lo & 3;
  req->len = this_time;
  req->context = context;
  req->ok = true;
  
  pthread_mutex_lock(&req->mutex);
  SEND_LOCK(state);
  dut_to_host_tlphdr_c_model(state, state->ts, valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id,
                            first_dw_be, last_dw_be, tag, num_dwords, bcnt, attr, tc, at, tph_on, tph_type,
                            st_tag, msg_code, 0, 0);
  SEND_UNLOCK(state);

  if (context == NULL) { /* No callback */
    pthread_cond_wait(&req->cond, &req->mutex);
    dump_buffer(dest_buffer, done);
  }
  pthread_mutex_unlock(&req->mutex);

  //fs_log("Got response\n");
  Xassert(this_time == len);
  return req->ok;
}

void fs_pcie_host_write(pcie_state_t *state, uint32_t rid, uint64_t host_addr,
                  void *src_buffer, size_t len)
{
  //fs_log("\nHost write: 0x%x bytes to 0x%llx\n", len, (unsigned long long)host_addr);
  unsigned maxpayload;
  int valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id;
  int first_dw_be, last_dw_be, tag, num_dwords, bcnt, attr, tc, at, tph_on, tph_type;
  int st_tag, msg_code;
  uint32_t buffer[1025];
  uint8_t *src = src_buffer;
  SEND_LOCK(state);
  tlp_type = TLP0_TYPE_MEM;
  valid = 1;
  eop = 0;
  cpl_status = 0;
  ep = 0;
  addr_hi = (host_addr >> 32);
  tlp_fmt = (addr_hi == 0) ? TLP0_FMT_3DW_DATA : TLP0_FMT_4DW_DATA;
  rqstr_id = internal_to_bus_rid(state, rid);
  cpltr_id = 0;
  tph_type = 0;
  msg_code = 0;
  tag = 0;
  memset(buffer, 0, sizeof(buffer));

  maxpayload = pcie_max_payload(state);
  
  while (len) {
    unsigned this_time;
    uint32_t *words = buffer;
  
    //fs_log("len: %x\n", len);
    addr_lo = (host_addr & 0xffffffff);
    num_dwords = calculate_bes_and_len(addr_lo, len, maxpayload, &first_dw_be, &last_dw_be, &this_time);
    memcpy((uint8_t*)buffer + (addr_lo & 3), src, this_time);

    bcnt = 0;
    attr = 0;
    tc = 0;
    at = 0;
    tph_on = 0;
    st_tag = 0;
    dut_to_host_tlphdr_c_model(state, state->ts, valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id,
                              first_dw_be, last_dw_be, tag, num_dwords, bcnt, attr, tc, at, tph_on, tph_type,
                              st_tag, msg_code, 0, 0);
    for (;num_dwords > 0; num_dwords--) {
      int data = *(words++);
      eop = (num_dwords > 1);
      dut_to_host_tlpdat_c_model(state, state->ts, 1, eop, data);
    }
    host_addr += this_time;
    len -= this_time;
    src += this_time;
  }
  SEND_UNLOCK(state);
}

void pcie_send_dut_to_host_cmodel(pcie_state_t *state,
                                  uint64_t host_addr,
                                  void *src_buffer,
                                  size_t len,
                                  bool is_write,
                                  void *context,
                                  pcie_tlp_info_t *tlp_info)
{
  int valid, eop, num_dwords, addr_hi, addr_lo, rqstr_id;
  uint32_t *buffer = (uint32_t *)src_buffer;
  read_req *req = NULL;
  if (!is_write) {
    req = &state->requests_by_tag[tlp_info->tag];
  }
  SEND_LOCK(state);

  rqstr_id = internal_to_bus_rid(state, tlp_info->rqstr_id);

  valid = true;
  eop = (is_write) ? 0 : 1;
  addr_hi = (host_addr >> 32);
  addr_lo = (host_addr & 0xffffffff);
  num_dwords = len/4;

  /* Store the context for read requests to process completions */
  if (!is_write) {
    req->context = context;
    req->dut_buffer = src_buffer;
    req->len = len;
    req->start_offset = tlp_info->addr_offset;
    req->ok = true;
  }

  dut_to_host_tlphdr_c_model(
      state, state->ts, valid, eop, tlp_info->tlp_fmt, tlp_info->tlp_type,
      tlp_info->cpl_status, tlp_info->poisoned_req, addr_hi, addr_lo,
      rqstr_id, tlp_info->cpltr_id, tlp_info->first_be,
      tlp_info->last_be, tlp_info->tag, num_dwords, tlp_info->bcnt,
      tlp_info->attr, tlp_info->tc, tlp_info->at, tlp_info->tph_present,
      tlp_info->tph_type, tlp_info->tph_st_tag, tlp_info->msg_code, 0, 0);

  if (is_write == true)
    dut_to_host_tlpdat_buf_c_model(state, state->ts, valid,
                                     buffer, num_dwords);

  SEND_UNLOCK(state);
}

static uint64_t pcie_tlp_get_addr(pcie_tlp_req_t *req)
{
  uint64_t addr;
  if (req->fmt == TLP_FMT_3DW_DATA || req->fmt == TLP_FMT_3DW_NODAT) {
    addr = req->addrlo;
  }
  else {
    addr = ((uint64_t)req->addrhi << 32) | req->addrlo;
  }
  return addr;
}

bool pcie_tlp_to_host(pcie_state_t *state, uint32_t rid, pcie_tlp_req_t *req,  void *src_buffer,
                      void *dest_buffer)
{
  uint64_t host_addr;
  read_req *rdreq = NULL;
  int valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id;
  int first_dw_be, last_dw_be, num_dwords, bcnt, attr, tc, at, tph_on, tph_type, tag;
  int st_tag, msg_code;
  unsigned this_time = 0;
  uint32_t buffer[1025];
  bool data = (req->fmt == TLP0_FMT_3DW_DATA || req->fmt == TLP0_FMT_4DW_DATA);
  bool expect_cpl = (req->type <=TLP0_TYPE_CFG1) && !data;
  
  int maxpayload = pcie_max_payload(state);
  
  tlp_type = req->type;
  tlp_fmt = req->fmt;
  valid = 1;
  eop = !data;
  cpl_status = 0;
  ep = 0;
  rqstr_id = internal_to_bus_rid(state, rid);
  cpltr_id = 0;
  bcnt = 0;
  attr = 0;
  tc = 0;
  at = 0;
  tph_on = 0;
  st_tag = 0;
  tph_type = 0;
  msg_code = 0;
  tag = 0;
  first_dw_be = last_dw_be = 0;

  SEND_LOCK(state);
  if (req->type == TLP0_TYPE_MEM) {
    host_addr = pcie_tlp_get_addr(req);
    addr_hi = (host_addr >> 32);
    addr_lo = (host_addr & 0xffffffff);
    tag = expect_cpl ? req->tag : 0;  
    /* The core does length/be automagic calculation on memory accesses */
    num_dwords = calculate_bes_and_len(addr_lo, req->len, maxpayload, &first_dw_be, &last_dw_be, &this_time);
    if (data)
      memcpy((uint8_t*)buffer + (addr_lo & 3), src_buffer, this_time);
  } else if (TLP_TYPE_IS_MSG(tlp_type)) {
    tag = req->vdmtag;
    msg_code = req->msgcode;
    /* These are not necessarily addresses and we don't do any massaging of them */
    addr_hi = req->addrhi;
    addr_lo = req->addrlo;
    num_dwords = req->len / 4;
    memcpy(buffer, src_buffer, req->len);
  } else {
    fi_log("Dropping non-memory TLP request (for now)\n");
    SEND_UNLOCK(state);
    return false;
  }

  if (expect_cpl) {
    rdreq = &state->requests_by_tag[tag];
    rdreq->start_offset = addr_lo & 3;
    rdreq->len = this_time;
    rdreq->context = NULL;
    rdreq->ok = true;
    Xassert(rdreq->dut_buffer == NULL);
    rdreq->dut_buffer = (uint8_t *)dest_buffer;
  }


  /* We need the lock before we send the request to ensure the respone cannot arrive before
   * we take it. */
  if (expect_cpl)
    pthread_mutex_lock(&rdreq->mutex);

  dut_to_host_tlphdr_c_model(state, state->ts, valid, eop, tlp_fmt, tlp_type, cpl_status, ep, addr_hi, addr_lo, rqstr_id, cpltr_id,
                            first_dw_be, last_dw_be, tag, num_dwords, bcnt, attr, tc, at, tph_on, tph_type,
                            st_tag, msg_code, 0, 0);
  if (data) {
    uint32_t *words = buffer;
    for (;num_dwords > 0; num_dwords--) {
      int value = *(words++);
      eop = (num_dwords > 1);
      dut_to_host_tlpdat_c_model(state, state->ts, 1, eop, value);
    }
  }
  SEND_UNLOCK(state);

  if (expect_cpl) {
    pthread_cond_wait(&rdreq->cond, &rdreq->mutex);
    dump_buffer(dest_buffer, 4 * num_dwords);
    rdreq->dut_buffer = NULL;
    pthread_mutex_unlock(&rdreq->mutex);
  }
  
  return expect_cpl;
}

void fi_pcie_assert_intx(pcie_state_t *state, uint8_t pin, bool onoff)
{
  int valid, eop, tlp_fmt, tlp_type, cpl_status, ep, rqstr_id, cpltr_id;
  int tag,  bcnt, attr, tc, at, tph_on, tph_type;
  int st_tag, msg_code;

  
  tlp_type = TLP_TYPE_MSG_LOCAL;
  valid = 1;
  eop = 1;
  cpl_status = 0;
  ep = 0;
  tlp_fmt = TLP0_FMT_4DW_NO_DATA;
  rqstr_id = 0;
  cpltr_id = 0;
  tph_type = 0;
  msg_code = onoff ? TLP1_MSG_CODE_ASSERT(pin) : TLP1_MSG_CODE_DEASSERT(pin);
  tag = 0;
  bcnt = 0;
  attr = 0;
  tc = 0;
  at = 0;
  tph_on = 0;
  st_tag = 0;
  

  SEND_LOCK(state);
  dut_to_host_tlphdr_c_model(state, state->ts, valid, eop, tlp_fmt, tlp_type, cpl_status, ep, 0, 0, rqstr_id, cpltr_id,
                              0, 0, tag, 0, bcnt, attr, tc, at, tph_on, tph_type,
                              st_tag, msg_code, 0, 0);
  SEND_UNLOCK(state); 
}

void pcie_reset(pcie_state_t *state)
{
  SEND_LOCK(state);
  fs_pci_cfg_retry_en(state, true, false);
  SEND_UNLOCK(state);
}


unsigned long long dpi_gettime(pcie_state_t *state)
{
  state->ts++; /* Make sure we never return the same value twice */
  return state->ts * 1000;
}

void fs_pcie_complete_host_read(pcie_state_t *state,
                                int tag,
                                uint32_t *data,
                                uint32_t num_dwords,
                                pcie_tlp_info_t *tlp_info)
{
  pcie_tlp_info_t *pend_tlp_info = checked_calloc(sizeof(*pend_tlp_info));

  /* Copy the entire TLP info rather than using pointer.

     This will solve the below sync issue
        - c-model sends the completion (complete_read_host) and immediately
          makes context swicth to prod_pcie()

        - previous cleanup for the completion is not done yet
         (pending_host_reads[tag].valid = false;)

        - as the other end (host) receives the completion, it can send another
          read request with same tag. This can cause assertion in
          completion_read_host() -- Xassert(!pend_tlp_info->valid);
  */
  if ((tlp_info != NULL) && (state->process_tlp_once)) {
    *pend_tlp_info = *tlp_info;
  }
  else {
    *pend_tlp_info = state->pending_host_reads[tag];
    Xassert_eq(state->pending_host_reads[tag].valid, true);
    /* We must mark the tag slot as invalid before calling
     * complete_read_from_host because that returns the tag to the host, and
     * another request with the same tag may arrive at any point thereafter. */
    state->pending_host_reads[tag].valid = false;
  }

  /* Now, send the completion packet */
  complete_read_from_host(state, data /* data_in */, num_dwords,
                          pend_tlp_info);
}

#ifndef CONFIG_TLM
static void init_req(fs_hw_access_t *req,
                     pcie_func_t *pfunc,
                     pci_bar_type_t bar,
                     pci_bar_num_t bar_num,
                     unsigned int offset,
                     tlp_data_t *data,
                     int tag,
                     bool write,
                     int credits,
                     pcie_tlp_info_t *tlp_info)
{
  req->func = pfunc;
  req->bar = bar;
  req->bar_num = bar_num;
  req->offset = offset;
  if (!data->is_buf_data) {
    req->data = data->word.elem;
    req->be = data->word.be;
  }
  else {
    req->data_buf = data->buf.ptr;
    req->len = data->buf.len;
  }
  req->is_write = write;
  req->complete = false;
  req->accepted = false;
  req->tag = tag;
  req->credits = credits;
  req->is_buf_data = data->is_buf_data;
  req->tlp_info = tlp_info;
}

void fs_hw_fifo_put(fs_hw_t *hw, fs_hw_access_t req)
{
  pthread_mutex_lock(&hw->tgt_mutex);
  Xassert(!HW_MSG_FIFO_FULL(&hw->tgt_fifo));
  HW_MSG_FIFO_PUSH(&hw->tgt_fifo, req);
  pthread_mutex_unlock(&hw->tgt_mutex);
}

void fs_hw_fifo_put_w32(pcie_func_t *pfunc,
                        pci_bar_type_t bar,
                        pci_bar_num_t bar_num,
                        unsigned int offset,
                        tlp_data_t data,
                        int credits,
                        pcie_tlp_info_t *tlp_info)
{
  fs_hw_t *hw = pfunc->hw;
  fs_hw_access_t req;

  init_req(&req, pfunc, bar, bar_num, offset, &data, 0, true, credits,
           tlp_info);
  fs_hw_fifo_put(hw, req);
}

void fs_hw_fifo_put_r32(pcie_func_t *pfunc,
                        pci_bar_type_t bar,
                        pci_bar_num_t bar_num,
                        unsigned int offset,
                        tlp_data_t *rd_data,
                        int tag,
                        pcie_tlp_info_t *tlp_info)
{
  fs_hw_t *hw = pfunc->hw;
  fs_hw_access_t req;

  init_req(&req, pfunc, bar, bar_num, offset, rd_data, tag, false, 0, tlp_info);
  fs_hw_fifo_put(hw, req);
}
#endif
