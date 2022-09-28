/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Pavan Prerepa,
 *            Jackson Rigby,
 *            Charlie Palmer,
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

#include "cosim_common.h"
#include "cosim_utils.h"
#include "cosim_interface.h"
#include "sfc_cosim_comms.h"
#include "dpi_platform.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>

#include "pcie_internal.h"

/*** Types and constants *************************************************/

/* Timeout values in ns. */
#define STARTUP_TIMEOUT    5000000
#define COMPLETION_TIMEOUT 50000

typedef struct {
  uint64_t last_request_time;
  uint32_t request_count;
  int startup_done;
} completion_timer_t;

struct pcie_tlp_state_s {
  /* Host to DUT state.  Protected by the global cosim mutex. */
  cosim_buffer_head *host_to_dut_tlp_list_head;
  cosim_buffer_head **host_to_dut_tlp_list_tail;
  int host_to_dut_tlp_offset;
  int host_to_dut_tlp_length;
  /* DUT to host state.  Protected by the global cosim mutex. */
  cosim_buffer_head *dut_to_host_tlp_buf ;
  int dut_to_host_tlp_offset;
  int dut_to_host_tlp_length;
  completion_timer_t dut_completion_timer;
  uint16_t pcie_ds_p_credits;
  uint16_t pcie_ds_p_credits_batch;

  uint64_t cosim_time;
  bool host_ready;

#if WITH_COSIM_TLP_LOG
  FILE *log;
  bool sync;
#endif
};


#define DUMP_DS(NAME) TRACE(OVM_MEDIUM, "%-20s = 0x%x\n", #NAME, *NAME)
#define DUMP_US(NAME) TRACE(OVM_MEDIUM, "%-20s = 0x%x\n", #NAME, NAME)

#define DUMP_DS_FMT_TYPE(FMT, TYPE) tlp_print_fmt(#TYPE, #FMT, *TYPE, *FMT)
#define DUMP_US_FMT_TYPE(FMT, TYPE) tlp_print_fmt(#TYPE, #FMT, TYPE, FMT)

/*** Debug and error handling ********************************************/

/* ### TODO - move this somewhere that's not specifically pcie. 
   platform.c ? */

typedef struct {
  const char *text;
  unsigned verbosity;
} debug_spec;

/* We recognise the same options as the system verilog - calling
 * them OVM_* is a lie, but keeps the UI regular. Range is 0 - 4.
 */
static const debug_spec debug_levels[] = {
  {"0", OVM_NONE},
  {"1", OVM_MEDIUM},
  {"OVM_NONE", OVM_NONE},
  {"OVM_LOW", OVM_LOW},
  {"OVM_MEDIUM", OVM_MEDIUM},
  {"OVM_HIGH", OVM_HIGH},
  {"OVM_FULL", OVM_FULL}
};

#if WITH_COSIM_TLP_LOG
static const char *tlp_log_name(int instance)
{
  /* The result is used once just after the call. A static buffer is fine. */
  static char buffer[16];
  char envvar[16];
  const char *envval;
  /* First check the environment for overrides */
  sprintf(envvar, "COSIM_TLP_LOG%d", instance);
  envval = getenv(envvar);
  if (envval)
    return envval;
  /* Compatibility */
  if (instance == 0)
    return "cosim_tlp.log"; 
  sprintf(buffer, "cosim_tlp%d.log", instance);
  return buffer;
}
#endif

void pcie_tlp_init(pcie_state_t *state)
{
  const static int n = sizeof(debug_levels)/sizeof(debug_levels[0]);
  int i, new_verb;
  const char *var;
  
  state->tlp = checked_calloc(sizeof(*state->tlp));

  state->tlp->host_to_dut_tlp_list_tail = &state->tlp->host_to_dut_tlp_list_head;
  state->tlp->host_to_dut_tlp_offset = -1;
  state->tlp->dut_to_host_tlp_offset = -1;
  state->tlp->pcie_ds_p_credits = 16;
  state->tlp->host_ready = false;
  
  var = getenv("PCIE_TLP_DEBUG");

  new_verb = OVM_NONE;

  if (var != NULL) {
    for (i = 0; i < n; i++)
      if (strcmp(debug_levels[i].text, var) == 0) {
        new_verb = debug_levels[i].verbosity;
        break;
      }
    if (i == n)
      printf("Failed to decode PCIE_TLP_DEBUG setting %s\n", var);
  }
  printf("PCIe TLP debug level set to %d\n", new_verb);
  dbg_set_verbosity(new_verb);

  var = getenv("PCIE_DS_P_CREDITS");
  if (var != NULL)
    state->tlp->pcie_ds_p_credits = strtoul(var, NULL, 0);
  state->tlp->pcie_ds_p_credits_batch = state->tlp->pcie_ds_p_credits / 2;

#if WITH_COSIM_TLP_LOG
  state->tlp->log = fopen(tlp_log_name(state->instance), "w");
  assert(state->tlp->log != NULL);
  /* This is off by default as it does have a measurable performance impact
   * and is only useful if the model itself is crashing. */
  state->tlp->sync = (getenv("PCIE_LOG_SYNC") != NULL);
  cosim_dump_buffer_hdr(state->tlp->log);
#endif
  cosim_dpi_init(state, state->instance,  &state->cosim_state, prod_pcie, state);
}

int pcie_cleanup(void)
{
/* FIXME: this is never actually called. If we want it
 * to be we need to plumb through a pcie_state_t */
#if WITH_COSIM_TLP_LOG && 0
  if (fclose(tlp_log) != 0)
    return errno;
#endif

  return 0;
}

bool is_10bit_tag_supported(pcie_state_t *state)
{
  if (state->tag_type == TAG_10BIT) {
    return true;
  }
  else {
    return false;
  }
}

int get_10bit_tag(uint32_t tlp0, int tag_8bit)
{
  int tag_t8;
  int tag_t9;
  int tag_10bit;

  tag_t8 = TLP_GET(0, TAG_T8);
  tag_t9 = TLP_GET(0, TAG_T9);
  if (tag_t9 == 0 && tag_t8 == 1) {
    /* (t9,t8) = (0, 1) means tags are from 0 to 255, so no change */
    tag_10bit = tag_8bit;
  }
  else if (tag_t9 == 1 && tag_t8 == 0) {
    /* (t9,t8) = (1, 0) means tags are from 256 to 511 */
    tag_10bit = tag_8bit + 0x100;
  }
  else if (tag_t9 == 1 && tag_t8 == 1) {
    /* (t9,t8) = (1, 1) means tags are from 512 to 767 */
    tag_10bit = tag_8bit + 0x200;
  }
  else {
    ERR("10 bit tag is not enabled.\n");
    tag_10bit = tag_8bit;
  }
  return tag_10bit;
}

int set_tag_bit_8_9(pcie_state_t *state, uint32_t tlp0, int tag)
{
  if (is_10bit_tag_supported(state) == true) {
    /* For tags 0-255 (t9, t8) = (0, 1) */
    if (tag >= 0 && tag < 0x100) {
      TLP_SET(0, TAG_T9, 0);
      TLP_SET(0, TAG_T8, 1);
    }
    /* For tags 256-511 (t9, t8) = (1, 0) */
    else if (tag >= 0x100 && tag < 0x200) {
      TLP_SET(0, TAG_T9, 1);
      TLP_SET(0, TAG_T8, 0);
    }
    /* For tags 512-767 (t9, t8) = (1, 1) */
    else if (tag >= 0x200 && tag < 0x300) {
      TLP_SET(0, TAG_T9, 1);
      TLP_SET(0, TAG_T8, 1);
    }
    else {
      ERR("Invalid Tag number provided : %d, valid range (0, 767)\n", tag);
    }
  }
  else {
    if (tag >= 0 && tag < 0x100) {
      /* For 8-bit tags, (t9, t8) = (0, 0) */
      TLP_SET(0, TAG_T9, 0);
      TLP_SET(0, TAG_T8, 0);
    }
    else {
      ERR("Invalid Tag number provided : %d, valid range (0, 255)\n", tag);
    }
  }
  return tlp0;
}

/*** Completion timer handling *******************************************/

static void completion_timer_handle_request(completion_timer_t *timer,
                                            uint64_t time_now,
                                            struct cosim_request *request)
{
  uint32_t tlp0;
  unsigned int type;
  unsigned int fmt;
  enum fc_type fc_type;

  tlp0 = request->tlp_header[0];
  fmt = TLP_GET(0, FMT);
  type = TLP_GET(0, TYPE);

  fc_type = tlp_get_fc_type(fmt, type);

  if(fc_type == FC_TYPE_NP) {
    timer->last_request_time = time_now;
    timer->request_count++;
  }
}

static void
completion_timer_handle_completion(completion_timer_t *timer,
                                   uint64_t time_now,
                                   struct cosim_request *request)
{
  uint32_t tlp0;
  uint32_t tlp1;
  uint32_t tlp2;
  unsigned int type;
  unsigned int fmt;
  unsigned int length;
  unsigned int byte_count;
  unsigned int lower_address;
  unsigned int bytes_this_tlp;
  enum fc_type fc_type;

  tlp0 = request->tlp_header[0];
  fmt = TLP_GET(0, FMT);
  type = TLP_GET(0, TYPE);

  fc_type = tlp_get_fc_type(fmt, type);

  if(fc_type == FC_TYPE_CPL) {
    tlp1 = request->tlp_header[1];
    tlp2 = request->tlp_header[2];
    length = TLP_GET(0, LENGTH);
    if (length == 0)
      length = MAX_PAYLOAD_WORDS;
    byte_count = TLP_GET(1, BYTE_COUNT);
    lower_address = TLP_GET(2, LOWER_ADDRESS);

    bytes_this_tlp = length*4 - (lower_address & 3);
    if (bytes_this_tlp >= byte_count) {
      if (timer->request_count == 0) {
        ERR("Request count already zero.\n");
        return;
      }
      timer->startup_done = 1;
      timer->request_count--;
    }
  }
}

static int completion_timer_expired(completion_timer_t *timer,
                                    uint64_t time_now)
{
  if (timer->request_count == 0)
    return 0;
  if (!timer->startup_done && time_now < STARTUP_TIMEOUT)
    return 0;
  if( time_now >= timer->last_request_time )
    return (time_now - timer->last_request_time) > COMPLETION_TIMEOUT;
  else
    return (timer->last_request_time - time_now ) > COMPLETION_TIMEOUT;
}


/*** Cosim handlers ******************************************************/

static bool is_termination(pcie_state_t *state, cosim_buffer_head *buffer)
{
  struct cosim_request_big *request;
  uint32_t tlp0;
  uint32_t tlp1;
  uint32_t tlp2;
  unsigned type;

  request = cosim_buffer_data(buffer);
  tlp0 = request->sm.tlp_header[0];
  tlp1 = request->sm.tlp_header[1];
  tlp2 = request->sm.tlp_header[2];

  type = TLP_GET(0, TYPE);
  if (type == TLP0_TYPE_CFG0 && TLP_GET(0, FMT_HAS_DATA)) {
    unsigned len     = TLP_GET(0, LENGTH);
    unsigned fbe     = TLP_GET(1, FIRST_DW_BE);
    unsigned reg_num = TLP_GET(2, REG_NUM) << 2;
    if (reg_num == COSIM_CONTROL_EXIT_REG && len == 1 && fbe == 0xf) {
      cosim_finish(state->cosim_state, request->payload[0]);
      return true;
    }
  }

  return false;
}

void cosim_pcie_accept_buffer(void *opaque,
                              cosim_buffer_head *buffer)
{
  pcie_state_t *state = opaque;
  cosim_state_assert_have_lock(state->cosim_state);

#if WITH_COSIM_TLP_LOG
  cosim_dump_buffer(state->tlp->log, dpi_gettime(state), buffer);
  if (state->tlp->sync)
    fflush(state->tlp->log);
#endif

  if (state->pseudo_rc_enabled && is_termination(state, buffer))
    return;

  cosim_buffer_add_reference(buffer);

  /* FIXME: We don't actually own the buffer here.  We just have a
   * reference to it. */
  *state->tlp->host_to_dut_tlp_list_tail = buffer;
  buffer->next = NULL;
  state->tlp->host_to_dut_tlp_list_tail = &(buffer->next);
}

static void cosim_pcie_send_buffer(pcie_state_t *state,
                            cosim_client *client,
			    cosim_buffer_head *buffer)
{

    cosim_state_lock(state->cosim_state);

#if WITH_COSIM_TLP_LOG
    cosim_dump_buffer(state->tlp->log, dpi_gettime(state), buffer);
#endif

    cosim_buffer_send(client, buffer);

    cosim_state_unlock(state->cosim_state);
}


void cosim_pci_reset_accept_buffer(void *opaque,
                              cosim_buffer_head *buffer)
{
  pcie_state_t *state = opaque;
  cosim_client *client;
  fprintf(stderr, "** Seen PCIE reset\n");
  /* When this returns everything the DUT was sending is done, and nothing more
   * can be sent. The next config. access to the DUT will hang until the MC
   * clears retry-enable. */
  pcie_reset(state);
  buffer = cosim_buffer_alloc(0,
                              COSIM_TYPE_PCI_RESET_ACK);  
  client = cosim_get_pci_reset_client(state->cosim_state);
  cosim_buffer_send(client, buffer);
}

#ifdef CONFIG_TLM
/*
 * Transmit TLPs (to SystemC)
 */
void pcie_core_cb_pcie_tx_tlp(void *opaque, uint32_t *data,
                              unsigned int len_u32)
{
  pcie_state_t *s = (pcie_state_t *) opaque;
  pcie_core_callbacks_t *cbs = s->core_cbs;

  cbs->pcie_tx_tlp(opaque, data, len_u32);
}

/*
 * Receive incoming TLPs (from SystemC)
 */
void pcie_rx_tlp(pcie_state_t *state, uint8_t *data, unsigned int len)
{
  cosim_buffer_head *buffer;
  struct cosim_request_big *request;
  uint32_t buffer_sz = sizeof(struct cosim_request_big);
  uint32_t *data_u32 = (uint32_t *) data;
  uint32_t len_u32 = len/4;
  cosim_client *client;
  int data_pos = 0;
  bool is_4DWHdr;

  buffer = cosim_buffer_alloc(buffer_sz, COSIM_TYPE_TLP_DOWNSTREAM);
  if (buffer == NULL) {
    ERR("[%s]: buffer memory allocation failed.\n", __func__);
    return;
  }

  request = cosim_buffer_data(buffer);

  request->sm.marker1 = MARKER1;

  request->sm.tlp_header[0] = data_u32[data_pos++];
  request->sm.tlp_header[1] = data_u32[data_pos++];
  request->sm.tlp_header[2] = data_u32[data_pos++];

  /* 4 DW hdr bit */
  is_4DWHdr = ((request->sm.tlp_header[0] >> 29) & 0x1) ? true : false;

  if (is_4DWHdr && len_u32 > 3) {
    request->sm.tlp_header[3] = data_u32[data_pos++];
  }

  for (int idx = 0; idx < MAX_PAYLOAD_WORDS && data_pos < len_u32;
       idx++, data_pos++) {
    request->payload[idx] = data_u32[data_pos];
  }

  request->sm.time = 0;
  request->sm.bar_sel_hint = 0;
  request->sm.bar_size_hint = 0;
  request->sm.bdf_hint = 0;
  request->sm.marker2 = MARKER2;

  client = cosim_get_tlp_client(state->cosim_state);
  if (client == NULL) {
    ERR("[%s]: client memory allocation failed.\n", __func__);
    return;
  }
  cosim_pcie_send_buffer(state, client, buffer);
}
#endif

/*** pcie core callbacks *************************************************/

pcie_core_settings_t *pcie_core_cb_get_pcie_core_settings(pcie_state_t *s)
{
  pcie_core_callbacks_t *cbs = s->core_cbs;
  pcie_core_settings_t *c_settings = NULL;

  c_settings = cbs->get_pcie_core_settings(s->hw);
  if (c_settings == NULL) {
    ERR("[%s]: no pcie core settings found.\n", __func__);
  }
  return c_settings;
}

/*** Host to DUT communications ******************************************/

static void host_to_dut_pop_tlp(pcie_tlp_state_t *state)
{
  cosim_buffer_head *buffer = state->host_to_dut_tlp_list_head;
  cosim_buffer_head *next = buffer->next;

  /* Remove the buffer from the list. */
  state->host_to_dut_tlp_list_head = next;
  if (next == NULL)
    state->host_to_dut_tlp_list_tail = &state->host_to_dut_tlp_list_head;

  /* Drop the reference count on the buffer. */
  cosim_buffer_free(buffer);

  /* Start the next TLP. */
  state->host_to_dut_tlp_offset = -1;
}

static void
host_to_dut_update_credits(pcie_tlp_state_t *state, cosim_client *client)
{
  
  cosim_buffer_head *buffer;
  struct cosim_flow_ctrl *flow_ctrl;
  if (state->host_ready && state->pcie_ds_p_credits >= state->pcie_ds_p_credits_batch) {
    buffer = cosim_buffer_alloc(sizeof(*flow_ctrl),
                                COSIM_TYPE_FLOW_CTRL_UPSTREAM);
    flow_ctrl = cosim_buffer_data(buffer);
    flow_ctrl->credits_type_p = state->pcie_ds_p_credits;
    state->pcie_ds_p_credits = 0;
    cosim_buffer_send(client, buffer);
  }
}

void pcie_release_posted_credits(pcie_state_t *state, int credits)
{
  pcie_tlp_state_t *tstate = state->tlp;
  if (credits == 0)
    return;
  cosim_state_lock(state->cosim_state);
  tstate->pcie_ds_p_credits += credits;
  cosim_state_unlock(state->cosim_state);
}

void host_to_dut_tlphdr_c_model(pcie_state_t *state,
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
                                /* OUTPUT */uint32_t *raw_hdr,
                                /* OUTPUT */int *credits
                               )
{
  pcie_tlp_state_t *tstate = state->tlp;
  cosim_client *client;
  int rc;
  unsigned int fmt;
  unsigned int type;
  unsigned int length;
  unsigned int tph;

  tstate->cosim_time = time;

  *valid = 0;

  rc = cosim_state_lock(state->cosim_state);
  if (rc != 0)
    return;

  client = cosim_get_tlp_client(state->cosim_state);
  if (client == NULL)
    goto unlock_out;

  if (tstate->host_to_dut_tlp_offset != -1) {
    ERR( "Header requested while data remain in current TLP.\n");
    goto unlock_out;
  }

  if (completion_timer_expired(&tstate->dut_completion_timer,
                               tstate->cosim_time)) {
    ERR("@ %lld: DUT completion timeout from downstream request at %lld.\n",
        (long long)tstate->cosim_time, (long long)tstate->dut_completion_timer.last_request_time);
    tstate->dut_completion_timer.last_request_time = tstate->cosim_time;
  }

  host_to_dut_update_credits(tstate, client);

  if (tstate->host_to_dut_tlp_list_head != NULL) {
    struct cosim_request *request;
    uint32_t tlp0;
    uint32_t tlp1;
    uint32_t tlp2;
    uint32_t tlp3;
    int has_data;
    int is_4dw;

    /* Ghastly hack #5712. There seems to be a race, probably exposed by frankington
     * starting up much faster than simv whereby if we advertising credits to the host on
     * the first call to host_to_dut_tlphdr_c_model it sometimes fails to notice and
     * get stuck. Since the host needs to enumerate us and Cfg accesses are not posted
     * we can hold off the credits until we see the first TLP from the host. This should
     * ensure PCIe is fully set up at its end. */
    tstate->host_ready = true;
    
    request = cosim_buffer_data(tstate->host_to_dut_tlp_list_head);

    completion_timer_handle_request(&tstate->dut_completion_timer,
                                    tstate->cosim_time,
                                    request);

    tlp0 = request->tlp_header[0];
    tlp1 = request->tlp_header[1];
    tlp2 = request->tlp_header[2];
    tlp3 = request->tlp_header[3];

    memcpy(raw_hdr, request->tlp_header, 4 * sizeof(uint32_t));
    *valid = 1;

    TRACE(OVM_MEDIUM, "@ %lld ======== Downstream TLP header.\n",
          (long long)tstate->cosim_time);

    request = cosim_buffer_data(tstate->host_to_dut_tlp_list_head);

    fmt = TLP_GET(0, FMT);
    has_data = TLP_GET(0, FMT_HAS_DATA);
    is_4dw = TLP_GET(0, FMT_4DW);
    type = TLP_GET(0, TYPE);
    tph = TLP_GET(0, TH);

    /* Common fields. */
    *eop = !has_data;

    length = TLP_GET(0, LENGTH);
    *num_dwords = length;
    if (length == 0)
      length = MAX_PAYLOAD_WORDS;

    *at = TLP_GET(0, AT);
    *attr = TLP_GET(0, ATTR);
    *ep = TLP_GET(0, EP);
    *tph_on = tph;
    *tc = TLP_GET(0, TC);
    *tlp_type = type;
    *tlp_fmt = fmt;

    /* Default values for optional fields. */
    *addr_hi = 0;
    *tph_type = 0;
    *st_tag = 0;
    *cpl_status = TLP1_COMPL_STATUS_SC;
    *bcnt = 0;
    *first_dw_be = 0;
    *last_dw_be = 0;
    *msg_code = 0;

    switch(type) {
    case TLP0_TYPE_MEM:
    case TLP0_TYPE_MEM_LOCKED:
      *first_dw_be = TLP_GET(1, FIRST_DW_BE);
      *last_dw_be = TLP_GET(1, LAST_DW_BE);
      *tag = TLP_GET(1, TAG);
      if (is_10bit_tag_supported(state) == true) {
        *tag = get_10bit_tag(tlp0, *tag);
      }

      *rqstr_id = TLP_GET(1, SRC_BDF);

      if (!is_4dw) {
        *addr_lo = TLP_GET(2, ADDR_LO) * 4;
        if (tph)
          *tph_type = TLP_GET(2, PH);
      } else {
        *addr_hi = TLP_GET(2, ADDR_HI);
        *addr_lo = TLP_GET(3, ADDR_LO) * 4;
        if (tph)
          *tph_type = TLP_GET(3, PH);
      }

      if (tph) {
        if (has_data) {
          /* This is for memory writes. */
          *st_tag = TLP_GET(1, ST_POSTED_REQ);
        } else {
          /* This is for memory reads and atomic operations. */
          *st_tag = TLP_GET(1, ST_NON_POSTED_REQ);
          /* Do not overwrite the byte enable fields even though they
           * aren't present in this request.  The Denali VIP requires
           * them to match ST_TAG.
           *
           *  *first_dw_be = 0;
           *  *last_dw_be = 0;
           */
        }
      }

      *cpltr_id = request->bdf_hint;

      break;

    case TLP0_TYPE_IO:
      *first_dw_be = TLP_GET(1, FIRST_DW_BE);
      *last_dw_be = TLP_GET(1, LAST_DW_BE);
      *tag = TLP_GET(1, TAG);
      if (is_10bit_tag_supported(state) == true) {
        *tag = get_10bit_tag(tlp0, *tag);
      }

      *rqstr_id = TLP_GET(1, SRC_BDF);

      *addr_lo = TLP_GET(2, ADDR_LO) * 4;

      *cpltr_id = request->bdf_hint;

      break;

    case TLP0_TYPE_CFG0:
    case TLP0_TYPE_CFG1:
      *first_dw_be = TLP_GET(1, FIRST_DW_BE);
      *last_dw_be = TLP_GET(1, LAST_DW_BE);
      *tag = TLP_GET(1, TAG);
      if (is_10bit_tag_supported(state) == true) {
        *tag = get_10bit_tag(tlp0, *tag);
      }

      *rqstr_id = TLP_GET(1, SRC_BDF);

      *addr_lo = TLP_GET(2, REG_NUM) * 4;
      *cpltr_id = TLP_GET(2, DEST_BDF);
      break;

    case TLP0_TYPE_CPL:
    case TLP0_TYPE_CPL_LOCKED:
      *bcnt = TLP_GET(1, BYTE_COUNT);
      /* BCM missing */
      *cpl_status = TLP_GET(1, COMPL_STATUS);
      *cpltr_id = TLP_GET(1, SRC_BDF);

      *addr_lo = TLP_GET(2, LOWER_ADDRESS);
      *tag = TLP_GET(2, TAG);
      if (is_10bit_tag_supported(state) == true) {
        *tag = get_10bit_tag(tlp0, *tag);
      }

      *rqstr_id = TLP_GET(2, DEST_BDF);
      break;

    case TLP_TYPE_MSG_LOCAL:
      *tag = TLP_GET(1, TAG);
      if (is_10bit_tag_supported(state) == true) {
        *tag = get_10bit_tag(tlp0, *tag);
      }

      *rqstr_id = TLP_GET(1, SRC_BDF);
      *msg_code = TLP_GET(1, MSG_CODE);
      *addr_lo = 0;

      switch (*msg_code) {
      case TLP1_MSG_CODE_OBFF:
        *obff_code = TLP_GET(3, OBFF_CODE);
        break;

      default:
        ERR( "Downstream local message not supported.\n");
        *valid = 0;
        host_to_dut_pop_tlp(tstate);
        goto unlock_out;
      }
      break;

    default:
      if (type >= TLP0_TYPE_MSG_LO &&
          type <= TLP0_TYPE_MSG_HI) {
        *tag = TLP_GET(1, TAG);
        if (is_10bit_tag_supported(state) == true) {
          *tag = get_10bit_tag(tlp0, *tag);
        }
        *rqstr_id = TLP_GET(1, SRC_BDF);
        *msg_code = TLP_GET(1, MSG_CODE);
      } else {
        ERR( "Unknown downstream TLP type %d\n", type);
        *valid = 0;
        host_to_dut_pop_tlp(tstate);
        goto unlock_out;
      }
    }

    /*DUMP_DS(valid);*/
    DUMP_DS_FMT_TYPE(tlp_fmt, tlp_type);
    DUMP_DS(cpl_status);
    DUMP_DS(ep);
    DUMP_DS(addr_hi);
    DUMP_DS(addr_lo);

    DUMP_DS(rqstr_id);
    DUMP_DS(cpltr_id);
    DUMP_DS(first_dw_be);
    DUMP_DS(last_dw_be);
    DUMP_DS(tag);
    DUMP_DS(num_dwords);
    DUMP_DS(bcnt);
    DUMP_DS(attr);
    DUMP_DS(tc);
    DUMP_DS(at);

    DUMP_DS(tph_on);
    DUMP_DS(tph_type);
    DUMP_DS(st_tag);
    DUMP_DS(msg_code);
    DUMP_DS(eop);
    TRACE(OVM_MEDIUM, "----\n");

    *credits = (tlp_get_fc_type(fmt, type) == FC_TYPE_P) ? 1 : 0;
    
    if (has_data) {
      tstate->host_to_dut_tlp_offset = 0;
      tstate->host_to_dut_tlp_length = length;
    } else {
      host_to_dut_pop_tlp(tstate);
    }
  }

unlock_out:
  rc = cosim_state_unlock(state->cosim_state);
  if (rc != 0)
    return;
}

void host_to_dut_tlpdat_c_model(pcie_state_t *state,
                                uint64_t time,
                                /* OUTPUT */int *valid,
                                /* OUTPUT */int *eop,
                                /* OUTPUT */int *data)
{
  pcie_tlp_state_t *tstate = state->tlp;
  struct cosim_request_big *request;
  //  uint32_t tlp0;
  uint32_t length;
  int rc;

  tstate->cosim_time = time;

  *valid = 0;

  rc = cosim_state_lock(state->cosim_state);
  if (rc != 0)
    return;

  if (tstate->host_to_dut_tlp_offset == -1) {
    ERR( "Data requested with no data remaining.\n");
    goto unlock_out;
  }

  request = cosim_buffer_data(tstate->host_to_dut_tlp_list_head);
  //tlp0 = request->sm.tlp_header[0];

  TRACE(OVM_MEDIUM, "@ %lld ======== Downstream TLP data.\n", (long long)tstate->cosim_time);

  *valid = 1;

  *data = request->payload[tstate->host_to_dut_tlp_offset];

  length = tstate->host_to_dut_tlp_length;
  if (tstate->host_to_dut_tlp_offset + 1 < length) {
    tstate->host_to_dut_tlp_offset += 1;
    *eop = 0;
  } else {
    host_to_dut_pop_tlp(tstate);
    *eop = 1;
  }

  DUMP_DS(data);
  DUMP_DS(eop);

unlock_out:
  rc = cosim_state_unlock(state->cosim_state);
  if (rc != 0)
    return;
}

void host_to_dut_tlpdat_buf_c_model(pcie_state_t *state,
                                    uint64_t time,
                                    /* OUTPUT */int *valid,
                                    /* OUTPUT */void *data,
                                    uint32_t word_len)
{
  pcie_tlp_state_t *tstate = state->tlp;
  struct cosim_request_big *request;
  uint32_t tlp_len;
  int rc;
  uint32_t index;
  uint32_t *data_buf = (uint32_t *)data;

  if (data == NULL || word_len == 0) {
    ERR( "Invalid parameters data, word_len\n");
    return ;
  }

  tstate->cosim_time = time;

  *valid = 0;

  rc = cosim_state_lock(state->cosim_state);
  if (rc != 0)
    return;

  if (tstate->host_to_dut_tlp_offset == -1) {
    ERR( "Data requested with no data remaining.\n");
    goto unlock_out;
  }

  request = cosim_buffer_data(tstate->host_to_dut_tlp_list_head);

  TRACE(OVM_MEDIUM, "@ %lld ======== Downstream TLP data.\n", (long long)tstate->cosim_time);

  *valid = 1;

  tlp_len = tstate->host_to_dut_tlp_length;

  if (tlp_len > word_len) {
    ERR( "TLP length mismatch.(tlp_len %d > word_len %d)\n", tlp_len, word_len);
    goto unlock_out;
  }

  for (index = 0; ((index < word_len) &&
    (tstate->host_to_dut_tlp_offset < tlp_len)); index++) {
    data_buf[index] = request->payload[tstate->host_to_dut_tlp_offset];
    tstate->host_to_dut_tlp_offset++;
  }

  host_to_dut_pop_tlp(tstate);

unlock_out:
  rc = cosim_state_unlock(state->cosim_state);
  if (rc != 0)
    return;
}


/*** DUT to host communications ******************************************/

void dut_to_host_tlphdr_c_model(pcie_state_t *state,
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
                                int ltr_no_snp_lat)
{
  pcie_tlp_state_t *tstate = state->tlp;
  cosim_buffer_head *buffer;
  cosim_client *client;
  struct cosim_request_big *request;
  uint32_t buffer_size;
  uint32_t tlp0;
  uint32_t tlp1;
  uint32_t tlp2;
  uint32_t tlp3;

  tstate->cosim_time = time;
  if (!valid) {
    ERR( "Upstream TLP header handler called with valid=0.\n");
    return;
  }

  TRACE(OVM_MEDIUM, "@ %lld ======== Upstream TLP header.\n", (long long)tstate->cosim_time);

  DUMP_US_FMT_TYPE(tlp_fmt, tlp_type);
  DUMP_US(cpl_status);
  DUMP_US(ep);
  DUMP_US(addr_hi);
  DUMP_US(addr_lo);

  DUMP_US(rqstr_id);
  DUMP_US(cpltr_id);
  DUMP_US(first_dw_be);
  DUMP_US(last_dw_be);
  DUMP_US(tag);
  DUMP_US(num_dwords);
  DUMP_US(bcnt);
  DUMP_US(attr);
  DUMP_US(tc);
  DUMP_US(at);

  DUMP_US(tph_on);
  DUMP_US(tph_type);
  DUMP_US(st_tag);
  DUMP_US(msg_code);
  DUMP_US(eop);
  TRACE(OVM_MEDIUM, "----\n");

  if (tstate->dut_to_host_tlp_offset != -1) {
    ERR( "Upstream TLP header arrived when expecting data.\n");
    tstate->dut_to_host_tlp_offset = -1;
  }

  if (tstate->dut_to_host_tlp_buf != NULL) {
    cosim_buffer_free(tstate->dut_to_host_tlp_buf);
    tstate->dut_to_host_tlp_buf = NULL;
  }

  tlp0 = 0;
  tlp1 = 0;
  tlp2 = 0;
  tlp3 = 0;

  /* Fill out the generic fields. */
  TLP_SET(0, LENGTH, num_dwords);
  TLP_SET(0, AT, at);
  TLP_SET(0, ATTR, attr);
  /* attr is logically an ATTR field,
   * physically it consists of two parts: NS+RO and IDO
   */
  TLP_SET(0, ATTR_IDO, attr >> 2);
  TLP_SET(0, EP, ep);
  TLP_SET(0, TD, 0);
  TLP_SET(0, TH, tph_on);
  TLP_SET(0, TC, tc);
  TLP_SET(0, TYPE, tlp_type);
  TLP_SET(0, FMT, tlp_fmt);

  if (num_dwords == 0)
    num_dwords = MAX_PAYLOAD_WORDS;

  /* Fill out the type specific fields. */
  switch(tlp_type) {
  case TLP0_TYPE_MEM:
  case TLP0_TYPE_MEM_LOCKED:
    TLP_SET(1, FIRST_DW_BE, first_dw_be);
    TLP_SET(1, LAST_DW_BE, last_dw_be);
    TLP_SET(1, TAG, tag);
    tlp0 = set_tag_bit_8_9(state, tlp0, tag);
    TLP_SET(1, SRC_BDF, rqstr_id);
    if (tph_on) {
      if (TLP_GET(0, FMT_HAS_DATA)) {
        TLP_SET(1, ST_POSTED_REQ, st_tag);
      } else {
        TLP_SET(1, ST_NON_POSTED_REQ, st_tag);
      }
    }
    if (addr_hi == 0) {
      TLP_SET(0, FMT_4DW, 0);
      TLP_SET(2, ADDR_LO, addr_lo >> 2);
      TLP_SET(2, PH, tph_type);
    } else {
      TLP_SET(0, FMT_4DW, 1);
      TLP_SET(2, ADDR_HI, addr_hi);
      TLP_SET(3, ADDR_LO, addr_lo >> 2);
      TLP_SET(3, PH, tph_type);
    }
    break;

  case TLP0_TYPE_IO:
    if (tph_on != 0) {
      ERR( "TPH not supported for IO TLPs.\n");
      return;
    }

    TLP_SET(0, FMT_4DW, 0);
    TLP_SET(1, FIRST_DW_BE, first_dw_be);
    TLP_SET(1, LAST_DW_BE, last_dw_be);
    TLP_SET(1, TAG, tag);
    tlp0 = set_tag_bit_8_9(state, tlp0, tag);
    TLP_SET(1, SRC_BDF, rqstr_id);
    TLP_SET(2, ADDR_LO, addr_lo >> 2);
    break;

  case TLP0_TYPE_CFG0:
  case TLP0_TYPE_CFG1:
    if (tph_on != 0) {
      ERR( "TPH not supported for configuration TLPs.\n");
      return;
    }

    TLP_SET(0, FMT_4DW, 0);
    TLP_SET(1, FIRST_DW_BE, first_dw_be);
    TLP_SET(1, LAST_DW_BE, last_dw_be);
    TLP_SET(1, TAG, tag);
    tlp0 = set_tag_bit_8_9(state, tlp0, tag);
    TLP_SET(1, SRC_BDF, rqstr_id);
    TLP_SET(2, REG_NUM, addr_lo >> 2);
    TLP_SET(2, DEST_BDF, cpltr_id);
    break;

  case TLP0_TYPE_CPL:
  case TLP0_TYPE_CPL_LOCKED:
    if (tph_on != 0) {
      ERR( "TPH not supported for completion TLPs.\n");
      return;
    }
    TLP_SET(0, FMT_4DW, 0);
    TLP_SET(1, BYTE_COUNT, bcnt);
    TLP_SET(1, BCM, 0);
    TLP_SET(1, COMPL_STATUS, cpl_status);
    TLP_SET(1, SRC_BDF, cpltr_id);
    TLP_SET(2, LOWER_ADDRESS, addr_lo);
    TLP_SET(2, TAG, tag);
    tlp0 = set_tag_bit_8_9(state, tlp0, tag);
    TLP_SET(2, DEST_BDF, rqstr_id);
    break;

  default:
    if (tlp_type >= TLP0_TYPE_MSG_LO &&
        tlp_type <= TLP0_TYPE_MSG_HI) {
      TLP_SET(0, FMT_4DW, 1);
      TLP_SET(1, SRC_BDF, rqstr_id);
      TLP_SET(1, TAG, tag);
      tlp0 = set_tag_bit_8_9(state, tlp0, tag);
      TLP_SET(1, MSG_CODE, msg_code);
      /* This may, or may not be an address. Treat as opaque bag of bits */
      tlp2 = addr_hi;
      tlp3 = addr_lo; 

      switch (msg_code) {
      case TLP1_MSG_CODE_LTR:
        TLP_SET(3, LTR_SNOOP_LAT, ltr_snp_lat);
        TLP_SET(3, LTR_NO_SNOOP_LAT, ltr_no_snp_lat);
        break;
      }

    } else {
      ERR( "Unknown upstream TLP type %d.\n", tlp_type);
      return;
    }
  }

  if (TLP_GET(0, FMT_HAS_DATA)) {
    tstate->dut_to_host_tlp_offset = 0;
    if (eop)
      ERR( "EOP set on upstream TLP with data.\n");
  } else {
    tstate->dut_to_host_tlp_offset = -1;
    if (!eop)
      ERR( "EOP not set on upstream TLP without data.\n");
  }

  client = cosim_get_tlp_client(state->cosim_state);
  if (client == NULL) {
    ERR( "Cannot send upstream TLP with no client.\n");
    return;
  }

  buffer_size = sizeof(struct cosim_request);
  if (TLP_GET(0, FMT_HAS_DATA))
    buffer_size += num_dwords * 4;
  buffer = cosim_buffer_alloc(buffer_size, COSIM_TYPE_TLP_UPSTREAM);
  if (buffer == NULL) {
    ERR( "Failed to allocate buffer for upstream TLP.\n");
    return;
  }

  request = cosim_buffer_data(buffer);

  request->sm.marker1 = MARKER1;

  request->sm.tlp_header[0] = tlp0;
  request->sm.tlp_header[1] = tlp1;
  request->sm.tlp_header[2] = tlp2;
  request->sm.tlp_header[3] = tlp3;

  request->sm.time = tstate->cosim_time;

  request->sm.bar_sel_hint = 0;
  request->sm.bar_size_hint = 0;
  request->sm.bdf_hint = 0;

  request->sm.marker2 = MARKER2;

  completion_timer_handle_completion(&tstate->dut_completion_timer,
                                     tstate->cosim_time,
                                     &request->sm);

  if (TLP_GET(0, FMT_HAS_DATA)) {
    /* Wait for the data to arrive. */
    tstate->dut_to_host_tlp_buf = buffer;
    tstate->dut_to_host_tlp_length = num_dwords;
    TRACE(OVM_MEDIUM, "Waiting for %d words of data.\n", num_dwords);
  } else {
    /* Send the TLP now. */
    TRACE(OVM_MEDIUM, "Sending upstream TLP with no data.\n");
    cosim_pcie_send_buffer(state, client, buffer);
  }
}

void dut_to_host_tlpdat_c_model(pcie_state_t *state,
                                uint64_t time,
                                int valid,
                                int eop,
                                int data)
{
  pcie_tlp_state_t *tstate = state->tlp;
  cosim_buffer_head *buffer;
  cosim_client *client;
  struct cosim_request_big *request;

  tstate->cosim_time = time;
  if (!valid) {
    ERR( "Upstream TLP data handler called with valid=0.\n");
    return;
  }

  TRACE(OVM_MEDIUM, "@ %lld ======== Upstream TLP data.\n",
        (long long)tstate->cosim_time);

  DUMP_US(data);
  DUMP_US(eop);

  if (tstate->dut_to_host_tlp_offset == -1) {
    ERR( "Upstream TLP data arrived with no header.\n");
    return;
  }

  buffer = tstate->dut_to_host_tlp_buf;
  if (buffer == NULL)
    /* This can be caused by an allocation failure or failure to
     * connect to QEMU.  In either case, we've already reported the
     * error. */
    return;

  client = cosim_get_tlp_client(state->cosim_state);
  if (client == NULL) {
    ERR( "Cannot send upstream TLP with no client.\n");
    return;
  }

  request = cosim_buffer_data(buffer);

  request->payload[tstate->dut_to_host_tlp_offset] = data;
  tstate->dut_to_host_tlp_offset++;

  if (tstate->dut_to_host_tlp_offset >= tstate->dut_to_host_tlp_length) {
    TRACE(OVM_MEDIUM, "Sending completed TLP with data.\n");
    cosim_pcie_send_buffer(state, client, buffer);
    tstate->dut_to_host_tlp_buf = NULL;
    tstate->dut_to_host_tlp_offset = -1;
  } else {
    TRACE(OVM_MEDIUM, "Waiting for more data.\n");
  }
}

void dut_to_host_tlpdat_buf_c_model(pcie_state_t *state,
                                    uint64_t time,
                                    int valid,
                                    void *data,
                                    uint32_t word_len)

{
  pcie_tlp_state_t *tstate = state->tlp;
  cosim_buffer_head *buffer;
  cosim_client *client;
  struct cosim_request_big *request;
  uint32_t index;
  uint32_t tlp_len;
  uint32_t *data_buf = (uint32_t *)data;

  if (data == NULL || word_len == 0) {
    ERR( "Invalid parameters data, word_len\n");
    return ;
  }

  tstate->cosim_time = time;
  if (!valid) {
    ERR( "Upstream TLP data handler called with valid=0.\n");
    return;
  }

  TRACE(OVM_MEDIUM, "@ %lld ======== Upstream TLP data.\n",
        (long long)tstate->cosim_time);

  if (tstate->dut_to_host_tlp_offset == -1) {
    ERR( "Upstream TLP data arrived with no header.\n");
    return;
  }

  buffer = tstate->dut_to_host_tlp_buf;
  if (buffer == NULL)
    /* This can be caused by an allocation failure or failure to
     * connect to QEMU.  In either case, we've already reported the
     * error. */
    return;

  client = cosim_get_tlp_client(state->cosim_state);
  if (client == NULL) {
    ERR( "Cannot send upstream TLP with no client.\n");
    return;
  }

  request = cosim_buffer_data(buffer);

  tlp_len = tstate->dut_to_host_tlp_length;

  if (tlp_len < word_len) {
    ERR( "TLP length mismatch.(tlp_len < word_len)\n");
    return;
  }

  for (index = 0; ((index < word_len) && \
    (tstate->dut_to_host_tlp_offset < tlp_len)); index++) {
    request->payload[tstate->dut_to_host_tlp_offset] = data_buf[index];
    tstate->dut_to_host_tlp_offset++;
  }

  TRACE(OVM_MEDIUM, "Sending completed TLP with data.\n");
  cosim_pcie_send_buffer(state, client, buffer);
  tstate->dut_to_host_tlp_buf = NULL;
  tstate->dut_to_host_tlp_offset = -1;
}

void pcie_cfg_handler_set(pcie_state_t *state,
                          const pcie_cfg_handler_t *handler,
                          void *user)
{
  state->cfg_handler = handler;
  state->cfg_handler_user = user;
}

void pcie_cfg_handler_cpl(pcie_state_t *state,
                          const pcie_tlp_info_t *cfg_tlp,
                          uint8_t status)
{
  pthread_mutex_lock(&state->send_mutex);

  dut_to_host_tlphdr_c_model(
      state, state->ts, /*valid=*/1, /*eop=*/1, TLP0_FMT_3DW_NO_DATA,
      TLP0_TYPE_CPL, status, cfg_tlp->poisoned_req, 0, cfg_tlp->addr_lo,
      cfg_tlp->rqstr_id, cfg_tlp->cpltr_id,
      /*first_dw_be=*/0, /*last_dw_be=*/0, cfg_tlp->tag, /*num_dwords=*/0,
      /*bcnt=*/0, cfg_tlp->attr, cfg_tlp->tc, /*at=*/0, cfg_tlp->tph_present,
      cfg_tlp->tph_type, cfg_tlp->tph_st_tag, cfg_tlp->msg_code, 0, 0);

  pthread_mutex_unlock(&state->send_mutex);
}

void pcie_cfg_handler_cpld(pcie_state_t *state,
                           const pcie_tlp_info_t *cfg_tlp,
                           uint8_t status,
                           uint32_t data)
{
  pthread_mutex_lock(&state->send_mutex);

  dut_to_host_tlphdr_c_model(
      state, state->ts, /*valid=*/1, /*eop=*/0, TLP0_FMT_3DW_DATA,
      TLP0_TYPE_CPL, status, cfg_tlp->poisoned_req, 0, cfg_tlp->addr_lo,
      cfg_tlp->rqstr_id, cfg_tlp->cpltr_id,
      /*first_dw_be=*/0, /*last_dw_be=*/0, cfg_tlp->tag, /*num_dwords=*/1,
      /*bcnt=*/4, cfg_tlp->attr, cfg_tlp->tc, /*at=*/0, cfg_tlp->tph_present,
      cfg_tlp->tph_type, cfg_tlp->tph_st_tag, cfg_tlp->msg_code, 0, 0);

  dut_to_host_tlpdat_c_model(state, state->ts, /*valid=*/1, /*eop=*/1, data);

  pthread_mutex_unlock(&state->send_mutex);
}
