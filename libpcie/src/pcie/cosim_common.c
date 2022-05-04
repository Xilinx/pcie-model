/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Pavan Prerepa,
 *            Jackson Rigby
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

#include "cosim_common.h"
#include "cosim_utils.h"
#include "cosim_platform.h"
#include "sfc_cosim_comms.h"
#include "cosim_internal.h"
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* This is the heart of cosim packet processing.  Each client attached
 * to the hub has an ingress port and an egress port.  Each ingress
 * port can buffer one packet if it needs to be delivered to an egress
 * port which is blocked. */
struct cosim_hub {
  cosim_client *(clients[COSIM_NUM_TYPES]);
  int tracing;
};


struct cosim_client {
  /* The hub this client is attached to. */
  cosim_hub *hub;
  /* Linked list of clients with the same consumer type attached to a
   * hub. */
  cosim_client *next;
  /* The type of packets consumed by this client. */
  uint16_t consumer_type;
  /* Handler functions and data. */
  void *opaque;
  cosim_accept_buffer_fn *accept_buffer_fn;
};

/* Forward declarations. */
static void cosim_hub_send_all(cosim_hub *hub,
                               cosim_buffer_head *packet);

/*** Hub tracing **************************************************/

void cosim_hub_set_tracing(struct cosim_hub *hub, int val)
{
  hub->tracing = val;
}

void ch_vtrace(const struct cosim_hub *hub, const char *fmt, va_list args)
{
  if (hub->tracing)
    vprintf(fmt, args);
}

static void ch_trace(const struct cosim_hub *hub, const char* fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  ch_vtrace(hub, fmt, args);
  va_end(args);
}


/*** Buffer management ***************************************************/

static void cosim_default_buffer_free(cosim_buffer_head *buffer)
{
  assert(buffer != NULL);

  free(buffer);
}

cosim_buffer_head *cosim_buffer_alloc(uint16_t size, uint16_t type)
{
  cosim_buffer_head *buffer;

  buffer = malloc(sizeof(cosim_buffer_head) + size);
  if (buffer == NULL) {
    ERR( "Failed to allocate buffer.\n");
    return NULL;
  }

  buffer->size = size;
  buffer->type = type;
  buffer->next = NULL;
  buffer->free_fn = &cosim_default_buffer_free;
  buffer->ref_count = 1;

  return buffer;
}


/*** Packet hub **********************************************************/

cosim_hub *cosim_hub_new(void)
{
  cosim_hub *hub;
  char *tracing;
  int i;

  hub = calloc(1,sizeof(cosim_hub));

  if (hub == NULL)
    return NULL;

  for(i=0; i<COSIM_NUM_TYPES; i++)
    hub->clients[i] = NULL;

  tracing = getenv("COSIM_HUB_TRACE");
  if (tracing != NULL)
    hub->tracing = strtoul(tracing, NULL, 0);

  return hub;
}

void cosim_hub_delete(cosim_hub *hub)
{
  assert(hub != NULL);

  free(hub);
}

static void cosim_send_type_subscriptions(cosim_hub *hub, uint16_t type)
{
  cosim_buffer_head *buffer;
  struct cosim_subscribe_request *request;

  ch_trace(hub, "Sending subscriptions for type %d\n", type);

  if(hub->clients[type] != NULL) {
    ERR( "Duplicate receiver for packet type %d\n", type);
    return;
  }

  buffer = cosim_buffer_alloc(sizeof(*request),
                              COSIM_TYPE_SUBSCRIBE_REQUEST);
  if (buffer == NULL) {
    ERR("Failed to allocate buffer for subscription.\n");
    return;
  }

  request = cosim_buffer_data(buffer);
  request->subscription_type = type;

  cosim_hub_send_all(hub, buffer);
  cosim_buffer_free(buffer);
}

static void
cosim_send_client_subscriptions(cosim_hub *hub, cosim_client *client)
{
  uint16_t type;
  cosim_buffer_head *buffer;
  struct cosim_subscribe_request *request;

  ch_trace(hub, "Adding subscriptions to client %p\n", client);

  for (type=0; type<COSIM_NUM_TYPES; type++) {
    if (type == COSIM_TYPE_SUBSCRIBE_REQUEST ||
        hub->clients[type] == NULL)
      continue;

    ch_trace(hub, "Subscribing client %p to type %d\n", client, type);

    buffer = cosim_buffer_alloc(sizeof(*request),
                                COSIM_TYPE_SUBSCRIBE_REQUEST);
    if (buffer == NULL) {
      ERR("Failed to allocate buffer for subscription.\n");
      return;
    }
    
    request = cosim_buffer_data(buffer);
    request->subscription_type = type;

    client->accept_buffer_fn(client->opaque, buffer);

    cosim_buffer_free(buffer);
  }
}

static void cosim_hub_add_client(cosim_hub *hub, cosim_client *client)
{
  uint16_t type;

  assert(hub != NULL);
  assert(client != NULL);

  type = client->consumer_type;
  assert(type < COSIM_NUM_TYPES);

  if (type == COSIM_TYPE_SUBSCRIBE_REQUEST)
    cosim_send_client_subscriptions(hub, client);
  else
    cosim_send_type_subscriptions(hub, type);

  client->next = hub->clients[type];
  hub->clients[type] = client;
}

static void cosim_hub_del_client(cosim_hub *hub, cosim_client *client)
{
  uint16_t type = client->consumer_type;
  cosim_client **p;

  assert(hub != NULL);
  assert(client != NULL);

  for(p = &(hub->clients[type]); *p != NULL; p=&(*p)->next) {
    if (*p == client) {
      *p = client->next;
      client->next = NULL;
      return;
    }
  }
}

static void cosim_hub_send_all(cosim_hub *hub,
                               cosim_buffer_head *packet)
{
  uint16_t type = packet->type;
  cosim_client *client;

  assert(hub != NULL);
  assert(packet != NULL);

  for(client = hub->clients[type]; client != NULL; client = client->next) {
    client->accept_buffer_fn(client->opaque, packet);
  }
}

void cosim_hub_dispatch_packet(cosim_hub *hub,
                               cosim_buffer_head *packet)
{
  uint16_t type;

  assert(packet != NULL);

  type = packet->type;

  ch_trace(hub, "Handling packet type %d length %d\n", type, packet->size);

  cosim_hub_send_all(hub, packet);

  cosim_buffer_free(packet);
}


/*** Client management ***************************************************/

cosim_client *cosim_client_new(cosim_socket *socket,
                               uint16_t consumer_type,
                               void *opaque,
                               cosim_accept_buffer_fn *accept_buffer)
{
  cosim_client *client;

  assert(socket != NULL);
  assert(accept_buffer != NULL);

  client = malloc(sizeof(cosim_client));
  if (client == NULL) {
    ERR( "Failed to allocate client.\n");
    return NULL;
  }

  client->hub = cosim_socket_get_hub(socket);
  client->next = NULL;
  client->consumer_type = consumer_type;
  client->opaque = opaque;
  client->accept_buffer_fn = accept_buffer;

  ch_trace(client->hub, "Client type %d, accept_buffer %p\n",
           consumer_type, accept_buffer);

  cosim_hub_add_client(client->hub, client);

  return client;
}

void cosim_client_delete(cosim_client *client)
{
  assert(client != NULL);

  cosim_hub_del_client(client->hub, client);
  free(client);
}

void cosim_buffer_send(cosim_client *client,
                       cosim_buffer_head *buffer)
{
  cosim_hub_dispatch_packet(client->hub, buffer);
}


static inline unsigned binfmt(unsigned x) {
  int i;
  unsigned y = 0;


  for (i = 0; i < 8; i++) {
    y >>= 4;
    if (x & 1U) {
      y |= 1U << ((8-1)*4);
    }
    x >>= 1;
  }

  return y;
}

#if WITH_COSIM_TLP_LOG
void cosim_dump_buffer_hdr(FILE *f)
{

  fprintf(f, " ----------- --- --- ---- ---- -- --- ----- ---- ---- ---- ---- --- ---------------- -------- -- -- -- --- -------- -------- \n");
  fprintf(f, " TIME        DIR TAG RID  CID  CS FMT TYPE  LAST FBE  LBE  BCNT LEN ADDRESS/REG_NUM  DATA     TH PH ST MSG MSGCODE    TYPE   \n");
  fprintf(f, " ----------- --- --- ---- ---- -- --- ----- ---- ---- ---- ---- --- ---------------- -------- -- -- -- --- -------- -------- \n");
}

void cosim_dump_buffer(FILE *f, unsigned long long timestamp, cosim_buffer_head *buffer)
{
  struct cosim_request_big *request = cosim_buffer_data(buffer);

  assert(buffer->type == COSIM_TYPE_TLP_UPSTREAM ||
	 buffer->type == COSIM_TYPE_TLP_DOWNSTREAM);

  switch (buffer->type) {
  case COSIM_TYPE_TLP_UPSTREAM:
  case COSIM_TYPE_TLP_DOWNSTREAM:
    {
      char buf[256], *ptr;

      uint32_t tlp0 = request->sm.tlp_header[0];
      uint32_t tlp1 = request->sm.tlp_header[1];
      uint32_t tlp2 = request->sm.tlp_header[2];
      uint32_t tlp3 = request->sm.tlp_header[3];

      unsigned tlp_fmt, tlp_type;
      enum fc_type fc_type;
      uint32_t *data = NULL;
      unsigned th = 0;
      unsigned ph = 0;
      unsigned tag = 0;
      unsigned tag_t8 = 0;
      unsigned tag_t9 = 0;
      unsigned st_tag = 0;
      unsigned rqstr_id = 0, cpltr_id = 0;
      unsigned byte_count = 0;
      unsigned length = 0;
      unsigned first_dw_be = 0, last_dw_be = 0;
      unsigned addr_hi = 0, addr_lo = 0;
      unsigned addr_cpl = 0, reg_num = 0;
      unsigned msg_code = 0;
      unsigned compl_status = 0;

      int last = 0;

      int have_data = 0;
      int have_th = 0;
      int have_ph = 0;
      int have_tag = 0;
      int have_st_tag = 0;
      int have_rqstr_id = 0, have_cpltr_id = 0;
      int have_byte_count = 0;
      int have_length = 0;
      int have_first_dw_be = 0, have_last_dw_be = 0;
      int have_addr_hi = 0, have_addr_lo = 0;
      int have_addr_cpl = 0, have_reg_num = 0;
      int have_msg_code = 0;
      int have_compl_status = 0;

      int have_last = 0;


      int i;


      tlp_fmt = TLP_GET(0, FMT);
      tlp_type = TLP_GET(0, TYPE);

      fc_type =  tlp_get_fc_type(tlp_fmt, tlp_type);

      if (!TLP_FMT_NO_DATA(tlp_fmt)) {
	have_data = 1;
	data = &request->payload[0];
      }

      th = TLP_GET(0, TH);
      have_th = 1;

      assert(have_th);
      if (TLP_TYPE_IS_MEM(tlp_type) && th) {
	ph = TLP_FMT_IS_3DW(tlp_fmt) ? TLP_GET(2, PH) : TLP_GET(3, PH);
	have_ph = 1;
      }

      if (TLP_TYPE_IS_CPL(tlp_type)) {
	tag = TLP_GET(2, TAG);
	have_tag = 1;
      } else if (fc_type != FC_TYPE_P) {
	assert(fc_type == FC_TYPE_NP);
	tag = TLP_GET(1, TAG);
	have_tag = 1;
      }

  if (have_tag == 1) {
    tag_t8 = TLP_GET(0, TAG_T8);
    tag_t9 = TLP_GET(0, TAG_T9);
    if (tag_t9 == 1 && tag_t8 == 0) {
      tag = tag + 0x100;
    }
    else if (tag_t9 == 1 && tag_t8 == 1) {
      tag = tag + 0x200;
    }
  }

      assert(have_th);
      if (TLP_TYPE_IS_MEM(tlp_type) && th) {
	assert(fc_type == FC_TYPE_P || fc_type == FC_TYPE_NP);
	st_tag = (fc_type == FC_TYPE_P) ? TLP_GET(1, ST_POSTED_REQ) : TLP_GET(1, ST_NON_POSTED_REQ);;
	have_st_tag = 1;
      }

      rqstr_id = TLP_TYPE_IS_CPL(tlp_type) ? TLP_GET(2, DEST_BDF) : TLP_GET(1, SRC_BDF);
      have_rqstr_id = 1;

      if (TLP_TYPE_IS_CPL(tlp_type)) {
	cpltr_id =  TLP_GET(1, SRC_BDF);
	have_cpltr_id = 1;
      }
      
      if (TLP_TYPE_IS_CFG(tlp_type)) {
	cpltr_id = TLP_GET(2, DEST_BDF);
	have_cpltr_id = 1;
      }

      if (TLP_TYPE_IS_CPL(tlp_type)) {
	byte_count = TLP_GET(1, BYTE_COUNT);
	have_byte_count = 1;
      }

      length =  TLP_GET(0, LENGTH);
      /*
       * PCIe base specification rev 2.0 table 2.4
       * Table 2-4: Length[9:0] Field Encoding
       * Length[9:0] Corresponding TLP Data Payload Size
       * 00 0000 0001b   1 DW
       * 00 0000 0010b   2 DW
       * ...
       * 11 1111 1111b 1023 DW
       * 00 0000 0000b 1024 DW
       */
      if (length == 0)
        length = MAX_PAYLOAD_WORDS;
      have_length = 1;

      if (TLP_TYPE_IS_IO(tlp_type) ||
	  TLP_TYPE_IS_MEM(tlp_type) ||
	  TLP_TYPE_IS_CFG(tlp_type)) {
	assert(have_th);
	if (th) {
#if 0
	  /*
	    TODO: these values are implicit, should we still output them?
	    It depends whether we are meant to be dumping the packet data (in which case we should omit
	    them and only output the fields actually present), or dumping the semantic content of the
	    packet (in which case we should include them).
	    The requirements of anything/anyone parsing our output will answer the above question.
	  */
	  first_dw_be = 0xffff;
	  assert(have_length);
	  last_dw_be = (length) > 1 ? 0xffff : 0x0000;
	  have_first_dw_be = have_last_dw_be = 1;
#endif
	} else {
	  first_dw_be = TLP_GET(1, FIRST_DW_BE);
	  last_dw_be = TLP_GET(1, LAST_DW_BE);
	  have_first_dw_be = have_last_dw_be = 1;
	}
      }

      if ((TLP_TYPE_IS_MEM(tlp_type) && TLP_FMT_IS_4DW(tlp_fmt)) || TLP_TYPE_IS_MSG(tlp_type)) {
	addr_hi = TLP_GET(2, ADDR_HI);
	have_addr_hi = 1;
      }

      if (TLP_TYPE_IS_IO(tlp_type) || (TLP_TYPE_IS_MEM(tlp_type) && TLP_FMT_IS_3DW(tlp_fmt))) {
	addr_lo = TLP_GET(2, ADDR_LO) << 2;
	have_addr_lo = 1;
      } else if ((TLP_TYPE_IS_MEM(tlp_type) && TLP_FMT_IS_4DW(tlp_fmt))) {
	addr_lo = TLP_GET(3, ADDR_LO) << 2;
	have_addr_lo = 1;
      } else if (TLP_TYPE_IS_MSG(tlp_type)) {
        /* The last DWORD of the header may not be an address (and never
         * is for a VDM), so just take the full 32 bits as-is */
        addr_lo = tlp3;
        have_addr_lo = 1;
      }

      if (TLP_TYPE_IS_CPL(tlp_type)) {
	addr_cpl = TLP_GET(2, LOWER_ADDRESS);
	have_addr_cpl = 1;
      }

      if (TLP_TYPE_IS_CFG(tlp_type)){
	reg_num = TLP_GET(2, REG_NUM) << 2;
	have_reg_num = 1;
      }

      if (TLP_TYPE_IS_MSG(tlp_type)) {
	msg_code = TLP_GET(1, MSG_CODE);
	have_msg_code = 1;
      }

      if (TLP_TYPE_IS_CPL(tlp_type)) {
	compl_status = TLP_GET(1, COMPL_STATUS);
	have_compl_status = 1;
      }


      if (TLP_TYPE_IS_CPL(tlp_type)) {
	assert(have_length && have_addr_cpl && have_byte_count);
	last = !have_data || (length == ((addr_cpl & 3) + byte_count + 3) >> 2);
	have_last = 1;
      }


      ptr = buf;

      ptr += sprintf(ptr, "%s ", (buffer->type == COSIM_TYPE_TLP_UPSTREAM) ? "UP" : "DN");

      if (have_tag)
	ptr += sprintf(ptr, " %03x", tag);
      else
	ptr += sprintf(ptr, " -- ");

      if (have_rqstr_id)
	ptr += sprintf(ptr, " %04x", rqstr_id);
      else
	ptr += sprintf(ptr, " ----");

      if (have_cpltr_id)
	ptr += sprintf(ptr, " %04x", cpltr_id);
      else
	ptr += sprintf(ptr, " ----");

      if (have_compl_status)
	ptr += sprintf(ptr, " %s", TLP_COMPL_STATUS_STR(compl_status));
      else
	ptr += sprintf(ptr, " --");

      ptr += sprintf(ptr, " %02x  %05x", binfmt(tlp_fmt), binfmt(tlp_type));

      if (have_last)
	ptr += sprintf(ptr, " %x   ", last);
      else
	ptr += sprintf(ptr, " ----");


      if (have_first_dw_be)
	ptr += sprintf(ptr, " %04x", binfmt(first_dw_be));
      else
	ptr += sprintf(ptr, " ----");
      if (have_last_dw_be)
	ptr += sprintf(ptr, " %04x", binfmt(last_dw_be));
      else
	ptr += sprintf(ptr, " ----");

      if (have_byte_count)
	ptr += sprintf(ptr, " %04x", byte_count);
      else
	ptr += sprintf(ptr, " ----");

      if (have_length)
	ptr += sprintf(ptr, " %03x", length);
      else
	ptr += sprintf(ptr, " ---");

      assert(!have_addr_hi || have_addr_lo);
      assert(
	     (!have_addr_lo  && !have_addr_cpl) ||
	     (!have_addr_cpl && !have_reg_num ) ||
	     (!have_reg_num  && !have_addr_lo )
	     );
      if (have_addr_hi)
	ptr += sprintf(ptr, " %08x", addr_hi);
      else
	ptr += sprintf(ptr, "         ");
      if (have_addr_lo)
	ptr += sprintf(ptr, "%08x", addr_lo);
      else if (have_addr_cpl)
	ptr += sprintf(ptr, "      %02x", addr_cpl);
      else if (have_reg_num)
	ptr += sprintf(ptr, "     %03x", reg_num);
      else
	ptr += sprintf(ptr, "--------");

      if (have_data)
	ptr += sprintf(ptr, " %08x", data[0]);
      else
	ptr += sprintf(ptr, " --------");

      if (have_th)
	ptr += sprintf(ptr, " %x ", th);
      else
	ptr += sprintf(ptr, " -- ");

      if (have_ph)
	ptr += sprintf(ptr, " %02x", binfmt(ph));
      else
	ptr += sprintf(ptr, " --");

      if (have_st_tag)
	ptr += sprintf(ptr, " %02x", st_tag);
      else
	ptr += sprintf(ptr, " --");

      if (TLP_TYPE_IS_MSG(tlp_type))
	ptr += sprintf(ptr, " %s", TLP_TYPE_MSG_STR(tlp_type));
      else
	ptr += sprintf(ptr, " ---");

      if (have_msg_code)
	ptr += sprintf(ptr, " %08x", msg_code);
      else
	ptr += sprintf(ptr, " --------");

      ptr += sprintf(ptr, " " TLP_TYPE_PRINTF_FMT, TLP_TYPE_PRINTF_ARGS(tlp_fmt, tlp_type));

      assert(ptr + 1 - buf <= sizeof buf);

      fprintf(f, "[%10llu] %s\n", timestamp, buf);

      if (have_data)
	for (i = 1; i < length; i++)
	  fprintf(f, "%85s%08x\n", "", data[i]);
    }
    break;

  default:
    assert(0);
  }
}
#endif
