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

#include <stdint.h>
#include "tlp_header_defs.h"

#ifndef SFC_COSIM_COMMS
#define SFC_COSIM_COMMS

#define QEMU_RELEASE 0

/* UPDATE ME when the protocol changes */
/* Please see Neil before changing this */
#if !QEMU_RELEASE
#define SFC_COSIM_COMMS_VER 13
#else
#define SFC_COSIM_COMMS_VER 14
#endif
#define PORTBASE 2000

#define COSIM_TYPE_SUBSCRIBE_REQUEST  0
#define COSIM_TYPE_TLP_UPSTREAM       1
#define COSIM_TYPE_TLP_DOWNSTREAM     2
#define COSIM_TYPE_FLOW_CTRL_UPSTREAM 3
#define COSIM_TYPE_PCI_RESET_REQ      4
#define COSIM_TYPE_PCI_RESET_ACK      5      
#define COSIM_NUM_TYPES               6

/* This header goes at the start of every packet.  It is used by the
 * mux/demux layer to split the stream into packets and to direct the
 * packets to the relevant targets. */
struct __attribute__ ((__packed__)) cosim_packet_header {
  /* Indicates the type of the packet. */
  uint16_t type;
  /* The length of the data after the header (in bytes). */
  uint16_t length;
};

/** Subscriptions ********************************************************/

/* This is sent down a socket to request packets to the specified
 * type be sent in the opposite direction. */
struct __attribute__ ((__packed__)) cosim_subscribe_request {
  uint16_t subscription_type;
};

/** TLPs *****************************************************************/

/* The following describes the format of PCIe TLP packets. */
#define MARKER1 0x11111111
#define MARKER2 0x22222222
#define SMALL_DATA_WORDS 2
#define MAX_HEADER_WORDS 4
/* NB. This value is used if the length field is zero. */
#define MAX_PAYLOAD_WORDS 1024

/* Ensure that all types used have a specified size - NO ENUMS */
struct __attribute__ ((__packed__)) cosim_request {
  uint32_t marker1;

  /* Each of these 4 byte values are the same as the PCIe headers
   * except they are byte reversed.  Standard PCIe headers are
   * big-endian because the most significant part of every field is
   * sent first in order to reduce latency.  These headers are sent in
   * little-endian order to make them easier to parse on little-endian
   * systems.  NB. A 64-bit address is still sent with the high
   * 32-bits first.  This is to keep the encoding rules simple. */
  uint32_t tlp_header[MAX_HEADER_WORDS];

  uint64_t time;

#if !QEMU_RELEASE
  uint8_t bar_sel_hint;
  uint8_t bar_size_hint; /* log2 scale */
  uint16_t bdf_hint;
#endif

  uint32_t marker2;

  /* data to follow (if any) by being contained in a big request */
};

/* Should be malloced rather than on the stack */
struct cosim_request_big {
  struct cosim_request sm;
  uint32_t payload[MAX_PAYLOAD_WORDS];
};


/** Flow control *********************************************************/

struct __attribute__ ((__packed__)) cosim_flow_ctrl {
  uint16_t credits_type_p;
};

#endif
