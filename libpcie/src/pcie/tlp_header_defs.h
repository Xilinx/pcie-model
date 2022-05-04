/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Charlie Palmer,
 *            Jackson Rigby,
 *            Pavan Prerepa
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
#ifndef TLP_HEADER_DEFS_H
#define TLP_HEADER_DEFS_H

/* tlp_header[0] */

#define TLP0_LENGTH_LBN     0
#define TLP0_LENGTH_WIDTH   10
#define TLP0_AT_LBN         10
#define TLP0_AT_WIDTH       2
#define TLP0_ATTR_LBN       12
#define TLP0_ATTR_WIDTH     2
#define TLP0_ATTR_NS_LBN    12
#define TLP0_ATTR_NS_WIDTH  1
#define TLP0_ATTR_RO_LBN    13
#define TLP0_ATTR_RO_WIDTH  1
#define TLP0_ATTR_LBN       12
#define TLP0_ATTR_WIDTH     2
#define TLP0_EP_LBN         14 /* Poisoned */
#define TLP0_EP_WIDTH       1
#define TLP0_TD_LBN         15 /* Not used */
#define TLP0_TD_WIDTH       1
#define TLP0_TH_LBN         16 /* TLP processing hints */
#define TLP0_TH_WIDTH       1
#define TLP0_ATTR_IDO_LBN   18
#define TLP0_ATTR_IDO_WIDTH 1
#define TLP0_TAG_T8_LBN     19 /* tag[8] -- for 10 bit tag support */
#define TLP0_TAG_T8_WIDTH   1
#define TLP0_TC_LBN         20
#define TLP0_TC_WIDTH       3
#define TLP0_TAG_T9_LBN     23 /* tag[9] -- for 10 bit tag support */
#define TLP0_TAG_T9_WIDTH   1

#define TLP0_TYPE_LBN     24
#define TLP0_TYPE_WIDTH   5
#define TLP0_TYPE_MEM        0
#define TLP0_TYPE_MEM_LOCKED 1
#define TLP0_TYPE_IO         2
#define TLP0_TYPE_CFG0       4
#define TLP0_TYPE_CFG1       5
#define TLP0_TYPE_MSG_LO     16
#define TLP0_TYPE_MSG(N) (16+(N))
#define TLP_TYPE_MSG_TO_ROOT  16
#define TLP_TYPE_MSG_BY_ADDR  17
#define TLP_TYPE_MSG_BY_ID    18
#define TLP_TYPE_MSG_BCAST    19
#define TLP_TYPE_MSG_LOCAL    20
#define TLP_TYPE_MSG_GATHER   21
#define TLP_TYPE_MSG_RSVD1    22
#define TLP_TYPE_MSG_RSVD2    23
#define TLP0_TYPE_MSG_HI     23
#define TLP0_TYPE_CPL        10
#define TLP0_TYPE_CPL_LOCKED 11

#define TLP_TYPE_MSG_STR(type) (                \
  (type) == TLP_TYPE_MSG_TO_ROOT ? "TO_ROOT" :  \
  (type) == TLP_TYPE_MSG_BY_ADDR ? "BY_ADDR" :  \
  (type) == TLP_TYPE_MSG_BY_ID ? "BY_ID"     :  \
  (type) == TLP_TYPE_MSG_BCAST ? "BCAST"     :  \
  (type) == TLP_TYPE_MSG_LOCAL ? "LOCAL"     :  \
  (type) == TLP_TYPE_MSG_GATHER ? "GATHER"   :  \
  (type) == TLP_TYPE_MSG_RSVD1 ? "RSVD1"     :  \
  (type) == TLP_TYPE_MSG_RSVD2 ? "RSVD2"     :  \
  "")

#define TLP_TYPE_IS_MEM(type) (                                         \
                               (type) == TLP0_TYPE_MEM ||               \
                               (type) == TLP0_TYPE_MEM_LOCKED           \
                              )

#define TLP_TYPE_IS_IO(type) (                          \
                              (type) == TLP0_TYPE_IO    \
                             )

#define TLP_TYPE_IS_CFG(type) (                                 \
                               (type) == TLP0_TYPE_CFG0 ||      \
                               (type) == TLP0_TYPE_CFG1         \
                              )

#define TLP_TYPE_IS_MSG(type) (                                         \
                               (type) >= TLP0_TYPE_MSG_LO &&            \
                               (type) <= TLP0_TYPE_MSG_HI               \
                              )

#define TLP_TYPE_IS_CPL(type) (                                         \
                               (type) == TLP0_TYPE_CPL ||               \
                               (type) == TLP0_TYPE_CPL_LOCKED           \
                              )

#define TLP0_FMT_LBN             29
#define TLP0_FMT_WIDTH           2
#define TLP0_FMT_3DW_NO_DATA     0
#define TLP0_FMT_4DW_NO_DATA     1
#define TLP0_FMT_3DW_DATA        2
#define TLP0_FMT_4DW_DATA        3
#define TLP0_FMT_4DW_LBN         29 /* Overlaps FMT */
#define TLP0_FMT_4DW_WIDTH       1
#define TLP0_FMT_HAS_DATA_LBN    30 /* Overlaps FMT */
#define TLP0_FMT_HAS_DATA_WIDTH  1
#define TLP0_FMT_PREFIX_LBN      31
#define TLP0_FMT_PREFIX_WIDTH    1

#define TLP_FMT_NO_DATA(fmt) (                                  \
                              (fmt) == TLP0_FMT_3DW_NO_DATA ||  \
                              (fmt) == TLP0_FMT_4DW_NO_DATA     \
                             )

#define TLP_FMT_IS_3DW(fmt)  (                                  \
                              (fmt) == TLP0_FMT_3DW_NO_DATA ||  \
                              (fmt) == TLP0_FMT_3DW_DATA        \
                             )

#define TLP_FMT_IS_4DW(fmt)  (                                  \
                              (fmt) == TLP0_FMT_4DW_NO_DATA ||  \
                              (fmt) == TLP0_FMT_4DW_DATA        \
                             )


#define TLP_TYPE_PRINTF_FMT "%s%s"
#define TLP_TYPE_PRINTF_ARGS(fmt, type)                                        \
  TLP_TYPE_IS_MEM(type)    ? "MEM"   :                                         \
  TLP_TYPE_IS_IO(type)     ? "IO"    : /* ? */                                 \
  (type == TLP0_TYPE_CFG0) ? "CFG0_" :                                         \
  (type == TLP0_TYPE_CFG1) ? "CFG1_" :                                         \
  TLP_TYPE_IS_MSG(type)    ? "MSG"   : /* ? */                                 \
  TLP_TYPE_IS_CPL(type)    ? "CPL"   :                                         \
  "",                                                                          \
  TLP_TYPE_IS_MEM(type)    ? TLP_FMT_NO_DATA(fmt) ? "RD"   : "WR"   :          \
  TLP_TYPE_IS_IO(type)     ? TLP_FMT_NO_DATA(fmt) ? "RD"   : "WR"   : /* ? */  \
  TLP_TYPE_IS_CFG(type)    ? TLP_FMT_NO_DATA(fmt) ? "RREQ" : "WREQ" :          \
  TLP_TYPE_IS_MSG(type)    ? ""                                     : /* ? */  \
  TLP_TYPE_IS_CPL(type)    ? TLP_FMT_NO_DATA(fmt) ? ""     : "D"    :          \
  ""



/* tlp_header[1] */

#define TLP1_FIRST_DW_BE_LBN              0 /* IO, MEM, CFG */
#define TLP1_FIRST_DW_BE_WIDTH            4

#define TLP1_LAST_DW_BE_LBN               4 /* IO, MEM, CFG */
#define TLP1_LAST_DW_BE_WIDTH             4

#define TLP1_MSG_CODE_LBN                 0 /* MSG */
#define TLP1_MSG_CODE_WIDTH               8

#define TLP1_ST_NON_POSTED_REQ_LBN        0 /* MEM(RD,TH) */
#define TLP1_ST_NON_POSTED_REQ_WIDTH      8

#define TLP1_TAG_LBN                      8 /* IO, MEM, CFG, MSG */
#define TLP1_TAG_WIDTH                    8

#define TLP1_ST_POSTED_REQ_LBN            8 /* MEM(WR,TH) */
#define TLP1_ST_POSTED_REQ_WIDTH          8

#define TLP1_BYTE_COUNT_LBN               0 /* CPL */
#define TLP1_BYTE_COUNT_WIDTH             12

#define TLP1_BCM_LBN                      12 /* CPL */
#define TLP1_BCM_WIDTH                    1

#define TLP1_COMPL_STATUS_LBN             13 /* CPL */
#define TLP1_COMPL_STATUS_WIDTH           3
#define TLP1_COMPL_STATUS_SC    0 /* Success. */
#define TLP1_COMPL_STATUS_UR    1 /* Unsupported Request. */
#define TLP1_COMPL_STATUS_CR    2 /* Config Req Retry Status. */
#define TLP1_COMPL_STATUS_CA    4 /* Completer Abort. */

#define TLP_COMPL_STATUS_STR(compl_status) (    \
  compl_status == TLP1_COMPL_STATUS_SC ? "SC" : \
  compl_status == TLP1_COMPL_STATUS_UR ? "UR" : \
  compl_status == TLP1_COMPL_STATUS_CR ? "CR" : \
  compl_status == TLP1_COMPL_STATUS_CA ? "CA" : \
  "")

/* Completer ID for completions.  Otherwise Requester ID if present. */
#define TLP1_SRC_BDF_LBN                  16 /* IO, MEM, CFG, CPL, MSG */
#define TLP1_SRC_BDF_WIDTH                16


#define TLP1_MSG_CODE_ASSERT(N)           (0x20 | (N))
#define TLP1_MSG_CODE_DEASSERT(N)         (0x24 | (N))

#define MSG_CODE_INTX_LBN                 0
#define MSG_CODE_INTX_WIDTH               2

#define TLP1_MSG_CODE_LTR                 0x10
#define TLP1_MSG_CODE_OBFF                0x12

#define TLP1_MSG_CODE_PM_ACTIVE_STATE_NAK 0x14
#define TLP1_MSG_CODE_PM_PME              0x18
#define TLP1_MSG_CODE_PME_TURN_OFF        0x19
#define TLP1_MSG_CODE_PME_TO_ACK          0x1B


/* tlp_header[2] */

#define TLP2_PH_LBN                       0 /* MEM(3DW,TH) */
#define TLP2_PH_WIDTH                     2

#define TLP2_ADDR_LO_LBN                  2 /* IO, MEM(3DW) */
#define TLP2_ADDR_LO_WIDTH                30

#define TLP2_ADDR_HI_LBN                  0 /* MEM(4DW), MSG(ADDR) */
#define TLP2_ADDR_HI_WIDTH                32

/* NB. This includes Ext Reg Number. */
#define TLP2_REG_NUM_LBN                  2 /* CFG */
#define TLP2_REG_NUM_WIDTH                10

#define TLP2_LOWER_ADDRESS_LBN            0 /* CPL */
#define TLP2_LOWER_ADDRESS_WIDTH          7

#define TLP2_TAG_LBN                      8 /* CPL */
#define TLP2_TAG_WIDTH                    8

/* Requester ID for completions.  Otherwise target function if present. */
#define TLP2_DEST_BDF_LBN                 16 /* CFG, CPL, MSG(ID) */
#define TLP2_DEST_BDF_WIDTH               16


/* tlp_header[3] */

#define TLP3_PH_LBN                       0 /* MEM(4DW,TH) */
#define TLP3_PH_WIDTH                     2

#define TLP3_ADDR_LO_LBN                  2 /* MEM(4DW), MSG(ADDR) */
#define TLP3_ADDR_LO_WIDTH                30

#define TLP3_LTR_SNOOP_LAT_LBN            0 /* MSG */
#define TLP3_LTR_SNOOP_LAT_WIDTH          16
#define TLP3_LTR_NO_SNOOP_LAT_LBN         16
#define TLP3_LTR_NO_SNOOP_LAT_WIDTH       16

#define TLP3_OBFF_CODE_LBN                0 /* MSG */
#define TLP3_OBFF_CODE_WIDTH              4

#endif /* TLP_HEADER_DEFS_H */
