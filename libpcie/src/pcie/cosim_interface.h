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

#ifndef COSIM_INTERFACE_H
#define COSIM_INTERFACE_H

#if defined(__KERNEL__)
#include <linux/errno.h>
#else
#include <errno.h>
#endif

/* Registers defined in verilog. *****************************************/

#define COSIM_HW_CONTROL_START   0x80
#define COSIM_HW_CONTROL_END     0xc0

#define COSIM_CONTROL_TIME_LO_REG 0x80
#define COSIM_CONTROL_TIME_HI_REG 0x84
#define COSIM_CONTROL_LOOPBACK_PORT0_REG 0x88
#define COSIM_CONTROL_LOOPBACK_PORT1_REG 0x8c
#define COSIM_CONTROL_EXIT_REG  0x90

#define COSIM_EXIT_SUCCESS      0xcafebabe
#define COSIM_EXIT_DPI_ERROR    0xdead0000
#define COSIM_EXIT_CONN_CLOSED  0xdead0001
#define COSIM_EXIT_GENERAL_ERR	0xdead0002
#define COSIM_EXIT_NONE         0xffffffff

typedef enum {
  MAC_0=0,/*unlimited*/
  MAC_100M=1,
  MAC_1G=2,
  MAC_10G=3,
  MAC_0_40G=4,
  MAC_40G=5,/*unlimited 40G*/
} mac_speed_e;


/* Registers defined in QEMU. ********************************************/

#define COSIM_CFG_ADDR_REG     0xdc
#define COSIM_CFG_DATA_REG     0xcc

#define COSIM_COMMAND_LO_REG   0xc00
#define COSIM_COMMAND_HI_REG   0xc04

#define COSIM_IOMMU_FAULT_ADDR_LO_REG 0xc08
#define COSIM_IOMMU_FAULT_ADDR_HI_REG 0xc0c
#define COSIM_IOMMU_FAULT_INFO_REG    0xc10


/* The command interface. ************************************************/

#define COSIM_CMD_VERSION 1


/* The command header is used for both requests and responses.
 *
 * For a request, the fields are used as follows:
 *   CMDNUM  - The command number
 *   ERRNO   - Must hold ENOSYS for compatibility purposes.
 *   IN_LEN  - The size of the request payload in 32-bit words.
 *   OUT_LEN - The maximum size of the response payload in 32-bit words.
 *
 * For the response, the fields are used as follows:
 *   CMDNUM  - Unused
 *   ERRNO   - The command result code
 *   IN_LEN  - Unused
 *   OUT_LEN - The size of the response payload in 32-bit words.
 *
 * All lengths are in 32-bit words.  The payload immediately follows
 * the header.
 */
#define COSIM_CMD_HEADER_LEN           2
#define COSIM_CMD_HEADER_CMDNUM_LBN      0
#define COSIM_CMD_HEADER_CMDNUM_WIDTH      16
#define COSIM_CMD_HEADER_ERRNO_LBN        16
#define COSIM_CMD_HEADER_ERRNO_WIDTH        16
#define COSIM_CMD_HEADER_IN_LEN_LBN       32
#define COSIM_CMD_HEADER_IN_LEN_WIDTH       16
#define COSIM_CMD_HEADER_OUT_LEN_LBN      48
#define COSIM_CMD_HEADER_OUT_LEN_WIDTH      16


/* Common command definitions. */

/* The offset for IOV BARs in the WC functions. */
#define COSIM_IOV_BAR_OFFSET 8



/*****************************
 * The version command retrieves the cosim command version number of
 * the responder.
 */
#define COSIM_CMD_VERSION_CMDNUM        1

#define COSIM_CMD_VERSION_IN_LEN                 0

#define COSIM_CMD_VERSION_OUT_LEN                1
#define COSIM_CMD_VERSION_OUT_NUM_LBN              0
#define COSIM_CMD_VERSION_OUT_NUM_WIDTH              32


/*****************************
 * The cosim IOMMU maps EPFNs to physical addresses and information
 * about the mapping.  An EPFN combines the function number with the
 * page number part of the PCIe bus address.  The mapping information
 * specifies the properties of transactions which are allowed for the
 * mapping.
 */
#define COSIM_IOMMU_EPFN_PFN_LO_LBN    0
#define COSIM_IOMMU_EPFN_PFN_LO_WIDTH    32
#define COSIM_IOMMU_EPFN_PFN_HI_LBN    32
#define COSIM_IOMMU_EPFN_PFN_HI_WIDTH    20
#define COSIM_IOMMU_EPFN_FUNC_LBN      52
#define COSIM_IOMMU_EPFN_FUNC_WIDTH      8

#define COSIM_IOMMU_INFO_READ_MODE_LBN           0
#define COSIM_IOMMU_INFO_READ_MODE_WIDTH         4
#define   COSIM_IOMMU_INFO_MODE_SC               0
#define   COSIM_IOMMU_INFO_MODE_UR               1
#define   COSIM_IOMMU_INFO_MODE_CRS              2
#define   COSIM_IOMMU_INFO_MODE_CA               4
#define   COSIM_IOMMU_INFO_MODE_ERROR            8
#define COSIM_IOMMU_INFO_WRITE_MODE_LBN          4
#define COSIM_IOMMU_INFO_WRITE_MODE_WIDTH        4
#define COSIM_IOMMU_INFO_RO_LBN                  8
#define COSIM_IOMMU_INFO_RO_WIDTH                1
#define COSIM_IOMMU_INFO_IDO_LBN                 9
#define COSIM_IOMMU_INFO_IDO_WIDTH               1
#define COSIM_IOMMU_INFO_NOSNOOP_LBN            10
#define COSIM_IOMMU_INFO_NOSNOOP_WIDTH           1
#define COSIM_IOMMU_INFO_QWORD_ACCESS_LBN       11
#define COSIM_IOMMU_INFO_QWORD_ACCESS_WIDTH      1
#define COSIM_IOMMU_INFO_MASK_MAX_RD_REQ_LBN    12
#define COSIM_IOMMU_INFO_MASK_MAX_RD_REQ_WIDTH   1
#define COSIM_IOMMU_INFO_TH_LBN                 13
#define COSIM_IOMMU_INFO_TH_WIDTH                1
#define COSIM_IOMMU_INFO_PH_LBN                 14
#define COSIM_IOMMU_INFO_PH_WIDTH                2
#define COSIM_IOMMU_INFO_ST_LBN                 16
#define COSIM_IOMMU_INFO_ST_WIDTH                8
#define COSIM_IOMMU_INFO_MASK_RO_LBN            24
#define COSIM_IOMMU_INFO_MASK_RO_WIDTH           1
#define COSIM_IOMMU_INFO_MASK_IDO_LBN           25
#define COSIM_IOMMU_INFO_MASK_IDO_WIDTH          1
#define COSIM_IOMMU_INFO_MASK_NOSNOOP_LBN       26
#define COSIM_IOMMU_INFO_MASK_NOSNOOP_WIDTH      1
#define COSIM_IOMMU_INFO_MASK_TH_LBN            27
#define COSIM_IOMMU_INFO_MASK_TH_WIDTH           1
#define COSIM_IOMMU_INFO_MASK_MAX_PAYL_LBN      28
#define COSIM_IOMMU_INFO_MASK_MAX_PAYL_WIDTH     1
/* This field is shared by MAX_PAYL and MAX_RD_REQ */
#define COSIM_IOMMU_INFO_MAX_PAYL_SIZE_LBN      29
#define COSIM_IOMMU_INFO_MAX_PAYL_SIZE_WIDTH     3

/* This magic address prefix causes the bottom 32 bits
 * of the address to be passed through untranslated.
 * This is a hack to save cmdclient and friends needing
 * to interface to the COSIM IOMMMU */
#define COSIM_IOMMU_NO_LOOKUP (0xaBadC0de)

/*****************************
 * The IOMMU configure command enables or disables the IOMMU.
 */
#define COSIM_CMD_IOMMU_CONFIG_CMDNUM   2

#define COSIM_CMD_IOMMU_CONFIG_IN_LEN            1
#define COSIM_CMD_IOMMU_CONFIG_IN_ENABLE_LBN       0
#define COSIM_CMD_IOMMU_CONFIG_IN_ENABLE_WIDTH       1

#define COSIM_CMD_IOMMU_CONFIG_OUT_LEN           0


/*****************************
 * The IOMMU flush command removes all entries from the IOMMU.
 */
#define COSIM_CMD_IOMMU_FLUSH_CMDNUM    3

#define COSIM_CMD_IOMMU_FLUSH_IN_LEN             0

#define COSIM_CMD_IOMMU_FLUSH_OUT_LEN            0


/*****************************
 * The IOMMU insert command adds a new entry to the IOMMU.
 */
#define COSIM_CMD_IOMMU_INSERT_CMDNUM   4

#define COSIM_CMD_IOMMU_INSERT_IN_LEN                5
#define COSIM_CMD_IOMMU_INSERT_IN_EPFN_OFST            0
#define COSIM_CMD_IOMMU_INSERT_IN_EPFN_LEN               2
#define COSIM_CMD_IOMMU_INSERT_IN_PHYS_ADDR_OFST       2
#define COSIM_CMD_IOMMU_INSERT_IN_PHYS_ADDR_LEN          2
#define COSIM_CMD_IOMMU_INSERT_IN_INFO_OFST            4

#define COSIM_CMD_IOMMU_INSERT_OUT_LEN               0


/*****************************
 * The IOMMU remove command deletes an entry to the IOMMU.
 */
#define COSIM_CMD_IOMMU_DELETE_CMDNUM   5

#define COSIM_CMD_IOMMU_DELETE_IN_LEN                2
#define COSIM_CMD_IOMMU_DELETE_IN_EPFN_OFST            0
#define COSIM_CMD_IOMMU_DELETE_IN_EPFN_LEN               2

#define COSIM_CMD_IOMMU_DELETE_OUT_LEN               0


/*****************************
 * The IOMMU remap command removes one mapping from the IOMMU and
 * replaces it with another mapping.  The old mapping is indexed by
 * the EPFN.  On failure, the old mapping will remain.
 */
#define COSIM_CMD_IOMMU_REMAP_CMDNUM    6

#define COSIM_CMD_IOMMU_REMAP_IN_LEN                 7
#define COSIM_CMD_IOMMU_REMAP_IN_NEW_EPFN_OFST         0
#define COSIM_CMD_IOMMU_REMAP_IN_NEW_EPFN_LEN            2
#define COSIM_CMD_IOMMU_REMAP_IN_OLD_EPFN_OFST         2
#define COSIM_CMD_IOMMU_REMAP_IN_OLD_EPFN_LEN            2
#define COSIM_CMD_IOMMU_REMAP_IN_PHYS_ADDR_OFST         4
#define COSIM_CMD_IOMMU_REMAP_IN_PHYS_ADDR_LEN            2
#define COSIM_CMD_IOMMU_REMAP_IN_INFO_OFST             6

#define COSIM_CMD_IOMMU_REMAP_OUT_LEN                0

/************************************
 * Enable/disable write combining on a per-page basis
 */
#define COSIM_CMD_WC_SET_CMDNUM    7
#define COSIM_CMD_WC_SET_IN_LEN    4
#define COSIM_CMD_WC_SET_IN_DEVFN_OFST 0
#define COSIM_CMD_WC_SET_IN_BAR_OFST 1
#define COSIM_CMD_WC_SET_IN_PAGE_OFST 2
#define COSIM_CMD_WC_SET_IN_STATE_OFST 3

#define COSIM_CMD_WC_SET_OUT_LEN 0

/*************************************
 * Flush all WC buffers
 */
#define COSIM_CMD_WC_FLUSH_CMDNUM    8
#define COSIM_CMD_WC_FLUSH_IN_LEN    0
#define COSIM_CMD_WC_FLUSH_OUT_LEN    0

/*************************************
 * Reconfigure write combining: sets
 * - the number of write buffers
 * - the per-buffer depth in bytes
 * - the timeout in us (passed as lo part and hi part)
 */
#define COSIM_CMD_WC_RECONF_CMDNUM    9
#define COSIM_CMD_WC_RECONF_IN_LEN    4
#define COSIM_CMD_WC_RECONF_IN_BUFS_OFST 0
#define COSIM_CMD_WC_RECONF_IN_BYTES_OFST 1
#define COSIM_CMD_WC_RECONF_IN_US_LO_OFST 2
#define COSIM_CMD_WC_RECONF_IN_US_HI_OFST 3
#define COSIM_CMD_WC_RECONF_OUT_LEN   0

/*****************************
 * The IOMMU lookup command retrieves an entry from the IOMMU.
 */
#define COSIM_CMD_IOMMU_LOOKUP_CMDNUM   10

#define COSIM_CMD_IOMMU_LOOKUP_IN_LEN                2
#define COSIM_CMD_IOMMU_LOOKUP_IN_EPFN_OFST            0
#define COSIM_CMD_IOMMU_LOOKUP_IN_EPFN_LEN               2

#define COSIM_CMD_IOMMU_LOOKUP_OUT_LEN               3
#define COSIM_CMD_IOMMU_LOOKUP_OUT_PHYS_ADDR_OFST      0
#define COSIM_CMD_IOMMU_LOOKUP_OUT_PHYS_ADDR_LEN         2
#define COSIM_CMD_IOMMU_LOOKUP_OUT_INFO_OFST           2

/************************************
 * Set up func -> INTx mapping, please don't use
 */
#define COSIM_CMD_MAP_FUNC_TO_INTX_CMDNUM       11

/************************************
 * Send OBFF MSG
 */
#define COSIM_CMD_OBFF_SEND_MSG_CMDNUM       12

#define COSIM_CMD_OBFF_SEND_MSG_IN_LEN        1
#define COSIM_CMD_OBFF_SEND_MSG_IN_OBFF_OFST  0
#define COSIM_CMD_OBFF_SEND_MSG_IN_OBFF_LEN   1

#define COSIM_CMD_OBFF_SEND_MSG_OUT_LEN       0

/************************************
 * Control ST randomization
 */
#define COSIM_CMD_ENABLE_RANDOM_ST_CMDNUM          13

#define COSIM_CMD_ENABLE_RANDOM_ST_IN_LEN           1
#define COSIM_CMD_ENABLE_RANDOM_ST_IN_ENABLE_OFST   0
#define COSIM_CMD_ENABLE_RANDOM_ST_IN_ENABLE_LEN    1

#define COSIM_CMD_ENABLE_RANDOM_ST_OUT_LEN          0

/*****************************
 * Retrieve memory access histogram from the IOMMU mapping
 */
#define COSIM_CMD_IOMMU_GET_HISTOGRAM_CMDNUM                   14

#define COSIM_CMD_IOMMU_GET_HISTOGRAM_IN_LEN                    2
#define COSIM_CMD_IOMMU_GET_HISTOGRAM_IN_EPFN_OFST              0
#define COSIM_CMD_IOMMU_GET_HISTOGRAM_IN_EPFN_LEN               2

#define COSIM_CMD_IOMMU_GET_HISTOGRAM_OUT_LEN                  12
#define COSIM_CMD_IOMMU_GET_HISTOGRAM_OUT_READ_HISTOGRAM_OFST   0
#define COSIM_CMD_IOMMU_GET_HISTOGRAM_OUT_READ_HISTOGRAM_LEN   12

/************************************
 * Set up devfn -> INTx mapping
 */
#define COSIM_CMD_MAP_DEVFN_TO_INTX_CMDNUM          15

#define COSIM_CMD_MAP_DEVFN_TO_INTX_IN_LEN           2
#define COSIM_CMD_MAP_DEVFN_TO_INTX_IN_DEVFN_OFST    0
#define COSIM_CMD_MAP_DEVFN_TO_INTX_IN_DEVFN_LEN     1 
#define COSIM_CMD_MAP_DEVFN_TO_INTX_IN_INTX_OFST     1
#define COSIM_CMD_MAP_DEVFN_TO_INTX_IN_INTX_LEN      1 

#define COSIM_CMD_MAP_DEVFN_TO_INTX_OUT_LEN          0

/************************************
 * Create TLP log
 */
#define COSIM_CMD_TLP_LOG_ALLOC_CMDNUM               16

#define COSIM_CMD_TLP_LOG_ALLOC_IN_LEN                2
#define COSIM_CMD_TLP_LOG_ALLOC_IN_BUF_SIZE_OFST      0
#define COSIM_CMD_TLP_LOG_ALLOC_IN_BUF_SIZE_LEN       1
#define COSIM_CMD_TLP_LOG_ALLOC_IN_CAP_SIZE_OFST      1
#define COSIM_CMD_TLP_LOG_ALLOC_IN_CAP_SIZE_LEN       1

#define COSIM_CMD_TLP_LOG_ALLOC_OUT_LEN               1
#define COSIM_CMD_TLP_LOG_ALLOC_OUT_ID_OFST           0
#define COSIM_CMD_TLP_LOG_ALLOC_OUT_ID_LEN            1

/************************************
 * Free TLP log
 */
#define COSIM_CMD_TLP_LOG_FREE_CMDNUM                17

#define COSIM_CMD_TLP_LOG_FREE_IN_LEN                 1
#define COSIM_CMD_TLP_LOG_FREE_IN_ID_OFST             0
#define COSIM_CMD_TLP_LOG_FREE_IN_ID_LEN              1

#define COSIM_CMD_TLP_LOG_FREE_OUT_LEN                0

/************************************
 * Set TLP log filter
 */
#define COSIM_CMD_TLP_LOG_SET_FILTER_CMDNUM          18

#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_LEN        1000
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_ID_OFST       0
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_ID_LEN        1
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_FILTER_OFST   1

#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_FILTER_LEN            1
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_FILTER_TYPE_OFST      0
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_FILTER_TYPE_LEN       1

#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_LEN             6
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_TYPE_OFST       0
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_TYPE_LEN        1
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_OFFSET_OFST     1
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_OFFSET_LEN      1
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_MASK_OFST       2
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_MASK_LEN        2
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_VALUE_OFST      4
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_MATCH_VALUE_LEN       2

#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_OP_TYPE_OFST          0
#define COSIM_CMD_TLP_LOG_SET_FILTER_IN_OP_TYPE_LEN           1

#define COSIM_CMD_TLP_LOG_SET_FILTER_OUT_LEN          0

/************************************
 * Enable/disable TLP logging
 */
#define COSIM_CMD_TLP_LOG_ENABLE_CMDNUM              19

#define COSIM_CMD_TLP_LOG_ENABLE_IN_LEN               2
#define COSIM_CMD_TLP_LOG_ENABLE_IN_ID_OFST           0
#define COSIM_CMD_TLP_LOG_ENABLE_IN_ID_LEN            1
#define COSIM_CMD_TLP_LOG_ENABLE_IN_ENABLED_OFST      1
#define COSIM_CMD_TLP_LOG_ENABLE_IN_ENABLED_LEN       1

#define COSIM_CMD_TLP_LOG_ENABLE_OUT_LEN              0

/************************************
 * Get TLP log data
 */
#define COSIM_CMD_TLP_LOG_GET_DATA_CMDNUM            20

#define COSIM_CMD_TLP_LOG_GET_DATA_IN_LEN             1
#define COSIM_CMD_TLP_LOG_GET_DATA_IN_ID_OFST         0
#define COSIM_CMD_TLP_LOG_GET_DATA_IN_ID_LEN          1

#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_LEN         1000
#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_FRAME_LEN_OFST 0
#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_FRAME_LEN_LEN  1
#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_HDR_OFST       1
#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_HDR_LEN        4
#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_DATA_OFST      5
#define COSIM_CMD_TLP_LOG_GET_DATA_OUT_DATA_LEN       0


#endif
