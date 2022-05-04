/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Charlie Palmer,
 *            Boleslav Stankevich,
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

#ifndef PCIE_API_H
#define PCIE_API_H

#include <stddef.h>
#include <pcie/pcie_msix.h>

#define PCI_MAX_FNS_PER_IFACE (256)

/** Maximum number of tags supported by PCIe (including 10 bit tag support) */
#define PCI_MAX_TAGS_SUPPORTED 768

/* This enum encode what device hardware is mapped by a BAR. 
 * Note this assumes that hardware gives the MSI-X registers
 * their own BAR, which is recommended but not mandatory */ 
typedef enum pci_bar_type_e {
  PCI_BAR_TYPE_IO = 0x1000,
  PCI_BAR_TYPE_DBELL,
  PCI_BAR_TYPE_MSIX,
  PCI_BAR_TYPE_EXPROM,
  PCI_BAR_TYPE_MEM = 0x4000,
  PCI_BAR_TYPE_INVAL
} pci_bar_type_t;


/* Internal numbering for the BARs */
typedef enum pci_bar_num_e {
  PCI_MAIN_BAR0 = 0,
  PCI_MAIN_BAR1 = 1,
  PCI_MAIN_BAR2 = 2,
  PCI_MAIN_BAR3 = 3,
  PCI_MAIN_BAR4 = 4,
  PCI_MAIN_BAR5 = 5,
  PCI_DUMMY_BAR1 = 6, /* These just make the arithmetic of  */
  PCI_DUMMY_BAR2 = 7, /* address -> BAR number simpler */
  PCI_EXPROM_BAR = 8,
  PCI_SRIOV_BAR0 = 9,
  PCI_SRIOV_BAR1 = 10,
  PCI_SRIOV_BAR2 = 11,
  PCI_SRIOV_BAR3 = 12,
  PCI_SRIOV_BAR4 = 13,
  PCI_SRIOV_BAR5 = 14,
  PCI_NUM_BARS
} pci_bar_num_t;

typedef enum  {
  FS_IRQMODE_NONE = 0,
  FS_IRQMODE_LEGACY,
  FS_IRQMODE_MSI,
  FS_IRQMODE_MSIX
} pcie_irqmode_t;

typedef enum tap_type_e { TAG_8BIT = 0, TAG_10BIT } tag_type_t;

/* The main model knows what this looks like */
struct fs_hw_s;
struct mc_s;

typedef struct pcie_func_s {
  struct fs_hw_s *hw;
  const uint8_t *config_space;
  int pf_num;
  int vf_num;
  unsigned int iface; /* The PCIe interface this func belongs to */
  unsigned int bus;
  unsigned int dev;
  unsigned int func;
  unsigned int rid;
  bool mem_enabled;
  bool master_enabled;
  bool io_enabled;
  pcie_irqmode_t irqmode;
  pcie_msix_state_t msix;
} pcie_func_t;

/* This value of vf_num indicates the pcie_func_t is
 * not a VF */
#define VF_NONE  ((unsigned int)-1)

typedef enum tlp_type_e {
  TLP_TYPE_MEM_RD_WR = 0,
  TLP_TYPE_MEM_RD_LK = 1,
  TLP_TYPE_IO_RW = 2,
  TLP_TYPE_CFG0 = 4,
  TLP_TYPE_CFG1 = 5,
  TLP_TYPE_MSG_BASE = 16,
  TLP_TYPE_CPL = 10,
  TLP_TYPE_CPL_LK = 11,
} tlp_type_t;

typedef enum tlp_fmt_e {
  TLP_FMT_3DW_NODAT = 0,
  TLP_FMT_4DW_NODAT = 1,
  TLP_FMT_3DW_DATA = 2,
  TLP_FMT_4DW_DATA = 3
} tlp_fmt_t;

typedef enum {
  PCIE_INTF_HOST = 0,
  PCIE_INTF_ARM = 1,
  PCIE_INTF_MAX,
} pcie_intf_t;

/* A structure representing the details of an MC-generated TLP request.
 */
typedef struct pcie_tlp_req_s {
  tlp_type_t type;
  tlp_fmt_t fmt;
  unsigned tag;
  unsigned len;
  unsigned addrlo;
  unsigned addrhi;
  unsigned vdmtag;
  unsigned msgcode;
  int  pf;
  int  vf;
  bool vf_active;
} pcie_tlp_req_t;

/** This structure holds the data portion of the TLP
 *
 *  If the target configures to process the tlp data word by word,
 *  is_buf_data should be set to false and use word portion of the
 *  structure
 *     elem -- TLP data word
 *     be   -- byte enable info
 *
 *  If the target configures to process the entire tlp packet once,
 *  is_buf_data should be set to true and use buf portion of the
 *  structure
 *     ptr -- contains the tlp data address
 *     len -- contains the num_dwords
 *     byte enables and additional info of TLP should be retrieved from
 *     pcie_tlp_info_t and which must be passed to function as argument
 *
 */
typedef struct tlp_data_s {
  bool is_buf_data;
  union {
    struct {
      uint32_t elem;
      uint32_t be;
    } word;
    struct {
      uint32_t *ptr;
      uint32_t len;
    } buf;
  };
} tlp_data_t;

typedef struct pcie_tlp_info_s {
  int valid;          /*!< Valid */
  pcie_intf_t instance; /*!< PCIe interface instance */
  int is_io;          /*!< is this IO /Memory Transaction */
  int tlp_fmt;        /*!< PCIe : TLP Format - 3 bits */
  int tlp_type;       /*!< PCIe : TLP Type : 5 bits */
  int rqstr_id;       /*!< PCIe : Requestetr ID */
  int rqstr_id_en;    /*!< PCIe : Requester ID Enable */
  int cpltr_id;       /*!< PCIe : Completer ID */
  int cpl_status;     /*!< PCIe : Completion Status */
  int poisoned_req;   /*!< PCIe : EP Field */
  int at;             /*!< PCIe : Address Type Field */
  int tc;             /*!< PCIe : Traffic Class Field */
  int bcnt;           /*!< PCIe : Byte Count */
  int attr;           /*!< PCIe : Attr field (IDO, RO, NO_SNOOP) */
  int tag;            /*!< PCIe : Tag for NPR/Read requests */
  int tph_ind_tag_en; /*!< TPH indirect tag enable */
  int tph_st_tag;     /*!< PCIe : TPH Steering Tag */
  int tph_type;       /*!< PCIe : Processing hints */
  int tph_present;    /*!< PCIe : TPH enable */
  int addr_offset;    /*!< Address offset to correct the address */
  int last_be;        /*!< PCIe : lbe */
  int first_be;       /*!< PCIe : fbe */
  int msg_code;       /*!< PCIe : Message code for Msg requests */
  int addr_lo;        /*!< lower addr: valid for completions */
  int addr_hi;        /*!< Higher 32 bits of address */
  int cpl_dwords;     /*!< Used only by the completion path */
} pcie_tlp_info_t;

typedef struct {
  int highest_visible_pf;
  int dev_cap;
  int msix_tbl_off[PCI_MAX_FNS_PER_IFACE];
  int msix_pba_off[PCI_MAX_FNS_PER_IFACE];
} pcie_core_settings_t;

typedef struct pcie_state_s pcie_state_t;
typedef struct pcie_settings_s pcie_settings_t;

/* Functions provided to the model */
extern void fs_pci_lies_init(struct fs_hw_s *hw,  const pcie_settings_t *settings_in, pcie_state_t **state_out, pcie_intf_t pcie_instance);

extern void fs_pci_assert_intx(pcie_state_t *state, pcie_func_t *func, bool onoff); 

/* Passing in NULL for the context implies a synchronous call */
extern bool pcie_host_read(pcie_state_t *state, uint32_t rid, void *dest_buffer,
                           uint64_t host_addr, size_t len, uint8_t tag, void *context);

extern void fs_pcie_host_write(pcie_state_t *state, uint32_t rid, uint64_t host_addr,
                  void *src_buffer, size_t len);

/* Send a request to the host. The caller is responsible for ensuring that
 * the request can go out as a single TLP */
extern void pcie_send_dut_to_host_cmodel(pcie_state_t *state,
                                         uint64_t host_addr,
                                         void *src_buffer,
                                         size_t len,
                                         bool is_write,
                                         void *context,
                                         pcie_tlp_info_t *tlp_info);

extern uint32_t fs_pci_cfg_read(struct mc_s *mc,
                                pcie_state_t *state,
                                unsigned int pf,
                                unsigned int vf,
                                bool vf_active,
                                unsigned int addr);
extern void fs_pci_cfg_write(struct mc_s *mc,
                             pcie_state_t *state,
                             unsigned int pf,
                             unsigned int vf,
                             bool vf_active,
                             unsigned int addr,
                             bool cs2,
                             unsigned int be,
                             uint32_t val);

extern void fs_pcie_complete_host_read(pcie_state_t *state,
                                       int tag,
                                       uint32_t *data,
                                       uint32_t num_dwords,
                                       pcie_tlp_info_t *tlp_info);

extern pcie_func_t *fs_rid_to_func(pcie_state_t *state, unsigned rid);

extern uint64_t fs_get_cycles(int instance);

extern bool pcie_retry_active(pcie_state_t *state);

extern void fs_pci_cfg_retry_en(pcie_state_t *state, bool en_set, bool en_reset);

extern unsigned long long dpi_gettime(pcie_state_t *state);

/* Returns config space for the specified function as an array of bytes */
extern const uint8_t *fs_pcie_get_cfgspc(pcie_state_t *state, unsigned int pf, unsigned int vf, bool use_vf);

/*! \brief The offset of the VSEC extended capability into the configuration
 * space. */
uint16_t fs_pcie_get_cfgspc_xcap_vsec(pcie_state_t *state);

/* Pass posted credits back to the PCIe core, which will in turn return them to the host */
extern void pcie_release_posted_credits(pcie_state_t *state, int credits);

enum pcie_exit_decision {
  PCIE_EXIT_CLEAN = 1,
  PCIE_EXIT_ABORT = 2,
  PCIE_EXIT_CONTINUE = 3
};

/* Called to allow the application to supply a cosim address. This can return NULL, in which case
 * the cosim socket code will check the environment */
extern const char *fs_pcie_get_cosim_addr(int instance);

/* Convenience functions for getting at PCIe settings the caller may need to know about */
extern uint32_t pcie_max_payload(pcie_state_t *state);

extern uint32_t pcie_max_read_req_size(pcie_state_t *state);

/** Represents an implementation of a configuration space handler */
typedef struct {
  /** Invoked to handle a configuration space read TLP
   *
   * The function pointed to should decode the routing ID & address information
   * from @p tlp_info, handle the read accordingly & then call either
   * pcie_cfg_handler_cpl or pcie_cfg_handler_cpld as appropriate. Successful
   * reads must call pcie_cfg_handler_cpld to provide data.
   *
   * @param[in] tlp_info  description of the config access TLP
   * @param[in] user      opaque pointer provided to pcie_cfg_handler_set
   */
  void (*read)(const pcie_tlp_info_t *tlp_info, void *user);

  /** Invoked to handle a configuration space write TLP
   *
   * The function pointed to should decode the routing ID & address information
   * from @p tlp_info, handle the write accordingly & then call
   * pcie_cfg_handler_cpl.
   *
   * @param[in] tlp_info  description of the config access TLP
   * @param[in] data      data written by the host
   * @param[in] user      opaque pointer provided to pcie_cfg_handler_set
   */
  void (*write)(const pcie_tlp_info_t *tlp_info, uint32_t data, void *user);
} pcie_cfg_handler_t;

/** Set up a configuration space handler for a PCIe core
 *
 * Causes all configuration accesses sent to the PCIe core to be routed to a
 * configuration space handler - i.e. the functions pointed to by the read &
 * write fields of @p handler will be invoked to handle accesses to
 * configuration space. Those functions are expected to invoke
 * pcie_cfg_handler_cpl or pcie_cfg_handler_cpld to complete the configuration
 * access.
 *
 * @param[in] state    state of the PCIe core
 * @param[in] handler  the config space handler implementation to use
 * @param[in] user     opaque pointer to pass through to the user argument of
 *                     functions pointed to by byp
 */
extern void pcie_cfg_handler_set(pcie_state_t *state,
                                 const pcie_cfg_handler_t *handler,
                                 void *user);

/** Complete a configuration access transaction
 *
 * Generate a completion TLP (type Cpl) for the configuration access initiated
 * by the TLP described by @p cfg_tlp.
 *
 * @param[in] state    state of the PCIe core
 * @param[in] cfg_tlp  description of the config access TLP
 * @param[in] status   completion status
 */
extern void pcie_cfg_handler_cpl(pcie_state_t *state,
                                 const pcie_tlp_info_t *cfg_tlp,
                                 uint8_t status);

/** Complete a configuration access transaction, with data
 *
 * Generate a completion TLP (type CplD) for the configuration access initiated
 * by the TLP described by @p cfg_tlp, including data to return to the host.
 *
 * @param[in] state    state of the PCIe core
 * @param[in] cfg_tlp  description of the config access TLP
 * @param[in] status   completion status
 * @param[in] data     data to return to the host
 */
extern void pcie_cfg_handler_cpld(pcie_state_t *state,
                                  const pcie_tlp_info_t *cfg_tlp,
                                  uint8_t status,
                                  uint32_t data);

/* In: address and length. Out: fbe, lbe, bytes this tlp. Returns num_dword. */
unsigned calculate_bes_and_len(unsigned addr, unsigned len, unsigned max,
                               int *first, int *last, unsigned *this_time);

/* Functions in the model that the PCIe core calls into */

typedef struct pcie_core_callbacks_s {
  /*
   * Transmit TLPs (to SystemC)
   */
  void (*pcie_tx_tlp)(void *opaque, uint32_t *data, unsigned int len_u32);

  /* Process VDM */
  void (*pcie_vdm)(struct fs_hw_s *hw,
                    uint32_t *hdr,
                    uint32_t *payload,
                    int payload_dwords);

  pcie_core_settings_t *(*get_pcie_core_settings)(struct fs_hw_s *hw);

  /** Handle host write to a BAR
   *
   * @param[in] func    function BAR belongs to
   * @param[in] bar     BAR type
   * @param[in] bar_num BAR ID (see enum definition)
   * @param[in] offset  offset at BAT to which host performs a write
   * @param[in] data    value written
   * @param[in] be      byte-enables for this transaction (see PCI Express Base
   *                    Specification, First/Last DW Byte Enables Rules)
   * @param[in] credits TODO: describe credits system
   *
   * @returns number of credits to return
   */
  unsigned (*host_w32)(pcie_func_t *func,
                       pci_bar_type_t bar,
                       pci_bar_num_t bar_num,
                       unsigned int offset,
                       tlp_data_t data,
                       int credits,
                       pcie_tlp_info_t *tlp_info);

  /** Handle host read from a BAR
   *
   * @param[in]  func     function BAR belongs to
   * @param[in]  bar      BAR type
   * @param[in]  bar_num  BAR ID (see enum definition)
   * @param[in]  offset   offset at BAR from which host performs a read
   * @param[in]  be       byte-enables for this transaction (see PCI Express Base
   *                      Specification, First/Last DW Byte Enables Rules)
   * @param[in]  tag      transaction tag
   * @param[out] complete if true, the return value is valid (and will be passed
   *                      back to the host); if false, response will be issued later,
   *                      return value is invalid
   *
   * @returns data to pass back to the host
   */
  uint32_t (*host_r32)(pcie_func_t *func,
                       pci_bar_type_t bar,
                       pci_bar_num_t bar_num,
                       unsigned int offset,
                       tlp_data_t *rd_data,
                       int tag,
                       bool *complete,
                       pcie_tlp_info_t *tlp_info);

  void (*biu_dma_read_complete)(void *context,
                                uint16_t tag,
                                int len,
                                bool ok,
                                pcie_tlp_info_t *tlp_info);

  /*
   * Called from cosim_finish to allow the application to decide whether to
   * exit or not. Note that it is called *before* decrementing live_instances
   */
  enum pcie_exit_decision (*pcie_exit_hook)(const int live_instances,
                                            const int instance_closing);

  /*
   * Allow the app to check an access against any routing setup it may have. This
   * does not return pass/fail but may grumble to the log or hard fail if it doesn't like
   * the access. Used to check MSI-X read/writes which otherwise are handled entirely
   * in the PCIe core model.
   */
  void (*pcie_validate_access)(struct fs_hw_s *hw, pcie_func_t *func,
                               int bar, unsigned offset);

} pcie_core_callbacks_t;

void pcie_core_cb_pcie_tx_tlp(void *opaque, uint32_t *data,
                              unsigned int len_u32);
pcie_core_settings_t *pcie_core_cb_get_pcie_core_settings(pcie_state_t *s);

struct cosim_platform_state_s;
enum pcie_exit_decision pcie_core_cb_pcie_exit_hook(struct cosim_platform_state_s *c_s,
                                                    const int live_instances,
                                                    const int instance_closing);

/*
 * Input a TLP packet into the PCIe model (this function is only used in TLM
 * 2.0 simulations).
 *
 * @param[in] state    state of the PCIe core
 * @param[in] data     TLP packet data
 * @param[in] len      TLP packet data length (in bytes)
 */
void pcie_rx_tlp(pcie_state_t *state, uint8_t *data, unsigned int len);
#endif
