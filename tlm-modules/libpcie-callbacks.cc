/*
 * TLM-2.0 PCIe controller model libpcie callbacks.
 *
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Francisco Iglesias.
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
#include "pcie-controller.h"

extern "C" {
	static void pcie_tx_tlp(void *opaque, uint32_t *data, unsigned int len_u32)
	{
		pcie_state_t *state = reinterpret_cast<pcie_state_t*>(opaque);
		PCIeController *pcie_ctrlr =
			reinterpret_cast<PCIeController*>(state->hw);

		pcie_ctrlr->tx_tlp(reinterpret_cast<uint8_t*>(data), len_u32 * 4);
	}

	static uint32_t fs_host_r32(pcie_func_t *pfunc,
			     pci_bar_type_t bar,
			     pci_bar_num_t bar_num,
			     unsigned int addr,
			     tlp_data_t *rd_data,
			     int tag,
			     bool *complete,
			     pcie_tlp_info_t *tlp_info)

	{
		PCIeController *pcie_ctrlr = reinterpret_cast<PCIeController*>(pfunc->hw);
		pcie_state_t *state = pcie_ctrlr->GetPCIeState();
		uint32_t dwlen = rd_data->buf.len;
		pcie_tlp_info_t cpl_info = { 0 };
		uint32_t bcnt = dwlen * 4;
		uint32_t data[1024];

		pcie_ctrlr->handle_MemRd(bar_num, addr,
				reinterpret_cast<uint8_t*>(&data), bcnt);

		cpl_info.tlp_fmt = 2;   		// 3DW hdr with Data
		cpl_info.tlp_type = 10;			// Type Cpl
		cpl_info.rqstr_id = tlp_info->rqstr_id; // requester ID
		cpl_info.cpltr_id = pfunc->rid;         // completer ID
		cpl_info.tag = tlp_info->tag;
		cpl_info.bcnt = bcnt;  			// Return all requested
		cpl_info.cpl_status = 0;		// Succesful completion
		cpl_info.valid = true;

		fs_pcie_complete_host_read(state, cpl_info.tag,
			&data[0], dwlen, &cpl_info);

		free(tlp_info);

		*complete = false;
		return 0x12345678;
	}

	static unsigned fs_host_w32(pcie_func_t *pfunc,
			     pci_bar_type_t bar,
			     pci_bar_num_t bar_num,
			     unsigned int addr,
			     tlp_data_t data,
			     int credits,
			     pcie_tlp_info_t *tlp_info)
	{
		PCIeController *pcie_ctrlr = reinterpret_cast<PCIeController*>(pfunc->hw);
		uint32_t dwlen = data.buf.len;
		uint32_t bcnt = dwlen * 4;

		pcie_ctrlr->handle_MemWr(bar_num, addr,
				reinterpret_cast<uint8_t*>(data.buf.ptr), bcnt);

		free(tlp_info);
		free(data.buf.ptr);

		return 0;
	}

	static pcie_core_settings_t *get_pcie_core_settings(struct fs_hw_s *hw)
	{
		PCIeController *pcie_ctrlr = reinterpret_cast<PCIeController*>(hw);
		return pcie_ctrlr->GetPCIeCoreSettings();
	}

	static void hwsim_pci_cfg_fill_reset_values(pcie_state_t *s,
						unsigned int pf,
						unsigned int vf,
						uint8_t *config_space)
	{
		PCIeController *pc = reinterpret_cast<PCIeController*>(s->hw);
		PhysFuncConfig &cfg = pc->GetPFConfig();

		memcpy(config_space, cfg.data(), cfg.size());
	}


	/*
	 * Initially unused callbacks
	 */
	static void fs_pcie_apply_app_vf_hacks(pcie_state_t *state,
			int parent_pf,
			int vf,
			uint8_t *config_space)
	{}

	static enum pcie_exit_decision fs_pcie_exit_hook(const int live_instances,
			const int instance_closing)
	{
		return PCIE_EXIT_CLEAN;
	}

	static void hw_exprom_init(pcie_func_t *fs_func)
	{
		printf("fs_hw_exprom_init: dummy\n");
	}

	static void hw_dpa_ctrl_notification(struct fs_hw_s *hw, int pf)
	{
		printf("Ignoring DPA update for PF %d\n", pf);
	}

	static void hw_pcie_vdm(struct fs_hw_s *hw,
			uint32_t *hdr,
			uint32_t *payload,
			int payload_dwords)
	{
		assert(0);
	}

	static void hw_set_pf_bme(struct fs_hw_s *hw, uint32_t pf, uint32_t enabled)
	{
		printf("fs_biu_set_pf_bme: dummy\n");
	}

	static void hw_set_vf_bme(struct fs_hw_s *hw, uint32_t vf, uint32_t enabled)
	{
		printf("fs_biu_set_vf_bme: dummy\n");
	}

	/*
	 * Functions that might later make calls into user logic callbacks
	 * PCIe controller can be grabbed through the func->hw
	 */
	static void hwsim_trigger_flr(struct mc_s *in_mc, pcie_func_t *func)
	{}

	static void hwsim_trigger_vpd(pcie_func_t *func, uint16_t cap_pos, uint16_t addr)
	{
	}

	static void fs_pcie_validate_access(struct fs_hw_s *hw,
			pcie_func_t *func,
			int bar,
			unsigned offset)
	{
	}

	static void biu_dma_read_complete(
			void *ctxt, uint16_t tag, int len, bool ok, pcie_tlp_info_t *tlp_info)
	{
		pcie_state_t *s = reinterpret_cast<pcie_state_t*>(ctxt);
		PCIeController *p = reinterpret_cast<PCIeController*>(s->hw);

		p->PCIeHostReadDone();
	}

	/*
	 * Allocate / free a pcie_func_t (called from inside
	 * libpcie/src/pcie/pcie_cfgspc.c).
	 *
	 * This is a variant of fs_new_pcie_func where the difference is that below
	 * callbacks do not wrap the pcie_func_t into another structure.
	 */
	static pcie_func_t *new_pcie_func(void *hw_opaque)
	{
		return reinterpret_cast<pcie_func_t*>(calloc(1, sizeof(pcie_func_t)));
	}

	static void free_pcie_func(pcie_func_t *pfunc)
	{
		free(pfunc);
	}

	pci_bar_type_t hwsim_bar_num_to_type(pcie_state_t *s,
						int fn, pci_bar_num_t bar)
	{
		PCIeController *pc = reinterpret_cast<PCIeController*>(s->hw);
		return pc->GetBarType(fn, bar);
	}

	void init_pcie_cfgspc_callbacks(pcie_cfgspc_callbacks_t *cfg_cbs)
	{
		cfg_cbs->fill_reset_values = hwsim_pci_cfg_fill_reset_values;
		cfg_cbs->trigger_vpd = hwsim_trigger_vpd;
		cfg_cbs->trigger_flr = hwsim_trigger_flr;
		cfg_cbs->bar_num_to_type = hwsim_bar_num_to_type;
		cfg_cbs->exprom_init = hw_exprom_init;
		cfg_cbs->dpa_ctrl_notification = hw_dpa_ctrl_notification;
		cfg_cbs->set_pf_bme = hw_set_pf_bme;
		cfg_cbs->set_vf_bme = hw_set_vf_bme;
		cfg_cbs->new_pcie_func = new_pcie_func;
		cfg_cbs->free_pcie_func = free_pcie_func;
		cfg_cbs->pcie_apply_app_vf_hacks = fs_pcie_apply_app_vf_hacks;
	}

	void init_pcie_core_callbacks(pcie_core_callbacks_t *core_cbs)
	{
		core_cbs->pcie_tx_tlp = pcie_tx_tlp;
		core_cbs->pcie_vdm = hw_pcie_vdm;
		core_cbs->get_pcie_core_settings = get_pcie_core_settings;
		core_cbs->host_r32 = fs_host_r32;
		core_cbs->host_w32 = fs_host_w32;
		core_cbs->biu_dma_read_complete = biu_dma_read_complete;
		core_cbs->pcie_exit_hook = fs_pcie_exit_hook;
		core_cbs->pcie_validate_access = fs_pcie_validate_access;
	}
}
