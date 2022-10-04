/*
 * TLM-2.0 PCIe controller model.
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
#include <byteswap.h>
#include "pcie-controller.h"
#include <sstream>

PCIeController::PCIeController(sc_core::sc_module_name name,
				PhysFuncConfig cfg,
				bool aligner_enable) :
	sc_module(name),
	init_socket("init_socket"),
	tgt_socket("tgt-socket"),

	bar0_init_socket("bar0_init_socket"),
	bar1_init_socket("bar1_init_socket"),
	bar2_init_socket("bar2_init_socket"),
	bar3_init_socket("bar3_init_socket"),
	bar4_init_socket("bar4_init_socket"),
	bar5_init_socket("bar5_init_socket"),

	dma_tgt_socket("dma_tgt_socket"),

	signals_irq("signals_irq", cfg.GetNumIrqs()),

	ats_req("ats_req"),
	ats_inv("ats_inv"),
	cfgspc_snoop("cfgspc_snoop"),

	irq("irq", cfg.GetNumIrqs()),

	m_aligner(NULL),
	proxy_init_socket(NULL),
	proxy_target_socket(NULL),

	m_tx_event("tx-event"),
	m_wr_event("wr-event"),

	m_dma_ongoing(false),
	m_dma_done_event("dma-done-event")
{
	m_cfg = cfg; // store the cfg

	init();

	tgt_socket.register_b_transport(this,
				&PCIeController::b_transport);
	ats_req.register_b_transport(this,
				&PCIeController::b_transport_ats_req);

	if (aligner_enable) {
		m_aligner = new tlm_aligner("aligner", SZ_4KB,
						SZ_4KB, SZ_4KB, true),
		proxy_init_socket = new
			tlm_utils::simple_initiator_socket<PCIeController>
				("proxy_init_socket");

		proxy_target_socket = new
			tlm_utils::simple_target_socket<PCIeController>
				("proxy_target_socket");

		dma_tgt_socket.register_b_transport(this,
			&PCIeController::b_transport_proxy);

		proxy_target_socket->register_b_transport(this,
			&PCIeController::b_transport_dma);

		proxy_init_socket->bind(m_aligner->target_socket);
		m_aligner->init_socket.bind((*proxy_target_socket));
	} else {
		dma_tgt_socket.register_b_transport(this,
					&PCIeController::b_transport_dma);
	}

	SC_THREAD(TLP_tx_thread);
	SC_THREAD(memwr_thread);

	for (unsigned int i = 0; i < cfg.GetNumIrqs(); i++) {
		irq[i](signals_irq[i]);
	}

	for (unsigned int i = 0; i < irq.size(); i++) {
		sc_spawn(sc_bind(&PCIeController::irq_thread,
				this,
				i));
	}
}

PCIeController::~PCIeController() {
	delete proxy_init_socket;
	delete proxy_target_socket;
	delete m_aligner;
}

void PCIeController::irq_thread(unsigned int i)
{
	pcie_func_t *func = fs_rid_to_func(m_pcie_state, 0);

	if (!func) {
		SC_REPORT_ERROR("PCIeController",
				"Function not found when configuring MSI-X");
	}

	while (true) {
		tlm::tlm_generic_payload trans;
		wait(irq[i].posedge_event());

		//
		// One outstanding DMA transaction is supported at the moment
		//
		while (m_dma_ongoing) {
			wait(m_dma_done_event);
		}
		m_dma_ongoing = true;

		pcie_hw_msix_irq(m_pcie_state, func, i);

		//
		// Wait for the transaction to finish
		//
		wait(m_dma_done_event);
		wait(SC_ZERO_TIME);
	}
}

void PCIeController::init()
{
	std::vector<PCIBARConfig> &bars = m_cfg.GetBarConfigs();
	unsigned int i = 0;

	memset(reinterpret_cast<void*>(&m_pcie_settings), 0x0, sizeof(m_pcie_settings));
	memset(reinterpret_cast<void*>(&m_core_settings), 0x0, sizeof(m_core_settings));

	init_pcie_core_callbacks(&m_core_cbs);
	init_pcie_cfgspc_callbacks(&m_cfg_cbs);

	m_pcie_settings.max_pfs = 1;
	m_pcie_settings.max_vfs = 0;
	m_pcie_settings.max_total_funcs = 1;
	m_pcie_settings.initial_cfg_retry = false;
	m_pcie_settings.process_tlp_once = true;
	m_pcie_settings.tag_type = TAG_8BIT;

	m_pcie_settings.pf_cfgspc_fixups[0].addr = 0xffff; // CFGSPC_FIXUPS_END
	m_pcie_settings.vf_cfgspc_fixups[0].addr = 0xffff; // CFGSPC_FIXUPS_END

	m_pcie_settings.pf_new_xcaps[0].id = 0; // XCAP_ADDITIONS_END
	m_pcie_settings.vf_new_xcaps[0].id = 0; // XCAP_ADDITIONS_END

	m_pcie_settings.core_cbs = &m_core_cbs;
	m_pcie_settings.cfg_cbs = &m_cfg_cbs;

	for (std::vector<PCIBARConfig>::iterator it = bars.begin();
		it != bars.end(); it++, i++) {
		PCIBARConfig &bar = (*it);
		uint64_t bar_sz = bar.GetSize();

		m_pcie_settings.pf_barmask_fixups[i].bar = bar.GetBARNum();
		m_pcie_settings.pf_barmask_fixups[i].mask = bar_sz - 1;
		m_pcie_settings.pf_barmask_fixups[i].flags = bar.GetFlags();
		m_pcie_settings.pf_barmask_fixups[i].pf_mask = 1;

		//
		// Setup 64bit BAR
		//
		if (bar.Is64BitBAR()) {
			i++;
			m_pcie_settings.pf_barmask_fixups[i].bar = bar.GetBARNum() + 1;
			m_pcie_settings.pf_barmask_fixups[i].mask = (bar_sz - 1) >> 32;
			m_pcie_settings.pf_barmask_fixups[i].flags = 0;
			m_pcie_settings.pf_barmask_fixups[i].pf_mask = 1;
		}
	}
	m_pcie_settings.pf_barmask_fixups[i].bar = -1; // BARMASK_FIXUPS_END
	m_pcie_settings.vf_barmask_fixups[i].bar = -1; // BARMASK_FIXUPS_END

	//
	// Init pci
	//
	fs_pci_lies_init(reinterpret_cast<fs_hw_s*>(this),
			 &m_pcie_settings, &m_pcie_state, PCIE_INTF_HOST);
}

uint8_t PCIeController::GetFmt(uint8_t data)
{
	return data >> TLP0_TYPE_WIDTH;
}

uint8_t PCIeController::GetType(uint8_t data)
{
	uint8_t mask = (1 << TLP0_TYPE_WIDTH) - 1;
	return data & mask;
}

void PCIeController::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	unsigned int hdrlen;

	assert(trans.get_data_length() >= TLPHdr_3DW_Sz);
	hdrlen = GetHdrLen(trans.get_data_ptr()[0]);

	//
	// ByteSwap the header for libpcie
	//
	ByteSwap(trans.get_data_ptr(), hdrlen);
#endif
	pcie_rx_tlp(m_pcie_state, trans.get_data_ptr(), trans.get_data_length());
	prod_pcie(m_pcie_state);

	trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

void PCIeController::b_transport_proxy(tlm::tlm_generic_payload& trans, sc_time& delay)
{
	uint32_t max_sz = pcie_max_payload(m_pcie_state);
	uint32_t max_rd_sz = pcie_max_read_req_size(m_pcie_state);

	if (trans.get_byte_enable_length()) {
		SC_REPORT_ERROR("PCIeController",
				"Byte enables on DMA transactions "
				"are currently not supported");
	}
	if (trans.get_streaming_width() < trans.get_data_length()) {
		SC_REPORT_ERROR("PCIeController",
				"Streaming width < data length on DMA "
				"transactions are currently not supported");
	}
	if (trans.is_read() && max_rd_sz < max_sz) {
		max_sz = max_rd_sz;
	}
	m_aligner->set_max_len(max_sz);
	(*proxy_init_socket)->b_transport(trans, delay);
}

void PCIeController::b_transport_dma(tlm::tlm_generic_payload& trans, sc_time& delay)
{
	//
	// One outstanding DMA transaction is supported at the moment
	//
	while (m_dma_ongoing) {
		wait(m_dma_done_event);
	}
	m_dma_ongoing = true;

	//
	// Wait for the delay before generating the TLP
	//
	wait(delay);
	delay = SC_ZERO_TIME;

	if (trans.is_read()){
		PCIeHostRead(trans, delay);
	} else if (trans.is_write()) {
		PCIeHostWrite(trans, delay);
	}

	//
	// Wait for the transaction to finish
	//
	wait(m_dma_done_event);
	wait(SC_ZERO_TIME);

	trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

void PCIeController::PCIeHostRead(tlm::tlm_generic_payload& trans, sc_time& delay)
{
	uint32_t tag = 0;
	uint32_t pf = 0;
	uint64_t addr = trans.get_address();
	void *data = static_cast<void*>(trans.get_data_ptr());
	unsigned len = trans.get_data_length();
	void *context = static_cast<void*>(m_pcie_state);

	pcie_host_read(m_pcie_state, pf, data, addr, len, tag, context);
}

void PCIeController::PCIeHostReadDone()
{
	//
	// DMA read data has been received, the DMA transaction has been
	// completed.
	//
	m_dma_ongoing = false;
	m_dma_done_event.notify();
}

void PCIeController::PCIeHostWrite(tlm::tlm_generic_payload& trans, sc_time& delay)
{
	uint32_t pf = 0;

	//
	// Ends up calling 'tx_tlp'
	//
	fs_pcie_host_write(m_pcie_state,
			pf,
			trans.get_address(),
			trans.get_data_ptr(),
			trans.get_data_length());
}

void PCIeController::ByteSwap(uint8_t *data, unsigned int len)
{
	assert(len > 0);
	assert((len % 4) == 0);
	for (unsigned int i = 0; i < len; i += 4) {
		uint32_t *d_u32 = reinterpret_cast<uint32_t*>(&data[i]);
		d_u32[0] = bswap_32(d_u32[0]);
	}
}

unsigned int PCIeController::GetHdrLen(uint8_t data)
{
	unsigned int hdrlen;
	uint8_t fmt;

	fmt = GetFmt(data);
	hdrlen = TLP_FMT_IS_4DW(fmt) ? TLPHdr_4DW_Sz : TLPHdr_3DW_Sz;

	return hdrlen;
}

void PCIeController::tx_tlp(uint8_t *data, unsigned int len)
{
	tlm::tlm_generic_payload *gp = new tlm::tlm_generic_payload();
	uint8_t *gp_data = new uint8_t[len];
	const unsigned MaxTxListSize = 20;

#if __BYTE_ORDER == __LITTLE_ENDIAN
	unsigned int hdrlen;

	assert(len >= TLPHdr_3DW_Sz);
	hdrlen = GetHdrLen(data[3]);

	//
	// ByteSwap the header generated by libpcie
	//
	ByteSwap(data, hdrlen);
#endif

	memcpy(gp_data, data, len);

	gp->set_command(tlm::TLM_WRITE_COMMAND);
	gp->set_data_ptr(gp_data);
	gp->set_data_length(len);
	gp->set_streaming_width(len);

	if (m_txList.size() > MaxTxListSize) {
		SC_REPORT_ERROR("PCIeController",
				"TLP TX list stall detected (max size hit)");
	}
	m_txList.push_back(gp);
	m_tx_event.notify();
}

bool PCIeController::IsTLPMemWr(tlm::tlm_generic_payload *gp)
{
	uint8_t *d = gp->get_data_ptr();
	unsigned int len = gp->get_data_length();
	uint8_t fmt;
	uint8_t type;

	assert(len);

	fmt = GetFmt(d[0]);
	type = GetType(d[0]);

	if (type == 0 &&
		(fmt == TLP0_FMT_3DW_DATA || fmt == TLP0_FMT_4DW_DATA)) {
		// MemWr 3DW or 4DW data header
		return true;
	}

	return false;
}

void PCIeController::TLP_tx_thread()
{
	while (true) {
		tlm::tlm_generic_payload *tlp;
		sc_time delay(SC_ZERO_TIME);

		if (m_txList.empty()) {
			wait(m_tx_event);
		}
		tlp = m_txList.front();

		init_socket->b_transport(*tlp, delay);

		//
		// Wait for the annotated delay
		//
		wait(delay);

		//
		// If it is a DMA write towards host the transaction is done
		// here
		//
		if (IsTLPMemWr(tlp) || IsCplD(tlp)) {
			m_dma_ongoing = false;
			m_dma_done_event.notify();
		}

		m_txList.remove(tlp);
		delete[] tlp->get_data_ptr();
		delete tlp;
	}
}

void PCIeController::handle_MemRd(unsigned int bar_num, uint64_t addr,
				  uint8_t *data, unsigned int len)
{
	tlm::tlm_generic_payload trans;
	sc_time delay(SC_ZERO_TIME);

	trans.set_command(tlm::TLM_READ_COMMAND);
	trans.set_address(addr);
	trans.set_data_ptr(data);
	trans.set_data_length(len);
	trans.set_streaming_width(len);

	//
	// One outstanding DMA transaction is supported at the moment
	//
	while (m_dma_ongoing) {
		wait(m_dma_done_event);
	}
	m_dma_ongoing = true;

	switch (bar_num) {
	case 0:
		bar0_init_socket->b_transport(trans, delay);
		break;
	case 1:
		bar1_init_socket->b_transport(trans, delay);
		break;
	case 2:
		bar2_init_socket->b_transport(trans, delay);
		break;
	case 3:
		bar3_init_socket->b_transport(trans, delay);
		break;
	case 4:
		bar4_init_socket->b_transport(trans, delay);
		break;
	case 5:
		bar5_init_socket->b_transport(trans, delay);
		break;
	default:
		break;
	};

	wait(delay);
	delay = SC_ZERO_TIME;
}

bool PCIeController::IsCplD(tlm::tlm_generic_payload *gp)
{
	uint8_t *d = gp->get_data_ptr();
	unsigned int len = gp->get_data_length();
	uint8_t fmt;
	uint8_t type;

	assert(len);

	fmt = GetFmt(d[0]);
	type = GetType(d[0]);

	if (type == TLP0_TYPE_CPL && fmt == TLP0_FMT_3DW_DATA) {
		// CplD
		return true;
	}

	return false;
}

void PCIeController::handle_MemWr(unsigned int bar_num, uint64_t addr,
				  uint8_t *data, unsigned int len)
{
	WrTxn *txn = new WrTxn(bar_num, addr, data, len);
	m_wrList.push_back(txn);
	m_wr_event.notify();
}

void PCIeController::memwr_thread()
{
	while (true) {
		WrTxn *txn;
		sc_time delay(SC_ZERO_TIME);

		if (m_wrList.empty()) {
			wait(m_wr_event);
		}
		txn = m_wrList.front();

		switch (txn->GetBarNum()) {
		case 0:
			bar0_init_socket->b_transport(txn->GetGP(), delay);
			break;
		case 1:
			bar1_init_socket->b_transport(txn->GetGP(), delay);
			break;
		case 2:
			bar2_init_socket->b_transport(txn->GetGP(), delay);
			break;
		case 3:
			bar3_init_socket->b_transport(txn->GetGP(), delay);
			break;
		case 4:
			bar4_init_socket->b_transport(txn->GetGP(), delay);
			break;
		case 5:
			bar5_init_socket->b_transport(txn->GetGP(), delay);
			break;
		default:
			break;
		};

		//
		// Wait for the annotated delay
		//
		wait(delay);

		m_wrList.remove(txn);
		delete txn;
	}
}

pcie_core_settings_t *PCIeController::GetPCIeCoreSettings()
{
	return &m_core_settings;
}

pcie_state_t *PCIeController::GetPCIeState()
{
	return m_pcie_state;
}

pci_bar_type_t PCIeController::GetBarType(int fn, pci_bar_num_t bar_num)
{
	std::vector<PCIBARConfig> &bars = m_cfg.GetBarConfigs();

	//
	// Exclusive MSI-X BAR at the moment
	//
	if (m_cfg.IsBARTypeMSIX(bar_num)) {
		return PCI_BAR_TYPE_MSIX;
	}

	for (std::vector<PCIBARConfig>::iterator it = bars.begin();
		it != bars.end(); it++) {
		PCIBARConfig &bar = (*it);
		uint32_t b_num = static_cast<uint32_t>(bar_num);

		//
		// Check that the BAR has a configured size
		//
		if (bar.GetSize() == 0) {
			continue;
		}

		//
		// Check that BAR num matches (or in case it is 64 bit BAR,
		// that it matches the next BAR)
		//
		if (b_num == bar.GetBARNum() ||
                    (bar.Is64BitBAR() && b_num == (bar.GetBARNum() + 1))) {

			if (bar.IsBARTypeMem()) {
				return PCI_BAR_TYPE_MEM;
			} else {
				return PCI_BAR_TYPE_IO;
			}
		}
	}
	return PCI_BAR_TYPE_INVAL;
}

void PCIeController::b_transport_tieoff(tlm::tlm_generic_payload& trans,
					sc_time& delay)
{
	std::ostringstream msg;

	msg << name() << ": Tied off socket accessed"  << endl;
	SC_REPORT_WARNING("PCIeController", msg.str().c_str());
	trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
}

void PCIeController::b_transport_ats_req(tlm::tlm_generic_payload& trans,
						sc_time& delay)
{
	SC_REPORT_WARNING("PCIeController",
				"ATS requests are not yet supported\n");
	trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
}

void
PCIeController::tieoff(tlm_utils::simple_initiator_socket<PCIeController> &i_sock,
			const char *name)
{
	tlm_utils::simple_target_socket<PCIeController> *tieoff_sk =
		new tlm_utils::simple_target_socket<PCIeController>(name);

	tieoff_sk->register_b_transport(this,
					&PCIeController::b_transport_tieoff);
	i_sock.bind(*tieoff_sk);
}

void
PCIeController::tieoff(tlm_utils::simple_target_socket<PCIeController> &t_sock,
			const char *name)
{
	tlm_utils::simple_initiator_socket<PCIeController> *tieoff_sk =
		new tlm_utils::simple_initiator_socket<PCIeController>(name);
	tieoff_sk->bind(t_sock);
}

void PCIeController::before_end_of_elaboration()
{
	if (!bar0_init_socket.size()) {
		tieoff(bar0_init_socket, "bar0-tieoff");
	}
	if (!bar1_init_socket.size()) {
		tieoff(bar1_init_socket, "bar1-tieoff");
	}
	if (!bar2_init_socket.size()) {
		tieoff(bar2_init_socket, "bar2-tieoff");
	}
	if (!bar3_init_socket.size()) {
		tieoff(bar3_init_socket, "bar3-tieoff");
	}
	if (!bar4_init_socket.size()) {
		tieoff(bar4_init_socket, "bar4-tieoff");
	}
	if (!bar5_init_socket.size()) {
		tieoff(bar5_init_socket, "bar5-tieoff");
	}
	if (!dma_tgt_socket.size()) {
		tieoff(dma_tgt_socket, "dma_tgt_socket-tieoff");
	}
	if (!ats_req.size()) {
		tieoff(ats_req, "ats_req-tieoff");
	}
	if (!ats_inv.size()) {
		tieoff(ats_inv, "ats_inv->tieoff");
	}
	if (!cfgspc_snoop.size()) {
		tieoff(cfgspc_snoop, "cfgspc_snoop->tieoff");
	}
}
