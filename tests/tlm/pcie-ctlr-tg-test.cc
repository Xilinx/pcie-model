/*
 * TLM-2.0 PCIe controller model test.
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
 */

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <inttypes.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"

#include "traffic-generators/tg-tlm.h"
#include "traffic-generators/traffic-desc.h"
#include "tlm-bridges/tlm2tlp-bridge.h"
#include "soc/interconnect/iconnect.h"

#include "test-modules/memory.h"
#include "test-modules/utils.h"

#include "tlm-modules/pcie-controller.h"

using namespace sc_core;
using namespace sc_dt;
using namespace std;
using namespace utils;

#define RAM_SIZE (8 * 1024)

#define NR_MASTERS	1
#define NR_DEVICES	5

#define PCI_VENDOR_ID_XILINX            (0x10ee)
#define PCI_DEVICE_ID_XILINX_EF100      (0x0080)
#define PCI_SUBSYSTEM_ID_XILINX_TEST    (0x000A)

#define PCI_CLASS_BASE_NETWORK_CONTROLLER     (0x02)

#define NUM_MSIX 8

#define KiB (1024)
#define MiB (1024 * KiB)

class MSIXDev
: public sc_core::sc_module
{
public:
	tlm_utils::simple_target_socket<MSIXDev> tgt_socket;
	sc_vector<sc_out<bool> > irq;
	sc_event m_irqevent;

	SC_HAS_PROCESS(MSIXDev);

	MSIXDev(sc_core::sc_module_name name, int n_irqs) :
		sc_module(name),
		tgt_socket("tgt-socket"),
		irq("irq", n_irqs),
		m_irqevent("irqevent")
	{
		tgt_socket.register_b_transport(this, &MSIXDev::b_transport);

		SC_THREAD(irq_thread);
	}

	virtual void b_transport(tlm::tlm_generic_payload& trans,
					sc_time& delay)
	{
		m_irqevent.notify();
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}

	void irq_thread()
	{
		while (true) {
			wait(m_irqevent);

			for (unsigned int i = 0; i < irq.size(); i++) {
				irq[i].write(true);
			}

			wait(sc_time(1, SC_US));

			for (unsigned int i = 0; i < irq.size(); i++) {
				irq[i].write(false);
			}
		}
	}
};

TrafficDesc transfers(merge({
	Read(0x0, 4),
		Expect(DATA(0xEE, 0x10, 0x80, 0x0), 4),
	Read(0x4, 4),
		Expect(DATA(0x0, 0x00, 0x10, 0x0), 4),
	Read(0x8, 4),
		Expect(DATA(0x0, 0x0, 0x0, 0x2), 4),

	//
	// Test checking BAR sizes
	//
	Write(0x10, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x10, 4),
		Expect(DATA(0x4, 0x0, 0x80, 0xFF), 4),

	Write(0x14, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x14, 4),
		Expect(DATA(0xFF, 0xFF, 0xFF, 0xFF), 4),

	Write(0x18, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x18, 4),
		Expect(DATA(0x4, 0x0, 0xFC, 0xFF), 4),

	Write(0x1c, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x1c, 4),
		Expect(DATA(0xFF, 0xFF, 0xFF, 0xFF), 4),

	Write(0x20, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x20, 4),
		Expect(DATA(0x4, 0x0, 0xFC, 0xFF), 4),

	Write(0x24, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x24, 4),
		Expect(DATA(0xFF, 0xFF, 0xFF, 0xFF), 4),

	//
	// Test checking Expansion ROM BAR size
	//
	Write(0x30, DATA(0xFF, 0xFF, 0xFF, 0xFF)),
	Read(0x30, 4),
		Expect(DATA(0x0, 0x0, 0x0, 0x0), 4),

	//
	// Configure BAR 0 position at 0xfc000000 : 0xfc7fffff
	// Configure BAR 2 position at 0xfe000000_00000000 : 0xfe000000_0003ffff
	// Configure BAR 4 position at 0xfe040000 : 0xfe07ffff
	//
	// (Skip init of BAR1 && BAR3)
	//
	Write(0x10, DATA(0x00, 0x00, 0x00, 0xfc)),
	Write(0x14, DATA(0x00, 0x00, 0x00, 0x00)),

	Write(0x18, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0x1c, DATA(0x00, 0x00, 0x00, 0xfe)),

	Write(0x20, DATA(0x00, 0x00, 0x04, 0xfe)),
	Write(0x24, DATA(0x00, 0x00, 0x00, 0x00)),

	Read(0x10, 4),
	Read(0x18, 4),
	Read(0x20, 4),

	//
	// Enable Memory Space
	//
        Write(0x4, DATA(0x2), 2),

	//
	// Test BAR0 at 0xfc000000
	//
	Read(0xfc000000, 4),
	Read(0xfc000004, 4),
	Read(0xfc000008, 4),
	Read(0xfc00000c, 4),
	Write(0xfc000000, DATA(0x11, 0x12, 0x13, 0x14)),
	Write(0xfc000004, DATA(0x21, 0x22, 0x23, 0x24)),
	Write(0xfc000008, DATA(0x31, 0x32, 0x33, 0x34)),
	Write(0xfc00000c, DATA(0x41, 0x42, 0x43, 0x44)),
	Read(0xfc000000, 4),
	Read(0xfc000004, 4),
	Read(0xfc000008, 4),
	Read(0xfc00000c, 4),

	//
	// Test BAR2 at 0xfe000000_00000000
	//
	Read(0xfe00000000000000, 4),
	Read(0xfe00000000000004, 4),
	Read(0xfe00000000000008, 4),
	Read(0xfe0000000000000c, 4),
	Write(0xfe00000000000000, DATA(0x11, 0x12, 0x13, 0x14)),
	Write(0xfe00000000000004, DATA(0x11, 0x12, 0x13, 0x14)),
	Write(0xfe00000000000008, DATA(0x21, 0x22, 0x23, 0x24)),
	Write(0xfe0000000000000c, DATA(0x31, 0x32, 0x33, 0x34)),
	Read(0xfe00000000000000, 4),
	Read(0xfe00000000000004, 4),
	Read(0xfe00000000000008, 4),
	Read(0xfe0000000000000c, 4),

	//
	// Configure MSI-X
	//
	Read(0xb0, 4),
		Expect(DATA(0x11, 0x00, 0x07, 0x00), 4),
	Write(0xb0, DATA(0x00, 0x80, 0x00, 0x80)),
		ByteEnable(DATA(0x00, 0x00, 0xFF, 0xFF), 4),
	Read(0xb0, 4),
		Expect(DATA(0x11, 0x00, 0x07, 0x80), 4),

	// MSI-X Table entry 0
	Write(0xfe040100, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040104, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040108, DATA(0x00, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04010c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 1
	Write(0xfe040110, DATA(0x04, 0x00, 0x00, 0x00)),
	Write(0xfe040114, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040118, DATA(0x11, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04011c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 2
	Write(0xfe040120, DATA(0x08, 0x00, 0x00, 0x00)),
	Write(0xfe040124, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040128, DATA(0x22, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04012c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 3
	Write(0xfe040130, DATA(0x0c, 0x00, 0x00, 0x00)),
	Write(0xfe040134, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040138, DATA(0x33, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04013c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 4
	Write(0xfe040140, DATA(0x10, 0x00, 0x00, 0x00)),
	Write(0xfe040144, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040148, DATA(0x44, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04014c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 5
	Write(0xfe040150, DATA(0x14, 0x00, 0x00, 0x00)),
	Write(0xfe040154, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040158, DATA(0x55, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04015c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 6
	Write(0xfe040160, DATA(0x18, 0x00, 0x00, 0x00)),
	Write(0xfe040164, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040168, DATA(0x66, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04016c, DATA(0x00, 0x00, 0x00, 0x00)),

	// MSI-X Table entry 7
	Write(0xfe040170, DATA(0x1c, 0x00, 0x00, 0x00)),
	Write(0xfe040174, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0xfe040178, DATA(0x77, 0xBB, 0xCC, 0xDD)),
	Write(0xfe04017c, DATA(0x00, 0x00, 0x00, 0x00)),

	// Trigger MSI-X
	Write(0x0000f000, DATA(0x00, 0x00, 0x00, 0x00)),

}));

TrafficDesc dma_transfers(merge({
	Read(0x0, 4),
		Expect(DATA(0x00, 0xBB, 0xCC, 0xDD), 4),
	Read(0x4, 4),
		Expect(DATA(0x11, 0xBB, 0xCC, 0xDD), 4),
	Read(0x8, 4),
		Expect(DATA(0x22, 0xBB, 0xCC, 0xDD), 4),
	Read(0xc, 4),
		Expect(DATA(0x33, 0xBB, 0xCC, 0xDD), 4),
	Read(0x10, 4),
		Expect(DATA(0x44, 0xBB, 0xCC, 0xDD), 4),
	Read(0x14, 4),
		Expect(DATA(0x55, 0xBB, 0xCC, 0xDD), 4),
	Read(0x18, 4),
		Expect(DATA(0x66, 0xBB, 0xCC, 0xDD), 4),
	Read(0x1c, 4),
		Expect(DATA(0x77, 0xBB, 0xCC, 0xDD), 4),

	Write(0x0, DATA(0x11, 0x12, 0x13, 0x14)),
	Write(0x4, DATA(0x21, 0x22, 0x23, 0x24)),
	Write(0x8, DATA(0x31, 0x32, 0x33, 0x34)),
	Write(0xc, DATA(0x41, 0x42, 0x43, 0x44)),

	Read(0x0, 4),
		Expect(DATA(0x11, 0x12, 0x13, 0x14), 4),
	Read(0x4, 4),
		Expect(DATA(0x21, 0x22, 0x23, 0x24), 4),
	Read(0x8, 4),
		Expect(DATA(0x31, 0x32, 0x33, 0x34), 4),
	Read(0xc, 4),
		Expect(DATA(0x41, 0x42, 0x43, 0x44), 4),
}));

PhysFuncConfig getPhysFuncConfig()
{
	PhysFuncConfig cfg;
	PMCapability pmCap;
	PCIExpressCapability pcieCap;
	MSIXCapability msixCap;
	uint32_t bar_flags = PCI_BASE_ADDRESS_MEM_TYPE_64;
	uint32_t msixTableSz = NUM_MSIX;
	uint32_t tableOffset = 0x100 | 4; // Table offset: 0, BIR: 4
	uint32_t pba = 0x140000 | 4; // BIR: 4

	cfg.SetPCIVendorID(PCI_VENDOR_ID_XILINX);
	cfg.SetPCIDeviceID(PCI_DEVICE_ID_XILINX_EF100);

	cfg.SetPCIClassProgIF(0);
	cfg.SetPCIClassDevice(0);
	cfg.SetPCIClassBase(PCI_CLASS_BASE_NETWORK_CONTROLLER);

	cfg.SetPCIBAR0(8 * MiB,  bar_flags);
	cfg.SetPCIBAR2(256 * KiB, bar_flags);
	cfg.SetPCIBAR4(256 * KiB, bar_flags);

	cfg.SetPCISubsystemVendorID(PCI_VENDOR_ID_XILINX);
	cfg.SetPCISubsystemID(PCI_SUBSYSTEM_ID_XILINX_TEST);
	cfg.SetPCIExpansionROMBAR(0, 0);

	cfg.AddPCICapability(pmCap);
	cfg.AddPCICapability(pcieCap);

	msixCap.SetMessageControl(msixTableSz-1);
	msixCap.SetTableOffsetBIR(tableOffset);
	msixCap.SetPendingBitArray(pba);
	cfg.AddPCICapability(msixCap);

	return cfg;
}

SC_MODULE(Top)
{
public:
        TLMTrafficGenerator tg;
        TLMTrafficGenerator tg_dma;
	iconnect<NR_MASTERS, NR_DEVICES> *bus;

	PCIeController pcie_ctrlr;
	tlm2tlp_bridge tlm2tlp;

	memory b0_mem;
	memory b2_mem;
	memory b4_mem;
	tlm_utils::simple_target_socket<Top> dummy_b1_tgt_socket;
	tlm_utils::simple_target_socket<Top> dummy_b3_tgt_socket;
	tlm_utils::simple_target_socket<Top> dummy_b5_tgt_socket;

	MSIXDev msixdev;

	//
	// For testing DMA towards host
	//
	memory host_mem;

	sc_clock clk;
	sc_signal<bool> rst;
	sc_signal<bool> rst_n;

	SC_HAS_PROCESS(Top);

	void pull_reset(void) {
		/* Pull the reset signal.  */
		rst.write(true);
		wait(1, SC_US);
		rst.write(false);
	}
	void gen_rst_n(void) {
		rst_n.write(!rst.read());
	}

	Top(sc_module_name name) :
		sc_module(name),
        	tg("tg"),
		tg_dma("tg-dma"),

		pcie_ctrlr("pcie-ctrlr", getPhysFuncConfig()),
		tlm2tlp("tlm2tlp-bridge"),

		b0_mem("b0_mem", sc_time(0, SC_NS), RAM_SIZE),
		b2_mem("b2_mem", sc_time(0, SC_NS), RAM_SIZE),
		b4_mem("b4_mem", sc_time(0, SC_NS), RAM_SIZE),

		dummy_b1_tgt_socket("dummy-b1-tgt-socket"),
		dummy_b3_tgt_socket("dummy-b3-tgt-socket"),
		dummy_b5_tgt_socket("dummy-b5-tgt-socket"),

		msixdev("msixdev", NUM_MSIX),

		host_mem("host_mem", sc_time(0, SC_NS), RAM_SIZE),

		clk("clk", sc_time(10, SC_MS)),
		rst("rst"),
		rst_n("rst_n")
	{
		SC_THREAD(pull_reset);
		SC_METHOD(gen_rst_n);

		sensitive << rst;

		//
		// Setup traffic generator
		//
		tg.addTransfers(transfers);
		tg.enableDebug();

		//
		// Setup bus
		//
		bus = new iconnect<NR_MASTERS, NR_DEVICES> ("bus");

		bus->memmap(0x00000000ULL, 0x1000 - 1,
				ADDRMODE_ABSOLUTE, -1, tlm2tlp.cfg_tgt_socket);

		bus->memmap(0x00008000ULL, 0x1000 - 1,
				ADDRMODE_ABSOLUTE, -1, tlm2tlp.io_tgt_socket);

		bus->memmap(0x0000c000ULL, 0x1000 - 1,
				ADDRMODE_ABSOLUTE, -1, tlm2tlp.msg_tgt_socket);

		bus->memmap(0x0000f000ULL, 0x1000 - 1,
				ADDRMODE_ABSOLUTE, -1, msixdev.tgt_socket);

		bus->memmap(0xfc000000ULL, 0xfdffffff04040000 - 1,
				ADDRMODE_ABSOLUTE, -1, tlm2tlp.mem_tgt_socket);

		tg.socket.bind(*(bus->t_sk[0]));

		//
		// Setup TLP sockets (tlm2tlp <-> pcie-ctrlr)
		//
		tlm2tlp.init_socket.bind(pcie_ctrlr.tgt_socket);
		pcie_ctrlr.init_socket.bind(tlm2tlp.tgt_socket);

		//
		// Bar connections to mem
		//
		pcie_ctrlr.bar0_init_socket.bind(b0_mem.socket);
		pcie_ctrlr.bar2_init_socket.bind(b2_mem.socket);
		pcie_ctrlr.bar4_init_socket.bind(b4_mem.socket);

		//
		// Bar dummy connections
		//
		pcie_ctrlr.bar1_init_socket.bind(dummy_b1_tgt_socket);
		pcie_ctrlr.bar3_init_socket.bind(dummy_b3_tgt_socket);
		pcie_ctrlr.bar5_init_socket.bind(dummy_b5_tgt_socket);

		//
		// Setup DMA to host memory traffic generator
		//
		tg_dma.addTransfers(dma_transfers);
		tg_dma.enableDebug();
		tg_dma.setStartDelay(sc_time(1000, SC_NS));

		tg_dma.socket(pcie_ctrlr.dma_tgt_socket);
		tlm2tlp.dma_init_socket(host_mem.socket);

		//
		// Connect interrupts
		//
		for (int i = 0; i < NUM_MSIX; i++) {
			msixdev.irq[i](pcie_ctrlr.signals_irq[i]);
		}
	}
};

void usage(void)
{
	cout << "tlm socket-path sync-quantum-ns + <cdx main args>" << endl;
}

int sc_main(int argc, char* argv[])
{
	Top *top = new Top("top");

	cout << "Run: " << top->name() << endl;

        sc_start(10000, SC_MS);
	sc_stop();

	return 0;
}
