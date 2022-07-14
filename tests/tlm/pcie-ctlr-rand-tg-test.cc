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
#include "traffic-generators/random-traffic.h"
#include "tlm-bridges/tlm2tlp-bridge.h"
#include "soc/interconnect/iconnect.h"

#include "test-modules/memory.h"
#include "test-modules/utils.h"

#include "tlm-modules/pcie-controller.h"

using namespace sc_core;
using namespace sc_dt;
using namespace std;
using namespace utils;

#define NR_MASTERS	1
#define NR_DEVICES	4

#define PCI_VENDOR_ID_XILINX            (0x10ee)
#define PCI_DEVICE_ID_XILINX_EF100      (0x0080)
#define PCI_SUBSYSTEM_ID_XILINX_TEST    (0x000A)

#define PCI_CLASS_BASE_NETWORK_CONTROLLER     (0x02)

#define KiB (1024)
#define MiB (1024 * KiB)

#define PCIE_CFGSPC_SIZE (4 * KiB)
#define RAM_SIZE (8 * KiB)
#define BAR0_ADDR 0xfc000000

#ifdef CONFIG_64BIT_BAR
#define BAR2_ADDR 0xfe00000000000000
#else
#define BAR2_ADDR 0xfe000000
#endif

sc_event *tgDoneEvent;
void tgDoneCB(TLMTrafficGenerator *gen, int threadId)
{
	if (tgDoneEvent) {
		tgDoneEvent->notify();
	}
}

TrafficDesc config_xfers(merge({
	Read(0x0, 4),
	Read(0x4, 4),
	Read(0x8, 4),
	Read(0xc, 4),

	//
	// Configure BAR 0 position at 0xfc000000 : 0xfc7fffff
	// (Skip init of BAR1 && BAR3)
	//
	Write(0x10, DATA(0x00, 0x00, 0x00, 0xfc)),

#if CONFIG_64BIT_BAR
	//
	// Configure BAR 2 position at 0xfe000000_00000000 : 0xfe000000_0003ffff
	//
	Write(0x18, DATA(0x00, 0x00, 0x00, 0x00)),
	Write(0x1c, DATA(0x00, 0x00, 0x00, 0xfe)),
#else
	//
	// Configure BAR 2 position at 0xfe000000 : 0xfe03ffff or
	//
	Write(0x18, DATA(0x00, 0x00, 0x00, 0xfe)),
#endif
	//
	// Configure BAR 4 position at 0xfe040000 : 0xfe07ffff
	//
	Write(0x20, DATA(0x00, 0x00, 0x04, 0xfe)),

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
	// Test BAR2 at 0xfe000000_00000000 or 0xfe000000
	//
	Read(BAR2_ADDR + 0x0, 4),
	Read(BAR2_ADDR + 0x4, 4),
	Read(BAR2_ADDR + 0x8, 4),
	Read(BAR2_ADDR + 0xc, 4),
	Write(BAR2_ADDR + 0x0, DATA(0x11, 0x12, 0x13, 0x14)),
	Write(BAR2_ADDR + 0x4, DATA(0x11, 0x12, 0x13, 0x14)),
	Write(BAR2_ADDR + 0x8, DATA(0x21, 0x22, 0x23, 0x24)),
	Write(BAR2_ADDR + 0xc, DATA(0x31, 0x32, 0x33, 0x34)),
	Read(BAR2_ADDR + 0x0, 4),
	Read(BAR2_ADDR + 0x4, 4),
	Read(BAR2_ADDR + 0x8, 4),
	Read(BAR2_ADDR + 0xc, 4),

	//
	// Configure MSI-X
	//
	Write(0xd8, DATA(0x00, 0x80, 0x00, 0x80)),
	ByteEnable(DATA(0x00, 0x00, 0xFF, 0xFF), 4),

	Write(0xfe040100, DATA(0x00, 0x00, 0x00, 0xb0)),
	Write(0xfe040104, DATA(0xAA, 0xBB, 0xCC, 0xDD)),

}));

PhysFuncConfig getPhysFuncConfig()
{
	PhysFuncConfig cfg;
	PMCapability pmCap;
	PCIExpressCapability pcieCap;
	MSIXCapability msixCap;
#ifdef CONFIG_64BIT_BAR
	uint32_t bar_flags = PCI_BASE_ADDRESS_MEM_TYPE_64;
#else
	uint32_t bar_flags = PCI_BASE_ADDRESS_MEM_TYPE_32;
#endif
	uint32_t msixTableSz = 8;
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
	RandomTraffic rand_cfg_transfers;
	RandomTraffic bar0_transfers;
	RandomTraffic bar2_transfers;
	RandomTraffic rand_dma_transfers;
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

	//
	// For testing DMA towards host
	//
	memory host_mem;

	sc_clock clk;
	sc_signal<bool> rst;
	sc_signal<bool> rst_n;

	sc_event tgDoneEvent;

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
		rand_cfg_transfers(0, PCIE_CFGSPC_SIZE, (~(0x3llu)), 4, 4, 0, 12000),
		bar0_transfers(BAR0_ADDR, BAR0_ADDR + RAM_SIZE, (~(0x3llu)), 4, 4, 0, 12000),
		bar2_transfers(BAR2_ADDR, BAR2_ADDR + RAM_SIZE, (~(0x3llu)), 4, 4, 0, 12000),
		rand_dma_transfers(0, RAM_SIZE, (~(0x3llu)), 4, 4, 0, 12000),
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

		host_mem("host_mem", sc_time(0, SC_NS), RAM_SIZE),

		clk("clk", sc_time(10, SC_MS)),
		rst("rst"),
		rst_n("rst_n"),

		tgDoneEvent("tg-done-event")
	{
		SC_THREAD(pull_reset);
		SC_METHOD(gen_rst_n);

		sensitive << rst;

		//
		// Setup traffic generator
		//
		tg.addTransfers(config_xfers, 0, tgDoneCB);
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
		tg_dma.enableDebug();
		tg_dma.socket(pcie_ctrlr.dma_tgt_socket);
		tlm2tlp.dma_init_socket(host_mem.socket);


		SC_THREAD(tg_done_thread);
	}

	//
	// Traffic generator done thread
	//
	void tg_done_thread()
	{
		while (true) {
			//
			// Wait for the transactions to finish
			//
			wait(tgDoneEvent);

			if (!bar0_transfers.done()) {
				//
				// Generate bar0 transfers
				//
				tg.addTransfers(bar0_transfers, 0,
						tgDoneCB);
			} else if (!bar2_transfers.done()) {
				//
				// Generate bar2 transfers
				//
				tg.addTransfers(bar2_transfers, 0,
						tgDoneCB);
			} else if (!rand_dma_transfers.done()) {
				//
				// Config done issue random DMA transfers
				//
				tg_dma.addTransfers(rand_dma_transfers, 0, tgDoneCB);
			} else if (!rand_cfg_transfers.done()) {
				//
				// Finish with issuing random cfgspc transfers
				//
				tg.addTransfers(rand_cfg_transfers, 0,
						tgDoneCB);
			} else {
				assert(config_xfers.done());
				assert(bar0_transfers.done());
				assert(bar2_transfers.done());
				assert(rand_dma_transfers.done());
				assert(rand_cfg_transfers.done());

				cout << endl << " ---  All transfers done!" << endl;
				sc_stop();
			}
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

	tgDoneEvent = &top->tgDoneEvent;

	cout << "Run: " << top->name() << endl;

	sc_start();

	return 0;
}
