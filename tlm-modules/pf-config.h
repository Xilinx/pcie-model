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
#ifndef __PHYSICAL_FUNCTION_CONFIG_H__
#define __PHYSICAL_FUNCTION_CONFIG_H__

#include <vector>
#include <linux/pci_regs.h>

#define PCI_CLASS_BASE                        (0x0b)

#define GEN_PCI_CFGSPC_FIELD_FUNCS(func, field_offset, field_type)	\
void Set ## func(field_type value)					\
{									\
	Set<field_offset, field_type>(value);				\
}									\
field_type Get ## func()						\
{									\
	return Get<field_offset, field_type>();				\
}									\

#define GEN_PCI_CFGSPC_BAR_HELPER_FUNC(bar)				\
void SetPCIBAR ## bar(uint64_t bar_sz, uint32_t flags)			\
{									\
	m_bars.push_back(PCIBARConfig(bar, bar_sz, flags));		\
}									\

class PCICapability
{
public:
	PCICapability(uint32_t id, uint32_t sz) :
		m_cap(sz, 0)
	{
		SetCapabilityID(id);
	}

	GEN_PCI_CFGSPC_FIELD_FUNCS(CapabilityID, 0, uint8_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(NextCapPtr, 1, uint8_t);

	template<uint32_t offset, typename T>
	void Set(T val)
	{
		if (offset <= (m_cap.size()-sizeof(T))) {
			T *pos = reinterpret_cast<T*>(&m_cap[offset]);
			pos[0] = val;
		}
	}
	template<uint32_t offset, typename T>
	T Get()
	{
		T ret = 0;
		if (offset <= (m_cap.size()-sizeof(T))) {
			T *pos = reinterpret_cast<T*>(&m_cap[offset]);
			ret = pos[0];
		}
		return ret;
	}

	void *data() { return reinterpret_cast<void*>(m_cap.data()); }
	uint32_t size() { return m_cap.size(); }
private:
	std::vector<uint8_t> m_cap;
};

class PMCapability: public PCICapability
{
public:
	PMCapability():
		PCICapability(PCI_CAP_ID_PM, PCI_PM_SIZEOF)
	{}

	GEN_PCI_CFGSPC_FIELD_FUNCS(PMCapabilities, PCI_PM_PMC, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PMControlStatus, PCI_PM_CTRL, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PMData, PCI_PM_DATA_REGISTER, uint8_t);
};

class PCIExpressCapability: public PCICapability
{
public:
	enum {
		//
		// Make this extra large so that the unused registers are kept
		// hardwired to '0' as according to PCI Express Base
		// specification.
		//
		PCIExpressCapSize = PCI_CAP_EXP_ENDPOINT_SIZEOF_V2 + 0x8,
	};

	PCIExpressCapability():
		PCICapability(PCI_CAP_ID_EXP, PCIExpressCapSize)
	{}

	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIECapabilities, PCI_EXP_FLAGS, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(DeviceCapabilities, PCI_EXP_DEVCAP, uint32_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(DeviceControl, PCI_EXP_DEVCTL, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(DeviceStatus, PCI_EXP_DEVSTA, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(LinkCapabilities, PCI_EXP_LNKCAP, uint32_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(LinkControl, PCI_EXP_LNKCTL, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(LinkStatus, PCI_EXP_LNKSTA, uint16_t);
};

class MSIXCapability: public PCICapability
{
public:
	MSIXCapability():
		PCICapability(PCI_CAP_ID_MSIX, PCI_CAP_MSIX_SIZEOF)
	{}
	GEN_PCI_CFGSPC_FIELD_FUNCS(MessageControl, PCI_MSIX_FLAGS, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(TableOffsetBIR, PCI_MSIX_TABLE, uint32_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PendingBitArray, PCI_MSIX_PBA, uint32_t);

	uint16_t GetTableSize()
	{
		return (GetMessageControl() & PCI_MSIX_FLAGS_QSIZE) + 1;
	}

	int GetTableBIR()
	{
		uint32_t bir = GetTableOffsetBIR() & PCI_MSIX_TABLE_BIR;
		return static_cast<int>(bir);
	}

	uint32_t GetTableOffset()
	{
		return GetTableOffsetBIR() & PCI_MSIX_TABLE_OFFSET;
	}
	uint32_t GetPBABIR()
	{
		return GetTableOffsetBIR() & PCI_MSIX_PBA_BIR;
	}

	uint32_t GetPBAOffset()
	{
		return GetTableOffsetBIR() & PCI_MSIX_PBA_OFFSET;
	}
};

class PCIBARConfig
{
public:
	PCIBARConfig(uint32_t bar, uint64_t sz, uint32_t flags) :
		m_bar(bar), m_sz(sz), m_flags(flags)
	{}
	uint32_t GetBARNum() { return m_bar; }
	uint64_t GetSize() { return m_sz; }
	uint32_t GetFlags() { return m_flags; }
	bool IsBARTypeMem() { return !(m_flags & PCI_BASE_ADDRESS_SPACE_IO); }
	bool Is64BitBAR() { return (m_flags & PCI_BASE_ADDRESS_MEM_TYPE_64); }
private:
	uint32_t m_bar;
	uint64_t m_sz;
	uint32_t m_flags;
};

class PhysFuncConfig
{
public:
	enum {
		PCI_CONFIG_SIZE = 256,
		PCIE_CONFIG_SIZE = 4096,
	};

	PhysFuncConfig() :
		m_pciCapTail(0),
		m_msixCopied(false)
	{
		memset(m_config, 0, sizeof(m_config));
	}

	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIVendorID, PCI_VENDOR_ID, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIDeviceID, PCI_DEVICE_ID, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIRevisionID, PCI_CLASS_REVISION, uint8_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIClassProgIF, PCI_CLASS_PROG, uint8_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIClassDevice, PCI_CLASS_DEVICE, uint8_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIClassBase, PCI_CLASS_BASE, uint8_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIHeaderType, PCI_HEADER_TYPE, uint8_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIBist, PCI_BIST, uint8_t);

	GEN_PCI_CFGSPC_BAR_HELPER_FUNC(0);
	GEN_PCI_CFGSPC_BAR_HELPER_FUNC(1);
	GEN_PCI_CFGSPC_BAR_HELPER_FUNC(2);
	GEN_PCI_CFGSPC_BAR_HELPER_FUNC(3);
	GEN_PCI_CFGSPC_BAR_HELPER_FUNC(4);
	GEN_PCI_CFGSPC_BAR_HELPER_FUNC(5);

	GEN_PCI_CFGSPC_FIELD_FUNCS(PCIInterruptPin, PCI_INTERRUPT_PIN, uint8_t);

	GEN_PCI_CFGSPC_FIELD_FUNCS(PCISubsystemVendorID,
				   PCI_SUBSYSTEM_VENDOR_ID, uint16_t);
	GEN_PCI_CFGSPC_FIELD_FUNCS(PCISubsystemID, PCI_SUBSYSTEM_ID, uint16_t);

	void SetPCIExpansionROMBAR(uint32_t sz, uint32_t flags)
	{
		m_bars.push_back(PCIBARConfig(PCI_EXPROM_BAR, sz, flags));
	}

	GEN_PCI_CFGSPC_FIELD_FUNCS(NextCapPtr, PCI_CAPABILITY_LIST, uint8_t);

	template<uint32_t offset, typename T>
	void Set(T val)
	{
		if (offset <= (PCIE_CONFIG_SIZE-sizeof(T))) {
			T *pos = reinterpret_cast<T*>(&m_config[offset]);
			pos[0] = val;
		}
	}
	template<uint32_t offset, typename T>
	T Get()
	{
		T ret = 0;
		if (offset <= (PCIE_CONFIG_SIZE-sizeof(T))) {
			T *pos = reinterpret_cast<T*>(&m_config[offset]);
			ret = pos[0];
		}
		return ret;
	}

	//
	// Prepend from bottom
	//
	void CopyCapability(void *cap_data, uint32_t cap_size)
	{
		if (m_pciCapTail == 0) {
			m_pciCapTail = PCI_CONFIG_SIZE - cap_size;
			SetNextCapPtr(m_pciCapTail);
			//
			// Hardwire status 'Capabilities List' to 1
			//
			m_config[PCI_STATUS] |= PCI_STATUS_CAP_LIST;
		} else {
			uint32_t nextCapPtr = m_pciCapTail + 1;

			m_pciCapTail -= cap_size;
			m_config[nextCapPtr] = m_pciCapTail;
		}
		memcpy(reinterpret_cast<void*>(&m_config[m_pciCapTail]),
			cap_data, cap_size);
	}
	template<typename T>
	void AddPCICapability(T &cap)
	{
		CopyCapability(cap.data(), cap.size());
	}
	void AddPCICapability(MSIXCapability &cap)
	{
		CopyCapability(cap.data(), cap.size());
		m_msixCap = cap;
		m_msixCopied = true;
	}

	std::vector<PCIBARConfig> &GetBarConfigs() { return m_bars; }

	bool IsBARTypeMSIX(int bar)
	{
		return m_msixCopied && m_msixCap.GetTableBIR() == bar;
	}

	uint16_t GetNumIrqs()
	{
		if (m_msixCopied) {
			return m_msixCap.GetTableSize();
		}
		return 0;
	}

	void *data() { return reinterpret_cast<void*>(m_config); }
	uint32_t size() { return PCIE_CONFIG_SIZE; }
private:
	uint8_t m_config[PCIE_CONFIG_SIZE];
	std::vector<PCIBARConfig> m_bars;

	uint32_t m_pciCapTail;
	uint32_t m_pciExpCapTail;

	MSIXCapability m_msixCap;
	bool m_msixCopied;
};

#undef GEN_PCI_CFGSPC_BAR_HELPER_FUNC
#undef GEN_PCI_CFGSPC_FIELD_FUNCS

#endif
