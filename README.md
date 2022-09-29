<!--
Copyright (c) 2022 Xilinx Inc.
Written by Francisco Iglesias.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-->

## PCI Express model repository

#### Table of Content
   * [1 Overview](#1-overview)
   * [2 The SystemC / TLM 2.0 PCIeController model](#2-the-systemc-/-tlm-2.0-pcieController-model)
     * [2.1 PCIeController model overview](#21-pciecontroller-model-overview)
     * [2.2 PCIeController model configuration](#22-pciecontroller-model-configuration)
     * [2.3 How To Embed Into Your SystemC Project](#23-how-to-embed-into-your-systemc-project)
   * [References](#references)

### 1 Overview

This repository contains a model of a PCI Express controller in a C library and
also a SystemC / TLM 2.0 PCI Express controller model using the library.

### 2 The SystemC / TLM 2.0 PCIeController model

#### 2.1 PCIeController model overview

An overview of the SystemC / TLM 2.0 PCIeController model can be seen in
picture 1.

#### Picture 1 Overview of the [PCIeController](tlm-modules/pcie-controller.h#L40)
```
 PCIE Transaction                        TLM initiator sockets
      Layer           .----------------.      (BAR0-BAR5)
 [TLP packet side]    |                |--------------------->
                      |                |--------------------->
                      |                |--------------------->  [User logic side]
   TLM target socket  | PCIeController |--------------------->
   ------------------>|                |--------------------->
                      |                |--------------------->
                      |                |
 TLM initiator socket |                |  TLM target socket
   <------------------|                | (DMA to the PCIE interface)
                      |                |<---------------------
                      |                |
                      |                | MSI-X interrupts
                      |                | (sc_signal vector)
                      |                |<---------------------
                      '----------------'
```

The two connections depicted on the left side of the PCIeController communicate
through TLM generic payloads containing TLP packets as data and is the PCI
Express Transaction layer interface of the PCIeController (towards a potential
Data link layer). The PCIeController receives TLP packets through the TLM
target socket and transmits TLP packets through its TLM initiator socket. The
data inside a transferred or received TLM generic payload contains one TLP
packet and the data length of the TLM generic payload contains the size in
bytes of the TLP packet.

The right side of PCIeController shows the interface towards user logic. There
are 6 BAR TLM initiator sockets that forward translated PCI Express memory read
and write requests (received on the left side of the picture) as standard TLM
generic payload read and write requests through the corresponding TLM initiator
BAR. There is also a DMA TLM target socket through which the user logic can
perform read and write requests towards the PCI express side, incoming TLM
requests on the DMA target socket will be translated and forwarded as PCI
Express requests on the PCIe Express side (the left side of Picture 1) by the
PCIeController.

User logic can generate MSI-X interrupts by toggling an sc_vector sc_signal on
the PCIeController. The sc_vector index of the toggled sc_signal is the index
of the MSI-X table entry that will be signaled by the PCIeController.

#### 2.2 PCIeController model configuration

The PCIeController currently supports one physical function and is configured
through a PhysFuncConfig which contains the function's configuration space
(device ID, Vendor Id, number of BARs and BAR types, number MSI-X and more).
The configuration is provided to the PCIeController at construction time.

#### 2.3 How To Embed Into Your Project

Please make sure to have the /usr/include/linux/pci_regs.h header (providing
PCI defines) accessable on the build host. On a Ubuntu LTS system the header is
provided by the `linux-libc-dev` package.

To include and use the PCIeController model in your project you can follow the
steps below. This assumes that you have cloned this repository in the root
directory of your project.

See [systemctlm-cosim-demo](https://github.com/Xilinx/systemctlm-cosim-demo)'s [1]
[pcie/versal/cpm-qdma-demo.cc](https://github.com/Xilinx/systemctlm-cosim-demo/blob/master/pcie/versal/cpm-qdma-demo.cc)
for an example project and SystemC application using the PCIeController model.

```
#
# SC_APP Makefile
#
PCIE_MODEL_OBJS = pcie-model/tlm-modules/pcie-controller.o
PCIE_MODEL_OBJS += pcie-model/tlm-modules/libpcie-callbacks.o

CPPFLAGS += -I pcie-model/libpcie/src -I pcie-model/
LDLIBS += libpcie.a

SC_APP_OBJS += $(PCIE_MODEL_OBJS)

# Grab the libpcie.a rule
-include pcie-model/libpcie/libpcie.mk

$(SC_APP): $(SC_APP_OBJS) libpcie.a
	$(CXX) $(LDFLAGS) -o $@ $(SC_APP_OBJS) $(LDLIBS)
```

# References

[1] systemctlm-cosim-demo, [https://github.com/Xilinx/systemctlm-cosim-demo](https://github.com/Xilinx/systemctlm-cosim-demo)
