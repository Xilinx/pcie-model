#
# Copyright (c) 2022 Xilinx Inc.
# Written by Guido Barzini,
#            Francisco Iglesias
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# libpci
# ======
libpci_sources = [
  'cosim_common.c',
  'cosim_socket.c',
  'cosim_tlp.c',
  'pcie.c',
  'pseudocore.c',
  'platform.c',
  'pcie_cfgspc.c',
  'pcie_cfgutil.c',
  'pcie_msix.c',
]

# This library must be linked as link_whole - it has a constructor!
pci_lib_static = static_library(
  'pci',
  libpci_sources,
  include_directories : [incdir, cpucore_api_incdir],
  c_args : cflags + ['-DWITH_COSIM_TLP_LOG=1', '-D_GNU_SOURCE']
	   + cmc_hwdef_dynamic_header_args,
  dependencies : libpcidep
)
