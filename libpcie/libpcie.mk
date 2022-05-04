#
# Copyright (c) 2022 Xilinx Inc.
# Written by Francisco Iglesias
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

CONFIG_TLM ?= y

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
LIBPCIE_DIR := $(dir $(MKFILE_PATH))
LIBPCIE_BUILD ?= libpcie

CFLAGS += -D__STDC_WANT_LIB_EXT2__=1
CFLAGS-$(CONFIG_TLM) += -DCONFIG_TLM=1
CFLAGS += $(CFLAGS-y)

LIBPCIE_SRC += src/pcie/cosim_common.c
LIBPCIE_SRC += src/pcie/cosim_socket.c
LIBPCIE_SRC += src/pcie/cosim_tlp.c
LIBPCIE_SRC += src/pcie/pcie.c
LIBPCIE_SRC += src/pcie/pseudocore.c
LIBPCIE_SRC += src/pcie/platform.c
LIBPCIE_SRC += src/pcie/pcie_cfgspc.c
LIBPCIE_SRC += src/pcie/pcie_cfgutil.c
LIBPCIE_SRC += src/pcie/pcie_msix.c

LIBPCIE_OBJS = $(addprefix $(LIBPCIE_BUILD)/, $(LIBPCIE_SRC:.c=.o))

$(LIBPCIE_BUILD)/%.o: $(LIBPCIE_DIR)/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

libpcie.a: $(LIBPCIE_OBJS)
	ar rcs $@ $^
