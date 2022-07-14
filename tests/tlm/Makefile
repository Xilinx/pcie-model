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

-include ../../.config.mk
include libsystemctlm-soc/tests/Rules.mk

CPPFLAGS += -I libsystemctlm-soc -I libsystemctlm-soc/tests
CPPFLAGS += -I ../../libpcie/src -I ../../ -I .
CXXFLAGS += -g
CFLAGS += -g

OBJS_COMMON += test-modules/memory.o

OBJS_COMMON += tlm-modules/pcie-controller.o
OBJS_COMMON += tlm-modules/libpcie-callbacks.o

PCIE_CTRL_TG_TEST_SRC += pcie-ctlr-tg-test.cc
PCIE_CTRL_TG_TEST_OBJS = $(PCIE_CTRL_TG_TEST_SRC:.cc=.o)

PCIE_CTRL_RAND_TG_TEST_SRC += pcie-ctlr-rand-tg-test.cc
PCIE_CTRL_RAND_TG_TEST_OBJS = $(PCIE_CTRL_RAND_TG_TEST_SRC:.cc=.o)

ALL_OBJS += $(OBJS_COMMON)
ALL_OBJS += $(PCIE_CTRL_TG_TEST_OBJS)
ALL_OBJS += $(PCIE_CTRL_RAND_TG_TEST_OBJS)

TARGETS += pcie-ctlr-tg-test
TARGETS += pcie-ctlr-bar64-rand-tg-test
TARGETS += pcie-ctlr-rand-tg-test

################################################################################

all: $(TARGETS)

## Dep generation ##
-include $(ALL_OBJS:.o=.d)
-include $(wildcard *-test.d)

## libpcie ##
include ../../libpcie/libpcie.mk

c-objs/%.o: ../../tlm-modules/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

tlm-modules/%.o:  ../../tlm-modules/%.cc
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

test-modules/%.o: libsystemctlm-soc/tests/test-modules/%.cc
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

pcie-ctlr-tg-test.o: pcie-ctlr-tg-test.cc
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

pcie-ctlr-rand-tg-test.o: pcie-ctlr-rand-tg-test.cc
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

pcie-ctrl-bar64-rand-tg-test.o: pcie-ctlr-rand-tg-test.cc
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

pcie-ctlr-tg-test: LDLIBS += libpcie.a
pcie-ctlr-tg-test: $(PCIE_CTRL_TG_TEST_OBJS) libpcie.a $(OBJS_COMMON)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

pcie-ctlr-rand-tg-test: LDLIBS += libpcie.a
pcie-ctlr-rand-tg-test: $(PCIE_CTRL_RAND_TG_TEST_OBJS) libpcie.a $(OBJS_COMMON)
	$(CXX) $(LDFLAGS) -o $@ $^  $(LDLIBS)

pcie-ctlr-bar64-rand-tg-test: CXXFLAGS += -DCONFIG_64BIT_BAR
pcie-ctlr-bar64-rand-tg-test: LDLIBS += libpcie.a
pcie-ctlr-bar64-rand-tg-test: pcie-ctrl-bar64-rand-tg-test.o libpcie.a $(OBJS_COMMON)
	$(CXX) $(LDFLAGS) -o $@ $^  $(LDLIBS)

clean:
	$(RM) $(ALL_OBJS) $(ALL_OBJS:.o=.d)
	$(RM) $(LIBPCIE_OBJS) $(LIBPCIE_OBJS:.o=.d)
	$(RM) $(wildcard *-test.o) $(wildcard *-test.d)
	$(RM) $(wildcard *-test) $(wildcard *-test.vcd)
	$(RM) -rf $(LIBPCIE_BUILD) libpcie.a
