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

# How to build and launch the tests

## Download and install SystemC

```
$ tar xzf systemc-2.3.2.tar.gz
$ cd systemc-2.3.2
$ CXXFLAGS=-std=c++11 ./configure
$ make
$ make install
```

## Create a .config.mk in the pcie-model project root dir

```
$ cd /path/to/pcie-model
# Replace /path/to/systemc-2.3.2 to where above was installed
$ cat > .config.mk <<EOF
SYSTEMC = /path/to/systemc-2.3.2
CXXFLAGS += -std=c++11
EOF
```

## Git clone libsystemctlm-soc inside the tests/tlm directory and build

```
$ cd /path/to/pcie-model/tests/tlm
$ git clone https://github.com/Xilinx/libsystemctlm-soc.git
$ make
```

## How to launch the tests

```
$ LD_LIBRARY_PATH=/path/to/systemc-2.3.2/lib-linux64/ ./pcie-ctlr-tg-test
$ LD_LIBRARY_PATH=/path/to/systemc-2.3.2/lib-linux64/ ./pcie-ctlr-rand-tg-test
$ LD_LIBRARY_PATH=/path/to/systemc-2.3.2/lib-linux64/ ./pcie-ctlr-bar64-rand-tg-test
```
