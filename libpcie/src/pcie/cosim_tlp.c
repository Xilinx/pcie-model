/*
 * Copyright (c) 2022 Xilinx Inc.
 * Written by Guido Barzini,
 *            Francisco Iglesias,
 *            Konstantin Ushakov
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

#include "cosim_common.h"
#include "cosim_utils.h"
#include "sfc_cosim_comms.h"
#include <string.h>

#define NUM_TYPES 32
#define NUM_FMTS  4

typedef struct {
  const char *mnem;
  enum fc_type fc_type;
} fmt_decode;

typedef struct {
  const char *type_name;
  fmt_decode fmts[NUM_FMTS];
} type_decode;

type_decode tlp_types[NUM_TYPES];
const char *tlp_fmts[NUM_FMTS];

/*** TLP types ***********************************************************/

#define ADD_TYPE(TYPE) \
  do {\
    assert((TYPE) < NUM_TYPES);\
    tlp_types[TYPE].type_name = #TYPE;\
  } while(0)

#define ADD_FMT(TYPE, FMT, MNEM, FC) \
  do {\
    assert((TYPE) < NUM_TYPES);\
    assert((FMT) < NUM_FMTS);\
    tlp_types[TYPE].fmts[FMT].mnem = MNEM;\
    tlp_types[TYPE].fmts[FMT].fc_type = FC_TYPE_##FC;\
    tlp_fmts[FMT] = #FMT;\
  } while(0)

static void __attribute__((constructor)) setup_decodes(void)
{
  unsigned int type;
  unsigned int fmt;
  const char *unknown = "Unknown";

  memset(tlp_types, 0 ,sizeof(tlp_types));
  for(type=0; type<NUM_TYPES; type++) {
    tlp_types[type].type_name = unknown;
    for(fmt=0; fmt<NUM_FMTS; fmt++)
      tlp_types[type].fmts[fmt].mnem = unknown;
  }
  for(fmt=0; fmt<NUM_FMTS; fmt++)
    tlp_fmts[fmt] = unknown;

  ADD_TYPE( TLP0_TYPE_MEM );
  ADD_FMT( TLP0_TYPE_MEM, TLP0_FMT_3DW_NO_DATA, "MRd", NP);
  ADD_FMT( TLP0_TYPE_MEM, TLP0_FMT_4DW_NO_DATA, "MRd", NP);
  ADD_FMT( TLP0_TYPE_MEM, TLP0_FMT_3DW_DATA, "MWr", P);
  ADD_FMT( TLP0_TYPE_MEM, TLP0_FMT_4DW_DATA, "MWr", P);
  ADD_TYPE( TLP0_TYPE_MEM_LOCKED );
  ADD_FMT( TLP0_TYPE_MEM_LOCKED, TLP0_FMT_3DW_NO_DATA, "MRdLk", NP);
  ADD_FMT( TLP0_TYPE_MEM_LOCKED, TLP0_FMT_4DW_NO_DATA, "MRdLk", NP);
  ADD_TYPE( TLP0_TYPE_IO );
  ADD_FMT( TLP0_TYPE_IO, TLP0_FMT_3DW_NO_DATA, "IORd", NP);
  ADD_FMT( TLP0_TYPE_IO, TLP0_FMT_3DW_DATA, "IOWr", NP);
  ADD_TYPE( TLP0_TYPE_CFG0 );
  ADD_FMT( TLP0_TYPE_CFG0, TLP0_FMT_3DW_NO_DATA, "CfgRd0", NP);
  ADD_FMT( TLP0_TYPE_CFG0, TLP0_FMT_3DW_DATA, "CfgWr0", NP);
  ADD_TYPE( TLP0_TYPE_CFG1 );
  ADD_FMT( TLP0_TYPE_CFG1, TLP0_FMT_3DW_NO_DATA, "CfgRd1", NP);
  ADD_FMT( TLP0_TYPE_CFG1, TLP0_FMT_3DW_DATA, "CfgWr1", NP);
  ADD_TYPE( TLP0_TYPE_MSG(0) );
  ADD_FMT( TLP0_TYPE_MSG(0), TLP0_FMT_4DW_NO_DATA, "Msg to RC", P);
  ADD_FMT( TLP0_TYPE_MSG(0), TLP0_FMT_4DW_DATA, "MsgD to RC", P);
  ADD_TYPE( TLP0_TYPE_MSG(1) );
  ADD_FMT( TLP0_TYPE_MSG(1), TLP0_FMT_4DW_NO_DATA, "Msg to address", P);
  ADD_FMT( TLP0_TYPE_MSG(1), TLP0_FMT_4DW_DATA, "MsgD to address", P);
  ADD_TYPE( TLP0_TYPE_MSG(2) );
  ADD_FMT( TLP0_TYPE_MSG(2), TLP0_FMT_4DW_NO_DATA, "Msg to ID", P);
  ADD_FMT( TLP0_TYPE_MSG(2), TLP0_FMT_4DW_DATA, "MsgD to ID", P);
  ADD_TYPE( TLP0_TYPE_MSG(3) );
  ADD_FMT( TLP0_TYPE_MSG(3), TLP0_FMT_4DW_NO_DATA, "Broadcast Msg", P);
  ADD_FMT( TLP0_TYPE_MSG(3), TLP0_FMT_4DW_DATA, "Broadcast MsgD", P);
  ADD_TYPE( TLP0_TYPE_MSG(4) );
  ADD_FMT( TLP0_TYPE_MSG(4), TLP0_FMT_4DW_NO_DATA, "Local Msg", P);
  ADD_FMT( TLP0_TYPE_MSG(4), TLP0_FMT_4DW_DATA, "Local MsgD", P);
  ADD_TYPE( TLP0_TYPE_MSG(5) );
  ADD_FMT( TLP0_TYPE_MSG(5), TLP0_FMT_4DW_NO_DATA, "Gather Msg", P);
  ADD_FMT( TLP0_TYPE_MSG(5), TLP0_FMT_4DW_DATA, "Gather MsgD", P);
  ADD_TYPE( TLP0_TYPE_CPL );
  ADD_FMT( TLP0_TYPE_CPL, TLP0_FMT_3DW_NO_DATA, "Cpl", CPL);
  ADD_FMT( TLP0_TYPE_CPL, TLP0_FMT_3DW_DATA, "CplD", CPL);
  ADD_TYPE( TLP0_TYPE_CPL_LOCKED );
  ADD_FMT( TLP0_TYPE_CPL_LOCKED, TLP0_FMT_3DW_NO_DATA, "CplLk", CPL);
  ADD_FMT( TLP0_TYPE_CPL_LOCKED, TLP0_FMT_3DW_DATA, "CplLkD", CPL);
}


void tlp_print_fmt(char *typestr, char *fmtstr, unsigned typeval, unsigned fmtval) {
  const char *lcl_type_name = NULL;
  const char *lcl_fmt_name = NULL;
  const char *lcl_mnem = NULL;

  lcl_type_name = tlp_types[typeval].type_name;
  lcl_fmt_name =  tlp_fmts[fmtval];
  lcl_mnem = tlp_types[typeval].fmts[fmtval].mnem;

  TRACE(OVM_MEDIUM, "%-20s = 0x%x (%s)\n%-20s = 0x%x (%s)\n%-20s = %s\n",
	    typestr, typeval, lcl_type_name,
	    fmtstr, fmtval, lcl_fmt_name,
	    "decoded type", lcl_mnem);
}

enum fc_type tlp_get_fc_type(unsigned int fmt, unsigned int type)
{
  enum fc_type fc_type;

  if (type >= NUM_TYPES) {
    ERR("Unknown TLP type %d\n", type);
    return FC_TYPE_NP;
  }
  if (fmt >= NUM_FMTS) {
    ERR("Unknown TLP format %d\n", fmt);
    return FC_TYPE_NP;
  }
  fc_type = tlp_types[type].fmts[fmt].fc_type;

  if (fc_type != FC_TYPE_NP &&
      fc_type != FC_TYPE_P &&
      fc_type != FC_TYPE_CPL) {
    ERR("Unknown FC type: fmt=%d (%s) type=%d (%s)\n",
        fmt, tlp_fmts[fmt],
               type, tlp_types[type].type_name);
    return FC_TYPE_NP;
  }

  return fc_type;
}
