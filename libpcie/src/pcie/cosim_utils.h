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

#ifndef COSIM_UTILS_H
#define COSIM_UTILS_H

#include "cosim_interface.h"
#include "dpi_platform.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#endif

/*
 * Bitfield macros.
 */
#ifndef BITFIELD_MBN
#define BITFIELD_MBN(_field) \
  (_field ## _LBN + _field ## _WIDTH - 1)
#endif

#ifndef BITFIELD_MASK
#define BITFIELD_MASK(_field)						\
  ((_field ## _WIDTH == 32) ? 0xffffffff : ((1 << _field ## _WIDTH) - 1))
#endif

#ifndef BITFIELD_GET
#define BITFIELD_GET(_dword, _field)					\
  (((_dword) >> _field ## _LBN) & BITFIELD_MASK(_field))
#endif

#ifndef BITFIELD_SET
#define BITFIELD_SET(_dword, _field, _value)				\
  do {									\
    assert(((_value) & ~BITFIELD_MASK(_field)) == 0);			\
    (_dword) = (((_dword) & ~(BITFIELD_MASK(_field) << _field ## _LBN)) | \
	       ((_value) << _field ## _LBN));				\
  } while(0)
#endif

#define BITFIELD_SET_MASK(_dword, _field) \
  BITFIELD_SET(_dword, _field, BITFIELD_MASK(_field))

#define BITFIELD_SET_MASKED(_dword, _field, _value)            \
  BITFIELD_SET(_dword, _field, (_value) & BITFIELD_MASK(_field))

#define _BITFIELD_INDIRECT_GET(_AI, _array, _bpw, _lbn, _mbn, _mask)    \
  ( ( ( _AI(_array, (_lbn) / (_bpw)) >> ((_lbn) % (_bpw)) ) |           \
      ( (_lbn) / (_bpw) != (_mbn) / (_bpw)                              \
        ? ( _AI(_array, (_mbn) / (_bpw)) << ((-(_lbn)) % (_bpw)) )      \
        : 0 ) ) & (_mask) )

/* Get field _field from array _array.  The macro _AI is used to
 * access elements of the array and is called as:
 *   _AI(_array, _index).
 */
#define BITFIELD_INDIRECT_GET(_AI, _array, _field)                      \
  _BITFIELD_INDIRECT_GET(_AI, _array, 8 * sizeof(_AI(_array, 0)),       \
                         _field ## _LBN,                                \
                         BITFIELD_MBN(_field), BITFIELD_MASK(_field))

#define _BITFIELD_INDIRECT_SET(_AI, _array, _bpw, _lbn, _mbn, _mask, _value) \
  do {                                                                  \
    _AI(_array, (_lbn) / (_bpw)) &= ~((_mask)  << ((_lbn) % (_bpw)));   \
    _AI(_array, (_lbn) / (_bpw)) |=  ((_value) << ((_lbn) % (_bpw)));   \
    if ( (_lbn) / (_bpw) != (_mbn) / (_bpw) ) {                         \
      _AI(_array, (_mbn) / (_bpw)) &=                                   \
        ~((_mask)  >> ((-(_lbn)) & ((_bpw)-1)));                        \
      _AI(_array, (_mbn) / (_bpw)) |=                                   \
        ((_value) >> ((-(_lbn)) & ((_bpw)-1)));                         \
    }                                                                   \
  } while(0)


/* Set field _field of array _array to _value.  The macro _AI is used
 * to access elements of the array and is called as:
 *   _AI(_array, _index).
 */
#define BITFIELD_INDIRECT_SET(_AI, _array, _field, _value)              \
  _BITFIELD_INDIRECT_SET(_AI, _array, 8 * sizeof(_AI(_array, 0)),       \
                         _field ## _LBN,                                \
                         BITFIELD_MBN(_field), BITFIELD_MASK(_field),   \
                         (uint32_t)(_value))

#define ARRAY_ELEMENT(_var, _idx) ((_var)[_idx])

#define BITFIELD_ARRAY_GET(_array, _field) \
  BITFIELD_INDIRECT_GET(ARRAY_ELEMENT, _array, _field)

#define BITFIELD_ARRAY_SET(_array, _field, _value)      \
  BITFIELD_INDIRECT_SET(ARRAY_ELEMENT, _array, _field, _value)


#define TLP_GET(N,F) BITFIELD_GET(tlp##N, TLP##N##_##F)

#define TLP_SET(N, F, V) BITFIELD_SET_MASKED(tlp##N, TLP##N##_##F, V)

#ifndef OVM_FATAL_ERR_PRINT
#define OVM_FATAL_ERR_PRINT (-1)
#endif

#ifndef OVM_NONE
#define OVM_NONE   (0)
#endif

#ifndef OVM_LOW
#define OVM_LOW    (1)
#endif

#ifndef OVM_MEDIUM
#define OVM_MEDIUM (2)
#endif

#ifndef OVM_HIGH
#define OVM_HIGH   (3)
#endif

#ifndef OVM_FULL
#define OVM_FULL   (4)
#endif

extern void dbg_print_handler(int level, const char *fmt, ...)
  __attribute__ ((format (printf, 2, 3)));
extern void dbg_error_handler(const char *fmt, ...)
  __attribute__ ((format (printf, 1, 2)));
extern void dbg_quit_handler(uint32_t reason);
extern void dbg_set_verbosity(int level);

#define TRACE(_level, ...) \
  do { dbg_print_handler(_level, __VA_ARGS__); } while(0)

#define ERR(...) \
  do { dbg_error_handler(__VA_ARGS__);\
       dbg_quit_handler(COSIM_EXIT_GENERAL_ERR); } while(0)

#define ERR_NOQUIT(...) dbg_error_handler(__VA_ARGS__)

#define checked_calloc(size_) calloc_or_die(size_, __func__)

void *calloc_or_die(size_t size, const char *caller);

#ifndef NDEBUG

extern void pcie_model_set_Xassert_failure_hook(void (*hook_fn)(void *), void *ctxt);

extern void Xassert_failed(const char *file, unsigned int line, const char *fn,
                           const char *desc) __attribute__((noreturn));

extern void Xassert_expr2_failed(const char *file, unsigned int line,
                                 const char *fn, const char *desc,
                                 unsigned long expr1_val,
                                 unsigned long expr2_val)
                                __attribute__((noreturn));

#define Xassert(e) \
  do {                                                      \
    if (!(e))                                               \
      Xassert_failed(__FILE__, __LINE__, __FUNCTION__, #e); \
  } while(0)

#define Xassert_expr2(e1,test,e2) \
  do {                                                                 \
    if (!((e1) test (e2)))                                             \
      Xassert_expr2_failed(__FILE__, __LINE__, __FUNCTION__,           \
                           #e1 " " #test " " #e2,                      \
                           (unsigned long)(e1), (unsigned long)(e2));  \
  } while(0)

#else

#define Xassert(e)                ((void)0)
#define Xassert_expr2(e1,test,e2) ((void)0)

#endif

#define Xassert_gt(e1,e2) Xassert_expr2(e1,>,e2)
#define Xassert_ge(e1,e2) Xassert_expr2(e1,>=,e2)
#define Xassert_eq(e1,e2) Xassert_expr2(e1,==,e2)
#define Xassert_le(e1,e2) Xassert_expr2(e1,<=,e2)
#define Xassert_lt(e1,e2) Xassert_expr2(e1,<,e2)
#define Xassert_ne(e1,e2) Xassert_expr2(e1,!=,e2)


#endif /* COSIM_UTILS_H */
