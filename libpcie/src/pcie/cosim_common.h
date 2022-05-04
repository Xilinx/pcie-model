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

#ifndef COSIM_COMMON_H
#define COSIM_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

typedef struct cosim_platform_state_s cosim_platform_state_t;
typedef union cosim_buffer_head cosim_buffer_head;
typedef struct cosim_client cosim_client;
typedef struct cosim_socket cosim_socket;

/* Lock the cosim state mutex. */
extern int cosim_state_lock(cosim_platform_state_t *state);

/* Unlock the cosim state mutex. */
extern int cosim_state_unlock(cosim_platform_state_t *state);

/* Assert that cosim state mutex is locked (and if debug is enabled
 * check that we own it) */
extern void cosim_state_assert_have_lock(cosim_platform_state_t *state);

/* Different buffers can be associated with different functions to
 * free the buffer.  This is so that some can be static or dynamic
 * according to usage. */
typedef void cosim_buffer_free_fn(cosim_buffer_head *buffer);

union cosim_buffer_head {
  struct {
    /* The size of the data area.  Must not be modified. */
    uint16_t size;

    /* The type of the payload. */
    uint16_t type;

    /* The buffer creator can put buffers into lists.  Recipients of
     * the buffer generally cannot use this field since the buffer
     * might be shared. */
    cosim_buffer_head *next;

    /* A function to call to free the buffer. */
    cosim_buffer_free_fn *free_fn;

    /* The number of references to this buffer. */
    int32_t ref_count;
  } /* anonymous */;

  /* This is to make sure the buffer data area is aligned. */
  uint64_t dummy_u64;
};

/* The data area follows the buffer head. */
static inline void *cosim_buffer_data(cosim_buffer_head *buffer)
{
  assert(buffer != NULL);
  return buffer+1;
}

/* Allocate a dynamic buffer. */
extern cosim_buffer_head *cosim_buffer_alloc(uint16_t size, uint16_t type);

static inline void cosim_buffer_free(cosim_buffer_head *buffer)
{
  assert(buffer != NULL);
  assert(buffer->ref_count >= 1);
  buffer->ref_count--;
  if (buffer->ref_count == 0)
    buffer->free_fn(buffer);
}

static inline void cosim_buffer_add_reference(cosim_buffer_head *buffer)
{
  assert(buffer != NULL);
  buffer->ref_count++;
}

/* A function prototype used to pass a buffer to a client. */
typedef void cosim_accept_buffer_fn(void *opaque,
                                    cosim_buffer_head *buffer);

/* Create a new client. */
extern cosim_client *cosim_client_new(cosim_socket *socket,
                                      uint16_t consumer_type,
                                      void *opaque,
                                      cosim_accept_buffer_fn *accept_buffer);

/* Destroy a client. */
extern void cosim_client_delete(cosim_client *client);

/* Used by a client to send a buffer. */
extern void cosim_buffer_send(cosim_client *client,
                              cosim_buffer_head *buffer);

typedef void cosim_socket_close_fn(void *);

extern cosim_socket *cosim_socket_new(cosim_platform_state_t *state,
                                      int is_listening,
                                      const char *address, int instance,
                                      cosim_socket_close_fn *close_fn,
                                      void *opaque);

extern void cosim_socket_delete(cosim_socket *socket);

extern int cosim_socket_connected(cosim_socket *socket);

extern void cosim_socket_set_tracing(struct cosim_socket *socket, int val);

void tlp_print_fmt(char *typestr, char *fmtstr, unsigned typeval, unsigned fmtval);

enum fc_type {
  FC_TYPE_NONE,
  FC_TYPE_NP,
  FC_TYPE_P,
  FC_TYPE_CPL
};

enum fc_type tlp_get_fc_type(unsigned int fmt, unsigned int type);

void cosim_dump_buffer_hdr(FILE *f);
void cosim_dump_buffer(FILE *f, unsigned long long timestamp, cosim_buffer_head *buffer);

#endif /* COSIM_COMMON_H */
