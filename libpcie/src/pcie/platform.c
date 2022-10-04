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

#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include "pcie/pcie_api.h"
#include "pcie/dpi_platform.h"
#include "pcie/cosim_platform.h"
#include "pcie/cosim_common.h"
#include "pcie/cosim_interface.h"
#include "sfc_cosim_comms.h"
#include "pcie/cosim_utils.h"
#ifndef CONFIG_TLM
  /*
   * The reg_api is used for obtaining the cosim socket address to bind to.
   * Since no cosim socket is setup when running the PCIe model in SystemC /
   * TLM 2.0 simulations (TLP packets are instead directly injected into the
   * PCIe model and also transmitted through SystemC), the reg_api is not
   * required in TLM builds.
   */
  #include "lib/registry.h"
#endif

typedef struct cosim_handler_info {
  cosim_io_handler *read_handler;
  cosim_io_handler *write_handler;
  void *opaque;
} cosim_handler_info;

/* This can be turned off if desired, but the overall difference
 * in performance is small enough that I've failed to measure it
 * and the checks is enables are probably a good idea. */
#define COSIM_MUTEX_DEBUG 1

struct cosim_platform_state_s {
  cosim_socket *dpi_cosim_socket;
  cosim_client *cosim_pcie_client;
  cosim_client *cosim_pci_reset_client;
  void (*post_poll_hook)(void *);
  void *hook_context;
  /* Protects the call to poll() and any state it uses. */
  pthread_mutex_t poll_mutex;
  /* The following variables can only be modified with both poll_mutex
   * and the global cosim mutex held. */
  struct pollfd *poll_fds;
  nfds_t num_fds;
  /* These variables are protected by the global cosim mutex. */
  cosim_handler_info *handler_info;
  int pipe_fds[2];
  pthread_t poll_thread;
  bool finished;
  int instance;
  /* Protects the whole of the state, replacing the old global mutex */
  pthread_mutex_t mutex;
#if COSIM_MUTEX_DEBUG
  /* There is no defined invalid value for pthread_t, so validity is indicated
   * by lock_addr being non-NULL */
  pthread_t locker;
  void *lock_addr;
#endif  
};

static int live_instance_count = 0;

#ifndef CONFIG_TLM
static void pipe_read_handler(void *opaque)
{
  cosim_platform_state_t *state = opaque;
  char buffer[64];
  int rc;

  rc = read(state->pipe_fds[0], buffer, sizeof(buffer));
  if (rc <= 0) {
    if (rc == 0) {
      ERR( "EOF reading from wakeup pipe.\n");
    } else {
      ERR( "Error reading from wakeup pipe: %s\n",
              strerror(errno));
    }
  }
}
#endif

static int poll_thread_wake(cosim_platform_state_t *state)
{
  int rc;
  char c = 'W';
  rc = write(state->pipe_fds[1], &c, sizeof(c));
  if (rc < 0) {
    ERR( "Error writing to wakeup pipe: %s\n",
            strerror(errno));
    return -errno;
  }
  return 0;
}

static int poll_fds_lock(cosim_platform_state_t *state)
{
  int rc;

  if (pthread_mutex_trylock(&state->poll_mutex) != 0) {
    rc = poll_thread_wake(state);
    if (rc < 0)
      return rc;


    rc = pthread_mutex_lock(&state->poll_mutex);
    if (rc < 0) {
      ERR( "Error locking poll mutex: %s\n",
              strerror(rc));
      return -rc;
    }
  }

  return 0;
}

static void poll_fds_unlock(cosim_platform_state_t *state)
{
  int rc;

  rc = pthread_mutex_unlock(&state->poll_mutex);
  if (rc < 0)
    ERR( "Error unlocking poll mutex: %s\n",
            strerror(rc));
}

#ifndef CONFIG_TLM
static void *poll_thread_main(void *arg)
{
  cosim_platform_state_t *state = arg;
  int rc;
  int index;

  while(1) {
    rc = pthread_mutex_lock(&state->poll_mutex);
    if (rc < 0) {
      ERR( "Error locking poll mutex: %s\n",
              strerror(rc));
      return NULL;
    }

    rc = poll(state->poll_fds, state->num_fds, 1);

    rc = pthread_mutex_unlock(&state->poll_mutex);
    if (rc < 0) {
      ERR( "Error unlocking poll mutex: %s\n",
              strerror(rc));
      return NULL;
    }

    rc = cosim_state_lock(state);
    if (rc != 0)
      return NULL;

    for (index = 0; index < state->num_fds; index++) {
      if ((state->poll_fds[index].revents & POLLIN) != 0 &&
          state->handler_info[index].read_handler != NULL) {
        (state->handler_info[index].read_handler)(state->handler_info[index].opaque);
      }
      if ((state->poll_fds[index].revents & POLLOUT) != 0 &&
          state->handler_info[index].write_handler != NULL) {
        (state->handler_info[index].write_handler)(state->handler_info[index].opaque);
      }
    }

    rc = cosim_state_unlock(state);

    if (state->post_poll_hook)
      state->post_poll_hook(state->hook_context);

    if (rc != 0)
      return NULL;
  }
}

static int poll_init(cosim_platform_state_t *state)
{
  int rc;
  int i;

  rc = pipe(state->pipe_fds);
  if (rc < 0) {
    ERR( "Failed to create wakeup pipe: %s\n",
            strerror(errno));
    return -1;
  }

  for(i=0; i<2; i++) {
    rc = fcntl(state->pipe_fds[1], F_SETFL, O_NONBLOCK);
    if (rc < 0) {
      ERR( "Failed to set O_NONBLOCK on pipe[%d]: %s\n",
              i, strerror(errno));
      close(state->pipe_fds[0]);
      close(state->pipe_fds[1]);
      return -1;
    }
  }

  rc = cosim_set_fd_handler(state, state->pipe_fds[0], pipe_read_handler, NULL, state);
  if (rc < 0) {
    ERR( "Failed to set read handler for wakeup pipe.\n");
    close(state->pipe_fds[0]);
    close(state->pipe_fds[1]);
    return -1;
  }

  return 0;
}

static void poll_start(cosim_platform_state_t *state)
{
  int rc;
  rc = pthread_create(&state->poll_thread, NULL, &poll_thread_main, state);
  if (rc < 0) {
    ERR( "Failed to create poll thread.\n");
    free(state->poll_fds);
    free(state->handler_info);
    close(state->pipe_fds[0]);
    close(state->pipe_fds[1]);
  }
}
#endif

int cosim_set_fd_handler(cosim_platform_state_t *state,
                         int fd,
                         cosim_io_handler *read_handler,
                         cosim_io_handler *write_handler,
                         void *opaque)
{
  void *p;
  int index;
  short events;
  int rc;

  rc = poll_fds_lock(state);
  if (rc != 0)
    return rc;

  for (index = 0; index <state->num_fds; index++)
    if (state->poll_fds[index].fd == fd)
      break;

  if (index >= state->num_fds) {
    p = realloc(state->poll_fds, (state->num_fds+1)*sizeof(struct pollfd));
    if (p == NULL) {
      poll_fds_unlock(state);
      return -ENOMEM;
    }
    state->poll_fds = p;

    p = realloc(state->handler_info, (state->num_fds+1)*sizeof(cosim_handler_info));
    if (p == NULL) {
      poll_fds_unlock(state);
      return -ENOMEM;
    }
    state->handler_info = p;

    state->num_fds++;

    state->poll_fds[index].revents = 0;
  }

  state->handler_info[index].read_handler = read_handler;
  state->handler_info[index].write_handler = write_handler;
  state->handler_info[index].opaque = opaque;
  state->poll_fds[index].fd = fd;
  events = 0;
  if (read_handler != NULL)
    events |= POLLIN;
  if (write_handler != NULL)
    events |= POLLOUT;
  state->poll_fds[index].events = events;

  poll_fds_unlock(state);

  return 0;
}


#ifndef CONFIG_TLM
static void dpi_cosim_socket_closed(void *opaque)
{
  ERR_NOQUIT( "Cosim socket closed.  Terminating test.\n");
  cosim_finish(opaque, COSIM_EXIT_CONN_CLOSED);
}
#endif

void cosim_finish(cosim_platform_state_t *state, uint32_t code)
{
  if (!state->finished) {
    enum pcie_exit_decision decision =
      pcie_core_cb_pcie_exit_hook(state, live_instance_count, state->instance);

    printf("%s: QEMU instance %d out of %d total disconnects. Decision is %d\n", __FUNCTION__,
          state->instance, live_instance_count, decision);
    state->finished = true;
    live_instance_count--;
    switch(decision) {
      case PCIE_EXIT_CLEAN:
       cosim_exit(code);
       break;
      case PCIE_EXIT_ABORT:
       raise(SIGABRT);
       break;
      case PCIE_EXIT_CONTINUE:
       break;
      default:
       ERR("Unexpected return from fs_pcie_exit_hook: %d\n", decision);
       break;
    }
  }
}

cosim_client *cosim_get_tlp_client(cosim_platform_state_t *state)
{
  if (state == NULL)
    return NULL;

  return state->cosim_pcie_client;
}

cosim_client *cosim_get_pci_reset_client(cosim_platform_state_t *state)
{
  if (state == NULL)
    return NULL;

  return state->cosim_pci_reset_client;
}

#ifndef CONFIG_TLM
static void dpi_cosim_socket_init(void *context, int instance, cosim_platform_state_t *state,
                                  void(*post_poll_hook)(void*), void *hook_context)
{
  int rc;
  const reg_entry_t *arr;
  const char *address = NULL;
  reg_err_t err = REGISTRY_ERR_OK;

  rc = poll_init(state);
  if (rc != 0)
    return;

  arr = reg_get_arr("cosim.addr", &err);
  if (arr) {
    reg_entry_t *entry = reg_get_arr_entry(arr, instance, &err);
    if (entry)
      address = reg_entry_get_str(entry, &err);
  }

  state->dpi_cosim_socket = cosim_socket_new(state, 0, address, instance,
                                             dpi_cosim_socket_closed, state);
  if (state->dpi_cosim_socket == NULL) {
    ERR("Failed to create cosim socket for instance %d.\n", instance);
    return;
  }

  state->cosim_pcie_client = cosim_client_new(state->dpi_cosim_socket,
                                       COSIM_TYPE_TLP_DOWNSTREAM,
                                       context,
                                       &cosim_pcie_accept_buffer);

  state->post_poll_hook = post_poll_hook;
  state->hook_context = hook_context;

  if (state->cosim_pcie_client == NULL) {
    ERR("Failed to create PCIE cosim client.\n");
    return;
  }

  if(getenv("ENABLE_PCI_RESET") != NULL) {
    state->cosim_pci_reset_client = cosim_client_new(state->dpi_cosim_socket,
                                        COSIM_TYPE_PCI_RESET_REQ,
                                        context,
                                        &cosim_pci_reset_accept_buffer);
    if (state->cosim_pci_reset_client == NULL) {
      ERR("Failed to create PCI reset cosim client.\n");
      return;
    }
  }
  
  poll_start(state);
}

void cosim_dpi_init(void *context, int instance, cosim_platform_state_t **outstate,
                    void(*post_poll_hook)(void*), void *hook_context)
{
  if (getenv("NEED_QEMU") != NULL) {
    cosim_platform_state_t *state = checked_calloc(sizeof(*state));
    state->instance = instance;
    pthread_mutex_init(&state->poll_mutex, NULL);
    pthread_mutex_init(&state->mutex, NULL);
#if COSIM_MUTEX_DEBUG
    state->lock_addr = NULL;
#endif
    dpi_cosim_socket_init(context, instance, state, post_poll_hook, hook_context);
    *outstate = state;
  }
}

#else /* CONFIG_TLM */
void sc_cosim_socket_accept_buffer(void *opaque, cosim_buffer_head *buffer)
{
  uint32_t data[1024 + 4]; // 4 KB data size + 4 DW header (max TLP size)
  struct cosim_request_big *request = cosim_buffer_data(buffer);
  unsigned int tlp_dw_len;
  unsigned int fmt;
  unsigned int pos = 0;
  unsigned int data_pos = 0;

  data[pos++] = request->sm.tlp_header[0];
  data[pos++] = request->sm.tlp_header[1];
  data[pos++] = request->sm.tlp_header[2];

  fmt = data[0] >> 29 & 0x7;

  if (fmt == 1 || fmt == 3) {
    data[pos++] = request->sm.tlp_header[3];
  }

  tlp_dw_len = data[0] & 0x3ff;
  tlp_dw_len = tlp_dw_len ? tlp_dw_len : 1024;
  while (data_pos < tlp_dw_len) {
    data[pos++] = request->payload[data_pos++];
  }

  pcie_core_cb_pcie_tx_tlp(opaque, data, pos);
}

cosim_socket *sc_cosim_socket_new(cosim_platform_state_t *state, int instance);

void sc_pcie_setup(pcie_state_t *pcie_state, int instance,
		   cosim_platform_state_t **outstate)
{
  cosim_platform_state_t *state;
  void *context = pcie_state;

  state = checked_calloc(sizeof(*state));

  assert(state != NULL);

  /* Store in pcie_state_t */
  *outstate = state;

  state->instance = instance;
  pthread_mutex_init(&state->poll_mutex, NULL);
  pthread_mutex_init(&state->mutex, NULL);

  state->dpi_cosim_socket = sc_cosim_socket_new(state, instance);

  state->cosim_pcie_client = cosim_client_new(state->dpi_cosim_socket,
                                       COSIM_TYPE_TLP_DOWNSTREAM,
                                       context,
                                       &cosim_pcie_accept_buffer);

  /* Steal this one for now for handling upstream TLPs */
  state->cosim_pci_reset_client = cosim_client_new(state->dpi_cosim_socket,
                                       COSIM_TYPE_TLP_UPSTREAM,
                                       context,
                                       sc_cosim_socket_accept_buffer);
}

void cosim_dpi_init(void *context, int instance, cosim_platform_state_t **outstate,
                    void(*post_poll_hook)(void*), void *hook_context)
{
  sc_pcie_setup(context, instance, outstate);
}
#endif

void cosim_have_connection(void)
{
  live_instance_count++;
}

void cosim_exit(uint32_t code)
{
  exit(code == COSIM_EXIT_SUCCESS ? 0 : code);
}

/* SIMULATOR ONLY implementation of debugging functions */
static int debug_verbosity = OVM_FULL;

void dbg_set_verbosity(int new_verb) {
 debug_verbosity = new_verb;
}

void dbg_print_handler(int level, const char *fmt, ...) {
  va_list args;
  char *strp;

  if (level <= debug_verbosity) {
    va_start(args, fmt);
    if (vasprintf(&strp, fmt, args) >= 0) {
      printf("%s", strp);
      free(strp);
    }
    va_end(args);
  }
}

void dbg_error_handler(const char *fmt, ...) {
  va_list args;
  char *strp;

  va_start(args, fmt);
  if (vasprintf(&strp, fmt, args) >= 0) {
    printf("ERROR: %s", strp);
    free(strp);
  }
  va_end(args);
}

void dbg_quit_handler(uint32_t reason) {
  //Should quit sim fairly soon.
  printf("%s called with reason 0x%08x\n", __FUNCTION__, reason);
  cosim_exit(reason);
}

void ci_log(const char *fmt, ...) {
  va_list args;
  char *strp;

  va_start(args, fmt);
    if (vasprintf(&strp, fmt, args) >= 0) {
      printf("%s", strp);
      free(strp);
    }
    va_end(args);
}

void __ci_fail(const char *fmt, ...) {
  va_list args;
  char *strp;

  va_start(args, fmt);
    if (vasprintf(&strp, fmt, args) >= 0) {
      printf("%s", strp);
      free(strp);
    }
    va_end(args);
    abort();
}

void eftest_log_lock_claim(void) {};
void eftest_log_lock_release(void) {};

/*** Locking */

int cosim_state_lock(cosim_platform_state_t *state)
{
  int rc;
#if COSIM_MUTEX_DEBUG
  /* There should be no race here: we are only checking whether the current thread
   * holds the mutex, and that thread can't be part way through unlocking */
  if (state->lock_addr != NULL) {
    if (pthread_equal(state->locker, pthread_self()))
      ERR("Tried to lock cosim state instance %d, which we already have locked, from %p\n",
             state->instance, __builtin_return_address(0));
  }
#endif  
  rc = pthread_mutex_lock(&state->mutex);
  if (rc != 0)
    ERR( "Failed to acquire state mutex for instance %d: %s\n", state->instance,
         strerror(rc));
#if COSIM_MUTEX_DEBUG
  state->locker = pthread_self();
  state->lock_addr = __builtin_return_address(0);
#endif
  return rc;
}

int cosim_state_unlock(cosim_platform_state_t *state)
{
  int rc;
#if COSIM_MUTEX_DEBUG
  if (state->lock_addr == NULL)
    ERR("Release state mutex for instance %d, which is not locked, from %p.\n",
           state->instance, __builtin_return_address(0));
  else if (!pthread_equal(state->locker, pthread_self()))
    ERR("Release state mutex for instance %d, which is locked by someone else, from %p.\n",
           state->instance, __builtin_return_address(0));
  state->lock_addr = NULL;
#endif
  rc = pthread_mutex_unlock(&state->mutex);
  if (rc != 0)
    ERR( "Failed to acquire state mutex for instance %d: %s\n", state->instance,
         strerror(rc));
  return rc;
}

void cosim_state_assert_have_lock(cosim_platform_state_t * state)
{
  /* not a recursive mutex, so this is okay */
  int rc = pthread_mutex_trylock(&state->mutex);
  if(rc != EBUSY
#if COSIM_MUTEX_DEBUG
    || state->lock_addr == NULL || !pthread_equal(state->locker, pthread_self())
#endif
  )
    ERR("cosim_socket_assert_locked failed, called from %p for instance %d\n",
           __builtin_return_address(0), state->instance);
}

void *calloc_or_die(size_t size, const char *caller)
{
  void *ptr = calloc(1, size);
  if (ptr == NULL) {
    ERR("calloc call from %s for %lu bytes returned NULL\n", caller, size);
  }
  return ptr;
}

/* Xassert functionality */
static void (*Xassert_hook_fn)(void *) = 0;
static void *Xassert_hook_ctxt;

void pcie_model_set_Xassert_failure_hook(void (*hook_fn)(void *), void *ctxt)
{
  Xassert_hook_fn = hook_fn;
  Xassert_hook_ctxt = ctxt;
}

void Xassert_failed(const char *file, unsigned int line, const char *fn,
                    const char *desc)
{
  fflush(stdout);
  fprintf(stderr, "ASSERTION FAILED at %s:%d (%s)", file, line, fn);
  fprintf(stderr, "\n  %s\n", desc);
  fflush(stderr);
  if (Xassert_hook_fn)
    (*Xassert_hook_fn)(Xassert_hook_ctxt);
  abort();
}

void Xassert_expr2_failed(const char *file, unsigned int line, const char *fn,
                          const char *desc,
                          unsigned long expr1_val, unsigned long expr2_val)
{
  fflush(stdout);
  fprintf(stderr, "ASSERTION FAILED at %s:%d (%s)", file, line, fn);
  fprintf(stderr, "\n  %s\n  (values are 0x%lx, 0x%lx)\n", desc, expr1_val,
          expr2_val);
  fflush(stderr);
  if (Xassert_hook_fn)
    (*Xassert_hook_fn)(Xassert_hook_ctxt);
  abort();
}
