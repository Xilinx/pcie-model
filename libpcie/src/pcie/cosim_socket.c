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
#include "cosim_platform.h"
#include "cosim_internal.h"
#include "sfc_cosim_comms.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <netdb.h>

/* There are tradeoffs associated with this value.  Setting it lower
 * will cause more recv system calls.  Setting it higher will cause
 * more bytes to be copied into the payload buffer.  A higher value
 * will also consume more memory but that's unlikely to be an
 * issue. */
#define RECV_BUFFER_SIZE 128

/* Bug78540: Make several connect retries (total wait time of 250s) */
#define COSIM_CONNECT_RETRIES 1000
/* Bug78540: Retry to connect every 0.25s */
#define COSIM_CONNECT_RETRY_INTERVAL_US 250000UL

#define COSIM_SOCK_ERR (-1)
#define COSIM_SOCK_UNUSED (-2)

struct cosim_socket {
  int fd;
  cosim_hub *hub;
  cosim_socket *next;
  cosim_socket *master;
  union {
    uint8_t header_buffer[RECV_BUFFER_SIZE];
    uint32_t header_buffer_32[RECV_BUFFER_SIZE/4];
  };
  cosim_buffer_head *payload_buffer;
  int buffer_offset;
  cosim_client *(clients[COSIM_NUM_TYPES]);
  uint32_t subscriptions;
  cosim_socket_close_fn *close_fn;
  void *opaque;
  int connected;
  unsigned int rx_sequence;
  unsigned int tx_sequence;
  cosim_platform_state_t *state;
};

static void cosim_socket_accept_buffer(void *opaque,
                                       cosim_buffer_head *buffer);

/*** Socket tracing ***********************************************/

static void cs_trace(const struct cosim_socket *socket, const char* fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  ch_vtrace(socket->hub, fmt, args);
  va_end(args);
}

void cosim_socket_set_tracing(struct cosim_socket *socket, int val)
{
  cosim_hub_set_tracing(socket->hub, val);
}

/*** Socket handling *****************************************************/

cosim_hub *cosim_socket_get_hub(cosim_socket *socket)
{
  return socket->hub;
}

static void cosim_handle_subscription(cosim_socket *socket,
                                      cosim_buffer_head *buffer)
{
  struct cosim_subscribe_request *request;
  cosim_client *client;
  uint16_t type;

  if (buffer->size != sizeof(*request)) {
    ERR( "Invalid SUBSCRIBE_REQUEST size: %d.\n",
         buffer->size);
    return;
  }

  request = cosim_buffer_data(buffer);
  type = request->subscription_type;

  cs_trace(socket, "Incoming subscription for type %d\n", type);

  if (type >= COSIM_NUM_TYPES) {
    ERR( "Invalid SUBSCRIBE_REQUEST type: %d.\n",
         type);
    return;
  }

  if (socket->clients[type] != NULL) {
    ERR( "Duplicate SUBSCRIBE_REQUEST type: %d.\n",
         type);
    return;
  }

  /* We need to mark the subscription as valid before calling
   * cosim_client_new so that we can filter it out when the hub calls
   * us back. */
  assert(COSIM_NUM_TYPES <= 8*sizeof(socket->subscriptions));
  socket->subscriptions |= 1<<type;

  client = cosim_client_new(socket, type, socket,
                            cosim_socket_accept_buffer);
  if (client == NULL) {
    ERR( "Failed to allocate client for subscription %d\n",
         type);
    return;
  }
  socket->clients[type] = client;
  
  return;
}

static void cosim_dispatch_payload(cosim_socket *socket)
{
  cosim_buffer_head *buffer;
  uint16_t type;

  assert(socket != NULL);

  buffer  = socket->payload_buffer;
  socket->payload_buffer = NULL;

  type = buffer->type;

  if (type == COSIM_TYPE_SUBSCRIBE_REQUEST) {
    cosim_handle_subscription(socket, buffer);
    cosim_buffer_free(buffer);
    return;
  }

  cs_trace(socket, "Dispatching packet id %d type %d length %d\n",
           socket->rx_sequence, buffer->type, buffer->size);
  socket->rx_sequence++;

  cosim_hub_dispatch_packet(socket->hub, buffer);
}

static int cosim_socket_do_recv(cosim_socket *socket, int buffer_size)
{
  cosim_buffer_head *buffer_head;
  struct iovec msg_iov[2];
  struct msghdr msg_hdr = {
    .msg_iov = msg_iov,
  };
  uint8_t *data;
  unsigned length;
  int rc;

  assert(socket != NULL);

  if (socket->payload_buffer) {
    /* We need a payload.  Read this payload and the next set of
     * headers in a single system call. */
    buffer_head = socket->payload_buffer;
    data = cosim_buffer_data(buffer_head);
    msg_iov[0].iov_base = data + socket->buffer_offset;
    msg_iov[0].iov_len = buffer_head->size - socket->buffer_offset;
    msg_iov[1].iov_base = socket->header_buffer;
    msg_iov[1].iov_len = buffer_size;
    msg_hdr.msg_iovlen = 2;
    cs_trace( socket, "Receiving payload %d/%d and headers 0/%d\n",
           socket->buffer_offset, buffer_head->size,
           buffer_size);
  } else {
    msg_iov[0].iov_base = socket->header_buffer + socket->buffer_offset;
    msg_iov[0].iov_len = buffer_size - socket->buffer_offset;
    msg_hdr.msg_iovlen = 1;
    cs_trace(socket, "Receiving headers %d/%d\n",
           socket->buffer_offset, buffer_size);
  }

  /* Receive bytes from the network. */
  rc = recvmsg(socket->fd, &msg_hdr, MSG_DONTWAIT);
  if (rc <= 0) {
    if (rc == 0) {
      TRACE(OVM_LOW, "EOF reading from socket %d\n", socket->fd);
    } else {
      ERR( "Error reading from socket %d: %s\n", socket->fd,
              strerror(errno));
    }
    cosim_set_fd_handler(socket->state, socket->fd, NULL, NULL, socket);
    socket->master->connected--;
    if (socket->master->close_fn != NULL)
      socket->master->close_fn(socket->master->opaque);
    return 0;
  }
 cs_trace(socket, "Received %d bytes.\n", rc);

#if 0
 {
   int i;
   int j;
   int bytes = rc;

   for(i=0; i<2 && bytes > 0; i++) {
     uint8_t *buffer = msg_iov[i].iov_base;
     for(j=0; j<msg_iov[i].iov_len && bytes > 0; j++, bytes--) {
       cs_trace(socket, "Byte 0x%02x\n", buffer[j]);
     }
   }
 }
#endif

  length = socket->buffer_offset + rc;

  /* Dispatch a payload buffer if possible. */
  if (socket->payload_buffer) {
    if (rc < msg_iov[0].iov_len) {
      /* Payload is incomplete. */
      socket->buffer_offset = length;
      cs_trace(socket, "Payload progress: %d/%d\n", length,
             socket->payload_buffer->size);
      return 0;
    }

    cosim_dispatch_payload(socket);
    length = rc - msg_iov[0].iov_len;
  }

  cs_trace(socket, "Returning with %d bytes in receive buffer.\n",
         length);

  return length;
}

static void cosim_socket_process_buffer(cosim_socket *socket,
                                        unsigned length)
{
  cosim_buffer_head *buffer_head;
  struct cosim_packet_header packet_header;
  uint8_t *data;
  unsigned offset = 0;

  assert(socket != NULL);

  while (length >= sizeof(struct cosim_packet_header)) {
    /* Consume the packet header. */
    memcpy(&packet_header, socket->header_buffer + offset,
           sizeof(packet_header));
    offset += sizeof(struct cosim_packet_header);
    length -= sizeof(struct cosim_packet_header);

    cs_trace(socket, "Got packet type %d, length %d\n",
           packet_header.type,
           packet_header.length);

    /* Allocate a buffer for the payload. */
    buffer_head = cosim_buffer_alloc(packet_header.length,
                                     packet_header.type);
    if (buffer_head == NULL) {
      ERR( "Failed to allocate packet buffer, size %d.\n",
              packet_header.length);
      cosim_set_fd_handler(socket->state, socket->fd, NULL, NULL, socket);
      return;
    }

    socket->payload_buffer = buffer_head;
    data = cosim_buffer_data(buffer_head);

    /* If we need to receive more data, copy the initial portion and
     * poll again. */
    if (length < packet_header.length) {
      cs_trace(socket, "Partial payload available: %d/%d\n",
             length, packet_header.length);
      memcpy(data, socket->header_buffer + offset, length);
      socket->buffer_offset = length;
      return;
    }

    /* If the whole packet has arrived, copy the whole thing and
     * dispatch it.  Then go round the loop again for more packets. */
    memcpy(data, socket->header_buffer + offset, packet_header.length);
    cosim_dispatch_payload(socket);
    length -= packet_header.length;
    offset += packet_header.length;
    cs_trace(socket, "Have %d bytes remaining in receive buffer.\n", length);
  }

  /* Move the received data to the start of the buffer. */
  memmove(socket->header_buffer, socket->header_buffer + offset, length);
  socket->buffer_offset = length;
}

static void cosim_socket_read_ready(void *opaque)
{
  cosim_socket *socket = opaque;
  unsigned length;

  assert(socket != NULL);

  /* Attempt to fill the receive buffer. */
  length = cosim_socket_do_recv(socket, RECV_BUFFER_SIZE);

  /* Empty the receive buffer.  This doesn't apply if there is a
   * payload buffer.  In that case, buffer_offset has been filled in
   * already and we just need to wait for more data to arrive. */
  if (socket->payload_buffer == NULL)
    cosim_socket_process_buffer(socket, length);
}

static void cosim_socket_read_version(void *opaque)
{
  cosim_socket *socket = opaque;
  uint32_t version;
  unsigned length;

  assert(socket != NULL);

  /* Attempt to read the version into the receive buffer. */
  length = cosim_socket_do_recv(socket, sizeof(uint32_t));

  if (length < sizeof(int))
    return;

  version = *(uint32_t*)(socket->header_buffer_32);
  if (version != SFC_COSIM_COMMS_VER) {
    ERR( "COSIM version mismatch.\n"
            "This end supports version %d\n"
            "The other end supports version %d\n",
            SFC_COSIM_COMMS_VER,
            version);
    cosim_set_fd_handler(socket->state, socket->fd, NULL, NULL, socket);
  } else {
    cosim_set_fd_handler(socket->state, socket->fd, cosim_socket_read_ready, NULL,
                         socket);
  }

  printf("Version matched.\n");

  /* Remove the version number from the input buffer. */
  socket->buffer_offset = 0;
}

static int cosim_socket_send_version(cosim_socket *socket)
{
  static const uint32_t version = SFC_COSIM_COMMS_VER;
  int rc;

  assert(socket != NULL);

  rc = send(socket->fd, &version, sizeof(version), 0);
  if (rc < 0) {
    ERR( "Failed to send version number: %s\n",
            strerror(errno));
    return rc;
  }

 printf("Sent version.\n");

  rc = cosim_set_fd_handler(socket->state, socket->fd, cosim_socket_read_version,
                            NULL, socket);
  if (rc != 0) {
    ERR( "Failed to set FD handlers (rc=%d)\n", rc);
    return rc;
  }

  return 0;
}

static void cosim_socket_accept_buffer(void *opaque,
                                       cosim_buffer_head *buffer)
{
  cosim_socket *socket = opaque;
  struct cosim_packet_header packet_header;
  struct iovec msg_iov[2];
  struct msghdr msg_hdr = {
    .msg_iov = msg_iov,
    .msg_iovlen = ARRAY_SIZE(msg_iov),
  };
  int rc;

  assert(socket != NULL);
  assert(buffer != NULL);

  packet_header.type = buffer->type;
  packet_header.length = buffer->size;

  msg_iov[0].iov_base = &packet_header;
  msg_iov[0].iov_len = sizeof(packet_header);
  msg_iov[1].iov_base = cosim_buffer_data(buffer);
  msg_iov[1].iov_len = buffer->size;

  /* FIXME: This can deadlock. */
  cs_trace(socket, "Sending packet id %d type %d length %d\n",
           socket->tx_sequence, buffer->type, buffer->size);
  socket->tx_sequence++;

  rc = sendmsg(socket->fd, &msg_hdr, 0);

  if (rc < msg_iov[0].iov_len + msg_iov[1].iov_len) {
    if (rc > 0) {
      ERR( "Short write to socket %d\n", socket->fd);
    } else if (rc == 0) {
      ERR( "EOF writing to socket %d\n", socket->fd);
    } else {
      ERR( "Error writing to socket %d: %s\n", socket->fd,
              strerror(errno));
    }
    cosim_set_fd_handler(socket->state, socket->fd, NULL, NULL, socket);
    socket->master->connected--;
    if (socket->master->close_fn != NULL)
      socket->master->close_fn(socket->master->opaque);
  }
}

static void
cosim_socket_accept_subscription_buffer(void *opaque,
                                        cosim_buffer_head *buffer)
{
  cosim_socket *socket = opaque;
  struct cosim_subscribe_request *request;
  uint16_t type;

  if (buffer->size != sizeof(*request)) {
    ERR( "Invalid SUBSCRIBE_REQUEST size: %d.\n",
         buffer->size);
    return;
  }

  request = cosim_buffer_data(buffer);
  type = request->subscription_type;

  cs_trace(socket, "Outgoing subscription for type %d\n", type);

  if (type >= COSIM_NUM_TYPES) {
    ERR( "Invalid SUBSCRIBE_REQUEST type: %d.\n",
         type);
    return;
  }

  /* Ignore subscriptions which were generated by this socket. */
  if (socket->subscriptions & (1<<type)) {
    cs_trace(socket, "Ignoring subscription\n");
    return;
  }

  cosim_socket_accept_buffer(opaque, buffer);
}

static void cosim_socket_fini(cosim_socket *socket)
{
  int fd;
  int i;

  assert(socket != NULL);

  for(i=0; i<COSIM_NUM_TYPES; i++) {
    cosim_client *client = socket->clients[i];
    if (client != NULL)
      cosim_client_delete(client);
  }

  fd = socket->fd;
  if (fd != -1) {
    cosim_set_fd_handler(socket->state, fd, NULL, NULL, NULL);
    close(fd);
  }
}

static void cosim_socket_init(cosim_socket *socket, cosim_hub *hub,
                              int fd, cosim_socket_close_fn close_fn,
                              void *opaque)
{
  int type;

  assert(socket != NULL);

  socket->fd = fd;
  socket->hub = hub;
  socket->next = NULL;
  socket->payload_buffer = NULL;
  socket->buffer_offset = 0;
  socket->close_fn = close_fn;
  socket->opaque = opaque;

  for(type=0; type<COSIM_NUM_TYPES; type++)
    socket->clients[type] = NULL;
}

static int cosim_socket_add_clients(cosim_socket *socket)
{
  int type;

  type = COSIM_TYPE_SUBSCRIBE_REQUEST;
  cosim_client *client = cosim_client_new(socket, type, socket,
                                          cosim_socket_accept_subscription_buffer);
  if (client == NULL)
    return -ENOMEM;
  socket->clients[type] = client;

  return 0;
}

static void cosim_socket_accept(void *opaque)
{
  cosim_socket *master = opaque;
  cosim_socket *slave;
  union {
    struct sockaddr     sa;
    struct sockaddr_in  sin;
    struct sockaddr_in6 sin6;
  } u;
  socklen_t sock_len = sizeof(u);
  int fd;
  int rc;

  assert(master != NULL);

  fd = accept(master->fd, &u.sa, &sock_len);
  if (fd < 0) {
    ERR( "Error from accept: %s\n", strerror(errno));
    return;
  }
  
  slave = malloc(sizeof(cosim_socket));
  if (slave == NULL) {
    ERR( "Failed to allocate slave socket.\n");
    close(fd);
  }

  cosim_socket_init(slave, master->hub, fd, NULL, NULL);

  rc = cosim_socket_send_version(slave);
  if (rc != 0) {
    cosim_socket_fini(slave);
    free(slave);
    return;
  }

  rc = cosim_socket_add_clients(slave);
  if (rc != 0) {
    ERR( "Failed to add socket clients.\n");
    cosim_socket_fini(slave);
    free(slave);
    return;
  }
    
  slave->next = master->next;
  slave->master = master;
  master->next = slave;
  master->connected++;
}

static struct addrinfo *cosim_get_addrinfo(int is_listening,
                                           const char *address,
                                           int instance
                                          )
{
  struct addrinfo ai_hints = {
    .ai_family = AF_INET,
    .ai_socktype = SOCK_STREAM,
    .ai_protocol = 0,
    .ai_flags = 0,
  };
  struct addrinfo *addrinfo;
  char *allocated_name = NULL;
  const char *hostname = NULL;
  const char *port = NULL;
  const char *colon;
  int len;
  int rc;

  if (is_listening)
    ai_hints.ai_flags |= AI_PASSIVE;

  /* Try the old style for instance 0 */
  if (address == NULL && instance == 0)
    address = getenv("COSIM_ADDRESS");

  if (address == NULL) {
    char buffer[16];
    sprintf(buffer, "COSIM_ADDRESS%d", instance);
    address = getenv(buffer);
  }

  /* Parse the address if specified. Give up if not */
  if (address == NULL) {
    TRACE(OVM_NONE, "WARN: Cosim socket instance %d will not be set up as no address provided.\n",
               instance);
    return NULL;
  }

  colon = strrchr(address, ':');
  if (colon) {
    /* A name with a colon is of the form <host>:<port>. */
    len = colon - address;
    if (len != 0) {
      /* If the host part is not empty then use it. */
      allocated_name = strndup(address, len);
      if (allocated_name == NULL)
        return NULL;
      hostname = allocated_name;
    }
    /* The port part always comes after the colon. */
    port = colon + 1;
  } else {
    /* Specifying the port is no longer optional */
    ERR("Cosim socket instance %d cannot be set up as address provided, '%s' does"
        " not specify the port and there is no longer a default.\n", instance, address); 
    return NULL;
  }

  /* At this point, hostname may be NULL but port is not NULL. */
  rc = getaddrinfo(hostname, port, &ai_hints, &addrinfo);
  if (allocated_name)
    free(allocated_name);

  if (rc != 0) {
    ERR( "Failed to resolve socket address(%s : %s): %s\n",
            hostname, port, gai_strerror(rc));
    return NULL;
  }

  return addrinfo;
}

static int cosim_socket_bind(int fd, struct addrinfo *addrinfo)
{
  int rc;
  int opt;

  assert(addrinfo != NULL);

  opt = 1;
  rc = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  if (rc < 0) {
    ERR( "Failed to set SO_REUSEADDR socket: %s\n",
            strerror(errno));
    return rc;
  }

  rc = bind(fd, addrinfo->ai_addr, addrinfo->ai_addrlen);
  if (rc < 0) {
    ERR( "Failed to bind socket: %s\n",
            strerror(errno));
    freeaddrinfo(addrinfo);
    return rc;
  }

  rc = listen(fd, 5);
  if (rc < 0) {
    ERR( "Failed to listen on socket: %s\n",
            strerror(errno));
    freeaddrinfo(addrinfo);
    return rc;
  }

  return 0;
}

static int cosim_socket_connect(int fd, struct addrinfo *addrinfo)
{
  int rc;
  int opt;
  int retries;

  assert(addrinfo != NULL);

  opt = 1;
  rc = setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)); 

  /* Bug78540: make several connect retries in case QEMU is not started yet */
  for (retries = 0; retries < COSIM_CONNECT_RETRIES; retries++) {
    int err;
    rc = connect(fd, addrinfo->ai_addr, addrinfo->ai_addrlen);
    if( rc == 0 ) {
      cosim_have_connection();
      break;
    }

    err = errno;

    TRACE(OVM_LOW, "Failed to connect socket: %s\n",
          strerror(err));

    /* Bug78540: retry only in case of connection refused */
    if (err != ECONNREFUSED)
      break;

    usleep(COSIM_CONNECT_RETRY_INTERVAL_US);
  }

  return rc;
}

static int cosim_socket_open(int is_listening, const char *address, int instance)
{
  struct addrinfo *addrinfo;
  int fd;
  int rc;

  addrinfo = cosim_get_addrinfo(is_listening, address, instance);
  if (addrinfo == NULL)
    return COSIM_SOCK_UNUSED;

  /* Create the socket. */
  fd = socket(addrinfo->ai_family, addrinfo->ai_socktype,
              addrinfo->ai_protocol);
  if (fd < 0) {
    ERR( "Failed to create socket: %s\n", strerror(errno));
    freeaddrinfo(addrinfo);
    return COSIM_SOCK_ERR;
  }

  /* Either bind or connect the socket. */
  if (is_listening)
    rc = cosim_socket_bind(fd, addrinfo);
  else
    rc = cosim_socket_connect(fd, addrinfo);

  freeaddrinfo(addrinfo);

  if (rc != 0) {
    close(fd);
    return COSIM_SOCK_ERR;
  }

  return fd;
}

cosim_socket *cosim_socket_new(cosim_platform_state_t *state, int is_listening, const char *address,
                               int instance, cosim_socket_close_fn close_fn,
                               void *opaque)
{
  cosim_socket *sock;
  cosim_hub *hub;
  int fd;
  int rc;

  fd = cosim_socket_open(is_listening, address, instance);
  if (fd == COSIM_SOCK_ERR)
    return NULL;

  /* If we got back COSIM_SOCK_UNUSED we complete the setup, except trying
   * to send the version number. That way we end up with a valid data structure
   * to keep the upper layers happy, even though the socket will never pass
   * any traffic */
  sock = calloc(1, sizeof(cosim_socket));
  if (sock == NULL) {
    ERR( "Failed to allocate %s socket.\n",
            is_listening ? "listening" : "connecting");
    close(fd);
    return NULL;
  }

  sock->state = state;
  
  hub = cosim_hub_new();
  if (hub == NULL) {
    free(sock);
    close(fd);
    return NULL;
  }

  cosim_socket_init(sock, hub, fd, close_fn, opaque);

  if (is_listening) {
    rc = cosim_set_fd_handler(state, fd, cosim_socket_accept, NULL, sock);
    if (rc != 0) {
      ERR( "Failed to set FD handlers (rc=%d)\n", rc);
      cosim_socket_delete(sock);
      return NULL;
    }
    sock->connected = 0;
  } else {
    if (fd != COSIM_SOCK_UNUSED) {
      rc = cosim_socket_send_version(sock);
      if (rc < 0) {
        cosim_socket_delete(sock);
        return NULL;
      }
    }
    rc = cosim_socket_add_clients(sock);
    if (rc != 0) {
      ERR( "Failed to add socket clients.\n");
      cosim_socket_delete(sock);
      return NULL;
    }

    sock->connected = (fd >= 0);
    sock->master = sock;
  }

  return sock;
}

#ifdef CONFIG_TLM
cosim_socket *sc_cosim_socket_new(cosim_platform_state_t *state, int instance)
{
  cosim_socket *sock;
  cosim_hub *hub;

  sock = calloc(1, sizeof(cosim_socket));
  if (sock == NULL) {
    return NULL;
  }

  sock->state = state;

  hub = cosim_hub_new();
  if (hub == NULL) {
    free(sock);
    return NULL;
  }

  cosim_socket_init(sock, hub, -1, NULL, NULL);

  return sock;
}
#endif

void cosim_socket_delete(cosim_socket *socket)
{
  cosim_socket *next;
  cosim_hub *hub;

  assert(socket != NULL);

  hub = socket->hub;

  while (socket != NULL) {
    next = socket->next;
    cosim_socket_fini(socket);
    free(socket);
    socket = next;
  }

  if (hub != NULL)
    cosim_hub_delete(hub);
}

int cosim_socket_connected(cosim_socket *socket)
{
  return (socket->connected != 0);
}
