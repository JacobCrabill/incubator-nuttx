/****************************************************************************
 * net/tcp/tcp_callback.c
 *
 *   Copyright (C) 2007-2009, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/uip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/netstats.h>

#include "uip/uip.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_dataevent
 *
 * Description:
 *   Handle data that is not accepted by the application because there is no
 *   listener in place ready to receive the data.
 *
 * Assumptions:
 * - The caller has checked that UIP_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static inline uint16_t
uip_dataevent(FAR struct uip_driver_s *dev, FAR struct tcp_conn_s *conn,
              uint16_t flags)
{
  uint16_t ret;

  /* Assume that we will ACK the data.  The data will be ACKed if it is
   * placed in the read-ahead buffer -OR- if it zero length
   */

  ret = (flags & ~UIP_NEWDATA) | UIP_SNDACK;

  /* Is there new data?  With non-zero length?  (Certain connection events
   * can have zero-length with UIP_NEWDATA set just to cause an ACK).
   */

  if (dev->d_len > 0)
    {
#ifdef CONFIG_NET_TCP_READAHEAD
      uint8_t *buffer = dev->d_appdata;
      int      buflen = dev->d_len;
      uint16_t recvlen;
#endif

      nllvdbg("No listener on connection\n");

#ifdef CONFIG_NET_TCP_READAHEAD
      /* Save as the packet data as in the read-ahead buffer.  NOTE that
       * partial packets will not be buffered.
       */

      recvlen = tcp_datahandler(conn, buffer, buflen);
      if (recvlen < buflen)
#endif
        {
          /* There is no handler to receive new data and there are no free
           * read-ahead buffers to retain the data -- drop the packet.
           */

         nllvdbg("Dropped %d bytes\n", dev->d_len);

 #ifdef CONFIG_NET_STATISTICS
          g_netstats.tcp.syndrop++;
          g_netstats.tcp.drop++;
#endif
          /* Clear the UIP_SNDACK bit so that no ACK will be sent */

          ret &= ~UIP_SNDACK;
        }
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tcp_callback
 *
 * Description:
 *   Inform the application holding the TCP socket of a change in state.
 *
 * Assumptions:
 *   This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

uint16_t tcp_callback(FAR struct uip_driver_s *dev, FAR struct tcp_conn_s *conn,
                      uint16_t flags)
{
  /* Preserve the UIP_ACKDATA, UIP_CLOSE, and UIP_ABORT in the response.
   * These is needed by uIP to handle responses and buffer state.  The
   * UIP_NEWDATA indication will trigger the ACK response, but must be
   * explicitly set in the callback.
   */

  nllvdbg("flags: %04x\n", flags);

  /* Perform the data callback.  When a data callback is executed from 'list',
   * the input flags are normally returned, however, the implementation
   * may set one of the following:
   *
   *   UIP_CLOSE   - Gracefully close the current connection
   *   UIP_ABORT   - Abort (reset) the current connection on an error that
   *                 prevents UIP_CLOSE from working.
   *
   * And/Or set/clear the following:
   *
   *   UIP_NEWDATA - May be cleared to indicate that the data was consumed
   *                 and that no further process of the new data should be
   *                 attempted.
   *   UIP_SNDACK  - If UIP_NEWDATA is cleared, then UIP_SNDACK may be set
   *                 to indicate that an ACK should be included in the response.
   *                 (In UIP_NEWDATA is cleared but UIP_SNDACK is not set, then
   *                 dev->d_len should also be cleared).
   */

  flags = uip_callbackexecute(dev, conn, flags, conn->list);

  /* There may be no new data handler in place at them moment that the new
   * incoming data is received.  If the new incoming data was not handled, then
   * either (1) put the unhandled incoming data in the read-ahead buffer (if
   * enabled) or (2) suppress the ACK to the data in the hope that it will
   * be re-transmitted at a better time.
   */

  if ((flags & UIP_NEWDATA) != 0)
    {
      /* Data was not handled.. dispose of it appropriately */

      flags = uip_dataevent(dev, conn, flags);
    }

  /* Check if there is a connection-related event and a connection
   * callback.
   */

  if (((flags & UIP_CONN_EVENTS) != 0) && conn->connection_event)
    {
      /* Perform the callback */

      conn->connection_event(conn, flags);
    }

  return flags;
}

/****************************************************************************
 * Function: tcp_datahandler
 *
 * Description:
 *   Handle data that is not accepted by the application.  This may be called
 *   either (1) from the data receive logic if it cannot buffer the data, or
 *   (2) from the TCP event logic is there is no listener in place ready to
 *   receive the data.
 *
 * Input Parameters:
 *   conn - A pointer to the TCP connection structure
 *   buffer - A pointer to the buffer to be copied to the read-ahead
 *     buffers
 *   buflen - The number of bytes to copy to the read-ahead buffer.
 *
 * Returned value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that UIP_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_READAHEAD
uint16_t tcp_datahandler(FAR struct tcp_conn_s *conn, FAR uint8_t *buffer,
                         uint16_t buflen)
{
  FAR struct iob_s *iob;
  int ret;

  /* Allocate on I/O buffer to start the chain (throttling as necessary) */

  iob = iob_alloc(true);
  if (iob == NULL)
    {
      nlldbg("ERROR: Failed to create new I/O buffer chain\n");
      return 0;
    }

  /* Copy the new appdata into the I/O buffer chain */

  ret = iob_copyin(iob, buffer, buflen, 0, true);
  if (ret < 0)
    {
      /* On a failure, iob_copyin return a negated error value but does
       * not free any I/O buffers.
       */

      nlldbg("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue */

  ret = iob_add_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  nllvdbg("Buffered %d bytes\n", buflen);
  return buflen;
}
#endif /* CONFIG_NET_TCP_READAHEAD */

#endif /* CONFIG_NET && CONFIG_NET_TCP */
