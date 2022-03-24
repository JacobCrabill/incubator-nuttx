/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h> /// DEBUGGING

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

#include <arch/board/board.h>

#ifdef CONFIG_NET_CMSG
#include <sys/time.h>
#endif

#ifndef OK
#define OK 0
#endif

#ifdef CONFIG_STM32H7_FDCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#define CANWORK LPWORK

#define CAN_STD_MASK                0x000007ff
#define CAN_EXT_MASK                0x1fffffff
#define CAN_EXT_FLAG                (1 << 31) /* Extended frame format */
#define CAN_RTR_FLAG                (1 << 30) /* Remote transmission request */
#define CAN_ERR_FLAG                (1 << 29) /* Error state indicator */

/* Message RAM */

#define RAM_STDID_SHIFT             18
#define RAM_STDID_MASK              0x1ffc0000 /* (CAN_STD_MASK << 18) & CAN_EXT_MASK */

#define WORD_LENGTH                 4U

#ifdef CONFIG_NET_CAN_CANFD
# define FIFO_ELEMENT_SIZE 18 // size (in Words) of a FIFO element in message RAM (CANFD_MTU / 4)
# define NUM_RX_FIFO0      14 // 14 elements max for RX FIFO0
# define NUM_RX_FIFO1      0  // No elements for RX FIFO1
# define NUM_TX_FIFO       7  // 7 elements max for TX queue
#else
# define FIFO_ELEMENT_SIZE 4  // size (in Words) of a FIFO element in message RAM (CAN_MTU / 4)
# define NUM_RX_FIFO0      64 // 64 elements max for RX FIFO0
# define NUM_RX_FIFO1      0  // No elements for RX FIFO1
# define NUM_TX_FIFO       32 // 32 elements max for TX queue
#endif

/* Intermediate message buffering */

#define POOL_SIZE                   1

#ifdef CONFIG_NET_CMSG
#define MSG_DATA                    sizeof(struct timeval)
#else
#define MSG_DATA                    0
#endif

/* CAN bit timing values  */

#define STM32_FDCANCLK              STM32_HSE_FREQUENCY
#define CLK_FREQ                    STM32_FDCANCLK
#define PRESDIV_MAX                 256
/// TODO: Be a good developer and put other important constants/constraints here

#define FDCAN_IR_MASK 0x3fcfffff // Mask of all non-reserved bits in FDCAN_IR

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE

#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support is required
#  endif

#define TX_TIMEOUT_WQ
#endif

#ifdef CONFIG_NET_CAN_LOOPBACK
/// TODO: Options to enable loopback, listen-only mode
///       HW-level external/internal loopback, etc.
/// SW_LOOPBACK intended to be used when HW loopback not enabed
# define SW_LOOPBACK
#endif

static int peak_tx_mbi_ = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN ID word, as defined by FDCAN device (Note xtd/rtr/esi bit positions) */
union can_id_e
{
  volatile uint32_t can_id;
  struct
  {
    volatile uint32_t extid : 29;
    volatile uint32_t resex : 3;
  };
  struct
  {
    volatile uint32_t res   : 18;
    volatile uint32_t stdid : 11;
    volatile uint32_t rtr   : 1;
    volatile uint32_t xtd   : 1;
    volatile uint32_t esi   : 1;
  };
};

/* Union of 4 bytes as 1 register */
union payload_e
{
  volatile uint32_t word;
  struct
  {
    volatile uint32_t b00 : 8;
    volatile uint32_t b01 : 8;
    volatile uint32_t b02 : 8;
    volatile uint32_t b03 : 8;
  };
};

/* Message RAM Structures */

// Rx FIFO Element Header -- RM0433 pg 2536
union rx_fifo_header_e
{
  struct
  {
    volatile uint32_t w0;
    volatile uint32_t w1;
  };

  struct
  {
    // First word
    union can_id_e id;

    // Second word
    volatile uint32_t rxts : 16; // Rx timestamp
    volatile uint32_t dlc  : 4;  // Data length code
    volatile uint32_t brs  : 1;  // Bitrate switching
    volatile uint32_t fdf  : 1;  // FD frame
    volatile uint32_t res  : 2;  // Reserved for Tx Event
    volatile uint32_t fidx : 7;  // Filter index
    volatile uint32_t anmf : 1;  // Accepted non-matching frame
  };
};

// Tx FIFO Element Header -- RM0433 pg 2538
union tx_fifo_header_e
{
  struct
  {
    volatile uint32_t w0;
    volatile uint32_t w1;
  };

  struct
  {
    // First word
    union can_id_e id;

    // Second word
    volatile uint32_t res1 : 16; // Reserved for Tx Event timestamp
    volatile uint32_t dlc  : 4;  // Data length code
    volatile uint32_t brs  : 1;  // Bitrate switching
    volatile uint32_t fdf  : 1;  // FD frame
    volatile uint32_t res2 : 1;  // Reserved for Tx Event
    volatile uint32_t efc  : 1;  // Event FIFO control
    volatile uint32_t mm   : 8;  // Message marker (user data; copied to Tx Event)
  };
};

// Rx FIFO Element
struct rx_fifo_s
{
  union rx_fifo_header_e header;
#ifdef CONFIG_NET_CAN_CANFD
  union payload_e data[16]; // 64-byte FD payload
#else
  union payload_e data[2];  // 8-byte Classic payload
#endif
};

// Tx FIFO Element
struct tx_fifo_s
{
  union tx_fifo_header_e header;
#ifdef CONFIG_NET_CAN_CANFD
  union payload_e data[16]; // 64-byte FD payload
#else
  union payload_e data[2];  // 8-byte Classic payload
#endif
};

/* Tx Mailbox Status Tracking */

#define TX_ABORT -1
#define TX_FREE 0
#define TX_BUSY 1

struct txmbstats
{
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct timeval deadline;
#endif
#ifdef SW_LOOPBACK
  bool loopback; /// TODO: not a feature that's currenlty enabled
# ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame frame;
# else
  struct can_frame frame;
# endif
#endif
  int8_t pending;  /* -1 = abort, 0 = free, 1 = busy  */
};

/* FDCAN Device hardware configuration */

struct fdcan_config_s
{
  uint32_t tx_pin;           /* GPIO configuration for TX */
  uint32_t rx_pin;           /* GPIO configuration for RX */
  uint32_t mb_irq[2];        /* FDCAN Interrupt 0, 1 (Rx, Tx) */
};

struct fdcan_timeseg
{
  uint32_t bitrate;
  uint8_t sjw;
  uint8_t bs1;
  uint8_t bs2;
  uint8_t prescaler;
};

struct fdcan_message_ram
{
  uint32_t filt_stdid_addr;
  uint32_t filt_extid_addr;
  uint32_t rxfifo0_addr;
  uint32_t rxfifo1_addr;
  uint32_t txfifo_addr;
  uint8_t n_stdfilt;
  uint8_t n_extfilt;
  uint8_t n_rxfifo0;
  uint8_t n_rxfifo1;
  uint8_t n_txfifo;
};

/* FDCAN device structures */

#ifdef CONFIG_STM32H7_FDCAN1
static const struct fdcan_config_s stm32_fdcan0_config =
{
  .tx_pin      = GPIO_CAN1_TX,
  .rx_pin      = GPIO_CAN1_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN1_0,
    STM32_IRQ_FDCAN1_1,
  },
};
#endif

#ifdef CONFIG_STM32H7_FDCAN2
static const struct fdcan_config_s stm32_fdcan1_config =
{
  .tx_pin      = GPIO_CAN2_TX,
  .rx_pin      = GPIO_CAN1_RX,
  .mb_irq      =
  {
    STM32_IRQ_FDCAN2_0,
    STM32_IRQ_FDCAN2_1 ,
  },
};
#endif

/* The stm32_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct stm32_driver_s
{
  uint32_t base;                /* FDCAN base address */
  uint8_t iface_idx;            /* FDCAN interface index (0 or 1) */
  bool bifup;                   /* true:ifup false:ifdown */
#ifdef TX_TIMEOUT_WQ
  wdog_s txtimeout[NUM_TX_FIFO]; /* TX timeout timer */
#endif
  struct work_s irqwork;        /* For deferring interrupt work to the wq */
  struct work_s pollwork;       /* For deferring poll work to the work wq */
#ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame *txdesc;   /* A pointer to the list of TX descriptors */
  struct canfd_frame *rxdesc;   /* A pointer to the list of RX descriptors */
#else
  struct can_frame *txdesc;     /* A pointer to the list of TX descriptors */
  struct can_frame *rxdesc;     /* A pointer to the list of RX descriptors */
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;      /* Interface understood by the network */

  struct rx_fifo_s *rx;
  struct tx_fifo_s *tx;

  struct fdcan_timeseg arbi_timing; /* Timing for arbitration phase */
#ifdef CONFIG_NET_CAN_CANFD
  struct fdcan_timeseg data_timing; /* Timing for data phase */
#endif

  struct fdcan_message_ram message_ram;

  const struct fdcan_config_s *config;

  struct txmbstats txmb[NUM_TX_FIFO];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN1
static struct stm32_driver_s g_fdcan0;
#endif

#ifdef CONFIG_STM32H7_FDCAN2
static struct stm32_driver_s g_fdcan1;
#endif

#ifdef CONFIG_NET_CAN_CANFD
static uint8_t g_tx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
#else
static uint8_t g_tx_pool[sizeof(struct can_frame)*POOL_SIZE];
static uint8_t g_rx_pool[sizeof(struct can_frame)*POOL_SIZE];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lsb
 *
 * Description:
 *   Calculate position of lsb that's equal to 1
 *
 * Input Parameters:
 *   value - The value to perform the operation on
 *
 * Returned Value:
 *   location of lsb which is equal to 1, returns 32 when value is 0
 *
 ****************************************************************************/

static inline uint32_t arm_lsb(unsigned int value)
{
  uint32_t ret;
  volatile uint32_t rvalue = value;
  __asm__ __volatile__ ("rbit %1,%0" : "=r" (rvalue) : "r" (rvalue));
  __asm__ __volatile__ ("clz %0, %1" : "=r"(ret) : "r"(rvalue));
  return ret;
}

static void dumpregs(FAR struct stm32_driver_s *priv)
{
  printf("-------------- Reg Dump ----------------\n");
  printf("CAN%d Base: 0x%lx\n", priv->iface_idx, priv->base);

  uint32_t regval;
  regval = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
  printf("CCCR = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
  printf("ECR  = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);
  printf("IR   = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  printf("IE   = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ILE_OFFSET);
  printf("ILE = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  printf("ILS = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_NBTP_OFFSET);
  printf("NBTP = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_DBTP_OFFSET);
  printf("DBTP = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET);
  printf("TXBC = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_RXF0C_OFFSET);
  printf("RXF0C = 0x%lx\n", regval);

  regval = getreg32(priv->base + STM32_FDCAN_RXESC_OFFSET);
  printf("RXESC = 0x%lx\n", regval);
  regval = getreg32(priv->base + STM32_FDCAN_TXESC_OFFSET);
  printf("TXESC = 0x%lx\n", regval);
}

/****************************************************************************
 * Name: stm32_bitratetotimeseg
 *
 * Description:
 *   Convert bitrate to timeseg values
 *
 * Input Parameters:
 *   timeseg - structure to store bit timing
 *   can_fd - if set to true calculate CAN FD data-segment bit timings,
 *            otherwise calculate classical can timings
 *
 * Returned Value:
 *   OK on success; >0 on failure.
 ****************************************************************************/

int32_t stm32_bitratetotimeseg(struct fdcan_timeseg *timeseg, bool can_fd)
{
  /// TODO: Verify this works for data phase of CAN-FD as well
  /*
   * Implementation ported from PX4's uavcan_drivers/stm32[h7]
   *
   * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
   *      CAN in Automation, 2003
   *
   * According to the source, optimal quanta per bit are:
   *   Bitrate        Optimal Maximum
   *   1000 kbps      8       10
   *   500  kbps      16      17
   *   250  kbps      16      17
   *   125  kbps      16      17
   */
  const uint32_t target_bitrate = timeseg->bitrate;
  static const int32_t MaxBS1 = 16;
  static const int32_t MaxBS2 = 8;
  const uint8_t max_quanta_per_bit = (timeseg->bitrate >= 1000000) ? 10 : 17;
  static const int MaxSamplePointLocation = 900;

  /*
   * Computing (prescaler * BS):
   *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
   *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
   * let:
   *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
   *   PRESCALER_BS = PRESCALER * BS
   * ==>
   *   PRESCALER_BS = PCLK / BITRATE
   */
  const uint32_t prescaler_bs = CLK_FREQ / target_bitrate;
  printf("CLK_FREQ %lu, target_bitrate %lu, prescaler %lu\n", CLK_FREQ, target_bitrate, prescaler_bs);

  /*
   * Searching for such prescaler value so that the number of quanta per bit is highest.
   */
  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;

  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
      if (bs1_bs2_sum <= 2)
        {
          syslog(LOG_INFO, "foobar ----------------\n");
          ninfo("Target bitrate too high - no solution possible.");
          return 1; // No solution
        }

      bs1_bs2_sum--;
    }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);

  if ((prescaler < 1U) || (prescaler > 1024U))
    {
      ninfo("Target bitrate invalid - bad prescaler.");
      return 2; // No solution
    }

  /*
   * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
   * We need to find the values so that the sample point is as close as possible to the optimal value.
   *
   *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
   *   {{bs2 -> (1 + bs1)/7}}
   *
   * Hence:
   *   bs2 = (1 + bs1) / 7
   *   bs1 = (7 * bs1_bs2_sum - 1) / 8
   *
   * Sample point location can be computed as follows:
   *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
   *
   * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
   *   - With rounding to nearest
   *   - With rounding to zero
   */

  // First attempt with rounding to nearest
  uint8_t bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
  uint16_t sample_point_permill = (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));

  if (sample_point_permill > MaxSamplePointLocation)
    {
      // Second attempt with rounding to zero
      bs1 = (7 * bs1_bs2_sum - 1) / 8;
      bs2 = bs1_bs2_sum - bs1;
    }

  bool valid = (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);

  /*
   * Final validation
   * Helpful Python:
   * def sample_point_from_btr(x):
   *     assert 0b0011110010000000111111000000000 & x == 0
   *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
   *     return (1+ts1+1)/(1+ts1+1+ts2+1)
   *
   */
  if ((target_bitrate != (CLK_FREQ / (prescaler * (1 + bs1 + bs2)))) || !valid)
    {
      printf("valid: %d, found bitrate: %lu\n", valid, (CLK_FREQ / (prescaler * (1 + bs1 + bs2))));
      ninfo("Target bitrate invalid - solution does not match.");
      return 3; // Solution not found
    }

  timeseg->bs1 = (uint8_t)(bs1 - 1);
  timeseg->bs2 = (uint8_t)(bs2 - 1);
  timeseg->prescaler = (uint16_t)(prescaler - 1);
  timeseg->sjw = 0; // Which means one

  return 0;
}

/* Common TX logic */

static bool stm32_txringfull(FAR struct stm32_driver_s *priv);
static int  stm32_transmit(FAR struct stm32_driver_s *priv);
static int  stm32_txpoll(struct net_driver_s *dev);

/* Helper functions */

static void stm32_apb1hreset(void);
static void stm32_setinit(uint32_t base, uint32_t init);
static void stm32_setenable(uint32_t base, uint32_t enable);
static void stm32_setconfig(uint32_t base, uint32_t config_enable);
static uint32_t stm32_waitccr_change(uint32_t base,
                                       uint32_t mask,
                                       uint32_t target_state);

static void stm32_enable_interrupts(struct stm32_driver_s *priv);
static void stm32_disable_interrupts(struct stm32_driver_s *priv);

/* Interrupt handling */

static void stm32_receive(FAR struct stm32_driver_s *priv);
static void stm32_txdone(FAR struct stm32_driver_s *priv);

static int  stm32_fdcan_interrupt(int irq, FAR void *context,
                                      FAR void *arg);

static void stm32_check_errors_isr(FAR struct stm32_driver_s *priv);

/* Watchdog timer expirations */
#ifdef TX_TIMEOUT_WQ
static void stm32_txtimeout_work(FAR void *arg);
static void stm32_txtimeout_expiry(int argc, uint32_t arg, ...);
#endif

/* NuttX callback functions */

static int  stm32_ifup(struct net_driver_s *dev);
static int  stm32_ifdown(struct net_driver_s *dev);

static void stm32_txavail_work(FAR void *arg);
static int  stm32_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  stm32_ioctl(struct net_driver_s *dev, int cmd,
                          unsigned long arg);
#endif

/* Initialization */

static int  stm32_initialize(struct stm32_driver_s *priv);
static void stm32_reset(struct stm32_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool stm32_txringfull(FAR struct stm32_driver_s *priv)
{
  // Check that we even _have_ a Tx FIFO allocated
  /// JACOB TODO: Decide if this needs to be checked every time, or just during init
  if ((getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET) & FDCAN_TXBC_TFQS) == 0) {
    // Your queue size is 0, you did something wrong
    nerr("No Tx FIFO buffers assigned?  Check your message RAM config\n");
    return true;
  }

  // Check if the Tx queue is full
  if ((getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET) & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
    // Sorry, out of room, try back later
    return true;
  }

  return false;
}

/****************************************************************************
 * Function: stm32_transmit
 *
 * Description:
 *   Start hardware transmission of the data contained in priv->d_buf.  Called
 *   either from the txdone interrupt handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_transmit(FAR struct stm32_driver_s *priv)
{
  /* From UAVCAN uc_stm32h7_can.cpp: */
  /*
   * Normally we should perform the same check as in @ref canAcceptNewTxFrame(), because
   * it is possible that the highest-priority frame between select() and send() could have been
   * replaced with a lower priority one due to TX timeout. But we don't do this check because:
   *
   *  - It is a highly unlikely scenario.
   *
   *  - Frames do not timeout on a properly functioning bus. Since frames do not timeout, the new
   *    frame can only have higher priority, which doesn't break the logic.
   *
   *  - If high-priority frames are timing out in the TX queue, there's probably a lot of other
   *    issues to take care of before this one becomes relevant.
   *
   *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
   */
  irqstate_t flags = enter_critical_section();

  // First, check if there are any slots available in the queue
  if ((getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET) & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF)
    {
      // Tx FIFO / Queue is full
      leave_critical_section(flags);
      return -EBUSY;
    }

  // Next, get the next available FIFO index from the controller
  const uint8_t mbi = (getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET) & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_SHIFT;

  // Now, we can copy the CAN frame to the FIFO (in message RAM)

  if (mbi >= NUM_TX_FIFO)
    {
      nerr("Invalid Tx mailbox index encountered in transmit\n");
      leave_critical_section(flags);
      return -EIO;
    }

  struct tx_fifo_s *mb = &priv->tx[mbi];

  /* Setup timeout deadline if enabled */

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t timeout = 0;
  struct timespec ts;
  clock_systimespec(&ts);

  if (priv->dev.d_sndlen > priv->dev.d_len)
    {
      struct timeval *tv =
             (struct timeval *)(priv->dev.d_buf + priv->dev.d_len);
      priv->txmb[mbi].deadline = *tv;
      timeout  = (tv->tv_sec - ts.tv_sec)*CLK_TCK
                 + ((tv->tv_usec - ts.tv_nsec / 1000)*CLK_TCK) / 1000000;
      if (timeout < 0)
        {
          leave_critical_section(flags);
          return 0;  /* No transmission for you! */
        }
    }
  else
    {
      /* Default TX deadline defined in NET_CAN_RAW_DEFAULT_TX_DEADLINE */

      if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
        {
          timeout = ((CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000)
              *CLK_TCK);
          priv->txmb[mbi].deadline.tv_sec = ts.tv_sec +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000;
          priv->txmb[mbi].deadline.tv_usec = (ts.tv_nsec / 1000) +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE % 1000000;
        }
      else
        {
          priv->txmb[mbi].deadline.tv_sec = 0;
          priv->txmb[mbi].deadline.tv_usec = 0;
        }
    }
#endif

  /* Attempt to write frame */

  /* For statistics purposes, keep track of highest waterline in TX mailbox */
  peak_tx_mbi_ = (peak_tx_mbi_ > mbi ? peak_tx_mbi_ : mbi);

  union tx_fifo_header_e header;

  if (priv->dev.d_len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;

      if (frame->can_id & CAN_EXT_FLAG)
        {
          header.id.xtd = 1;  // Extended ID frame
          header.id.extid = frame->can_id & CAN_EXT_MASK;
        }
      else
        {
          header.id.xtd = 0;  // Standard ID frame
          header.id.stdid = frame->can_id & CAN_STD_MASK;
        }

      header.id.esi = frame->can_id & CAN_ERR_FLAG ? 1 : 0;
      header.id.rtr = frame->can_id & CAN_RTR_FLAG ? 1 : 0;
      header.dlc = frame->can_dlc;
      header.efc = 0;  // Don't store Tx events
      header.mm = mbi; // Marker for our own use; just store FIFO index
      header.fdf = 0;  // Classic CAN frame, not CAN-FD
      header.brs = 0;  // No bitrate switching

      // Store into message RAM
      mb->header.w0 = header.w0;
      mb->header.w1 = header.w1;
      mb->data[0].word = *(uint32_t *)&frame->data[0];
      mb->data[1].word = *(uint32_t *)&frame->data[4];

#ifdef SW_LOOPBACK
      priv->txmb[mbi].frame = *frame;
#endif
    }
#ifdef CONFIG_NET_CAN_CANFD
  else /* CAN FD frame */
    {
      struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;

      if (frame->can_id & CAN_EXT_FLAG)
        {
          header.id.xtd = 1;  // Extended ID frame
          header.id.extid = frame->can_id & CAN_EXT_MASK;
        }
      else
        {
          header.id.xtd = 0;  // Standard ID frame
          header.id.stdid = frame->can_id & CAN_STD_MASK;
        }

      const bool brs = (priv->arbi_timing.bitrate == priv->data_timing.bitrate) ? 0 : 1;

      header.id.esi = (frame->can_id & CAN_ERR_FLAG) ? 1 : 0;
      header.id.rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;
      header.dlc = len_to_can_dlc[frame->len];
      header.brs = brs; // Bitrate switching
      header.fdf = 1; // CAN-FD frame
      header.efc = 0;  // Don't store Tx events
      header.mm = mbi; // Marker for our own use; just store FIFO index

      // Store into message RAM
      mb->header.w1 = header.w1;
      mb->header.w0 = header.w0;

      uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

      for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
        {
          mb->data[i].word = frame_data_word[i];
        }

#ifdef SW_LOOPBACK
      priv->txmb[mbi].frame = *frame;
#endif
    }
#endif

  /* GO - Submit the transmission request for this element */

  putreg32(1 << mbi, priv->base + STM32_FDCAN_TXBAR_OFFSET);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  priv->txmb[mbi].pending = TX_BUSY;

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout > 0)
    {
      wd_start(&priv->txtimeout[mbi], timeout + 1, stm32_txtimeout_expiry,
                1, (wdparm_t)priv);
    }
#endif

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Function: stm32_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_txpoll(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          stm32_transmit(priv);

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if (stm32_txringfull(priv))
            {
              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: stm32_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

// static void stm32_irprint(FAR struct stm32_driver_s *priv)
// {
//   uint32_t regval = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

//   printf(">> IR = 0x%lx\n", regval);
//   printf("IR.PEA (Protocol Error Arb): %d\n", (regval & FDCAN_IR_PEA) > 0);
//   printf("IR.WDI (Watchdog intr): %d\n", (regval & FDCAN_IR_WDI) > 0);
//   printf("IR.BO (Bus Off): %d\n", (regval & FDCAN_IR_BO) > 0);
//   printf("IR.EW (Warning Status) : %d\n", (regval & FDCAN_IR_EW) > 0);
//   printf("IR.EP (Error Passive): %d\n", (regval & FDCAN_IR_EP) > 0);
//   printf("IR.ELO (Error Logging Overflow): %d\n", (regval & FDCAN_IR_ELO) > 0);
//   printf("IR.TOO (Timeout Occurred): %d\n", (regval & FDCAN_IR_TOO) > 0);
//   printf("IR.MRAF (Msg RAM Access Failure): %d\n", (regval & FDCAN_IR_MRAF) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
//   // printf("IR. (): %d\n", (regval & FDCAN_IR_) > 0);
// }

static void stm32_receive(FAR struct stm32_driver_s *priv)
{
  // stm32_irprint(priv); /// DEBUGGING

  uint32_t regval = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

  const uint32_t ir_fifo0 = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
  const uint32_t ir_fifo1 = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
  uint8_t fifo_id;

  if (regval & ir_fifo0)
    {
      regval = ir_fifo0;
      fifo_id = 0;
    }
  else if (regval & ir_fifo1)
    {
      regval = ir_fifo1;
      fifo_id = 1;
    }
  else
    {
      nerr("ERROR: Bad RX IR flags");
      return;
    }

  /* Write the corresponding interrupt bits to reset them */

  putreg32(regval, priv->base + STM32_FDCAN_IR_OFFSET);

  // Bitwise register definitions are the same for FIFO 0/1
  const uint32_t FDCAN_RXFnC_FnS        = FDCAN_RXF0C_F0S;  // Rx FIFO Size
  const uint32_t FDCAN_RXFnS_RFnL       = FDCAN_RXF0S_RF0L; // Rx Message Lost
  const uint32_t FDCAN_RXFnS_FnFL       = FDCAN_RXF0S_F0FL; // Rx FIFO Fill Level
  const uint32_t FDCAN_RXFnS_FnGI       = FDCAN_RXF0S_F0GI; // Rx FIFO Get Index
  const uint32_t FDCAN_RXFnS_FnGI_SHIFT = FDCAN_RXF0S_F0GI_SHIFT;

  uint32_t offset_rxfnc = (fifo_id == 0) ? STM32_FDCAN_RXF0C_OFFSET : STM32_FDCAN_RXF1C_OFFSET;
  uint32_t offset_rxfns = (fifo_id == 0) ? STM32_FDCAN_RXF0S_OFFSET : STM32_FDCAN_RXF1S_OFFSET;
  uint32_t offset_rxfna = (fifo_id == 0) ? STM32_FDCAN_RXF0A_OFFSET : STM32_FDCAN_RXF1A_OFFSET;

  volatile uint32_t *const RXFnC = (uint32_t *)(priv->base + offset_rxfnc);
  volatile uint32_t *const RXFnS = (uint32_t *)(priv->base + offset_rxfns);
  volatile uint32_t *const RXFnA = (uint32_t *)(priv->base + offset_rxfna);

  // Check number of elements in message RAM allocated to this FIFO
  if ((*RXFnC & FDCAN_RXFnC_FnS) == 0)
    {
      nerr("ERROR: No RX FIFO elements allocated");
      return;
    }

  // Check for message lost; count an error
  if ((*RXFnS & FDCAN_RXFnS_RFnL) != 0) {
    NETDEV_RXERRORS(&priv->dev);
  }

  // Check number of elements available (fill level)
  const uint8_t n_elem = (*RXFnS & FDCAN_RXFnS_FnFL);

  if (n_elem == 0) {
    // No elements available?
    nerr("RX interrupt but 0 frames available");
    return;
  }

  struct rx_fifo_s *rf = NULL;

  while ((*RXFnS & FDCAN_RXFnS_FnFL) > 0)
    {
      // Copy the frame from message RAM

      const uint8_t index = (*RXFnS & FDCAN_RXFnS_FnGI) >> FDCAN_RXFnS_FnGI_SHIFT;

      rf = &priv->rx[index];

      /* Read the frame contents */

#ifdef CONFIG_NET_CAN_CANFD
      if (rf->header.fdf) /* CAN FD frame */
        {
          struct canfd_frame *frame = (struct canfd_frame *)priv->rxdesc;

          if (rf->header.id.xtd)
            {
              frame->can_id  = CAN_EXT_MASK & rf->header.id.extid;
              frame->can_id |= CAN_EXT_FLAG;
            }
          else
            {
              frame->can_id = CAN_STD_MASK & rf->header.id.stdid;
            }

          if (rf->header.id.rtr)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          frame->len = can_dlc_to_len[rf->header.dlc];

          uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

          for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
            {
              frame_data_word[i] = rf->data[i].word;
            }

          /* Acknowledge receipt of this FIFO element */

          putreg32(index, RXFnA);

          /* Copy the buffer pointer to priv->dev
           * Set amount of data in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct canfd_frame);
          priv->dev.d_buf = (uint8_t *)frame;
        }
      else /* CAN 2.0 Frame */
#endif
        {
          struct can_frame *frame = (struct can_frame *)priv->rxdesc;

          if (rf->header.id.xtd)
            {
              frame->can_id  = CAN_EXT_MASK & rf->header.id.extid;
              frame->can_id |= CAN_EXT_FLAG;
            }
          else
            {
              frame->can_id = CAN_STD_MASK & rf->header.id.stdid;
            }

          if (rf->header.id.rtr)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          frame->can_dlc = rf->header.dlc;

          *(uint32_t *)&frame->data[0] = rf->data[0].word;
          *(uint32_t *)&frame->data[4] = rf->data[1].word;


          /* Acknowledge receipt of this FIFO element */

          putreg32(index, RXFnA);

          /* Copy the buffer pointer to priv->dev
           * Set amount of data in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct can_frame);
          priv->dev.d_buf = (uint8_t *)frame;
        }

      /* Send to socket interface */

      can_input(&priv->dev);

      /* Update iface statistics */

      NETDEV_RXPACKETS(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }

  // update_event_.signalFromInterrupt();

  stm32_check_errors_isr(priv);
}

/****************************************************************************
 * Function: stm32_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *   The network is locked.
 *
 ****************************************************************************/

static void stm32_txdone(FAR struct stm32_driver_s *priv)
{
  uint32_t IR = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);
  if (IR & FDCAN_IR_TC)
  {
    putreg32(FDCAN_IR_TC, priv->base + STM32_FDCAN_IR_OFFSET);
  }
  else
  {
    // Not the interrupt we're looking for
    nerr("Unexpected FCAN interrupt on line 1\n");
    return;
  }

  /* Process TX completions */

  // Update counters for successful transmissions
  // Process loopback messages (NOT PROPERLY SUPPORED YET!)
  for (uint8_t i = 0; i < NUM_TX_FIFO; i++)
    {
      if ((getreg32(priv->base + STM32_FDCAN_TXBTO_OFFSET) & (1 << i)) > 0)
        {
          // Transmission Occurred in buffer i [not necessarily a 'new' transmission, however]
          // Check that it's a new transmission, not a previously handled transmission
          /// TODO: Still need to check on handling timeouts and cancellations

          struct txmbstats *txi = &priv->txmb[i];

          if (txi->pending == TX_BUSY)
            {
              /* This was a transmission that just now completed */

              NETDEV_TXDONE(&priv->dev);

#ifdef SW_LOOPBACK
              /// TODO: How is loopback handled in SocketCAN?
              /// TODO: 'loopback' var is useless currently. Can user request loopback at socket level?
              // if (txi->loopback)
                {
                  /* Send to socket interface */
                  /// TODO: I don't think this will work as-is
                  priv->dev.d_len = sizeof(txi->frame);
                  priv->dev.d_buf = (uint8_t *)&txi->frame;
                  can_input(&priv->dev);
                }
#endif
             txi->pending = TX_FREE;

#ifdef TX_TIMEOUT_WQ
              /* We are here because a transmission completed, so the
              * corresponding watchdog can be canceled.
              */

              wd_cancel(&priv->txtimeout[i]);
#endif
            }
        }
    }

  /* Check for errors and abort-transmission requests */

  stm32_check_errors_isr(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  devif_poll(&priv->dev, stm32_txpoll);
}

/****************************************************************************
 * Function: stm32_fdcan_interrupt
 *
 * Description:
 *   Three interrupt sources will vector this this function:
 *   1. CAN MB transmit interrupt handler
 *   2. CAN MB receive interrupt handler
 *   3.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_fdcan_interrupt(int irq, FAR void *context,
                                 FAR void *arg)
{
  switch (irq)
    {
#ifdef CONFIG_STM32H7_FDCAN1
      case STM32_IRQ_FDCAN1_0:
        stm32_receive(&g_fdcan0);
        break;

      case STM32_IRQ_FDCAN1_1:
        stm32_txdone(&g_fdcan0);
        break;
#endif

#ifdef CONFIG_STM32H7_FDCAN2
      case STM32_IRQ_FDCAN2_0:
        stm32_receive(&g_fdcan1);
        break;

      case STM32_IRQ_FDCAN2_1:
        stm32_txdone(&g_fdcan1);
        break;
#endif

      default:
        nerr("Unexpected IRQ [%d]\n", irq);
        return -1;
    }

  return OK;
}

/****************************************************************************
 * Function: stm32_check_errors_isr
 *
 * Description:
 *   Check error flags and any cancel any timed out transmissions
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Assumptions:
 *  Called from interrupt context
 ****************************************************************************/

static void stm32_check_errors_isr(FAR struct stm32_driver_s *priv)
{
  // Read CAN Error Logging counter (This also resets the error counter)
  uint32_t regval = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET) & FDCAN_ECR_CEL;
  const uint8_t cel = (uint8_t)(regval >> FDCAN_ECR_CEL_SHIFT);

  if (cel > 0)
    {
      // We've had some errors; check the status of the device
      regval = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
      bool restricted_op_mode = (regval & FDCAN_CCCR_ASM) > 0;

      if (restricted_op_mode)
        {
          nerr("Tx handler message RAM error -- resctricted mode enabled\n");
          // Reset the CCCR.ASM register to exit restricted op mode
          putreg32(regval & ~FDCAN_CCCR_ASM, priv->base + STM32_FDCAN_CCCR_OFFSET);
        }
    }

  // Serve abort requests
  for (uint8_t i = 0; i < NUM_TX_FIFO; i++)
    {
      struct txmbstats *txi = &priv->txmb[i];

      if (txi->pending == TX_ABORT && ((1 << i) & getreg32(priv->base + STM32_FDCAN_TXBRP_OFFSET)))
        {
          // Request to Cancel Tx item
          putreg32(1 << i, priv->base + STM32_FDCAN_TXBCR_OFFSET);
          txi->pending = TX_FREE;
          NETDEV_TXERRORS(&priv->dev);
#ifdef TX_TIMEOUT_WQ
          wd_cancel(&priv->txtimeout[mbi]);
#endif
        }
    }
}

/****************************************************************************
 * Function: stm32_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Assumptions:
 *
 ****************************************************************************/
#ifdef TX_TIMEOUT_WQ

static void stm32_txtimeout_work(FAR void *arg)
{
  FAR struct stm32_driver_s *priv = (FAR struct stm32_driver_s *)arg;

  struct timespec ts;
  struct timeval *now = (struct timeval *)&ts;
  clock_systimespec(&ts);
  now->tv_usec = ts.tv_nsec / 1000; /* timespec to timeval conversion */

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  for (int mbi = 0; mbi < NUM_TX_FIFO; mbi++)
    {
      if (priv->txmb[mbi].deadline.tv_sec != 0
          && (now->tv_sec > priv->txmb[mbi].deadline.tv_sec
          || now->tv_usec > priv->txmb[mbi].deadline.tv_usec))
        {
          NETDEV_TXTIMEOUTS(&priv->dev);
          struct tx_fifo_s *mb = &priv->tx[mbi];
          priv->txmb[mbi].pending = TX_ABORT;
        }
    }

  stm32_check_errors_isr(priv);
}

/****************************************************************************
 * Function: stm32_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  FAR struct stm32_driver_s *priv = (FAR struct stm32_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread
   */

  work_queue(CANWORK, &priv->irqwork, stm32_txtimeout_work, priv, 0);
}

#endif

static void stm32_apb1hreset(void)
{
  modifyreg32(STM32_RCC_APB1HRSTR, 0, RCC_APB1HRSTR_FDCANRST);
  modifyreg32(STM32_RCC_APB1HRSTR, RCC_APB1HRSTR_FDCANRST, 0);
}

static void stm32_setinit(uint32_t base, uint32_t init)
{
  if (init)
    {
      /* Enter hardware initialization mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);
      stm32_waitccr_change(base, FDCAN_CCCR_INIT, FDCAN_CCCR_INIT);
    }
  else
    {
      /* Exit hardware initialization mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_INIT, 0);
      stm32_waitccr_change(base, FDCAN_CCCR_INIT, 0);
    }
}

/* Power On / Power Off the device with the Clock Stop Request bit */
static void stm32_setenable(uint32_t base, uint32_t enable)
{
  if (enable)
    {
      /* Clear CSR bit */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CSR, 0);
      stm32_waitccr_change(base, FDCAN_CCCR_CSA, 0);
    }
  else
    {
      /* Set CSR bit */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CSR);
      stm32_waitccr_change(base, FDCAN_CCCR_CSA, 1);
    }
}

static void stm32_setconfig(uint32_t base, uint32_t config_enable)
{
  if (config_enable)
    {
      /* Configuration Changes Enabled (CCE) mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CCE);
      stm32_waitccr_change(base, FDCAN_CCCR_CCE, 1);
    }
  else
    {
      /* Exit CCE mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CCE, 0);
      stm32_waitccr_change(base, FDCAN_CCCR_CCE, 0);
    }
}

static uint32_t stm32_waitccr_change(uint32_t base, uint32_t mask,
                                       uint32_t target_state)
{
  const unsigned timeout = 1000;
  for (unsigned wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      const bool state = (getreg32(base + STM32_FDCAN_CCCR_OFFSET) & mask) != 0;
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

static void stm32_enable_interrupts(struct stm32_driver_s *priv)
{
  // Enable both interrupt lines at the device level
  modifyreg32(priv->base + STM32_FDCAN_ILE_OFFSET, 0, FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1);

  // Enable both lines at the NVIC (Nested Vector Interrupt Controller) level
  up_enable_irq(priv->config->mb_irq[0]);
  up_enable_irq(priv->config->mb_irq[1]);
}

static void stm32_disable_interrupts(struct stm32_driver_s *priv)
{
  // Disable both lines at the NVIC (Nested Vector Interrupt Controller) level
  up_disable_irq(priv->config->mb_irq[0]);
  up_disable_irq(priv->config->mb_irq[1]);

  // Disable both interrupt lines at the device level
  modifyreg32(priv->base + STM32_FDCAN_ILE_OFFSET, FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1, 0);
}


/****************************************************************************
 * Function: stm32_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the CAN interface when a socket is opened
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device is initialized and waiting to be brought online
 ****************************************************************************/

static int stm32_ifup(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  /* Wake up the device and enter config mode */

  irqstate_t flags = enter_critical_section();

  dumpregs(priv);

  stm32_setenable(priv->base, 1);
  stm32_setinit(priv->base, 1);
  stm32_setconfig(priv->base, 1);

  /* Enable interrupts (at both device and NVIC level) */

  stm32_enable_interrupts(priv);

  /* Leave init mode */

  stm32_setinit(priv->base, 0);

  dumpregs(priv);

  leave_critical_section(flags);

  priv->bifup = true;

  return OK;
}

/****************************************************************************
 * Function: stm32_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifdown(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  stm32_reset(priv);

  priv->bifup = false;

  return OK;
}

/****************************************************************************
 * Function: stm32_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void stm32_txavail_work(FAR void *arg)
{
  FAR struct stm32_driver_s *priv = (FAR struct stm32_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!stm32_txringfull(priv))
        {
          /* There is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, stm32_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: stm32_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus to perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int stm32_txavail(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      stm32_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: stm32_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int stm32_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg)
{
  FAR struct stm32_driver_s *priv = dev->d_private;

  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_bitrate_s *req =
              (struct can_ioctl_bitrate_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->arbi_timing.bitrate / 1000; /* kbit/s */
#ifdef CONFIG_NET_CAN_CANFD
          req->data_bitrate = priv->data_timing.bitrate / 1000; /* kbit/s */
#else
          req->data_bitrate = 0;
#endif
          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct can_ioctl_bitrate_s *req =
              (struct can_ioctl_bitrate_s *)((uintptr_t)arg);

          priv->arbi_timing.bitrate = req->arbi_bitrate * 1000;
#ifdef CONFIG_NET_CAN_CANFD
          priv->data_timing.bitrate = req->data_bitrate * 1000;
#endif

          /* Reset CAN controller and start with new timings */

          ret = stm32_initialize(priv);

          if (ret == OK)
            {
              ret = stm32_ifup(dev);
            }
        }
        break;
#endif /* CONFIG_NETDEV_CAN_BITRATE_IOCTL */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
      case SIOCACANEXTFILTER:
        {
          /// TODO: Add hardware-level filter...

          stm32_addextfilter(priv, (FAR struct canioc_extfilter_s *)arg);
        }
        break;

      case SIOCDCANEXTFILTER:
        {
          /// TODO: Delete hardware-level filter...

          stm32_delextfilter(priv, (FAR struct canioc_extfilter_s *)arg);
        }
        break;

      case SIOCACANSTDFILTER:
        {
          /// TODO: Add hardware-level filter...

          stm32_addstdfilter(priv, (FAR struct canioc_stdfilter_s *)arg);
        }
        break;

      case SIOCDCANSTDFILTER:
        {
          /// TODO: Delete hardware-level filter...

          stm32_delstdfilter(priv, (FAR struct canioc_stdfilter_s *)arg);
        }
        break;
#endif

      default:
        ret = -ENOTSUP;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: stm32_initialize
 *
 * Description:
 *   Initialize FDCAN device
 *
 * Input Parameters:
 *   priv - Reference to the private FDCAN driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_initialize(struct stm32_driver_s *priv)
{
  uint32_t regval;

  irqstate_t flags = enter_critical_section();

  /*
   * Wake up the device and enable configuration changes
   */
  if (priv->iface_idx == 0)
    {
      stm32_apb1hreset();
    }

  // Exit Power-down / Sleep mode, then wait for acknowledgement
  stm32_setenable(priv->base, 1);

  // Request Init mode, then wait for completion
  stm32_setinit(priv->base, 1);

  // Configuration Changes Enable.  Can only be set during Init mode;
  // cleared when INIT bit is cleared.
  stm32_setconfig(priv->base, 1);

  // Disable interrupts while we configure the hardware
  putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

  dumpregs(priv);

  /*
   * CAN timings for this bitrate
   */

  // Nominal / arbitration phase bitrate

  if (stm32_bitratetotimeseg(&priv->arbi_timing, false) != OK)
    {
      stm32_setinit(priv->base, 0);
      leave_critical_section(flags);
      return -EIO;
    }

#ifdef DEBUG
  {
    const fdcan_timeseg *tim = &priv->arbi_timing;
    ninfo("[data] Timings: presc=%u sjw=%u bs1=%u bs2=%u\r\n", tim->prescaler, tim->sjw, tim->bs1, tim->bs2);
  }
#endif

  /* Set bit timings and prescalers (Nominal bitrate) */

  regval = ((priv->arbi_timing.sjw << FDCAN_NBTP_NSJW_SHIFT)  |
            (priv->arbi_timing.bs1 << FDCAN_NBTP_NTSEG1_SHIFT) |
            (priv->arbi_timing.bs2 << FDCAN_NBTP_TSEG2_SHIFT)  |
            (priv->arbi_timing.prescaler << FDCAN_NBTP_NBRP_SHIFT));
  putreg32(regval, priv->base + STM32_FDCAN_NBTP_OFFSET);

#ifdef CONFIG_NET_CAN_CANFD
  // CAN-FD Data phase bitrate

  if (stm32_bitratetotimeseg(&priv->data_timing, true) != OK)
    {
      stm32_setinit(priv->base, 0);
      leave_critical_section(flags);
      return -EIO;
    }

#ifdef DEBUG
  const fdcan_timeseg *tim = &priv->data_timing;
  ninfo("[data] Timings: presc=%u sjw=%u bs1=%u bs2=%u\r\n", tim->prescaler, tim->sjw, tim->bs1, tim->bs2);
#endif

  /* Set bit timings and prescalers (Data bitrate) */

  regval = ((priv->data_timing.sjw << FDCAN_DBTP_DSJW_SHIFT)  |
            (priv->data_timing.bs1 << FDCAN_DBTP_DTSEG1_SHIFT) |
            (priv->data_timing.bs2 << FDCAN_DBTP_DTSEG2_SHIFT)  |
            (priv->data_timing.prescaler << FDCAN_DBTP_DBRP_SHIFT));
#endif // NET_CAN_CANFD

  // Be sure to fill data-phase register even if we're not using CAN FD
  putreg32(regval, priv->base + STM32_FDCAN_DBTP_OFFSET);

  /*
   * Operation Configuration
   */

#ifdef CONFIG_NET_CAN_LOOPBACK
  /* Enable External Loopback Mode (Rx pin disconnected) (RM0433 pg 2494) */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_TEST);
  modifyreg32(priv->base + STM32_FDCAN_TEST_OFFSET, 0, FDCAN_TEST_LBCK);
#endif

#ifdef CONFIG_NET_CAN_SILENT
  /* Enable Bus Monitoring or Restricted Operation Mode (RM0433 pg 2492)*/
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_MON);
#endif

#ifdef CONFIG_NET_CAN_CANFD
  // Enable CAN-FD communication
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_FDOE);
  if (priv->arbi_timing.bitrate != priv->data_timing.bitrate)
    {
      // Enable bitrate switching if requested via config
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_BRSE);
    }
#else
  // Disable CAN-FD communications ("classic" CAN only)
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_FDOE, 0);
#endif

  // Disable Automatic Retransmission of frames upon error
  // NOTE: This will even disable automatic retry due to lost arbitration!!
#if 0
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_DAR);
#endif

  // Disable Time Triggered (TT) operation -- TODO (must use TTCAN_TypeDef)
  //ttcan_->TTOCF &= ~FDCAN_TTOCF_OM

  /*
   * Configure Interrupts
   */

  // Clear all interrupt flags
  // Note: A flag is cleared by writing a 1 to the corresponding bit position
  putreg32(FDCAN_IR_MASK, priv->base + STM32_FDCAN_IR_OFFSET);

  // Enable relevant interrupts
  regval = FDCAN_IE_TCE     // Transmit Complete
        //  | FDCAN_IE_PEAE    // Protocol Error Arbitration Phase  /// DEBUGGING
        //  | FDCAN_IE_PEDE    // Protocol Error Data Phase  /// DEBUGGING
        //  | FDCAN_IE_MRAFE   // Message RAM Access Failure  /// DEBUGGING
        //  | FDCAN_IE_TOOE    // Time Out Occurred  /// DEBUGGING
        //  | FDCAN_IE_EWE     // Warning Status  /// DEBUGGING
        //  | FDCAN_IE_EPE     // Error Passive  /// DEBUGGING
        //  | FDCAN_IE_ELOE    // Error Logging Overflow  /// DEBUGGING
         | FDCAN_IE_RF0NE   // Rx FIFO 0 new message
         | FDCAN_IE_RF0FE   // Rx FIFO 0 FIFO full
         | FDCAN_IE_RF1NE   // Rx FIFO 1 new message
         | FDCAN_IE_RF1FE;  // Rx FIFO 1 FIFO full
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

  // Keep Rx interrupts on Line 0; move Tx to Line 1
  // TC (Tx Complete) interrupt on line 1
  regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  regval |= FDCAN_ILS_TCL;
  putreg32(FDCAN_ILS_TCL, priv->base + STM32_FDCAN_ILS_OFFSET);

  // Enable Tx buffer transmission interrupt
  putreg32(FDCAN_TXBTIE_TIE, priv->base + STM32_FDCAN_TXBTIE_OFFSET);

  // Note: You must still call stm32_enable_interrupts() to set ILE
  // (interrupt line enable)

  dumpregs(priv);

  /*
   * Configure Message RAM
   *
   * The available 2560 words (10 kiB) of RAM are shared between both FDCAN
   * interfaces. It is up to us to ensure each interface has its own non-
   * overlapping region of RAM assigned to it by properly assignin the start and
   * end addresses for all regions of RAM.
   *
   * We will give each interface half of the available RAM.
   *
   * Rx buffers are only used in conjunction with acceptance filters; we don't
   * have any specific need for this, so we will only use Rx FIFOs.
   *
   * Each FIFO can hold up to 64 elements, where each element (for a classic CAN
   * 2.0B frame) is up to 4 words long (8 bytes data + header bits)
   *
   * Let's make use of the full 64 FIFO elements for FIFO0.  We have no need to
   * separate messages between FIFO0 and FIFO1, so ignore FIFO1 for simplicity.
   *
   * Note that the start addresses given to FDCAN are in terms of _words_, not
   * bytes, so when we go to read/write to/from the message RAM, there will be a
   * factor of 4 necessary in the address relative to the SA register values.
   */

  // Location of this interface's message RAM - address in CPU memory address
  // and relative address (in words) used for configuration
  const uint32_t iface_ram_base = (2560 / 2) * priv->iface_idx;
  const uint32_t gl_ram_base = STM32_CANRAM_BASE;
  uint32_t ram_offset = iface_ram_base;

  /* Standard ID Filters: Allow space for 128 filters (128 words) */

  const uint8_t n_stdid = 128;
  priv->message_ram.filt_stdid_addr = gl_ram_base + ram_offset * WORD_LENGTH;

  regval  = (n_stdid << FDCAN_SIDFC_LSS_SHIFT) & FDCAN_SIDFC_LSS_MASK;
  regval |= ram_offset << FDCAN_SIDFC_FLSSA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_SIDFC_OFFSET);
  ram_offset += n_stdid;

  /* Extended ID Filters: Allow space for 128 filters (128 words) */

  const uint8_t n_extid = 128;
  priv->message_ram.filt_extid_addr = gl_ram_base + ram_offset * WORD_LENGTH;

  regval = (n_extid << FDCAN_XIDFC_LSE_SHIFT) & FDCAN_XIDFC_LSE_MASK;
  regval |= ram_offset << FDCAN_XIDFC_FLESA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_XIDFC_OFFSET);
  ram_offset += n_extid;

  // Set size of each element in the Rx/Tx buffers and FIFOs
#ifdef CONFIG_NET_CAN_CANFD
  // Set full 64 byte space for every Rx/Tx FIFO element
  modifyreg32(priv->base + STM32_FDCAN_RXESC_OFFSET, 0, FDCAN_RXESC_RBDS); // Rx Buffer
  modifyreg32(priv->base + STM32_FDCAN_RXESC_OFFSET, 0, FDCAN_RXESC_F0DS); // Rx FIFO 0
  modifyreg32(priv->base + STM32_FDCAN_RXESC_OFFSET, 0, FDCAN_RXESC_F1DS); // Rx FIFO 1
  modifyreg32(priv->base + STM32_FDCAN_TXESC_OFFSET, 0, FDCAN_TXESC_TBDS); // Tx Buffer
#else
  putreg32(0, priv->base + STM32_FDCAN_RXESC_OFFSET);  // 8 byte space for every element (Rx buffer, FIFO1, FIFO0)
  putreg32(0, priv->base + STM32_FDCAN_TXESC_OFFSET);  // 8 byte space for every element (Tx buffer)
#endif

  priv->message_ram.n_rxfifo0 = NUM_RX_FIFO0;
  priv->message_ram.n_rxfifo1 = NUM_RX_FIFO1;
  priv->message_ram.n_txfifo = NUM_TX_FIFO;

  /* Assign Rx Mailbox pointer in the driver structure */

  priv->message_ram.rxfifo0_addr = gl_ram_base + ram_offset * WORD_LENGTH;
  priv->rx = (struct rx_fifo_s *)(priv->message_ram.rxfifo0_addr);

  // Set Rx FIFO0 size (64 elements max)
  regval = (ram_offset << FDCAN_RXF0C_F0SA_SHIFT) & FDCAN_RXF0C_F0SA_MASK;
  regval |= (NUM_RX_FIFO0 << FDCAN_RXF0C_F0S_SHIFT) & FDCAN_RXF0C_F0S_MASK;
  putreg32(regval, priv->base + STM32_FDCAN_RXF0C_OFFSET);
  ram_offset += NUM_RX_FIFO0 * FIFO_ELEMENT_SIZE;

  /* Not using Rx FIFO1 */

  /* Assign Tx Mailbox pointer in the driver structure */

  priv->message_ram.txfifo_addr = gl_ram_base + ram_offset * WORD_LENGTH;
  priv->tx = (struct tx_fifo_s *)(priv->message_ram.txfifo_addr);

  // Set Tx FIFO size (32 elements max)
  regval = (NUM_TX_FIFO << FDCAN_TXBC_TFQS_SHIFT) & FDCAN_TXBC_TFQS_MASK;
  regval &= ~FDCAN_TXBC_TFQM;  // Use FIFO
  regval |= (ram_offset << FDCAN_TXBC_TBSA_SHIFT) & FDCAN_TXBC_TBSA_MASK;
  putreg32(regval, priv->base + STM32_FDCAN_TXBC_OFFSET);

  /*
   * Default filter configuration
   *
   * Accept all messages into Rx FIFO0 by default
   */
  regval = getreg32(priv->base + STM32_FDCAN_GFC_OFFSET);
  regval &= ~FDCAN_GFC_ANFS;  // Accept non-matching stdid frames into FIFO0
  regval &= ~FDCAN_GFC_ANFE;  // Accept non-matching extid frames into FIFO0
  putreg32(regval, priv->base + STM32_FDCAN_GFC_OFFSET);

  dumpregs(priv);

  /*
   * Exit Initialization mode
   */
  stm32_setinit(priv->base, 0);

  dumpregs(priv);

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Function: stm32_reset
 *
 * Description:
 *   Put the device in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private FDCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device has previously been initialized, including message RAM
 ****************************************************************************/

static void stm32_reset(struct stm32_driver_s *priv)
{
  /* Request Init Mode */

  irqstate_t flags = enter_critical_section();

  stm32_setenable(priv->base, 1);
  stm32_setinit(priv->base, 1);

  /* Enable Configuration Change Mode */

  stm32_setconfig(priv->base, 1);

  /* Disable Interrupts */

  stm32_disable_interrupts(priv);

  // Clear all interrupt flags
  // Note: A flag is cleared by writing a 1 to the corresponding bit position
  putreg32(FDCAN_IR_MASK, priv->base + STM32_FDCAN_IR_OFFSET);

  /* Reset all message RAM mailboxes; RX and TX */

  if (priv->rx == NULL || priv->tx == NULL)
    {
      nerr("reset requies prior initialization\n");
      leave_critical_section(flags);
      return;
    }

#ifdef CONFIG_NET_CAN_CANFD
  const uint8_t n_data_words = 16;
#else
  const uint8_t n_data_words = 2;
#endif

  for (uint32_t i = 0; i < NUM_RX_FIFO0; i++)
    {
      #ifdef DEBUG
      ninfo("MB RX %i %p\r\n", i, &priv->rx[i]);
      #endif
      priv->rx[i].header.w1 = 0x0;
      priv->rx[i].header.w0 = 0x0;
      for (uint8_t j = 0; j < n_data_words; j++)
      {
        priv->rx[i].data[j].word = 0x0;
      }
    }

  for (uint32_t i = 0; i < NUM_TX_FIFO; i++)
    {
      #ifdef DEBUG
      ninfo("MB TX %i %p\r\n", i, &priv->tx[i]);
      #endif
      priv->tx[i].header.w1 = 0x0;
      priv->tx[i].header.w0 = 0x0;
      for (uint8_t j = 0; j < n_data_words; j++)
      {
        priv->tx[i].data[j].word = 0x0;
      }
    }

  /* Power off the device -- See RM0433 pg 2493 */

  stm32_setinit(priv->base, 0);
  // stm32_setenable(priv->base, 0); /// TODO: Does this reset our config?

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_caninitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple transceivers, this value
 *          identifies which transceiver is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_caninitialize(int intf)
{
  struct stm32_driver_s *priv;

  switch (intf)
    {
#ifdef CONFIG_STM32H7_FDCAN1
    case 0:
      priv             = &g_fdcan0;
      memset(priv, 0, sizeof(struct stm32_driver_s));
      priv->base       = STM32_FDCAN1_BASE;
      priv->iface_idx  = 0;
      priv->config     = &stm32_fdcan0_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FDCAN1_ARBI_BITRATE;
      priv->data_timing.bitrate = CONFIG_FDCAN1_DATA_BITRATE;
#  else
      priv->arbi_timing.bitrate = CONFIG_FDCAN1_BITRATE;
#  endif
      break;
#endif

#ifdef CONFIG_STM32H7_FDCAN2
    case 1:
      priv             = &g_fdcan1;
      memset(priv, 0, sizeof(struct stm32_driver_s));
      priv->base       = STM32_FDCAN2_BASE;
      priv->iface_idx  = 1;
      priv->config     = &stm32_fdcan1_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FDCAN2_ARBI_BITRATE;
      priv->data_timing.bitrate = CONFIG_FDCAN2_DATA_BITRATE;
#  else
      priv->arbi_timing.bitrate = CONFIG_FDCAN2_BITRATE;
#  endif
      break;
#endif

    default:
      return -ENODEV;
    }

  if (stm32_bitratetotimeseg(&priv->arbi_timing, false) != OK)
    {
      printf("ERROR: Invalid CAN timings\n");
      return -1;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (stm32_bitratetotimeseg(&priv->data_timing, true) != OK)
    {
      printf("ERROR: Invalid CAN data phase timings\n");
      return -1;
    }
#endif

  /* Configure the pins we're using to interface to the controller */

  stm32_configgpio(priv->config->tx_pin);
  stm32_configgpio(priv->config->rx_pin);

  /* Attach the fdcan interrupt handlers */

  if (irq_attach(priv->config->mb_irq[0], stm32_fdcan_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      printf("ERROR: Failed to attach CAN RX IRQ\n");
      return -EAGAIN;
    }

  if (irq_attach(priv->config->mb_irq[1], stm32_fdcan_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      printf("ERROR: Failed to attach CAN TX IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = stm32_ifup;     /* I/F up callback */
  priv->dev.d_ifdown  = stm32_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = stm32_txavail;  /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = stm32_ioctl;    /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = (void *)priv;   /* Used to recover private state from dev */

#ifdef CONFIG_NET_CAN_CANFD
  priv->txdesc = (struct canfd_frame *)&g_tx_pool;
  priv->rxdesc = (struct canfd_frame *)&g_rx_pool;
#else
  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;
#endif

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  stm32_initialize(priv);

  /* Put the interface in the down state (disable interrupts, enter sleep mode) */

  stm32_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  dumpregs(priv);

  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the CAN device interfaces.  If there is more than one device
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, CAN interfaces should be
 *   initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_STM32H7_FDCAN1
  stm32_caninitialize(0);
#endif

#ifdef CONFIG_STM32H7_FDCAN2
  stm32_caninitialize(1);
#endif
}
#endif

#endif /* CONFIG_STM32H7_FDCAN */
