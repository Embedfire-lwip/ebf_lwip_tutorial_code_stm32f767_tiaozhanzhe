/**
  ******************************************************************************
  * @file    lwipopts.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   lwIP Options Configuration.
  *          This file is based on Utilities\lwip_v1.4.1\src\include\lwip\opt.h 
  *          and contains the lwIP configuration for the STM32F4x7 demonstration.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/*********************************************************************************************************
** Compiler related configuration
*********************************************************************************************************/
//#define LWIP_NO_STDINT_H        1
#define BYTE_ORDER              LITTLE_ENDIAN


/**
 * NO_SYS==1: Provides VERY minimal functionality. Otherwise,
 * use lwIP facilities.
 */
#define NO_SYS                  0
#define LWIP_TIMERS                     !NO_SYS        // 使用lwip内部提供的定时功能
#define LWIP_TIMERS_CUSTOM              !LWIP_TIMERS  //　用户提供定时功能

#define LWIP_RAND                      rand

/*********************************************************************************************************
** Core locking
*********************************************************************************************************/
#define LWIP_MPU_COMPATIBLE             0            // 如果有MPU支持，可以使能该定义
#define LWIP_TCPIP_CORE_LOCKING         1            // lwip内核锁
#define LWIP_TCPIP_CORE_LOCKING_INPUT   0            // 接收锁

/**
 * SYS_LIGHTWEIGHT_PROT==1: if you want inter-task protection for certain
 * critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT    (NO_SYS==0) 

/**
 * NO_SYS_NO_TIMERS==1: Drop support for sys_timeout when NO_SYS==1
 * Mainly for compatibility to old versions.
 */
#define NO_SYS_NO_TIMERS        0

/* ---------- Memory options ---------- */
/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT           4

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE                (25*1024)

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#define MEMP_NUM_PBUF           25
/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
   per active UDP "connection". */
#define MEMP_NUM_UDP_PCB        4
/* MEMP_NUM_TCP_PCB: the number of simulatenously active TCP
   connections. */
#define MEMP_NUM_TCP_PCB        6
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP
   connections. */
#define MEMP_NUM_TCP_PCB_LISTEN 6
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP
   segments. */
#define MEMP_NUM_TCP_SEG        120

#define MEMP_NUM_RAW_PCB                6
#define MEMP_NUM_ALTCP_PCB              MEMP_NUM_TCP_PCB
#define MEMP_NUM_REASSDATA              5   //每个重组包允许的最大分配数量
#define MEMP_NUM_FRAG_PBUF              15  //每个分片包允许的最大分配数量
#define MEMP_NUM_ARP_QUEUE              30
#define MEMP_NUM_IGMP_GROUP             8

//　timeout定时器数量定义
#define LWIP_NUM_SYS_TIMEOUT_INTERNAL_USE   (LWIP_TCP + IP_REASSEMBLY + LWIP_ARP + (2*LWIP_DHCP) + \
                                         LWIP_AUTOIP + LWIP_IGMP + LWIP_DNS + PPP_NUM_TIMEOUTS + \
                                         (LWIP_IPV6 * (1 + LWIP_IPV6_REASS + LWIP_IPV6_MLD))+\
                                         + 16)
#define MEMP_NUM_SYS_TIMEOUT            LWIP_NUM_SYS_TIMEOUT_INTERNAL_USE
#define MEMP_NUM_NETBUF                 24
#define MEMP_NUM_NETCONN                8
#define MEMP_NUM_SELECT_CB              8
#define MEMP_NUM_TCPIP_MSG_API          16
#define MEMP_NUM_TCPIP_MSG_INPKT        16
#define MEMP_NUM_NETDB                  3
#define MEMP_NUM_LOCALHOSTLIST          2

/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE          32
/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */
#define PBUF_POOL_BUFSIZE       LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_ENCAPSULATION_HLEN+PBUF_LINK_HLEN)


/*********************************************************************************************************
** IP协议参数的设定——IP options
*********************************************************************************************************/
#define LWIP_IPV4                       1
#define IP_FORWARD                      0
#define IP_REASSEMBLY                   1
#define IP_FRAG                         1
#define IP_OPTIONS_ALLOWED              1

#define IP_REASS_MAXAGE                 15
#define IP_REASS_MAX_PBUFS              10
#define IP_DEFAULT_TTL                  255
#define IP_SOF_BROADCAST                0
#define IP_SOF_BROADCAST_RECV           0
// aodvh和6lowpan rpl路由协议需要定义为1
#define IP_FORWARD_ALLOW_TX_ON_RX_NETIF 0

/*********************************************************************************************************
** ICMP协议参数的设定——ICMP options
*********************************************************************************************************/
#define LWIP_ICMP                       1
//#define ICMP_TTL                       (IP_DEFAULT_TTL)
//#define LWIP_BROADCAST_PING             0
//#define LWIP_MULTICAST_PING             0

/*********************************************************************************************************
** AUTOIP参数配置——AUTOIP options
*********************************************************************************************************/
#define LWIP_AUTOIP                     0
#define LWIP_DHCP_AUTOIP_COOP           0
#define LWIP_DHCP_AUTOIP_COOP_TRIES     9

/*********************************************************************************************************
** IGMP参数配置——IGMP options
*********************************************************************************************************/
#define LWIP_IGMP                       1

/* ---------- TCP options ---------- */
#define LWIP_TCP                1
#define TCP_TTL                 255

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ         0

/* TCP Maximum segment size. */
#define TCP_MSS                 (1500 - 40)	  /* TCP_MSS = (Ethernet MTU - IP header size - TCP header size) */

/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             (6*TCP_MSS)

/*  TCP_SND_QUEUELEN: TCP sender buffer space (pbufs). This must be at least
  as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work. */

#define TCP_SND_QUEUELEN        (8* TCP_SND_BUF/TCP_MSS)

/* TCP receive window. */
#define TCP_WND                 (14*TCP_MSS)

#define LWIP_EVENT_API                  0
#define LWIP_CALLBACK_API               1

#define LWIP_WND_SCALE                  0
#define TCP_RCV_SCALE                   0
#define LWIP_ALTCP                      0
#define LWIP_ALTCP_TLS                  0

/* ---------- DHCP options ---------- */
/* Define LWIP_DHCP to 1 if you want DHCP configuration of
   interfaces. DHCP is not implemented in lwIP 0.5.1, however, so
   turning this on does currently not work. */
#define LWIP_DHCP               1


/* ---------- UDP options ---------- */
#define LWIP_UDP                1
#define UDP_TTL                 255


/* ---------- Statistics options ---------- */
#define LWIP_STATS 0
#define LWIP_PROVIDE_ERRNO 1

/* ---------- link callback options ---------- */
/* LWIP_NETIF_LINK_CALLBACK==1: Support a callback function from an interface
 * whenever the link changes (i.e., link down)
 */
#define LWIP_NETIF_LINK_CALLBACK        0


/*********************************************************************************************************
** Sequential API参数设定——Sequential layer options
*********************************************************************************************************/
#define LWIP_NETCONN                    1
//#define LWIP_TCPIP_TIMEOUT              1
//#define LWIP_NETCONN_SEM_PER_THREAD     0
//#define LWIP_NETCONN_FULLDUPLEX         0


/*********************************************************************************************************
** Socket参数设定——Socket Options
*********************************************************************************************************/
#define LWIP_SOCKET                     1
//#define LWIP_COMPAT_SOCKETS             0
//#define LWIP_POSIX_SOCKETS_IO_NAMES     0
#define LWIP_SOCKET_OFFSET              0
#define LWIP_TCP_KEEPALIVE              1
#define LWIP_SO_SNDTIMEO                1
#define LWIP_SO_RCVTIMEO                1
//#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD 0
#define LWIP_SO_RCVBUF                  1
#define LWIP_SO_LINGER                  1
//#define RECV_BUFSIZE_DEFAULT            INT_MAX
//#define LWIP_TCP_CLOSE_TIMEOUT_MS_DEFAULT 20000
#define SO_REUSE                        1
#define SO_REUSE_RXTOALL                1
//#define LWIP_FIONREAD_LINUXMODE         0
#define LWIP_SOCKET_SELECT              1
#define LWIP_SOCKET_POLL                0

/*
   ---------------------------------
   ---------- OS options ----------
   ---------------------------------
*/
#define DEFAULT_UDP_RECVMBOX_SIZE       10
#define DEFAULT_TCP_RECVMBOX_SIZE       10
#define DEFAULT_ACCEPTMBOX_SIZE         10
#define DEFAULT_THREAD_STACKSIZE        1024


#define TCPIP_THREAD_NAME              "lwip"
#define TCPIP_THREAD_STACKSIZE          1024
#define TCPIP_MBOX_SIZE                 8
#define TCPIP_THREAD_PRIO               9

/*********************************************************************************************************
** Statistics options
*********************************************************************************************************/
#define LWIP_STATS                      0

#if LWIP_STATS

#define LWIP_STATS_DISPLAY              0
#define LINK_STATS                      1
#define ETHARP_STATS                    (LWIP_ARP)
#define IP_STATS                        1
#define IPFRAG_STATS                    (IP_REASSEMBLY || IP_FRAG)
#define ICMP_STATS                      1
#define IGMP_STATS                      (LWIP_IGMP)
#define UDP_STATS                       (LWIP_UDP)
#define TCP_STATS                       (LWIP_TCP)
#define MEM_STATS                       ((MEM_LIBC_MALLOC == 0) && (MEM_USE_POOLS == 0))
#define MEMP_STATS                      (MEMP_MEM_MALLOC == 0)
#define SYS_STATS                       (NO_SYS == 0)
#define IP6_STATS                       (LWIP_IPV6)
#define ICMP6_STATS                     (LWIP_IPV6 && LWIP_ICMP6)
#define IP6_FRAG_STATS                  (LWIP_IPV6 && (LWIP_IPV6_FRAG || LWIP_IPV6_REASS))
#define MLD6_STATS                      (LWIP_IPV6 && LWIP_IPV6_MLD)
#define ND6_STATS                       (LWIP_IPV6)
#define MIB2_STATS                      0

#else

#define LINK_STATS                      0
#define ETHARP_STATS                    0
#define IP_STATS                        0
#define IPFRAG_STATS                    0
#define ICMP_STATS                      0
#define IGMP_STATS                      0
#define UDP_STATS                       0
#define TCP_STATS                       0
#define MEM_STATS                       0
#define MEMP_STATS                      0
#define SYS_STATS                       0
#define LWIP_STATS_DISPLAY              0
#define IP6_STATS                       0
#define ICMP6_STATS                     0
#define IP6_FRAG_STATS                  0
#define MLD6_STATS                      0
#define ND6_STATS                       0
#define MIB2_STATS                      0

#endif /* LWIP_STATS */

/*********************************************************************************************************
**  校验和参数配置——checksum options
*********************************************************************************************************/
#define LWIP_CHECKSUM_CTRL_PER_NETIF    0
#define CHECKSUM_BY_HARDWARE            0

#if (CHECKSUM_BY_HARDWARE > 0)
#define CHECKSUM_GEN_IP                 0
#define CHECKSUM_GEN_UDP                0
#define CHECKSUM_GEN_TCP                0
#define CHECKSUM_GEN_ICMP               0
#define CHECKSUM_GEN_ICMP6              0
#define CHECKSUM_CHECK_IP               0
#define CHECKSUM_CHECK_UDP              0
#define CHECKSUM_CHECK_TCP              0
#define CHECKSUM_CHECK_ICMP             0
#define CHECKSUM_CHECK_ICMP6            0
#else
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_GEN_ICMP6              1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1
#define CHECKSUM_CHECK_ICMP6            1
#endif

#define LWIP_CHECKSUM_ON_COPY           0

/*********************************************************************************************************
** PPP options
*********************************************************************************************************/
#define PPP_SUPPORT                     0

//#define PPPOE_SUPPORT                   0
//#define PPPOL2TP_SUPPORT                0
#define PPPOS_SUPPORT                   PPP_SUPPORT

#define PPP_USE_PBUF_RAM                1     // ppp内存分配从堆中分配。ppp缓存必须是一个缓冲区，不能链式

//#define PPP_INPROC_IRQ_SAFE             0

#define PAP_SUPPORT                 1      /* Set > 0 for PAP. */
#define CHAP_SUPPORT                1      /* Set > 0 for CHAP. */
#define MSCHAP_SUPPORT              1      /* Set > 0 for MSCHAP. */
#define EAP_SUPPORT                 1      /* Set > 0 for EAP */
#define CCP_SUPPORT                 1      /* Set > 0 for CCP */
#define MPPE_SUPPORT                1      /* Set > 0 for CBCP  */
#define CBCP_SUPPORT                0      /* CURRENTLY NOT SUPPORTED! DO NOT SET! */
#define ECP_SUPPORT                 0      /* CURRENTLY NOT SUPPORTED! DO NOT SET! */
//#define DEMAND_SUPPORT              0      /* CURRENTLY NOT SUPPORTED! DO NOT SET! */
//#define LQR_SUPPORT                 0
#define VJ_SUPPORT                  1      /* Set > 0 for VJ header compression. */


// FSM超时参数配置
#define FSM_DEFTIMEOUT          10      // FSM_DEFTIMEOUT: Timeout time in seconds
#define FSM_DEFMAXTERMREQS      5       // FSM_DEFMAXTERMREQS: Maximum Terminate-Request transmissions
#define FSM_DEFMAXCONFREQS      10      // FSM_DEFMAXCONFREQS: Maximum Configure-Request transmissions
#define FSM_DEFMAXNAKLOOPS      5      // FSM_DEFMAXNAKLOOPS: Maximum number of nak loops
#define UPAP_DEFTIMEOUT         10      // UPAP_DEFTIMEOUT: Timeout (seconds) for retransmitting req
#define UPAP_DEFTRANSMITS       10      //Maximum number of auth-reqs to send


#define PRINTPKT_SUPPORT            0


/*********************************************************************************************************
** IP Route功能
*********************************************************************************************************/
#define LWIP_ROUTE                      0     // 使能支持路由表功能
#if LWIP_ROUTE
#define LWIP_HOOK_IP4_ROUTE(a)                  route_ip4_find((ip4_addr_t *)a)          
#define LWIP_HOOK_IP6_ROUTE(IP6_ADDR_ANY6, a)   route_ip6_find((ip6_addr_t *)a)
#endif

/*********************************************************************************************************
** DHCP服务器配置
*********************************************************************************************************/
#define LWIP_DHCP_SERVER                0
#define DHCP_OPTIONS_LEN                300  // 必须配置足够发送DHCP消息

/*********************************************************************************************************
** RPL路由协议使能配置, 必须同时使能IPv6
*********************************************************************************************************/
#define LWIP_RPL                        0
#define RPL_TIMER_PRECISION             40
#define RPL_STATS                       0
#define RPL_DEFAULT_LIFETIME_UNIT       20
#define RPL_DEFAULT_LIFETIME            60
//#define RPL_DAG_MC                      RPL_DAG_MC_NONE
//#define RPL_MOP_DEFAULT                 RPL_MOP_STORING_NO_MULTICAST
//#define RPL_RPLIF_DEBUG                 LWIP_DBG_ON
//#define RPL_TIMERCB_DEBUG               LWIP_DBG_ON
//#define RPL_ROUTE_DEBUG                 LWIP_DBG_ON
//#define RPL_PROTO_DEBUG                 LWIP_DBG_ON
//#define RPL_PARENT_DEBUG                LWIP_DBG_ON
//#define RPL_OF_DEBUG                    LWIP_DBG_ON
//#define RPL_DODAG_DEBUG                 LWIP_DBG_ON
//#define RPL_ROUTE_PRINT                  LWIP_DBG_ON
//#define RPL_PARENT_PRINT                 LWIP_DBG_ON

/*********************************************************************************************************
** nat配置,必须支持IPv4
*********************************************************************************************************/
#define LWIP_NAT                   0
#if (LWIP_NAT > 0)
#define LWIP_NAT_MALLOC            rt_malloc
#define LWIP_NAT_FREE              rt_free
#define NAT_MAP_PORT_START         10000
#define NAT_MAP_PORT_END           60000
#define NAT_ICMP_TTL               2
#define NAT_UDP_TTL                18         
#define NAT_SYN_TTL                2          
#define NAT_RST_TTL                2           
#define NAT_TCP_TTL                18    
#endif

/*********************************************************************************************************
** naptpt配置,必须同时支持IPv4和IPv6
*********************************************************************************************************/
#define LWIP_NAPTPT                  0
#if (LWIP_NAPTPT > 0)
#define NAPTPT_ICMP_TTL              15
#define NAPTPT_UDP_TTL               60*3          
#define NAPTPT_SYN_TTL               15          
#define NAPTPT_RST_TTL               15           
#define NAPTPT_TCP_TTL               60*3    
#endif

#endif /* __LWIPOPTS_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

