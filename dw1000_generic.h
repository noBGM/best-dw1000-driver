/*
 * DW1000 UWB 收发器驱动 (Generic版本)
 * 
 * 此驱动实现了混合模式，同时支持字符设备和网络设备接口，
 * 满足通用UWB网络通信和高精度测距等特殊应用的需求。
 */

#ifndef __DW1000_GENERIC_H__
#define __DW1000_GENERIC_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>

// --- 通用常量定义 ---
#define DRIVER_NAME "dw1000_generic_hybrid"
#define DW1000_MAX_FRAME_LEN  1023  // 最大IEEE 802.15.4 UWB帧长度
#define DW1000_DEFAULT_MTU    127   // 默认MTU大小
#define DW1000_HYBRID_HDR_LEN  12   // 混合帧头长度
#define MAX_PENDING_TX        16    // 最大挂起发送请求
#define NUM_RX_DESC           8     // RX描述符数量
#define RING_BUFFER_SIZE      (64*1024) // 64KB环形缓冲区
#define NAPI_WEIGHT           64    // NAPI轮询权重

// --- DW1000寄存器与命令定义 ---
// 寄存器ID
#define RG_DEV_ID       0x00  // 设备ID寄存器
#define RG_SYS_CFG      0x04  // 系统配置寄存器
#define RG_SYS_CTRL     0x0D  // 系统控制寄存器
#define RG_SYS_STATUS   0x0F  // 系统状态寄存器
#define RG_SYS_MASK     0x0E  // 中断掩码寄存器
#define RG_TX_FCTRL     0x08  // 发送帧控制寄存器
#define RG_TX_BUFFER    0x09  // 发送数据缓冲区
#define RG_RX_FINFO     0x10  // 接收帧信息寄存器
#define RG_RX_BUFFER    0x11  // 接收数据缓冲区
#define RG_TX_POWER     0x1E  // 发射功率控制寄存器
#define RG_CHAN_CTRL    0x1F  // 信道控制寄存器
#define RG_PANADR       0x03  // PAN地址寄存器
#define RG_PMSC         0x36  // 电源管理系统控制寄存器

// --- Register Bits and Masks (Moved from .c) ---
// SYS_CFG Register (0x04)
#define SYS_CFG_RXDBUFFEN      (1 << 0)  // RX Double Buffer Enable (Bit 0)
#define SYS_CFG_FFEN           (1 << 5)  // Frame Filtering Enable (Bit 5)
#define SYS_CFG_FFBC           (1 << 6)  // Frame Filtering Beacon Enable (Bit 6)
#define SYS_CFG_FFAB           (1 << 7)  // Frame Filtering Data Enable (Bit 7)
#define SYS_CFG_FFAA           (1 << 8)  // Frame Filtering ACK Enable (Bit 8)
#define SYS_CFG_FFAM           (1 << 9)  // Frame Filtering MAC Enable (Bit 9)
#define SYS_CFG_PHR_MODE_SHIFT 12
#define SYS_CFG_PHR_MODE_MASK  (0x3 << SYS_CFG_PHR_MODE_SHIFT)
#define SYS_CFG_PHR_MODE_STD   (0x0 << SYS_CFG_PHR_MODE_SHIFT)
#define SYS_CFG_PHR_MODE_EXT   (0x3 << SYS_CFG_PHR_MODE_SHIFT)

// CHAN_CTRL Register (0x1F)
#define CHAN_CTRL_CH_NUM_MASK    0x0F      // Channel Number Mask (Bits 0-3)
#define CHAN_CTRL_PRF_SHIFT      4
#define CHAN_CTRL_PRF_MASK       (0x03 << CHAN_CTRL_PRF_SHIFT) // PRF Mask (Bits 4-5)
#define CHAN_CTRL_RX_PRCODE_MASK 0x1F      // RX Preamble Code Mask (Bits 8-12)
#define CHAN_CTRL_TX_PRCODE_SHIFT 13
#define CHAN_CTRL_TX_PRCODE_MASK (0x1F << CHAN_CTRL_TX_PRCODE_SHIFT) // TX Preamble Code Mask (Bits 13-17)

// TX_FCTRL Register (0x08 - 5 bytes)
#define TX_FCTRL_TXFLEN_MASK   0x03FF    // TX Frame Length (Bits 0-9)
#define TX_FCTRL_TXBR_SHIFT    5         // TX Data Rate Shift (Byte 1, Bits 5-6)
#define TX_FCTRL_TXBR_MASK     (0x03 << TX_FCTRL_TXBR_SHIFT)
#define TX_FCTRL_TR_BIT        (1 << 7)  // TX Ranging Bit (Byte 1, Bit 7)
#define TX_FCTRL_PE_SHIFT      4         // Preamble Length Shift (Byte 2, Bits 4-5)
#define TX_FCTRL_PE_MASK       (0x03 << TX_FCTRL_PE_SHIFT)
#define TX_FCTRL_PE_64         0x00      // PE code for 64 symbols
#define TX_FCTRL_PE_128        0x01      // PE code for 128 symbols
#define TX_FCTRL_PE_256        0x02      // PE code for 256 symbols
#define TX_FCTRL_PE_512        0x03      // PE code for 512 symbols
#define TX_FCTRL_PE_NS_00      0x00      // PE code for non-standard (e.g., 1024, 4096)
#define TX_FCTRL_PE_NS_01      0x01      // PE code for non-standard (e.g., 1536)
#define TX_FCTRL_PE_NS_10      0x02      // PE code for non-standard (e.g., 2048)
#define TX_FCTRL_TXPRF_SHIFT   2         // TX PRF Shift (Byte 2, Bits 2-3)
#define TX_FCTRL_TXPRF_MASK    (0x03 << TX_FCTRL_TXPRF_SHIFT)
#define TX_FCTRL_TXPSR_SHIFT   6         // TX Preamble Symbol Repetitions Shift (Byte 2, Bits 6-7) - Corrected Shift?
#define TX_FCTRL_TXPSR_MASK    (0x03 << TX_FCTRL_TXPSR_SHIFT)
#define TX_FCTRL_TXPSR_STD     0x01      // TXPSR for standard lengths (e.g., 64-512)
#define TX_FCTRL_TXPSR_1024    0x02      // TXPSR for 1024, 1536, 2048 lengths
#define TX_FCTRL_TXPSR_4096    0x03      // TXPSR for 4096 length
#define TX_FCTRL_TXPSR_N_16MHZ 0x01      // TXPSR value for 16MHz PRF with std preamble
#define TX_FCTRL_TXPSR_N_64MHZ 0x01      // TXPSR value for 64MHz PRF with std preamble (usually same for std)
#define RG_TX_FCTRL_LEN        5         // Length of TX_FCTRL register

// SYS_CTRL Register (0x0D)
#define SYS_CTRL_TXSTRT_BIT    (1 << 1)  // TX Start Bit
#define SYS_CTRL_TRXOFF_BIT    (1 << 6)  // Force Transceiver Off Bit
#define SYS_CTRL_RXENAB_BIT    (1 << 8)  // Enable Receiver Bit

// SPI Command Format
#define SPI_CMD_WRITE          0x80      // Write command prefix (bit 7 = 1)
#define SPI_CMD_READ           0x00      // Read command prefix (bit 7 = 0)
#define SPI_CMD_RW_BIT         0x80
#define SPI_CMD_REG_MASK       0x3F      // Register address mask (6 bits, 0-5)
#define SPI_CMD_SUB_EXT_BIT    0x40      // Extended Sub-Address Bit (bit 6)
#define SPI_CMD_SUB_SHORT_MASK 0x7F      // Short Sub-Address Mask (7 bits, 0-6 of byte 1)
#define SPI_CMD_SUB_LONG_MASK  0x7FFF    // Long Sub-Address Mask (15 bits, 0-6 of byte 1, 0-7 of byte 2)
#define SPI_CMD_SUB_EXT1_BIT   0x80      // Bit 7 of byte 1 indicates 2nd sub-address byte follows
#define SPI_CMD_SUB_EXT2_BIT   0x80      // Bit 7 of byte 2 indicates 3rd sub-address byte follows (Not standard DW1000?)

// Frame Control Field (IEEE 802.15.4 - simplified)
#define FRAME_CTRL_TYPE_BEACON (0x0000)
#define FRAME_CTRL_TYPE_DATA   (0x0001)
#define FRAME_CTRL_TYPE_ACK    (0x0002)
#define FRAME_CTRL_TYPE_MAC_CMD (0x0003)
#define FRAME_CTRL_TYPE_MASK   (0x0007)
#define FRAME_CTRL_SEC_EN      (1 << 3)
#define FRAME_CTRL_FRAME_PEND  (1 << 4)
#define FRAME_CTRL_ACK_REQ     (1 << 5)
#define FRAME_CTRL_PANID_COMP  (1 << 6)
#define FRAME_CTRL_DST_ADDR_NONE (0x0 << 10)
#define FRAME_CTRL_DST_ADDR_SHORT (0x2 << 10)
#define FRAME_CTRL_DST_ADDR_EXT  (0x3 << 10)
#define FRAME_CTRL_DST_ADDR_MASK (0x3 << 10)
#define FRAME_CTRL_SRC_ADDR_NONE (0x0 << 14)
#define FRAME_CTRL_SRC_ADDR_SHORT (0x2 << 14)
#define FRAME_CTRL_SRC_ADDR_EXT  (0x3 << 14)
#define FRAME_CTRL_SRC_ADDR_MASK (0x3 << 14)
// Specific combinations
#define FRAME_CTRL_DATA_SHORTADDR_PANIDCOMP (FRAME_CTRL_TYPE_DATA | FRAME_CTRL_PANID_COMP | FRAME_CTRL_DST_ADDR_SHORT | FRAME_CTRL_SRC_ADDR_SHORT) // 0x8841
#define FRAME_CTRL_DATA_EXTADDR_PANIDCOMP   (FRAME_CTRL_TYPE_DATA | FRAME_CTRL_PANID_COMP | FRAME_CTRL_DST_ADDR_EXT | FRAME_CTRL_SRC_ADDR_EXT)     // 0xCC41

// Additional Register Definitions (Moved from .c)
#define RG_EUI          0x01    /* Extended Unique Identifier */
#define RG_SYS_TIME     0x06    /* System Time */
#define RG_DX_TIME      0x0A    /* Delayed Send/Receive Time */
#define RG_RX_FQUAL     0x12    /* Receive Frame Quality Information */
#define RG_AGC_CTRL     0x23    /* Automatic Gain Control */
#define RG_RF_CONF      0x28    /* RF Configuration */
#define RG_TX_CAL       0x2A    /* Transmitter Calibration */
#define RG_FS_CTRL      0x2B    /* Frequency Synthesizer Control */
#define RG_AON          0x2C    /* Always-On Control */
#define RG_OTP_IF       0x2D    /* OTP Memory Interface */
#define RG_LDE_IF       0x2E    /* Leading Edge Detection Interface */
#define RG_DMA_CONF     0x37    /* DMA Configuration */
#define RG_DMA_ADDR     0x38    /* DMA Address Configuration */
#define RG_DMA_STATUS   0x39    /* DMA Status */

// --- Common Sub-Addresses (Moved from .c) ---
#define SUB_PMSC_CTRL0  0x00    /* PMSC Control Register 0 */
#define SUB_PMSC_CTRL1  0x04    /* PMSC Control Register 1 */
#define SUB_SYS_STATUS_ALL 0x00  /* System Status Register All */

// --- System Event Flags (Moved from .c) ---
#define SYS_STATUS_RXFCG        0x00004000UL  // RX Frame Checksum Good (Bit 14)
#define SYS_STATUS_RXPHD        0x00001000UL  // RX PHY Header Detected (Bit 12)
#define SYS_STATUS_RXSFDD       0x00000200UL  // RX SFD Detected (Bit 9)
#define SYS_STATUS_RXPRD        0x00000100UL  // RX Preamble Detected (Bit 8)
#define SYS_STATUS_TXFRS        0x00000080UL  // TX Frame Sent (Bit 7)
#define SYS_STATUS_TXPHS        0x00000040UL  // TX PHY Header Sent (Bit 6)
#define SYS_STATUS_TXPRS        0x00000020UL  // TX Preamble Sent (Bit 5)
#define SYS_STATUS_LDEDONE      0x00000400UL  // LDE Processing Done (Bit 10)
#define SYS_STATUS_CLKPLL_LL    0x02000000UL  // Clock PLL Lock Lost (Bit 25)
#define SYS_STATUS_RFPLL_LL     0x01000000UL  // RF PLL Lock Lost (Bit 24)
#define SYS_STATUS_IRQS         (1UL << 0)    // IRQ Status (Overall flag? Check datasheet)
// Combined masks
#define SYS_STATUS_ALL_RX_GOOD  (SYS_STATUS_RXFCG | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD)
#define SYS_STATUS_ALL_TX       (SYS_STATUS_TXFRS | SYS_STATUS_TXPHS | SYS_STATUS_TXPRS)
#define SYS_STATUS_ALL_RX_ERR   0x007F8000UL // Placeholder for RX Error flags (Need specific flags from datasheet, e.g., PHE, RSE, CRCE, SFDTO, PTO, FTO)
#define SYS_STATUS_ALL_ERR      (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_CLKPLL_LL | SYS_STATUS_RFPLL_LL)
#define SYS_STATUS_ALL_DECA     0xFFFFFFFFUL // Not recommended, use specific masks

// --- System Control Register Bits (SYS_CTRL - 0x0D) (Moved from .c) ---
// #define SYS_CTRL_TXSTRT         (1 << 1) already defined
// #define SYS_CTRL_TRXOFF         (1 << 6) already defined
// #define SYS_CTRL_RXENAB         (1 << 8) already defined
#define SYS_CTRL_SFCST          (1 << 0) // Suppress Frame Check Sequence Transmission
#define SYS_CTRL_TXDLYS         (1 << 2) // TX Delayed Send
#define SYS_CTRL_RXDLYS         (1 << 9) // RX Delayed Receive
#define SYS_CTRL_RXWTE          (1 << 10)// RX Wait Timeout Enable
#define SYS_CTRL_RXWTOE         (1 << 13)// RX Wait Timeout Override Enable (Check datasheet)

// --- DMA Configuration Register Bits (RG_DMA_CONF - 0x37) (Moved from .c) ---
#define DMA_CONF_RXEN           (1 << 0) // Enable RX DMA
#define DMA_CONF_TXEN           (1 << 1) // Enable TX DMA
#define DMA_CONF_OFFSET_SHIFT   8        // Offset shift for DMA buffer offset
#define DMA_CONF_OFFSET_MASK    (0xFF << DMA_CONF_OFFSET_SHIFT) // Mask for DMA buffer offset

// --- 帧类型定义 ---
#define DW1000_FRAME_TYPE_NETWORK  0x01  // 网络数据帧
#define DW1000_FRAME_TYPE_SENSING  0x02  // 传感器数据帧
#define DW1000_FRAME_TYPE_RANGING  0x03  // 测距帧

// --- 混合帧头部定义 (自定义格式) ---
struct dw1000_hybrid_hdr {
    u64 src_addr;     // 源地址 
    u8 frame_type;    // 帧类型
    u8 seq_num;       // 序列号
    u16 reserved;     // 保留字段
} __packed;

// --- 发送帧结构 ---
struct dw1000_tx_frame {
    u16 len;          // 帧长度
    u8 wait_resp;     // 是否等待响应
    u8 tx_mode;       // 发送模式
    u8 data[0];       // 变长数据段
} __packed;

// --- 传感器帧结构 ---
struct dw1000_sensing_frame {
    u64 kernel_timestamp_ns;  // 内核时间戳
    u16 data_len;             // 数据长度
    u8 seq_num;               // 序列号
    u64 src_addr;             // 源地址
    u8 status[8];             // RX Frame Quality (e.g., read from RG_RX_FQUAL, first 8 bytes)
    u8 data[0];               // 变长数据段
} __packed;

// 计算传感器数据条目大小的辅助函数
static inline size_t dw1000_sensing_entry_size(size_t data_len)
{
    // Ensure proper alignment for the start of the next entry
    return ALIGN(offsetof(struct dw1000_sensing_frame, data) + data_len, sizeof(long));
}

// --- DMA接收描述符 ---
struct dw1000_rx_desc {
    struct list_head list;       // 链表节点
    void *buffer;                // 虚拟地址
    dma_addr_t dma_addr;         // 物理地址
    size_t buffer_size;          // 缓冲区大小
    size_t data_len;             // 实际数据长度
    bool in_use;                 // 是否正在使用
};

// --- 设备私有数据结构 ---
struct dw1000_hybrid_priv {
    // 基本设备指针
    struct spi_device *spi;              // SPI设备指针
    struct net_device *netdev;           // 网络设备指针
    
    // 字符设备相关
    struct cdev cdev;                    // 字符设备
    dev_t cdev_devt;                     // 字符设备号
    struct class *cdev_class;            // 字符设备类
    struct mutex ioctl_lock;             // IOCTL互斥锁
    
    // DMA相关
    struct dma_chan *tx_dma_chan;        // 发送DMA通道
    struct dma_chan *rx_dma_chan;        // 接收DMA通道
    bool rx_dma_active;                  // 接收DMA是否活动
    
    // 锁与同步
    spinlock_t config_lock;              // 配置锁
    spinlock_t stats_lock;               // 统计锁
    spinlock_t rx_desc_lock;             // 接收描述符锁
    spinlock_t tx_desc_lock;             // TX描述符锁
    struct mutex spi_reg_mutex;          // SPI寄存器访问互斥锁
    spinlock_t fifo_lock;                // kfifo访问锁
    
    // GPIO控制
    struct gpio_desc *irq_gpio;          // 中断GPIO
    int irq_num;                         // 中断号
    
    // 网络设备相关
    struct napi_struct napi;             // NAPI结构
    
    // 配置与状态
    struct dw1000_hybrid_config config;  // 设备配置
    struct dw1000_hybrid_stats stats;    // 设备统计
    atomic_t tx_pending_count;           // 待处理发送计数
    atomic_t tx_sequence_num;            // 发送序列号计数器
    
    // 环形缓冲区 (字符设备接口的传感数据) - kfifo based
    wait_queue_head_t read_wq;           // 读取等待队列
    struct kfifo sensing_fifo;           // kfifo 结构
    void *sensing_buffer;                // vmalloc 分配的缓冲区指针
    size_t sensing_buffer_size;          // 缓冲区大小
    
    // 接收描述符
    struct dw1000_rx_desc rx_desc[NUM_RX_DESC]; // 接收描述符数组
    struct list_head rx_free_list;       // 空闲描述符链表
    struct list_head rx_pending_list;    // 待处理描述符链表

    // 发送描述符
    struct dw1000_tx_desc tx_desc[NUM_TX_DESC];  // TX descriptor array
    struct list_head tx_free_list;     // Free TX descriptors
    struct list_head tx_pending_list;  // Submitted TX descriptors to DMA

    // SPI通信缓冲
    u8 spi_buf[8];                       // SPI transaction frame header buffer
};

// --- 函数声明 ---
// SPI通信
int dw1000_read_reg(struct dw1000_hybrid_priv *priv, u8 reg, u16 offset, u16 len, void *buf);
int dw1000_write_reg(struct dw1000_hybrid_priv *priv, u8 reg, u16 offset, u16 len, const void *buf);

// 硬件控制
int dw1000_reset_device(struct dw1000_hybrid_priv *priv);
int dw1000_configure_device(struct dw1000_hybrid_priv *priv);
int dw1000_tx_test(struct dw1000_hybrid_priv *priv, int count);
int dw1000_rx_test(struct dw1000_hybrid_priv *priv, int duration_ms);
int dw1000_send_frame(struct dw1000_hybrid_priv *priv, struct dw1000_tx_frame *frame);

// 网络设备操作
int dw1000_hybrid_net_open(struct net_device *dev);
int dw1000_hybrid_net_stop(struct net_device *dev);
int dw1000_hybrid_net_start_xmit(struct sk_buff *skb, struct net_device *dev);

// 中断和NAPI
irqreturn_t dw1000_hybrid_irq_handler(int irq, void *dev_id);
int dw1000_hybrid_poll(struct napi_struct *napi, int budget);

// 帧处理
int process_rx_frame(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc);


// 帧类型标识符
#define DW1000_FRAME_TYPE_NET     (0) // Network frame (via netdev)
#define DW1000_FRAME_TYPE_SENSING (1) // Sensing data frame (via chardev->mmap)
#define DW1000_FRAME_TYPE_RANGING (2) // Ranging data frame (via chardev)
#define DW1000_FRAME_TYPE_CONFIG  (3) // Configuration command frame (via chardev)

// DMA 配置
#define NUM_RX_DESC         (16)
#define RX_DESC_BUFFER_SIZE (2048)
#define NUM_TX_DESC         (16)
#define TX_DESC_BUFFER_SIZE (2048)

// 发送回调上下文结构体
struct dw1000_tx_cb_context {
    struct dw1000_hybrid_priv *priv;
    dma_addr_t dma_handle;
    size_t len;
    struct sk_buff *skb;     // SKB 指针，用于网络设备层释放
};

// 传感器帧结构体
struct dw1000_sensing_frame {
    u64 kernel_timestamp_ns;
    u16 data_len; // Length of the sensing payload ONLY
    u8 status[4]; // RX status
    u8 seq_num;
    u64 src_addr;
    u8 data[];
};

// 辅助函数：计算传感器数据条目大小
static inline size_t dw1000_sensing_entry_size(u16 data_len) {
    return ALIGN(offsetof(struct dw1000_sensing_frame, data) + data_len, sizeof(long));
}

#endif /* __DW1000_GENERIC_H__ */ 