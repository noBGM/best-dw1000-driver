#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/skbuff.h>
#include <linux/kfifo.h>
#include <linux/types.h>
#include <linux/netdevice.h>
#include "dw1000_generic.h"

// Process sensing frame
static int process_sensing_frame(struct dw1000_hybrid_priv *priv, void *frame_data, size_t frame_len)
{
    struct dw1000_hybrid_hdr *hdr = frame_data;
    ktime_t now = ktime_get_real();
    u64 timestamp_ns = ktime_to_ns(now);
    u16 payload_len = frame_len - DW1000_HYBRID_HDR_LEN;
    struct dw1000_sensing_frame entry_hdr;
    unsigned int bytes_copied;
    unsigned long flags;
    int spi_ret;

    // Fill sensing frame header
    entry_hdr.kernel_timestamp_ns = timestamp_ns;
    entry_hdr.data_len = payload_len;
    
    // Get reception quality info from hardware register
    spi_ret = dw1000_read_reg(priv, RG_RX_FQUAL, 0, sizeof(entry_hdr.status), entry_hdr.status);
    if (spi_ret != 0) {
        dev_warn_ratelimited(&priv->spi->dev, "Failed to read RX_FQUAL: %d, status zeroed\n", spi_ret);
        memset(entry_hdr.status, 0, sizeof(entry_hdr.status));
    }
    
    entry_hdr.seq_num = hdr->seq_num;
    entry_hdr.src_addr = le64_to_cpu(hdr->src_addr);

    // Write header to FIFO
    bytes_copied = kfifo_in_spinlocked(&priv->sensing_fifo,
                                     &entry_hdr,
                                     offsetof(struct dw1000_sensing_frame, data),
                                     &priv->fifo_lock);

    if (bytes_copied != offsetof(struct dw1000_sensing_frame, data)) {
        dev_warn_ratelimited(&priv->spi->dev, "kfifo full (header), dropping sensing frame\n");
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_ring++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        return -ENOSPC;
    }

    // Write payload to FIFO
    bytes_copied = kfifo_in_spinlocked(&priv->sensing_fifo,
                                     frame_data + DW1000_HYBRID_HDR_LEN,
                                     payload_len,
                                     &priv->fifo_lock);

    if (bytes_copied != payload_len) {
        dev_warn_ratelimited(&priv->spi->dev, "kfifo full (payload), dropping sensing frame\n");
        // Rollback previously written header
        kfifo_skip_spinlocked(&priv->sensing_fifo, 
                            offsetof(struct dw1000_sensing_frame, data),
                            &priv->fifo_lock);
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_ring++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        return -ENOSPC;
    }

    // Wake up waiting readers
    wake_up_interruptible(&priv->read_wq);

    // Update statistics
    spin_lock_irqsave(&priv->stats_lock, flags);
    priv->stats.rx_sensing_packets++;
    priv->stats.rx_sensing_bytes += payload_len;
    spin_unlock_irqrestore(&priv->stats_lock, flags);

    return 0;
}

// Process network frame
static int process_network_frame(struct dw1000_hybrid_priv *priv, void *frame_data, size_t frame_len)
{
    struct dw1000_hybrid_hdr *hdr = frame_data;
    struct net_device *netdev = priv->netdev;
    struct sk_buff *skb;
    u16 payload_len = frame_len - DW1000_HYBRID_HDR_LEN;
    unsigned long flags;

    // Allocate SKB
    skb = napi_alloc_skb(&priv->napi, payload_len + NET_IP_ALIGN);
    if (!skb) {
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_nomem++;
        priv->net_stats.rx_dropped++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        dev_warn_ratelimited(&priv->spi->dev, "Failed to allocate SKB for network frame\n");
        return -ENOMEM;
    }

    // Setup SKB
    skb_reserve(skb, NET_IP_ALIGN);
    skb_put_data(skb, frame_data + DW1000_HYBRID_HDR_LEN, payload_len);

    // Set SKB metadata
    skb->dev = netdev;
    skb->protocol = htons(ETH_P_IEEE802154); // 使用正确的协议类型
    skb->ip_summed = CHECKSUM_NONE;
    
    // 可选：设置额外的802.15.4元数据
    skb_set_mac_header(skb, 0);

    // Update statistics
    spin_lock_irqsave(&priv->stats_lock, flags);
    priv->stats.rx_packets++;
    priv->stats.rx_bytes += payload_len;
    priv->net_stats.rx_packets++;
    priv->net_stats.rx_bytes += payload_len;
    spin_unlock_irqrestore(&priv->stats_lock, flags);

    // Pass frame to network stack
    netif_receive_skb(skb);

    return 0;
}

// 辅助函数：处理接收到的帧
int process_rx_frame(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc)
{
    void *frame_data = desc->buffer;
    size_t frame_len;
    int spi_ret;
    unsigned long flags;
    struct dw1000_hybrid_hdr *hdr;

    // 从RX_FINFO寄存器获取实际接收长度
    spi_ret = dw1000_read_reg(priv, RG_RX_FINFO, 0, sizeof(u16), &frame_len);
    if (spi_ret) {
        dev_warn_ratelimited(&priv->spi->dev, "Failed to read RX_FINFO: %d\n", spi_ret);
        return -EIO;
    }
    frame_len = le16_to_cpu((__le16)frame_len) & RX_FINFO_RXFLEN_MASK;

    if (frame_len > desc->buffer_size) {
        dev_warn_ratelimited(&priv->spi->dev, "Frame len %zu > buffer size %zu\n",
                         frame_len, desc->buffer_size);
        return -EINVAL;
    }

    // 同步DMA缓冲区以供CPU访问
    dma_sync_single_for_cpu(&priv->spi->dev, desc->dma_addr,
                      desc->buffer_size, DMA_FROM_DEVICE);

    // 基本验证
    if (frame_len < DW1000_HYBRID_HDR_LEN) {
        dev_warn_ratelimited(&priv->spi->dev, "RX runt frame (len %zu)\n", frame_len);
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_other++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        return -EINVAL;
    }

    hdr = (struct dw1000_hybrid_hdr *)frame_data;

    // 根据帧类型分发处理
    switch (hdr->frame_type) {
        case DW1000_FRAME_TYPE_SENSING:
            return process_sensing_frame(priv, frame_data, frame_len);
        case DW1000_FRAME_TYPE_NETWORK:
            return process_network_frame(priv, frame_data, frame_len);
        default:
            dev_warn_ratelimited(&priv->spi->dev, "Unknown frame type: 0x%02x\n",
                             hdr->frame_type);
            spin_lock_irqsave(&priv->stats_lock, flags);
            priv->stats.rx_dropped_unknown_type++;
            spin_unlock_irqrestore(&priv->stats_lock, flags);
            return -EINVAL;
    }
}
EXPORT_SYMBOL(process_rx_frame);