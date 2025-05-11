#include "dw1000_generic.h"
#include "dw1000_frame_processing.h"
#include <linux/of_address.h> // 用于设备属性读取64位值

// 前向声明
static int dw1000_generic_hybrid_probe(struct spi_device *spi);
static int dw1000_generic_hybrid_remove(struct spi_device *spi);

// SPI设备ID表
static const struct spi_device_id dw1000_generic_hybrid_id[] = {
    { "dw1000-generic-hybrid", 0 },
    { "dw1000-generic", 0 }, // 允许匹配通用设备
    { }
};
MODULE_DEVICE_TABLE(spi, dw1000_generic_hybrid_id);

// 设备树匹配表
static const struct of_device_id dw1000_generic_hybrid_of_match[] = {
    { .compatible = "qorvo,dw1000-generic-hybrid" },
    { .compatible = "qorvo,dw1000-generic" },
    { }
};
MODULE_DEVICE_TABLE(of, dw1000_generic_hybrid_of_match);

// 网络设备操作函数集
static const struct net_device_ops dw1000_hybrid_netdev_ops = {
    .ndo_open               = dw1000_hybrid_net_open,
    .ndo_stop               = dw1000_hybrid_net_stop,
    .ndo_start_xmit         = dw1000_hybrid_net_start_xmit,
    .ndo_get_stats64        = dw1000_hybrid_get_stats64,
};

// 获取网络设备统计信息
static void dw1000_hybrid_get_stats64(struct net_device *netdev,
                                      struct rtnl_link_stats64 *stats)
{
    struct dw1000_hybrid_priv *priv = netdev_priv(netdev);
    unsigned long flags;

    spin_lock_irqsave(&priv->stats_lock, flags);
    stats->rx_packets = priv->stats.rx_packets;
    stats->tx_packets = priv->stats.tx_packets;
    stats->rx_bytes = priv->stats.rx_bytes;
    stats->tx_bytes = priv->stats.tx_bytes;
    stats->rx_errors = priv->stats.rx_rf_timeout +
                       priv->stats.rx_preamble_timeout +
                       priv->stats.rx_sfd_timeout +
                       priv->stats.rx_header_error +
                       priv->stats.rx_data_error +
                       priv->stats.rx_fcs_error +
                       priv->stats.rx_dma_errors;
    spin_unlock_irqrestore(&priv->stats_lock, flags);
}

// 网络设备打开
int dw1000_hybrid_net_open(struct net_device *netdev)
{
    struct dw1000_hybrid_priv *priv = netdev_priv(netdev);
    int ret;

    // 启用NAPI
    napi_enable(&priv->napi);

    // 启动网络设备
    netif_start_queue(netdev);

    // 重置设备
    ret = dw1000_reset_device(priv);
    if (ret) {
        netif_stop_queue(netdev);
        napi_disable(&priv->napi);
        return ret;
    }

    return 0;
}

// 网络设备关闭
int dw1000_hybrid_net_stop(struct net_device *netdev)
{
    struct dw1000_hybrid_priv *priv = netdev_priv(netdev);

    // 停止网络队列
    netif_stop_queue(netdev);

    // 禁用NAPI
    napi_disable(&priv->napi);

    return 0;
}

// 网络设备发送数据包
int dw1000_hybrid_net_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    struct dw1000_hybrid_priv *priv = netdev_priv(netdev);
    struct dw1000_tx_frame *tx_frame;
    unsigned long flags;
    int ret;

    // 检查是否有足够的发送描述符
    if (atomic_read(&priv->tx_pending_count) >= MAX_PENDING_TX) {
        netif_stop_queue(netdev);
        return NETDEV_TX_BUSY;
    }

    // 分配发送帧
    tx_frame = kzalloc(sizeof(struct dw1000_tx_frame) + skb->len, GFP_ATOMIC);
    if (!tx_frame) {
        dev_err(&priv->spi->dev, "无法分配发送帧\n");
        return NETDEV_TX_BUSY;
    }

    // 准备发送帧
    tx_frame->len = skb->len;
    tx_frame->wait_resp = 0;
    tx_frame->tx_mode = 0;
    memcpy(tx_frame->data, skb->data, skb->len);

    // 发送帧
    ret = dw1000_send_frame(priv, tx_frame);
    kfree(tx_frame);

    if (ret) {
        dev_err(&priv->spi->dev, "发送帧失败: %d\n", ret);
        return NETDEV_TX_BUSY;
    }

    // 更新统计信息
    spin_lock_irqsave(&priv->stats_lock, flags);
    priv->stats.tx_packets++;
    priv->stats.tx_bytes += skb->len;
    spin_unlock_irqrestore(&priv->stats_lock, flags);

    // 释放SKB
    dev_kfree_skb(skb);

    return NETDEV_TX_OK;
}

// --- 共享TX辅助函数 --- 
static int dw1000_hybrid_lowlevel_tx(struct dw1000_hybrid_priv *priv, 
                                   const void *frame_data, 
                                   size_t frame_len, 
                                   struct sk_buff *skb_to_free)
{
    struct dw1000_tx_desc *desc;
    int ret;

    // 1. 获取空闲的TX描述符
    desc = dw1000_get_free_tx_desc(priv);
    if (!desc) {
        dev_warn_ratelimited(&priv->spi->dev, "lowlevel_tx没有可用的TX描述符\n");
        // 如果提供了skb，由于无法发送需要释放它
        if (skb_to_free) {
            dev_kfree_skb_any(skb_to_free);
        }
        return -EBUSY;
    }

    // 2. 检查帧长度是否超过描述符缓冲区大小
    if (frame_len == 0 || frame_len > desc->buffer_size) {
        dev_err(&priv->spi->dev, "Lowlevel_tx: 无效的帧长度 %zu 对于缓冲区大小 %zu\n", 
                frame_len, desc->buffer_size);
        dw1000_put_tx_desc_on_error(desc); // 将描述符返回到空闲列表
        if (skb_to_free) {
            dev_kfree_skb_any(skb_to_free);
        }
        return -EINVAL;
    }
    
    // 3. 将帧数据复制到描述符的缓冲区
    memcpy(desc->buffer, frame_data, frame_len);

    // 4. 设置描述符字段
    desc->data_len = frame_len;
    desc->skb = skb_to_free; // 关联SKB以便回调时释放(如果有)

    // 5. 调用execute_tx辅助函数处理硬件配置和DMA提交
    ret = dw1000_execute_tx(priv, desc);
    if (ret) {
        // dw1000_execute_tx失败
        // 如果在DMA提交前失败，tx_pending_count会被它递减
        // 如果在DMA提交后失败(例如TXSTRT命令)，DMA处于挂起状态
        // dw1000_execute_tx failed. 
        // If it failed *before* DMA submission, tx_pending_count was decremented by it.
        // If it failed *after* DMA submission (e.g. TXSTRT command), DMA is pending.
        // The descriptor is NOT on the tx_pending_list if dw1000_submit_tx_desc failed.
        // The descriptor IS on the tx_pending_list if only TXSTRT failed.

        // Regardless, if execute_tx returns an error, we need to clean up here.
        // The descriptor should be returned to free_list ONLY if DMA was NOT successfully submitted.
        // dw1000_execute_tx handles atomic_dec if submit_tx_desc fails.
        // If submit_tx_desc succeeded but TXSTRT failed, desc is on pending_list, callback will clean it.
        // So, only call put_tx_desc_on_error if submit_tx_desc within execute_tx failed.
        // This is tricky. Let's assume execute_tx ensures desc is not on pending list if it returns error for pre-TXSTRT issues.

        // Simpler: If execute_tx returns error, it means the TX sequence was not fully successful.
        // The descriptor itself should be put back on the free list by the caller of execute_tx if DMA submission failed. 
        // dw1000_execute_tx already decremented tx_pending_count if dma_submit_tx_desc failed.
        // If only TXSTRT failed, desc is on pending_list and callback will handle it.
        // We only need to free the skb here and ensure desc is put back if not handled by callback.

        dev_err(&priv->spi->dev, "dw1000_execute_tx failed with %d in lowlevel_tx\n", ret);
        
        // Check if the descriptor is on the pending list. If not, it means DMA submission likely failed or was not attempted.
        // This requires checking list status, which adds complexity. 
        // A robust way: dw1000_execute_tx should guarantee that if it returns an error where DMA isn't pending,
        // the descriptor isn't on tx_pending_list. And tx_pending_count is correct.

        // For now, assume if dw1000_execute_tx fails, the descriptor state is such that
        // it should be returned to free list (unless it was due to TXSTRT failing post-DMA-submit).
        // The current dw1000_execute_tx logic: if dw1000_submit_tx_desc fails, it returns error, atomic_dec occurs.
        // Caller (this function) must put desc to free list.
        // If dw1000_submit_tx_desc succeeds but TXSTRT fails, desc IS on pending list. Callback handles.

        if (ret != -EAGAIN) { // Errors typically from submit_tx_desc or pre-submit checks
			dw1000_put_tx_desc_on_error(desc);
			// Else (e.g. error from TXSTRT), descriptor is on pending list, callback will free.

			if (desc->skb) { // Check desc->skb as skb_to_free might be different if logic changes
				dev_kfree_skb_any(desc->skb);
				desc->skb = NULL; // Important as descriptor might be reused from free list
			}

		} 
        return ret;
    }

    return 0; // Success
}
// --- Character Device Write Implementation (优化后) ---
static ssize_t dw1000_hybrid_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	struct dw1000_hybrid_priv *priv = filp->private_data;
	struct dw1000_hybrid_hdr hdr;
	// struct dw1000_tx_desc *desc; // No longer directly managed here
	size_t frame_len_with_hdr;
	int ret;
	u8 frame_type = DW1000_FRAME_TYPE_SENSING;
	u64 dest_addr = 0xFFFFFFFFFFFFFFFF;         
	const char __user *payload_buf = buf;       
	size_t payload_count = count;              
	unsigned long flags; // For config_lock
	void *tx_frame_buf = NULL; // Buffer for header + payload

	// Basic validation
	if (count == 0) return 0;
	
	// Check for optional frame type header byte
	u8 header_byte_user = 0;
	if (payload_count > 0 && copy_from_user(&header_byte_user, buf, 1) == 0) {
		u8 type_field = (header_byte_user >> 4) & 0x0F;
		if (type_field >= DW1000_FRAME_TYPE_NET && type_field <= DW1000_FRAME_TYPE_CONFIG) {
			frame_type = type_field;
			payload_buf++;
			payload_count--;
		}
	}

	if (payload_count == 0 && frame_type != DW1000_FRAME_TYPE_CONFIG) { // Config frames can be header-only
		dev_warn(&priv->spi->dev, "Cdev write: No payload after optional header byte\n");
		return -EINVAL;
	}
	
	// Total frame length: Our header + user payload
	frame_len_with_hdr = DW1000_HYBRID_HDR_LEN + payload_count;

	// Allocate buffer for the full frame (header + payload)
	tx_frame_buf = kmalloc(frame_len_with_hdr, GFP_KERNEL);
	if (!tx_frame_buf) {
		dev_err(&priv->spi->dev, "Failed to allocate TX frame buffer for cdev\n");
		return -ENOMEM;
	}

	// Prepare our custom frame header
	memset(&hdr, 0, sizeof(hdr));
	hdr.frame_type = frame_type;
	hdr.seq_num = atomic_inc_return(&priv->tx_sequence_num) & 0xFF;
	hdr.dest_addr = cpu_to_le64(dest_addr); 
	if (priv->netdev && priv->netdev->dev_addr) { // Use netdev MAC as source if available
		memcpy(&hdr.src_addr, priv->netdev->dev_addr, ETH_ALEN);
	} else {
		hdr.src_addr = cpu_to_le64(0); // Default source if no netdev MAC
	}
	spin_lock_irqsave(&priv->config_lock, flags);
	hdr.pan_id = cpu_to_le16(priv->config.pan_id);
	hdr.frame_ctrl = cpu_to_le16(FRAME_CTRL_DATA_SHORTADDR_PANIDCOMP);
	spin_unlock_irqrestore(&priv->config_lock, flags);

	// Copy header to the start of our allocated buffer
	memcpy(tx_frame_buf, &hdr, DW1000_HYBRID_HDR_LEN);

	// Copy payload from user space to our buffer, after the header
	if (payload_count > 0) {
		if (copy_from_user(tx_frame_buf + DW1000_HYBRID_HDR_LEN, payload_buf, payload_count)) {
			dev_err(&priv->spi->dev, "Failed to copy from user for cdev write\n");
			kfree(tx_frame_buf);
			return -EFAULT;
		}
	}

	// Call the low-level TX function (skb is NULL for cdev writes)
	ret = dw1000_hybrid_lowlevel_tx(priv, tx_frame_buf, frame_len_with_hdr, NULL);

	kfree(tx_frame_buf); // Free the temporary buffer

	if (ret) {
		dev_err(&priv->spi->dev, "Lowlevel TX failed for cdev write: %d\n", ret);
		return ret; // Return the error from lowlevel_tx (-EBUSY, -EINVAL, etc.)
	}

	// Update file position if needed 
	*f_pos += count; // Increment by the original user count (including optional header byte)
	return count;    
}

static int dw1000_generic_hybrid_probe(struct spi_device *spi) {
	struct dw1000_hybrid_priv *priv;
	struct net_device *netdev;
	int ret;
	dma_cap_mask_t mask;
	// struct dma_slave_config dma_cfg = {0}; // dma_cfg 在每个通道中单独定义
	int irq_num_local = -1; // 用于存储IRQ号的本地变量

	dev_info(&spi->dev, "正在探测 DW1000 通用混合驱动...\n");

	// 分配netdev（包含priv空间）
	netdev = alloc_netdev(sizeof(struct dw1000_hybrid_priv), "dw%d", NET_NAME_UNKNOWN, ether_setup);
	if (!netdev) {
		return -ENOMEM;
	}
	
	priv = netdev_priv(netdev);
	memset(priv, 0, sizeof(*priv)); // 确保完全清零
	
	// 初始化基本关联
	priv->netdev = netdev;  // 设置网络设备指针
	priv->spi = spi;        // 设置SPI设备指针
	spi_set_drvdata(spi, priv);
	
	// 初始化锁
	spin_lock_init(&priv->stats_lock);
	spin_lock_init(&priv->rx_desc_lock);
	spin_lock_init(&priv->tx_desc_lock);
	spin_lock_init(&priv->fifo_lock);
	spin_lock_init(&priv->config_lock);
	mutex_init(&priv->ioctl_lock);
	mutex_init(&priv->spi_reg_mutex);

	// 初始化原子变量
	atomic_set(&priv->tx_pending_count, 0);
	atomic_set(&priv->tx_sequence_num, 0);

	// 初始化默认配置 (使用 dw1000_hybrid_config)
	priv->config.channel = 5;          // 默认使用信道5
	priv->config.spi_speed_hz = DW1000_DEFAULT_SPI_HZ; // 8MHz SPI速率
	priv->config.prf = DW1000_DEFAULT_PRF;              // PRF 64M (0=16MHz, 1=64MHz)
	priv->config.preamble_length = DW1000_DEFAULT_PREAMBLE_LEN;  // 默认前导码长度 (例如: 8表示256)
	priv->config.preamble_code = DW1000_DEFAULT_PREAMBLE_CODE;    // 默认前导码索引
	priv->config.data_rate = DW1000_DEFAULT_DATA_RATE;        // 110kbps数据速率 (0=110k, 1=850k, 2=6.8M)
	priv->config.smart_power = true;   // 启用智能功率控制
	priv->config.tx_power = DW1000_DEFAULT_TX_POWER_REG;      // 默认TX功率 (寄存器值)
	priv->config.pan_id = DW1000_DEFAULT_PAN_ID;      // 默认PAN ID
	priv->config.default_dest_addr = 0xFFFFFFFFFFFFFFFF; // 默认广播地址
	priv->config.accept_bad_frames = false;
	priv->config.promiscuous = false;
	priv->config.ranging_enabled = false;
	priv->config.sfd_timeout = DW1000_DEFAULT_SFD_TIMEOUT; // 默认SFD超时值
	priv->config.preamble_timeout = 0; // 默认：使用硬件默认值

	// 设置网络设备参数
	netdev->netdev_ops = &dw1000_hybrid_netdev_ops;
	netdev->mtu = DW1000_DEFAULT_MTU;
	netdev->type = ARPHRD_IEEE802154;  // 802.15.4 协议类型
	netdev->needs_free_netdev = true;   // 让内核在最后释放netdev
	netdev->priv_destructor = NULL;     // 使用默认的释放函数
	eth_hw_addr_random(netdev);         // 随机MAC地址

	// 初始化NAPI
	netif_napi_add(netdev, &priv->napi, dw1000_hybrid_poll, NAPI_WEIGHT);

	// 获取中断GPIO
	priv->irq_gpio = devm_gpiod_get_optional(&spi->dev, "irq", GPIOD_IN);
	if (IS_ERR(priv->irq_gpio)) {
		ret = PTR_ERR(priv->irq_gpio);
		dev_err(&spi->dev, "获取IRQ GPIO失败: %d\n", ret);
		goto cleanup_napi;
	}

	if (!priv->irq_gpio) {
		dev_err(&spi->dev, "在设备树中未找到或未指定IRQ GPIO。\n");
		ret = -ENODEV;
		goto cleanup_napi;
	}

	// 获取并验证IRQ号
	irq_num_local = gpiod_to_irq(priv->irq_gpio);
	if (irq_num_local <= 0) {
		dev_err(&spi->dev, "从GPIO获取有效IRQ号失败: %d\n", irq_num_local);
		ret = irq_num_local < 0 ? irq_num_local : -EINVAL;
		goto cleanup_napi;
	}
	priv->irq_num = irq_num_local;
	dev_info(&spi->dev, "使用来自GPIO的IRQ号 %d\n", priv->irq_num);

	// 请求共享中断
	ret = devm_request_irq(&spi->dev, priv->irq_num,
			      dw1000_hybrid_irq_handler,
			      IRQF_TRIGGER_RISING | IRQF_SHARED,
			      DRIVER_NAME, priv);
	if (ret) {
		dev_err(&spi->dev, "IRQ请求失败: %d\n", ret);
		goto cleanup_napi;
	}
	disable_irq(priv->irq_num); // 在设备打开前禁用中断

	// 分配字符设备区域
	ret = alloc_chrdev_region(&priv->cdev_devt, 0, 1, "dw1000_sensor");
	if (ret < 0) {
		dev_err(&spi->dev, "字符设备区域分配失败: %d\n", ret);
		goto cleanup_napi;
	}
	
	// 初始化并添加字符设备
	cdev_init(&priv->cdev, &dw1000_hybrid_cdev_fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, priv->cdev_devt, 1);
	if (ret) {
		dev_err(&spi->dev, "cdev_add失败: %d\n", ret);
		goto cleanup_chrdev_region;
	}
	
	// 创建字符设备类/节点
	priv->cdev_class = class_create(THIS_MODULE, "dw1000_sensor");
	if (IS_ERR(priv->cdev_class)) {
		ret = PTR_ERR(priv->cdev_class);
		dev_err(&spi->dev, "创建设备类失败: %d\n", ret);
		goto cleanup_cdev;
	}

	if (!device_create(priv->cdev_class, &spi->dev, priv->cdev_devt, 
			  priv, "dw1000_sensor%d", MINOR(priv->cdev_devt))) {
		dev_err(&spi->dev, "创建设备节点失败\n");
		ret = -ENOMEM;
		goto cleanup_class;
	}
	
	// 初始化等待队列和kfifo
	init_waitqueue_head(&priv->read_wq);
	
	// 使用 vmalloc 分配缓冲区
	priv->sensing_buffer_size = RING_BUFFER_SIZE;
	priv->sensing_buffer = vmalloc(priv->sensing_buffer_size);
	if (!priv->sensing_buffer) {
		dev_err(&spi->dev, "分配传感缓冲区失败\n");
		ret = -ENOMEM;
		goto cleanup_kfifo;
	}

	// 使用分配的缓冲区初始化 kfifo
	if (kfifo_alloc(&priv->sensing_fifo, priv->sensing_buffer_size, GFP_KERNEL)) {
		dev_err(&spi->dev, "分配kfifo失败\n");
		ret = -ENOMEM;
		goto cleanup_sensing_buffer;
	}

	// 设置DMA缓冲区大小 (这些是描述符负载缓冲区，确保在dw1000_generic.h中定义正确)
	// priv->tx_dma_buf_size = DW1000_DEFAULT_TX_DMA_BUF_SIZE; // 示例: DEFAULT_TX_DESC_BUF_SIZE
	// priv->rx_dma_buf_size = DW1000_DEFAULT_RX_DMA_BUF_SIZE; // 示例: DEFAULT_RX_DESC_BUF_SIZE
	
	// 申请DMA通道
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	
	// 获取SPI控制器的物理地址 - 移除旧逻辑
	// struct resource *res;
	// phys_addr_t spi_phys_addr = 0;
	
	// 尝试从SPI控制器的父设备获取物理地址 - 移除旧逻辑
	// res = platform_get_resource(to_platform_device(spi->controller->dev.parent),
	// IORESOURCE_MEM, 0);
	// if (!res) {
	// dev_err(&spi->dev, "获取SPI控制器物理地址失败\n");
	// ret = -ENODEV;
	// goto cleanup_ring_buffer; // 应该是cleanup_kfifo或类似的
	// }
	// spi_phys_addr = res->start;
	// dev_info(&spi->dev, "SPI控制器物理地址: 0x%llx\n", 
	// (unsigned long long)spi_phys_addr);

	// 定义SPI控制器寄存器偏移量 - 移除硬编码定义
	// #define SPI_TXDR_OFFSET 0x400  // TX数据寄存器偏移量
	// #define SPI_RXDR_OFFSET 0x800  // RX数据寄存器偏移量

	phys_addr_t tx_fifo_phys = 0;
	phys_addr_t rx_fifo_phys = 0;

	if (!spi->dev.of_node) {
		dev_err(&spi->dev, "未找到设备树节点\n");
		ret = -EINVAL;
		goto cleanup_kfifo; // 更新标签
	}

	ret = device_property_read_u64(spi->dev.of_node, "qorvo,spi-tx-fifo-phys", &tx_fifo_phys);
	if (ret) {
		dev_err(&spi->dev, "读取'qorvo,spi-tx-fifo-phys'属性失败: %d\n", ret);
		goto cleanup_kfifo; // 更新标签
	}

	ret = device_property_read_u64(spi->dev.of_node, "qorvo,spi-rx-fifo-phys", &rx_fifo_phys);
	if (ret) {
		dev_err(&spi->dev, "读取'qorvo,spi-rx-fifo-phys'属性失败: %d\n", ret);
		goto cleanup_kfifo; // 更新标签
	}
	
	dev_info(&spi->dev, "从DT使用TX FIFO地址: 0x%llx, RX FIFO地址: 0x%llx\n",
			 (unsigned long long)tx_fifo_phys, (unsigned long long)rx_fifo_phys);

	// RX DMA通道
	priv->rx_dma_chan = dma_request_slave_channel_compat(mask, NULL, NULL, &spi->dev, "rx");
	if (!priv->rx_dma_chan) {
		dev_err(&spi->dev, "请求RX DMA通道失败\n");
		ret = -ENODEV;
		goto cleanup_kfifo; // 更新标签
	}

	// TX DMA通道
	priv->tx_dma_chan = dma_request_slave_channel_compat(mask, NULL, NULL, &spi->dev, "tx");
	if (!priv->tx_dma_chan) {
		dev_err(&spi->dev, "请求TX DMA通道失败\n");
		ret = -ENODEV;
		goto cleanup_rx_dma_chan;
	}

	// 配置RX DMA通道
	struct dma_slave_config rx_dma_cfg = {0};
	rx_dma_cfg.direction = DMA_DEV_TO_MEM;
	rx_dma_cfg.src_addr = rx_fifo_phys; // 使用来自DT的地址
	rx_dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	rx_dma_cfg.src_maxburst = 1;
	
	ret = dmaengine_slave_config(priv->rx_dma_chan, &rx_dma_cfg);
	if (ret) {
		dev_err(&spi->dev, "配置RX DMA通道失败: %d\n", ret);
		goto cleanup_rx_dma_chan; // 修正此失败点的标签
	}

	// 配置TX DMA通道
	struct dma_slave_config tx_dma_cfg = {0};
	tx_dma_cfg.direction = DMA_MEM_TO_DEV;
	tx_dma_cfg.dst_addr = tx_fifo_phys; // 使用来自DT的地址
	tx_dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	tx_dma_cfg.dst_maxburst = 1;

	ret = dmaengine_slave_config(priv->tx_dma_chan, &tx_dma_cfg);
	if (ret) {
		dev_err(&spi->dev, "配置TX DMA通道失败: %d\n", ret);
		goto cleanup_tx_dma_chan; 
	}

	// 初始化DW1000硬件
	// 1. 首先执行设备复位和基础配置
	ret = dw1000_reset_device(priv);
	if (ret) {
		dev_err(&spi->dev, "设备复位失败: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// 2. 执行额外的硬件初始化（中断掩码等）
	ret = dw1000_hw_init(priv);
	if (ret) {
		dev_err(&spi->dev, "硬件初始化失败: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// 初始化 TX DMA 描述符
	ret = dw1000_setup_tx_desc(priv);
	if (ret) {
		dev_err(&spi->dev, "设置TX DMA描述符失败: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// 设置初始RX DMA
	ret = dw1000_setup_rx_dma(priv);
	if (ret) {
		dev_err(&spi->dev, "设置RX DMA失败: %d\n", ret);
		goto cleanup_tx_desc_and_channels;
	}

	// 注册网络设备
	ret = register_netdev(netdev);
	if (ret) {
		dev_err(&spi->dev, "注册netdev失败: %d\n", ret);
		goto cleanup_rx_tx_desc_and_channels;
	}

	dev_info(&spi->dev, "DW1000通用混合驱动探测成功! (网络: %s, 字符设备: %d:%d)\n",
			 netdev->name, MAJOR(priv->cdev_devt), MINOR(priv->cdev_devt));

	return 0;

cleanup_rx_tx_desc_and_channels:
	dw1000_teardown_rx_dma(priv);
cleanup_tx_desc_and_channels:
	dw1000_cleanup_tx_desc(priv);
cleanup_dma_channels:
	if (priv->tx_dma_chan)
		dma_release_channel(priv->tx_dma_chan);
cleanup_rx_dma_chan: // 添加此标签以提高清晰度
	if (priv->rx_dma_chan)
		dma_release_channel(priv->rx_dma_chan);
cleanup_kfifo: // kfifo_alloc分配的新标签
	kfifo_free(&priv->sensing_fifo);
cleanup_sensing_buffer: // vmalloc的原始标签
	if (priv->sensing_buffer) {
		vfree(priv->sensing_buffer);
		priv->sensing_buffer = NULL;
	}
cleanup_device:
	if (priv->cdev_class && !IS_ERR(priv->cdev_class))
		device_destroy(priv->cdev_class, priv->cdev_devt);
cleanup_class:
	if (priv->cdev_class && !IS_ERR(priv->cdev_class))
		class_destroy(priv->cdev_class);
cleanup_cdev:
	cdev_del(&priv->cdev);
cleanup_chrdev_region:
	unregister_chrdev_region(priv->cdev_devt, 1);
cleanup_napi:
	netif_napi_del(&priv->napi);
cleanup_free_netdev:
	free_netdev(netdev);
	return ret;
}

static int dw1000_generic_hybrid_remove(struct spi_device *spi)
{
    struct dw1000_hybrid_priv *priv = spi_get_drvdata(spi);
    struct net_device *netdev = priv->netdev;
    int timeout;

    dev_info(&spi->dev, "Removing DW1000 Generic HYBRID driver...\n");

    // 1. 首先注销网络设备 (如果在运行)
    if (netdev) {
        unregister_netdev(netdev);
    }

    // 2. 停止所有 DMA 操作
    if (priv->rx_dma_chan) {
        dmaengine_terminate_all(priv->rx_dma_chan);
        dw1000_teardown_rx_dma(priv);
    }

    if (priv->tx_dma_chan) {
        dmaengine_terminate_all(priv->tx_dma_chan);
        timeout = 100; // Wait up to 100ms for callbacks
        while (atomic_read(&priv->tx_pending_count) > 0 && timeout > 0) {
            msleep(1);
            timeout--;
        }
        dmaengine_synchronize(priv->tx_dma_chan);
        dw1000_cleanup_tx_desc(priv);
    }

    // 3. 释放 DMA 通道
    if (priv->rx_dma_chan) {
        dma_release_channel(priv->rx_dma_chan);
    }
    if (priv->tx_dma_chan) {
        dma_release_channel(priv->tx_dma_chan);
    }

    // 4. 清理字符设备资源
    if (priv->sensing_buffer) {
        // 确保没有正在进行的 mmap 操作
        msleep(100); // 给时间让所有 VM 操作完成
        
        // 清理 kfifo
        kfifo_free(&priv->sensing_fifo);
        
        // 释放 vmalloc 分配的内存
        vfree(priv->sensing_buffer);
        priv->sensing_buffer = NULL;
    }

    if (priv->cdev_class) {
        device_destroy(priv->cdev_class, priv->cdev_devt);
        class_destroy(priv->cdev_class);
    }
    cdev_del(&priv->cdev);
    unregister_chrdev_region(priv->cdev_devt, 1);

    // 5. 最后释放网络设备结构
    if (netdev) {
        free_netdev(netdev);
    }

    dev_info(&spi->dev, "DW1000 Generic HYBRID driver removed successfully\n");
    return 0;
}

// 发送测试函数
static int dw1000_tx_test(struct dw1000_hybrid_priv *priv, int count)
{
    int i, ret;
    u8 test_frame[DW1000_TX_TEST_FRAME_SIZE] = {0};
    struct dw1000_hybrid_hdr *hdr;
    struct dw1000_tx_frame *tx_frame;
    
    dev_info(&priv->spi->dev, "Starting TX test with %d frames\n", count);
    
    // 分配发送帧
    tx_frame = kzalloc(sizeof(struct dw1000_tx_frame) + sizeof(test_frame), GFP_KERNEL);
    if (!tx_frame) {
        return -ENOMEM;
    }
    
    // 准备测试帧
    hdr = (struct dw1000_hybrid_hdr *)test_frame;
    hdr->frame_type = DW1000_FRAME_TYPE_NET;
    hdr->dest_addr = cpu_to_le64(0xFFFFFFFFFFFFFFFF); // 广播
    if (priv->netdev && priv->netdev->dev_addr) {
        memcpy(&hdr->src_addr, priv->netdev->dev_addr, ETH_ALEN);
    }
    hdr->pan_id = cpu_to_le16(priv->config.pan_id);
    
    // 发送测试帧
    for (i = 0; i < count; i++) {
        hdr->seq_num = i & 0xFF;
        
        tx_frame->len = sizeof(test_frame);
        tx_frame->wait_resp = 0; // 不等待响应
        memcpy(tx_frame->data, test_frame, sizeof(test_frame));
        
        ret = dw1000_send_frame(priv, tx_frame);
        if (ret) {
            dev_err(&priv->spi->dev, "TX test: frame %d send failed\n", i);
            break;
        }
        
        // 可选：添加短暂延迟
        usleep_range(10000, 15000);
    }
    
    kfree(tx_frame);
    return ret;
}

// 接收测试函数
static int dw1000_rx_test(struct dw1000_hybrid_priv *priv, int duration_ms)
{
    unsigned long timeout = jiffies + msecs_to_jiffies(duration_ms);
    int frames_received = 0;
    
    dev_info(&priv->spi->dev, "Starting RX test for %d ms\n", duration_ms);
    
    // 启用接收
    dw1000_hw_init(priv);
    
    while (time_before(jiffies, timeout)) {
        // 检查是否有帧接收
        if (!list_empty(&priv->rx_pending_list)) {
            struct dw1000_rx_desc *desc;
            
            // 处理接收到的帧
            list_for_each_entry(desc, &priv->rx_pending_list, list) {
                process_rx_frame(priv, desc);
                frames_received++;
            }
            
            // 清空待处理列表
            INIT_LIST_HEAD(&priv->rx_pending_list);
        }
        
        // 短暂延迟
        usleep_range(1000, 2000);
    }
    
    dev_info(&priv->spi->dev, "RX test completed. Frames received: %d\n", frames_received);
    
    return frames_received;
}

// 导出符号，允许在其他模块中使用
EXPORT_SYMBOL(dw1000_tx_test);
EXPORT_SYMBOL(dw1000_rx_test);

// 在文件末尾添加低级发送函数
int dw1000_send_frame(struct dw1000_hybrid_priv *priv, struct dw1000_tx_frame *frame)
{
    struct dw1000_tx_desc *desc;
    int ret;

    // 获取空闲的发送描述符
    desc = dw1000_get_free_tx_desc(priv);
    if (!desc) {
        dev_err(&priv->spi->dev, "No free TX descriptor available\n");
        return -EBUSY;
    }

    // 准备发送描述符
    desc->buffer_size = frame->len;
    desc->data_len = frame->len;
    memcpy(desc->buffer, frame->data, frame->len);
    desc->wait_resp = frame->wait_resp;

    // 提交发送描述符
    ret = dw1000_submit_tx_desc(priv, desc);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to submit TX descriptor\n");
        dw1000_put_tx_desc_on_error(desc);
        return ret;
    }

    return 0;
}
EXPORT_SYMBOL(dw1000_send_frame);

// 字符设备打开函数
static int dw1000_hybrid_cdev_open(struct inode *inode, struct file *filp)
{
    struct dw1000_hybrid_priv *priv = container_of(inode->i_cdev, struct dw1000_hybrid_priv, cdev);
    
    // 检查是否已经打开
    if (atomic_inc_return(&priv->cdev_open_count) > 1) {
        atomic_dec(&priv->cdev_open_count);
        return -EBUSY;
    }
    
    filp->private_data = priv;
    
    // 初始化等待队列和相关状态
    init_waitqueue_head(&priv->read_wq);
    
    return 0;
}

// 字符设备释放函数
static int dw1000_hybrid_cdev_release(struct inode *inode, struct file *filp)
{
    struct dw1000_hybrid_priv *priv = filp->private_data;
    
    // 减少打开计数
    atomic_dec(&priv->cdev_open_count);
    
    return 0;
}

// 字符设备轮询函数
static __poll_t dw1000_hybrid_cdev_poll(struct file *filp, poll_table *wait)
{
    struct dw1000_hybrid_priv *priv = filp->private_data;
    __poll_t mask = 0;
    
    poll_wait(filp, &priv->read_wq, wait);
    
    // 检查是否有数据可读
    if (!kfifo_is_empty(&priv->sensing_fifo)) {
        mask |= POLLIN | POLLRDNORM;
    }
    
    return mask;
}

// 字符设备内存映射函数
static int dw1000_hybrid_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct dw1000_hybrid_priv *priv = filp->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;
    
    // 检查映射大小是否超过缓冲区
    if (size > priv->sensing_buffer_size) {
        return -EINVAL;
    }
    
    // 设置 VM 操作
    vma->vm_ops = &dw1000_vm_ops;
    vma->vm_private_data = priv;
    
    // 映射内核缓冲区
    if (remap_pfn_range(vma, 
                        vma->vm_start, 
                        virt_to_phys(priv->sensing_buffer) >> PAGE_SHIFT,
                        size, 
                        vma->vm_page_prot)) {
        return -EAGAIN;
    }
    
    dw1000_vm_open(vma);
    return 0;
}

// --- 字符设备IOCTL (添加了统计锁) ---
static long dw1000_hybrid_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct dw1000_hybrid_priv *priv = filp->private_data;
	long ret = 0;
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	struct dw1000_hybrid_config old_config;
	bool config_changed = false; 
	// 移除了之前尝试中重复声明的old_config

	mutex_lock(&priv->ioctl_lock); // 在开始时添加锁

	switch (cmd) {
	case DW1000_IOC_SET_CONFIG:
	{
		struct dw1000_hybrid_config user_config; // 使用正确的结构体类型
		if (copy_from_user(&user_config, argp, sizeof(user_config)))
			return -EFAULT;

        // 可选：在此处验证user_config参数
        if (user_config.channel < 1 || user_config.channel > 8 ||
            user_config.prf > 1 ||
            user_config.data_rate > 2) { // 根据需要添加其他检查
            return -EINVAL;
        }

		bool was_running = netif_running(priv->netdev);
		
		old_config = priv->config;

		// 检查是否需要硬件重新配置（在写入前的读取访问在config_lock外是可以的）
		spin_lock_irqsave(&priv->config_lock, flags); // 在比较/写入前加锁
		if (memcmp(&user_config, &old_config, sizeof(user_config)) != 0) {
			config_changed = true;
		}
		spin_unlock_irqrestore(&priv->config_lock, flags); // 比较/保存后解锁
		
		// 保存新配置（无论是否更改）
		// 这个写入需要保护
		spin_lock_irqsave(&priv->config_lock, flags);
		memcpy(&priv->config, &user_config, sizeof(struct dw1000_hybrid_config));
		spin_unlock_irqrestore(&priv->config_lock, flags);
		
		// 如果硬件配置参数改变，重新配置硬件
		if (config_changed) {
			// 暂停网络设备和NAPI（在config_lock外）
			if (was_running) {
				netif_stop_queue(priv->netdev);
				napi_disable(&priv->napi);
			}
			
			// 重新配置硬件（apply_config内部已处理其自身的锁定）
			ret = dw1000_apply_config(priv);
			
			// 恢复网络设备和NAPI或处理失败
			if (ret == 0) { // 配置成功
				if (was_running) {
					napi_enable(&priv->napi);
					netif_start_queue(priv->netdev);
				}
			} else { // 配置失败
				dev_err(&priv->spi->dev, "应用新配置失败 (%d)，尝试恢复...\n", ret);
				// 在内存中恢复旧配置（受保护的写入）
				spin_lock_irqsave(&priv->config_lock, flags);
				priv->config = old_config; // 使用保存的old_config
				spin_unlock_irqrestore(&priv->config_lock, flags);
				
				// 尝试应用恢复的旧配置
				int restore_ret = dw1000_apply_config(priv); // 再次调用配置函数
				if (restore_ret != 0) {
					dev_err(&priv->spi->dev, "致命错误：恢复旧配置失败 (%d)。接口保持停止状态。\n", restore_ret);
					// 保持停止状态
				} else {
					dev_info(&priv->spi->dev, "成功恢复旧配置。\n");
					if (was_running) { // 如果之前在运行则重新启动
						napi_enable(&priv->napi);
						netif_start_queue(priv->netdev);
					}
				}
				// 返回原始错误代码'ret'
			}
		}
		break;
	}
	case DW1000_IOC_GET_STATS:
	{
		struct dw1000_hybrid_stats current_stats; // 使用正确的统计结构体
		spin_lock_irqsave(&priv->stats_lock, flags);
		current_stats = priv->stats;
		spin_unlock_irqrestore(&priv->stats_lock, flags);

		if (copy_to_user(argp, &current_stats, sizeof(current_stats)))
			return -EFAULT;
		break;
	}
	case DW1000_IOC_RESET:
	{
        bool was_running = netif_running(priv->netdev);
		dev_info(&priv->spi->dev, "IOCTL：通过IOCTL重置硬件\n");

        if (was_running) {
            netif_stop_queue(priv->netdev);
            napi_disable(&priv->napi);
        }

		ret = dw1000_reset_device(priv); // dw1000_reset_device现在会调用apply_config
		
		// 移除冗余的apply_config调用
		/* 
        if (ret == 0) { 
             ret = dw1000_apply_config(priv); 
        }
		*/
		
        if (ret == 0 && was_running) { // 重置成功（这意味着最终配置也成功了）
            napi_enable(&priv->napi);
            netif_start_queue(priv->netdev);
        } else if (ret != 0 && was_running) { // 重置失败
             dev_err(&priv->spi->dev, "通过IOCTL的硬件重置失败 (%d)。接口保持停止状态。", ret);
            // 如果重置失败，不要重启NAPI/队列
        }
		break;
	}
	case DW1000_IOC_SET_DEST_ADDR:
	{
		u64 dest_addr;
		if (copy_from_user(&dest_addr, argp, sizeof(dest_addr)))
			return -EFAULT;
		// 保护对配置的写入
		spin_lock_irqsave(&priv->config_lock, flags);
		priv->config.default_dest_addr = dest_addr; 
		spin_unlock_irqrestore(&priv->config_lock, flags);
		dev_info(&priv->spi->dev, "IOCTL：设置默认目标地址为 0x%llx\n", priv->config.default_dest_addr); // 在锁外读取是可以的
		// 这不需要硬件重新配置
		break;
	}
	case DW1000_IOC_ACCEPT_BAD_FRAMES:
	{
		bool accept_bad_frames;
		if (copy_from_user(&accept_bad_frames, argp, sizeof(accept_bad_frames)))
			return -EFAULT;
		// 仅在值改变时应用
		// 在锁外进行比较读取是可以的
		if (priv->config.accept_bad_frames != accept_bad_frames) {
            bool was_running = netif_running(priv->netdev);
		    // 保护对配置的写入
		    spin_lock_irqsave(&priv->config_lock, flags);
		    priv->config.accept_bad_frames = accept_bad_frames;
		    spin_unlock_irqrestore(&priv->config_lock, flags);
		    dev_info(&priv->spi->dev, "IOCTL：设置接受错误帧：%d\n", priv->config.accept_bad_frames); // 在锁外读取是可以的
            
            if (was_running) {
                netif_stop_queue(priv->netdev);
                napi_disable(&priv->napi);
            }
            // apply_config内部处理其自身的config_lock
            ret = dw1000_apply_config(priv); // 重新应用完整配置
            if (ret == 0) { // 应用成功
                if (was_running) { // 如果之前在运行，则重新启动
                    napi_enable(&priv->napi);
                    netif_start_queue(priv->netdev);
                }
            } else { // 应用失败
                 dev_err(&priv->spi->dev, "应用accept_bad_frames的配置失败 (%d)，尝试恢复...\n", ret);
                 // 尝试恢复先前的值
                 spin_lock_irqsave(&priv->config_lock, flags);
                 priv->config.accept_bad_frames = !accept_bad_frames; // 恢复相反的值
                 spin_unlock_irqrestore(&priv->config_lock, flags);
                 // 尝试用恢复的值重新应用配置
                 int restore_ret = dw1000_apply_config(priv);
                 if (restore_ret != 0) {
                    dev_err(&priv->spi->dev, "致命错误：accept_bad_frames更改失败后恢复配置失败 (%d)。接口保持停止状态。\n", restore_ret);
                 } else {
                    dev_info(&priv->spi->dev, "accept_bad_frames更改失败后成功恢复配置。\n");
                    if (was_running) { // 如果之前在运行则重新启动
                        napi_enable(&priv->napi);
                        netif_start_queue(priv->netdev);
                    }
                 }
                 // 返回原始错误'ret'
            }
        }
		break;
	}
	default:
		ret = -ENOTTY; // 无效的IOCTL命令
		break;
	}

	mutex_unlock(&priv->ioctl_lock); // 在返回前解锁
	return ret;
}
// VM 操作：打开
static void dw1000_vm_open(struct vm_area_struct *vma)
{
    struct dw1000_hybrid_priv *priv = vma->vm_private_data;
    atomic_inc(&priv->mmap_ref_count);
}

// VM 操作：关闭
static void dw1000_vm_close(struct vm_area_struct *vma)
{
    struct dw1000_hybrid_priv *priv = vma->vm_private_data;
    atomic_dec(&priv->mmap_ref_count);
}

// VM 操作：处理页面错误
static vm_fault_t dw1000_vm_fault(struct vm_fault *vmf)
{
    struct dw1000_hybrid_priv *priv = vmf->vma->vm_private_data;
    struct page *page;
    
    // 获取对应的页面
    page = vmalloc_to_page(priv->sensing_buffer + (vmf->pgoff << PAGE_SHIFT));
    
    if (!page) {
        return VM_FAULT_SIGBUS;
    }
    
    get_page(page);
    vmf->page = page;
    return 0;
}

// --- 虚拟内存操作结构体(mmap) ---
static const struct vm_operations_struct dw1000_vm_ops = {
    .open = dw1000_vm_open,
    .close = dw1000_vm_close,
    .fault = dw1000_vm_fault
}; 

// --- 文件操作结构体(字符设备) ---
static const struct file_operations dw1000_hybrid_cdev_fops = {
	.owner = THIS_MODULE,
	.open = dw1000_hybrid_cdev_open,
	.release = dw1000_hybrid_cdev_release,
	.poll = dw1000_hybrid_cdev_poll,
	.mmap = dw1000_hybrid_cdev_mmap,
	.write = dw1000_hybrid_cdev_write,
	.unlocked_ioctl = dw1000_hybrid_cdev_ioctl,
};

// --- 网络设备操作结构体 ---
static const struct net_device_ops dw1000_hybrid_netdev_ops = {
	.ndo_open = dw1000_hybrid_net_open,
	.ndo_stop = dw1000_hybrid_net_stop,
	.ndo_start_xmit = dw1000_hybrid_net_start_xmit,
	.ndo_get_stats64 = dev_get_tstats64,
};

// SPI驱动定义
static struct spi_driver dw1000_generic_hybrid_spi_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = dw1000_generic_hybrid_of_match,
    },
    .id_table = dw1000_generic_hybrid_id,
    .probe = dw1000_generic_hybrid_probe,
    .remove = dw1000_generic_hybrid_remove,
};

// 模块初始化
static int __init dw1000_generic_hybrid_init(void)
{
    pr_info("正在初始化 DW1000 通用混合驱动\n");
    return spi_register_driver(&dw1000_generic_hybrid_spi_driver);
}
module_init(dw1000_generic_hybrid_init);

// 模块退出
static void __exit dw1000_generic_hybrid_exit(void)
{
    pr_info("正在移除 DW1000 通用混合驱动\n");
    spi_unregister_driver(&dw1000_generic_hybrid_spi_driver);
}
module_exit(dw1000_generic_hybrid_exit);

MODULE_AUTHOR("Jinting Liu");
MODULE_DESCRIPTION("Refined High-performance DW1000 HYBRID driver skeleton (Generic Edition)");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.6");
