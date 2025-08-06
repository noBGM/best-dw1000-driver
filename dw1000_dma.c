#include "dw1000_generic.h"

/*
 * DW1000 DMA管理模块
 * 
 * 重要优化说明：
 * 1. 使用流式DMA代替一致性DMA，提高性能
 *    - kmalloc + dma_map_single 替代 dma_alloc_coherent
 *    - 允许CPU缓存优化，显著提升大数据量处理性能
 * 
 * 2. 缓存一致性管理：
 *    - TX: dma_sync_single_for_device() 确保CPU写入对DMA可见
 *    - RX: dma_sync_single_for_cpu() 确保DMA数据对CPU可见
 * 
 * 3. 正确的内存管理顺序：
 *    - 分配：kmalloc -> dma_map_single
 *    - 释放：dma_unmap_single -> kfree
 */

// 初始化RX DMA描述符
int dw1000_setup_rx_desc(struct dw1000_hybrid_priv *priv)
{
    int i;
    unsigned long flags;
    
    // 初始化链表头
    INIT_LIST_HEAD(&priv->rx_free_list);
    INIT_LIST_HEAD(&priv->rx_pending_list);
    
    // 分配并初始化所有描述符
    for (i = 0; i < NUM_RX_DESC; i++) {
        struct dw1000_rx_desc *desc = &priv->rx_desc[i];
        
        // 为每个描述符分配流式DMA缓冲区
        desc->buffer_size = RX_DESC_BUFFER_SIZE;
        
        // 使用kmalloc分配可缓存的内存，提高CPU访问性能
        // 相比dma_alloc_coherent，这种方式允许CPU缓存优化，
        // 对于大量数据的处理可以显著提升性能
        desc->buffer = kmalloc(desc->buffer_size, GFP_KERNEL);
        if (!desc->buffer) {
            dev_err(&priv->spi->dev, "分配RX缓冲区 %d 失败\n", i);
            goto fail_cleanup;
        }
        
        // 使用流式DMA映射，允许CPU缓存优化
        desc->dma_addr = dma_map_single(&priv->spi->dev, 
                                        desc->buffer,
                                        desc->buffer_size,
                                        DMA_FROM_DEVICE);
        if (dma_mapping_error(&priv->spi->dev, desc->dma_addr)) {
            dev_err(&priv->spi->dev, "映射RX DMA缓冲区 %d 失败\n", i);
            kfree(desc->buffer);
            desc->buffer = NULL;
            goto fail_cleanup;
        }
        
        desc->priv = priv;
        desc->in_use = false;
        desc->data_len = 0;
        
        // 添加到空闲链表
        spin_lock_irqsave(&priv->rx_desc_lock, flags);
        list_add_tail(&desc->list, &priv->rx_free_list);
        spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    }
    
    return 0;
    
fail_cleanup:
    // 清理已分配的描述符
    while (--i >= 0) {
        struct dw1000_rx_desc *desc = &priv->rx_desc[i];
        if (desc->buffer) {
            // 先解除DMA映射
            if (desc->dma_addr) {
                dma_unmap_single(&priv->spi->dev,
                                desc->dma_addr,
                                desc->buffer_size,
                                DMA_FROM_DEVICE);
            }
            // 释放kmalloc分配的内存
            kfree(desc->buffer);
            desc->buffer = NULL;
        }
    }
    return -ENOMEM;
}
EXPORT_SYMBOL(dw1000_setup_rx_desc);

// 清理RX DMA描述符
void dw1000_cleanup_rx_desc(struct dw1000_hybrid_priv *priv)
{
    int i;
    unsigned long flags;
    
    // 确保所有链表为空
    spin_lock_irqsave(&priv->rx_desc_lock, flags);
    INIT_LIST_HEAD(&priv->rx_free_list);
    INIT_LIST_HEAD(&priv->rx_pending_list);
    spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    
    // 释放所有描述符缓冲区
    for (i = 0; i < NUM_RX_DESC; i++) {
        struct dw1000_rx_desc *desc = &priv->rx_desc[i];
        if (desc->buffer) {
            // 先解除DMA映射
            if (desc->dma_addr) {
                dma_unmap_single(&priv->spi->dev,
                                desc->dma_addr,
                                desc->buffer_size,
                                DMA_FROM_DEVICE);
            }
            // 释放kmalloc分配的内存
            kfree(desc->buffer);
            desc->buffer = NULL;
        }
    }
}
EXPORT_SYMBOL(dw1000_cleanup_rx_desc);

// 获取一个空闲RX描述符
struct dw1000_rx_desc *dw1000_get_free_rx_desc(struct dw1000_hybrid_priv *priv)
{
    struct dw1000_rx_desc *desc = NULL;
    unsigned long flags;
    
    spin_lock_irqsave(&priv->rx_desc_lock, flags);
    if (!list_empty(&priv->rx_free_list)) {
        desc = list_first_entry(&priv->rx_free_list, struct dw1000_rx_desc, list);
        list_del(&desc->list);
        desc->in_use = true;
        desc->data_len = 0;
    }
    spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    
    return desc;
}
EXPORT_SYMBOL(dw1000_get_free_rx_desc);

// 提交RX描述符到DMA控制器
int dw1000_submit_rx_desc(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc)
{
    struct dma_async_tx_descriptor *dma_desc;
    dma_cookie_t cookie;
    unsigned long flags;
    int ret = 0;
    
    // 在DMA传输前同步缓存，为DMA接收准备缓冲区
    dma_sync_single_for_device(&priv->spi->dev,
                              desc->dma_addr,
                              desc->buffer_size,
                              DMA_FROM_DEVICE);
    
    // 准备DMA描述符
    dma_desc = dmaengine_prep_slave_single(priv->rx_dma_chan,
                                          desc->dma_addr,
                                          desc->buffer_size,
                                          DMA_DEV_TO_MEM,
                                          DMA_PREP_INTERRUPT);
    if (!dma_desc) {
        dev_err(&priv->spi->dev, "准备RX DMA描述符失败\n");
        return -EIO;
    }
    
    // 设置回调和参数
    dma_desc->callback = dw1000_hybrid_dma_rx_callback;
    dma_desc->callback_param = desc;
    
    // 提交DMA描述符
    cookie = dmaengine_submit(dma_desc);
    if (dma_submit_error(cookie)) {
        dev_err(&priv->spi->dev, "提交RX DMA描述符失败\n");
        return -EIO;
    }
    
    // 存储cookie用于状态跟踪
    desc->dma_cookie = cookie;
    
    // DMA提交成功后，将描述符添加到pending列表
    spin_lock_irqsave(&priv->rx_desc_lock, flags);
    list_add_tail(&desc->list, &priv->rx_pending_list);
    spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    
    // 启动DMA
    dma_async_issue_pending(priv->rx_dma_chan);
    
    return 0;
}
EXPORT_SYMBOL(dw1000_submit_rx_desc);

// 重新填充RX描述符
int dw1000_refill_rx_descriptor(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc)
{
    unsigned long flags;
    int ret;
    
    if (!desc) {
        dev_err(&priv->spi->dev, "重新填充时遇到NULL描述符\n");
        return -EINVAL;
    }

    // 准备描述符以便重用
    desc->data_len = 0;
    
    // 重新提交到DMA引擎
    ret = dw1000_submit_rx_desc(priv, desc);
    if (ret) {
        dev_err(&priv->spi->dev, "重新提交RX描述符失败: %d\n", ret);
        // 提交失败，将描述符归还到free列表
        spin_lock_irqsave(&priv->rx_desc_lock, flags);
        if (!list_empty(&desc->list)) {
            list_del_init(&desc->list);
        }
        desc->in_use = false;
        list_add_tail(&desc->list, &priv->rx_free_list);
        spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
        return ret;
    }
    
    return 0;
}
EXPORT_SYMBOL(dw1000_refill_rx_descriptor);

// 设置RX DMA
int dw1000_setup_rx_dma(struct dw1000_hybrid_priv *priv)
{
    int i, ret;
    struct dw1000_rx_desc *desc;
    
    dev_info(&priv->spi->dev, "设置RX DMA描述符\n");
    
    // 初始化描述符
    ret = dw1000_setup_rx_desc(priv);
    if (ret)
        return ret;
        
    // 提交多个描述符到DMA
    for (i = 0; i < NUM_RX_DESC; i++) {
        desc = dw1000_get_free_rx_desc(priv);
        if (!desc) {
            dev_err(&priv->spi->dev, "没有可用的RX描述符\n");
            return -ENOSPC;
        }
        
        ret = dw1000_submit_rx_desc(priv, desc);
        if (ret) {
            dev_err(&priv->spi->dev, "提交RX描述符 %d 失败\n", i);
            // 提交失败时，将描述符归还到free列表
            unsigned long flags;
            spin_lock_irqsave(&priv->rx_desc_lock, flags);
            desc->in_use = false;
            list_add_tail(&desc->list, &priv->rx_free_list);
            spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
            return ret;
        }
    }
    
    priv->rx_dma_active = true;
    return 0;
}
EXPORT_SYMBOL(dw1000_setup_rx_dma);

// 拆解RX DMA
void dw1000_teardown_rx_dma(struct dw1000_hybrid_priv *priv)
{
    if (!priv->rx_dma_active)
        return;
        
    dev_info(&priv->spi->dev, "拆解RX DMA\n");
    
    // 停止所有进行中的DMA传输
    dmaengine_terminate_all(priv->rx_dma_chan);
    
    // 清理所有描述符
    dw1000_cleanup_rx_desc(priv);
    
    priv->rx_dma_active = false;
}
EXPORT_SYMBOL(dw1000_teardown_rx_dma);

// RX DMA回调
void dw1000_hybrid_dma_rx_callback(void *param)
{
    struct dw1000_rx_desc *desc = param;
    struct dw1000_hybrid_priv *priv;
    unsigned long flags;
    enum dma_status status;

    if (!desc) {
        pr_err("DW1000: RX DMA回调中遇到NULL描述符!\n");
        return;
    }
    
    priv = desc->priv;
    if (!priv) {
        pr_err("DW1000: RX DMA回调中遇到NULL私有数据! Cookie: %d\n", desc->dma_cookie);
        return;
    }

    // 检查DMA状态
    status = dmaengine_tx_status(priv->rx_dma_chan, desc->dma_cookie, NULL);
    if (status == DMA_ERROR) {
        dev_warn(&priv->spi->dev, "RX DMA传输错误\n");
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dma_errors++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        
        // DMA错误时，将描述符移回free列表
        spin_lock_irqsave(&priv->rx_desc_lock, flags);
        list_del(&desc->list);
        desc->in_use = false;
        list_add_tail(&desc->list, &priv->rx_free_list);
        spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    } else if (status == DMA_COMPLETE) {
        // DMA传输完成，同步缓存以便CPU访问
        dma_sync_single_for_cpu(&priv->spi->dev,
                               desc->dma_addr,
                               desc->buffer_size,
                               DMA_FROM_DEVICE);
        
        // 成功完成时，让NAPI处理数据
        if (napi_schedule_prep(&priv->napi)) {
            __napi_schedule(&priv->napi);
        }
    }
}
EXPORT_SYMBOL(dw1000_hybrid_dma_rx_callback); 

// --- 初始化 TX DMA 描述符 ---
static int dw1000_setup_tx_desc(struct dw1000_hybrid_priv *priv)
{
	int i;
	unsigned long flags;

	// 初始化链表头
	INIT_LIST_HEAD(&priv->tx_free_list);
	INIT_LIST_HEAD(&priv->tx_pending_list);

	// 分配并初始化所有描述符
	for (i = 0; i < NUM_TX_DESC; i++) {
		struct dw1000_tx_desc *desc = &priv->tx_desc[i];

		// 为每个描述符分配流式DMA缓冲区
		desc->buffer_size = TX_DESC_BUFFER_SIZE; // 使用定义的TX描述符缓冲区大小
		
		// 使用kmalloc分配可缓存的内存，提高CPU访问性能
		// 流式DMA + CPU缓存 = 更好的性能，特别是对于频繁的数据访问
		desc->buffer = kmalloc(desc->buffer_size, GFP_KERNEL);
		if (!desc->buffer) {
			dev_err(&priv->spi->dev, "分配TX缓冲区 %d 失败\n", i);
			goto fail_cleanup;
		}
		
		// 使用流式DMA映射，允许CPU缓存优化
		desc->dma_addr = dma_map_single(&priv->spi->dev,
										desc->buffer,
										desc->buffer_size,
										DMA_TO_DEVICE);
		if (dma_mapping_error(&priv->spi->dev, desc->dma_addr)) {
			dev_err(&priv->spi->dev, "映射TX DMA缓冲区 %d 失败\n", i);
			kfree(desc->buffer);
			desc->buffer = NULL;
			goto fail_cleanup;
		}

		desc->priv = priv; // 设置指向私有数据的指针
		desc->in_use = false;
		desc->data_len = 0;
		desc->skb = NULL;   // 初始化 skb 指针

		// 添加到空闲链表
		spin_lock_irqsave(&priv->tx_desc_lock, flags);
		list_add_tail(&desc->list, &priv->tx_free_list);
		spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
	}

	return 0;

fail_cleanup:
	// 清理已分配的描述符
	while (--i >= 0) {
		struct dw1000_tx_desc *desc = &priv->tx_desc[i];
		if (desc->buffer) {
			// 先解除DMA映射
			if (desc->dma_addr) {
				dma_unmap_single(&priv->spi->dev,
								desc->dma_addr,
								desc->buffer_size,
								DMA_TO_DEVICE);
			}
			// 释放kmalloc分配的内存
			kfree(desc->buffer);
			desc->buffer = NULL;
		}
	}
	// 清空可能已添加的列表项
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	INIT_LIST_HEAD(&priv->tx_free_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
	return -ENOMEM;
}

// --- 清理 TX DMA 描述符 ---
void dw1000_cleanup_tx_desc(struct dw1000_hybrid_priv *priv)
{
	int i;
	unsigned long flags;

	// 确保所有链表为空 (加锁保护)
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	INIT_LIST_HEAD(&priv->tx_free_list);
	INIT_LIST_HEAD(&priv->tx_pending_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);

	// 释放所有描述符缓冲区
	for (i = 0; i < NUM_TX_DESC; i++) {
		struct dw1000_tx_desc *desc = &priv->tx_desc[i];
		if (desc->buffer) {
			// 先解除DMA映射
			if (desc->dma_addr) {
				dma_unmap_single(&priv->spi->dev,
								desc->dma_addr,
								desc->buffer_size,
								DMA_TO_DEVICE);
			}
			// 释放kmalloc分配的内存
			kfree(desc->buffer);
			desc->buffer = NULL;
		}
	}
}
EXPORT_SYMBOL(dw1000_cleanup_tx_desc);
// --- Helper function to execute TX --- 
int dw1000_execute_tx(struct dw1000_hybrid_priv *priv, struct dw1000_tx_desc *desc)
{
	int ret;
	u8 tx_fctrl[RG_TX_FCTRL_LEN];
	unsigned long flags; // 用于tx_desc_lock

	// Check frame length against hardware limit (TXFLEN is 10 bits)
	if (desc->data_len == 0 || desc->data_len > 1023) { // TXFLEN最大为1023
		dev_err(&priv->spi->dev, "Invalid TX frame length for HW: %zu\n", desc->data_len);
		return -EINVAL;
	}

	// 1. Set TX_FCTRL.TXFLEN (Transmit Frame Length)
	// TXFLEN是TX_FCTRL的低9位。帧长度包括PHR和MAC PDU。
	// DW1000硬件自动追加2字节CRC。
	ret = dw1000_read_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to read TX_FCTRL: %d\n", ret);
		return ret;
	}

	tx_fctrl[0] = (u8)(desc->data_len & 0x00FF); // TXFLEN的低字节
	// 在设置TXFLEN[9:8]时保留tx_fctrl[1]中的其他位（TXBR, TR）
	tx_fctrl[1] = (tx_fctrl[1] & 0xFC) | (u8)((desc->data_len >> 8) & 0x03);

	ret = dw1000_write_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to write TX_FCTRL: %d\n", ret);
		return ret;
	}

	// 2. Sync buffer for device
	dma_sync_single_for_device(&priv->spi->dev, desc->dma_addr,
						   desc->data_len, DMA_TO_DEVICE);

	// 3. Increment pending TX count (before submitting DMA)
	atomic_inc(&priv->tx_pending_count);

	// 4. Submit DMA descriptor
	ret = dw1000_submit_tx_desc(priv, desc); // 此函数设置回调和参数
	if (ret) {
		dev_err(&priv->spi->dev, "dw1000_submit_tx_desc failed: %d\n", ret);
		atomic_dec(&priv->tx_pending_count); // 提交失败时减少计数
		return ret; // 调用者将处理描述符和SKB清理
	}

	// 5. If DMA submission was successful, add to pending list and start HW TX
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	list_add_tail(&desc->list, &priv->tx_pending_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);

	// Start hardware transmission
	ret = dw1000_write_reg32(priv, RG_SYS_CTRL, 0, SYS_CTRL_TXSTRT);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to start TX (SYS_CTRL_TXSTRT): %d\n", ret);
		// DMA已提交，回调最终将运行并处理描述符。
		// 对于TXSTRT失败，这里不执行atomic_dec和list_del，因为DMA处于活动状态。
		// 回调将减少tx_pending_count并管理描述符。
		// 记录错误并更新此特定失败的统计信息。
		spin_lock_irqsave(&priv->stats_lock, flags);
		priv->stats.tx_errors++; // 通用TX错误
		// 如果需要，添加更具体的计数器，例如 tx_start_hw_errors
		spin_unlock_irqrestore(&priv->stats_lock, flags);
		// 返回特定错误码以指示DMA提交后的失败
		return -EAGAIN; // 使用EAGAIN表示这种特定的失败模式
	}

	return 0; // Success
}
EXPORT_SYMBOL(dw1000_execute_tx);

// --- Submit TX Descriptor Function (Improved) ---
int dw1000_submit_tx_desc(struct dw1000_hybrid_priv *priv, struct dw1000_tx_desc *desc)
{
    struct dma_async_tx_descriptor *dma_desc;
    dma_cookie_t cookie;
    
    // 在DMA传输前同步缓存，确保CPU写入的数据对DMA可见
    dma_sync_single_for_device(&priv->spi->dev,
                              desc->dma_addr,
                              desc->data_len,
                              DMA_TO_DEVICE);
    
    // Prepare DMA descriptor
    dma_desc = dmaengine_prep_slave_single(priv->tx_dma_chan,
                                          desc->dma_addr,
                                          desc->data_len,
                                          DMA_MEM_TO_DEV,
                                          DMA_PREP_INTERRUPT);
    if (!dma_desc) {
        dev_err(&priv->spi->dev, "Failed to prepare TX DMA descriptor\n");
        return -EIO;
    }
    
    // Set callback with descriptor as parameter
    dma_desc->callback = dw1000_hybrid_dma_tx_callback;
    dma_desc->callback_param = desc;
    
    // Submit DMA descriptor
    cookie = dmaengine_submit(dma_desc);
    if (dma_submit_error(cookie)) {
        dev_err(&priv->spi->dev, "Failed to submit TX DMA descriptor\n");
        return -EIO;
    }
    
    // Store cookie for status tracking
    desc->dma_cookie = cookie;
    
    // Start DMA
    dma_async_issue_pending(priv->tx_dma_chan);
    
    return 0;
}
EXPORT_SYMBOL(dw1000_submit_tx_desc);

// --- 获取一个空闲 TX 描述符 ---
struct dw1000_tx_desc *dw1000_get_free_tx_desc(struct dw1000_hybrid_priv *priv)
{
	struct dw1000_tx_desc *desc = NULL;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	if (!list_empty(&priv->tx_free_list)) {
		desc = list_first_entry(&priv->tx_free_list, struct dw1000_tx_desc, list);
		list_del(&desc->list);
		desc->in_use = true;
		desc->data_len = 0;
		desc->skb = NULL;
		// 注意：不在这里加入 pending_list，在成功提交 DMA 后再加入
	}
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
	
	return desc;
}
EXPORT_SYMBOL(dw1000_get_free_tx_desc);

// --- 将出错的 TX 描述符放回空闲列表 ---
void dw1000_put_tx_desc_on_error(struct dw1000_tx_desc *desc)
{
	if (!desc || !desc->priv) return;
	struct dw1000_hybrid_priv *priv = desc->priv;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	desc->in_use = false;
	// 确保它不在任何列表上（如果之前添加到了pending_list）
	if (!list_empty(&desc->list)) {
		list_del_init(&desc->list);
	}
	list_add_tail(&desc->list, &priv->tx_free_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
}
EXPORT_SYMBOL(dw1000_put_tx_desc_on_error);
