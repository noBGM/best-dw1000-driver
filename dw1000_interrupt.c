#include "dw1000_generic.h"

// 定义可处理的中断状态位
#define HANDLED_STATUS_BITS (SYS_STATUS_RXFCG | SYS_STATUS_TXFRS | \
                             SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO | \
                             SYS_STATUS_RXSFDTO | SYS_STATUS_RXPHE | \
                             SYS_STATUS_RXPHD | SYS_STATUS_RXFCE)

// NAPI轮询函数
static int dw1000_hybrid_poll(struct napi_struct *napi, int budget)
{
    struct dw1000_hybrid_priv *priv = container_of(napi, struct dw1000_hybrid_priv, napi);
    unsigned long flags;
    int processed = 0;
    u32 processed_status = 0;
    u32 sys_status;
    struct dw1000_rx_desc *desc, *tmp;

    // 读取系统状态寄存器
    dw1000_read_reg(priv, RG_SYS_STATUS, 0, 4, &sys_status);

    // 检查是否有需要处理的中断
    if (!(sys_status & HANDLED_STATUS_BITS)) {
        napi_complete_done(napi, 0);
        return 0;
    }

    // 遍历所有pending的RX描述符
    list_for_each_entry_safe(desc, tmp, &priv->rx_pending_list, list) {
        if (processed >= budget)
            break;

        // 处理接收到的帧
        if (sys_status & SYS_STATUS_RXFCG) {
            int ret = process_rx_frame(priv, desc);
            if (ret == 0) {
                processed++;
                processed_status |= SYS_STATUS_RXFCG;
            }
        }

        // 处理其他错误状态
        if (sys_status & (SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO | 
                          SYS_STATUS_RXSFDTO | SYS_STATUS_RXPHE | 
                          SYS_STATUS_RXPHD | SYS_STATUS_RXFCE)) {
            // 记录错误统计
            spin_lock_irqsave(&priv->stats_lock, flags);
            if (sys_status & SYS_STATUS_RXRFTO)
                priv->stats.rx_rf_timeout++;
            if (sys_status & SYS_STATUS_RXPTO)
                priv->stats.rx_preamble_timeout++;
            if (sys_status & SYS_STATUS_RXSFDTO)
                priv->stats.rx_sfd_timeout++;
            if (sys_status & SYS_STATUS_RXPHE)
                priv->stats.rx_header_error++;
            if (sys_status & SYS_STATUS_RXPHD)
                priv->stats.rx_data_error++;
            if (sys_status & SYS_STATUS_RXFCE)
                priv->stats.rx_fcs_error++;
            spin_unlock_irqrestore(&priv->stats_lock, flags);

            processed_status |= (sys_status & (SYS_STATUS_RXRFTO | 
                                               SYS_STATUS_RXPTO | 
                                               SYS_STATUS_RXSFDTO | 
                                               SYS_STATUS_RXPHE | 
                                               SYS_STATUS_RXPHD | 
                                               SYS_STATUS_RXFCE));
        }

        // 处理发送完成
        if (sys_status & SYS_STATUS_TXFRS) {
            processed_status |= SYS_STATUS_TXFRS;
            // 可以添加发送完成后的处理逻辑
        }

        // 重新提交描述符
        dw1000_refill_rx_descriptor(priv, desc);
    }

    // 清除已处理的状态位
    if (processed_status) {
        dw1000_write_reg32(priv, RG_SYS_STATUS, 0, processed_status);
    }

    // 如果处理完所有事件，完成NAPI调度
    if (processed < budget) {
        napi_complete_done(napi, processed);
    }

    return processed;
}

irqreturn_t dw1000_hybrid_irq_handler(int irq, void *dev_id)
{
    struct dw1000_hybrid_priv *priv = dev_id;
    u32 sys_status;
    
    // 读取系统状态寄存器
    dw1000_read_reg(priv, RG_SYS_STATUS, 0, 4, &sys_status);
    
    // 检查是否是我们的中断
    if (!(sys_status & HANDLED_STATUS_BITS)) {
        return IRQ_NONE;
    }
    
    // 禁用中断，启动NAPI轮询
    disable_irq_nosync(priv->irq_num);
    napi_schedule(&priv->napi);
    
    return IRQ_HANDLED;
}
EXPORT_SYMBOL(dw1000_hybrid_irq_handler); 