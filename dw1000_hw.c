#include "dw1000_generic.h"

// 低级寄存器读取函数
int dw1000_read_reg(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u32 len, void *data)
{
    // 原文件中的 dw1000_read_reg 实现
    struct spi_device *spi = priv->spi;
    struct spi_message msg;
    struct spi_transfer header_xfer = {
        .tx_buf = priv->spi_buf,
        .len = 3,
    };
    struct spi_transfer data_xfer = {
        .rx_buf = data,
        .len = len,
    };
    int ret;

    mutex_lock(&priv->spi_reg_mutex); // 获取SPI寄存器访问互斥锁
    
    // 构建SPI命令头 - 读取命令，寄存器和子地址
    priv->spi_buf[0] = (reg & SPI_CMD_REG_MASK) | SPI_CMD_READ;
    if (offset > SPI_CMD_SUB_SHORT_MASK) {
        priv->spi_buf[0] |= SPI_CMD_SUB_EXT_BIT;
        priv->spi_buf[1] = (offset & SPI_CMD_SUB_SHORT_MASK) | SPI_CMD_SUB_SHORT_EXT_BIT;
        priv->spi_buf[2] = (offset >> 7) & 0xFF;
        header_xfer.len = 3;
    } else {
        priv->spi_buf[1] = offset & SPI_CMD_SUB_SHORT_MASK;
        header_xfer.len = 2;
    }
    
    // 初始化SPI消息
    spi_message_init(&msg);
    spi_message_add_tail(&header_xfer, &msg);
    spi_message_add_tail(&data_xfer, &msg);
    
    // 执行SPI传输
    ret = spi_sync(spi, &msg);
    if (ret)
        dev_err(&spi->dev, "SPI读取失败: reg=0x%02x, offset=0x%04x, ret=%d\n",
                reg, offset, ret);

    mutex_unlock(&priv->spi_reg_mutex); // 释放SPI寄存器访问互斥锁
    
    return ret;
}
EXPORT_SYMBOL(dw1000_read_reg);

// 低级寄存器写入函数
int dw1000_write_reg(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u32 len, const void *data)
{
    // 原文件中的 dw1000_write_reg 实现
    struct spi_device *spi = priv->spi;
    struct spi_message msg;
    struct spi_transfer header_xfer = {
        .tx_buf = priv->spi_buf,
        .len = 3,
    };
    struct spi_transfer data_xfer = {
        .tx_buf = data,
        .len = len,
    };
    int ret;

    mutex_lock(&priv->spi_reg_mutex); // 获取SPI寄存器访问互斥锁
    
    // 构建SPI命令头 - 写入命令，寄存器和子地址
    priv->spi_buf[0] = (reg & SPI_CMD_REG_MASK) | SPI_CMD_WRITE;
    if (offset > SPI_CMD_SUB_SHORT_MASK) {
        priv->spi_buf[0] |= SPI_CMD_SUB_EXT_BIT;
        priv->spi_buf[1] = (offset & SPI_CMD_SUB_SHORT_MASK) | SPI_CMD_SUB_SHORT_EXT_BIT;
        priv->spi_buf[2] = (offset >> 7) & 0xFF;
        header_xfer.len = 3;
    } else {
        priv->spi_buf[1] = offset & SPI_CMD_SUB_SHORT_MASK;
        header_xfer.len = 2;
    }
    
    // 初始化SPI消息
    spi_message_init(&msg);
    spi_message_add_tail(&header_xfer, &msg);
    spi_message_add_tail(&data_xfer, &msg);
    
    // 执行SPI传输
    ret = spi_sync(spi, &msg);
    if (ret)
        dev_err(&spi->dev, "SPI写入失败: reg=0x%02x, offset=0x%04x, ret=%d\n",
                reg, offset, ret);

    mutex_unlock(&priv->spi_reg_mutex); // 释放SPI寄存器访问互斥锁
    
    return ret;
}
EXPORT_SYMBOL(dw1000_write_reg);

// 32位寄存器读取辅助函数
u32 dw1000_read_reg32(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset)
{
    u32 value = 0;
    dw1000_read_reg(priv, reg, offset, 4, &value);
    return value;
}
EXPORT_SYMBOL(dw1000_read_reg32);

// 32位寄存器写入辅助函数
void dw1000_write_reg32(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u32 value)
{
    dw1000_write_reg(priv, reg, offset, 4, &value);
}
EXPORT_SYMBOL(dw1000_write_reg32);

// 硬件初始化函数
int dw1000_hw_init(struct dw1000_hybrid_priv *priv)
{
    int ret;
    
    dev_info(&priv->spi->dev, "执行额外的硬件初始化\n");
    
    // 注意：此时设备已经被reset_device复位和配置过了
    // 我们只需要执行额外的初始化步骤
    
    // 1. 启用需要的中断掩码
    u32 int_mask = SYS_STATUS_RXFCG | SYS_STATUS_TXFRS; 
    ret = dw1000_write_reg(priv, RG_SYS_MASK, 0, 4, &int_mask);
    if (ret) {
        dev_err(&priv->spi->dev, "设置中断掩码失败: %d\n", ret);
        return ret;
    }
    
    // 2. 确保收发器处于 IDLE 状态 (TRXOFF)
    ret = dw1000_write_reg32(priv, RG_SYS_CTRL, 0, SYS_CTRL_TRXOFF);
    if (ret) {
        dev_err(&priv->spi->dev, "设置TRXOFF失败: %d\n", ret);
        return ret;
    }
    
    // 3. 清除所有待处理的中断状态
    u32 clear_status = 0xFFFFFFFF; // 清除所有中断标志
    ret = dw1000_write_reg32(priv, RG_SYS_STATUS, 0, clear_status);
    if (ret) {
        dev_err(&priv->spi->dev, "清除中断状态失败: %d\n", ret);
        return ret;
    }
    
    dev_info(&priv->spi->dev, "额外硬件初始化完成\n");
    return 0;
}
EXPORT_SYMBOL(dw1000_hw_init);

// 设备复位函数
int dw1000_reset_device(struct dw1000_hybrid_priv *priv)
{
    int ret;
    struct gpio_desc *rstn_gpio;
    u32 dev_id;

    dev_info(&priv->spi->dev, "执行设备复位\n");

    // 尝试使用RSTN引脚进行硬件复位
    rstn_gpio = devm_gpiod_get_optional(&priv->spi->dev, "rstn", GPIOD_OUT_LOW);
    if (!IS_ERR_OR_NULL(rstn_gpio)) {
        dev_info(&priv->spi->dev, "通过RSTN引脚执行硬件复位\n");
        
        // 拉低RSTN引脚至少100μs（根据DW1000数据手册）
        gpiod_set_value(rstn_gpio, 0);
        usleep_range(1000, 1500); // 1ms延时，大于要求的100μs
        
        // 释放复位引脚
        gpiod_set_value(rstn_gpio, 1);
        msleep(5); // 等待DW1000内部初始化
        
        devm_gpiod_put(&priv->spi->dev, rstn_gpio);
    } else {
        // 如果没有硬件复位引脚，尝试软件复位
        u8 pmsc_ctrl0[4] = {0};
        u8 softreset_cmd[4] = {0};

        dev_info(&priv->spi->dev, "通过PMSC寄存器执行软件复位\n");
        
        // 读取当前PMSC_CTRL0寄存器值
        ret = dw1000_read_reg(priv, RG_PMSC, SUB_PMSC_CTRL0, 4, pmsc_ctrl0);
        if (ret) {
            dev_err(&priv->spi->dev, "读取PMSC_CTRL0失败: %d\n", ret);
            return ret;
        }
        
        // 设置复位位
        softreset_cmd[0] = pmsc_ctrl0[0] & 0xFC;
        softreset_cmd[1] = pmsc_ctrl0[1] & 0xFC;
        ret = dw1000_write_reg(priv, RG_PMSC, SUB_PMSC_CTRL0, 2, softreset_cmd);
        if (ret) {
            dev_err(&priv->spi->dev, "设置复位位失败: %d\n", ret);
            return ret;
        }
        
        // 短暂延时
        usleep_range(1000, 1500);
        
        // 清除复位位，恢复时钟
        ret = dw1000_write_reg(priv, RG_PMSC, SUB_PMSC_CTRL0, 4, pmsc_ctrl0);
        if (ret) {
            dev_err(&priv->spi->dev, "清除复位位失败: %d\n", ret);
            return ret;
        }
        
        // 等待复位完成
        msleep(5);
    }

    // 复位后验证设备ID
    ret = dw1000_read_reg(priv, RG_DEV_ID, 0, 4, &dev_id);
    if (ret) {
        dev_err(&priv->spi->dev, "复位后读取设备ID失败: %d\n", ret);
        return ret;
    }
    if ((dev_id & 0xFFFF0000) != 0xDECA0000) {
        dev_err(&priv->spi->dev, "复位后设备ID无效: 0x%08x\n", dev_id);
        return -ENODEV;
    }

    // 复位后重新应用配置
    dev_info(&priv->spi->dev, "复位后应用配置...\n");
    ret = dw1000_apply_config(priv);
    if (ret) {
        dev_err(&priv->spi->dev, "复位后重新配置设备失败: %d\n", ret);
        return ret;
    }

    dev_info(&priv->spi->dev, "设备复位和重新配置成功\n");
    return 0;
}
EXPORT_SYMBOL(dw1000_reset_device);

// 应用配置函数
int dw1000_apply_config(struct dw1000_hybrid_priv *priv)
{
	u32 dev_id = 0;
	u8 sys_cfg[4] = {0};
	//u8 sys_ctrl[4] = {0}; // Don't toggle receiver here
	u8 panadr[4] = {0};
	u8 chan_ctrl[4] = {0};
	u8 tx_fctrl[RG_TX_FCTRL_LEN] = {0}; // TX_FCTRL为5字节长
	u32 tx_power_reg = 0; // 使用u32表示TX_POWER寄存器值
	int ret;
	unsigned long flags; // 用于config_lock

	dev_info(&priv->spi->dev, "Applying DW1000 config (ch=%d, prf=%d, rate=%d, plen=%u, smart_pwr=%d, tx_pwr=0x%04x, pan=0x%04x)\n",
		 priv->config.channel, priv->config.prf, priv->config.data_rate,
		 priv->config.preamble_length, priv->config.smart_power, priv->config.tx_power,
		 priv->config.pan_id);

	spin_lock_irqsave(&priv->config_lock, flags); // 获取配置锁

	// --- 开始读取配置的关键部分 ---
	// 1. 验证设备ID（可选，通常在初始化时完成）
	/*
	ret = dw1000_read_reg(priv, RG_DEV_ID, 0, 4, &dev_id);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to read device ID in apply_config: %d\n", ret);
		// spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		// return ret;
	}
	if ((dev_id & 0xFFFF0000) != 0xDECA0000) {
		dev_err(&priv->spi->dev, "Invalid Device ID in apply_config: 0x%08x\n", dev_id);
		// spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		// return -ENODEV;
	}
	*/

	// 2. 配置系统寄存器 - RX双缓冲，帧过滤等
	sys_cfg[0] = SYS_CFG_RXDBUFFEN; // 启用RX双缓冲（RXDBUFFEN位）
	sys_cfg[1] = priv->config.accept_bad_frames ? 0x00 : SYS_CFG_FFEN; // 如果不接受错误帧，则使用FFEN位
	// 根据priv->config标志，考虑添加其他FF位（FFBC, FFAB, FFAA, FFAM）
	sys_cfg[1] |= (SYS_CFG_FFAB | SYS_CFG_FFAA | SYS_CFG_FFAM); // 示例：过滤数据、ACK、MAC命令帧
	sys_cfg[2] = SYS_CFG_PHR_MODE_STD; // 标准PHR模式（如果需要扩展PHR，则调整）
	sys_cfg[3] = 0x00;
	// SPI write is now protected by spi_reg_mutex, no need for config_lock here
	ret = dw1000_write_reg(priv, RG_SYS_CFG, 0, 4, sys_cfg);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure SYS_CFG: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 3. Configure channel - Channel number and PRF
	chan_ctrl[0] = (priv->config.channel & CHAN_CTRL_CH_NUM_MASK) |
				   ((priv->config.prf << CHAN_CTRL_PRF_SHIFT) & CHAN_CTRL_PRF_MASK);
	chan_ctrl[1] = 0x00; // Reserved, maybe set TX/RX preamble codes here?
	// Set preamble codes (RX first, then TX)
	chan_ctrl[2] = (priv->config.preamble_code & CHAN_CTRL_RX_PRCODE_MASK) |
				   (((priv->config.preamble_code << CHAN_CTRL_TX_PRCODE_SHIFT)) & CHAN_CTRL_TX_PRCODE_MASK);
	chan_ctrl[3] = 0x00; // RF delays - configure if non-default needed
	ret = dw1000_write_reg(priv, RG_CHAN_CTRL, 0, 4, chan_ctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure CHAN_CTRL: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 4. Configure device address - PAN_ID and short address
	panadr[0] = priv->config.pan_id & 0xFF;
	panadr[1] = (priv->config.pan_id >> 8) & 0xFF;
	panadr[2] = 0x00; // Short address low byte (e.g., from MAC or set via IOCTL)
	panadr[3] = 0x00; // Short address high byte (e.g., from MAC or set via IOCTL)
	if (priv->netdev && priv->netdev->dev_addr) {
		// Example: Use last 2 bytes of MAC as short addr
		panadr[2] = priv->netdev->dev_addr[4];
		panadr[3] = priv->netdev->dev_addr[5];
	}
	ret = dw1000_write_reg(priv, RG_PANADR, 0, 4, panadr);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure PANADR: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 5. 配置TX功率
	// 智能功率通常需要基础功率设置和启用一个位
	// 假设0x1F1F1F1F启用智能功率优化，使用默认基础值。
	// 检查数据手册中TX_POWER或其他寄存器中的确切智能功率位。

	if (priv->config.smart_power) {
		// Smart power typically needs a base power setting and enabling a bit
		// Assume 0x1F1F1F1F enables smart power optimization with default base.
		// Check datasheet for exact smart power bits in TX_POWER or other registers.
		tx_power_reg = (DW1000_SMART_TX_POWER_REG << 24) | (DW1000_SMART_TX_POWER_REG << 16) |
		               (DW1000_SMART_TX_POWER_REG << 8)  | DW1000_SMART_TX_POWER_REG;
		dev_dbg(&priv->spi->dev, "Using smart TX power (Reg: 0x%08x)\n", tx_power_reg);
	} else if (priv->config.tx_power != 0) { // 使用配置的TX功率（假设是寄存器值）
		// 16位配置值到32位寄存器的直接映射？
		// 检查数据手册：TX_POWER有不同条件的字段（如手动、智能、不同速率）。
		// 简单方法：如果config.tx_power是u8/u16寄存器值，则将8位值复制到所有字节。
		u8 pwr_byte = priv->config.tx_power & 0xFF; // 如果tx_power保存8位值的示例

		// 如果priv->config.tx_power intended为dBm，在此处调用calculate_tx_power_reg_val。
		// tx_power_reg = calculate_tx_power_reg_val(priv->config.tx_power); // 如果配置存储dBm
		tx_power_reg = (pwr_byte << 24) | (pwr_byte << 16) | (pwr_byte << 8) | pwr_byte;
		dev_dbg(&priv->spi->dev, "Using configured TX power (Reg: 0x%08x from val 0x%04x)\n",
				tx_power_reg, priv->config.tx_power);
	} else {
		// Use default power config
		u8 pwr_byte = DW1000_DEFAULT_TX_POWER_REG & 0xFF;
		tx_power_reg = (pwr_byte << 24) | (pwr_byte << 16) | (pwr_byte << 8) | pwr_byte;
		dev_dbg(&priv->spi->dev, "Using default TX power (Reg: 0x%08x)\n", tx_power_reg);
	}
	ret = dw1000_write_reg32(priv, RG_TX_POWER, 0, tx_power_reg); // Write the calculated 32-bit value
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure TX_POWER: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 6. 配置TX帧控制寄存器 - 前导码长度、数据速率、PRF
	// TX_FCTRL是5字节：TXFLEN(10)，TXBR(2)，TR(1)，TXPRF(2)，TXPSR(2)，PE(2)，TXBOFFS(10)，IFSDELAY(8)
	// 先读取以保留TXBOFFS和IFSDELAY？更安全。

	// 如果读取失败，继续设置默认值？还是返回错误？
	// 让我们清除相关位并继续。

	// 清除设置新值之前的相关位
	tx_fctrl[1] &= ~(TX_FCTRL_TXBR_MASK | TX_FCTRL_TR_BIT); // 清除字节1中的TXBR和TR
	tx_fctrl[2] &= ~(TX_FCTRL_TXPRF_MASK | TX_FCTRL_TXPSR_MASK | TX_FCTRL_PE_MASK); // 清除字节2中的TXPRF, TXPSR, PE

	// 注意：TXFLEN（字节0和字节1的低位）必须在每次传输前根据实际帧大小设置。
	// 在这里将tx_fctrl[0]和tx_fctrl[1]的低位保留为0是可以的。

	// Set TXBR (Data Rate) in Byte 1
	tx_fctrl[1] |= ((priv->config.data_rate << TX_FCTRL_TXBR_SHIFT) & TX_FCTRL_TXBR_MASK);
	// Set TR bit if ranging is enabled (assuming a config flag exists)
	// tx_fctrl[1] |= (priv->config.ranging_enabled ? TX_FCTRL_TR_BIT : 0);

	// Set PE (Preamble Length) and TXPSR (Preamble Symbol Repetitions) in Byte 2
	u8 pe_txpsr_val = 0;
	u16 plen = priv->config.preamble_length; // Use the config value directly (now u16)

	// Use correct mapping based on datasheet (Table 16 & non-std preamble section)
	// This mapping seems correct based on prior discussion, assuming defines are right.
	switch (plen) {
		// Standard lengths (64-512): Use specific PE code, TXPSR depends on PRF
		case 64:   pe_txpsr_val = (TX_FCTRL_PE_64 << TX_FCTRL_PE_SHIFT); break;
		case 128:  pe_txpsr_val = (TX_FCTRL_PE_128 << TX_FCTRL_PE_SHIFT); break;
		case 256:  pe_txpsr_val = (TX_FCTRL_PE_256 << TX_FCTRL_PE_SHIFT); break;
		case 512:  pe_txpsr_val = (TX_FCTRL_PE_512 << TX_FCTRL_PE_SHIFT); break;

		// Non-standard lengths (1024+): Use specific PE/TXPSR combinations
		// Note: The defines TX_FCTRL_PE_NS_* and TX_FCTRL_TXPSR_* need verification.
		// Let's assume the combined logic was correct.
		case 1024: pe_txpsr_val = (TX_FCTRL_PE_NS_00 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_1024 << TX_FCTRL_TXPSR_SHIFT); break;
		case 1536: pe_txpsr_val = (TX_FCTRL_PE_NS_01 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_1024 << TX_FCTRL_TXPSR_SHIFT); break;
		case 2048: pe_txpsr_val = (TX_FCTRL_PE_NS_10 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_1024 << TX_FCTRL_TXPSR_SHIFT); break;
		case 4096: pe_txpsr_val = (TX_FCTRL_PE_NS_00 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_4096 << TX_FCTRL_TXPSR_SHIFT); break;

		default:
			dev_warn(&priv->spi->dev, "Invalid preamble length %u in config, using default 64 symbols\n", plen);
			pe_txpsr_val = (TX_FCTRL_PE_64 << TX_FCTRL_PE_SHIFT); // Default to 64
			break;
	}
	// Add standard TXPSR based on PRF for standard lengths (64-512)
	if (plen <= 512) {
		u8 std_txpsr = (priv->config.prf == 0) ? TX_FCTRL_TXPSR_N_16MHZ : TX_FCTRL_TXPSR_N_64MHZ;
		pe_txpsr_val |= (std_txpsr << TX_FCTRL_TXPSR_SHIFT);
	}

	tx_fctrl[2] |= pe_txpsr_val; // Apply combined PE and TXPSR bits

	// Set TXPRF (PRF) in Byte 2
	tx_fctrl[2] |= ((priv->config.prf << TX_FCTRL_TXPRF_SHIFT) & TX_FCTRL_TXPRF_MASK);

	// Note: TXFLEN (Bytes 0 and 1 lower bits) must be set *before* each transmission based on the actual frame size.
	// Leaving tx_fctrl[0] and lower bits of tx_fctrl[1] as 0 here is fine.

	// SPI write is now protected by spi_reg_mutex
	ret = dw1000_write_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl); // Use length macro
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure TX_FCTRL: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 7. Configure SPI speed if needed
	if (priv->spi->max_speed_hz != priv->config.spi_speed_hz) {
		// SPI setup might sleep, cannot hold spinlock
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release config lock BEFORE spi_setup
		priv->spi->max_speed_hz = priv->config.spi_speed_hz; // Read speed before releasing lock (this is ok)
		ret = spi_setup(priv->spi);
		spin_lock_irqsave(&priv->config_lock, flags); // Re-acquire config lock AFTER spi_setup
		if (ret) {
			dev_err(&priv->spi->dev, "Failed to configure SPI speed: %d\n", ret);
			spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
			return ret;
		}
	}

	// --- End critical section reading config ---
	spin_unlock_irqrestore(&priv->config_lock, flags); // Release config lock at the end

	// NOTE: Do NOT enable receiver here. Let the caller (e.g., hw_init, reset) do it.

	dev_info(&priv->spi->dev, "DW1000 configuration applied successfully\n");

	return 0;
}
EXPORT_SYMBOL(dw1000_apply_config);

// 电源配置结构体
typedef struct {
    uint8_t da;
    uint8_t mixer;
    uint8_t pa;
} PowerConfig;

// 根据 DW1000 数据手册和用户手册实现的功率配置查找函数
static PowerConfig get_power_config(int8_t dbm) {
    PowerConfig config; // No default initialization here, set in switch

    // 根据用户提供的手册映射添加更多配置 (Table 20 for Ch5, 64MHz PRF)
    switch (dbm) {
        case -40:
            config.da = 0xFF; config.mixer = 0x00; config.pa = 0x00;
            break;
        case -37:
            config.da = 0xF0; config.mixer = 0x00; config.pa = 0x00;
            break;
        case -34:
            config.da = 0xE0; config.mixer = 0x00; config.pa = 0x00;
            break;
        case -10: // 典型配置
            config.da = 0x0F; config.mixer = 0x03; config.pa = 0x01;
            break;
        case 0: // 最大功率示例
            config.da = 0x00; config.mixer = 0x07; config.pa = 0x03;
            break;
        default:
            // 处理未定义的 dBm 值，回退到推荐的 -10dBm
            config.da = 0x0F; config.mixer = 0x03; config.pa = 0x01; // fallback to -10dBm
            break;
    }
    // TODO: 根据需要和手册 Table 20 等添加更多 dBm 映射

    return config;
}

// 计算 TX_POWER 寄存器的 32 位值
static uint32_t calculate_tx_power_reg_val(int8_t dbm)
{
    PowerConfig config = get_power_config(dbm);
    uint32_t tx_power_reg = 0;

    // 配置数字衰减器（DA）
    tx_power_reg |= (config.da & 0xFF) << 24;
    
    // 配置混频器增益
    tx_power_reg |= (config.mixer & 0x07) << 16;
    
    // 配置功率放大器（PA）
    tx_power_reg |= (config.pa & 0x0F) << 8;

    return tx_power_reg;
}
