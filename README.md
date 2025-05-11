# Qorvo DW1000 Generic Hybrid Linux Driver

## 1. 概述 (Overview)

本驱动是为 Qorvo DW1000 超宽带 (UWB) 无线收发器芯片设计的 Linux 内核模块。它采用“混合”设计，同时提供了标准的 Linux 网络设备接口和字符设备接口，以满足不同的应用需求，例如通用的 IEEE 802.15.4 网络通信、原始数据包收发、传感器数据采集以及高精度的测距应用。

驱动支持通过 SPI 与 DW1000 芯片通信，并利用 DMA 进行高效的数据传输。

## 2. 主要特性 (Key Features)

*   **DW1000 芯片支持**: 针对 Qorvo DW1000 UWB 收发器。
*   **混合接口**:
    *   **网络设备接口**: 暴露为标准的网络设备 (如 `dw0`)，支持 IEEE 802.15.4 协议 (ARPHRD_IEEE802154)，可用于 IP 网络通信。
    *   **字符设备接口**: 暴露为 `/dev/dw1000_sensorX`，用于：
        *   设备配置和控制 (通过 IOCTLs)。
        *   原始/传感器数据帧的读写。
        *   通过 `mmap` 实现对传感器数据接收缓冲区的零拷贝访问。
*   **SPI 通信**: 通过 SPI 接口与 DW1000 芯片进行命令和数据交互。
*   **DMA 支持**:
    *   使用 DMA 引擎进行 RX 和 TX 数据传输，减轻 CPU 负载。
    *   支持基于描述符的 DMA 操作。
*   **中断处理与 NAPI**:
    *   通过 GPIO 引脚处理 DW1000 硬件中断。
    *   使用 NAPI (New API) 进行高效的网络数据包接收处理。
*   **帧处理**:
    *   能够处理不同类型的自定义帧 (例如，网络帧、传感数据帧)。
    *   从硬件寄存器读取接收帧质量信息。
*   **动态配置**:
    *   支持通过 IOCTL 命令在运行时修改设备参数，如信道、数据速率、PAN ID 等。
    *   支持硬件复位。
*   **设备树支持**:
    *   通过设备树 (Device Tree) 进行硬件资源配置和驱动参数初始化。
*   **统计信息**:
    *   提供网络接口统计 (通过 `ndo_get_stats64`)。
    *   提供更详细的驱动和硬件统计信息 (通过 `DW1000_IOC_GET_STATS` IOCTL)。

## 3. 驱动架构 (Driver Architecture)

驱动主要由以下几个核心组件构成：

*   **Core (`dw1000_core.c`)**: 负责驱动的探测 (probe) 与移除 (remove) 逻辑，初始化网络设备和字符设备，管理设备私有数据结构，并处理设备树属性。
*   **Hardware Control (`dw1000_hw.c`)**: 包含与 DW1000 芯片寄存器直接交互的函数，如读写寄存器、设备复位、应用配置等。
*   **DMA Management (`dw1000_dma.c`)**: 管理 RX 和 TX 的 DMA 描述符和通道，处理 DMA 提交和回调。
*   **Interrupt Handling (`dw1000_interrupt.c`)**: 实现中断服务程序 (ISR) 和 NAPI 轮询函数，用于处理硬件中断和调度数据包接收。
*   **Frame Processing (`dw1000_frame_processing.c`)**: 包含解析和处理从 DW1000 接收到的不同类型数据帧的逻辑。
*   **Header (`dw1000_generic.h`)**: 定义了驱动使用的数据结构、常量、IOCTL 命令以及函数原型。

## 4. 硬件需求 (Hardware Requirements)

*   Qorvo DW1000 UWB 收发器芯片或模块。
*   连接到主处理器的 SPI 接口。
*   一个 GPIO 引脚用于 DW1000 的中断信号。
*   (可选) 一个 GPIO 引脚用于 DW1000 的硬件复位 (RSTn)。

## 5. 软件依赖 (Software Dependencies)

*   Linux 内核 (针对版本 4.14.x 及以上版本开发和测试，但可能兼容其他版本)。
*   内核需启用以下子系统：
    *   SPI子系统
    *   DMA引擎子系统
    *   GPIO子系统
    *   设备树 (Device Tree) 支持

## 6. 设备树配置 (Device Tree Configuration)

驱动依赖设备树来配置硬件资源。一个设备树片段示例 (`dw1000_device.dtsi`) 如下：

```dts
// 在您的主设备树文件中，将此节点作为 SPI 控制器的子节点
// &spi0 { // 用您的 SPI 控制器节点替换
//     dw1000_node: dw1000@0 { // "0" 是片选索引
//         compatible = "qorvo,dw1000-generic-hybrid", "qorvo,dw1000-generic";
//         reg = <0>; // SPI 片选索引
//         spi-max-frequency = <8000000>; // SPI 时钟频率, e.g., 8MHz

//         // DMA 使用的 SPI FIFO 物理地址 (必须根据平台硬件设置)
//         qorvo,spi-tx-fifo-phys = <0xPHYS_ADDR_SPI_TX_FIFO>;
//         qorvo,spi-rx-fifo-phys = <0xPHYS_ADDR_SPI_RX_FIFO>;

//         interrupt-parent = <&gpio_controller>; // GPIO 控制器句柄
//         interrupts = <PIN_NUMBER IRQ_TYPE_EDGE_RISING>; // GPIO 引脚号和触发类型

//         // 可选的复位引脚
//         rstn-gpios = <&gpio_controller PIN_NUMBER_RSTN GPIO_ACTIVE_LOW>;

//         status = "okay";
//     };
// };
```

**关键设备树属性:**

*   `compatible`: 必须包含 `"qorvo,dw1000-generic-hybrid"` 或 `"qorvo,dw1000-generic"`。
*   `reg`: SPI 设备的片选 (Chip Select) 索引。
*   `spi-max-frequency`: SPI 通信的最大时钟频率。
*   `qorvo,spi-tx-fifo-phys`: SPI 控制器的 TX FIFO 物理地址 (用于 DMA)。
*   `qorvo,spi-rx-fifo-phys`: SPI 控制器的 RX FIFO 物理地址 (用于 DMA)。
*   `interrupt-parent`: 指向中断所连接的 GPIO 控制器的句柄。
*   `interrupts`: 指定中断使用的 GPIO 引脚号和中断触发类型 (例如 `IRQ_TYPE_EDGE_RISING`)。
*   `rstn-gpios` (可选): 指定用于硬件复位的 GPIO 引脚。
*   其他 `qorvo,default-*` 属性可用于设置驱动初始化时的默认配置值。

请参考 `dw1000_device.dtsi` 文件获取更详细的说明和用法。

## 7. 编译驱动 (Building the Driver)

1.  **配置内核**: 确保您的 Linux 内核已正确配置并编译，支持 SPI、DMA、GPIO。
2.  **获取源码**: 将驱动源码放置在您的文件系统中。
3.  **Kconfig**: 驱动依赖一个 Kconfig 选项 `CONFIG_DW1000_GENERIC_HYBRID`。您需要在内核的 `menuconfig` (通常在 `drivers/spi/Kconfig` 或类似位置添加) 中启用此选项：
    ```kconfig
    config DW1000_GENERIC_HYBRID
        tristate "Qorvo DW1000 Generic Hybrid UWB driver"
        depends on SPI_MASTER && GPIOLIB && OF && DMA_ENGINE
        help
          Support for Qorvo DW1000 UWB transceiver with hybrid
          (netdev and chardev) interface.
    ```
4.  **Makefile**: 驱动包含一个 `Makefile`。
5.  **编译**:
    ```bash
    make -C /path/to/your/kernel/source M=$(pwd) modules
    ```
    其中 `/path/to/your/kernel/source` 是您的内核源码树路径，`$(pwd)` 是驱动源码所在的当前目录。
    编译成功后，会生成 `dw1000_generic_hybrid.ko` 内核模块文件。

## 8. 加载驱动 (Loading the Driver)

编译成功后，可以使用以下命令加载驱动模块：

```bash
sudo insmod dw1000_generic_hybrid.ko
```

或者，如果已将模块安装到标准的内核模块目录：

```bash
sudo modprobe dw1000_generic_hybrid
```

加载成功后，您应该可以在内核日志 (`dmesg`) 中看到驱动的初始化信息。

## 9. 接口说明 (Interfaces)

### 9.1. 网络接口 (Network Interface)

*   **设备名**: 通常为 `dwX` (例如 `dw0`)。
*   **类型**: IEEE 802.15.4 (`ARPHRD_IEEE802154`)。
*   **操作**: 可以使用标准的 Linux 网络工具进行操作，例如：
    *   查看接口: `ip link show dw0`
    *   启用/禁用接口: `sudo ip link set dw0 up/down`
    *   配置 IP 地址 (如果用于 6LoWPAN 等): `sudo ip addr add <address>/<prefix> dev dw0`
*   **MTU**: 默认为 127 字节 (由 `DW1000_DEFAULT_MTU` 定义)，可以通过 `sudo ip link set dw0 mtu <value>` 修改。
*   **数据收发**: 通过标准的 socket API 进行数据收发。

### 9.2. 字符设备接口 (Character Device Interface)

*   **设备节点**: 通常为 `/dev/dw1000_sensorX` (例如 `/dev/dw1000_sensor0`)。
*   **用途**:
    *   **配置**: 通过 `ioctl` 命令进行设备参数的详细配置。
    *   **传感数据**: 通过 `read` (如果驱动实现) 或 `mmap` 方式读取原始的传感数据或非网络协议帧。
    *   **原始帧发送**: 通过 `write` 发送自定义格式的帧。
*   **IOCTL 命令**: 定义在 `dw1000_generic.h` 中，主要包括：
    *   `DW1000_IOC_SET_CONFIG`: 设置设备配置参数 (如信道、速率、PAN ID 等)。参数为 `struct dw1000_hybrid_config`。
    *   `DW1000_IOC_GET_STATS`: 获取详细的设备统计信息。参数为 `struct dw1000_hybrid_stats`。
    *   `DW1000_IOC_RESET`: 执行硬件复位。
    *   `DW1000_IOC_SET_DEST_ADDR`: 设置字符设备写操作的默认目标地址。
    *   `DW1000_IOC_ACCEPT_BAD_FRAMES`: 设置是否接受CRC校验错误的帧。
*   **`mmap` 支持**:
    *   字符设备支持 `mmap` 操作，允许用户空间直接映射内核的传感数据接收环形缓冲区 (`sensing_buffer`)，实现零拷贝数据读取。
    *   环形缓冲区大小由 `RING_BUFFER_SIZE` (在 `dw1000_generic.h` 中定义，例如 64KB) 决定。
    *   数据通过 `kfifo` 进行管理。
*   **`write` 操作**:
    *   用户可以通过 `write` 系统调用向字符设备写入数据。
    *   写入的数据会被封装成自定义的 `DW1000_HYBRID_HDR_LEN` 字节头部和用户负载，然后通过 DW1000 发送。
    *   支持一个可选的单字节前缀来指定帧类型 (`DW1000_FRAME_TYPE_SENSING`, `DW1000_FRAME_TYPE_NETWORK` 等)。
*   **`poll` / `select` 支持**:
    *   支持 `poll` 和 `select` 等多路复用I/O操作，当有新的传感数据到达 `kfifo` 时，会唤醒等待的进程。

## 10. 配置参数 (Configuration)

驱动的配置主要通过以下方式进行：

1.  **设备树**: 初始化时的默认参数。
2.  **IOCTL (`DW1000_IOC_SET_CONFIG`)**: 运行时动态修改。`struct dw1000_hybrid_config` (定义于 `dw1000_generic.h`) 包含了可配置的参数，例如：
    *   `channel`: UWB 信道号 (1-8)。
    *   `spi_speed_hz`: SPI 时钟速度。
    *   `prf`: 脉冲重复频率 (0 for 16MHz, 1 for 64MHz)。
    *   `preamble_length`: 前导码长度。
    *   `preamble_code`: 前导码序列索引。
    *   `data_rate`: 数据传输速率 (0 for 110kbps, 1 for 850kbps, 2 for 6.8Mbps)。
    *   `smart_power`: 是否启用智能 TX 功率控制。
    *   `tx_power`: TX 功率寄存器值。
    *   `pan_id`: 设备的 PAN ID。
    *   `accept_bad_frames`: 是否接收 FCS 错误的帧。
    *   `sfd_timeout`: SFD 超时设置。

## 11. 统计信息 (Statistics)

*   **网络接口**: 标准的接口统计信息可以通过 `ip -s link show dw0` 或其他网络工具获取。这些数据由 `dw1000_hybrid_get_stats64` 函数提供。
*   **驱动/硬件**: 更详细的统计信息，包括特定的 RX/TX 错误、DMA 错误、传感数据包计数等，可以通过字符设备的 `DW1000_IOC_GET_STATS` IOCTL 命令获取。统计结构为 `struct dw1000_hybrid_stats`。

## 12. 许可 (License)
本驱动采用 GPL 许可证。

## 13. 作者 (Author)
Jinting Liu