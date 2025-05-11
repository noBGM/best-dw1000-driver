# Makefile for DW1000 Generic Hybrid Driver

# 模块名称
MODULE_NAME := dw1000_generic_hybrid

# 编译目标
obj-$(CONFIG_DW1000_GENERIC_HYBRID) += $(MODULE_NAME).o

# 源文件列表
$(MODULE_NAME)-objs := \
    dw1000_core.o \
    dw1000_hw.o \
    dw1000_dma.o \
    dw1000_interrupt.o \
    dw1000_frame_processing.o

# 编译选项
ccflags-y += -I$(src)
ccflags-y += -Wall -Wextra -Werror
ccflags-y += -DDEBUG

# 条件编译选项
ifeq ($(CONFIG_DW1000_GENERIC_HYBRID_VERBOSE_DEBUG),y)
    ccflags-y += -DVERBOSE_DEBUG
endif

# 依赖关系
$(MODULE_NAME)-objs: dw1000_generic.h

# 清理规则
clean-files := *.o *.ko *.mod.c .*.cmd Module.symvers modules.order

# 帮助目标
help:
	@echo "可用的编译目标:"
	@echo "  all     - 编译模块"
	@echo "  clean   - 清理编译生成的文件"
	@echo "  help    - 显示此帮助信息"

.PHONY: help clean 