APP  := hello_freertos

SRCS := hello_freertos.c

KERNEL_SRC_PATH := ../../..

# freertos kernel source
SRCS += $(KERNEL_SRC_PATH)/croutine.c
SRCS += $(KERNEL_SRC_PATH)/event_groups.c
SRCS += $(KERNEL_SRC_PATH)/list.c
SRCS += $(KERNEL_SRC_PATH)/queue.c
SRCS += $(KERNEL_SRC_PATH)/stream_buffer.c
SRCS += $(KERNEL_SRC_PATH)/tasks.c
SRCS += $(KERNEL_SRC_PATH)/timers.c
SRCS += $(KERNEL_SRC_PATH)/portable/MemMang/heap_4.c
SRCS += $(KERNEL_SRC_PATH)/portable/ThirdParty/GCC/Hazard3/port.c
SRCS += $(KERNEL_SRC_PATH)/portable/ThirdParty/GCC/Hazard3/portASM.S

INCDIR = .
INCDIR += $(KERNEL_SRC_PATH)/include
INCDIR += $(KERNEL_SRC_PATH)/portable/ThirdParty/GCC/Hazard3/include

CCFLAGS = -march=rv32imac_zicsr_zifencei -g

CCFLAGS += -DconfigNUMBER_OF_CORES=1

SDK_PATH := ../../../..

include $(SDK_PATH)/common/common.mk
