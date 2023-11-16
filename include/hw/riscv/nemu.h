#ifndef _HW_RISCV_NEMU_H_
#define _HW_RISCV_NEMU_H_

#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define NEMU_CPUS_MAX 1
#define NEMU_SOCKETS_MAX 1

#define TYPE_RISCV_NEMU_MACHINE MACHINE_TYPE_NAME("nemu")
typedef struct RISCVVirtState RISCVVirtState;
DECLARE_INSTANCE_CHECKER(RISCVVirtState, RISCV_VIRT_MACHINE,
                         TYPE_RISCV_NEMU_MACHINE)

struct RISCVVirtState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    Notifier machine_done;
    RISCVHartArrayState soc[NEMU_SOCKETS_MAX];
    DeviceState *irqchip[NEMU_CPUS_MAX];
};

enum {
    NEMU_MROM,
    NEMU_CLINT,
    NEMU_PLIC,
    NEMU_UART0,
    NEMU_DRAM,
};

enum {
    UART0_IRQ = 10,
};

#define NEMU_PLIC_HART_CONFIG   "MS"
#define NEMU_PLIC_NUM_SOURCES   127
#define NEMU_PLIC_NUM_PRIORITIES    7
#define NEMU_PLIC_PRIORITY_BASE 0x04
#define NEMU_PLIC_PENDING_BASE  0x1000
#define NEMU_PLIC_ENABLE_BASE   0x2000
#define NEMU_PLIC_ENABLE_STRIDE 0x80
#define NEMU_PLIC_CONTEXT_BASE  0x200000
#define NEMU_PLIC_CONTEXT_STRIDE 0x1000
#define NEMU_PLIC_SIZE(__num_context) \
    (NEMU_PLIC_CONTEXT_BASE + (__num_context) * NEMU_PLIC_CONTEXT_STRIDE)

#endif