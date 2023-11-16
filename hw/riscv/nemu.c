#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/char/serial.h"
#include "target/riscv/cpu.h"
#include "hw/core/sysbus-fdt.h"
#include "target/riscv/pmu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/nemu.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/numa.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/riscv_aplic.h"
#include "hw/intc/riscv_imsic.h"
#include "hw/intc/sifive_plic.h"
#include "hw/misc/sifive_test.h"
#include "hw/platform-bus.h"
#include "chardev/char.h"
#include "sysemu/device_tree.h"
#include "sysemu/sysemu.h"
#include "sysemu/tcg.h"
#include "sysemu/kvm.h"
#include "sysemu/tpm.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/gpex.h"
#include "hw/display/ramfb.h"
#include "hw/acpi/aml-build.h"
#include "qapi/qapi-visit-common.h"
#include "hw/char/xilinx_uartlite.h"

static const MemMapEntry nemu_memmap[] = {
    [NEMU_MROM]  = {      0x1000,        0x100 },
    [NEMU_PLIC]  = {   0xc000000,       NEMU_PLIC_SIZE(NEMU_CPUS_MAX * 2) },
    [NEMU_CLINT] = {  0x38000000,       0x10000 },
    [NEMU_UART0] = {  0x40600000,         0x100 },
    [NEMU_DRAM]  = {  0x80000000,           0x0 },
};

static DeviceState *nemu_create_plic(const MemMapEntry *memmap, int socket,
                                     int base_hartid, int hart_count)
{
    DeviceState *ret;
    char *plic_hart_config;

    plic_hart_config = riscv_plic_hart_config_string(hart_count);

    ret = sifive_plic_create(
        memmap[NEMU_PLIC].base + socket * memmap[NEMU_PLIC].size,
        plic_hart_config, hart_count, base_hartid,
        NEMU_PLIC_NUM_SOURCES,
        NEMU_PLIC_NUM_PRIORITIES,
        NEMU_PLIC_PRIORITY_BASE,
        NEMU_PLIC_PENDING_BASE,
        NEMU_PLIC_ENABLE_BASE,
        NEMU_PLIC_ENABLE_STRIDE,
        NEMU_PLIC_CONTEXT_BASE,
        NEMU_PLIC_CONTEXT_STRIDE,
        memmap[NEMU_PLIC].size);
    g_free(plic_hart_config);

    return ret;
}

static void nemu_machine_done(Notifier *notifier, void *data)
{
    RISCVVirtState *s = container_of(notifier, RISCVVirtState,
                                     machine_done);
    const MemMapEntry *memmap = nemu_memmap;
    MachineState *machine = MACHINE(s);
    target_ulong start_addr = memmap[NEMU_DRAM].base + 0x200000;
    target_ulong firmware_end_addr, kernel_start_addr;
    const char *firmware_name = riscv_default_firmware_name(&s->soc[0]);
    uint64_t kernel_entry = memmap[NEMU_DRAM].base + 0x200000;

    firmware_end_addr = riscv_find_and_load_firmware(machine, firmware_name,
                                                     start_addr, NULL);
    
    if(machine->kernel_filename && !kernel_entry){
        kernel_start_addr = riscv_calc_kernel_start_addr(&s->soc[0],
                                                         firmware_end_addr);

        kernel_entry = riscv_load_kernel(machine, &s->soc[0],
                                         kernel_start_addr, true, NULL);
    }
    riscv_setup_rom_reset_vec(machine, &s->soc[0], start_addr,
                              nemu_memmap[NEMU_MROM].base,
                              nemu_memmap[NEMU_MROM].size, kernel_entry,
                              kernel_entry);
    
}

static void nemu_machine_init(MachineState *machine)
{
    const MemMapEntry *memmap = nemu_memmap;
    RISCVVirtState *s = RISCV_VIRT_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *mask_rom = g_new(MemoryRegion, 1);
    char *soc_name;
    DeviceState *mmio_irqchip, *dev;
    int i, base_hartid, hart_count;
    mmio_irqchip = NULL;
    for (i = 0; i < riscv_socket_count(machine); i++) {
        if(!riscv_socket_check_hartids(machine, i)) {
            error_report("discontinuous harids in socket%d", i);
            exit(1);
        }
        base_hartid = riscv_socket_first_hartid(machine, i);
        if(base_hartid < 0){
            error_report("can't find hartid base for socket%d", i);
            exit(1);
        }
        hart_count = riscv_socket_hart_count(machine, i);
        if (hart_count < 0) {
            error_report("can't find hart count for socket%d", i);
            exit(1);
        }
        soc_name = g_strdup_printf("soc%d", i);
        object_initialize_child(OBJECT(machine), soc_name, &s->soc[i], 
                                TYPE_RISCV_HART_ARRAY);
        g_free(soc_name);
        object_property_set_str(OBJECT(&s->soc[i]), "cpu-type",
                                machine->cpu_type, &error_abort);
        object_property_set_int(OBJECT(&s->soc[i]), "hartid-base",
                                base_hartid, &error_abort);
        object_property_set_int(OBJECT(&s->soc[i]), "num-harts",
                                hart_count, &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->soc[i]), &error_fatal);
        riscv_aclint_swi_create(
            memmap[NEMU_CLINT].base + i * memmap[NEMU_CLINT].size,
            base_hartid, hart_count, false);
            riscv_aclint_mtimer_create(memmap[NEMU_CLINT].base +
                i * memmap[NEMU_CLINT].size + RISCV_ACLINT_SWI_SIZE,
            RISCV_ACLINT_DEFAULT_MTIMER_SIZE, base_hartid, hart_count,
            RISCV_ACLINT_DEFAULT_MTIMECMP, RISCV_ACLINT_DEFAULT_MTIME,
            RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, true);
        s->irqchip[i] = nemu_create_plic(memmap, i, base_hartid, hart_count);
        
        if(i == 0){
            mmio_irqchip = s->irqchip[i];
        }
    }
    /* register system main memory (actual RAM) */
    memory_region_add_subregion(system_memory, memmap[NEMU_DRAM].base,
                                machine->ram);
    
    /* boot rom */
    memory_region_init_rom(mask_rom, NULL, "riscv_virt_board.mrom",
                           memmap[NEMU_MROM].size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[NEMU_MROM].base,
                                mask_rom);

    dev = qdev_new(TYPE_XILINX_UARTLITE);
    qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, memmap[NEMU_UART0].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(mmio_irqchip, UART0_IRQ));

    s->machine_done.notify = nemu_machine_done;
    qemu_add_machine_init_done_notifier(&s->machine_done);
}

static void nemu_machine_class_init(ObjectClass *oc, void *data){
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V Nemu board";
    mc->init = nemu_machine_init;
    mc->max_cpus = NEMU_CPUS_MAX;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->pci_allow_0_address = true;
    mc->possible_cpu_arch_ids = riscv_numa_possible_cpu_arch_ids;
    mc->cpu_index_to_instance_props = riscv_numa_cpu_index_to_props;
    mc->get_default_cpu_node_id = riscv_numa_get_default_cpu_node_id;
    mc->numa_mem_supported = true;
    mc->default_ram_id = "riscv_nemu_board.ram";
}
static void nemu_machine_instance_init(Object *obj)
{
}


static const TypeInfo nemu_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("nemu"),
    .parent     = TYPE_MACHINE,
    .class_init = nemu_machine_class_init,
    .instance_init = nemu_machine_instance_init,
    .interfaces = (InterfaceInfo[]){
        { TYPE_HOTPLUG_HANDLER },
        {}
    },
};

static void nemu_machine_init_register_types(void){
    type_register_static(&nemu_machine_typeinfo);
}


type_init(nemu_machine_init_register_types)