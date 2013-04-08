/*
 * Define STM32 machine configuration
 */

#include "hw/sysbus.h"
#include "hw/arm-misc.h"
#include "hw/devices.h"
#include "qemu/timer.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"

#define STM_FLASH_BASE       0x08000000
#define STM_SRAM_BASE        0x20000000
#define STM_FLASH_SIZE_KB    0x0080
#define STM_FLASH_SIZE       STM_FLASH_SIZE_KB * 1024
#define STM_SRAM_SIZE_KB     0x0008
#define STM_SRAM_SIZE        STM_SRAM_SIZE_KB * 1024
#define STM_SECTOR_SIZE      (128 * 1024)

enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_COUNT
};

typedef const struct { 
    const char *name;
} stm32_board_info;

static stm32_board_info stm32_board = {
    "stm32l152rbt6",
};

typedef struct {
    uint32_t     int_status;
    uint32_t     int_mask;
    MemoryRegion iomem;
    qemu_irq     irq;
    stm32_board_info *board;
} ssys_state;

static void ssys_reset (void *opaque)
{
}

static uint64_t ssys_read (void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static void ssys_write (void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
}

static const MemoryRegionOps ssys_ops = {
    .read       = ssys_read,
    .write      = ssys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int stm32_sys_post_load (void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_stm32_sys = {
    .name                   = "stm32_sys",
    .version_id             = 2,
    .minimum_version_id     = 1,
    .minimum_version_id_old = 1,
    .post_load          = stm32_sys_post_load,
    .fields             = (VMStateField[]) {
        VMSTATE_UINT32(int_mask, ssys_state),
        VMSTATE_UINT32(int_status, ssys_state),
        VMSTATE_END_OF_LIST()
    }
};

static int stm32_sys_init (uint32_t base, qemu_irq irq,
           stm32_board_info *board)
{
    ssys_state *s;

    s = (ssys_state *)g_malloc0(sizeof(ssys_state));
    s->irq   = irq;
    s->board = board;

    memory_region_init_io(&s->iomem, &ssys_ops, s, "ssys", 0x00010000);
    memory_region_add_subregion(get_system_memory(), base, &s->iomem);

    ssys_reset(s);
    vmstate_register(NULL, -1, &vmstate_stm32_sys, s);

    return 0;
}

/* Init CPU and memory for a v7-M based board.
   flash_size and sram_size are in kb.
   Returns the NVIC array.  */

static void stm32_init(const char *kernel_filename, const char *cpu_model)
{
    MemoryRegion *address_space_mem = get_system_memory();
    
    int flash_size = STM_FLASH_SIZE;
    int sram_size = STM_SRAM_SIZE;
    qemu_irq *irq;

    irq = armv7m_init(address_space_mem, flash_size, sram_size, kernel_filename, "cortex-m3");
    stm32_sys_init(0x400fe000, irq[28], &stm32_board);
}

static void stm32evb_init(QEMUMachineInitArgs *args)
{
    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;

    stm32_init(kernel_filename, cpu_model);
}

static QEMUMachine stm32evb_machine = {
    .name = "stm32evb",
    .desc = "STM32 Discovery",
    .init = stm32evb_init,
    DEFAULT_MACHINE_OPTIONS,
};

static void stm32evb_machine_init(void)
{
    qemu_register_machine(&stm32evb_machine);
}

machine_init(stm32evb_machine_init);

