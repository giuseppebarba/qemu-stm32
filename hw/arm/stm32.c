/*
 * Define STM32 machine configuration
 */

#include "hw/sysbus.h"
#include "hw/arm-misc.h"
#include "hw/boards.h"
#include "hw/devices.h"
#include "exec/address-spaces.h"

#define STM_FLASH_BASE       0x08000000
#define STM_SRAM_BASE        0x20000000
#define STM_FLASH_SIZE_KB    0x0080
#define STM_FLASH_SIZE       STM_FLASH_SIZE_KB * 1024
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

static void armv7m_bitband_init(void)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "ARM,bitband-memory");
    qdev_prop_set_uint32(dev, "base", 0x20000000);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x22000000);

    dev = qdev_create(NULL, "ARM,bitband-memory");
    qdev_prop_set_uint32(dev, "base", 0x40000000);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x42000000);
}

static void armv7m_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

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

/* Init CPU and memory for a v7-M based board.
   flash_size and sram_size are in kb.
   Returns the NVIC array.  */

qemu_irq *stm32_core_init(MemoryRegion *address_space_mem,
                      int flash_size, int sram_size,
                      const char *kernel_filename, const char *cpu_model)
{
    ARMCPU *cpu;
    CPUARMState *env;
    DeviceState *nvic;
    /* FIXME: make this local state.  */
    static qemu_irq pic[64];
    qemu_irq *cpu_pic;
    int i;

    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *hack = g_new(MemoryRegion, 1);

    /* All memory size are expressed in bytes unit */
    flash_size *= 1024;
    sram_size *= 1024;

    if (cpu_model == NULL) {
	cpu_model = "cortex-m3";
    }
    cpu = cpu_arm_init(cpu_model);
    if (cpu == NULL) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    env = &cpu->env;

    /* Flash programming.  */

    memory_region_init_ram(flash, "stm32.flash", flash_size);
    vmstate_register_ram_global(flash);
    memory_region_set_readonly(flash, true);
    memory_region_add_subregion(address_space_mem, STM_FLASH_BASE, flash);

    memory_region_init_ram(sram, "stm32.sram", sram_size);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(address_space_mem, STM_SRAM_BASE, sram);

    /* Bit banding for SRAM and DEVICE spaces */
    armv7m_bitband_init();

    /* Nested vector interrupt controller (NVIC) */
    nvic = qdev_create(NULL, "armv7m_nvic");
    env->nvic = nvic;
    qdev_init_nofail(nvic);
    cpu_pic = arm_pic_init_cpu(cpu);
    sysbus_connect_irq(SYS_BUS_DEVICE(nvic), 0, cpu_pic[ARM_PIC_CPU_IRQ]);
    for (i = 0; i < 64; i++) {
        pic[i] = qdev_get_gpio_in(nvic, i);
    }

    /* Hack to map an additional page of ram at the top of the address
       space.  This stops qemu complaining about executing code outside RAM
       when returning from an exception.  */
    memory_region_init_ram(hack, "armv7m.hack", 0x1000);
    vmstate_register_ram_global(hack);
    memory_region_add_subregion(address_space_mem, 0xfffff000, hack);

    qemu_register_reset(armv7m_reset, cpu);
    return pic;
}


static int stm32l_sys_init (uint32_t base, qemu_irq irq,
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

static void stm32_init(QEMUMachineInitArgs *args)
{
    MemoryRegion *address_space_mem = get_system_memory();
    qemu_irq *pic;
    DriveInfo *dinfo;
    int fl_idx;
    uint16_t flash_size = 0x0080;   /* 128KBits */
    uint16_t sram_size  = 0x0010;   /* 16 KBits */

    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;

    /* Initialize processor and memory */
    pic = stm32_core_init(address_space_mem, STM_FLASH_SIZE_KB, sram_size,
                      kernel_filename, cpu_model);

    if ((dinfo = drive_get(IF_PFLASH, 0, 0)) != NULL) {
        if (!pflash_cfi01_register(0x0, NULL,
                                   "stm32.rom", STM_FLASH_SIZE,
                                   dinfo->bdrv, STM_SECTOR_SIZE,
                                   STM_FLASH_SIZE / STM_SECTOR_SIZE,
                                   4, 0, 0, 0, 0, 0)) {
            fprintf(stderr, "qemu: Error registering flash memory %d.\n",
                           fl_idx);
        }
        fl_idx++;
    }

    stm32l_sys_init(0x1FF00000, pic[28], &stm32_board);
}

static QEMUMachine stm32_machine = {
    .name = "stm32",
    .desc = "STM32 Discovery",
    .init = stm32_init,
    DEFAULT_MACHINE_OPTIONS,
};

static void stm32_machine_init(void)
{
    qemu_register_machine(&stm32_machine);
}

machine_init(stm32_machine_init);

