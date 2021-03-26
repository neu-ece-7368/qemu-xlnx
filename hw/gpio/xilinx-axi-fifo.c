/*
 * QEMU model of the Xilinx AXI4-Stream FIFO Registers
 *
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/irq.h"

#ifndef XLNX_AXI_FIFO_ERR_DEBUG
#define XLNX_AXI_FIFO_ERR_DEBUG 0
#endif

#define TYPE_XLNX_AXI_FIFO "xlnx.axi-fifo"

#define XLNX_AXI_FIFO(obj) \
     OBJECT_CHECK(XlnxAXIFIFO, (obj), TYPE_XLNX_AXI_FIFO)

REG32(ISR, 0x00)
REG32(IER, 0x04)
REG32(TDFR, 0x08)
REG32(TDFV, 0x0C)
REG32(TDFD, 0x10)
REG32(TLR, 0x14)
REG32(RDFR, 0x18)
REG32(RDFO, 0x1C)
REG32(RDFD, 0x20)
REG32(RLR, 0x24)
REG32(SRR, 0x28)
REG32(TDR, 0x2C)
REG32(RDR, 0x30)

#define R_MAX (R_RDR + 1)

typedef struct XlnxAXIFIFO {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[R_MAX];
    RegisterInfo regs_info[R_MAX];
} XlnxAXIFIFO;

static void xlnx_axi_fifo_post_write(RegisterInfo  *reg, uint64_t val)
{
    printf("in post write\n");
}

static uint64_t xlnx_axi_fifo_post_read(RegisterInfo  *reg, uint64_t val)
{
    printf("in post read\n");
    return val;
}

static RegisterAccessInfo  xlnx_axi_fifo_regs_info[] = {
    {   .name = "ISR",  .addr = A_ISR,
        .post_read = xlnx_axi_fifo_post_read,
        .post_write = xlnx_axi_fifo_post_write,
    },{ .name = "IER",  .addr = A_IER,
        .post_read = xlnx_axi_fifo_post_read,
        .post_write = xlnx_axi_fifo_post_write,
    },{ .name = "TDFR",  .addr = A_TDFR,
        .post_write = xlnx_axi_fifo_post_write,
    },{ .name = "TDFV",  .addr = A_TDFV,
        .post_read = xlnx_axi_fifo_post_read,
    },{ .name = "TDFD",  .addr = A_TDFD,
        .post_write = xlnx_axi_fifo_post_write,
    },{ .name = "TLR",  .addr = A_TLR,
        .post_write = xlnx_axi_fifo_post_write,
    },{ .name = "RDFR",  .addr = A_RDFR,
    },{ .name = "RDFO",  .addr = A_RDFO,
    },{ .name = "RDFD",  .addr = A_RDFD,
    },{ .name = "RLR",  .addr = A_RLR,
    },{ .name = "SRR",  .addr = A_SRR,
    },{ .name = "TDR",  .addr = A_TDR,
    },{ .name = "RDR",  .addr = A_RDR,
    }
};


static void xlnx_axi_fifo_reset(DeviceState *dev)
{
    XlnxAXIFIFO *s = XLNX_AXI_FIFO(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }
}

static const MemoryRegionOps xlnx_axi_fifo_ops = {
    .read = register_read_memory,
    .write = register_write_memory,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void xlnx_axi_fifo_init(Object *obj)
{
    XlnxAXIFIFO *s = XLNX_AXI_FIFO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    RegisterInfoArray *reg_array;

    memory_region_init(&s->iomem, obj,
                          TYPE_XLNX_AXI_FIFO, R_MAX * 4);
    reg_array =
        register_init_block32(DEVICE(obj), xlnx_axi_fifo_regs_info,
                              ARRAY_SIZE(xlnx_axi_fifo_regs_info),
                              s->regs_info, s->regs,
                              &xlnx_axi_fifo_ops,
                              XLNX_AXI_FIFO_ERR_DEBUG,
                              R_MAX * 4);
    memory_region_add_subregion(&s->iomem,
                                0x0,
                                &reg_array->mem);

    sysbus_init_mmio(sbd, &s->iomem);

    printf("FIFO instance init\n");

}

static void xlnx_axi_fifo_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = xlnx_axi_fifo_reset;

    printf("FIFO class init\n");
}

static const TypeInfo xlnx_axi_fifo_info = {
    .name          = TYPE_XLNX_AXI_FIFO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XlnxAXIFIFO),
    .class_init    = xlnx_axi_fifo_class_init,
    .instance_init = xlnx_axi_fifo_init,
};

static void xlnx_axi_fifo_register_types(void)
{
    type_register_static(&xlnx_axi_fifo_info);
}

type_init(xlnx_axi_fifo_register_types)
