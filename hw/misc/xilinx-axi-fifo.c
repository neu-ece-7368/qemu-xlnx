/*
 * QEMU model of the Xilinx AXI4-Stream FIFO Registers
 *
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "qemu/timer.h"

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

// Sampling frequency is 11025 Hz
#define TIME_PER_SAMPLE_NS 90703

#define FIFO_DEPTH 4096

typedef struct XlnxAXIFIFO {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[R_MAX];
    RegisterInfo regs_info[R_MAX];

    unsigned int samples_written;
    unsigned int samples_read;
    FILE* wavefile;
    int64_t time_check;
} XlnxAXIFIFO;

static int wav_init(XlnxAXIFIFO* fifo)
{
    uint8_t hdr[] = {
        0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x57, 0x41, 0x56,
        0x45, 0x66, 0x6d, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x02, 0x00, 0x11, 0x2B, 0x00, 0x00, 0x10, 0xb1, 0x02, 0x00, 0x04,
        0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00
    };

    fifo->wavefile = fopen("qemu.wav", "wb");
    if (!fifo->wavefile) {
        printf("Failed to open wave file: %s\n", strerror(errno));
        return -1;
    }

    fwrite(hdr, sizeof (hdr), 1, fifo->wavefile);
    
    fifo->time_check = qemu_clock_get_ns(QEMU_CLOCK_HOST);

    return 0;
}

static void write_sample_to_wav(RegisterInfo *reg, uint64_t val)
{
    XlnxAXIFIFO *s = XLNX_AXI_FIFO(reg->opaque);

    // Create wavefile if not initialized
    if (!s->wavefile) {
        if (wav_init(s) < 0) {
            return;
        }
    }
    
    // Write the 16-bit sample to the wavefile in little endian format
    uint8_t* bytes = (uint8_t*) &val;
    fwrite(bytes + 2, sizeof(uint8_t), 1, s->wavefile);
    fwrite(bytes + 3, sizeof(uint8_t), 1, s->wavefile);
    s->samples_written++;
}

static void close_wav_file(RegisterInfo *reg, uint64_t val)
{
    XlnxAXIFIFO *s = XLNX_AXI_FIFO(reg->opaque);

    // If the wavefile was not created yet, don't do anything
    if (!s->wavefile) {
        return;
    }

    // Write the file and data length to the header
    uint32_t data_len = s->samples_written * 2;
    uint32_t total_len = data_len + 36;

    fseek(s->wavefile, 4, SEEK_SET);
    fwrite(&total_len, sizeof(uint32_t), 1, s->wavefile);
    fseek(s->wavefile, 32, SEEK_CUR);
    fwrite(&data_len, sizeof(uint32_t), 1, s->wavefile);

    // Close the file
    fclose(s->wavefile);
    s->wavefile = NULL;
    s->samples_written = 0;
}

static uint64_t get_num_samples_in_fifo(XlnxAXIFIFO *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_HOST);
    unsigned int read = (now - s->time_check) / TIME_PER_SAMPLE_NS;

    // Can't read more samples than written
    if (s->samples_read + read >= s->samples_written) {
        s->samples_read = s->samples_written;
        // Reset the time when the fifo is empty
        s->time_check = now;
        return 0;
    }

    return s->samples_written - (s->samples_read + read);
}

static uint64_t get_fifo_vacancy(RegisterInfo *reg, uint64_t val)
{
    XlnxAXIFIFO *s = XLNX_AXI_FIFO(reg->opaque);

    uint64_t samples_in_fifo = get_num_samples_in_fifo(s);

    if (samples_in_fifo < FIFO_DEPTH) {
        return FIFO_DEPTH - samples_in_fifo;
    } else {
        return 0;
    }
}

static RegisterAccessInfo  xlnx_axi_fifo_regs_info[] = {
    {   .name = "ISR",  .addr = A_ISR,
    },{ .name = "IER",  .addr = A_IER,
    },{ .name = "TDFR",  .addr = A_TDFR,
    },{ .name = "TDFV",  .addr = A_TDFV,
        .post_read = get_fifo_vacancy,
    },{ .name = "TDFD",  .addr = A_TDFD,
        .post_write = write_sample_to_wav,
    },{ .name = "TLR",  .addr = A_TLR,
        .post_write = close_wav_file,
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
}

static void xlnx_axi_fifo_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = xlnx_axi_fifo_reset;
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
