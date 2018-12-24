
#ifndef __SPIDEV_STREAM_H__
#define __SPIDEV_STREAM_H__
#include <linux/spi/spidev.h>
#include <linux/types.h>

#define VIDEO_TRANSFER_SIZE         512
#define AUDIO_TRANSFER_SIZE         128
#define BULK_TRANSFER_SIZE          128
#define BLE_TRANSFER_SIZE           128
#define TALKBACK_TRANSFER_SIZE      256

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *  is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK       (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
        | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
        | SPI_NO_CS | SPI_READY)

#define SOCSPI_VIDEO_TRANSFER       _IOW(SPI_IOC_MAGIC, 5, __u8)
#define SOCSPI_AUDIO_TRANSFER       _IOW(SPI_IOC_MAGIC, 6, __u8)
#define SOCSPI_COMMAND_TRANSFER     _IOW(SPI_IOC_MAGIC, 7, __u8)
#define SOCSPI_STATUS_TRANSFER      _IOW(SPI_IOC_MAGIC, 8, __u8)
#define SOCSPI_J_TRANSFER           _IOW(SPI_IOC_MAGIC, 9, __u8)
#define SOCSPI_FLUSH_DATA           _IOW(SPI_IOC_MAGIC, 10, __u8)
#define SOCSPI_SET_DEBUG            _IOW(SPI_IOC_MAGIC, 11, __u8)
#define SOCSPI_U_READ               _IOR(SPI_IOC_MAGIC, 12, __u8)
#define SOCSPI_U_WRITE              _IOW(SPI_IOC_MAGIC, 13, __u8)
#define SOCSPI_DUMMY_ON             _IOW(SPI_IOC_MAGIC, 14, __u8)
#define SOCSPI_DUMMY_OFF            _IOW(SPI_IOC_MAGIC, 15, __u8)
#define SOCSPI_HARD_RESET           _IOW(SPI_IOC_MAGIC, 16, __u8)

// depend on which protocol we defined to communicate between two module via spi bus
#define SPI_CMD_AUDIO(l)            (('A' << 24) | (0x00 << 16) | (l & 0xFFFF))
#define SPI_CMD_VIDEO(l)            (('V' << 24) | (0x00 << 16) | (l & 0xFFFF))
#define SPI_CMD_J_WRITE(l)          (('J' << 24) | (0x00 << 16) | (l & 0xFFFF))
#define SPI_CMD_U_WRITE(l)          (('U' << 24) | (0x00 << 16) | (0x00 << 8) |  (l & 0xFF))
#define SPI_CMD_U_READ(l)           (('u' << 24) | (0x00 << 16) | (0x00 << 8) |  (l & 0xFF))
#define SPI_CMD_SEND_DUMMY          (('W' << 24) | (0x00 << 16) | (0xFF << 8) | (0xFF << 0))
#define SPI_CMD_BLE_MODE(m)         (('W' << 24) | (0xFF << 16) | ((m) << 8)  | (0x03 << 0))

typedef enum {
    VIDEO_FRAME,
    H264_FRAME_I,
    H264_FRAME_P,
    H264_FRAME_B
}H264_FRAME_TYPE;

struct socspi_transfer{
    u32 res_width;
    u32 res_height;
    u32 len;
    H264_FRAME_TYPE frame_type;
    void* data;
};

struct socspi_data {
    dev_t               devt;
    spinlock_t          spi_lock;
    struct spi_device   *spi;
    struct list_head    device_entry;

    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex        buf_lock;
    unsigned            users;
    u8                  *buffer;

    // For spi_soc
    struct kfifo        video_fifo;
    wait_queue_head_t   video_wq_list;
    struct mutex        video_lock;
    int                 video_blocked;
    int                 h264_skip_i;
    int                 h264_skip_p;
    char                video_buffer[4096];

    struct kfifo        audio_fifo;
    wait_queue_head_t   audio_wq_list;
    struct mutex        audio_lock;
    int                 audio_blocked;
    char                audio_buffer[AUDIO_TRANSFER_SIZE];

    struct kfifo        talkback_fifo;
    wait_queue_head_t   talkback_wq_list;
    struct mutex        talkback_lock;
    int                 talkback_blocked;
    char                talkback_buffer[256];

    struct kfifo        u_read_fifo;
    wait_queue_head_t   u_read_wq_list;
    struct mutex        u_read_lock;
    int                 u_read_priority;

    wait_queue_head_t   u_write_wq_list;
    unsigned char       u_write_length;
    char                u_write_buffer[128];

    wait_queue_head_t   jcmd_wq_list;
    unsigned int        jcmd_length;
    char                jcmd_buffer[4096];

    unsigned int        command;
    unsigned int        command_status;
    unsigned int        status;
    struct mutex        command_lock;
    wait_queue_head_t   command_wq_list;

    wait_queue_head_t   flush_wq_list;
    wait_queue_head_t   ble_wq_list;

    unsigned int        is_talkback;
    unsigned int        is_set_ble;
    unsigned int        is_flush;
};

#endif // __SPIDEV_STREAM_H__
