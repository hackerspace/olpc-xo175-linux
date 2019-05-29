/*
 * Marvell MMP ZSP audio message.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MMP_ZSP_AUDIO_MSG_H
#define __MMP_ZSP_AUDIO_MSG_H

/* definition of ZSP command */
#define AUDIO_STREAM_CMDID_BASE			0x0000
#define AUDIO_STREAM_CMDID_ACQUIRE		0x0000
#define AUDIO_STREAM_CMDID_PREPARE		0x0001
#define AUDIO_STREAM_CMDID_TRIGGER		0x0002
#define AUDIO_STREAM_CMDID_POSITION		0x0003
#define AUDIO_STREAM_CMDID_DATARXTXREQ	0x0004
#define AUDIO_STREAM_CMDID_DATARXTX		0x0005
#define AUDIO_STREAM_CMDID_RELEASE		0x0006
#define AUDIO_STREAM_CMDID_LIMITER		0x000F

/* definition of stream id */
#define AUDIO_STREAM_ID_SSPA1_PLAYBACK	0x0000
#define AUDIO_STREAM_ID_SSPA1_CAPTURE	0x0001
#define AUDIO_STREAM_ID_SSPA2_PLAYBACK	0x0002
#define AUDIO_STREAM_ID_SSPA2_CAPTURE	0x0003
#define AUDIO_STREAM_MAX_NUM			0x0004

/* sspa configuration parameter: audio word length */
#define SSPA_CONFIG_WORD_LENGTH_8_BITS  0x0000
#define SSPA_CONFIG_WORD_LENGTH_12_BITS 0x0001
#define SSPA_CONFIG_WORD_LENGTH_16_BITS 0x0002
#define SSPA_CONFIG_WORD_LENGTH_20_BITS 0x0003
#define SSPA_CONFIG_WORD_LENGTH_24_BITS 0x0004
#define SSPA_CONFIG_WORD_LENGTH_32_BITS 0x0005

/* sspa configuration parameter: data delay bit */
#define SSPA_CONFIG_DATA_DELAY_0_BIT    0x0000
#define SSPA_CONFIG_DATA_DELAY_1_BIT    0x0001

/* sspa configuration parameter: audio sample size */
#define SSPA_CONFIG_SAMPLE_SIZE_8_BITS  0x0000
#define SSPA_CONFIG_SAMPLE_SIZE_12_BITS 0x0001
#define SSPA_CONFIG_SAMPLE_SIZE_16_BITS 0x0002
#define SSPA_CONFIG_SAMPLE_SIZE_20_BITS 0x0003
#define SSPA_CONFIG_SAMPLE_SIZE_24_BITS 0x0004
#define SSPA_CONFIG_SAMPLE_SIZE_32_BITS 0x0005

/* adma configuration parameter: audio sample size */
#define ADMA_CONFIG_SAMPLE_SIZE_8_BITS  0x0000
#define ADMA_CONFIG_SAMPLE_SIZE_12_BITS 0x0001
#define ADMA_CONFIG_SAMPLE_SIZE_16_BITS 0x0002
#define ADMA_CONFIG_SAMPLE_SIZE_20_BITS 0x0003
#define ADMA_CONFIG_SAMPLE_SIZE_24_BITS 0x0004
#define ADMA_CONFIG_SAMPLE_SIZE_32_BITS 0x0005

/* adma configuration parameter: pack or unpack selection */
#define ADMA_CONFIG_PACKMOD_UNPACK      0x0000
#define ADMA_CONFIG_PACKMOD_PACK        0x0001

/* sspa configuration parameter: master or slave selection */
#define SSPA_CONFIG_MS_SEL_SLAVE        0x0000
#define SSPA_CONFIG_MS_SEL_MASTER       0x0001

/* sspa configuration parameter: frame sync active polarity */
#define SSPA_CONFIG_FRAME_SYNC_POLA_HIGH    0x0000
#define SSPA_CONFIG_FRAME_SYNC_POLA_LOW     0x0001

/* sspa configuration parameter: audio sample justification */
#define SSPA_CONFIG_SAMPLE_JST_LEFT     0x0000
#define SSPA_CONFIG_SAMPLE_JST_RIGHT    0x0001

/* trigger type */
#define AUDIO_STREAM_TRIGGER_START		0x0000
#define AUDIO_STREAM_TRIGGER_STOP		0x0001
#define AUDIO_STREAM_TRIGGER_SUSPEND	0x0002
#define AUDIO_STREAM_TRIGGER_RESUME		0x0003

/*
 * interrupt notificaton type to host
 * AUDIO_STREAM_INT_TYPE_NONE	: no interrupt to host
 * AUDIO_STREAM_INT_TYPE_FRAGMENT	: notify host once a frament done
 * AUDIO_STREAM_INT_TYPE_THRESHOLD	: notify host once avaiable room
 * less than specified threshold
 */
#define AUDIO_STREAM_INT_TYPE_NONE		0x0000
#define AUDIO_STREAM_INT_TYPE_FRAGMENT	0x0001
#define AUDIO_STREAM_INT_TYPE_THRESHOLD	0x0002

#define AUDIO_STREAM_STOP_IMMEDIATELY	0x0000
#define AUDIO_STREAM_STOP_AFTER_FLUSHED	0x0001

/* cmd and ack para for audio_stream_prepare */
typedef struct {
	u16 fwid;			/* fsync width */
	u16 fsync_active;
	u16 msl_select;		/* master/slave mode */
	u16 fsp;			/* fsync polarity */
	u16 ch_num;			/* channel number */
	u16 word_length;
	u16 sample_size;
	u16 jst;			/* audio sample justification */
	u16 data_delay;
	u16 resv;			/* for 32 bit align */
	u32 sample_rate;
	u32 mclk;
} sspa_config_t;

typedef struct {
	u16 sample_size;
	u16 pack_mode;
	u32 fragment_size;
} adma_config_t;

typedef struct {
	u16 command_id;
	u16 audio_stream_id;
	u16 parameters[28];
} cmd_audio_stream_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 return_code;
	u16 parameters[27];
} ack_audio_stream_t;

/* cmd and ack para for: audio_stream_acquire */
typedef struct {
	u16 command_id;
	u16 stream_id;
} cmd_audio_stream_acquire_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 return_code;
	u16 resv;
/***********************************************************************
 * maximum size of zsp audio buffer avaible for this acquired stream.
 * user can use this on demand. one usage example, user must constraint
 * the fragment size smaller than half of this size
 ***********************************************************************/
	u32 max_buffer_size;
} ack_audio_stream_acquire_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	sspa_config_t sspa_conf;
	adma_config_t adma_conf;
} cmd_audio_stream_prepare_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 return_code;
	u16 reservers;
	u32 sample_rate;
	u32 mclk;
} ack_audio_stream_prepare_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 trigger_type;
/* host notification type: no, fragment done or less than threshold */
	u16 int_type;
/* the threshold specified use to trigger the notification to host */
	u32 int_threshold;
/*
 * notes: the reason that put these two parameters in trigger api
 * but not prepare api is: provide user flexiblity to change the
 * notification type dynamically without re-config the sspa and adma
 */
} cmd_audio_stream_trigger_t;

/* cmd and ack para for: audio_stream_position */
typedef struct {
	u16 command_id;
	u16 stream_id;
} cmd_audio_stream_position_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 return_code;
	u16 reservers; /* for 32 bit align */
	u32 read_index;
	u32 write_index;
	u32 buffer_length;
} ack_audio_stream_position_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 trigger_type;
	u16 return_code;
} ack_audio_stream_trigger_t;

/* cmd para for: audio_stream_datarxtxreq */
typedef struct {
	u16 command_id;
	u16 stream_id;
	u32 read_index;
	u32 write_index;
	u32 buffer_length;
	u16 buf_full_flag;
} cmd_audio_stream_datarxtxreq_t;

/* cmd and ack para for: audio_stream_datarxtx */
typedef struct {
	u16 command_id;
	u16 stream_id;
	u32 addr;
	u32 size;
} cmd_audio_stream_datarxtx_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 return_code;
	u16 reservers; /* for 32 bit align */
	u32 size;
} ack_audio_stream_datarxtx_t;

/* cmd and ack para for: audio_stream_relese */
typedef struct {
	u16 command_id;
	u16 stream_id;
} cmd_audio_stream_release_t;

typedef struct {
	u16 command_id;
	u16 stream_id;
	u16 return_code;
} ack_audio_stream_release_t;

#ifdef CONFIG_MMP_ZSP
extern void *kzmq_open(int type_id);
extern void kzmq_close(void *hkzmq);
extern int kzmq_read(void *hkzmq, void *pmsg, int len);
extern int kzmq_write(void *hkzmq, void *pmsg, int len);
#else
static inline void *kzmq_open(int type_id)
{
	return NULL;
}

static inline void kzmq_close(void *hkzmq)
{
	return;
}

static inline int kzmq_read(void *hkzmq, void *pmsg, int len)
{
	return 0;
}

static inline int kzmq_write(void *hkzmq, void *pmsg, int len)
{
	return 0;
}
#endif

#endif
