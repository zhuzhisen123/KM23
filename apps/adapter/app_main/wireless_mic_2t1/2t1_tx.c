#include "adapter_idev.h"
#include "adapter_odev.h"
#include "adapter_idev_bt.h"
#include "adapter_odev_dac.h"
#include "adapter_process.h"
#include "adapter_media.h"
#include "application/audio_dig_vol.h"
#include "le_client_demo.h"
#include "system/malloc.h"
#include "key_event_deal.h"
#include "asm/pwm_led.h"
#include "ui_manage.h"
#include "adapter_idev_usb.h"
#include "adapter_wireless_command.h"
#include "ble_user.h"
#include "adapter_recorder.h"
#if CONFIG_CPU_BR28
#include "audio_effect/audio_eff_default_parm.h"
#elif CONFIG_CPU_BR30
#include "app_online_cfg.h"
#endif
#include "app_main.h"
#include "device/key_driver.h"


#if (APP_MAIN ==  APP_WIRELESS_MIC_2T1 && WIRELESS_ROLE_SEL == APP_WIRELESS_SLAVE)

#define POWER_OFF_CNT       6
#define SLAVE_ENC_COMMAND_LEN		2

static struct adapter_media *wireless_mic_media = NULL;
static u8 command = WIRELESS_MIC_DENOISE_OFF;
static u8 echo_cmd = WIRELESS_MIC_ECHO_OFF;
u8 wireless_conn_status = 0;
struct mic_tx2rx_info{
    u8 channel_flag;
    u8 command;
};
//static u16 tx_cmd = 0;
static struct mic_tx2rx_info *tx_cmd = NULL;

u8 user_voice_changer_mode = 0;

extern void adapter_wireless_enc_command_send(u8 command);
extern u8 get_charge_online_flag(void);
extern u8 app_common_device_event_deal(struct sys_event *event);


#if (WIRELESS_DENOISE_ENABLE)
const struct __stream_HighSampleRateSingleMicSystem_parm denoise_parm_1 = {
    .AggressFactor = 1.0f,
    .minSuppress = 0.5f,//0.5f,/*!< 最小压制,建议范围[0.04 - 0.5], 越大降得越少 */
    .init_noise_lvl = 0.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};

const struct __stream_HighSampleRateSingleMicSystem_parm denoise_parm_2 = {
    .AggressFactor = 1.0f,
    .minSuppress = 0.25f,/*!< 最小压制,建议范围[0.04 - 0.5], 越大降得越少 */
    .init_noise_lvl = 0.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};

const struct __stream_HighSampleRateSingleMicSystem_parm denoise_parm_3 = {
    .AggressFactor = 1.0f,
    .minSuppress = 0.1f,/*!< 最小压制,建议范围[0.04 - 0.5], 越大降得越少 */
    .init_noise_lvl = 0.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};

const struct __stream_HighSampleRateSingleMicSystem_parm denoise_parm_4 = {
    .AggressFactor = 1.0f,
    .minSuppress = 0.05f,/*!< 最小压制,建议范围[0.04 - 0.5], 越大降得越少 */
    .init_noise_lvl = 0.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};
#endif
#if WIRELESS_LLNS_ENABLE
const struct __stream_llns_parm llns_parm_1 = {
    .gainfloor = 0.5f,
    .suppress_level = 1.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};
const struct __stream_llns_parm llns_parm_2 = {
    .gainfloor = 0.25f,
    .suppress_level = 1.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};
const struct __stream_llns_parm llns_parm_3 = {
    .gainfloor = 0.1f,
    .suppress_level = 1.0f,
    .frame_len = WIRELESS_CODING_FRAME_LEN,
};
#endif
#if WIRELESS_PLATE_REVERB_ENABLE
Plate_reverb_parm plate_reverb_parm = {
    .wet = 40,                      //0-300%
    .dry = 80,                      //0-200%
    .pre_delay = 0,                 //0-40ms
    .highcutoff = 12200,                //0-20k 高频截止
    .diffusion = 43,                  //0-100%
    .decayfactor = 70,                //0-100%
    .highfrequencydamping = 26,       //0-100%
    .modulate = 1,                  // 0或1
    .roomsize = 100,                   //20%-100%
};
#endif

#if WIRELESS_ECHO_ENABLE
#ifndef CONFIG_EFFECT_CORE_V2_ENABLE
ECHO_PARM_SET echo_parm = {
    .delay = 120,//200,				//回声的延时时间 0-300ms
    .decayval = 35,//50,				// 0-70%
    .direct_sound_enable = 1,	//直达声使能  0/1
    .filt_enable = 1,			//发散滤波器使能
};
EF_REVERB_FIX_PARM echo_fix_parm = {
    .wetgain = 2048,			////湿声增益：[0:4096]
    .drygain = 4096,				////干声增益: [0:4096]
    .sr = WIRELESS_CODING_SAMPLERATE,		////采样率
    .max_ms = 200,				////所需要的最大延时，影响 need_buf 大小
};
#else
ECHO_PARM_SET echo_parm = {
    .delay = 120,                      //回声的延时时间 0-max_ms, 单位ms
    .decayval = 35,                   // 0-70%
    .filt_enable = 1,                //滤波器使能标志
    .lpf_cutoff = 5000,                 //0-20k
    .wetgain = 2048,                    //0-200%
    .drygain = 4096,                    //0-100%
};
EF_REVERB_FIX_PARM echo_fix_parm = {
    .sr = WIRELESS_CODING_SAMPLERATE,		////采样率
    .max_ms = 200,				////所需要的最大延时，影响 need_buf 大小
};
#endif
#endif


//从机音频设置
#if (WIRELESS_NOISEGATE_EN)
/* #ifndef CONFIG_EFFECT_CORE_V2_ENABLE */
/* const NOISEGATE_PARM slave_noisegate_parm = { */
/* .attackTime = 300, */
/* .releaseTime = 5, */
/* .threshold = -45000, */
/* .low_th_gain = 0, */
/* .sampleRate = WIRELESS_CODING_SAMPLERATE, */
/* .channel = 1, */
/* }; */
/* #else//701N跑下面的配置 */
const NoiseGateParam slave_noisegate_parm = {
    .attackTime = 300,
    .releaseTime = 5,
    .threshold = -45000,
    .low_th_gain = 0,
    .sampleRate = WIRELESS_CODING_SAMPLERATE,
    .channel = 1,
    .IndataInc = 0,
    .OutdataInc = 0,
};
/* #endif */
#endif//WIRELESS_NOISEGATE_EN
#if TCFG_EQ_ENABLE
#if TCFG_AEC_DCCS_EQ_ENABLE
struct stream_eq_dccs_parm dccs_parm = {
    .sr = WIRELESS_CODING_SAMPLERATE,
};
#endif
#endif
#if WIRELESS_VOICE_CHANGER_EN && TCFG_EFFECT_TOOL_ENABLE
//weak函数，库里面在线调试变声，会将模式通知到APP
void eff_cfg_set_voice_changer_mode(char mode)
{
    printf("%s,mode=%d", __func__, mode);
    user_voice_changer_mode = mode;
}
#endif
static volatile u8 g_ch_type = 0;
void set_ch_type(u8 ch)
{
    printf("%s,ch = %d", __func__, ch);
    g_ch_type = ch;
}

u8 usr_get_ch_type()
{
    return g_ch_type;
}

static void slave_audio_encode_callback(u8 *data, u16 len)
{
    //test code
    /* if (g_ch_type) { */
	
#if WIRELESS_MIC_DAC_ONLINE_CONFIG && TCFG_EQ_ENABLE
    data[0] = eq_online_get_dac_gain() | 0xA0;//0xA0用于校验
#endif

    data[0] = tx_cmd->command;
    data[1] = tx_cmd->channel_flag;
    tx_cmd = 0;
    /* data[1] = 0xBB; */
    /* } else { */
    /* data[0] = 0x55; */
    /* data[1] = 0x66; */
    /* } */
#if 0
    data[0] = g_ch_type;
    data[1] = (u8)app_var.flag_tx_mute; // app_var.flag_tx_mute;
#endif
    // tx_cmd = 0;
}

struct adapter_encoder_fmt slave_audio_stream_parm = {
    .enc_type = ADAPTER_ENC_TYPE_WIRELESS,
    .channel_type = &g_ch_type,//区分左右声道,这里传的是指针，目的可以动态获取
    .command_len = SLAVE_ENC_COMMAND_LEN,
    .command_callback = slave_audio_encode_callback,
};



int encoder_pro_handler(struct audio_stream_entry *entry,  struct audio_data_frame *in)
{
#if WIRELESS_MIC_RECORDER_ENABLE
    if (wl_mic_get_recorder_status()) {
        //putchar('w');
        wireless_mic_recorder_pcm_data_write(in->data, in->data_len);
    }
#endif
	if (/*app_var.flag_tx_mute || */app_var.tx_mute_cnt /*|| (!app_var.tx_mic_gain)*/) {
        memset(in->data, 0x00, in->data_len);
    }
}


static void auto_mute_handler(u8 event, u8 ch)
{
    printf(">>>> ch:%d %s\n", ch, event ? ("MUTE") : ("UNMUTE"));
    if (event) {
        adapter_process_event_notify(ADAPTER_EVENT_MEDIA_MUTE, 0);
    } else {
        adapter_process_event_notify(ADAPTER_EVENT_MEDIA_UNMUTE, 0);
    }
}
static const audio_energy_detect_param auto_mute_parm = {
    .mute_energy = 5,
    .unmute_energy = 10,
    .mute_time_ms = 1000,
    .unmute_time_ms = 100,
    .count_cycle_ms = 10,
    .sample_rate = WIRELESS_DECODE_SAMPLERATE,
    .event_handler = auto_mute_handler,
    .ch_total = 1,//app_audio_output_channel_get(),
    .dcc = 1,
};

static const struct adapter_stream_fmt slave_audio_stream_list[] = {

#if (TCFG_AUTO_SHUT_DOWN_BY_AUTO_MUTE)
    {
        .attr = ADAPTER_STREAM_ATTR_AUTO_MUTE,
        .value = {
            .auto_mute = {
                .parm = &auto_mute_parm,
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            }
        },
    },
#endif

#if 0
    {
        .attr = ADAPTER_STREAM_ATTR_STREAM_DEMO,
        .value = {
            .demo = {
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            },
        },
    },
#endif

#if TCFG_AEC_DCCS_EQ_ENABLE
    {
        .attr  = ADAPTER_STREAM_ATTR_DCCS,
        .value = {
            .dccs = {
                .parm = &dccs_parm,
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            },
        },
    },
#endif
#if WIRELESS_ECHO_ENABLE
    {
        .attr  = ADAPTER_STREAM_ATTR_ECHO,
        .value = {
            .echo = {
                .parm = &echo_parm,
                .fix_parm = &echo_fix_parm,
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            },
        },
    },
#endif
#if WIRELESS_PLATE_REVERB_ENABLE
    {
        .attr  = ADAPTER_STREAM_ATTR_PLATE_REVERB,
        .value = {
            .plate_reverb = {
                .parm = &plate_reverb_parm,
                .samplerate = WIRELESS_CODING_SAMPLERATE,
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            },
        },
    },
    {
        .attr = ADAPTER_STREAM_ATTR_CH_SW,	//reverb会把数据转成双声道，这里要重新转成单声道
        .value = {
            .ch_sw = {
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
                .out_ch_type = AUDIO_CH_DIFF,
                .buf_len = 0,
            },
        },
    },
#endif


#if TCFG_PHONE_EQ_ENABLE
    {
        .attr  = ADAPTER_STREAM_ATTR_MICEQ,
        .value = {
            .miceq = {
                .samplerate = WIRELESS_CODING_SAMPLERATE,
                .ch_num = 1,
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            },
        },
    },
#endif
#if (WIRELESS_DENOISE_ENABLE)
    //降噪节点
    {
        .attr  = ADAPTER_STREAM_ATTR_DENOISE,
        .value = {
            .denoise = {
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
                .sample_rate = 32000,
                .onoff = 0,
                .parm = {
                    .AggressFactor = 1.0f,
                    .minSuppress = 0.1f,
                    .init_noise_lvl = 0.0f,
                    .frame_len = WIRELESS_CODING_FRAME_LEN,
                },
            },
        },
    },
#endif
#if (WIRELESS_LLNS_ENABLE)
    {
        .attr  = ADAPTER_STREAM_ATTR_LLNS,
        .value = {
            .llns = {
                .samplerate  = WIRELESS_CODING_SAMPLERATE,
                .onoff = 0,
                .llns_parm = {
                    .gainfloor = 0.1f,
                    .suppress_level = 1.0f,
                    .frame_len = WIRELESS_CODING_FRAME_LEN,
                },
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
                //其他关于encoder的参数配置
            },
        },
    },
#endif
#if (WIRELESS_NOISEGATE_EN)
    {
        .attr  = ADAPTER_STREAM_ATTR_NOISEGATE,
        .value = {
            .noisegate = {
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
                .parm = &slave_noisegate_parm,
            },
        },
    },
#endif



#if WIRELESS_HOWLING_ENABLE
    {
        .attr  = ADAPTER_STREAM_ATTR_HOWLING,
        .value = {
            .howling = {
                .howl_para = NULL,
                .sample_rate = WIRELESS_CODING_SAMPLERATE,
                .channel = 0,
                .mode = 1,
                .data_handle = NULL,//如果要关注处理前的数据可以在这里注册回调
            },
        },
    },
    {
        .attr  = ADAPTER_STREAM_ATTR_HOWLINGSUPPRESS,
    },
#endif
#if WIRELESS_VOICE_CHANGER_EN
    {
        .attr  = ADAPTER_STREAM_ATTR_VOICE_CHANGER,
    },
#endif
#if WIRELESS_DRC_EN
    {
        .attr  = ADAPTER_STREAM_ATTR_DRC,
    },
#endif
    {
        .attr  = ADAPTER_STREAM_ATTR_ENCODE,
        .value = {
            .encoder = {
                .parm = &slave_audio_stream_parm,
                .data_handle = encoder_pro_handler,//如果要关注处理前的数据可以在这里注册回调
                //其他关于encoder的参数配置
            },
        },
    },
};
static struct adapter_decoder_fmt slave_audio_stream = {
    .dec_type = ADAPTER_DEC_TYPE_MIC,
    .list	  = slave_audio_stream_list,
    .list_num = sizeof(slave_audio_stream_list) / sizeof(slave_audio_stream_list[0]),
};
static const struct adapter_media_fmt slave_media_list[] = {
    [0] = {
        .upstream =	NULL,
        .downstream = &slave_audio_stream,
    },

};

static const struct adapter_media_config slave_media_config = {
    .list = slave_media_list,
    .list_num = sizeof(slave_media_list) / sizeof(slave_media_list[0]),
};

//bt idev config
struct _odev_bt_parm odev_bt_parm_list = {
    .mode = BIT(ODEV_BLE),
};

//bt odev config
#if !WIRELESS_TOOL_BLE_NAME_EN
static u8 dongle_remoter_name1[] = "W_MIC_120";
static client_match_cfg_t match_dev01 = {
    .create_conn_mode = BIT(CLI_CREAT_BY_NAME),
    .compare_data_len = sizeof(dongle_remoter_name1) - 1, //去结束符
    .compare_data = dongle_remoter_name1,
    .bonding_flag = 0,
};
#else
static u8 dongle_remoter_name1[] = "W_MIC_05";
static u8 *ble_pair_name;
client_match_cfg_t match_dev01 = {0};
#endif

void tx_send_data_to_rx(struct mic_tx2rx_info* mic_info)
{
    tx_cmd = mic_info;
    printf("command = %d,channel_flag=%d", tx_cmd->command,tx_cmd->channel_flag);
}

u8 usr_get_denoise_status()
{
    return (app_var.flag_wlm_denoise[4]==1);
    // return (app_var.flag_wlm_denoise[0] == 0x55);
	// return (command==WIRELESS_MIC_DENOISE_LEVEL_MAX);
}
u8 usr_get_mute_status()
{
	return app_var.flag_tx_mute;
}
u8 usr_get_echo_status()
{
	return (echo_cmd==WIRELESS_MIC_ECHO_ON);
}

#if WIRELESS_VOICE_CHANGER_EN
//参数调节的趋势：shiftv越小，音高越高， formant_shift越小，音色越明亮
typedef enum {
    VOICE_CHANGER_NONE,//原声
    VOICE_CHANGER_UNCLE,//大叔
    VOICE_CHANGER_GODDESS,//女神
    VOICE_CHANGER_BABY,//娃娃音
    VOICE_CHANGER_MONSTER,//怪兽音
    
    VOICE_CHANGER_MAGIC,//魔音女声
    VOICE_CHANGER_DONALD_DUCK,//唐老鸭
    VOICE_CHANGER_MINIONS,//小黄人
    VOICE_CHANGER_ROBOT,//机器音
    VOICE_CHANGER_WHISPER,//气音
    VOICE_CHANGER_MELODY,//固定旋律音
    VOICE_CHANGER_FEEDBACK,//调制音

    VOICE_CHANGER_MAX,

} VOICE_CHANGER_MODE;

static void audio_voice_changer_mode_switch(u32 voice_changer_name, VOICE_CHANGER_MODE voice_changer_mode)
{
    VOICECHANGER_PARM parm = {0};
    switch (voice_changer_mode) {
    case VOICE_CHANGER_NONE://原声
        // printf("VOICE_CHANGER_NONE\n");
        audio_voice_changer_bypass(voice_changer_name, RUN_BYPASS);
        break;
    case VOICE_CHANGER_UNCLE://大叔
        // printf("VOICE_CHANGER_UNCLE\n");
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 130;
        parm.formant_shift = 100;//无效参数
        parm.effect_v = EFFECT_VOICECHANGE_PITCHSHIFT;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_GODDESS://女神
        // printf("VOICE_CHANGER_GODDESS\n");
        // audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        // parm.shiftv = 56;
        // parm.formant_shift = 90;
        // parm.effect_v = EFFECT_VOICECHANGE_SPECTRUM;
        // audio_voice_changer_update_parm(voice_changer_name, &parm);

        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 64;
        parm.formant_shift = 60;
        parm.effect_v = EFFECT_VOICECHANGE_PITCHSHIFT;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_BABY://娃娃音
        // printf("VOICE_CHANGER_BABY\n");
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 50;
        parm.formant_shift = 100;//无效参数
        parm.effect_v = EFFECT_VOICECHANGE_PITCHSHIFT;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_MAGIC://魔音女声
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 75;
        parm.formant_shift = 80;
        parm.effect_v = EFFECT_VOICECHANGE_PITCHSHIFT;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_MONSTER://怪兽音
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 160;
        parm.formant_shift = 100;//无效参数
        parm.effect_v = EFFECT_VOICECHANGE_PITCHSHIFT;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_DONALD_DUCK://唐老鸭
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 60;
        parm.formant_shift = 170;
        parm.effect_v = EFFECT_VOICECHANGE_CARTOON;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_MINIONS://小黄人
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 50;
        parm.formant_shift = 60;
        parm.effect_v = EFFECT_VOICECHANGE_CARTOON;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_ROBOT://机器音
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 70;
        parm.formant_shift = 80;
        parm.effect_v = EFFECT_VOICECHANGE_ROBORT;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_WHISPER://气音
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 70;
        parm.formant_shift = 80;
        parm.effect_v = EFFECT_VOICECHANGE_WHISPER;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_MELODY://固定旋律音
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 70;
        parm.formant_shift = 80;
        parm.effect_v = EFFECT_VOICECHANGE_MELODY;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    case VOICE_CHANGER_FEEDBACK://调制音
        audio_voice_changer_bypass(voice_changer_name, RUN_NORMAL);
        parm.shiftv = 150;
        parm.formant_shift = 80;
        parm.effect_v = EFFECT_VOICECHANGE_FEEDBACK;
        audio_voice_changer_update_parm(voice_changer_name, &parm);
        break;
    default:
        break;
    }
}
#endif

#if WIRELESS_PAIR_BONDING
extern void clear_bonding_info();
static u8 pair_first = 0;
static u32 bond_pair_cnt = 0;
#endif

u8 flag_mute_exdac = 0;
extern u8 flag_key_sw_recorder;
extern ble_state_e get_adapter_server_ble_status(void);
static int adapter_key_event_handler(struct sys_event *event)
{
    int ret = 0;
    struct key_event *key = &event->u.key;

    u16 key_event = event->u.key.event;

    static u8 key_poweroff_cnt = 0;
    static u8 flag_poweroff = 0;

    if(key_event == KEY_WIRELESS_MIC_DENOISE_SET){
        return ret;
    }
    
    printf("key_event:%d %d %d\n", key_event, key->value, key->event);
    if(key_event == KEY_WLM_DENOISE_SW){
        key_event = KEY_WIRELESS_MIC_DENOISE_SET;
    }
    printf("key_event:%d %d %d\n", key_event, key->value, key->event);


    switch (key_event) {
    //case KEY_MUSIC_PP:
    //case KEY_MUSIC_NEXT:
    //    tx_send_data_to_rx(key_event);
    //    break;
#if WIRELESS_MIC_RECORDER_ENABLE
    case KEY_RECORD_SW:
        if (!adapter_get_sd_online_status() || adapter_get_otg_online_status()) {
            //sd卡不在线或者otg在线，不能录音
            printf("can't recode !!!!!!");
            break;
        }
        flag_key_sw_recorder = !flag_key_sw_recorder;
        printf("flag_key_sw_recorder=%d", flag_key_sw_recorder);
        if (flag_key_sw_recorder) {
            wireless_mic_recorder_start();
        } else {
            wireless_mic_recorder_stop();
        }
        break;
#endif
#if (WIRELESS_LLNS_ENABLE)
    case KEY_WIRELESS_MIC_DENOISE_SET:
#if !ALWAYS_RUN_STREAM	//未连接也可以设置降噪档位
        if (!wireless_conn_status) {
            printf("no conn,break");
            break;
        }
#endif
        if(app_var.flag_pwr_on_denoise){
			printf("--app_var.flag_pwr_on_denoise------adapter_key_event_handler-----------\n");
			syscfg_read(CFG_USER_DENISE_MEM, &command, 1);
			app_var.flag_pwr_on_denoise = 0;
			if ((command <= WIRELESS_MIC_DENOISE_OFF)||(command > WIRELESS_MIC_DENOISE_LEVEL_MAX)){
            	command = WIRELESS_MIC_DENOISE_LEVEL_1; // WIRELESS_MIC_DENOISE_OFF;
        	}

			if(command == WIRELESS_MIC_DENOISE_LEVEL_1){
	            adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SET_PARM, 1, (int *)&llns_parm_1); 
			}else{
	            adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SET_PARM, 1, (int *)&llns_parm_3); 
			}
			break;
		}
		
		if(app_var.flag_tx_mute){
			break;
		}
        //command++;
        if (command == WIRELESS_MIC_DENOISE_LEVEL_MAX) {
            command = WIRELESS_MIC_DENOISE_LEVEL_1; // WIRELESS_MIC_DENOISE_OFF;
        } else {
            command = WIRELESS_MIC_DENOISE_LEVEL_MAX;
        }

		syscfg_write(CFG_USER_DENISE_MEM, &command, 1);
        app_var.cnt_denoise = 10*2; // 2s

		if(app_var.flag_tx_mute){
			app_var.flag_tx_mute=0;
            flag_mute_exdac = 0;
			//audio_set_mic_mute(0);
		}
        printf("KEY_WIRELESS_MIC_LLNS_SET %d\n", command);
        if (wireless_mic_media) {
            switch (command) {
            case 1:
                //关闭降噪
                // adapter_media_stop(wireless_mic_media);
                // adapter_media_start(wireless_mic_media);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SWITCH, 1, (int *)0);
                break;
            case 2:
                //轻度降噪
                /* adapter_media_stop(wireless_mic_media); */
                /* adapter_media_start(wireless_mic_media); */
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SET_PARM, 1, (int *)&llns_parm_1);
                break;
            case 3:
                //中度降噪
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SET_PARM, 1, (int *)&llns_parm_2);
                break;
            case 4:
                //深度降噪
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_LLNS_SET_PARM, 1, (int *)&llns_parm_3);
                break;
            default:
                break;
            }
        }
        break;
#endif


#if (WIRELESS_DENOISE_ENABLE)
    case KEY_WIRELESS_MIC_DENOISE_SET:
        //主机响应实际按键事件， 通过命令转发到从机， 由从机实现降噪
        //主机发出按键降噪设置命令, 1:关闭降噪， 2:轻度降噪，3:中度降噪，4:深度降噪
        //主机的降噪等级可以通过VM记忆， 根据方案需求自行添加， 记忆command即可，每次开机读取默认值
#if !ALWAYS_RUN_STREAM
        if (!wireless_conn_status) {
            printf("no conn,break");
            break;
        }
#endif
        command++;
        if (command > WIRELESS_MIC_DENOISE_LEVEL_MAX) {
            command = WIRELESS_MIC_DENOISE_OFF;
        }
        printf("KEY_WIRELESS_MIC_DENOISE_SET %d\n", command);
        struct mic_tx2rx_info *mic_tx2rx_info = (struct mic_tx2rx_info *)zalloc(sizeof(struct mic_tx2rx_info));
        mic_tx2rx_info->channel_flag = 0x66;
        mic_tx2rx_info ->command =command;
        tx_send_data_to_rx(mic_tx2rx_info);
        // if (wireless_mic_media) {
        //     switch (command) {
        //     case 1:
        //         //关闭降噪
        //         /* adapter_media_stop(wireless_mic_media); */
        //         /* adapter_media_start(wireless_mic_media); */
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)0);
        //         break;
        //     case 2:
        //         //轻度降噪
        //         /* adapter_media_stop(wireless_mic_media); */
        //         /* adapter_media_start(wireless_mic_media); */
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_1);
        //         break;
        //     case 3:
        //         //中度降噪
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_2);
        //         break;
        //     case 4:
        //         //深度降噪
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
        //         adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_3);
        //         break;
        //     default:
        //         break;
        //     }
        // }
        break;
    case KEY_DENOISE_ON:
        printf("KEY_DENOISE_ON=%d\n", app_var.flag_wlm_denoise[1]);
        app_var.cnt_low_power_denoise = 10;
        switch (app_var.flag_wlm_denoise[1]){
           case 1:
                printf("关闭降噪\n");
                gpio_set_direction(TCFG_LED_BLUE_PIN, 1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_1);
                break;
            case 2:
                printf("1级降噪\n");
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_2);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 0);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 1);
                break;
            case 3:
                printf("2级降噪\n");
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_3);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 0);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 1);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 0);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 1);
                break;
            case 4:
                printf("3级降噪\n");
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_4);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 0);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 1);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 0);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 1);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 0);
                os_time_dly(100);
                gpio_set_direction(TCFG_LED_BLUE_PIN, 0);
                gpio_set_output_value(TCFG_LED_BLUE_PIN, 1);
                break;
            default:
                printf("关闭降噪\n");
                gpio_set_direction(TCFG_LED_BLUE_PIN, 1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_1);
                break;
        }
        app_var.flag_wlm_denoise[4] = 1;
        break;
    case KEY_DENOISE_DN:
        printf("KEY_DENOISE_DN\n");
        adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SWITCH, 1, (int *)1);
        adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_DENOISE_SET_PARM, 1, (int *)&denoise_parm_1);
        app_var.flag_wlm_denoise[4] = 0;
        break;
    case KEY_WLM_DENOISE_SW:
        if(!wireless_conn_status){
            break;
        }

        // if(app_var.flag_wlm_denoise[0] != 0x55){
        //     app_var.flag_wlm_denoise[0] = 0x55; // denoise on.
        // }else{
        //     app_var.flag_wlm_denoise[0] = 0x66; // denoise dn.
        // }
        // syscfg_write(CFG_USER_WLM_DENOISE_GEAR, app_var.flag_wlm_denoise, 2);
        // y_printf("KEY_MUSIC_PP=%x:%d\n", app_var.flag_wlm_denoise[0], app_var.flag_wlm_denoise[1]);
        // tx_send_data_to_rx();
       // tx_send_data_to_rx(0x66);
        break;
#endif//#if (WIRELESS_DENOISE_ENABLE)

    case KEY_WLM_EQ_SW:
        extern void user_eq_file_change(u8 index);
        static u8 index = 0;
        index++;
        if(index>4){
            index = 0;
        }
        printf("KEY_WLM_EQ_SW:%d\n", index);
        user_eq_file_change(index);
        break;
#if WIRELESS_ECHO_ENABLE||WIRELESS_PLATE_REVERB_ENABLE

    case KEY_WIRELESS_MIC_ECHO_SET:
        if (!wireless_conn_status) {
            printf("no conn,break");
            break;
        }
		if(app_var.flag_tx_mute){
			break;
		}
		
        echo_cmd++;
        if (echo_cmd > WIRELESS_MIC_ECHO_ON) {
            echo_cmd = WIRELESS_MIC_ECHO_OFF;
        }
        printf("KEY_WIRELESS_MIC_ECHO %d\n", echo_cmd);
		
		if(app_var.flag_tx_mute){
			app_var.flag_tx_mute=0;
            flag_mute_exdac = 0;
			//audio_set_mic_mute(0);
		}

        if (wireless_mic_media) {
            adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_ECHO_EN, 1, (int *)(echo_cmd));
        }
        break;
#endif//
    case KEY_USR_DUT:
        if(!wireless_conn_status){
        #if TCFG_RF_TEST_EN
            rf_test_process();
        #endif
        }
        break;
    case KEY_SW_SAMETIME_OUTPUT:
        //开关同时输出到dac
        flag_mute_exdac = !flag_mute_exdac;
        printf("flag_mute_exdac = %d", flag_mute_exdac);
        break;
#if WIRELESS_VOICE_CHANGER_EN
    case KEY_VOICE_CHANGE_SW:
#ifndef CONFIG_EFFECT_CORE_V2_ENABLE
        EQ_CFG *eq_cfg = get_eq_cfg_hdl();
        user_voice_changer_mode++;
        if (user_voice_changer_mode >= WIRELESS_VOICE_CHANGER_MAX_NUM) {
            user_voice_changer_mode = 0;
            // audio_voice_changer_bypass(AEID_MIC_VOICE_CHANGER, 1); // bypass.
        #if WIRELESS_ECHO_ENABLE
            echo_cmd = WIRELESS_MIC_ECHO_OFF;
            if (wireless_mic_media) {
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_ECHO_EN, 1, (int *)(echo_cmd));
            }
        #endif
        }else if(user_voice_changer_mode==1){
            // audio_voice_changer_bypass(AEID_MIC_VOICE_CHANGER, 0);
        #if WIRELESS_ECHO_ENABLE
            echo_cmd = WIRELESS_MIC_ECHO_ON;
            if (wireless_mic_media) {
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_ECHO_EN, 1, (int *)(echo_cmd));
            }
        #endif
        }
        printf("KEY_VOICE_CHANGE_SW,mode=%d", user_voice_changer_mode);
        // audio_voice_changer_update_parm(AEID_MIC_VOICE_CHANGER, &eq_cfg->cfg_parm[wirless_mic_mode].voice_changer_parm.parm[user_voice_changer_mode]);
        audio_voice_changer_mode_switch(AEID_MIC_VOICE_CHANGER, user_voice_changer_mode);
        break;
#endif
    case KEY_VOICE_CHANGE_SW_BY_ID:
    #ifndef CONFIG_EFFECT_CORE_V2_ENABLE
        user_voice_changer_mode = app_var.flag_wlm_denoise[3];
        if (user_voice_changer_mode >= WIRELESS_VOICE_CHANGER_MAX_NUM) {
            user_voice_changer_mode = 0;
            // audio_voice_changer_bypass(AEID_MIC_VOICE_CHANGER, 1); // bypass.
        #if WIRELESS_ECHO_ENABLE
            echo_cmd = WIRELESS_MIC_ECHO_OFF;
            if (wireless_mic_media) {
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_ECHO_EN, 1, (int *)(echo_cmd));
            }
        #endif
        }else if(user_voice_changer_mode==1){
            // audio_voice_changer_bypass(AEID_MIC_VOICE_CHANGER, 0);
        #if WIRELESS_ECHO_ENABLE
            echo_cmd = WIRELESS_MIC_ECHO_ON;
            if (wireless_mic_media) {
                adapter_decoder_ioctrl(wireless_mic_media->downdecode, ADAPTER_DEC_IOCTRL_CMD_ECHO_EN, 1, (int *)(echo_cmd));
            }
        #endif
        }
        printf("KEY_VOICE_CHANGE_SW_BY_ID,mode=%d", user_voice_changer_mode);
        audio_voice_changer_mode_switch(AEID_MIC_VOICE_CHANGER, user_voice_changer_mode);
    #endif
        break;
#endif
    case  KEY_POWEROFF:
        key_poweroff_cnt = 0;
        flag_poweroff = 1;
        break;
    case  KEY_POWEROFF_HOLD:
        printf("KEY POWEROFF_HOLD\n");
        if (flag_poweroff) {
            if (++key_poweroff_cnt >= POWER_OFF_CNT) {
                key_poweroff_cnt = 0;
                ret = 1;
            }
        }
    #if WIRELESS_PAIR_BONDING
        else{
            if(bond_pair_cnt<100){
                bond_pair_cnt++;
            }

            if((pair_first==0) && (bond_pair_cnt>=28)){
                y_printf("bond_pair_cnt=%d\n", bond_pair_cnt);

                clear_bonding_info();
                pair_first = 1;
                app_var.wlm_pair_clear = 10*60*5;

                ble_module_enable(1);
            }
        }
    #endif
        break;
    case KEY_POWEROFF_UP:
    #if WIRELESS_PAIR_BONDING
        printf("KEY_POWEROFF_UP\n");
        if(bond_pair_cnt>=10){
            // gpio_longpress_pin0_reset_config(IO_PORTB_01, 0, 8);
        }
        bond_pair_cnt = 0;
    #endif
        break;
    case  KEY_NULL:
        break;
	case KEY_MUSIC_PP:
		if(!app_var.flag_tx_conn){
			break;
        }

		app_var.flag_tx_mute = !app_var.flag_tx_mute;
        flag_mute_exdac = app_var.flag_tx_mute;
        // tx_send_data_to_rx(app_var.flag_tx_mute);
		break;
	case  KEY_VOL_UP:
		if(!app_var.flag_tx_conn || app_var.flag_tx_mute){
			break;
        }

        if(app_var.tx_mic_gain<3){
            app_var.tx_mic_gain = 3;
        }/*else if(app_var.tx_mic_gain<5){
            app_var.tx_mic_gain = 5;
        }*/else if(app_var.tx_mic_gain<6){
            app_var.tx_mic_gain = 6;
        }else if(app_var.tx_mic_gain<9){
            app_var.tx_mic_gain = 9;
        }else if(app_var.tx_mic_gain<12){
            app_var.tx_mic_gain = 12;
        }else{
			app_var.tx_mic_gain=15;
            usr_led_vol_deal(1);
		}

		printf("KEY_VOL_UP:%d\n", app_var.tx_mic_gain);
		user_audio_adc_mic_set_gain(app_var.tx_mic_gain);

        syscfg_write(CFG_USER_MICVOL_MEM, &app_var.tx_mic_gain, 1);
        break;
	case  KEY_VOL_DOWN:
		if(!app_var.flag_tx_conn || app_var.flag_tx_mute){
			break;
        }

        if(app_var.tx_mic_gain>12){
            app_var.tx_mic_gain = 12;
        }else if(app_var.tx_mic_gain>9){
            app_var.tx_mic_gain = 9;
        }else if(app_var.tx_mic_gain>6){
            app_var.tx_mic_gain = 6;
        }/*else if(app_var.tx_mic_gain>5){
            app_var.tx_mic_gain = 5;
        }*/else if(app_var.tx_mic_gain>3){
            app_var.tx_mic_gain = 3;
        }else{
			app_var.tx_mic_gain=1;
            usr_led_vol_deal(0);
		}

		printf("KEY_VOL_DOWN:%d\n", app_var.tx_mic_gain);
		user_audio_adc_mic_set_gain(app_var.tx_mic_gain);

        syscfg_write(CFG_USER_MICVOL_MEM, &app_var.tx_mic_gain, 1);
        break;
    }
    return ret;
}

// tx 接收 rx 发过来的数据
void wireless_mic_server_recieve_data(void *priv, void *data, u16 len)
{
    r_printf("wireless_mic_server_recieve_data=%d\n", len);
    // return ;
    put_buf(data, len);

    u8 ID = ((u8*)data)[0];
    u8 ID2 = ((u8*)data)[1];
    // set_ch_type(ID);
    app_var.flag_wlm_denoise[0] = ID;
    // app_var.flag_wlm_denoise[1] = ID2;

    if(ID==0x66){
        app_task_put_key_msg(KEY_DENOISE_ON, 0);
        app_var.flag_wlm_denoise[1] = ID2;
        command = ID2;
    }else if(ID==0x55){
        app_task_put_key_msg(KEY_DENOISE_ON, 0);
        app_var.flag_wlm_denoise[1] = ID2;
        command = ID2;
    }else if(ID==0x44){
        app_task_put_key_msg(KEY_VOICE_CHANGE_SW, 0);
    }else if(ID==0x33){
        app_task_put_key_msg(KEY_VOICE_CHANGE_SW_BY_ID, 0);
        app_var.flag_wlm_denoise[3] = ID2;
    }

    // tx_send_data_to_rx(app_var.flag_tx_mute);
}


void wlm_denoise_action(u8 on)
{
    struct mic_tx2rx_info *mic_tx2rx_info = (struct mic_tx2rx_info *)zalloc(sizeof(struct mic_tx2rx_info));
    if(on){
        app_task_put_key_msg(KEY_DENOISE_ON, 0);
        mic_tx2rx_info->channel_flag =0x55;
        tx_send_data_to_rx(mic_tx2rx_info);
    }else{
        app_task_put_key_msg(KEY_DENOISE_DN, 0);
        mic_tx2rx_info->channel_flag =0x66;
        tx_send_data_to_rx(mic_tx2rx_info);
    }
}

void usr_tx_init()
{
    app_var.flag_tx_conn = 0;
	app_var.flag_pwr_on_denoise = 1;
#if USER_LED_EN	
	sys_timer_add(NULL, led_scan, 10); // 10ms
#endif
}

void usr_tx_conn_deal()
{
    g_printf("usr_tx_conn_deal\n");
    app_var.flag_tx_conn = 1;
    app_var.tx_mute_cnt = 150;
    app_var.tx_mic_gain = WIRELESS_MIC_ADC_GAIN;
}

void usr_tx_dconn_deal()
{
    g_printf("usr_tx_dconn_deal\n");
    ui_update_status(STATUS_BT_DISCONN);
    app_var.flag_tx_conn = 0;
    app_var.flag_tx_mute = 0;
    flag_mute_exdac = 0;
    app_var.tx_mute_cnt = 150;
    app_var.flag_pwr_on_denoise = 1;
    command = WIRELESS_MIC_DENOISE_OFF;
    command = WIRELESS_MIC_ECHO_OFF;
    sys_auto_shut_down_enable();
}

static int event_handle_callback(struct sys_event *event)
{
    //处理用户关注的事件
    int ret = 0;
    switch (event->type) {
    case SYS_KEY_EVENT:
        ret = adapter_key_event_handler(event);
        break;
    case SYS_DEVICE_EVENT:
        app_common_device_event_deal(event);
#if (TCFG_DEV_MANAGER_ENABLE)
        dev_status_event_filter(event);
#endif
        switch ((u32)event->arg) {
        case DEVICE_EVENT_FROM_ADAPTER:
            switch (event->u.dev.event) {
            case ADAPTER_EVENT_CONNECT :
                printf("ADAPTER_EVENT_CONNECT\n");
                wireless_conn_status = 1;
                ui_update_status(STATUS_BT_CONN);
                sys_auto_shut_down_disable();
                break;
            case ADAPTER_EVENT_DISCONN :
                printf("ADAPTER_EVENT_DISCONN\n");
                wireless_conn_status = 0;
                ui_update_status(STATUS_BT_DISCONN);
                sys_auto_shut_down_enable();
                break;
            case ADAPTER_EVENT_IDEV_MEDIA_CLOSE :
            case ADAPTER_EVENT_ODEV_MEDIA_CLOSE :
                break;
            case ADAPTER_EVENT_POWEROFF:
                ret = 1;
                break;
            case ADAPTER_EVENT_MEDIA_MUTE:
                sys_auto_shut_down_enable();
                break;
            case ADAPTER_EVENT_MEDIA_UNMUTE:
                sys_auto_shut_down_disable();
                break;
            }
            break;
        }
        break;

    default:
        break;

    }
    return ret;
}

void wireless_cfg_tool_read_name(void)
{
    u8 ble_name_len;
    extern const char *bt_get_local_name();
    ble_pair_name = (u8 *)(bt_get_local_name());
    ble_name_len = strlen(ble_pair_name);

    match_dev01.create_conn_mode = BIT(CLI_CREAT_BY_NAME);
    match_dev01.compare_data_len = ble_name_len;
    match_dev01.compare_data = ble_pair_name;
    match_dev01.bonding_flag = 0;

}
void app_main_run(void)
{
    ui_update_status(STATUS_POWERON);
#if WIRELESS_TOOL_BLE_NAME_EN
    wireless_cfg_tool_read_name();
#endif

    while (1) {
        //初始化
        struct idev *idev = adapter_idev_open(ADAPTER_IDEV_MIC, NULL);
        struct odev *odev = adapter_odev_open(ADAPTER_ODEV_BT, (void *)&odev_bt_parm_list);
        struct adapter_media *media = adapter_media_open((struct adapter_media_config *)&slave_media_config);
        printf("wireless_mic_2t1_tx ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

        ASSERT(idev);
        ASSERT(odev);
        ASSERT(media);
        struct adapter_pro *pro = adapter_process_open(idev, odev, media, event_handle_callback);//event_handle_callback 用户想拦截处理的事件

        ASSERT(pro, "adapter_process_open fail!!\n");

        wireless_mic_media = media;

        //执行(包括事件解析、事件执行、媒体启动/停止, HID等事件转发)
        adapter_process_run(pro);
#if WIRELESS_MIC_RECORDER_ENABLE
        if (wl_mic_get_recorder_status()) {
            wireless_mic_recorder_stop();
        }
#endif
        wireless_mic_media = NULL;

        //退出/关闭
        adapter_process_close(&pro);
        adapter_media_close(&media);
        adapter_idev_close(idev);
        adapter_odev_close(odev);

        ui_update_status(STATUS_POWEROFF);
        ///run idle off poweroff
        printf("enter poweroff !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        if (get_charge_online_flag()) {
            printf("charge_online,cpu reset");
            cpu_reset();
        } else {
            power_set_soft_poweroff();
        }
    }
}


#endif

