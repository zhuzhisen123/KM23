#ifndef APP_MAIN_H
#define APP_MAIN_H

typedef struct _APP_VAR {
    s8 music_volume;
    s8 call_volume;
    s8 wtone_volume;
    u8 opid_play_vol_sync;
    u8 aec_dac_gain;
    u8 aec_mic_gain;
    u8 rf_power;
    u8 poweron_charge;                  //开机进充电标志
    u8 goto_poweroff_flag;
    u8 goto_poweroff_cnt;
    u8 play_poweron_tone;
    u8 remote_dev_company;
    u8 siri_stu;
    int auto_stop_page_scan_timer;     //用于1拖2时，有一台连接上后，超过三分钟自动关闭Page Scan
    volatile int auto_shut_down_timer;
    volatile int wait_exit_timer;
    u16 auto_off_time;
    u16 warning_tone_v;
    u16 poweroff_tone_v;
    u32 start_time;
    s8  usb_mic_gain;
    u8  reverb_status;                 //用于tws+混响在pc模式时，同步关闭混响, 0:非pc模式
    u8 	cycle_mode;
    u8 need_save_box_lvl;
    u8 chargestore_online;
    u8 charge_full_flag;
    u8 flag_rf_dut;
    u8 flag_tx_mute;
    u8 flag_tx_conn;
    u8 flag_pwr_on_denoise;
    u8 flag_led_anti;
    u8 tx_mute_cnt;
    u8 tx_mic_gain;
    u8 cnt_low_power_denoise;
    // bond
    u8 usr_pair_celar_cnt;
    u16 wlm_pair_clear;
    u8 cnt_denoise;
    u8 flag_charge;
    u8 flag_low_pwr;
    u8 flag_vbat_low_pwr;
	u8 flag_wlm_denoise[4];
} APP_VAR;


#define    BT_EMITTER_EN     1
#define    BT_RECEIVER_EN    2

typedef struct _BT_USER_COMM_VAR {
} BT_USER_COMM_VAR;

extern APP_VAR app_var;

extern void app_main();

#define earphone (&bt_user_priv_var)

extern u8 flag_mute_exdac;

extern u8 usr_get_denoise_status();
extern u8 usr_get_mute_status();

extern void usr_tx_init();
extern void usr_tx_conn_deal();
extern void usr_tx_dconn_deal();
#endif
