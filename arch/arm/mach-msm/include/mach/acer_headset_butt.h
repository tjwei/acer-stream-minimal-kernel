/*
 * Acer Headset device button driver.
 *
 *
 * Copyright (C) 2008 acer Corporation.
 *
 * Authors:
 *    Shawn Tu <Shawn_Tu@acer.com.tw>
 */
#include <mach/msm_rpcrouter.h>

#define ADC_RPC_PROG     0x30000060
#define ADC_RPC_VERS     0x00010002
#define ADC_PROC         0x70

struct hs_butt_gpio {
	int gpio_hs_butt;
	int gpio_hs_dett;
	int gpio_hs_mic;
};

extern void set_hs_state(bool state);
extern void set_hs_type_state(bool state);
extern bool get_hs_type_state(void);
extern int get_butt_adc(void);

#pragma pack(1)

typedef struct _adc_req{
	struct rpc_request_hdr req_hdr;
	unsigned int req;
} adc_req;

typedef struct _adc_rep{
	struct rpc_reply_hdr req_hdr;
	unsigned int reply;
} adc_rep;

#pragma pack()
