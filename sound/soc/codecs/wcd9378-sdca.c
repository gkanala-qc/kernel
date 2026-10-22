// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
// Copyright (c) 2025 Qualcomm Technologies, Inc. All rights reserved.

/*
 * WCD9378 (Tambora) SDCA SimpleJack codec. Supplies the static SDCA
 * topology on DT platforms where no ACPI/DisCo enumeration exists.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soundwire/sdw.h>
#include <linux/soundwire/sdw_registers.h>
#include <linux/soundwire/sdw_type.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>
#include <sound/sdca.h>
#include <sound/sdca_class.h>
#include <sound/sdca_function.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include "wcd9378-sdca.h"

struct wcd9378_priv {
	struct sdca_class_drv class;
	/*
	 * MBHC mic bias left raised at 2.7V for a special headset, and the
	 * VOUT_CTL it was raised from, so removal can put it back without
	 * assuming a reset value.  See wcd9378_mbhc_check_spl_headset().
	 */
	unsigned int micb2_vout_saved;
	bool spl_headset;
};

static struct sdca_entity wcd9378_sdca_entities[];
/*
 * Entity array index map. Order matches the ASL entity-id-list;
 * Function (Entity 0) is last.
 *
 *  [0]  E001 IT 41    (0x1)   [12] E00F IT 33    (0xF)
 *  [1]  E002 CS 41    (0x2)   [13] E010 PDE 34   (0x10)
 *  [2]  E003 MFPU 21  (0x3)   [14] E011 FU 33    (0x11)
 *  [3]  E004 XU 42    (0x4)   [15] E012 SU 35    (0x12)
 *  [4]  E007 SU 43    (0x7)   [16] E013 XU 36    (0x13)
 *  [5]  E008 SU 45    (0x8)   [17] E015 CS 36    (0x15)
 *  [6]  E009 PDE 47   (0x9)   [18] E016 OT 36    (0x16)
 *  [7]  E00A OT 43    (0xA)   [19] E017 MFPU 236 (0x17)
 *  [8]  E00B OT 45    (0xB)   [20] E018 CS 236   (0x18)
 *  [9]  E00C GE 35    (0xC)   [21] E019 OT 236   (0x19)
 *  [10] E00D IT 131   (0xD)   [22] E006 FU 6     (0x6)
 *  [11] E00E CS 131   (0xE)   [23] E000 Function (0x0)
 */
#define QSJ_IT41	0
#define QSJ_CS41	1
#define QSJ_MFPU21	2
#define QSJ_XU42	3
#define QSJ_SU43	4
#define QSJ_SU45	5
#define QSJ_PDE47	6
#define QSJ_OT43	7
#define QSJ_OT45	8
#define QSJ_GE35	9
#define QSJ_IT131	10
#define QSJ_CS131	11
#define QSJ_IT33	12
#define QSJ_PDE34	13
#define QSJ_FU33	14
#define QSJ_SU35	15
#define QSJ_XU36	16
#define QSJ_CS36	17
#define QSJ_OT36	18
#define QSJ_MFPU236	19
#define QSJ_CS236	20
#define QSJ_OT236	21
#define QSJ_FU6		22

/* Range data: {cols, rows, data[cols*rows]}, transcribed from ASL. */

/* IT 41 Usage: HIFI (0x2). ULP (0x3) does not route to the HPH analog driver. */
static u32 range_it41_usage_data[] = {
	/* usage, CBN, sample_rate, sample_width, full_scale, noise_floor, tag */
	0x2, 0x2D0, 0xBB80, 0x10, 0x0, 0x0, 0x0,
};

static u32 range_it41_cluster_data[] = { 0x1, 0x1 };

/* IT 41 DataPort selector: DP6 (HPH render). */
static u32 range_it41_dp_data[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* CS 41 SampleRateIndex: 1 -> 48kHz PCM. */
static u32 range_cs41_sr_data[] = { 0x1, 0xBB80 };

/* SU selector: disconnected / connected. */
static u32 range_su_sel_data[] = { 0x0, 0x1 };

/* PDE Requested_PS: PS0 / PS3. */
static u32 range_pde_req_ps_data[] = { 0x0, 0x3 };

/*
 * GE 35 SelectedMode -> terminal type.
 *
 * Modes 3 and 4 are the real results, chosen by the ADC classification in
 * wcd9378_sdca_post_jack_irq_process().  Mode 1 (Unknown) is aliased to
 * Headphone (mode 4) to keep the DAPM path alive; it is unreachable in
 * practice while MBHC_CTRL DEVICE_DET ACCESSORY_TYPE stays at its reset
 * value 3.  Mode 2 (Line-out) is not fitted on this board.  Mode 0
 * (Unplugged/Safe_Mode) has no Terminal Type per the SDCA spec, and is
 * intentionally left out of this table -- sdca_range_search() returns 0
 * ("no terminal") for it, and the GE 35 mode 0 SU route is left
 * disconnected (see ge35_mode0_controls below) instead of aliasing it to a
 * fake entry.
 */
static u32 range_ge35_mode_data[] = {
	0x1, 0x6C0,	/* Unknown -> HPH (alias) */
	0x3, 0x6D0,	/* Headset */
	0x4, 0x6C0,	/* Headphone */
};

/* IT 131 Usage: optimization render stream at 192kHz. */
static u32 range_it131_usage_data[] = {
	0x3, 0x334, 0x2EE00, 0x8, 0x0, 0x0, 0x0,
};

static u32 range_it131_cluster_data[] = { 0x1, 0x3 };

/* IT 131 DataPort selector: DP7. */
static u32 range_it131_dp_data[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* CS 131 SampleRateIndex: 1 -> 192kHz. */
static u32 range_cs131_sr_data[] = { 0x1, 0x2EE00 };

/* IT 33 MIC_BIAS default: 2.75V (patched per-slave in populate_function). */
static u32 range_it33_micbias_data[] = { 0x5 };

static u32 range_it33_usage_data[] = {
	0x1, 0x2C6, 0x0, 0x0, 0x0, 0x0, 0x0,
};

static u32 range_it33_cluster_data[] = { 0x1, 0x2 };

static u32 range_cs36_sr_data[] = { 0x1, 0xBB80 };

/* OT 36 Usage: PDM capture, host-visible 48kHz/16-bit PCM. */
static u32 range_ot36_usage_data[] = {
	0x1, 0x2C6, 0xBB80, 0x10, 0x0, 0x0, 0x0,
};

/* OT 36 DataPort selector: DP2. */
static u32 range_ot36_dp_data[] = {
	0xFF, 0xFF, 0x2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static u32 range_cs236_sr_data[] = { 0x1, 0xBB80 };

/* OT 236 Usage: optimization capture at 192kHz (clocked by CS 131). */
static u32 range_ot236_usage_data[] = {
	0x1, 0x334, 0x2EE00, 0x8, 0x0, 0x0, 0x0,
};

/* OT 236 DataPort selector: DP5. */
static u32 range_ot236_dp_data[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* Entity 0 (Function) Control Values */
static int ctrl_fun_sdca_ver_vals[] = { 0x11 };
static int ctrl_fun_type_vals[] = { 0x08 };	/* SimpleJack */
static int ctrl_fun_man_id_vals[] = { 0x0217 };
static int ctrl_fun_id_vals[] = { 0x3 };
static int ctrl_fun_ver_vals[] = { 0x0 };
static int ctrl_dev_sdca_ver_vals[] = { 0x11 };

/* Entity 1 (IT 41) Control Values */
static int ctrl_it41_latency_vals[] = { 0x0 };
static int ctrl_it41_cluster_vals[] = { 0x1 };
static int ctrl_it41_dp_vals[] = { 0x6 };

/* Entity 2 (CS 41) Control Values */
static int ctrl_cs41_sr_vals[] = { 0x1 };

/* Entity 3 (MFPU 21) Control Values */
static int ctrl_mfpu21_bypass_vals[] = { 0x1 };

/* Entity 4 (XU 42) Control Values */
static int ctrl_xu42_id_vals[] = { 0x2131 };
static int ctrl_xu42_ver_vals[] = { 0x1 };

/* Entity 0xD (IT 131) Control Values */
static int ctrl_it131_latency_vals[] = { 0x0 };
static int ctrl_it131_cluster_vals[] = { 0x1 };
static int ctrl_it131_dp_vals[] = { 0x7 };

/* Entity 0xE (CS 131) Control Values */
static int ctrl_cs131_sr_vals[] = { 0x1 };

/* IT 33 MIC_BIAS: fixed 2.75V (SDCA MIC_BIAS index 0x5), reapplied by PDE34. */
static int ctrl_it33_micbias_vals[] = { 0x5 };
static int ctrl_it33_latency_vals[] = { 0x0 };
static int ctrl_it33_cluster_vals[] = { 0x1 };

/* Entity 0x13 (XU 36) Control Values */
static int ctrl_xu36_bypass_vals[] = { 0x1 };
static int ctrl_xu36_id_vals[] = { 0x2131 };
static int ctrl_xu36_ver_vals[] = { 0x1 };

/* Entity 0x15 (CS 36) Control Values */
static int ctrl_cs36_sr_vals[] = { 0x1 };

/* Entity 0x16 (OT 36) Control Values */
static int ctrl_ot36_latency_vals[] = { 0x0 };
static int ctrl_ot36_dp_vals[] = { 0x2 };

/* Entity 0x17 (MFPU 236) Control Values */
static int ctrl_mfpu236_bypass_vals[] = { 0x1 };

/* Entity 0x18 (CS 236) Control Values */
static int ctrl_cs236_sr_vals[] = { 0x1 };

/* Entity 0x19 (OT 236) Control Values */
static int ctrl_ot236_latency_vals[] = { 0x0 };
static int ctrl_ot236_dp_vals[] = { 0x5 };

/* Entity 0 (Function) Controls */
static struct sdca_control entity0_controls[] = {
	{ .sel = 0x1,  .mode = SDCA_ACCESS_MODE_RW,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_COMMIT_GROUP_MASK_NAME },
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_DC,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_fun_sdca_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_SDCA_VERSION_NAME },
	{ .sel = 0x5,  .mode = SDCA_ACCESS_MODE_DC,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_fun_type_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_TYPE_NAME },
	{ .sel = 0x6,  .mode = SDCA_ACCESS_MODE_DC,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_fun_man_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_MANUFACTURER_ID_NAME },
	{ .sel = 0x7,  .mode = SDCA_ACCESS_MODE_DC,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_fun_id_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_ID_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_fun_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_FUNCTION_VERSION_NAME },
	{ .sel = 0x9,  .mode = SDCA_ACCESS_MODE_RO,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_FUNCTION_EXTENSION_ID_NAME },
	{ .sel = 0xA,  .mode = SDCA_ACCESS_MODE_RO,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_FUNCTION_EXTENSION_VERSION_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RW1C, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .is_volatile = true, .label = SDCA_CTL_FUNCTION_STATUS_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_RW1S, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_FUNCTION_ACTION_NAME },
	{ .sel = 0x2C, .mode = SDCA_ACCESS_MODE_RO,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_DEVICE_MANUFACTURER_ID_NAME },
	{ .sel = 0x2D, .mode = SDCA_ACCESS_MODE_RO,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_DEVICE_PART_ID_NAME },
	{ .sel = 0x2E, .mode = SDCA_ACCESS_MODE_RO,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_DEVICE_VERSION_NAME },
	{ .sel = 0x2F, .mode = SDCA_ACCESS_MODE_DC,   .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_dev_sdca_ver_vals, .has_fixed = true,
	  .label = SDCA_CTL_DEVICE_SDCA_VERSION_NAME },
};

/* Entity 1 (IT 41) Controls */
static struct sdca_control entity_it41_controls[] = {
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x1, .data = range_it41_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it41_latency_vals, .has_fixed = true, .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it41_cluster_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_it41_cluster_data },
	  .label = SDCA_CTL_CLUSTERINDEX_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it41_dp_vals, .has_fixed = true,
	  .range = { .cols = 0x10, .rows = 0x4, .data = range_it41_dp_data },
	  .label = SDCA_CTL_DATAPORT_SELECTOR_NAME },
};

/* Entity 2 (CS 41) Controls */
static struct sdca_control entity_cs41_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_cs41_sr_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_cs41_sr_data },
	  .label = SDCA_CTL_SAMPLERATEINDEX_NAME },
};

/* Entity 3 (MFPU 21) Controls */
static struct sdca_entity *entity_mfpu21_sources[] = {
	&wcd9378_sdca_entities[QSJ_IT41],
	&wcd9378_sdca_entities[QSJ_IT131],
};

static struct sdca_control entity_mfpu21_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_mfpu21_bypass_vals, .has_fixed = true, .label = SDCA_CTL_BYPASS_NAME },
};

/* Entity 4 (XU 42) Controls */
static struct sdca_entity *entity_xu42_sources[] = { &wcd9378_sdca_entities[QSJ_MFPU21] };

static struct sdca_control entity_xu42_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .label = SDCA_CTL_BYPASS_NAME },
	{ .sel = 0x7, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_xu42_id_vals, .has_fixed = true, .label = SDCA_CTL_XU_ID_NAME },
	{ .sel = 0x8, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_xu42_ver_vals, .has_fixed = true, .label = SDCA_CTL_XU_VERSION_NAME },
};

/* Entity 7 (SU 43) Controls */
static struct sdca_entity *entity_su43_sources[] = { &wcd9378_sdca_entities[QSJ_XU42] };

static struct sdca_control entity_su43_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_DEVICE,
	  .cn_list = 0x1,
	  .range = { .cols = 0x1, .rows = 0x2, .data = range_su_sel_data },
	  .label = SDCA_CTL_SELECTOR_NAME },
};

/* Entity 8 (SU 45) Controls */
static struct sdca_entity *entity_su45_sources[] = { &wcd9378_sdca_entities[QSJ_XU42] };

static struct sdca_control entity_su45_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_DEVICE,
	  .cn_list = 0x1,
	  .range = { .cols = 0x1, .rows = 0x2, .data = range_su_sel_data },
	  .label = SDCA_CTL_SELECTOR_NAME },
};

/*
 * PDE 47 manages the HPH render path (OT 43, OT 45). FU 6 is listed
 * so its cached mute/volume are reasserted on PS0 entry.
 */
static struct sdca_entity *entity_pde47_managed[] = {
	&wcd9378_sdca_entities[QSJ_FU6],
	&wcd9378_sdca_entities[QSJ_OT43],
	&wcd9378_sdca_entities[QSJ_OT45],
};

static struct sdca_pde_delay pde47_delays[] = {
	{ .from_ps = 3, .to_ps = 0, .us = 30000 },
	{ .from_ps = 0, .to_ps = 3, .us = 30000 },
};

static struct sdca_control entity_pde47_controls[] = {
	{ .sel = 0x1,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x1, .rows = 0x2, .data = range_pde_req_ps_data },
	  .label = SDCA_CTL_REQUESTED_PS_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .is_volatile = true, .label = SDCA_CTL_ACTUAL_PS_NAME },
	/*
	 * HPH protection IRQs (OCP/CNP/SURGE) fire in codec HW but are
	 * not exposed to Linux; needs an SDCA framework interface for
	 * standalone status-bit IRQs. Wire up in a follow-up.
	 */
};

/* OT 43 (Headphone), OT 45 (Headset): no controls. */
static struct sdca_entity *entity_ot43_sources[] = { &wcd9378_sdca_entities[QSJ_IT41] };
static struct sdca_entity *entity_ot45_sources[] = { &wcd9378_sdca_entities[QSJ_IT41] };

/*
 * GE 35 mode -> SU selector. Mode 1 aliases to Headphone (see
 * range_ge35_mode_data). SU 45 = source 1 (XU 42) for HPH paths,
 * SU 43 = source 1 (XU 42) for Headset. Mode 0 (Unplugged) leaves
 * SU 45 disconnected (val = 0x0) rather than aliasing to HPH -- this
 * is purely DAPM-graph bookkeeping, no register is ever written from
 * this table (see ge_count_routes()/entity_parse_su_device() in
 * sdca_asoc.c), so it has no effect on actual hardware behaviour.
 */
static struct sdca_ge_control ge35_mode0_controls[] = {
	{ .id = 0x8, .sel = 0x1, .cn = 0x0, .val = 0x0 },
};

static struct sdca_ge_control ge35_mode1_controls[] = {
	{ .id = 0x8, .sel = 0x1, .cn = 0x0, .val = 0x1 },
};

static struct sdca_ge_control ge35_mode3_controls[] = {
	{ .id = 0x7, .sel = 0x1, .cn = 0x0, .val = 0x1 },
};

static struct sdca_ge_control ge35_mode4_controls[] = {
	{ .id = 0x8, .sel = 0x1, .cn = 0x0, .val = 0x1 },
};

static struct sdca_ge_mode ge35_modes[] = {
	{ .val = 0x0, .num_controls = ARRAY_SIZE(ge35_mode0_controls), .controls = ge35_mode0_controls },
	{ .val = 0x1, .num_controls = ARRAY_SIZE(ge35_mode1_controls), .controls = ge35_mode1_controls },
	{ .val = 0x3, .num_controls = ARRAY_SIZE(ge35_mode3_controls), .controls = ge35_mode3_controls },
	{ .val = 0x4, .num_controls = ARRAY_SIZE(ge35_mode4_controls), .controls = ge35_mode4_controls },
};

static struct sdca_control entity_ge35_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .interrupt_position = SDCA_NO_INTERRUPT,
	  .range = { .cols = 0x2, .rows = 0x3, .data = range_ge35_mode_data },
	  .label = SDCA_CTL_SELECTED_MODE_NAME },
	{ .sel = 0x2, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .interrupt_position = 4, /* SDCA_4 = GE_DETECTED_MODE, see init_table INTMASK_1 */
	  .is_volatile = true, .label = SDCA_CTL_DETECTED_MODE_NAME },
};

static struct sdca_control entity_it131_controls[] = {
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x1, .data = range_it131_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it131_latency_vals, .has_fixed = true, .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it131_cluster_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_it131_cluster_data },
	  .label = SDCA_CTL_CLUSTERINDEX_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it131_dp_vals, .has_fixed = true,
	  .range = { .cols = 0x10, .rows = 0x4, .data = range_it131_dp_data },
	  .label = SDCA_CTL_DATAPORT_SELECTOR_NAME },
};

/* Entity 0xE (CS 131) Controls */
static struct sdca_control entity_cs131_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_cs131_sr_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_cs131_sr_data },
	  .label = SDCA_CTL_SAMPLERATEINDEX_NAME },
};

/* Entity 0xF (IT 33) - headset mic input */
static struct sdca_control entity_it33_controls[] = {
	{ .sel = 0x3,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it33_micbias_vals, .has_default = true,
	  .range = { .cols = 0x1, .rows = 0x1, .data = range_it33_micbias_data },
	  .label = SDCA_CTL_MIC_BIAS_NAME },
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x1, .data = range_it33_usage_data },
	  .has_reset = true,
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it33_latency_vals, .has_fixed = true, .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_it33_cluster_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_it33_cluster_data },
	  .label = SDCA_CTL_CLUSTERINDEX_NAME },
};

/* Entity 0x10 (PDE 34) - manages IT 33 */
static struct sdca_entity *entity_pde34_managed[] = { &wcd9378_sdca_entities[QSJ_IT33] };

static struct sdca_pde_delay pde34_delays[] = {
	{ .from_ps = 3, .to_ps = 0, .us = 30000 },
	{ .from_ps = 0, .to_ps = 3, .us = 30000 },
};

static struct sdca_control entity_pde34_controls[] = {
	{ .sel = 0x1,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x1, .rows = 0x2, .data = range_pde_req_ps_data },
	  .label = SDCA_CTL_REQUESTED_PS_NAME },
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .is_volatile = true, .label = SDCA_CTL_ACTUAL_PS_NAME },
};

/*
 * FU 6 (vendor FU42): HPH mute + Q7.8 volume. DUAL-mode CVR-alias
 * writes take effect without an SCP_COMMIT.
 */
/*
 * FU 6 volume range: MIN, MAX, STEP in Q7.8 (LSB = 1/256 dB).
 * 0x8000 = -128 dB, 0x7FFF = +127.996 dB, STEP = 1. Framework
 * sign-extends and converts to 0.01 dB TLV via (val * 100) >> 8.
 */
static u32 range_fu6_vol_data[] = {
	0x00008000, 0x00007FFF, 0x00000001,
};

static struct sdca_control entity_fu6_controls[] = {
	{ .sel = SDCA_CTL_FU_MUTE, .mode = SDCA_ACCESS_MODE_DUAL,
	  .layers = SDCA_ACCESS_LAYER_USER, .cn_list = 0x6, .nbits = 1,
	  .type = SDCA_CTL_DATATYPE_ONEBIT,
	  .label = SDCA_CTL_MUTE_NAME },
	{ .sel = SDCA_CTL_FU_CHANNEL_VOLUME, .mode = SDCA_ACCESS_MODE_DUAL,
	  .layers = SDCA_ACCESS_LAYER_USER, .cn_list = 0x6, .nbits = 16,
	  .type = SDCA_CTL_DATATYPE_Q7P8DB,
	  .range = { .cols = SDCA_VOLUME_LINEAR_NCOLS, .rows = 1,
		     .data = range_fu6_vol_data },
	  .label = SDCA_CTL_CHANNEL_VOLUME_NAME },
};

/* Entity 0x11 (FU 33) - no controls */
static struct sdca_entity *entity_fu33_sources[] = { &wcd9378_sdca_entities[QSJ_IT33] };

/* Entity 0x12 (SU 35) Controls */
static struct sdca_entity *entity_su35_sources[] = { &wcd9378_sdca_entities[QSJ_FU33] };

static struct sdca_control entity_su35_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_RO, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x1, .rows = 0x2, .data = range_su_sel_data },
	  .label = SDCA_CTL_SELECTOR_NAME },
};

/* Entity 0x13 (XU 36) Controls */
static struct sdca_entity *entity_xu36_sources[] = { &wcd9378_sdca_entities[QSJ_SU35] };

static struct sdca_control entity_xu36_controls[] = {
	/* bypass=1: pass mic signal through XU36 without proprietary processing */
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_xu36_bypass_vals, .has_default = true, .label = SDCA_CTL_BYPASS_NAME },
	{ .sel = 0x7, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_xu36_id_vals, .has_fixed = true, .label = SDCA_CTL_XU_ID_NAME },
	{ .sel = 0x8, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_xu36_ver_vals, .has_fixed = true, .label = SDCA_CTL_XU_VERSION_NAME },
};

/* Entity 0x15 (CS 36) Controls */
static struct sdca_control entity_cs36_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_cs36_sr_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_cs36_sr_data },
	  .label = SDCA_CTL_SAMPLERATEINDEX_NAME },
};

/* OT 36 mic capture: IT33 -> FU33 -> SU35 -> XU36 -> OT36. */
static struct sdca_entity *entity_ot36_sources[] = { &wcd9378_sdca_entities[QSJ_XU36] };

static struct sdca_control entity_ot36_controls[] = {
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x1, .data = range_ot36_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_ot36_latency_vals, .has_fixed = true, .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_ot36_dp_vals, .has_fixed = true,
	  .range = { .cols = 0x10, .rows = 0x4, .data = range_ot36_dp_data },
	  .label = SDCA_CTL_DATAPORT_SELECTOR_NAME },
};

/* Entity 0x17 (MFPU 236) Controls */
static struct sdca_control entity_mfpu236_controls[] = {
	{ .sel = 0x1, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_mfpu236_bypass_vals, .has_fixed = true, .label = SDCA_CTL_BYPASS_NAME },
};

/* Entity 0x18 (CS 236) Controls */
static struct sdca_control entity_cs236_controls[] = {
	{ .sel = 0x10, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_cs236_sr_vals, .has_fixed = true,
	  .range = { .cols = 0x2, .rows = 0x1, .data = range_cs236_sr_data },
	  .label = SDCA_CTL_SAMPLERATEINDEX_NAME },
};

/* Entity 0x19 (OT 236) - optimization stream capture output */
static struct sdca_entity *entity_ot236_sources[] = { &wcd9378_sdca_entities[QSJ_IT33] };

static struct sdca_control entity_ot236_controls[] = {
	{ .sel = 0x4,  .mode = SDCA_ACCESS_MODE_RW, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .range = { .cols = 0x7, .rows = 0x1, .data = range_ot236_usage_data },
	  .label = SDCA_CTL_USAGE_NAME },
	{ .sel = 0x8,  .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_ot236_latency_vals, .has_fixed = true, .label = SDCA_CTL_LATENCY_NAME },
	{ .sel = 0x11, .mode = SDCA_ACCESS_MODE_DC, .layers = SDCA_ACCESS_LAYER_CLASS, .cn_list = 0x1,
	  .values = ctrl_ot236_dp_vals, .has_fixed = true,
	  .range = { .cols = 0x10, .rows = 0x4, .data = range_ot236_dp_data },
	  .label = SDCA_CTL_DATAPORT_SELECTOR_NAME },
};

static struct sdca_entity wcd9378_sdca_entities[] = {
	/* [0] E001: IT 41 - PDM render stream input */
	{ .id = 0x1, .label = "IT 41", .type = SDCA_ENTITY_TYPE_IT,
	  .iot = { .type = 0x0191, .is_dataport = true, .clock = &wcd9378_sdca_entities[QSJ_CS41] },
	  .num_controls = ARRAY_SIZE(entity_it41_controls), .controls = entity_it41_controls },
	/* [1] E002: CS 41 */
	{ .id = 0x2, .label = "CS 41", .type = SDCA_ENTITY_TYPE_CS,
	  .cs = { .type = 0x0 },
	  .num_controls = ARRAY_SIZE(entity_cs41_controls), .controls = entity_cs41_controls },
	/* [2] E003: MFPU 21 */
	{ .id = 0x3, .label = "MFPU 21", .type = SDCA_ENTITY_TYPE_MFPU,
	  .num_controls = ARRAY_SIZE(entity_mfpu21_controls), .controls = entity_mfpu21_controls,
	  .num_sources = ARRAY_SIZE(entity_mfpu21_sources), .sources = entity_mfpu21_sources },
	/* [3] E004: XU 42 */
	{ .id = 0x4, .label = "XU 42", .type = SDCA_ENTITY_TYPE_XU,
	  .num_controls = ARRAY_SIZE(entity_xu42_controls), .controls = entity_xu42_controls,
	  .num_sources = ARRAY_SIZE(entity_xu42_sources), .sources = entity_xu42_sources },
	/* [4] E007: SU 43 - render selector (headset path), driven by GE 35 jack detection */
	{ .id = 0x7, .label = "SU 43", .type = SDCA_ENTITY_TYPE_SU,
	  .group = &wcd9378_sdca_entities[QSJ_GE35],
	  .num_controls = ARRAY_SIZE(entity_su43_controls), .controls = entity_su43_controls,
	  .num_sources = ARRAY_SIZE(entity_su43_sources), .sources = entity_su43_sources },
	/* [5] E008: SU 45 - render selector (headphone path), driven by GE 35 jack detection */
	{ .id = 0x8, .label = "SU 45", .type = SDCA_ENTITY_TYPE_SU,
	  .group = &wcd9378_sdca_entities[QSJ_GE35],
	  .num_controls = ARRAY_SIZE(entity_su45_controls), .controls = entity_su45_controls,
	  .num_sources = ARRAY_SIZE(entity_su45_sources), .sources = entity_su45_sources },
	/* [6] E009: PDE 47 - render power domain */
	{ .id = 0x9, .label = "PDE 47", .type = SDCA_ENTITY_TYPE_PDE,
	  .pde = { .num_managed = ARRAY_SIZE(entity_pde47_managed), .managed = entity_pde47_managed,
		   .num_max_delay = ARRAY_SIZE(pde47_delays), .max_delay = pde47_delays },
	  .num_controls = ARRAY_SIZE(entity_pde47_controls), .controls = entity_pde47_controls },
	/* [7] E00A: OT 43 - Headphone on jack */
	{ .id = 0xA, .label = "OT 43", .type = SDCA_ENTITY_TYPE_OT,
	  .iot = { .type = 0x06C0 },
	  .num_sources = ARRAY_SIZE(entity_ot43_sources), .sources = entity_ot43_sources },
	/* [8] E00B: OT 45 - Headset output on jack */
	{ .id = 0xB, .label = "OT 45", .type = SDCA_ENTITY_TYPE_OT,
	  .iot = { .type = 0x06D0 },
	  .num_sources = ARRAY_SIZE(entity_ot45_sources), .sources = entity_ot45_sources },
	/* [9] E00C: GE 35 - jack detection group entity */
	{ .id = 0xC, .label = "GE 35", .type = SDCA_ENTITY_TYPE_GE,
	  .ge = { .num_modes = ARRAY_SIZE(ge35_modes), .modes = ge35_modes },
	  .num_controls = ARRAY_SIZE(entity_ge35_controls), .controls = entity_ge35_controls },
	/* [10] E00D: IT 131 - optimization stream input */
	{ .id = 0xD, .label = "IT 131", .type = SDCA_ENTITY_TYPE_IT,
	  .iot = { .type = 0x0190, .is_dataport = true, .clock = &wcd9378_sdca_entities[QSJ_CS131] },
	  .num_controls = ARRAY_SIZE(entity_it131_controls), .controls = entity_it131_controls },
	/* [11] E00E: CS 131 */
	{ .id = 0xE, .label = "CS 131", .type = SDCA_ENTITY_TYPE_CS,
	  .cs = { .type = 0x0 },
	  .num_controls = ARRAY_SIZE(entity_cs131_controls), .controls = entity_cs131_controls },
	/* [12] E00F: IT 33 - headset mic input */
	{ .id = 0xF, .label = "IT 33", .type = SDCA_ENTITY_TYPE_IT,
	  .iot = { .type = 0x06D0 },
	  .num_controls = ARRAY_SIZE(entity_it33_controls), .controls = entity_it33_controls },
	/* [13] E010: PDE 34 - mic power domain */
	{ .id = 0x10, .label = "PDE 34", .type = SDCA_ENTITY_TYPE_PDE,
	  .pde = { .num_managed = ARRAY_SIZE(entity_pde34_managed), .managed = entity_pde34_managed,
		   .num_max_delay = ARRAY_SIZE(pde34_delays), .max_delay = pde34_delays },
	  .num_controls = ARRAY_SIZE(entity_pde34_controls), .controls = entity_pde34_controls },
	/* [14] E011: FU 33 - mic feature unit */
	{ .id = 0x11, .label = "FU 33", .type = SDCA_ENTITY_TYPE_FU,
	  .num_sources = ARRAY_SIZE(entity_fu33_sources), .sources = entity_fu33_sources },
	/* [15] E012: SU 35 - mic selector */
	{ .id = 0x12, .label = "SU 35", .type = SDCA_ENTITY_TYPE_SU,
	  .num_controls = ARRAY_SIZE(entity_su35_controls), .controls = entity_su35_controls,
	  .num_sources = ARRAY_SIZE(entity_su35_sources), .sources = entity_su35_sources },
	/* [16] E013: XU 36 - mic extension unit */
	{ .id = 0x13, .label = "XU 36", .type = SDCA_ENTITY_TYPE_XU,
	  .num_controls = ARRAY_SIZE(entity_xu36_controls), .controls = entity_xu36_controls,
	  .num_sources = ARRAY_SIZE(entity_xu36_sources), .sources = entity_xu36_sources },
	/* [17] E015: CS 36 */
	{ .id = 0x15, .label = "CS 36", .type = SDCA_ENTITY_TYPE_CS,
	  .cs = { .type = 0x0 },
	  .num_controls = ARRAY_SIZE(entity_cs36_controls), .controls = entity_cs36_controls },
	/* [18] E016: OT 36 - PDM mic capture output */
	{ .id = 0x16, .label = "OT 36", .type = SDCA_ENTITY_TYPE_OT,
	  .iot = { .type = 0x0191, .is_dataport = true, .clock = &wcd9378_sdca_entities[QSJ_CS36] },
	  .num_controls = ARRAY_SIZE(entity_ot36_controls), .controls = entity_ot36_controls,
	  .num_sources = ARRAY_SIZE(entity_ot36_sources), .sources = entity_ot36_sources },
	/* [19] E017: MFPU 236 - optimization TX processing */
	{ .id = 0x17, .label = "MFPU 236", .type = SDCA_ENTITY_TYPE_MFPU,
	  .num_controls = ARRAY_SIZE(entity_mfpu236_controls), .controls = entity_mfpu236_controls },
	/* [20] E018: CS 236 */
	{ .id = 0x18, .label = "CS 236", .type = SDCA_ENTITY_TYPE_CS,
	  .cs = { .type = 0x0 },
	  .num_controls = ARRAY_SIZE(entity_cs236_controls), .controls = entity_cs236_controls },
	/* [21] E019: OT 236 - optimization stream capture output */
	{ .id = 0x19, .label = "OT 236", .type = SDCA_ENTITY_TYPE_OT,
	  .iot = { .type = 0x0190, .is_dataport = true, .clock = &wcd9378_sdca_entities[QSJ_CS131] },
	  .num_controls = ARRAY_SIZE(entity_ot236_controls), .controls = entity_ot236_controls,
	  .num_sources = ARRAY_SIZE(entity_ot236_sources), .sources = entity_ot236_sources },
	/* E006: FU 6 (FU42) - vendor Feature Unit; HPH mute + Q7.8 volume. */
	{ .id = 0x6, .label = "FU 6", .type = SDCA_ENTITY_TYPE_FU,
	  .num_controls = ARRAY_SIZE(entity_fu6_controls), .controls = entity_fu6_controls },
	/* Entity 0 (Function) */
	{ .id = 0x0, .label = "entity0",
	  .num_controls = ARRAY_SIZE(entity0_controls), .controls = entity0_controls },
};

/* Clusters */
static struct sdca_channel cl1_channels[] = {	/* CL01 - render (HPH) stereo */
	{ .id = 0x1, .purpose = 0x1, .relationship = 0x2 },	/* Left */
	{ .id = 0x2, .purpose = 0x1, .relationship = 0x3 },	/* Right */
};

static struct sdca_channel cl2_channels[] = {	/* CL02 - mic capture mono */
	{ .id = 0xFF, .purpose = 0x1, .relationship = 0x1 },
};

static struct sdca_channel cl3_channels[] = {	/* CL03 - optimization RX */
	{ .id = 0xFF, .purpose = 0x1, .relationship = 0x1 },	/* Mono */
	{ .id = 0x1,  .purpose = 0x1, .relationship = 0x2 },	/* Left */
	{ .id = 0x2,  .purpose = 0x1, .relationship = 0x3 },	/* Right */
};

static struct sdca_channel cl5_channels[] = {	/* CL05 - optimization TX mono */
	{ .id = 0xFF, .purpose = 0x1, .relationship = 0x1 },
};

static struct sdca_cluster wcd9378_sdca_clusters[] = {
	{ .id = 0x1, .num_channels = ARRAY_SIZE(cl1_channels), .channels = cl1_channels },
	{ .id = 0x2, .num_channels = ARRAY_SIZE(cl2_channels), .channels = cl2_channels },
	{ .id = 0x3, .num_channels = ARRAY_SIZE(cl3_channels), .channels = cl3_channels },
	{ .id = 0x5, .num_channels = ARRAY_SIZE(cl5_channels), .channels = cl5_channels },
};

/* Init table transcribed from ASL. */
static struct sdca_init_write wcd9378_sdca_init_table[] = {
	{ .addr = 0x401804F0, .val = 0x00 }, /* DIGITAL_PLATFORM_CTL */
	{ .addr = 0x4018046E, .val = 0x10 }, /* DIGITAL_INTR_MODE */
	{ .addr = 0x0000004D, .val = 0x01 }, /* SWRS_SCP_BUSCLOCK_BASE */
	{ .addr = 0x00000062, .val = 0x02 }, /* SWRS_SCP_BUSCLOCK_SCALE_BANK */
	{ .addr = 0x4018016A, .val = 0x80 }, /* CP_DTOP_CTRL_14 */
	{ .addr = 0x40180165, .val = 0x6b }, /* CP_DTOP_CTRL_9 */
	{ .addr = 0x40180103, .val = 0x1E }, /* SLEEP_CTL BG_CTL (0.9V) */
	{ .addr = 0x40180103, .val = 0x9E }, /* SLEEP_CTL BG_EN */
	{ .addr = 0x40180103, .val = 0xDE }, /* SLEEP_CTL LDOL_BG_SEL */
	{ .addr = 0x40180029, .val = 0xB5 }, /* BIAS_VBG_FINE_ADJ */
	{ .addr = 0x40180001, .val = 0x80 }, /* ANA_BIAS ANALOG_BIAS_EN */
	{ .addr = 0x40180001, .val = 0xC0 }, /* ANA_BIAS PRECHRG_EN(1) */
	{ .addr = 0x40180001, .val = 0x80 }, /* ANA_BIAS PRECHRG_EN(0) */
	{ .addr = 0x4018007B, .val = 0xA2 }, /* TX_COM_TXFE_DIV_CTL SEQ_BYPASS */
	{ .addr = 0x40180465, .val = 0x17 }, /* PDM_WD_CTL0 TIME_OUT_SEL_PCM */
	{ .addr = 0x40180466, .val = 0x17 }, /* PDM_WD_CTL1 TIME_OUT_SEL_PCM */
	/*
	 * Mic-bias bring-up, in the order wcd937x_MBIAS_MGR_ENABLE() writes it:
	 * the TX SC bias clock first, then the per-channel noise filter, then
	 * the LDO driver bias.
	 *
	 * ANA_TXSCBIAS_CLK_EN is bit 0 and comes up cleared (reset 0xE); the
	 * TX0/1/2_ADC_CLK_EN bits above it are already set at reset.
	 * NOISE_FILT_RES_VAL (bits 7:5) is left at its noisy POR default, so
	 * every channel is given R7_259M here, leaving the EN_VREFGEN, EN_LDO
	 * and LDO_BLEEDER_CTRL bits at their reset value 0x1A.
	 *
	 * IBIAS_LDO_DRIVER has to be set for the bias to drive its pin at all.
	 * MICB2 feeds the headset mic, and MBHC classification measures that
	 * line, so without this the mic reads 0mV and every accessory looks
	 * like a headphone.
	 */
	{ .addr = 0x40180417, .val = 0x0F }, /* CDC_ANA_TX_CLK_CTL ANA_TXSCBIAS_CLK_EN */
	{ .addr = 0x4018006B, .val = 0xFA }, /* MICB1_TEST_CTL_1 NOISE_FILT_RES_VAL */
	{ .addr = 0x4018006C, .val = 0x01 }, /* MICB1_TEST_CTL_2 IBIAS_LDO_DRIVER */
	{ .addr = 0x4018006E, .val = 0xFA }, /* MICB2_TEST_CTL_1 NOISE_FILT_RES_VAL */
	{ .addr = 0x4018006F, .val = 0x01 }, /* MICB2_TEST_CTL_2 IBIAS_LDO_DRIVER */
	{ .addr = 0x40180071, .val = 0xFA }, /* MICB3_TEST_CTL_1 NOISE_FILT_RES_VAL */
	{ .addr = 0x40180072, .val = 0x81 }, /* MICB3_TEST_CTL_2 IBIAS_LDO_DRIVER */
	{ .addr = 0x401800CE, .val = 0x38 }, /* HPH_OCP_CTL OCP_FSM_EN */
	{ .addr = 0x401800CE, .val = 0x3A }, /* HPH_OCP_CTL SCD_OP_EN */
	{ .addr = 0x401800D4, .val = 0xE1 }, /* HPH_L_TEST OCP_DET_EN */
	{ .addr = 0x401800D7, .val = 0xE1 }, /* HPH_R_TEST OCP_DET_EN */
	{ .addr = 0x4018044E, .val = 0x04 }, /* CDC_HPH_GAIN_CTL HPHL_RX_EN */
	{ .addr = 0x4018044E, .val = 0x0C }, /* CDC_HPH_GAIN_CTL HPHR_RX_EN */
	{ .addr = 0x4018000F, .val = 0x0C }, /* ANA_TX_CH2 GAIN (18.0dB) */
	{ .addr = 0x40180133, .val = 0x84 }, /* HPH_NEW_INT_RDAC_HD2_CTL_L */
	{ .addr = 0x40180136, .val = 0x84 }, /* HPH_NEW_INT_RDAC_HD2_CTL_R */
	{ .addr = 0x401800D9, .val = 0x19 }, /* HPH_RDAC_CLK_CTL1 OPAMP_CHOP_CLK_EN */
	{ .addr = 0x40180132, .val = 0x50 }, /* HPH_NEW_INT_RDAC_GAIN_CTL RDAC_GAINCTL(0.55) */
	{ .addr = 0x40180510, .val = 0x05 }, /* SEQR_CTRL HPH_UP_T0 */
	{ .addr = 0x40180519, .val = 0x05 }, /* SEQR_CTRL HPH_UP_T9 */
	{ .addr = 0x4018051B, .val = 0x06 }, /* SEQR_CTRL HPH_DN_T0 */
	{ .addr = 0x40180414, .val = 0x02 }, /* CDC_COMP_CTL_0 HPHL_COMP_EN */
	{ .addr = 0x40180414, .val = 0x03 }, /* CDC_COMP_CTL_0 HPHR_COMP_EN */
	{ .addr = 0x401804F2, .val = 0x80 }, /* DRE_DLY_VAL SWR_HPHL(0) */
	{ .addr = 0x401804F2, .val = 0x00 }, /* DRE_DLY_VAL SWR_HPHR(0) */
	{ .addr = 0x40180501, .val = 0x01 }, /* SEQR_CTRL SYS_USAGE_CTRL */
	{ .addr = 0x40180601, .val = 0x03 }, /* MBHC_CTRL DEVICE_DET */
	{ .addr = 0x40180414, .val = 0x00 }, /* CDC_COMP_CTL_0 */
	{ .addr = 0x401804F2, .val = 0x88 }, /* DRE_DLY_VAL */
	{ .addr = 0x40180517, .val = 0x07 }, /* SEQR_CTRL HPH_UP_T7 */
	{ .addr = 0x4018051C, .val = 0x07 }, /* SEQR_CTRL HPH_DN_T1 */
	{ .addr = 0x401800CE, .val = 0x28 }, /* HPH_OCP_CTL */
	{ .addr = 0x401800D4, .val = 0xe0 }, /* HPH_L_TEST */
	{ .addr = 0x401800D7, .val = 0xe0 }, /* HPH_R_TEST */
	{ .addr = 0x40180510, .val = 0x07 }, /* SEQR_CTRL HPH_UP_T0 */
	{ .addr = 0x40C80008, .val = 0x01 }, /* SMP_JACK_CTRL FUNC_ACT (RESET_FUNCTION_NOW) */
	{ .addr = 0x40C00008, .val = 0x02 }, /* SMP_JACK_CTRL CMT_GRP_MASK */
	{ .addr = 0x40C80000, .val = 0xFF }, /* SMP_JACK_CTRL FUNC_STAT */
	{ .addr = 0x00000041, .val = 0x08 }, /* SCP_INT_STATUS_MASK_1 PORT_0_CASCADE_3 */
	/* Only SDCA_4 unmasked; protection IRQs stay masked. */
	{ .addr = 0x0000005C, .val = 0x10 }, /* INTMASK_1 SDCA_4 (GE_DETECTED_MODE) */
	{ .addr = 0x4018016A, .val = 0x00 }, /* CP_DTOP_CTRL_14 */
	{ .addr = 0x40180165, .val = 0x6b }, /* CP_DTOP_CTRL_9 */
};

/* Function Descriptor */
static struct sdca_function_desc wcd9378_sdca_desc = {
	.adr  = 0x3,
	.type = SDCA_FUNCTION_TYPE_SIMPLE_JACK,
	.name = SDCA_FUNCTION_TYPE_SIMPLE_NAME,
};

/* Main Function Data */
static struct sdca_function_data wcd9378_sdca_data = {
	.desc             = &wcd9378_sdca_desc,
	.num_entities     = ARRAY_SIZE(wcd9378_sdca_entities),
	.entities         = wcd9378_sdca_entities,
	.num_clusters     = ARRAY_SIZE(wcd9378_sdca_clusters),
	.clusters         = wcd9378_sdca_clusters,
	.num_init_table   = ARRAY_SIZE(wcd9378_sdca_init_table),
	.init_table       = wcd9378_sdca_init_table,
	.reset_max_delay  = 100000, /* 100ms — WCD9378 power-on reset completes well within this */
};

/* Vendor SCP register: host clock divide-by-2 (bank 1 shadow). */
#define WCD9378_SCP_HOST_CLK_DIV2_CTL_B1	0xF0

#define WCD9378_ANA_MBHC_MECH_ADDR		0x40180014
#define WCD9378_ANA_MBHC_MECH_ENABLE		0xB5
#define WCD9378_ANA_MBHC_MECH_DET_TYP		0x20
#define WCD9378_ANA_MBHC_ELECT_ADDR		0x40180015
#define WCD9378_ANA_MBHC_ELECT_BIAS_EN		0x01
#define WCD9378_ANA_MBHC_ELECT_SCHMT_ISRC	0x06
#define WCD9378_ANA_MBHC_ELECT_FSM_EN		0x80
#define WCD9378_MBHC_NEW_INT_MECH_DET_CURRENT_ADDR	0x401801b1
#define WCD9378_MBHC_HSDET_PULLUP_CTL_2P0_UA		0x08
#define WCD9378_MBHC_NEW_PLUG_DETECT_CTL_ADDR		0x40180122
#define WCD9378_MBHC_INSREM_DBNC_96MS			0x06
/*
 * MBHC_NEW_CTL_1, reset 0x0e.  Do not copy the ADC and detection fields from
 * wcd937x.c's wcd_mbhc_fields[], which places ADC enable at 0x08 and
 * detection-done at 0x04 and sizes BTN_DBNC_CTL as 0x03 -- that is a
 * different die.  Here BTN_DBNC_CTL is bits 3:0, so the wcd937x ADC-enable
 * mask lands inside the button debounce field and leaves the ADC alone.  The
 * debounce encoding differs with the width: 0x0 is 0.9ms rising to 0xf at
 * 36.3ms, reset 0xe being 18.5ms, so the reference driver's "set 2 for 16ms"
 * does not carry over either.  The rest of the MBHC masks do agree between
 * the two parts.
 */
#define WCD9378_MBHC_NEW_CTL_1_ADDR		0x40180120
#define WCD9378_MBHC_NEW_CTL_1_RCO_EN		0x80
#define WCD9378_MBHC_NEW_CTL_1_ADC_EN		0x40
/*
 * Gates the button press/release interrupts.  Software latches it once it has
 * concluded the plug carries a mic; this driver does not do buttons and so
 * leaves it at its reset 0.
 */
#define WCD9378_MBHC_NEW_CTL_1_DETECTION_DONE	0x20
/* 0 = single measurement, 1 = continuous; this driver uses continuous */
#define WCD9378_MBHC_NEW_CTL_1_ADC_MODE		0x10
/* Everything above BTN_DBNC_CTL, which is left as the hardware set it */
#define WCD9378_MBHC_NEW_CTL_1_DET_MASK		0xf0
#define WCD9378_MBHC_NEW_CTL_2_ADDR		0x40180121
#define WCD9378_MBHC_NEW_CTL_2_MUX_CTL		0x70
/* MUX_CTL field values in place, see enum wcd_mbhc_adc_mux_ctl */
#define WCD9378_MBHC_NEW_CTL_2_MUX_CTL_IN2P	0x10
#define WCD9378_MBHC_NEW_CTL_2_MUX_CTL_HPH_L	0x40
#define WCD9378_MBHC_NEW_CTL_2_MUX_CTL_HPH_R	0x50
/*
 * The rest of FSM_STATUS, for reading the value this driver traces: 0x20
 * mechanical comparator (plug present, not moisture), 0x10 fast press, 0x08
 * fast removal, 0x04 removal, 0x02 electrical-removal comparator (set while
 * the mic line is pulled up), 0x01 button.
 */
#define WCD9378_MBHC_NEW_FSM_STATUS_ADDR	0x40180125
#define WCD9378_MBHC_NEW_FSM_STATUS_ADC_COMPLETE	0x40
#define WCD9378_MBHC_NEW_FSM_STATUS_ADC_TIMEOUT		0x80
#define WCD9378_MBHC_NEW_ADC_RESULT_ADDR	0x40180126
#define WCD9378_ANA_HPH_ADDR			0x40180009
#define WCD9378_ANA_HPH_PA_EN			0xC0
#define WCD9378_ANA_MICB2_ADDR			0x40180023
#define WCD9378_ANA_MICB2_CTRL			0xC0
#define WCD9378_ANA_MICB2_CTRL_ENABLE		0x40
/*
 * CTRL is bits 7:6 with the same field values as wcd937x.h's WCD937X_MICB_*:
 * 0 disable, 1 enable, 2 pull-up, 3 pull-down.  Pull-up is only used as a
 * stopover while VOUT_CTL changes, see wcd9378_mbhc_micb2_set_vout().
 */
#define WCD9378_ANA_MICB2_CTRL_PULL_UP		0x80
#define WCD9378_ANA_MICB2_VOUT			0x3F
#define WCD9378_DIGITAL_CDC_ANA_CLK_CTL_ADDR	0x40180408
#define WCD9378_DIGITAL_CDC_ANA_CLK_CTL_ANA_TX	0x10
#define WCD9378_DIGITAL_CDC_DIG_CLK_CTL_ADDR	0x40180409
#define WCD9378_DIGITAL_CDC_DIG_CLK_CTL_TX_CLK	0xF0

/*
 * Mic-line thresholds for ADC accessory classification, as
 * wcd-mbhc-v2.c's compiled-in defaults.  No MBHC device-tree
 * properties are described for this part, so there is nothing to
 * override them with.
 */
#define WCD9378_MBHC_ADC_MICBIAS_MV		1800
#define WCD9378_MBHC_ADC_HPH_THRESHOLD_MV	75
#define WCD9378_MBHC_ADC_HS_THRESHOLD_MV	1700

/*
 * wcd-mbhc-v2.h's WCD_MBHC_THR_HS_MICB_MV.  Some headsets, Apple's among them,
 * put a chip in series with the microphone that leaves it disconnected until
 * the bias reaches about 2.7V; at 1.8V their mic line floats and reads above
 * the headset band.  Re-measuring at this bias is what tells them apart from a
 * genuine extension cable, see wcd9378_mbhc_check_spl_headset().
 *
 * VOUT_CTL = (micbias_mv - 1000) / 50, and wcd-common.c's
 * wcd_get_micb_vout_ctl_val() caps the family at 2.85V, so 2.7V is in range.
 */
#define WCD9378_MBHC_SPL_HS_MICBIAS_MV		2700
#define WCD9378_ANA_MICB2_VOUT_2P7V		0x22

/*
 * wcd_check_cross_conn()'s HPHL_CROSS_CONN_THRESHOLD: a headphone pin
 * reading above this is carrying the mic bias, so ground and mic are
 * swapped in the plug.  Diagnostic only here, see
 * wcd9378_mbhc_trace_cross_conn().
 */
#define WCD9378_MBHC_ADC_CROSS_CONN_MV		100

/*
 * 3ms waits for the continuous-mode ADC to self-calibrate.  The reference
 * driver takes three and moves on, but it can afford to: wcd_correct_swch_plug()
 * re-measures for up to ~1.8s and only accepts a plug type that repeats.  This
 * driver classifies from one cold measurement, so take a couple more.
 *
 * Measured on Glymur CRD with the 50ms mic-bias settle below: a headset mic
 * line reads 1450mV on the first sample and is flat at 1475mV from the fourth
 * onwards, so five is the plateau plus margin.  A headphone reads 0mV from the
 * first sample, having nothing to charge.
 */
#define WCD9378_MBHC_ADC_SETTLE_COUNT		5

/* GE 35 Selected_Mode values, see range_ge35_mode_data[]. */
#define WCD9378_GE35_MODE_UNPLUGGED		0x0
#define WCD9378_GE35_MODE_HEADSET		0x3
#define WCD9378_GE35_MODE_HEADPHONE		0x4

/* Slave ports 1..8, mapped to master ports via qcom,port-mapping. */
#define WCD9378_SDCA_MAX_PORTS			8

static const char * const wcd9378_sdca_supplies[] = {
	"vdd-buck", "vdd-rxtx", "vdd-io", "vdd-mic-bias",
};

/*
 * The SDCA specification defines DP0_INT.SDCA_CASCADE as the read-only
 * output of an OR over the post-mask SDCA interrupt status bits, with no
 * clear path of its own, so the bus driver deliberately never writes it.
 *
 * This part deviates: the bit latches.  Measured on a jack insert, with
 * both reads issued back to back on the same bus with nothing in between,
 * all four SCP_SDCA_INT1-4 registers read 0x00 while DP0_INT still read
 * 0x08.  Left asserted it holds SCP_INT1.PORT_0 set, the peripheral never
 * leaves ALERT, and sdw_handle_slave_alerts() spins to MAX_RETRY on every
 * jack event.
 *
 * Clear it here rather than in the bus driver: this callback runs after
 * handle_nested_irq() has serviced the cascaded SDCA sources and before
 * the bus re-reads DP0_INT for its end-of-iteration check, so it is the
 * one place the write is both safe and effective.
 */
int wcd9378_sdca_interrupt_callback(struct sdw_slave *slave,
				    struct sdw_slave_intr_status *status)
{
	int ret;

	if (!status->sdca_cascade)
		return 0;

	ret = sdw_write_no_pm(slave, SDW_DP0_INT, SDW_DP0_SDCA_CASCADE);
	if (ret < 0)
		dev_err(&slave->dev, "SDCA_CASCADE clear failed:%d\n", ret);

	return 0;
}

int wcd9378_sdca_read_prop(struct sdw_slave *slave)
{
	struct sdw_slave_prop *prop = &slave->prop;
	struct device *dev = &slave->dev;
	struct sdw_dpn_prop *sink, *src;
	int ret;

	ret = sdw_slave_read_prop(slave);
	if (ret)
		return ret;

	prop->use_domain_irq = true;
	prop->scp_int1_mask = SDW_SCP_INT1_BUS_CLASH | SDW_SCP_INT1_PARITY |
			      SDW_SCP_INT1_IMPL_DEF;

	/* Compute-mode fixed SoundWire slave properties (not described in DT) */
	prop->simple_clk_stop_capable = true;
	prop->paging_support = true;
	prop->clock_reg_supported = true;
	prop->lane_control_support = true;

	/* Source ports: DP2 (headset mic), DP5 (optimisation TX). */
	prop->source_ports = BIT(2) | BIT(5);
	/* Sink ports: DP6 (HPH audio), DP7 (HPH envelope), DP8 (optimisation RX). */
	prop->sink_ports = BIT(6) | BIT(7) | BIT(8);

	src = devm_kcalloc(dev, 2, sizeof(*src), GFP_KERNEL);
	if (!src)
		return -ENOMEM;

	src[0].num = 2;
	src[0].type = SDW_DPN_SIMPLE;
	src[0].simple_ch_prep_sm = true;
	src[0].ch_prep_timeout = 10;
	src[0].max_ch = 1;
	src[0].min_ch = 1;

	src[1].num = 5;
	src[1].type = SDW_DPN_SIMPLE;
	src[1].simple_ch_prep_sm = true;
	src[1].ch_prep_timeout = 10;
	src[1].max_ch = 1;
	src[1].min_ch = 1;

	prop->src_dpn_prop = src;

	sink = devm_kcalloc(dev, 3, sizeof(*sink), GFP_KERNEL);
	if (!sink)
		return -ENOMEM;

	sink[0].num = 6;
	sink[0].type = SDW_DPN_SIMPLE;
	sink[0].simple_ch_prep_sm = true;
	sink[0].ch_prep_timeout = 10;
	sink[0].max_ch = 2;
	sink[0].min_ch = 1;

	sink[1].num = 7;
	sink[1].type = SDW_DPN_FULL;
	sink[1].simple_ch_prep_sm = true;
	sink[1].ch_prep_timeout = 10;
	sink[1].max_ch = 1;
	sink[1].min_ch = 1;

	sink[2].num = 8;
	sink[2].type = SDW_DPN_REDUCED;
	sink[2].simple_ch_prep_sm = true;
	sink[2].ch_prep_timeout = 10;
	sink[2].max_ch = 2;
	sink[2].min_ch = 1;

	prop->sink_dpn_prop = sink;

	ret = device_property_read_u32_array(dev, "qcom,port-mapping",
					     &slave->m_port_map[1],
					     WCD9378_SDCA_MAX_PORTS);
	if (ret)
		return dev_err_probe(dev, ret, "qcom,port-mapping missing\n");

	return 0;
}

static int wcd9378_sdca_populate_function(struct sdw_slave *slave,
					  struct sdca_function_data *function)
{
	/* @function->desc is already set by the framework; fill payload only. */
	if (function->desc->type != wcd9378_sdca_desc.type)
		return -EINVAL;

	function->num_entities    = wcd9378_sdca_data.num_entities;
	function->entities        = wcd9378_sdca_data.entities;
	function->num_clusters    = wcd9378_sdca_data.num_clusters;
	function->clusters        = wcd9378_sdca_data.clusters;
	function->num_init_table  = wcd9378_sdca_data.num_init_table;
	function->init_table      = wcd9378_sdca_data.init_table;
	function->reset_max_delay = wcd9378_sdca_data.reset_max_delay;

	/* Elevate is_volatile / has_reset to match the DisCo/ACPI path. */
	sdca_apply_default_control_classifiers(function);

	return 0;
}

static int wcd9378_sdca_set_jack(struct sdw_slave *slave, struct regmap *dev_regmap)
{
	struct device *dev = &slave->dev;
	unsigned int val;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "failed to resume for jack detect enable: %d\n", ret);
		return ret;
	}

	ret = regmap_write(dev_regmap, WCD9378_MBHC_NEW_INT_MECH_DET_CURRENT_ADDR,
			   WCD9378_MBHC_HSDET_PULLUP_CTL_2P0_UA);
	if (ret) {
		dev_err(dev, "MBHC_NEW_INT_MECH_DET_CURRENT: %d\n", ret);
		goto out;
	}

	ret = regmap_write(dev_regmap, WCD9378_ANA_MBHC_MECH_ADDR,
			   WCD9378_ANA_MBHC_MECH_ENABLE);
	if (ret) {
		dev_err(dev, "ANA_MBHC_MECH: %d\n", ret);
		goto out;
	}

	ret = regmap_write(dev_regmap, WCD9378_MBHC_NEW_PLUG_DETECT_CTL_ADDR,
			   WCD9378_MBHC_INSREM_DBNC_96MS);
	if (ret) {
		dev_err(dev, "MBHC_NEW_PLUG_DETECT_CTL: %d\n", ret);
		goto out;
	}

	ret = regmap_write(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR,
			   WCD9378_ANA_MBHC_ELECT_BIAS_EN);
	if (ret) {
		dev_err(dev, "ANA_MBHC_ELECT: %d\n", ret);
		goto out;
	}

	/*
	 * RCO on, ADC and detection idle.  Masked rather than written whole so
	 * BTN_DBNC_CTL in the low nibble keeps its reset value.
	 */
	ret = regmap_update_bits(dev_regmap, WCD9378_MBHC_NEW_CTL_1_ADDR,
				 WCD9378_MBHC_NEW_CTL_1_DET_MASK,
				 WCD9378_MBHC_NEW_CTL_1_RCO_EN);
	if (ret) {
		dev_err(dev, "MBHC_NEW_CTL_1: %d\n", ret);
		goto out;
	}

	ret = regmap_read(dev_regmap, WCD9378_ANA_MBHC_MECH_ADDR, &val);
	if (ret) {
		dev_err(dev, "ANA_MBHC_MECH readback: %d\n", ret);
		goto out;
	}

	dev_err(dev, "mbhc: jack detect armed for %s, ANA_MBHC_MECH %#x\n",
		val & WCD9378_ANA_MBHC_MECH_DET_TYP ? "insertion" : "removal", val);

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

/*
 * Bring up MBHC mic bias, following wcd937x_micbias_control()'s MICB_ENABLE
 * case: the TX clocks have to be running before MICB2 itself, or the ADC
 * samples an unsettled line.  That reference also sets MICBn_TEST_CTL_2
 * IBIAS_LDO_DRIVER here; wcd9378_sdca_init_table[] already establishes those
 * at function boot, so there is nothing left to do for them.
 *
 * This reaches around the SDCA mic-bias model (IT 33 MICB and its PDE), which
 * is safe only because classification runs on jack insertion, when no capture
 * stream can be open.  ANA_MICB2 is restored by the caller; the TX clock
 * enables are left on, matching the reference driver.
 */
static int wcd9378_mbhc_micbias_enable(struct device *dev, struct regmap *dev_regmap)
{
	unsigned int val;
	int ret;

	ret = regmap_update_bits(dev_regmap, WCD9378_DIGITAL_CDC_DIG_CLK_CTL_ADDR,
				 WCD9378_DIGITAL_CDC_DIG_CLK_CTL_TX_CLK,
				 WCD9378_DIGITAL_CDC_DIG_CLK_CTL_TX_CLK);
	if (ret)
		return ret;

	ret = regmap_update_bits(dev_regmap, WCD9378_DIGITAL_CDC_ANA_CLK_CTL_ADDR,
				 WCD9378_DIGITAL_CDC_ANA_CLK_CTL_ANA_TX,
				 WCD9378_DIGITAL_CDC_ANA_CLK_CTL_ANA_TX);
	if (ret)
		return ret;

	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MICB2_ADDR,
				 WCD9378_ANA_MICB2_CTRL,
				 WCD9378_ANA_MICB2_CTRL_ENABLE);
	if (ret)
		return ret;

	/*
	 * Let the bias reach the mic line before measuring.  The reference
	 * driver gets this for free from the worker hop plus its cross-connect
	 * checks and its repeated measurements; with only one measurement to
	 * work from, match the flat 50ms the Windows driver sleeps here.
	 *
	 * This wait, not the ADC settle loop, is what decides whether the
	 * reading has converged: at 10ms a headset mic line was still climbing
	 * through 1412mV towards the 1700mV ceiling, and at 50ms it arrives at
	 * 1450mV on the first sample and plateaus at 1475mV.
	 */
	usleep_range(50000, 51000);

	ret = regmap_read(dev_regmap, WCD9378_ANA_MICB2_ADDR, &val);
	if (ret)
		return ret;

	dev_dbg(dev, "mbhc: mic bias up, ANA_MICB2 %#x\n", val);

	return 0;
}

/* ADC_RESULT is 12.5mV per step, referred to the mic bias it was taken at */
static int wcd9378_mbhc_adc_to_mv(unsigned int adc, int micbias_mv)
{
	return ((int)adc * 125 * micbias_mv) /
	       (WCD9378_MBHC_ADC_MICBIAS_MV * 10);
}

/*
 * Measure one MBHC mux input with the ADC in continuous mode, the same way
 * wcd_measure_adc_continuous() does for the first measurement of a fresh
 * insertion.  Continuous mode needs no completion poll, just three 3ms waits
 * for the ADC to self-calibrate; single-shot mode is only worth its FSM
 * bookkeeping for the repeated measurements of the button/removal loop, which
 * this driver does not run.  FSM_STATUS is read anyway and traced, since
 * ADC_TIMEOUT there explains an implausible result.
 *
 * The whole detection state (ANA_MBHC_ELECT, MBHC_NEW_CTL_1, MBHC_NEW_CTL_2)
 * is saved and put back, so what wcd9378_sdca_set_jack() established for
 * mechanical detection survives the measurement.
 *
 * Return: measured millivolts, or a negative error code.
 */
static int wcd9378_mbhc_measure_adc(struct device *dev, struct regmap *dev_regmap,
				    unsigned int mux_ctl, const char *mux_name,
				    int micbias_mv)
{
	unsigned int elect, ctl_1, ctl_2, status, adc = 0;
	int output_mv = 0;
	int prev_mv = 0;
	int err_restore;
	int i, ret;

	ret = regmap_read(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR, &elect);
	if (ret)
		goto err;

	ret = regmap_read(dev_regmap, WCD9378_MBHC_NEW_CTL_1_ADDR, &ctl_1);
	if (ret)
		goto err;

	ret = regmap_read(dev_regmap, WCD9378_MBHC_NEW_CTL_2_ADDR, &ctl_2);
	if (ret)
		goto err;

	/* The state the restore path below has to put back, for debugging it */
	dev_dbg(dev, "mbhc: %s measure start: elect %#x ctl_1 %#x ctl_2 %#x\n",
		mux_name, elect, ctl_1, ctl_2);

	/* Legacy electrical detection would fight the ADC for the mic line */
	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR,
				 WCD9378_ANA_MBHC_ELECT_SCHMT_ISRC, 0);
	if (ret)
		goto restore;

	/* Continuous mode, but hold the ADC and the FSM off while the mux moves */
	ret = regmap_update_bits(dev_regmap, WCD9378_MBHC_NEW_CTL_1_ADDR,
				 WCD9378_MBHC_NEW_CTL_1_ADC_MODE |
				 WCD9378_MBHC_NEW_CTL_1_ADC_EN,
				 WCD9378_MBHC_NEW_CTL_1_ADC_MODE);
	if (ret)
		goto restore;

	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR,
				 WCD9378_ANA_MBHC_ELECT_FSM_EN, 0);
	if (ret)
		goto restore;

	ret = regmap_update_bits(dev_regmap, WCD9378_MBHC_NEW_CTL_2_ADDR,
				 WCD9378_MBHC_NEW_CTL_2_MUX_CTL, mux_ctl);
	if (ret)
		goto restore;

	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR,
				 WCD9378_ANA_MBHC_ELECT_FSM_EN,
				 WCD9378_ANA_MBHC_ELECT_FSM_EN);
	if (ret)
		goto restore;

	ret = regmap_update_bits(dev_regmap, WCD9378_MBHC_NEW_CTL_1_ADDR,
				 WCD9378_MBHC_NEW_CTL_1_ADC_EN,
				 WCD9378_MBHC_NEW_CTL_1_ADC_EN);
	if (ret)
		goto restore;

	for (i = 0; i < WCD9378_MBHC_ADC_SETTLE_COUNT; i++) {
		usleep_range(3000, 3100);

		ret = regmap_read(dev_regmap, WCD9378_MBHC_NEW_ADC_RESULT_ADDR, &adc);
		if (ret)
			goto restore;

		output_mv = wcd9378_mbhc_adc_to_mv(adc, micbias_mv);

		/*
		 * Per-sample trace, the one line here that is not needed to
		 * explain a classification after the fact.  Enable with:
		 *   echo 'format "mbhc: %s settle" +p' > \
		 *	/sys/kernel/debug/dynamic_debug/control
		 */
		if (i)
			dev_dbg(dev, "mbhc: %s settle %d/%d: adc %#x -> %dmV (%+dmV)\n",
				mux_name, i + 1, WCD9378_MBHC_ADC_SETTLE_COUNT,
				adc, output_mv, output_mv - prev_mv);
		else
			dev_dbg(dev, "mbhc: %s settle %d/%d: adc %#x -> %dmV\n",
				mux_name, i + 1, WCD9378_MBHC_ADC_SETTLE_COUNT,
				adc, output_mv);

		prev_mv = output_mv;
	}

	ret = regmap_read(dev_regmap, WCD9378_MBHC_NEW_FSM_STATUS_ADDR, &status);
	if (ret)
		goto restore;

	dev_err(dev, "mbhc: %s measured %dmV (adc %#x, fsm_status %#x%s%s)\n",
		mux_name, output_mv, adc, status,
		status & WCD9378_MBHC_NEW_FSM_STATUS_ADC_COMPLETE ? " complete" : "",
		status & WCD9378_MBHC_NEW_FSM_STATUS_ADC_TIMEOUT ? " TIMEOUT" : "");

	ret = output_mv;

restore:
	/*
	 * Park the mux with the FSM quiet, then undo the rest in the order
	 * wcd_check_cross_conn() does: mux, then ADC, then the FSM and legacy
	 * detection state together with the ELECT register.  Restoring
	 * MBHC_NEW_CTL_2 wholesale is what returns MUX_CTL to AUTO, since that
	 * is the field's reset value and nothing else writes it.
	 *
	 * Every write is attempted even if an earlier one failed: leaving the
	 * detection block half configured would cost the next transition.
	 */
	err_restore = regmap_update_bits(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR,
					 WCD9378_ANA_MBHC_ELECT_FSM_EN, 0);
	err_restore |= regmap_write(dev_regmap, WCD9378_MBHC_NEW_CTL_2_ADDR, ctl_2);
	err_restore |= regmap_write(dev_regmap, WCD9378_MBHC_NEW_CTL_1_ADDR, ctl_1);
	err_restore |= regmap_write(dev_regmap, WCD9378_ANA_MBHC_ELECT_ADDR, elect);
	if (err_restore)
		dev_err(dev, "mbhc: %s failed to restore detection state\n", mux_name);

	return ret;

err:
	dev_err(dev, "mbhc: %s failed to read detection state: %d\n", mux_name, ret);

	return ret;
}

/*
 * Trace whether the plug went in cross-connected, i.e. with ground and mic
 * swapped, which puts the mic bias on a headphone pin and makes the IN2P
 * reading meaningless.  wcd_check_cross_conn() calls anything above 100mV on
 * either headphone pin a cross-connection.
 *
 * This is diagnostics only and deliberately changes no decision: correcting a
 * cross-connection needs a board-level ground/mic swap callback that does not
 * exist for this part.  It does say whether a misclassification is the plug's
 * fault or the driver's.  Costs two more ADC measurements, so this is the
 * first thing to drop if the handler ever needs to be shorter.
 */
static void wcd9378_mbhc_trace_cross_conn(struct device *dev, struct regmap *dev_regmap,
					  int micbias_mv)
{
	int hphl_mv, hphr_mv;
	unsigned int hph;
	int ret;

	/* A running headphone PA would drive the pins the ADC wants to read */
	ret = regmap_read(dev_regmap, WCD9378_ANA_HPH_ADDR, &hph);
	if (ret) {
		dev_err(dev, "mbhc: ANA_HPH read: %d\n", ret);
		return;
	}

	if (hph & WCD9378_ANA_HPH_PA_EN) {
		dev_err(dev, "mbhc: hph pa on (ANA_HPH %#x), skipping cross-conn trace\n",
			hph);
		return;
	}

	hphl_mv = wcd9378_mbhc_measure_adc(dev, dev_regmap,
					   WCD9378_MBHC_NEW_CTL_2_MUX_CTL_HPH_L,
					   "hphl", micbias_mv);
	if (hphl_mv < 0) {
		dev_err(dev, "mbhc: hphl measurement: %d\n", hphl_mv);
		return;
	}

	hphr_mv = wcd9378_mbhc_measure_adc(dev, dev_regmap,
					   WCD9378_MBHC_NEW_CTL_2_MUX_CTL_HPH_R,
					   "hphr", micbias_mv);
	if (hphr_mv < 0) {
		dev_err(dev, "mbhc: hphr measurement: %d\n", hphr_mv);
		return;
	}

	if (hphl_mv > WCD9378_MBHC_ADC_CROSS_CONN_MV ||
	    hphr_mv > WCD9378_MBHC_ADC_CROSS_CONN_MV)
		dev_warn(dev, "mbhc: plug looks cross-connected: hphl %dmV hphr %dmV, over %dmV\n",
			 hphl_mv, hphr_mv, WCD9378_MBHC_ADC_CROSS_CONN_MV);
	else
		dev_err(dev, "mbhc: no cross-connection: hphl %dmV hphr %dmV, under %dmV\n",
			hphl_mv, hphr_mv, WCD9378_MBHC_ADC_CROSS_CONN_MV);
}

/*
 * Classify the accessory from the mic-line voltage and return the GE 35
 * Selected_Mode that describes it, following wcd_mbhc_get_plug_from_adc().
 *
 * A line that reads above the headset threshold is either a special headset
 * whose mic is still disconnected at this bias or a high-impedance plug or
 * extension cable; @above_band reports that case so the caller can raise the
 * bias and ask again.  The mode returned for it is headphone, which is what
 * both references settle on when the re-measure finds nothing.  Pass NULL for
 * @above_band when there is no second chance left to take.
 */
static unsigned int wcd9378_mbhc_mode_from_mv(struct device *dev, int output_mv,
					      int micbias_mv, bool *above_band)
{
	unsigned int mode;
	const char *reason;
	int hph_thr, hs_thr;

	hph_thr = (WCD9378_MBHC_ADC_HPH_THRESHOLD_MV * micbias_mv) /
		  WCD9378_MBHC_ADC_MICBIAS_MV;
	hs_thr = (WCD9378_MBHC_ADC_HS_THRESHOLD_MV * micbias_mv) /
		 WCD9378_MBHC_ADC_MICBIAS_MV;

	if (above_band)
		*above_band = false;

	if (output_mv < hph_thr) {
		mode = WCD9378_GE35_MODE_HEADPHONE;
		reason = "headphone, mic line sits at ground";
	} else if (output_mv > hs_thr) {
		mode = WCD9378_GE35_MODE_HEADPHONE;
		reason = "headphone, mic line above band (high-Z plug or extension cable)";
		if (above_band)
			*above_band = true;
	} else {
		mode = WCD9378_GE35_MODE_HEADSET;
		reason = "headset";
	}

	dev_err(dev, "mbhc: mic line %dmV vs headset band %d-%dmV (micbias %dmV) -> mode %u, %s\n",
		output_mv, hph_thr, hs_thr, micbias_mv, mode, reason);

	return mode;
}

/*
 * Move MICB2 to a new VOUT_CTL while the bias is live, as
 * wcd937x_mbhc_micb_adjust_voltage() does: drop CTRL to pull-up for the
 * change, then return it to enable.  Writing VOUT_CTL with the bias enabled
 * leaves the line ramping slowly to the new level instead.
 */
static int wcd9378_mbhc_micb2_set_vout(struct regmap *dev_regmap, unsigned int vout)
{
	int ret;

	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MICB2_ADDR,
				 WCD9378_ANA_MICB2_CTRL,
				 WCD9378_ANA_MICB2_CTRL_PULL_UP);
	if (ret)
		return ret;

	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MICB2_ADDR,
				 WCD9378_ANA_MICB2_VOUT, vout);
	if (ret)
		return ret;

	return regmap_update_bits(dev_regmap, WCD9378_ANA_MICB2_ADDR,
				  WCD9378_ANA_MICB2_CTRL,
				  WCD9378_ANA_MICB2_CTRL_ENABLE);
}

/*
 * Second stage of classification, for a mic line that read above the headset
 * band at the nominal bias.  Raise MICB2 to 2.7V and measure again: a special
 * headset's series chip connects its microphone there and the reading drops
 * into the band, while an extension cable just follows the bias up and stays
 * above it.  Follows wcd_mbhc_check_for_spl_headset().
 *
 * On a match the bias is deliberately left at 2.7V -- dropping it back would
 * disconnect the microphone just found -- and @micbias_mv is updated so the
 * caller's later conversions are referred to the bias actually applied.  The
 * raised bias is given up again on removal, see
 * wcd9378_mbhc_spl_headset_release().
 *
 * Returns the GE 35 Selected_Mode; any failure returns headphone, which is the
 * mode the first measurement had already settled on.
 */
static unsigned int wcd9378_mbhc_check_spl_headset(struct device *dev,
						   struct regmap *dev_regmap,
						   unsigned int micb2,
						   int *micbias_mv)
{
	struct wcd9378_priv *priv = dev_get_drvdata(dev);
	unsigned int mode = WCD9378_GE35_MODE_HEADPHONE;
	int output_mv;
	int ret;

	ret = wcd9378_mbhc_micb2_set_vout(dev_regmap,
					  WCD9378_ANA_MICB2_VOUT_2P7V);
	if (ret) {
		dev_err(dev, "mbhc: mic bias raise to %dmV: %d\n",
			WCD9378_MBHC_SPL_HS_MICBIAS_MV, ret);
		goto restore;
	}

	/*
	 * The reference driver waits 10ms here and the Windows driver 30ms,
	 * noting a waveform capture that takes 30-40ms to stabilise.  Use the
	 * same 50ms as wcd9378_mbhc_micbias_enable(), which is the only settle
	 * time measured on this board -- at 10ms a mic line was still climbing
	 * -- and which subsumes the 2ms the reference requires after MICB2 is
	 * re-enabled.
	 */
	usleep_range(50000, 51000);

	output_mv = wcd9378_mbhc_measure_adc(dev, dev_regmap,
					     WCD9378_MBHC_NEW_CTL_2_MUX_CTL_IN2P,
					     "in2p@2.7v",
					     WCD9378_MBHC_SPL_HS_MICBIAS_MV);
	if (output_mv < 0) {
		dev_err(dev, "mbhc: in2p measurement at %dmV: %d\n",
			WCD9378_MBHC_SPL_HS_MICBIAS_MV, output_mv);
		goto restore;
	}

	mode = wcd9378_mbhc_mode_from_mv(dev, output_mv,
					 WCD9378_MBHC_SPL_HS_MICBIAS_MV, NULL);
	if (mode == WCD9378_GE35_MODE_HEADSET) {
		priv->micb2_vout_saved = micb2 & WCD9378_ANA_MICB2_VOUT;
		priv->spl_headset = true;
		*micbias_mv = WCD9378_MBHC_SPL_HS_MICBIAS_MV;

		dev_err(dev, "mbhc: special headset, holding mic bias at %dmV\n",
			WCD9378_MBHC_SPL_HS_MICBIAS_MV);

		return mode;
	}

	dev_err(dev, "mbhc: not a special headset, mic line follows the bias\n");

restore:
	ret = wcd9378_mbhc_micb2_set_vout(dev_regmap,
					  micb2 & WCD9378_ANA_MICB2_VOUT);
	if (ret) {
		dev_err(dev, "mbhc: mic bias restore: %d\n", ret);
	} else {
		/* The reference's settle after dropping the bias back */
		usleep_range(10000, 10100);
	}

	return mode;
}

/*
 * Measure the mic line and work out what kind of accessory is on the jack.
 * Errors are reported and swallowed: the caller keeps the hardware's mode,
 * which is the permissive "headset" guess, so a failed measurement costs
 * classification but never the mic path.
 */
static void wcd9378_mbhc_classify(struct device *dev, struct regmap *dev_regmap,
				  unsigned int *selected_mode)
{
	unsigned int micb2;
	int micbias_mv, output_mv;
	bool above_band;
	int ret;

	ret = regmap_read(dev_regmap, WCD9378_ANA_MICB2_ADDR, &micb2);
	if (ret) {
		dev_err(dev, "mbhc: ANA_MICB2 read: %d\n", ret);
		return;
	}

	/* micbias = 1.0V + VOUT_CTL * 50mV */
	micbias_mv = 1000 + (micb2 & WCD9378_ANA_MICB2_VOUT) * 50;

	/* micbias_mv is repeated in the decision line, this is the raw source */
	dev_dbg(dev, "mbhc: classify start: ANA_MICB2 %#x -> micbias %dmV\n",
		micb2, micbias_mv);

	ret = wcd9378_mbhc_micbias_enable(dev, dev_regmap);
	if (ret) {
		dev_err(dev, "mbhc: mic bias enable: %d\n", ret);
		goto restore;
	}

	output_mv = wcd9378_mbhc_measure_adc(dev, dev_regmap,
					     WCD9378_MBHC_NEW_CTL_2_MUX_CTL_IN2P,
					     "in2p", micbias_mv);
	if (output_mv < 0) {
		dev_err(dev, "mbhc: in2p measurement: %d\n", output_mv);
		goto restore;
	}

	*selected_mode = wcd9378_mbhc_mode_from_mv(dev, output_mv, micbias_mv,
						   &above_band);

	/*
	 * Above the band the mic line could still belong to a headset whose
	 * microphone needs a higher bias to connect at all; raise it and ask
	 * again before settling for headphone.
	 */
	if (above_band)
		*selected_mode = wcd9378_mbhc_check_spl_headset(dev, dev_regmap,
								micb2,
								&micbias_mv);

	wcd9378_mbhc_trace_cross_conn(dev, dev_regmap, micbias_mv);

restore:
	/* Hand mic bias back to the SDCA model (IT 33 MICB and its PDE). */
	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MICB2_ADDR,
				 WCD9378_ANA_MICB2_CTRL,
				 micb2 & WCD9378_ANA_MICB2_CTRL);
	if (ret)
		dev_err(dev, "mbhc: ANA_MICB2 restore: %d\n", ret);
}

/*
 * Give up a mic bias that wcd9378_mbhc_check_spl_headset() left raised.
 *
 * Called on removal, which is where the Windows driver also drops it: "do not
 * reset any other place except when setting state to no plug".  Holding 2.7V
 * for the rest of the session would cost power and would move the headset band
 * for every later accessory, since the thresholds scale with the bias.
 *
 * Only VOUT_CTL is written.  MICB2 is already back under the SDCA model by
 * now, so CTRL is left exactly as it was found and there is no live bias to
 * ramp, which is why this does not need wcd9378_mbhc_micb2_set_vout().
 */
static void wcd9378_mbhc_spl_headset_release(struct device *dev,
					     struct regmap *dev_regmap)
{
	struct wcd9378_priv *priv = dev_get_drvdata(dev);
	int ret;

	if (!priv->spl_headset)
		return;

	ret = regmap_update_bits(dev_regmap, WCD9378_ANA_MICB2_ADDR,
				 WCD9378_ANA_MICB2_VOUT,
				 priv->micb2_vout_saved);
	if (ret) {
		dev_err(dev, "mbhc: mic bias release: %d\n", ret);
		return;
	}

	priv->spl_headset = false;

	dev_err(dev, "mbhc: special headset gone, mic bias back to %dmV\n",
		1000 + priv->micb2_vout_saved * 50);
}

/*
 * Re-arm mechanical detection and classify the accessory.
 *
 * MECH_DET_TYP selects which transition direction (insert/remove) the
 * mechanical comparator is armed for; toggle it after every GE 35
 * DETECTED_MODE interrupt so the opposite-direction transition is still
 * seen next time.
 *
 * The part has no accessory detection of its own, so on insertion also
 * measure the mic line and override the mode the caller applies to
 * GE 35 Selected_Mode.  Only a failed re-arm is reported as an error,
 * since that is what would cost the next transition entirely.
 */
static int wcd9378_sdca_post_jack_irq_process(struct sdw_slave *slave,
					       struct regmap *dev_regmap,
					       unsigned int detected_mode,
					       unsigned int *selected_mode)
{
	struct device *dev = &slave->dev;
	unsigned int val;
	int ret;

	dev_err(dev, "mbhc: ge mode detected %u, selected %u\n",
		detected_mode, *selected_mode);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "failed to resume for mech detect rearm: %d\n", ret);
		return ret;
	}

	ret = regmap_read(dev_regmap, WCD9378_ANA_MBHC_MECH_ADDR, &val);
	if (ret) {
		dev_err(dev, "ANA_MBHC_MECH read for rearm: %d\n", ret);
		goto out;
	}

	val ^= WCD9378_ANA_MBHC_MECH_DET_TYP;

	ret = regmap_write(dev_regmap, WCD9378_ANA_MBHC_MECH_ADDR, val);
	if (ret) {
		dev_err(dev, "ANA_MBHC_MECH write for rearm: %d\n", ret);
		goto out;
	}

	dev_err(dev, "mbhc: rearmed for %s, ANA_MBHC_MECH %#x\n",
		val & WCD9378_ANA_MBHC_MECH_DET_TYP ? "insertion" : "removal", val);

	/*
	 * On removal the hardware returns Selected_Mode to Safe_Mode itself and
	 * software must leave it alone, so there is nothing to classify -- only
	 * a mic bias to hand back if the last accessory was a special headset.
	 *
	 * Otherwise: MBHC_CTRL DEVICE_DET ACCESSORY_TYPE is left at its reset
	 * value 3, so the hardware calls every insertion a headset regardless
	 * of what was plugged in.  Measure the mic line and correct it.
	 */
	if (detected_mode != WCD9378_GE35_MODE_UNPLUGGED) {
		wcd9378_mbhc_classify(dev, dev_regmap, selected_mode);
	} else {
		dev_err(dev, "mbhc: unplugged, no classification\n");
		wcd9378_mbhc_spl_headset_release(dev, dev_regmap);
	}

	dev_err(dev, "mbhc: ge mode detected %u done, selected %u\n",
		detected_mode, *selected_mode);

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct sdca_class_hw_ops wcd9378_sdca_class_ops = {
	.populate_function	= wcd9378_sdca_populate_function,
	.set_jack		= wcd9378_sdca_set_jack,
	.post_jack_irq_process	= wcd9378_sdca_post_jack_irq_process,
};

int wcd9378_sdca_probe(struct sdw_slave *slave,
		       const struct sdw_device_id *id)
{
	struct device *dev = &slave->dev;
	struct sdca_device_data *data = &slave->sdca_data;
	struct wcd9378_priv *priv;
	struct gpio_desc *reset;
	int ret;

	/*
	 * 0x0217:0x0110 covers both mobile and compute modes; the
	 * qcom,wcd9378c variant compatible identifies compute-mode
	 * nodes only.  Mobile-mode nodes carry the plain class-ID
	 * compatible and are picked up by the mobile driver.
	 */
	if (!device_is_compatible(dev, "qcom,wcd9378c"))
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	/* No SPMI parent: supplies and reset live on the SoundWire DT node. */
	ret = devm_regulator_bulk_get_enable(dev,
					     ARRAY_SIZE(wcd9378_sdca_supplies),
					     wcd9378_sdca_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to get reset GPIO\n");

	if (reset) {
		gpiod_set_value(reset, 1);
		usleep_range(20, 30);
		gpiod_set_value(reset, 0);
		usleep_range(20, 30);
	}

	/* SCP writes below need the slave attached. */
	ret = sdw_slave_wait_for_init(slave, 5000);
	if (ret)
		return dev_err_probe(dev, ret,
				     "slave attach timeout: %d\n", ret);

	/*
	 * TX PDM clock: bank-1 shadow + SCP_COMMIT.  SCP survives PDE
	 * cycles; one-shot before any port is enabled.
	 */
	ret = sdw_write_no_pm(slave, WCD9378_SCP_HOST_CLK_DIV2_CTL_B1, 0x01);
	if (ret)
		return dev_err_probe(dev, ret,
				     "HOST_CLK_DIV2_CTL_B1: %d\n", ret);

	ret = sdw_write_no_pm(slave, SDW_SCP_COMMIT, 0x02);
	if (ret)
		return dev_err_probe(dev, ret, "SCP_COMMIT: %d\n", ret);

	/* DT has no DisCo enumeration; seed the descriptor here. */
	if (!data->num_functions) {
		data->function[0].type = wcd9378_sdca_desc.type;
		data->function[0].adr  = wcd9378_sdca_desc.adr;
		data->function[0].name = wcd9378_sdca_desc.name;
		data->num_functions    = 1;
	}

	return sdca_class_probe(slave, &priv->class, &wcd9378_sdca_class_ops);
}

void wcd9378_sdca_remove(struct sdw_slave *slave)
{
	struct wcd9378_priv *priv = dev_get_drvdata(&slave->dev);

	sdca_class_remove(&priv->class);
}

int wcd9378_sdca_runtime_suspend(struct device *dev)
{
	struct wcd9378_priv *priv = dev_get_drvdata(dev);

	return sdca_class_runtime_suspend(&priv->class);
}

int wcd9378_sdca_runtime_resume(struct device *dev)
{
	struct wcd9378_priv *priv = dev_get_drvdata(dev);

	return sdca_class_runtime_resume(&priv->class);
}

int wcd9378_sdca_system_suspend(struct device *dev)
{
	struct wcd9378_priv *priv = dev_get_drvdata(dev);

	return sdca_class_system_suspend(&priv->class);
}

int wcd9378_sdca_system_resume(struct device *dev)
{
	struct wcd9378_priv *priv = dev_get_drvdata(dev);

	return sdca_class_system_resume(&priv->class);
}

/* SoundWire slave-driver plumbing lives in wcd9378-sdw.c. */
