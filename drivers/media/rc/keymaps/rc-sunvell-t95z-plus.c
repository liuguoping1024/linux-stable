// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023 Christian Hewitt <christianshewitt@gmail.com
 */

#include <media/rc-map.h>
#include <linux/module.h>

/*
 * Keytable for Sunvell T95Z Plus remote control
 *
 */

static struct rc_map_table sunvell_t95z_plus[] = {
	{ 0xdf1c, KEY_POWER },
	// TV CONTROLS

	{ 0xdf4b, KEY_PREVIOUS },
	{ 0xdf01, KEY_SCREEN }, // TV
	{ 0xdf5d, KEY_VOLUMEUP },

	{ 0xdf4f, KEY_NEXT },
	{ 0xdf5f, KEY_FAVORITES }, // KODI
	{ 0xdf5c, KEY_VOLUMEDOWN },

	{ 0xdf42, KEY_HOME },
	{ 0xdf0a, KEY_BACK },

	{ 0xdf1a, KEY_UP },
	{ 0xdf47, KEY_LEFT },
	{ 0xdf06, KEY_ENTER },
	{ 0xdf07, KEY_RIGHT },
	{ 0xdf48, KEY_DOWN },

	{ 0xdf03, KEY_INFO }, // MOUSE
	{ 0xdf18, KEY_MENU },

	{ 0xdf54, KEY_1 },
	{ 0xdf16, KEY_2 },
	{ 0xdf15, KEY_3 },
	{ 0xdf50, KEY_4 },
	{ 0xdf12, KEY_5 },
	{ 0xdf11, KEY_6 },
	{ 0xdf4c, KEY_7 },
	{ 0xdf0e, KEY_8 },
	{ 0xdf0d, KEY_9 },
	{ 0xdf41, KEY_WWW }, // WORLD
	{ 0xdf0c, KEY_0 },
	{ 0xdf10, KEY_DELETE },
};

static struct rc_map_list sunvell_t95z_plus_map = {
	.map = {
		.scan     = sunvell_t95z_plus,
		.size     = ARRAY_SIZE(sunvell_t95z_plus),
		.rc_proto = RC_PROTO_NEC,
		.name     = RC_MAP_SUNVELL_T95Z_PLUS,
	}
};

static int __init init_rc_map_sunvell_t95z_plus(void)
{
	return rc_map_register(&sunvell_t95z_plus_map);
}

static void __exit exit_rc_map_sunvell_t95z_plus(void)
{
	rc_map_unregister(&sunvell_t95z_plus_map);
}

module_init(init_rc_map_sunvell_t95z_plus)
module_exit(exit_rc_map_sunvell_t95z_plus)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Christian Hewitt <christianshewitt@gmail.com>");
