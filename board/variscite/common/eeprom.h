/*
 * Copyright (C) 2015 Variscite Ltd. All Rights Reserved.
 * Maintainer: Ron Donio <ron.d@variscite.com>
 * Configuration settings for the Variscite VAR_SOM_MX6 board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _VAR_V2_EEPROM_H_
#define _VAR_V2_EEPROM_H_

#define MAXIMUM_COMMANDS_NUMBER	150

typedef struct __attribute__((packed)) eeprom_cmd
{
	unsigned char address_index;
	unsigned char value_index;
} eeprom_cmd;

typedef struct __attribute__((packed)) eeprom_config
{
	u32 variscite_magic; // == HEX("VAR2")?
	u8 part_number[16];
	u8 Assembly[16];
	u8 date[12];
	u32 custom_addresses_values[32];
	struct eeprom_cmd cmd[MAXIMUM_COMMANDS_NUMBER];
	u8 reserved[33];
	u8 som_info;	// 0x1=Nand Flash 0x02=eMMC Flash 0x04 WIFI included
	u8 ddr_size;
	u8 crc;
} eeprom_config;

int handle_eeprom_data(struct eeprom_config *var_eeprom_config_struct_v2);
int var_eeprom_v2_read_struct(struct eeprom_config *var_eeprom_config_struct_v2);

#endif /* _VAR_V2_EEPROM_H_ */
