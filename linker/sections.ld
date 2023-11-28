/* SPDX-License-Identifier: Apache-2.0 */

SECTION_PROLOGUE(my_ro_section,,ALIGN(32))
{
	KEEP(*("ec_info.*"));
} GROUP_LINK_IN(ROMABLE_REGION)

SECTION_PROLOGUE(my_rw_section,,ALIGN(32))
{
	__lp_lfw_start = .;
	KEEP(*("lfw.*"));
	__lp_lfw_end = .;
} GROUP_DATA_LINK_IN(LP_RAM, ROMABLE_REGION)
__lp_lfw_size = __lp_lfw_end - __lp_lfw_start;
__lp_lfw_load_start = LOADADDR(LP_RAM);
