/******************************************************************************
 *  File Name:
 *    fal_cfg.h
 *
 *  Description:
 *    FlashDB flash abstraction layer configuration
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FAL_CONFIG_HPP
#define ORBIT_ESC_FAL_CONFIG_HPP

#include <fdb_cfg.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define NOR_FLASH_DEV_NAME "norflash0"
#define KVDB_PARTITION_NAME "fdb_kv_db"

#if defined( EMBEDDED )
  extern const struct fal_flash_dev fdb_nor_flash0;
#define FAL_FLASH_DEV_TABLE \
  {                         \
    &fdb_nor_flash0,        \
  }

/*-----------------------------------------------------------------------------
Partition Configuration
-----------------------------------------------------------------------------*/
#define FAL_PART_HAS_TABLE_CFG
#define FAL_PART_TABLE                                                          \
  {                                                                             \
    { FAL_PART_MAGIC_WORD, KVDB_PARTITION_NAME, NOR_FLASH_DEV_NAME, 0, 1024 * 1024, 0 }, \
  }

/*-----------------------------------------------------------------------------
Override default printf with the OrbitESC version
-----------------------------------------------------------------------------*/
#define FAL_PRINTF orbit_esc_printf

#elif defined( SIMULATOR )


#endif  /* EMBEDDED */

#ifdef __cplusplus
}
#endif
#endif /* !ORBIT_ESC_FAL_CONFIG_HPP */
