/******************************************************************************
 *  File Name:
 *    fdb_cfg.h
 *
 *  Description:
 *    OrbitESC Flash Database Configuration
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_FLASHDB_CONFIG_HPP
#define ORBIT_FLASHDB_CONFIG_HPP

/*-----------------------------------------------------------------------------
Using Key-Value Database feature
-----------------------------------------------------------------------------*/
#define FDB_USING_KVDB
#define FDB_KV_AUTO_UPDATE

/*-----------------------------------------------------------------------------
Using Time-Series Database feature
-----------------------------------------------------------------------------*/
#define FDB_USING_TSDB

/*-----------------------------------------------------------------------------
Using File Storage feature
-----------------------------------------------------------------------------*/
#define FDB_USING_FILE_MODE

/* log print macro. default EF_PRINT macro is printf() */
/* #define FDB_PRINT(...)              my_printf(__VA_ARGS__) */

/* print debug information */
// #define FDB_DEBUG_ENABLE


#endif  /* !ORBIT_FLASHDB_CONFIG_HPP */

