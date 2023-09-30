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
Using flash abstraction layer
-----------------------------------------------------------------------------*/
#define FDB_USING_FAL_MODE
#define FDB_WRITE_GRAN 1

/*-----------------------------------------------------------------------------
Override default printf with the OrbitESC version
-----------------------------------------------------------------------------*/
#define FDB_PRINT(...)              orbit_esc_printf(__VA_ARGS__)

/* print debug information */
// #define FDB_DEBUG_ENABLE


#endif  /* !ORBIT_FLASHDB_CONFIG_HPP */

