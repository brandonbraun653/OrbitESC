/******************************************************************************
 *  File Name:
 *    syscalls.cpp
 *
 *  Description:
 *    Implementation of various system call functions
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <reent.h>
#include <Aurora/logging>

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------
Aliases
-----------------------------------------------------------------------------*/
namespace AL = Aurora::Logging;

/*-----------------------------------------------------------------------------
Forward Declarations
-----------------------------------------------------------------------------*/
struct _reent;
int _write(int file, char *ptr, int len);
int _write_r(struct _reent *r, int file, const void *ptr, size_t len);

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/
int _write(int file, char *ptr, int len) {
  (void) file;  /* Not used, avoid warning */
  AL::log( AL::Level::LVL_INFO, ptr, len );
  return len;
}

int _write_r(struct _reent *r, int file, const void *ptr, size_t len) {
  (void) file;  /* Not used, avoid warning */
  (void) r;     /* Not used, avoid warning */
  AL::log( AL::Level::LVL_INFO, ptr, len );
  return len;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
