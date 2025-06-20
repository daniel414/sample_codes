/**
 * @file pufs_log.h
 * @brief pufs log interface
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_LOG_H
#define PUFS_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define LOG_LEVEL_DEBUG   1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_WARN    3
#define LOG_LEVEL_ERROR   4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_WARN
#endif

#define LOG_PRINT(level, str, ...) printf("[%s] %s(): " str "\n", level, __func__, ## __VA_ARGS__)

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(...) LOG_PRINT("DEBUG", __VA_ARGS__)
#else
    #define LOG_DEBUG(...) {}
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
    #define LOG_INFO(...) LOG_PRINT("INFO", __VA_ARGS__)
#else
    #define LOG_INFO(...) {}
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
    #define LOG_WARN(...) LOG_PRINT("WARN", __VA_ARGS__)
#else
    #define LOG_WARN(...) {}
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
    #define LOG_ERROR(...) LOG_PRINT("ERROR", __VA_ARGS__)
#else
    #define LOG_ERROR(...) {}
#endif

/*******************************************************************************
 * Exported variables
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PUFS_LOG_H */

/*** end of file ***/
