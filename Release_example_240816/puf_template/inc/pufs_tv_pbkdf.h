/**
 * @file pufs_tv_pbkdf.h
 * @brief test vectors for PBKDF
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_TV_PBKDF_H
#define PUFS_TV_PBKDF_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "pufs_test_common.h"

/*******************************************************************************
 * Macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
static const struct pbkdf_test_pattern
{
    pufs_hash_t hash;
    uint32_t iter;
    uint32_t passlen;
    const void* pass;
    uint32_t saltlen;
    const void* salt;
    uint32_t keylen;
    const void* key;
} pbkdf_tp[] =
{
    { SHA_256, 1,
      6, "passwd",
      4, "salt",
      64, "\x55\xac\x04\x6e\x56\xe3\x08\x9f\xec\x16\x91\xc2\x25\x44\xb6\x05"
            "\xf9\x41\x85\x21\x6d\xde\x04\x65\xe6\x8b\x9d\x57\xc2\x0d\xac\xbc"
            "\x49\xca\x9c\xcc\xf1\x79\xb6\x45\x99\x16\x64\xb3\x9d\x77\xef\x31"
            "\x7c\x71\xb8\x45\xb1\xe3\x0b\xd5\x09\x11\x20\x41\xd3\xa1\x97\x83",
    },
    { SHA_256, 80000,
      8, "Password",
      4, "NaCl",
      64, "\x4d\xdc\xd8\xf6\x0b\x98\xbe\x21\x83\x0c\xee\x5e\xf2\x27\x01\xf9"
            "\x64\x1a\x44\x18\xd0\x4c\x04\x14\xae\xff\x08\x87\x6b\x34\xab\x56"
            "\xa1\xd4\x25\xa1\x22\x58\x33\x54\x9a\xdb\x84\x1b\x51\xc9\xb3\x17"
            "\x6a\x27\x2b\xde\xbb\xa1\xd0\x78\x47\x8f\x62\xb3\x97\xf3\x3c\x8d",
    },
};

/*******************************************************************************
 * Exported variables
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PUFS_TV_PBKDF_H */

/*** end of file ***/
