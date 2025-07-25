/**
 * @file pufs_tv_ecdsav.h
 * @brief test vectors for ECP
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_TV_ECDSAV_H
#define PUFS_TV_ECDSAV_H

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
static const struct ecdsav_test_pattern
{
    pufs_ec_name_t name;
    bool valid;
    const void* md;
    const void* Qx;
    const void* Qy;
    const void* r;
    const void* s;
} ecdsav_tp[] =
{
    { NISTB163, true,
        "\xc0\x28\xf2\xe1\xd8\xc9\x23\xe9\xbb\x81\x5e\x59\xda\xf5\x95\x52"
            "\x2b\x43\xba\x4d\x01",
        "\x05\x85\xe1\xc1\xe4\xc7\xe1\x58\x22\x87\x0d\x1c\x24\x4d\xd6\xaa"
            "\x34\xdb\x54\x17\x65",
        "\x01\xf1\xb2\x5e\x8a\xb7\xb4\x7d\x5b\xdc\xf6\x97\xdb\xb2\x31\x45"
            "\x8b\xc9\xdf\xd7\xc3",
        "\x01\xb4\x56\xc4\x29\x70\xd3\x8c\x89\x58\x92\x95\x56\xb1\x46\x23"
            "\x09\x2d\xcb\x09\x60",
        "\x00\x3f\x93\xe8\xa1\x22\x4f\x5f\x38\x90\x94\x44\x7f\xcc\xdb\xba"
            "\x90\xec\xbb\x6f\x07",
    },
    { NISTB163, false,
        "\xdd\x63\xf5\xd3\xe8\x30\x5b\x5c\xb5\xb6\xa3\x5b\xc9\x09\x50\xa8"
            "\xd5\x03\x84\xc2\xb5",
        "\x06\xfe\x80\x47\xf2\xdc\x25\x75\x9b\x67\xd5\xb2\x36\x91\x69\x12"
            "\xda\x9c\xd6\x7b\x6e",
        "\x04\x86\x3f\x66\x5b\x6b\x53\xad\xf5\x32\x17\xab\xef\xa6\x1a\xfb"
            "\x9a\x8a\x25\x6a\x6d",
        "\x00\x61\x75\xc7\x39\x41\x68\xab\xf5\x88\xb7\x10\xf6\xa0\x6d\x80"
            "\x75\xa0\x8c\x2d\x48",
        "\x01\x20\x0a\x7e\x03\x5c\x35\x76\x0f\xb8\x59\xe7\x57\xeb\x7d\x46"
            "\x08\x52\x40\x7f\x8e",
    },
    { NISTB233, true,
        "\x93\x4c\x81\x30\x46\x23\x44\x42\x30\xa1\x47\xb1\x64\xf0\xe1\x91"
            "\x04\x32\xe5\xd4\x5a\x2a\xc9\x62\xf9\xe7\xaa\xa5\x20\x16",
        "\x01\x51\xcd\x15\x73\xea\x0e\x5d\xe9\x17\xef\xfa\x18\x57\x47\x51"
            "\x75\x98\xdb\x76\xef\x15\xee\x32\xe2\x2a\x36\x30\xdc\x96",
        "\x01\xdd\xf0\x91\x80\x97\xdb\xed\x18\x97\xaf\x3f\x96\xce\x38\x91"
            "\x83\xd7\x7b\xa3\x68\xd4\xf6\x3e\x19\x64\x10\xe3\xe4\xb6",
        "\x00\x8d\xd3\x4c\x08\x0b\x4b\xe0\x97\xa3\x78\xfc\x62\x74\xc7\x76"
            "\xeb\xf6\x10\x21\xf3\xdf\xe8\x74\x0b\x94\xe1\x7d\xa7\xb6",
        "\x00\x40\x4d\xba\x4f\x3a\xd8\x9e\x02\xff\xdc\xc9\xe3\x0f\x75\x1b"
            "\x0e\x4d\xda\x8b\x63\x3b\xec\xd7\x5d\x99\xd0\x42\x09\xc7",
    },
    { NISTB233, false,
        "\x21\xb8\x5f\x0b\xef\xa5\x4f\xb2\x0d\x84\xcb\x8b\x11\xa1\x95\x6b"
            "\x5e\x69\xb6\x52\xf4\xb2\x98\x41\xad\xa1\x45\x4f\x69\x56",
        "\x00\xe1\xcc\xad\x4f\xd8\x2e\xd1\x86\x68\x76\x6b\x88\xcc\x42\xce"
            "\xb3\x50\x32\x5d\xd8\x74\x56\x68\x21\xfe\x6d\x0b\x9d\xbf",
        "\x00\x79\xf2\x6e\x92\xb2\x7e\xcf\x34\xb3\x6c\x6d\x39\x15\x62\x53"
            "\x56\x43\x01\xe2\x2d\x49\x1b\x86\x1e\xb1\x50\xb8\x50\x7e",
        "\x00\x70\xea\xfe\xd0\x83\xb9\x6a\x9b\x45\x4c\x50\x06\x67\xd4\x70"
            "\x6f\xc1\x93\x6b\xeb\x04\xd8\x66\x0f\x3d\x04\xcd\xf0\xdd",
        "\x00\xc2\xb4\x93\x4d\x73\x63\x45\xf9\xde\x22\x15\x11\xea\xc6\x9e"
            "\xeb\x6b\xa8\x78\x88\x37\x40\xd8\xf0\xb4\xa2\x1d\xaf\x2b",
    },
    { NISTB283, true,
        "\x4a\xaa\x9b\x7b\x8c\xf7\xfe\xdf\x8d\x56\x7a\xb2\x0b\x29\xc0\x47"
            "\x89\x1f\x7d\x35\x6d\xa7\xc5\x0e\xd3\x06\xa7\x25\x19\x27\xe8\xad"
            "\x6d\x48\xc3\xb9",
        "\x05\x85\x49\x01\x2f\xb0\x2c\xea\xce\xf6\x3c\x75\xa4\xf2\x3e\x6d"
            "\x2d\xe7\x49\xd8\x3e\xd9\x36\xda\x49\xa8\xc4\x55\x47\x8c\x50\x62"
            "\xbd\xb8\x00\xf9",
        "\x03\x38\x5c\xa7\x38\xdf\x7e\x56\xba\x2e\x35\x16\x7a\xa0\xbd\xad"
            "\xdc\xb7\x22\x13\x70\xa6\xea\xa1\x19\x68\x85\xe1\x1f\x7b\x46\x26"
            "\xb9\x70\xeb\x4a",
        "\x00\xdb\x0d\x34\x0b\x59\xaf\x49\xff\x78\xd0\x90\x88\x14\x80\x55"
            "\xf1\x5d\xf0\xe4\xae\xc1\xb2\x34\x09\xb9\x7c\x38\x70\xdd\x92\x76"
            "\x89\x4c\xa9\xa6",
        "\x00\x22\xb7\x46\xee\x1c\x71\xff\xe7\x77\x85\x35\x00\x04\x1a\xb5"
            "\x53\x4b\xd2\x30\xeb\xde\xe9\xb2\xa3\x70\xb5\x5a\x44\xef\xb5\x7c"
            "\x9b\x91\x6a\xe2",
    },
    { NISTB283, false,
        "\x2e\xa3\x7f\xf8\xb8\x2c\xe2\x49\x03\xac\xe4\xe4\xc5\x84\x5f\x60"
            "\xe8\xc8\x5c\xae\x87\x77\xc0\xa9\x88\x4d\x2d\xca\x84\xb3\x98\xc6"
            "\x2c\xa2\x2e\xed",
        "\x00\x1c\x10\x35\xbd\xab\x52\xd8\xe7\x3c\x00\x77\x8e\x20\xe4\xc0"
            "\x10\xd9\xd6\xac\xa6\x80\x25\x66\x08\xba\xe0\xef\x2f\xc5\x9c\xc4"
            "\x44\xa2\x90\x2b",
        "\x05\xf5\x0d\x87\xcf\xfe\x65\xe1\x1e\x34\x0c\x6b\x3a\x7a\x1d\x36"
            "\xdc\xd7\x8e\xc0\x0a\xcc\x69\x71\x16\x41\x3f\xbb\x35\xe3\xd2\x3d"
            "\x3e\xbe\xd1\xf3",
        "\x03\xe3\xc1\xa2\xb4\x04\x87\xfa\xa8\xe0\x7a\x02\x09\x20\xe7\x03"
            "\xdf\x48\x6e\xbd\x1a\x5b\x5e\xfe\xb4\xc7\xb6\x97\xff\x2d\xee\xff"
            "\x5f\xd5\x62\x8c",
        "\x00\x55\xdc\x20\xbf\x69\x44\x48\xc1\x04\x5b\xfe\x42\x3d\xa1\xad"
            "\x0b\xbf\xf7\xda\xbe\x3a\xfc\x71\x17\xec\xcc\x70\x28\xe1\x79\x9c"
            "\x02\x04\xf8\x1c",
    },
    { NISTB409, true,
        "\xc4\xa3\x37\x92\x19\x50\xa0\xf9\x0a\x17\x95\xa8\xae\xad\x01\x7e"
            "\x9a\x2d\x3b\x65\xdc\x3e\xe9\x67\x9e\xb3\xdb\xae\xa4\xce\x7a\x0f"
            "\x98\xf8\xf1\x96\xe2\xbe\x2e\x86\xfd\x4d\xcf\x2b\xe8\x1c\x53\x07"
            "\xdf\x7a\x59\x3c",
        "\x01\xc6\x3b\xe5\x57\x16\x2a\xb0\x67\x18\xdd\xda\xc5\x9d\x30\x7a"
            "\x5e\x89\xf1\x15\xf8\x0b\x8d\x0e\xce\x41\xec\x3b\x9c\xce\xb9\xcb"
            "\x6e\xd7\x4d\x1b\x0b\x37\xf4\xfc\x36\x92\xed\xba\x0c\x8f\xbd\x44"
            "\x2f\xfd\x7f\xe3",
        "\x01\x3d\x1a\x9e\x8b\xf9\x85\x7a\x6c\x0e\x9c\x93\xd3\x7a\x49\xf6"
            "\x5e\x46\x4e\xdf\x6f\xf3\x8e\x12\x23\xcc\x11\x6a\xae\x08\x01\x8b"
            "\x65\xb5\x44\x13\x67\xbc\x84\xaa\xdb\x08\xdf\x83\xa4\xe8\xa4\xeb"
            "\xe5\x7c\xac\xfa",
        "\x00\xf2\x35\x5c\x6d\x83\x61\x11\x4b\x48\xcc\x3d\x29\x9f\x5b\x4c"
            "\xf6\x32\xe4\xc6\xf3\x2b\x79\x95\x95\xd4\xea\x9c\xd2\xfd\x87\xba"
            "\x5c\x9d\xde\xb6\x05\x11\xca\x2b\x4a\xf0\xb1\xd8\x5a\xd8\xf7\x55"
            "\xa6\xe2\x9c\x27",
        "\x00\x7f\x87\x0e\xc8\x81\x09\xc0\xc4\x7a\xa0\x61\x4d\x5a\x26\xb7"
            "\x68\x63\x28\xb4\xf2\xf9\xe5\x2d\x90\xbc\x70\xc2\xac\x84\x22\x38"
            "\x98\xf2\x2d\x06\x88\x70\x8d\xd8\x87\x0d\x4e\xc0\x7d\xca\xf8\xd4"
            "\x7a\x53\x1a\x71",
    },
    { NISTB409, false,
        "\xf4\x0e\xc6\x67\xd5\x5c\x62\x2f\x13\xe7\xbf\x3b\x3c\x20\x50\xf9"
            "\xe0\x9d\x88\x1a\x01\x20\xb9\xaf\xf0\x7b\x18\x38\xf4\x34\xf0\xc9"
            "\x89\xdf\xec\x62\xb4\xe2\xc8\xcc\x78\x69\xdb\x4c\xb2\xf5\x19\x12"
            "\x4b\x0c\x2b\x2e",
        "\x01\xc9\xc1\x42\x35\x4f\xb2\x5f\xf7\x9e\x60\x98\x40\xb8\x72\x22"
            "\x4f\x05\x36\xe2\x15\x24\x3f\x65\x2b\x69\x68\x1e\xb3\x3f\xcf\xfe"
            "\x55\xfb\xff\x09\x3c\x67\xe1\x46\xfd\x50\x56\x6b\x0c\xfb\xec\x33"
            "\x8b\x51\xf8\x27",
        "\x01\x56\xfd\x08\x8e\x50\x32\x47\x8b\x5f\x7f\x72\xef\x33\x20\xd0"
            "\x04\x57\x4e\xb3\x81\x7d\xb8\xce\xf6\xa6\x2a\xc2\x26\xd0\xe3\xbb"
            "\x94\x19\x53\x6f\x85\x55\x0c\x8f\x91\x12\xa6\x74\xa5\x31\x88\xa6"
            "\xdd\x0a\x53\x1e",
        "\x00\xf2\x09\xb2\x03\x2a\xb3\x08\x19\x12\xe8\x15\x6d\xe5\x4e\x6a"
            "\xea\xe4\x4a\xa1\x52\x8a\x34\x57\x36\x5b\x1e\x2f\x60\x19\xac\xfe"
            "\x2e\xbb\x4f\xf4\xdc\xd4\xed\xbf\xbe\x45\xad\x74\x96\x91\xa0\xb8"
            "\x96\x4d\x3a\x1e",
        "\x00\x79\x23\xda\x13\x38\x46\xec\xae\x25\x7f\x7a\xcf\xea\xed\xdd"
            "\xb5\x97\x76\xae\x4f\x79\xe6\x33\x57\x2a\x6a\x3b\x82\xb5\xd4\x55"
            "\x0d\x10\xc4\x1d\xd6\x7d\x3f\x94\xe3\x3e\x17\xb1\xc1\x3c\xeb\x25"
            "\xfe\x3a\x7e\x47",
    },
    { NISTB571, true,
        "\xbb\xe8\xf1\xef\xbe\xa2\x27\x62\xf7\x6a\x4c\xdd\x6b\xce\x33\xe6"
            "\x61\x2a\xb9\x4c\x3c\xb7\xfe\x15\x45\x2e\x84\xf8\xde\x88\xd4\xeb"
            "\x58\x82\xe8\xd9\x9a\x6e\xe0\xbf\x27\x27\xdd\xc7\xdd\x1a\x65\xa0"
            "\x1e\x21\xe4\x38\xed\xd3\x79\x19\x6c\x9a\x84\xdf\x95\x52\x16\x91",
        "\x03\x5b\x8b\x21\x7a\x46\x15\xa0\x3a\xd0\xcc\x6e\x4e\xa1\xc0\x5b"
            "\xe4\x93\xc2\x77\x50\x84\xee\x0b\xcc\xd4\x94\xe3\x2d\x6c\x35\x3b"
            "\x47\xc1\x55\x7e\x02\x7f\x14\xb2\xca\x83\x58\x0d\x0a\xfd\xfc\x35"
            "\x2c\x4b\x7b\x61\x42\x64\x06\x3e\x76\x55\x58\x7c\x0e\x26\xd3\xb7"
            "\xbe\xad\x52\x66\x47\xd6\x82\xef",
        "\x03\x5c\xf9\x6a\xbc\x4b\x3e\xad\xf4\x3a\x3c\xce\x37\xda\x07\xb2"
            "\x37\x69\x13\x9b\xff\x7f\x6a\xb7\x1a\xfc\xbe\x32\x6b\x51\xf4\x74"
            "\xbb\x6f\xcf\xa6\x16\xbe\x3f\x58\x81\x11\x86\x4a\xba\x86\x26\xe8"
            "\x0e\x6a\xde\x70\x11\xad\xa3\x8b\x1a\xd7\x85\x48\x77\x3c\xcc\xa1"
            "\xa5\x8f\x43\x53\x54\xca\x42\x22",
        "\x01\xdb\x0c\x77\x49\xd8\xed\x8a\xef\x22\xb5\x83\x29\xec\xab\x60"
            "\x3e\x27\x96\x54\x59\x6d\x8b\x44\x8a\x12\x0f\x40\x77\xa1\x25\xcc"
            "\x02\x9c\xdf\xcb\x98\x6a\x60\xaf\x73\xb1\x66\xe9\xc8\xbc\xab\xce"
            "\xef\xbf\xaf\x23\x54\x0d\x87\x7e\x28\x90\xbc\x4a\x1d\xf0\x67\x78"
            "\x4d\x02\x28\x98\x83\x48\xe0\x12",
        "\x01\x6a\x28\x3b\x2e\xa2\x35\x32\xae\x5b\x08\xb2\xfb\xff\x44\xfd"
            "\x0c\xfe\xb4\xf3\x46\x24\xd0\x89\x83\x2a\xc9\xc9\xf0\x2f\xd1\xa4"
            "\x40\xf5\x32\x3d\x3e\xc0\x1a\xc3\x7b\x61\xa8\xa1\xb3\xce\xb9\xbc"
            "\x29\x9e\xab\x4d\x70\x76\xa7\xac\x55\x40\xa1\xda\xcc\x0c\x54\x05"
            "\x04\x3a\xb4\x53\x19\xfb\x54\x83",
    },
    { NISTB571, false,
        "\x6d\x15\x36\x64\x55\xdf\x4f\xfb\xe3\xef\xbd\xda\xec\x97\x17\x38"
            "\xa6\x44\x90\xcb\xa0\x95\x28\x6d\x40\x6d\x1b\x50\x0c\x5d\x76\x2c"
            "\xf1\xd4\xaa\x3b\x09\xf7\x12\xea\xb1\xb6\x90\xcb\xcc\xa1\x85\xdd"
            "\x10\x19\xcc\x47\x17\xd1\x6d\xf5\xf4\x3e\x9a\xfa\xd3\x7b\x50\x48",
        "\x04\x4e\xa7\x3e\x0d\x3a\xc8\x89\xbe\x47\x47\x70\x20\x7e\xa6\xb9"
            "\xa0\xe0\x56\xc9\x91\xbe\x02\xaa\x9c\xa6\xe0\x0c\x16\xef\x8a\x47"
            "\x25\x33\xec\x0d\x25\x39\xfa\xf1\x75\x3f\x4d\x69\xcc\x18\xfb\xec"
            "\x17\x1e\xf7\xb3\xfc\xf8\x1a\xc2\x0e\x28\xa1\x80\xf5\x64\xd4\x14"
            "\x9c\xb6\x50\xb0\x85\xef\xe2\xdc",
        "\x07\x9f\x5f\x20\xf4\xc7\xb2\xc8\x46\x7d\xdd\xb2\x10\x32\xf4\x73"
            "\xea\xee\xd3\x33\x8e\x5d\x0e\xeb\xf1\x37\x04\x48\x7f\x3e\x2d\x9d"
            "\xe8\x1e\x57\x51\x03\x25\x6f\x3e\x78\x13\xbb\x2d\x7c\x5c\x98\x52"
            "\x4c\x53\x4f\xb0\xa4\xfe\xe9\x62\xe5\xee\x7f\x14\x1b\x54\x83\x1e"
            "\xf7\xaf\x80\xee\xe6\x64\x31\x9b",
        "\x00\x53\x18\xa2\x1a\x17\x8a\x46\xbe\x52\xf3\xa3\xf0\x88\x84\x58"
            "\xa1\x17\x98\xc4\xf2\x6c\x63\x2f\xfb\xfe\x68\x59\x6a\x43\x4f\x43"
            "\x8d\xf7\x4d\x34\x61\xfd\xff\x32\xf4\xc7\x97\xc5\xbd\xc8\x47\x68"
            "\xb7\x72\x54\x5a\x57\xb8\x92\x83\xfb\x2a\x51\xe9\x74\x06\xe4\x11"
            "\x3a\x4e\x7b\xee\x32\xfc\xf1\xd5",
        "\x02\xf0\x42\x3e\x77\xce\xab\x4d\x52\x4a\xc9\xe3\xb8\x3e\x27\xe3"
            "\xdf\xec\xab\x93\x82\x51\x2b\x3d\x54\x3a\xff\x00\x58\x2b\xa0\xff"
            "\x87\x1e\x7e\x82\xc2\x3a\x90\xbf\x67\x2c\x7f\xf1\xc4\xe9\xfc\x84"
            "\x45\x0e\x8c\xac\x5e\xce\xd4\x6a\x9b\x3b\xa4\xb6\x5c\xbd\x37\xce"
            "\x0f\xd2\xfc\xc8\xd9\xeb\x3f\xff",
    },
    { NISTK163, true,
        "\x64\x9d\x71\x92\x7a\x1c\xe9\x8b\x11\x32\x5a\x95\x74\xfc\xd3\x72"
            "\x0e\xca\xe9\x41\x6e",
        "\x05\xf0\x3c\x98\xef\xd0\x76\x58\x82\x8a\x75\xf2\x9a\xf9\x5b\xb8"
            "\x83\x22\x5c\xd1\x70",
        "\x00\xdc\xba\x47\xba\xb2\x92\xad\x7e\xea\xea\xd1\x1c\x0f\x73\x52"
            "\x86\xa2\xc4\x07\xe3",
        "\x01\x3f\xcc\x19\x56\x65\x6d\xb0\x68\xfb\xf7\x04\x71\x5d\x4a\x1a"
            "\xa5\x9e\xea\x69\xb3",
        "\x03\x8c\x72\x24\x38\xa7\x67\x50\x7d\x10\x75\x9d\xc4\x93\x4f\x87"
            "\xe3\xce\x25\xad\xc1",
    },
    { NISTK163, false,
        "\x99\x04\x36\xbb\x01\x4d\x7a\xeb\xed\x12\xe1\x06\x0a\xd9\x25\x7a"
            "\x08\x6e\xbb\x0d\x84",
        "\x03\x89\xd9\x25\x96\x8c\x5e\x06\x39\xe8\xda\xb9\x27\x9b\xa8\x63"
            "\xa7\x0d\xf5\xa5\xef",
        "\x06\x3e\xae\x5a\x66\x54\x06\xed\x19\x1f\x6d\xf1\x52\x65\xfe\xc0"
            "\xe4\x55\xe2\xe9\xc1",
        "\x00\xf7\x55\x61\x7a\xa6\xe4\xcb\x39\xff\xaa\xd1\x47\x81\x8e\x26"
            "\xd1\x19\xec\x2f\xac",
        "\x00\xf3\xd0\xd8\xac\xb5\x3e\x9b\x2e\x66\x61\x3c\x4c\x2c\x13\x0a"
            "\x77\x71\x90\x04\x5a",
    },
    { NISTK233, true,
        "\xfc\x42\x00\x8d\xd1\xaf\x8f\x8c\x42\xed\x5a\x87\xd6\x89\x14\x43"
            "\x82\x3e\x27\xe1\x52\x9d\x32\xb2\x96\x9e\x48\x78\x31\xee",
        "\x00\x56\xda\x44\x69\xa2\xf0\x85\x4b\x92\x26\x2d\xb4\xdb\xab\x70"
            "\xd2\x13\x67\x0d\x89\xb5\x41\x36\xad\x43\x3b\x59\xa3\xa0",
        "\x00\x39\xc1\xe0\x69\x29\xda\x10\xec\x9e\x54\x94\xc9\x4a\x09\xe8"
            "\x59\xbb\x36\x86\x35\xe8\xec\x5b\xdc\x8c\x38\x23\xe2\x83",
        "\x00\x44\x9d\x9c\x77\xc3\x2b\x6d\x5a\xf2\xea\xa2\xf7\x3b\xc8\xf7"
            "\x0d\xe9\x8e\x24\x92\xfa\x9f\x19\x94\x19\xcd\xd7\x64\x9b",
        "\x00\x32\x84\x81\x3f\x43\x47\x9c\x61\x4c\xab\x54\x0c\x2d\x49\x14"
            "\x15\x1a\xc0\xd0\xe1\x34\x43\x2c\x29\xd6\x89\x37\x1c\x5c",
    },
    { NISTK233, false,
        "\x4d\x95\xf3\xee\xaf\xcf\xd8\x62\xd0\x79\xda\xb9\x7b\xda\x0e\x61"
            "\x60\xa5\xc5\x6b\x80\xcd\xfb\xfc\xcb\x4a\xcf\xb6\x79\xea",
        "\x01\x8f\xda\x2d\xe3\xee\x40\xf8\xe9\x56\x2d\x77\x3c\xa9\xbf\x95"
            "\x59\xda\xb8\x45\x11\xc7\x3d\x41\x31\xa7\x1c\xb3\x1c\xd6",
        "\x01\x4e\x31\x4e\x03\xdf\x5a\x8d\xf9\xf8\x84\x62\xdc\xc4\x28\xe5"
            "\xe2\x83\x12\x55\x7b\x3d\xfa\xc0\x91\x3f\x46\xda\x30\x75",
        "\x00\x44\xb8\x52\x00\x75\x52\xc7\x9e\xdb\x55\xd3\x96\x45\x4d\xd9"
            "\x6e\xfc\x6d\x6b\xcb\x9f\x11\xa4\x62\x23\x5b\x4d\xb1\x4d",
        "\x00\x2f\x06\xa4\xc5\x55\x0c\x45\x81\x1a\xe8\x90\xe3\x16\xb9\xee"
            "\x79\x1a\x42\xc7\x07\x9f\xc7\xd7\x87\xcf\xd4\xc2\xdb\x9c",
    },
    { NISTK283, true,
        "\x4d\xf7\x06\x72\x22\x59\x6d\x4f\xb3\xdd\x86\xdc\x64\x01\x4f\x72"
            "\x68\x35\xcc\xbf\x31\xd1\xdb\x5d\x91\xed\x54\xc2\x15\x33\xd7\xdf"
            "\x76\x63\xf6\xe2",
        "\x01\x82\x9e\x6a\x31\xea\xe6\xaa\x37\x3a\x69\xb1\x10\x2a\x8a\x4a"
            "\x2a\xbe\x7b\xe6\x5d\x06\xe3\x7d\xcf\xde\x68\x63\x49\xc2\x9f\xe0"
            "\x04\x61\x30\xad",
        "\x02\x14\x72\x4f\xb1\x6b\xea\xa7\xb4\x21\x54\x6c\x9f\xdd\x03\x59"
            "\x1c\xd4\x8e\x0c\x5b\x6d\x36\x25\x2a\x34\x90\xae\x69\x9a\xa6\xc6"
            "\x1e\x97\x15\xb7",
        "\x00\x9c\x6f\xa3\xbb\x3e\x9e\x68\x45\x45\xaf\x7d\x4f\x08\x33\x66"
            "\x0a\xcd\xc9\xfb\x46\x4b\x91\x22\xc3\x6c\xef\xc0\x3a\x37\x41\x14"
            "\xbc\x38\xb3\xcc",
        "\x01\x39\x08\x99\xd7\x8b\x97\xf7\xec\xef\x9f\x33\xa8\x2f\xcc\x25"
            "\x60\x5d\xf0\xb6\xd1\x6b\x31\x78\x7b\x29\xd8\xa1\xe5\xfb\x44\x49"
            "\x35\xdb\x42\x7e",
    },
    { NISTK283, false,
        "\xa6\x68\x6c\x17\x49\x4c\x09\xfb\x22\x22\x10\x8f\xcf\x82\x1c\x27"
            "\x57\xcd\x2c\x5c\xea\xa0\xe0\x57\x93\x21\x3c\x01\x42\x09\xa1\x99"
            "\xaf\xf4\x01\xf8",
        "\x03\x05\x57\xec\x31\xf7\xab\xb0\x62\xa4\xaf\x52\x07\x70\x7a\x7e"
            "\xcd\x81\x03\x41\x51\x46\x68\xaf\xea\x28\x18\xb5\xcd\x12\xda\xed"
            "\x07\x14\x59\x4a",
        "\x07\x1a\x7a\x1b\x4e\x88\xed\x37\xdd\xcc\xd6\xd7\xc6\x60\xd5\x72"
            "\x11\xac\x40\x3d\x53\xf6\x11\x9d\xbd\x56\xd9\x14\xfd\x8e\xba\x3e"
            "\xca\xa5\x7b\x87",
        "\x00\xe6\xd6\x87\xb3\x71\x2a\x2a\x85\x0a\xa7\x71\xc4\x08\x16\x7f"
            "\x97\x8c\x7e\x8c\xef\x50\x0e\x12\xdd\x88\x9d\xa8\x1d\x2c\x62\x88"
            "\x52\x6c\xb8\x2c",
        "\x00\xfa\x01\x19\x78\x1a\x09\x61\xeb\x7b\x23\xef\x18\x4b\xbe\x68"
            "\x63\xc4\x7d\x83\x61\x61\x9d\xf2\x97\x2e\xa3\xcf\x41\xe3\xd5\xf7"
            "\xb2\xcf\x41\xc0",
    },
    { NISTK409, true,
        "\x36\xe7\xda\xc9\x16\xdc\xd5\xa2\x4d\x14\x18\x86\x90\xb9\x12\xe7"
            "\x1a\x4d\xf4\x71\x18\xa5\x55\x1d\x64\x65\x1a\x0b\x86\x96\xce\x9f"
            "\x2f\xde\x8b\xc6\x8b\x11\xe1\x58\xc5\x61\x94\x88\xf5\xff\xbd\xf3"
            "\xf8\x78\xb4\xd2",
        "\x01\x7d\xa5\xaf\xbb\xef\x58\x2b\xf4\x55\xe5\x85\x4e\x52\x85\x0c"
            "\x9f\xa5\x9e\x8a\x20\x3f\xc8\xd4\x30\x3f\x6c\x25\x52\xb6\x8e\x04"
            "\x94\xa4\xac\x6e\x81\x57\x5e\x00\xe9\x45\x0a\xc3\x31\x42\x5e\x7b"
            "\x24\xee\x8c\xd6",
        "\x01\x25\xc3\x36\x70\xf9\x78\x9c\xbe\x50\x32\x15\xbb\xd5\xf0\x5a"
            "\x37\x50\xc6\x88\x8a\x39\x74\xab\x23\xac\xef\x89\x07\x25\xcf\x90"
            "\xa8\x6d\xb4\xd3\xa9\x1e\x9f\x04\x7b\xad\x6d\x52\x38\xb0\x23\xc7"
            "\xe0\x2d\x9c\x09",
        "\x00\x40\x16\x06\x1c\xe5\x6d\x01\xf1\x5e\x12\x43\xb4\x9b\x58\x14"
            "\x1f\x3e\x83\xb0\xb0\xfd\x4e\xce\x78\x6d\x2f\xc9\x66\xd6\xb4\x6a"
            "\x5b\x2b\x92\x9f\xd5\xb2\x64\xe9\x12\xcb\xd3\xb2\x5b\x92\x0e\x86"
            "\xc1\x72\xd9\x75",
        "\x00\x1c\x26\x7e\xe7\xe1\xc3\x93\x79\x97\x2a\x4b\x98\xb3\xea\xb1"
            "\xab\x9d\x6c\xed\xa3\x80\x16\x59\x5a\x08\xa9\xbc\x62\x4e\x98\x4b"
            "\xec\xd0\xf5\xd0\x7d\x51\xd9\x55\x31\x9a\x64\x9d\x28\x96\xe6\xcb"
            "\x80\x16\xc3\x32",
    },
    { NISTK409, false,
        "\x14\x7f\xc0\x4f\xd1\x6b\xb0\x3e\x99\xe9\x52\x66\xff\x07\x95\x20"
            "\xb7\xa8\xc0\x1d\xed\x26\x29\xfd\x6a\xe8\x4d\x20\x09\x47\x43\x25"
            "\xff\xbd\x01\x06\x41\xb9\xb3\x8d\x0a\x81\x45\x00\x24\x8e\x82\x11"
            "\xfb\x9c\x6a\x90",
        "\x01\x93\xaa\xeb\x5a\x88\xf3\x38\xcb\xcf\x78\x86\xfa\x69\x63\x30"
            "\x1f\x6a\x6f\xb6\xb2\x7e\xf9\xda\x31\xdd\x58\x76\x3f\xcf\xfa\xd9"
            "\x2f\x90\xf6\xdf\x7b\xd4\xc9\x7d\x41\xac\x03\xeb\xaf\x4a\xec\x14"
            "\x45\x20\x11\x9a",
        "\x00\x62\xb2\x5a\x58\x34\x50\xed\x90\x6c\xe9\x24\x73\x71\x6b\xcd"
            "\xaa\xab\xf1\x2b\xd9\x67\x1c\x06\xea\xf9\xdb\x76\x45\x21\x06\x88"
            "\x21\x1a\xad\xe7\x47\x96\x56\xb8\x2b\xf0\xb6\xc2\x35\xd9\x90\x16"
            "\x1a\x22\x88\x81",
        "\x00\x08\xc0\xf6\xaf\x47\xe8\xab\xd6\xcc\x14\xb3\xc1\x5e\xdb\xbd"
            "\x78\xd1\x04\x1a\xe1\x7d\xb9\x09\xc5\x01\x7a\xd9\xd5\x86\x4a\x6f"
            "\xd2\xf6\x30\xf3\xf6\xe3\x6d\xb6\x41\x6f\xdf\xd4\xe1\x79\x8c\xa0"
            "\x71\xfb\x76\xdd",
        "\x00\x54\x94\x40\xed\xb7\xca\x9b\xb8\x88\xa2\x60\x40\x6b\x64\x09"
            "\xe1\x0f\x48\x44\xbe\xee\x48\x33\x6d\x9d\xc4\xfe\x08\x6e\x1d\xa0"
            "\xdc\xde\x5a\xf0\x2a\x66\xba\x93\x26\x19\x73\x3d\xed\x8b\x0b\xdc"
            "\x13\x2a\x16\xd2",
    },
    { NISTK571, true,
        "\xd4\xff\x8d\xd2\x9d\x0f\xab\x4d\x45\x3b\x90\x6b\x23\xf2\xc8\x36"
            "\x0f\x9e\x90\xb0\x79\x4a\x32\x4b\xa3\x65\xfa\x42\x34\x04\x0e\x17"
            "\x44\x1a\x4c\x9d\x2a\x7d\x5b\x63\x8b\x84\xce\xf1\xe1\xfa\x28\x23"
            "\x3a\x92\x80\x0a\xf4\x41\xd2\x4e\x26\xfd\x5f\xee\x80\xd3\x9a\x71",
        "\x05\x8a\x26\x8f\x8f\x6a\xcb\xf1\xe6\xea\xf7\x68\xb4\x1f\xea\x5f"
            "\x3f\xe1\xde\x95\x5d\x21\x7b\x38\xaa\x0a\x0d\x17\x7f\x38\xfb\x55"
            "\x55\x95\x9d\xfd\xa2\xc4\xa5\xa0\x70\x17\xc6\x30\xa6\x3f\x1f\x12"
            "\xb9\x32\xb4\x9b\x3f\x4c\x63\xb8\x8d\xf9\x40\xcc\x00\x57\xbe\x9f"
            "\x75\x15\x74\xd4\x11\xa7\x59\x11",
        "\x06\x85\xaa\x85\xa9\xac\xca\x62\x02\xea\x39\xa4\xac\x70\x7e\x7c"
            "\x71\xa7\xd6\x86\x54\xaa\x66\x87\x7f\x59\xd2\x01\xe0\x27\xc0\x5c"
            "\x79\x87\x83\xa3\xd2\x49\xff\x7c\xe5\xe4\xce\x70\x2f\x62\xf8\x3e"
            "\x5d\x0e\x21\x1f\xd5\x49\xf9\xe9\x54\x71\x75\xd0\x72\xa4\xc6\x9f"
            "\x1f\x7e\x6f\x2f\xee\x79\xa0\x3d",
        "\x01\x56\xde\x73\xd8\xf4\x72\x45\xff\x8a\x48\xf8\xa6\xea\x83\x30"
            "\x4e\x10\x3a\x95\xa0\x4b\x68\x5a\x99\x70\xe1\x3b\x8e\x1d\xa8\xaa"
            "\x4f\x01\x4f\xae\x37\x79\x36\x4f\x9d\x48\x50\x9e\xcc\x08\x4d\x8e"
            "\x92\xb4\x53\x92\x96\xe2\xfb\x64\x85\x19\xef\x51\x7a\x06\xfb\x44"
            "\xce\xf9\x8a\xac\x27\x33\x6b\x49",
        "\x00\x38\xc4\x36\x41\xaa\x56\xf0\x2d\x51\x1a\xd6\x4f\xdf\x64\xf1"
            "\x86\x22\x38\x0e\xf0\xd2\x3c\xb2\xfc\xd5\x8e\x80\xc8\xd2\x9e\x35"
            "\x11\x76\x4e\xcd\xe6\xbc\xf8\x37\xf4\xf5\x1b\x55\xa2\x61\xb5\x52"
            "\x54\x20\x60\x2d\xd0\x0b\x12\xc3\x13\x35\x7b\x7f\x88\xa7\xd5\xe0"
            "\xad\xc3\xb6\x3c\xc3\x54\x3f\xba",
    },
    { NISTK571, false,
        "\x91\xc2\x74\xd6\x6d\x82\xc7\x86\x8e\x78\x1b\xa8\x02\xb0\x9b\x78"
            "\x35\x01\x23\x4e\xbc\xd7\xd3\x96\xba\xee\x53\xc6\x3d\xd9\x19\x17"
            "\xdc\xee\xd4\x98\xd1\xad\x2d\x3d\xeb\xf2\x5f\xbd\xc1\x57\x12\x00"
            "\xfb\x86\x26\xd9\x9f\x2e\xaf\x43\x97\x92\xe1\x59\x7d\x36\x8b\xb5",
        "\x06\x26\x4f\x9f\xa4\x5f\x65\xbe\xb6\xa5\xda\x6e\xab\xff\xc3\x66"
            "\x91\x97\xc8\x36\x19\x36\xcd\xfa\xf5\xdf\x1b\xd1\x7b\x13\x91\x2e"
            "\xea\xa8\xbb\xdb\x18\x3a\xb1\x57\xca\xdf\x23\xa8\x34\xfd\x96\x1e"
            "\xec\x71\xf9\xbe\xf0\xe4\x27\xfc\xfc\xac\xea\xa3\x8e\xa5\x13\xa7"
            "\x5b\xb9\x10\x3e\x06\x9c\x98\xb6",
        "\x01\xd5\xd5\x6e\xc9\x68\x91\x32\x7b\xc2\x20\x28\x2f\xf9\x7b\x3f"
            "\x17\x8d\x3d\x3e\x79\xf2\xef\xb9\x34\xc5\x89\xf0\x96\xed\xbf\x78"
            "\xfd\xfb\x0b\x1a\x4b\x1e\x1e\xf2\x4f\x76\x60\xae\x84\xa4\xc2\xfe"
            "\x11\x84\x2f\x2a\x6f\x6c\x50\x61\xf3\x40\x58\xce\xd9\x6e\xf1\x22"
            "\x24\x72\x83\xa3\x6b\x5f\x62\x4b",
        "\x01\x1d\xd8\xd3\x9f\x35\xf7\xbd\xc3\xc0\xb2\x75\x09\x66\xf7\x7c"
            "\x0e\xc2\x75\x98\x5f\x9a\x0a\x80\x3c\x8c\x82\xd9\xe2\x81\xb7\xbc"
            "\x41\xcc\xe0\x44\x6f\x3b\x10\xf5\x5b\x07\x64\x53\xa1\x75\x8b\x78"
            "\x48\x5f\x17\x5c\x9a\x31\xe8\x79\x86\x18\x6a\x58\x37\x63\xa7\x77"
            "\x16\xfd\x76\x72\xcd\xee\x32\x9c",
        "\x01\xfb\x18\x59\x9f\x3c\xae\x96\x9a\x34\xbb\xe2\x48\x84\x37\xeb"
            "\xa0\x21\x12\xfb\x19\xf9\x9f\xc2\x30\x3c\x26\xb1\x23\xca\x2c\x7e"
            "\xbe\x56\x88\x5e\xc3\x3b\x7c\xd2\x99\xce\xd0\x22\xa3\x02\x3c\xf3"
            "\x8a\x59\x87\xa9\x83\x3d\xf7\x99\xc0\xee\x5e\xa4\x4d\x71\x7c\xa1"
            "\x4e\x75\xc7\x95\x67\xaa\x4b\x6b",
    },
    { NISTP192, true,
        "\x85\x67\xaf\x1e\xf1\x3f\x90\x09\xfc\x19\xf1\xd5\xb3\x0b\xf0\x9a"
            "\x2a\x23\xb5\xb7\x6d\x79\x66\xa8",
        "\xd1\x64\x8a\xa0\x94\xdb\x4f\x15\x49\x56\x1d\xff\xa7\x21\x50\x07"
            "\xbf\x81\x73\x95\x81\xfb\xfa\x46",
        "\x44\xd9\xf9\xbb\x70\xff\x41\xd8\x6d\x47\x4e\xe1\xe5\xc6\xbc\x56"
            "\x16\x32\xbf\x08\x2c\x0b\xe9\xcf",
        "\xd2\x0d\xab\x74\x44\xa2\x06\x6a\xa9\x38\x15\x21\x7e\xa0\xb0\xd0"
            "\xc2\x55\x86\x80\xe7\x78\x29\xb3",
        "\xf6\x35\xff\xac\x94\x14\x47\x53\xfa\x06\x2e\xc3\x93\xa7\x95\xcc"
            "\x93\x23\xc4\x91\x4a\x30\x23\xe5",
    },
    { NISTP192, false,
        "\xcd\xd0\x6c\xa5\x71\xad\x0f\x57\xdf\x74\x95\x3a\xa8\x17\xb3\x0b"
            "\x1e\x13\x64\x38\x7b\xbb\xc2\x4d",
        "\xdb\x45\x1f\xd9\x24\x9d\x57\xe4\x9e\x7b\xae\xd4\x1c\x20\x20\x8d"
            "\x8a\x02\xe4\x66\xbd\xe8\x0e\xbf",
        "\x6d\xea\xa3\x38\x0a\xa3\x7b\xc6\x20\x6b\x69\xad\x79\xe9\xb8\x63"
            "\xd9\xbb\x50\x97\x0d\xdd\x82\x50",
        "\x4c\xb5\xa6\x3d\x5d\x05\x8a\x4e\xc5\xcf\x6f\xab\x41\x3d\x9d\x56"
            "\x03\x7e\x32\x3e\x0b\x34\xa2\x62",
        "\xb2\x8a\x9b\x10\x1d\x74\xa5\x18\xac\x71\x96\x70\x13\x20\xc1\x89"
            "\x21\x01\x3b\xbf\x24\x98\x78\x2b",
    },
    { NISTP224, true,
        "\x83\x19\x79\x40\x5d\xb4\xeb\x9d\xad\xf0\x12\x49\xfa\x15\xf6\x8d"
            "\x48\x46\xe0\xec\xe7\x0a\x32\x0d\x30\x22\xf7\x5f",
        "\x34\xc5\xff\x3d\xe5\x65\xb8\x5b\xfd\xd9\xf0\xa8\xb3\xfb\x0d\x46"
            "\xf9\x24\xc5\x7b\x27\x6b\xcc\x83\x0a\x1e\xd5\x80",
        "\x60\x9d\x22\x20\x0e\xf3\x8b\x41\x0d\xa7\x7f\x7a\x8f\xf2\xf5\x84"
            "\x48\x18\x80\x42\x97\x8f\xd9\xae\x1b\x2b\x44\x77",
        "\xf0\x13\x80\x24\xfe\x05\x16\x73\x8f\x3b\xd0\xe0\xfe\xc1\x0d\xef"
            "\xac\xa8\xc3\xb8\x9c\x16\x1a\x77\x48\x9c\xf2\xb7",
        "\x4a\xe0\x93\x42\x66\xd9\xe3\xd6\x4c\x2a\x12\xf5\x46\xb1\x32\xba"
            "\x0f\x33\xef\x50\xab\xc9\x0e\x7e\xf5\x97\x48\x05",
    },
    { NISTP224, false,
        "\xd8\x45\x46\x40\xad\x1f\x46\x32\xcc\x66\x78\x23\x41\x8a\xe5\x6c"
            "\x62\x02\x88\x25\xd7\x27\xad\xfc\x84\xaf\xdb\x08",
        "\x88\x56\xfb\x8b\x81\xa4\xea\xcd\x97\x1a\x95\x45\x60\x01\x8f\x33"
            "\xcb\xb7\x1c\xc1\xfc\x24\x3d\x03\xf6\x3c\xab\xcb",
        "\x28\xaf\xa2\x6b\xaf\x31\xb4\xd8\x9d\xe1\xda\xdd\x22\x89\x00\x6f"
            "\x83\x6f\x23\xa1\x13\x83\x81\x7e\xc7\xe4\xe7\x99",
        "\xef\xcc\xef\x33\x18\x05\xe7\x1b\xbf\x87\x6c\xbb\xc2\x34\x2a\x6b"
            "\xc4\x50\x8a\xea\x7c\x69\x10\x29\xc8\x39\x6a\xef",
        "\xbe\xd5\x44\xd0\x9e\x28\xdb\xf0\x1a\x30\xb2\xcf\xb6\x1b\x98\xad"
            "\x62\x01\xa9\x81\x8f\x22\xb4\xf5\x43\xf3\xe7\xf5",
    },
    { NISTP256, true,
        "\xad\xae\xad\xda\x3f\x0e\x94\x1f\xba\x1d\x3e\x20\x6a\x84\xe6\xd7"
            "\x53\x0d\x80\x0e\x0f\x21\x5b\x3d\xdd\x82\x02\x2f\x27\xc5\xbe\x44",
        "\xce\x4d\xcf\xa7\x38\x4c\x83\x44\x3a\xce\x0f\xb8\x2c\x4a\xc1\xad"
            "\xfa\x10\x0a\x9b\x2c\x7b\xf0\x9f\x09\x3f\x8b\x6d\x08\x4e\x50\xc2",
        "\xd9\x8a\xe7\xb9\x1a\xbe\xe6\x48\xd0\xbf\xde\x19\x27\x03\x74\x1a"
            "\xc2\x1d\xaa\xd7\x26\x2a\xf4\x18\xb5\x0e\x40\x6d\x82\x5e\xb0\xd6",
        "\x59\x7e\x1e\x04\xd9\x3a\x6b\x44\x4c\xcc\x44\x7a\x48\x65\x1f\x17"
            "\x65\x7f\xf4\x3f\xb6\x5f\xe9\x44\x61\xd2\xbf\x81\x6b\x01\xaf\x40",
        "\x35\x9f\xe3\x81\x79\x63\x54\x8e\x67\x6d\x6d\xa3\x4c\x2d\x08\x66"
            "\xaa\x42\x49\x92\x37\xb6\x82\x00\x28\x89\xea\xf8\x89\x38\x14\xd2",
    },
    { NISTP256, false,
        "\x5a\xa8\xe8\xa6\xf0\x62\x2b\x84\x14\x16\xe1\xa7\x0d\x79\xa5\x46"
            "\x41\xd2\xc6\x99\xa0\x75\xb6\x96\x0f\xe5\xdc\xf9\x63\x01\xda\x8c",
        "\x40\xde\xd1\x3d\xbb\xe7\x2c\x62\x9c\x38\xf0\x7f\x7f\x95\xcf\x75"
            "\xa5\x0e\x2a\x52\x48\x97\x60\x4c\x84\xfa\xfd\xe5\xe4\xca\xfb\x9f",
        "\xa1\x72\x02\xe9\x2d\x7d\x6a\x37\xc4\x38\x77\x93\x49\xfd\x79\x56"
            "\x7d\x75\xa4\x0e\xf2\x2b\x7d\x09\xca\x21\xcc\xf4\xae\xc9\xa6\x6c",
        "\xbe\x34\x73\x0c\x31\x73\x0b\x4e\x41\x2e\x6c\x52\xc2\x3e\xdb\xd3"
            "\x65\x83\xac\xe2\x10\x2b\x39\xaf\xa1\x1d\x24\xb6\x84\x8c\xb7\x7f",
        "\x03\x65\x52\x02\xd5\xfd\x8c\x9e\x3a\xe9\x71\xb6\xf0\x80\x64\x0c"
            "\x40\x61\x12\xfd\x95\xe7\x01\x58\x74\xe9\xb6\xee\x77\x75\x2b\x10",
    },
    { NISTP384, true,
        "\xea\x05\x6b\xeb\x11\x2f\xa9\xaa\xd6\x9c\x8d\xfe\x51\xea\x94\x7b"
            "\x77\x2b\xf1\xc1\x12\x87\xed\xce\xde\x43\xa9\x80\x89\xd2\x14\x92"
            "\xed\x58\x1e\xdc\xb6\xd1\x82\x3e\x28\x73\xaa\xbb\xa2\x13\xb8\x42",
        "\xc6\x65\xfe\xcc\xf5\x1e\x6b\xca\x31\x59\x30\x87\xdf\x60\xf6\x5b"
            "\x9f\xe1\x4a\x12\x02\x28\x14\x61\x5d\xeb\x89\x2e\xed\xb9\x9d\x86"
            "\x06\x9a\x82\xaa\x91\x31\x93\x10\xb6\x65\x88\x18\x52\x82\xda\xd6",
        "\x1e\x6e\x25\xbb\x8a\xe7\x71\x44\x15\xb9\x4f\x89\xde\xf0\xf7\x5d"
            "\xcb\x81\xd4\xaf\x6b\x78\xd6\x1f\x27\x7b\x74\xb9\x90\xc1\x1a\xff"
            "\x51\xbd\x12\xfc\x88\xd6\x91\xc9\x9f\x2a\xfd\xe7\xfb\xd1\x3e\x51",
        "\x0e\x18\xc4\x06\x31\x37\x46\x8f\xe8\x64\xfd\xc4\x05\xad\x4e\x12"
            "\x01\x76\xeb\x91\xb4\x53\x8b\x28\xce\x43\xa2\x2a\xe1\xa3\x10\xcc"
            "\x22\xa2\xf7\xa2\xb3\xa0\xf3\xd1\x5e\x0f\x82\x03\x8b\x4a\x43\x01",
        "\x5a\x16\x20\xe4\x20\x41\xce\x43\x57\xda\xf8\x24\xbe\xfb\xb2\xed"
            "\x65\x59\x6b\xcd\x82\x14\xe8\x87\x26\x14\x9b\x26\xb1\xf4\x16\xb9"
            "\x47\x2a\x88\x77\x41\x3f\x1c\x37\x05\xfc\x2e\xdf\x47\x31\x94\x3b",
    },
    { NISTP384, false,
        "\xd5\xc8\x2f\xf1\x1f\x55\x5c\xe2\x1c\x3f\x20\xa9\xec\xfa\x60\x47"
            "\xcb\x68\x95\xe3\x2f\xa0\xfb\x37\x9f\x49\x08\x5a\x59\xf6\x1b\x7c"
            "\x8f\xa0\x50\x58\xef\x14\x4c\xf4\x7d\xb5\x73\x8f\xa4\x0f\x48\x90",
        "\xd4\xe9\x3c\x4b\xaf\xb5\x4c\x06\x81\x40\x11\x30\x9e\x9f\x3d\x8e"
            "\x68\xb7\x6a\x54\x52\xe3\x64\xef\x05\xcc\xc3\xb4\x4b\x27\x1e\x57"
            "\x6c\x90\x28\x10\x6b\x15\x84\xf0\x92\x71\xc8\x86\xd4\x67\xf4\x1d",
        "\xdb\x73\x0c\xcf\xde\xb6\x64\x43\x62\xf4\xfb\x51\x0d\x52\x54\xbf"
            "\xe6\xf2\x3e\x89\x1e\x93\x61\x32\xf9\x0f\x19\x13\xe9\x3b\xaa\x8b"
            "\x1f\x8c\x06\x13\xa0\xf0\xc6\x1a\x76\x0c\xe6\x59\xf2\x2b\xab\xc6",
        "\x8d\x0f\xd1\x4a\x59\xc2\x4b\x0c\x2a\x34\xb4\x38\xe1\x62\xf1\xf5"
            "\x36\xfe\x09\xa6\x98\xca\xcf\xe0\x76\x0d\x02\x6d\x15\x93\x26\x5d"
            "\x02\xf2\x66\x8d\x2a\x5e\x49\xac\x0b\x21\xe9\x38\x07\xaa\x9c\x18",
        "\x31\x62\xff\xd2\xad\xc9\xdd\x5e\xc1\xbb\x1d\x97\xd2\xb0\xc2\x7b"
            "\x8a\xe2\x34\x23\x5f\xfb\x37\x48\x78\xd0\xb7\x63\x82\x00\x2e\xa5"
            "\x05\xe8\x85\xc1\x78\xd5\x6a\x2d\x78\x09\xbd\x1d\x83\x11\x7e\xf1",
    },
    { NISTP521, true,
        "\x97\xff\x5a\x81\xfc\x88\xf7\xdd\xd3\xbc\x58\x15\x4f\xfd\x26\x95"
            "\x91\x2f\xe5\x0c\xe7\xc6\x3b\x62\xbd\x79\x8f\xb6\x73\xc6\xaa\x49"
            "\xf5\x4b\xc7\x30\x1f\xb7\xbd\xdc\x6e\xdc\x51\xb7\xe0\xd0\xb4\xde"
            "\xc9\xf8\x08\x51\xff\xf0\x2a\x33\x67\x1a\xd9\xa4\x06\xbb\xab\xe5",
        "\x01\x53\xeb\x2b\xe0\x54\x38\xe5\xc1\xef\xfb\x41\xb4\x13\xef\xc2"
            "\x84\x3b\x92\x7c\xbf\x19\xf0\xbc\x9c\xc1\x4b\x69\x3e\xee\x26\x39"
            "\x4a\x0d\x88\x80\xdc\x94\x6a\x06\x65\x6b\xcd\x09\x87\x15\x44\xa5"
            "\xf1\x5c\x7a\x1f\xa6\x8e\x00\xcd\xc7\x28\xc7\xcf\xb9\xc4\x48\x03"
            "\x48\x67",
        "\x01\x43\xae\x8e\xec\xbc\xe8\xfc\xf6\xb1\x6e\x61\x59\xb2\x97\x0a"
            "\x9c\xeb\x32\xc1\x7c\x1d\x87\x8c\x09\x31\x73\x11\xb7\x51\x9e\xd5"
            "\xec\xe3\x37\x4e\x79\x29\xf3\x38\xdd\xd0\xec\x05\x22\xd8\x1f\x2f"
            "\xa4\xfa\x47\x03\x3e\xf0\xc0\x87\x2d\xc0\x49\xbb\x89\x23\x3e\xef"
            "\x9b\xc1",
        "\x00\xdd\x63\x39\x47\x44\x6d\x0d\x51\xa9\x6a\x01\x73\xc0\x11\x25"
            "\x85\x8a\xbb\x2b\xec\xe6\x70\xaf\x92\x2a\x92\xde\xdc\xec\x06\x71"
            "\x36\xc1\xfa\x92\xe5\xfa\x73\xd7\x11\x6a\xc9\xc1\xa4\x2b\x9c\xb6"
            "\x42\xe4\xac\x19\x31\x0b\x04\x9e\x48\xc5\x30\x11\xff\xc6\xe7\x46"
            "\x1c\x36",
        "\x00\xef\xbd\xc6\xa4\x14\xbb\x8d\x66\x3b\xb5\xcd\xb7\xc5\x86\xbc"
            "\xcf\xe7\x58\x90\x49\x07\x6f\x98\xce\xe8\x2c\xdb\x5d\x20\x3f\xdd"
            "\xb2\xe0\xff\xb7\x79\x54\x95\x9d\xfa\x5e\xd0\xde\x85\x0e\x42\xa8"
            "\x6f\x5a\x63\xc5\xa6\x59\x2e\x9b\x9b\x8b\xd1\xb4\x05\x57\xb9\xcd"
            "\x0c\xc0",
    },
    { NISTP521, false,
        "\x76\x79\xea\xaf\x04\x95\x72\x5f\xa9\x9c\x51\xa2\xdd\x0c\x35\xc8"
            "\x88\x2b\x84\x0e\x1c\x23\x40\xba\x79\x30\x13\xb1\xe2\x56\x74\x71"
            "\xcb\xa3\x5c\x0d\xd6\x24\x7c\xc2\xc2\xca\x14\xf6\x55\x69\x12\xa5"
            "\x68\x70\x23\xfb\x2f\x0e\xe0\x21\x14\x39\x3b\xed\x4c\x59\x87\x42",
        "\x01\x2a\x59\x3f\x56\x8c\xa2\x57\x1e\x54\x3e\x00\x06\x6e\xcd\x3a"
            "\x32\x72\xa5\x7e\x1c\x94\xfe\x31\x1e\x5d\xf9\x6a\xfc\x1b\x79\x2e"
            "\x58\x62\x72\x0f\xc7\x30\xe6\x20\x52\xbb\xf3\xe1\x18\xd3\xa0\x78"
            "\xf0\x14\x4f\xc0\x0c\x9d\x8b\xaa\xaa\x82\x98\xff\x63\x98\x1d\x09"
            "\xd9\x11",
        "\x01\x7c\xea\x5a\xe7\x5a\x74\x10\x0e\xe0\x3c\xdf\x24\x68\x39\x3e"
            "\xef\x55\xdd\xab\xfe\x8f\xd5\x71\x8e\x88\x90\x3e\xb9\xfd\x24\x1e"
            "\x8c\xbf\x9c\x68\xae\x16\xf4\xa1\xdb\x26\xc6\x35\x2a\xfc\xb1\x89"
            "\x4a\x98\x12\xda\x6d\x32\xcb\x86\x20\x21\xc8\x6c\xd8\xaa\x48\x3a"
            "\xfc\x26",
        "\x01\xaa\xc7\x69\x2b\xaf\x3a\xa9\x4a\x97\x90\x73\x07\x01\x08\x95"
            "\xef\xc1\x33\x7c\xdd\x68\x6f\x9e\xf2\xfd\x84\x04\x79\x6a\x74\x70"
            "\x1e\x55\xb0\x3c\xee\xf4\x1f\x3e\x6f\x50\xa0\xee\xea\x11\x86\x9c"
            "\x47\x89\xa3\xe8\xab\x5b\x77\x32\x49\x61\xd0\x81\xe1\xa3\x37\x7c"
            "\xcc\x91",
        "\x00\x09\xc1\xe7\xd9\x3d\x05\x6b\x5a\x97\x75\x94\x58\xd5\x8c\x49"
            "\x13\x4a\x45\x07\x18\x54\xb8\xa6\xb8\x27\x2f\x9f\xe7\xe7\x8e\x1f"
            "\x3d\x80\x97\xe8\xa6\xe7\x31\xf7\xab\x48\x51\xeb\x26\xd5\xaa\x4f"
            "\xda\xdb\xa6\x29\x6d\xc7\xaf\x83\x5f\xe3\xd1\xb6\xdb\xa4\xb0\x31"
            "\xd5\xf3",
    },
};

/*******************************************************************************
 * Exported variables
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PUFS_TV_ECDSAV_H */

/*** end of file ***/
