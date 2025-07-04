/**
 * @file pufs_tv_ecpuk.h
 * @brief test vectors for ECP
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

#ifndef PUFS_TV_ECPUK_H
#define PUFS_TV_ECPUK_H

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
static const struct ecpuk_test_pattern
{
    pufs_ec_name_t name;
    const void* prk;
    const void* Qx;
    const void* Qy;
} ecpuk_tp[] =
{
    { NISTB163,
        "\x00\x00\x00\x01\x87\xc0\xc5\x88\xfb\xdc\xf9\x4a\x53\xb4\x51\x6d"
            "\x62\xd8\x98\xd0\x20",
        "\x01\xe1\x6c\x18\xb8\x00\xd4\xd5\x5c\xea\x0e\x77\x22\x46\x73\x11"
            "\xd8\x98\xe4\x65\x4e",
        "\x06\x9e\xc2\xe9\x5a\x75\xa5\x9f\x01\x41\x4c\xed\x5c\xeb\x72\xb8"
            "\xf3\x47\xa9\x62\x09",
    },
    { NISTB233,
        "\x00\x00\x00\xd4\x67\xbb\x20\x60\xec\xb1\x05\x29\x68\x49\x47\xe1"
            "\xe9\x5a\x74\x03\x48\xab\x21\xc3\x8a\xfa\xf6\xe4\x64\xa2",
        "\x00\xb8\x93\x2e\xe9\x9a\xdf\x72\xb6\x20\xe1\xa6\x77\x9c\x9d\xc5"
            "\xf9\xad\x38\xbd\x3f\x64\x75\xca\xe8\x4e\x20\x9c\x70\x4e",
        "\x01\x04\x05\x1c\xed\x12\x59\xd5\x08\x5f\x2e\x1c\x8f\x5c\xa5\x39"
            "\x4e\x09\x4c\x33\xe7\x57\xa6\x3c\xe4\x3d\x1c\x06\xd3\xec",
    },
    { NISTB283,
        "\x00\x31\xca\x77\xb8\x9d\xf0\xc2\xf9\x0a\xe6\xd3\x8c\x93\x5a\x75"
            "\xeb\xd7\x4d\x64\xf8\x26\xcf\xe6\xd2\x3d\x13\x5e\x28\xbb\x66\xa0"
            "\xba\xea\xfb\xa9",
        "\x03\x23\x0e\x12\x93\x9f\x6d\x02\x0d\xbf\xe0\x77\x26\x7a\x40\xe8"
            "\xe3\xc4\x21\x9c\x5d\x3b\x1f\x04\x25\x89\x97\x66\xa5\x97\x0c\x90"
            "\x95\x06\x83\x85",
        "\x01\xc5\x57\xb8\x84\xef\x70\x03\xfd\x84\x69\x2b\x6b\x69\xcd\xa8"
            "\x5f\xf9\xb5\x86\xd5\xfd\x6a\xe2\x3a\xc9\x42\x1b\xb3\xb1\xb5\x70"
            "\x26\x5c\xa7\x7a",
    },
    { NISTB409,
        "\x00\x83\xd2\x07\x00\x93\xf0\xcf\x1e\xec\xec\x5b\xee\x3d\xeb\x44"
            "\x03\xb2\xe4\x71\x49\xbe\x09\x55\x08\x8f\x95\xa7\x50\xd7\x7f\xd2"
            "\xe5\x93\xd3\x7b\xaf\xe5\x37\xea\x79\xa7\xad\x33\xf8\x0b\xc3\xda"
            "\xf3\xb4\x58\xd1",
        "\x01\x2a\x05\xf6\xe3\x18\x4e\x97\x24\x00\xf2\x6f\xfa\xad\x2b\xee"
            "\x81\x3f\x4f\x0e\x9a\x7e\x76\xbc\x21\x65\xa1\x07\xc9\x77\x0c\x67"
            "\xfa\x14\x2f\xe5\x06\xb9\x39\xc7\x94\x06\xaa\x99\xde\x29\x1b\x65"
            "\x7f\x39\xe8\xa9",
        "\x00\xe2\x6d\x28\xe6\xa0\x56\xe9\x4e\xd6\xc0\x12\x60\x69\x5f\xaf"
            "\xe4\x4d\x44\xcd\xf3\x4f\xb4\x09\x58\xdc\xa1\x80\xd1\x5f\x3e\x67"
            "\xb6\xda\x1d\x4b\x2b\x0e\xc9\xfa\xa0\xdc\x3c\x6d\x60\x8f\x27\xb2"
            "\x5c\x2b\x08\x7a",
    },
    { NISTB571,
        "\x01\x60\x1d\x26\x08\xcc\xba\xc5\x9e\x9f\x16\xeb\x12\x3f\x98\x8c"
            "\xeb\x1e\xee\x23\x9c\xa0\x38\x80\xd1\xdc\x25\x44\x6d\x84\xae\x67"
            "\xff\xc4\xe0\xa7\xea\xf2\x6c\x33\xbf\x3d\x6d\x2b\x27\x91\x3e\x5e"
            "\x33\x7f\x58\xe4\x8b\x5e\xf2\xab\xa6\xbb\x5b\xeb\x42\x1d\x25\x58"
            "\x9e\xce\x18\xa6\x41\x2c\x0d\x50",
        "\x05\xac\xc7\x28\x79\x8f\xfb\x2d\xea\x9f\x3b\x34\x5c\x7e\x4d\x86"
            "\xb9\xd0\x53\x00\xd2\xc5\x9d\x2d\x15\xef\x99\xad\x0b\xc9\x87\x32"
            "\x89\x74\xc4\xe1\x78\x00\x74\x2d\x98\xc7\x8b\x2b\x70\x10\x93\xba"
            "\xed\xe6\x86\x33\x93\x4b\xaa\xa8\x7e\xb2\xb0\xa0\xe0\x10\x8b\x0f"
            "\x48\x3e\xd1\x87\xa0\xc7\x25\xd3",
        "\x00\x62\xe9\x27\xac\x17\xbf\xd2\x42\xdc\xac\x4a\xfa\xb0\xd2\x0c"
            "\x65\xba\xde\xf4\x64\x68\xf2\x02\x11\xad\xf5\x4e\xc4\xd2\x14\x5a"
            "\x85\x1d\x11\x21\xd4\xcb\xf0\x13\x10\x9e\x06\xc5\xaf\x21\x5c\xdb"
            "\x3d\x38\xa8\x5f\xf9\x2f\x6d\x54\x6b\x4a\xa8\x17\xa4\xe5\x80\x34"
            "\x9a\xea\xc0\xca\x74\x41\x25\x88",
    },
    { NISTK163,
        "\x02\x8a\x74\x47\xf9\x5b\x43\xc0\x72\x72\x2e\xe5\x2f\x2a\x68\x89"
            "\x75\x18\x83\x02\x72",
        "\x07\x2d\xad\xf2\x4b\x00\xf9\xa2\xa0\xad\x6f\xbf\xb9\xd8\x61\x81"
            "\xe9\x39\x90\x01\x74",
        "\x04\xbc\x1d\x49\x87\xdd\xe0\xd2\xf6\x33\xdf\x16\xd6\x86\xe2\xa7"
            "\x8d\x6d\x3f\x49\xf3",
    },
    { NISTK233,
        "\x00\x01\xda\x74\x22\xb5\x0e\x3f\xf0\x51\xf2\xaa\xae\xd1\x0a\xce"
            "\xa6\xcb\xf6\x11\x0c\x51\x7d\xa2\xf4\xea\xca\x8b\x5b\x87",
        "\x01\xc7\x47\x5d\xa9\xa1\x61\xe4\xb3\xf7\xd6\xb0\x86\x49\x40\x63"
            "\x54\x3a\x97\x9e\x34\xb8\xd7\xac\x44\x20\x4d\x47\xbf\x9f",
        "\x01\x31\xcb\xd4\x33\xf1\x12\x87\x1c\xc1\x75\x94\x39\x91\xb6\xa1"
            "\x35\x0b\xf0\xcd\xd5\x7e\xd8\xc8\x31\xa2\xa7\x71\x0c\x92",
    },
    { NISTK283,
        "\x01\xde\x6f\xc5\x61\xce\x8c\x3e\xc9\xa7\xc0\x3a\x51\xe0\xc6\x12"
            "\x04\x99\x1f\x8c\xac\xa8\xc7\xb0\x73\xcd\x07\x94\x5f\xfb\x22\xc4"
            "\x8c\x30\xe5\xd4",
        "\x02\x1e\x41\x03\x35\x85\x94\x9f\x5b\xf3\x0a\x73\xd9\x35\xc5\x80"
            "\x94\x6c\x3f\x15\xb9\x42\xb4\x2b\x54\xe3\x39\x7f\xc4\x11\x5e\xe9"
            "\x6b\xbb\xcf\xf0",
        "\x05\x07\x89\xe0\xc1\xda\xca\xeb\xb7\x2d\x7f\xe2\x70\x81\xb2\x04"
            "\x8a\x8f\xac\x3a\x58\x69\x3e\x52\x80\x7b\x8c\x34\x69\x30\xb5\xc4"
            "\xde\xb5\x49\xcb",
    },
    { NISTK409,
        "\x00\x01\x90\xc5\xa0\x03\x74\xcc\x32\x54\xfd\xd4\x21\xc8\xe5\x2b"
            "\x0c\xb0\xf0\x03\x17\xbb\xfb\x41\x53\x19\x5e\xb6\x19\x55\x57\x98"
            "\x9b\x8e\x78\xb2\x7d\xf3\x5c\x8f\x47\xbb\x4b\x4e\xe4\x60\x8e\xa0"
            "\x4f\x2a\xdb\x72",
        "\x00\x41\x5d\x29\x6d\x3d\x42\x18\x01\xdd\x4e\xf8\x70\xcd\xd2\x34"
            "\x22\x0a\xf5\x2c\x89\x6f\x2d\x8e\x70\xc3\x68\x62\x21\x67\x65\x5d"
            "\x45\xab\x7d\xb5\x24\x55\x2f\x7a\xeb\x9c\x11\x59\xbc\xac\x10\xf2"
            "\x4b\x9b\x18\x64",
        "\x00\x0f\x82\x4d\x69\xec\x62\x9e\x2d\xab\xd3\x23\xcf\xc9\x39\x92"
            "\xf2\x53\xc9\x01\xad\xa1\x42\x79\x67\xe5\x91\xca\x0e\x09\x70\xae"
            "\x7e\xd3\x5e\x25\x21\x59\x25\x5a\x3b\xdb\xf2\x1d\x09\xb0\xc7\xbf"
            "\xeb\x72\x62\x6a",
    },
    { NISTK571,
        "\x00\x4b\x72\x23\x99\x4f\x77\x70\x8d\xbe\xfe\x1e\x76\xfe\xdb\x62"
            "\x79\x71\x0b\x87\x69\x93\x3f\x87\xd1\x2d\x43\x04\xba\xc6\x46\xfc"
            "\x45\x30\x55\x63\x2b\xeb\x70\xf8\x7c\x6b\xcf\x6f\x28\xfc\xcc\xba"
            "\x25\x08\x87\x89\xd1\xf1\x50\x13\xf2\x53\x20\xff\x09\x32\x1e\x92"
            "\x1e\xb3\xe6\x6b\x08\x29\xe8\x7c",
        "\x02\x36\x91\xa3\x02\x8f\xc2\xea\x92\xf7\x07\xf1\x3c\x61\x95\x3e"
            "\xbf\x41\x1a\x24\x77\x39\xf2\x25\xf2\x18\x78\xfa\x78\x6e\x41\x6c"
            "\x5a\xac\x32\xa5\xd7\x33\x68\xbf\x3c\xa3\x50\xf1\xe0\x50\x22\xd1"
            "\x70\x93\xdc\x31\x8b\x42\xe5\xfa\x72\x34\xe3\x2f\x95\x9f\x20\x14"
            "\x6d\xa2\x16\x5d\xb3\x62\x30\xc0",
        "\x00\xfd\x26\x35\x48\x5e\x32\xd6\x37\xbf\xd8\xf5\x3f\xf6\x00\xb9"
            "\xb2\xbc\xc6\xd7\x98\x84\xbe\x54\xdc\x50\x10\x3e\x25\xc4\x60\xd4"
            "\x1c\x8d\x50\x2d\x79\x27\xbb\x19\xad\xfb\x2c\xd5\x9a\x83\xec\x92"
            "\xf4\x18\x6a\xc5\xc7\x50\x14\xd3\x94\x6f\x4a\x2a\x72\x5d\x33\x24"
            "\xf6\xdc\x20\x61\x97\xd1\x9d\x79",
    },
    { NISTP192,
        "\x00\x17\x89\x99\x49\xd0\x2b\x55\xf9\x55\x68\x46\x41\x1c\xc9\xde"
            "\x51\x2c\x6f\x16\xec\xde\xb1\xc4",
        "\x14\xf6\x97\x38\x59\x96\x89\xf5\x70\x6a\xb7\x13\x43\xbe\xcc\x88"
            "\x6e\xf1\x56\x9a\x2d\x11\x37\xfe",
        "\x0c\xf5\xa4\x33\x90\x9e\x33\x21\x7f\xb4\xdf\x6b\x95\x93\xf7\x1d"
            "\x43\xfb\x1c\x2a\x56\x53\xb7\x63",
    },
    { NISTP224,
        "\x00\x69\x75\xfe\xbc\x7b\x44\x2d\xab\x8f\xd2\x83\xec\x2e\x4b\xdd"
            "\x0e\x00\x09\x52\x5b\x1c\xe0\x71\xf9\xab\x59\x56",
        "\x56\xfb\x65\x38\xf1\x72\x3d\x2b\xef\x3c\x76\x41\x34\x32\x0b\x44"
            "\xba\x61\x5f\x66\x3d\xb8\x04\xe5\x40\x50\xb9\x5a",
        "\x95\x14\xa4\x42\xeb\x66\xdb\xf2\xb4\x50\x74\x6f\x66\xd5\x41\x01"
            "\x87\x7a\x50\xd4\xbc\x29\x10\xc6\x1d\x00\x5a\xdd",
    },
    { NISTP256,
        "\x00\x2a\x10\xb1\xb5\xb9\xfa\x0b\x78\xd3\x8e\xd2\x9c\xd9\xce\xc1"
            "\x85\x20\xe0\xfe\x93\x02\x3e\x35\x50\xbb\x71\x63\xab\x49\x05\xc6",
        "\xe9\xcd\x2e\x8f\x15\xbd\x90\xcb\x07\x07\xe0\x5e\xd3\xb6\x01\xaa"
            "\xce\x7e\xf5\x71\x42\xa6\x46\x61\xea\x1d\xd7\x19\x9e\xbb\xa9\xac",
        "\xc9\x6b\x01\x15\xbe\xd1\xc1\x34\xb6\x8f\x89\x58\x4b\x04\x0a\x19"
            "\x4b\xfa\xd9\x4a\x40\x4f\xdb\x37\xad\xad\x10\x7d\x5a\x0b\x4c\x5e",
    },
    { NISTP384,
        "\x00\x82\x75\x7d\x5b\x9d\xb0\x84\xbd\x2e\x09\x21\xa6\xce\x62\x10"
            "\x76\xf3\xf4\x3a\x52\x35\x65\xa7\x67\x10\xc9\xa1\xdc\xc7\xc5\xf4"
            "\xb1\xa4\x23\x7f\x24\xc5\x36\x14\x15\x3e\xd9\x7b\x42\x3a\x27\x77",
        "\x32\x71\x93\x8d\x4c\xd1\x44\x00\x6b\x45\xc7\x3f\x2a\x89\x30\xdf"
            "\xf8\x23\x8e\x72\x20\x49\x60\x00\xbe\x36\x94\xaa\x54\x18\x06\x32"
            "\x71\xc2\xe1\x29\x12\xb6\xc1\xcb\xc0\x3c\x41\x75\x37\x3c\x3a\x3e",
        "\xd3\x4f\x52\x57\xd2\x30\xa4\x64\x95\x8d\x1f\xff\x57\x0a\x2f\x55"
            "\xfe\x64\x01\xc8\x3a\x61\x3a\x53\x38\x9a\x82\x95\x7e\xaa\x63\x9e"
            "\x38\x98\xcf\xe9\x82\x3d\x57\xb3\x8b\xae\xe3\x0f\x5c\x94\xf7\xe1",
    },
    { NISTP521,
        "\x00\x3d\x72\xbc\xc7\x00\x69\x5d\x40\x0c\x81\x3d\xfd\xa1\xe3\xb3"
            "\xbf\x2c\xbc\x15\xf2\x8d\x27\x2d\xcb\x3b\xd7\xf9\x35\xfd\xab\xac"
            "\x9c\x27\x7d\xc4\x7a\x32\x45\x73\x7b\x40\xde\xd8\xd0\xd2\x46\x4c"
            "\xaa\x6a\xfa\x8b\x97\x16\x93\xcb\x25\x8a\x8f\x58\xe6\x3f\xab\x2c"
            "\x48\x56",
        "\x00\xf1\xdc\x7c\xcb\x09\xd6\x1e\x6a\xf3\x79\xb8\x9a\xca\x90\x5b"
            "\x49\x77\x9f\xbe\x43\xa9\x4c\x8e\xf3\x84\xcc\xbf\x66\x0f\x48\x05"
            "\xc9\x65\xa3\xa2\x4e\xd5\xa9\x62\xc2\x48\x09\x41\x5c\xde\xcf\xdf"
            "\xe5\x0f\xd1\x8f\x12\x66\x07\x31\x54\xb6\x2f\x35\x5f\xe4\xc9\x8a"
            "\xf6\xe5",
        "\x01\x74\x0e\xb9\x5b\x8e\x31\xa0\x43\x4c\x98\x8f\x2e\xdd\x55\x0b"
            "\x8d\xc6\xc4\x5c\x6f\x50\x43\x09\x25\x53\x70\xcc\xe5\x7e\x82\x1f"
            "\xcb\x4f\x60\xba\xd1\x7a\x8f\xb9\xa3\xf4\xdc\x67\xed\x48\x60\xae"
            "\x6d\xd3\xed\x4b\x1f\x51\xb9\x84\x51\xb7\xe7\x09\x5c\xc8\x7d\x4d"
            "\x62\x79",
    },
};

/*******************************************************************************
 * Exported variables
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PUFS_TV_ECPUK_H */

/*** end of file ***/
