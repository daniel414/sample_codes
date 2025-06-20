/**
 * @file usbd_interface.c
 * @brief An implementation of helper functions for USB device interface.
 * @copyright Copyright (c) 2023 ChipWon Technology. All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "usbd_interface.h"
#include "interrupt_manager.h"
#include "clock_tw9001.h"

/*******************************************************************************
 * Private macros, defines, enumerations, structures(with typedefs)
 ******************************************************************************/
#define MEMBER_SIZE(type, member)   sizeof(((type *)0u)->member)
#define MEMBER_OFFSET(type, member) ((uint32_t) & (((type *)0)->member))
#define LE16_TO_BE(val)             ((uint16_t)(((val) >> 8u) | ((val) << 8u)))
#define LE32_TO_BE(val)                                                                      \
    ((uint32_t)(((val) >> 24u) | (((val)&0x00FF0000u) >> 8u) | (((val)&0x0000FF00u) << 8u) | \
                ((val) << 24u)))
#define BE16_TO_LE(val) ((uint16_t)(((val) >> 8u) | ((val) << 8u)))
#define BE32_TO_LE(val)                                                                      \
    ((uint32_t)(((val) >> 24u) | (((val)&0x00FF0000u) >> 8u) | (((val)&0x0000FF00u) << 8u) | \
                ((val) << 24u)))

/* HID Class-Specific Descriptor type */
#define USBD_DESC_CS_HID      (0x21u)
#define USBD_DESC_CS_REPORT   (0x22u)
#define USBD_DESC_CS_PHY_DESC (0x23u)

/* Video Class-Specific Descriptor type */
#define USBD_DESC_CS_UNDEFINED     (0x20u)
#define USBD_DESC_CS_DEVICE        (0x21u)
#define USBD_DESC_CS_CONFIGURATION (0x22u)
#define USBD_DESC_CS_STRING        (0x23u)
#define USBD_DESC_CS_INTERFACE     (0x24u)
#define USBD_DESC_CS_ENDPOINT      (0x25u)

/* Communications Class-Specific Descriptor type */
// #define USBD_DESC_CS_INTERFACE (0x24u)
// #define USBD_DESC_CS_ENDPOINT  (0x25u)

/******  Generic Interface Function Class Code, Device  ******/
#define USBD_IFFCC_FROM_IF_DESC (0x00u) // Generic Interface, Class Code from Interface Descriptors
#define USBD_IFFSC_SUBCLASS_DEV (0x00u) // Common Class Sub Class
#define USBD_IFFPC_PROTOCOL_DEV (0x00u) // Interface Association Descriptor Protocol
/******  Generic Interface Function Class Code, Device  ******/

/******  Multi-Interface Function Class Code  ******/
#define USBD_MIFFCC_MISC         (0xEFu) // Miscellaneous Class
#define USBD_MIFFSC_SUBCLASS_IAD (0x02u) // Miscellaneous Class, IAD
#define USBD_MIFFPC_PROTOCOL_IAD (0x01u) // Interface Association Descriptor Protocol
/******  Multi-Interface Function Class Code  ******/

/******  Generic Subclass Codes  ******/
#define USBD_SC_UNDEFINED   (0x00u) // XXh
#define USBD_SC_UNSPECIFIED (0x00u)
/******  Generic Subclass Codes  ******/

/******  Generic Protocol Codes  ******/
#define USBD_PC_UNDEFINED   (0x00u) // XXh
#define USBD_PC_UNSPECIFIED (0x00u)
/******  Generic Protocol Codes  ******/

/******  USB Interface Class Specifications  ******/
#define USBD_IC_COMM   USB_CLS_CDC  // Communication Interface Class (CDC)
#define USBD_IC_HID    USB_CLS_HID  // Human Interface Device Class (HID)
#define USBD_IC_MSC    USB_CLS_MSC  // Mass Storage Interface Class (MSC)
#define USBD_IC_CCID   USB_CLS_CCID // Smart Card / Chip Card Interface Device Class (CCID)
#define USBD_IC_DATA   (0x0Au)      // Data Interface Class (CDC)
#define USBD_IC_VENDOR (0xFFu)      // Vendor Interface Class
/******  USB Class Specification  ******/

/******  HID Subclass Codes  ******/
#define USBD_SC_BOOT (0x01u)
/******  HID Subclass Codes  ******/

/******  Human Interface Device Class Protocol Codes  ******/
#define USBD_PC_KEYBOARD (0x01u)
#define USBD_PC_MOUSE    (0x02u)
/******  Human Interface Device Class Protocol Codes  ******/

/******  CDC Subclass Codes  ******/
#define USBD_SC_ACM   (0x02u) // Abstract Control Model
#define USBD_SC_ECM   (0x06u) // Ethernet Networking Control Model
#define USBD_SC_ATMCM (0x07u) // ATM Networking Control Model
#define USBD_SC_WHCM  (0x08u) // Wireless Handset Control Model
#define USBD_SC_EEM   (0x0Cu) // Ethernet Emulation Model
/******  CDC Subclass Codes  ******/

/******  Communications Interface Class Protocol Codes  ******/
#define USBD_PC_AT_CMD_V250    (0x01u) // AT Commands: V.250 etc
#define USBD_PC_AT_CMD_PCCA101 (0x02u) // AT Commands defined by PCCA-101
#define USBD_PC_AT_CMD_GSM     (0x04u) // AT Commands defined by GSM 07.07
#define USBD_PC_AT_CMD_3GPP    (0x04u) // AT Commands defined by 3GPP 27.007
/******  Communications Interface Class Protocol Codes  ******/

/******  SubType in Communications Class Functional Descriptors  ******/
#define USBD_COMM_FD_HEADER    (0x00u) // Header Functional Descriptor
#define USBD_COMM_FD_CALL_MGMT (0x01u) // Call Management Functional Descriptor
#define USBD_COMM_FD_ACM       (0x02u) // Abstract Control Management Functional Descriptor
#define USBD_COMM_FD_UNION     (0x06u) // Union Functional Descriptor
#define USBD_COMM_FD_ETHERNET  (0x0Fu) // Ethernet Networking Functional Descriptor
/******  SubType in Communications Class Functional Descriptors  ******/

/* USB Configuration */
/* DEVICE DESCRIPTOR */
#define USBD_STD_SPEC_REV_BCD   (0x0200u)
#define USBD_HID_SPEC_REV_BCD   (0x0111u)
#define USBD_CCID_SPEC_REV_BCD  (0x0110u)
#define USBD_CDC_SPEC_REV_BCD   (0x0110u)
#define USBD_DEVICE_CLASS       USBD_IFFCC_FROM_IF_DESC
#define USBD_DEVICE_SUB_CLASS   USBD_IFFSC_SUBCLASS_DEV
#define USBD_DEVICE_PROTOCOL    USBD_IFFPC_PROTOCOL_DEV
#define USBD_CTRL_EP0_PKT_LEN   (0x40u)
#define USBD_VENDOR_ID          (0xFF08u) /* ChipWon vendor ID = FF08h(Fake) */
#define USBD_PRODUCT_ID         (0x9001u) /* Product ID: 9001h = project code */
#define USBD_RELEASE_NUMBER_BCD (0x0099u) /* Not formal release yet */
// #define USBD_RELEASE_NUMBER_BCD (0x0100u)
#define USBD_STR_MFR_INDEX    USB_STR_IDX001
#define USBD_STR_PROD_INDEX   USB_STR_IDX002
#define USBD_STR_SN_INDEX     USB_STR_IDX003
#define USBD_NB_CONFIGURATION (1u)

/* CONFIGURATION DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _TWO_DISTINCT_FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_NB_INTERFACE (2u)
#elif (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_)            /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_NB_INTERFACE (3u)
#elif (_USB_DEVICE_FUNCTIONLITY_ == _CDC_ETHERNET_FACILITY_)       /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_NB_INTERFACE (2u)
#else                                                              /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_NB_INTERFACE (1u)
#endif                                                             /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_CONF_VAL   (1u)
#define USBD_CONF_INDEX (0u)
#if 0
#define USBD_CONF_ATTRIB                 \
    (0x80u | USB_CFG_ATTR_SELF_POWERED | \
     USB_CFG_ATTR_REMOTE_WAKEUP)                              /**< Self powered, remote wakeup. */
#define USBD_CONF_ATTRIB (0x80u | USB_CFG_ATTR_SELF_POWERED)  /**< Self powered. */
#endif
#if (_USB_DEVICE_FUNCTIONLITY_ == _CDC_ETHERNET_FACILITY_)    /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_CONF_ATTRIB (0x80u | USB_CFG_ATTR_REMOTE_WAKEUP) /**< Remote wakeup. */
#else                                                         /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_CONF_ATTRIB (0x80u)                              /**< Bus powered only */
#endif                                                        /* _USB_DEVICE_FUNCTIONLITY_ */
#if 0
#define USBD_MAX_POWER (0xFAu)                                /**< FAh(250) = 500 mA */
#endif
#define USBD_MAX_POWER (0x32u)                                /**< 32h(50) = 100 mA */

/* INTERFACE OTHER SPEED DESCRIPTOR (HID) */
#define USBD_HID_OSPD_NB_EP (0u)

/* INTERFACE 0 DESCRIPTOR (HID) */
#define USBD_HID_IF0_NB    (0u)
#define USBD_HID_IF0_ALT   (0u)
#define USBD_HID_IF0_NB_EP (1u)
#if (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_HID_CPL_FIDO_IF0_NB (1u)
#else                                                 /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_HID_CPL_FIDO_IF0_NB (0u)
#endif                                                /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_HID_CPL_FIDO_IF0_NB_EP (2u)
#define USBD_HID_IF0_CLASS          USBD_IC_HID       /* HID */
#if (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_HID_IF0_SUB_CLASS USBD_SC_BOOT           /* boot interface */
#define USBD_HID_IF0_PC        USBD_PC_KEYBOARD
#else                                                 /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_HID_IF0_SUB_CLASS USBD_SC_BOOT           /* boot interface */
#define USBD_HID_IF0_PC        USBD_PC_MOUSE
#endif                                                /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_HID_CPL_FIDO_IF0_SUB_CLASS USBD_SC_UNSPECIFIED
#define USBD_HID_CPL_FIDO_IF0_PC        USBD_PC_UNSPECIFIED

/* INTERFACE 1 DESCRIPTOR (HID) */
#define USBD_HID_CPL_FIDO_IF1_NB        (1u)
#define USBD_HID_CPL_FIDO_IF1_ALT       (0u)
#define USBD_HID_CPL_FIDO_IF1_NB_EP     (2u)
#define USBD_HID_IF1_CLASS              USBD_IC_HID  /* HID */
#define USBD_HID_IF1_SUB_CLASS          USBD_SC_BOOT /* boot interface */
#define USBD_HID_IF1_PC                 USBD_PC_MOUSE
#define USBD_HID_CPL_FIDO_IF1_SUB_CLASS USBD_SC_UNSPECIFIED
#define USBD_HID_CPL_FIDO_IF1_PC        USBD_PC_UNSPECIFIED

/* HID DESCRIPTOR */
#define USBD_HIDDESC_NB_DESC (1u)

/* INTERFACE 0 DESCRIPTOR (MSC) */
#define USBD_MSC_IF0_NB         (0u)
#define USBD_MSC_IF0_ALT0       (0u)
#define USBD_MSC_IF0_ALT0_NB_EP (2u)
#define USBD_MSC_OSPD_NB_EP     (0u)
#define USBD_MSC_IF0_CLASS      USBD_IC_MSC /* MSC */
#define USBD_MSC_IF0_SUB_CLASS  (0x06u)     /* 06h at usual. USBD_SC_UNDEFINED */
#define USBD_MSC_IF0_PC         (0x02u)     /* MSCPC_UNDEFINED */
// #define IF0_ALT0_INDEX         					FUNCTION_INDEX /* must be
// the same as FUNCTION_INDEX */

/* INTERFACE 0 DESCRIPTOR (CDC) */
#define USBD_CDC_IF0_NB         (0u)
#define USBD_CDC_IF0_ALT0       (0u)
#define USBD_CDC_IF0_ALT0_NB_EP (1u)
#define USBD_CDC_IF0_CLASS      USBD_IC_COMM /* CDC */
#define USBD_CDC_IF0_SUB_CLASS  USBD_SC_ECM  /* ECM */
#define USBD_CDC_IF0_PC         USBD_PC_UNSPECIFIED

/* INTERFACE 1 DESCRIPTOR (CDC-Data) */
#define USBD_CDC_IF1_NB         (1u)
#define USBD_CDC_IF1_ALT0       (0u)
#define USBD_CDC_IF1_ALT1       (1u)
#define USBD_CDC_IF1_ALT0_NB_EP (0u)
#define USBD_CDC_IF1_ALT1_NB_EP (1u)
#define USBD_CDC_IF1_CLASS      USBD_IC_DATA /* CDC-Data */
#define USBD_CDC_IF1_SUB_CLASS  USBD_SC_UNSPECIFIED
#define USBD_CDC_IF1_PC         USBD_PC_UNSPECIFIED

/* HEADER FUNCTIONAL DESCRIPTOR */

/* UNION FUNCTIONAL DESCRIPTOR */
#define USBD_CICUF_NB_IF \
    (0u) // Zero based index of the interface in this configuration (bInterfaceNum)
#define USBD_CICUF_FIRST_IF_IDX (1u) // First index

/* ETHERNET NETWORKING FUNCTIONAL DESCRIPTOR */
#define USBD_CICEN_MAC USBD_STR_SN_INDEX // Index of string descriptor, holds ethernet MAC address
// #define USBD_CICEN_ETHER_STAT   						(0x00000000u) //
// Ethernet statistics functions
#define USBD_CICEN_ETHER_STAT    (0x00000003u) // Ethernet statistics functions
#define USBD_CICEN_MAX_SEG_SIZE  (1514u)       // This is typically 1514 bytes (802.3)
#define USBD_CICEN_NB_MFC_FILTER (0x0000u)     // The number of multicast filters
#define USBD_CICEN_NB_PWR_FILTER \
    (0x00u) // The number of power filters that are available for causing wake-up of the host

/* INTERFACE 0 DESCRIPTOR (CCID) */
#if (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_CCID_IF0_NB (2u)
#else                                                 /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_CCID_IF0_NB (0u)
#endif                                                /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_CCID_IF0_ALT0       (0u)
#define USBD_CCID_IF0_ALT0_NB_EP (3u)
#define USBD_CCID_OSPD_NB_EP     (0u)
#define USBD_CCID_IF0_CLASS      USBD_IC_CCID /* CCID */
#define USBD_CCID_IF0_SUB_CLASS  USBD_SC_UNDEFINED
#define USBD_CCID_IF0_PC         USBD_PC_UNDEFINED

#if 0
/* ENDPOINT NUMBER */
#define USBD_EP_NB0 (0u)
#define USBD_EP_NB1 (1u)
#define USBD_EP_NB2 (2u)
#define USBD_EP_NB3 (3u)
#define USBD_EP_NB4 (4u)
#define USBD_EP_NB5 (5u)
#define USBD_EP_NB6 (6u)
#define USBD_EP_NB7 (7u)
#define USBD_EP_NB8 (8u)
#endif

/* ENDPOINT 0 DESCRIPTOR */
#define USBD_EP0_DO (USB_ENDPOINT_OUT | USBD_EP_NB0) // out
#define USBD_EP0_DI (USB_ENDPOINT_IN | USBD_EP_NB0)  // in

/* ENDPOINT 1 DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_)       /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP1_DI         (USB_ENDPOINT_IN | USBD_EP_NB1) // in
#define USBD_EP1_ATTRIBUTES (USB_EPT_INTERRUPT)             // interrupt
#define USBD_EP1_SIZE       (0x0008u)                       // 8u
#define USBD_EP1_INTERVAL   (0x0Au)
#else                                                       /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP1_DI         (USB_ENDPOINT_IN | USBD_EP_NB1) // in
#define USBD_EP1_ATTRIBUTES (USB_EPT_INTERRUPT)             // interrupt
#define USBD_EP1_SIZE       (0x0040u)                       // 64u
#define USBD_EP1_INTERVAL   (0x05u)
#endif                                                      /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 2 DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _TWO_DISTINCT_FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP2_DI         (USB_ENDPOINT_IN | USBD_EP_NB2)        // in
#define USBD_EP2_ATTRIBUTES (USB_EPT_INTERRUPT)                    // interrupt
#define USBD_EP2_SIZE       (0x0040u)                              // 64u
#define USBD_EP2_INTERVAL   (0x02u)
#elif (_USB_DEVICE_FUNCTIONLITY_ == \
       _MSC_BULK_ONLY_STORAGE_FACILITY_)                    /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP2_DI         (USB_ENDPOINT_IN | USBD_EP_NB2) // in
#define USBD_EP2_ATTRIBUTES (USB_EPT_BULK)                  // bulk
#define USBD_EP2_SIZE       (0x0040u)                       // 64u
#define USBD_EP2_INTERVAL   (0x00u)
#else                                                       /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP2_DI         (USB_ENDPOINT_IN | USBD_EP_NB2) // in
#define USBD_EP2_ATTRIBUTES (USB_EPT_INTERRUPT)             // interrupt
#define USBD_EP2_SIZE       (0x0040u)                       // 64u
#define USBD_EP2_INTERVAL   (0x05u)
#endif                                                      /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 3 DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_)        /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP3_DI         (USB_ENDPOINT_IN | USBD_EP_NB3)  // in
#define USBD_EP3_ATTRIBUTES (USB_EPT_INTERRUPT)              // interrupt
#define USBD_EP3_SIZE       (0x0008u)                        // 8u
#define USBD_EP3_INTERVAL   (0x20u)
#elif (_USB_DEVICE_FUNCTIONLITY_ == _CDC_ETHERNET_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP3_DI         (USB_ENDPOINT_IN | USBD_EP_NB3)  // in
#define USBD_EP3_ATTRIBUTES (USB_EPT_INTERRUPT)              // interrupt
#define USBD_EP3_SIZE       (0x0010u)                        // 16u
#define USBD_EP3_INTERVAL   (0x08u)
#else                                                        /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP3_DI         (USB_ENDPOINT_IN | USBD_EP_NB3)  // in
#define USBD_EP3_ATTRIBUTES (USB_EPT_INTERRUPT)              // interrupt
#define USBD_EP3_SIZE       (0x0040u)                        // 64u
#define USBD_EP3_INTERVAL   (0x05u)
#endif                                                       /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 4 DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _MSC_BULK_ONLY_STORAGE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ \
                                                                     */
#define USBD_EP4_DO         (USB_ENDPOINT_OUT | USBD_EP_NB4)        // out
#define USBD_EP4_ATTRIBUTES (USB_EPT_BULK)                          // bulk
#define USBD_EP4_SIZE       (0x0040u)                               // 64u
#define USBD_EP4_INTERVAL   (0x00u)
#else                                                               /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP4_DO         (USB_ENDPOINT_OUT | USBD_EP_NB4)        // out
#define USBD_EP4_ATTRIBUTES (USB_EPT_INTERRUPT)                     // interrupt
#define USBD_EP4_SIZE       (0x0040u)                               // 64u
#define USBD_EP4_INTERVAL   (0x05u)
#endif                                                              /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 5 DESCRIPTOR */
// #if (_USB_DEVICE_FUNCTIONLITY_ == _FACILITY_UNDEFINED_) /* _USB_DEVICE_FUNCTIONLITY_ */
// #else /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP5_DI         (USB_ENDPOINT_IN | USBD_EP_NB5) // in
#define USBD_EP5_ATTRIBUTES (USB_EPT_INTERRUPT)             // interrupt
#define USBD_EP5_SIZE       (0x0040u)                       // 64u
#define USBD_EP5_INTERVAL   (0x05u)
// #endif /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 6 DESCRIPTOR */
// #if (_USB_DEVICE_FUNCTIONLITY_ == _FACILITY_UNDEFINED_) /* _USB_DEVICE_FUNCTIONLITY_ */
// #else /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP6_DO         (USB_ENDPOINT_OUT | USBD_EP_NB6) // out
#define USBD_EP6_ATTRIBUTES (USB_EPT_INTERRUPT)              // interrupt
#define USBD_EP6_SIZE       (0x0040u)                        // 64u
#define USBD_EP6_INTERVAL   (0x05u)
// #endif /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 7 DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_)       /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP7_DI         (USB_ENDPOINT_IN | USBD_EP_NB7) // in
#define USBD_EP7_ATTRIBUTES (USB_EPT_BULK)                  // bulk
#define USBD_EP7_SIZE       (0x0040u)                       // 64u
#define USBD_EP7_INTERVAL   (0x00u)
#else                                                       /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP7_DI         (USB_ENDPOINT_IN | USBD_EP_NB7) // in
#define USBD_EP7_ATTRIBUTES (USB_EPT_INTERRUPT)             // interrupt
#define USBD_EP7_SIZE       (0x0040u)                       // 64u
#define USBD_EP7_INTERVAL   (0x05u)
#endif                                                      /* _USB_DEVICE_FUNCTIONLITY_ */

/* ENDPOINT 8 DESCRIPTOR */
#if (_USB_DEVICE_FUNCTIONLITY_ == _TWO_DISTINCT_FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP8_DO         (USB_ENDPOINT_OUT | USBD_EP_NB8)       // out
#define USBD_EP8_ATTRIBUTES (USB_EPT_INTERRUPT)                    // interrupt
#define USBD_EP8_SIZE       (0x0040u)                              // 64u
#define USBD_EP8_INTERVAL   (0x02u)
#elif (_USB_DEVICE_FUNCTIONLITY_ == _FIDO_SE_FACILITY_)            /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP8_DO         (USB_ENDPOINT_OUT | USBD_EP_NB8)       // out
#define USBD_EP8_ATTRIBUTES (USB_EPT_BULK)                         // bulk
#define USBD_EP8_SIZE       (0x0040u)                              // 64u
#define USBD_EP8_INTERVAL   (0x00u)
#else                                                              /* _USB_DEVICE_FUNCTIONLITY_ */
#define USBD_EP8_DO         (USB_ENDPOINT_OUT | USBD_EP_NB8)       // out
#define USBD_EP8_ATTRIBUTES (USB_EPT_INTERRUPT)                    // interrupt
#define USBD_EP8_SIZE       (0x0040u)                              // 64u
#define USBD_EP8_INTERVAL   (0x05u)
#endif                                                             /* _USB_DEVICE_FUNCTIONLITY_ */

/* OTHER SPEED CONFIGURATION DESCRIPTOR */
#define USBD_NB_IF_MSC_OSPD (1u)

/* STRING DESCRIPTOR */
#define USBD_LANGUAGE_ID (0x0409u)

/* "Chipwon" */
#define USBD_MANUFACTURER_NAME            \
    {                                     \
        'C', 'h', 'i', 'p', 'W', 'o', 'n' \
    }
#define USBD_MFR_LENGTH (7u)
#if 0
/* "Authentication, embedded Secure Elements (eSE)" */
#define USBD_PRODUCT_NAME                 \
    {                                     \
        'U', 'S', 'B', ' ', 'e', 'S', 'E' \
    }
#define USBD_PN_LENGTH (7u)
#endif
#if 0
/* "USB Flash Drive" */
#define USBD_PRODUCT_NAME                                                         \
    {                                                                             \
        'U', 'S', 'B', ' ', 'F', 'l', 'a', 's', 'h', ' ', 'D', 'r', 'i', 'v', 'e' \
    }
#define USBD_PN_LENGTH (15u)
#endif
#if 0
/* "USB 10/100 LAN" */
#define USBD_PRODUCT_NAME                                                    \
    {                                                                        \
        'U', 'S', 'B', ' ', '1', '0', '/', '1', '0', '0', ' ', 'L', 'A', 'N' \
    }
#define USBD_PN_LENGTH (14u)
#endif
#if 1
/* "USB HID" */
#define USBD_PRODUCT_NAME                 \
    {                                     \
        'U', 'S', 'B', ' ', 'H', 'I', 'D' \
    }
#define USBD_PN_LENGTH (7u)
#endif
/* "00E0600DC0DE" */
#define USBD_SERIAL_NUMBER                                         \
    {                                                              \
        '0', '0', 'E', '0', '6', '0', '0', 'D', 'C', '0', 'D', 'E' \
    }
#define USBD_SN_LENGTH (12u)
#if 0
/* "00F1600DC0DE" */
#define USBD_2ND_SERIAL_NUMBER                                     \
    {                                                              \
        '0', '0', 'F', '1', '6', '0', '0', 'D', 'C', '0', 'D', 'E' \
    }
#endif

// Vendor Unique Request Codes
#define USBD_REQ_REG_READ        (0x00u)
#define USBD_REQ_REG_WRITE       (0x01u)
#define USBD_REQ_EEPROM_STS_READ (0x02u)
#define USBD_REQ_EEPROM_READ     (0x03u)
#define USBD_REQ_EEPROM_WRITE    (0x04u)
#define USBD_REQ_IIC_READ        (0x05u)
#define USBD_REQ_IIC_WRITE       (0x06u)
#define USBD_REQ_SET_LED         (0x40u)

// HID
#define USBD_REQ_GET_REPORT   (0x01u)
#define USBD_REQ_GET_IDLE     (0x02u)
#define USBD_REQ_GET_PROTOCOL (0x03u)
#define USBD_REQ_SET_REPORT   (0x09u)
#define USBD_REQ_SET_IDLE     (0x0Au)
#define USBD_REQ_SET_PROTOCOL (0x0Bu)
// UVC
#define USBD_REQ_SYNCH_FRAME (0x0Cu)
// CDC
#define USBD_REQ_SEND_ENCAPSULATED_COMMAND  (0x00u)
#define USBD_REQ_GET_ENCAPSULATED_RESPONSE  (0x01u)
#define USBD_REQ_SET_ETHERNET_PACKET_FILTER (0x43u)
// MSC-BOT
#define USBD_REQ_BULK_ONLY_RESET (0xFFu)
#define USBD_REQ_GET_MAX_LUN     (0xFEu)

/** CDC connection status */
#define USBD_CSNTFY_NW_CONN         (0x00u)
#define USBD_CSNTFY_RESP_AVA        (0x01u)
#define USBD_CSNTFY_CONN_SPD_CHANGE (0x2Au)

#define USBD_NCONN_DISCONNECT (0u)
#define USBD_NCONN_CONNECT    (1u)

#define USBD_NCONN_BR_ZERO LE32_TO_BE(0u)
#define USBD_NCONN_BR_10M  LE32_TO_BE(10000000u)
#define USBD_NCONN_BR_100M LE32_TO_BE(100000000u)

/** Tunnel ready status */
#define USBD_TUNNEL_DEFAULT            (0x00u)
#define USBD_TUNNEL_USB_IF_EP_RDY      (1u << 0u)
#define USBD_TUNNEL_ETH_PKT_FILTER_RDY (1u << 1u)
#define USBD_TUNNEL_INT_RESP_RDY       (1u << 2u)
#define USBD_TUNNEL_RNDIC_RDY          (1u << 3u)

/*******************************************************************************
 * Private typedefs
 ******************************************************************************/
typedef enum
{
    USBD_DEFAULT_STATE    = 0x01u,
    USBD_ADDRESSED_STATE  = 0x02u,
    USBD_CONFIGURED_STATE = 0x03u,
} usbd_bus_configured_state_t;

typedef enum
{
    USBD_FEAT_BUS_POWERED   = (uint8_t)0x00u,
    USBD_FEAT_SELF_POWERED  = ((uint8_t)0x01u << 0u),
    USBD_FEAT_REMOTE_WAKEUP = ((uint8_t)0x01u << 1u),
} usbd_dev_feature_status_t;

/**
 * @brief A structure for a mechanism of job queue.
 */
typedef struct _usbd_job_queue_t
{
    uint8_t  ep_num;
    uint8_t *p_source_buffer;
    uint32_t source_buf_len;
} usbd_job_queue_t;

/**
 * @brief A structure type of state variables for USB device side.
 */
typedef struct _usbd_dev_if_state_t
{
    FSUSB_t                    *p_base;
    const IRQn_t               *p_irq_id;
    const clock_names_t        *p_clk_name;
    uint8_t                    *p_ep_shared_sram0_ptr;
    uint8_t                    *p_ep_shared_sram1_ptr;
#if 0
    uint8_t                    *p_ep_shared_sram2_ptr;
#endif
    uint8_t                    *p_ep_shared_sram3_ptr;
    uint8_t                    *p_ep0_buffer;
    uint32_t                    ep0_buf_size;
    uint8_t                    *p_ep1_buffer;
    uint32_t                    ep1_buf_size;
    uint8_t                    *p_ep2_buffer;
    uint32_t                    ep2_buf_size;
    uint8_t                    *p_ep3_buffer;
    uint32_t                    ep3_buf_size;
    uint8_t                    *p_ep4_buffer;
    uint32_t                    ep4_buf_size;
    uint8_t                    *p_ep5_buffer;
    uint32_t                    ep5_buf_size;
    uint8_t                    *p_ep6_buffer;
    uint32_t                    ep6_buf_size;
#if 0
    uint8_t                    *p_ep7_buffer;
    uint32_t                    ep7_buf_size;
    uint8_t                    *p_ep8_buffer;
    uint32_t                    ep8_buf_size;
#endif
    usbd_bus_configured_state_t bus_cfg_state;
    uint8_t                     cur_cfg_val;
    usbd_dev_feature_status_t   dev_feat_status;
    bool                        b_full_spd_mode;
    bool                        b_bus_power_on;
    volatile bool               b_bus_rst_immediately;
    volatile bool               b_new_ctrl_setup_arrival;
    volatile bool               b_ctrl_setup_duplication;
    volatile bool               b_ctrl_setup_zero_len;
    volatile bool               b_ep_sram1_job_in_que;
#if 0
    volatile bool               b_ep_sram2_job_in_que;
#endif
    volatile bool               b_ep_sram3_job_in_que;
    volatile bool               b_ep1_sent_completed;
    volatile bool               b_ep2_sent_completed;
    volatile bool               b_ep3_sent_completed;
    volatile bool               b_ep4_new_arrival;
    volatile bool               b_ep5_sent_completed;
    volatile bool               b_ep6_new_arrival;
#if 0
    volatile bool               b_ep7_sent_completed;
    volatile bool               b_ep8_new_arrival;
#endif
    volatile uint8_t            setup_pkt_len;
    uint8_t                     ep4_rcvd_pkt_len;
    uint8_t                     ep6_rcvd_pkt_len;
#if 0
    uint8_t                     ep8_rcvd_pkt_len;
#endif
    uint8_t                     hid_report_id0_idle_rate;
    uint32_t                    hid_tagged_tick;
    uint32_t                    monitor_tagged_tick;
    usb_dev_req_t               dev_req;
    usb_dev_req_t               dev_req_abnormal;
    usbd_job_queue_t            sram1_job_queue[2u];
#if 0
    usbd_job_queue_t            sram2_job_queue[2u];
#endif
    usbd_job_queue_t            sram3_job_queue[2u];
    semaphore_t                 sema_hw_event;
    semaphore_t                 sema_hid;
    semaphore_t                 sema_monitor;
} usbd_dev_if_state_t;

typedef struct _usbd_ep_num_cfg_t
{
    uint8_t group_id;
    uint8_t ep_num;
    uint8_t sram_id;
} usbd_ep_num_cfg_t;

typedef signed short utf16_t;

#pragma pack(1)
/**
 * @brief USB HID Descriptor.
 * @param items 7.
 * @param size 9(9h).
 */
typedef struct _usbd_hid_desc_t
{
    // size 9 bytes, 7 items
    uint8_t blength;    /**< bLength: Size of this descriptor in bytes */
    uint8_t bdesc_type; /**< bDescriptorType: INTERFACE descriptor type */
    uint16_t
        bcdhid; /**< bcdHID: Numeric expression identifying the HID Class Specification release */
    uint8_t bcountry_code; /**< bCountryCode: Numeric expression identifying country code of the
                              localized hardware */
    uint8_t bnum_desc;     /**< bNumDescriptors: Numeric expression specifying the number of class
                              descriptors */
    uint8_t
        bdesc_type0; /**< bDescriptorType0: Constant name identifying type of class descriptor */
    uint16_t wdesc_len0; /**< wdesc_len0: Numeric expression that is the total size of the Report
                            descriptor */
} usbd_hid_desc_t;

/**
 * @brief USB CCID Descriptor.
 * @param items 22.
 * @param size 54(34h).
 */
typedef struct _usbd_ccid_desc_t
{
    uint8_t blength;    /**< bLength: Size of this descriptor in bytes */
    uint8_t bdesc_type; /**< bDescriptorType: INTERFACE descriptor type */
    uint16_t
        bcdccid; /**< bcdCCID: Numeric expression identifying the CID Class Specification release */
    uint8_t
        bmax_slot_idx; /**< bMaxSlotIndex: The index of the highest available slot on this device */
    uint8_t bvolt_support;    /**< bVoltageSupport: This value indicates what voltages the CCID can
                                 supply to its slots */
    uint32_t dwprotocols;     /**< dwprotocols: Protocols specificied */
    uint32_t dwdefault_clk;   /**< dwDefaultClock: Default ICC clock frequency in KHz */
    uint32_t dwmaximum_clk;   /**< dwMaximumClock: Maximum supported ICC clock frequency in KHz */
    uint8_t  bnum_clk_sptd;   /**< bNumClockSupported: The number of clock frequencies that are
                                 supported by the CCID */
    uint32_t dwdata_rate;     /**< dwDataRate: Default ICC I/O data rate in bps. */
    uint32_t dwmax_data_rate; /**< dwMaxDataRate: Maximum supported ICC I/O data rate in bps */
    uint8_t  bnum_data_rate_sptd; /**< bNumDataRatesSupported: The number of data rates that are
                                     supported by the CCID */
    uint32_t
        dwmax_ifsd; /**< dwMaxIFSD: Indicates the maximum IFSD supported by CCID for protocol T=1 */
    uint32_t dwsync_protoc; /**< dwSynchProtocols: Protocols specificied */
    uint32_t dwmechanical;  /**< dwmechanical: The value is a bitwise */
    uint32_t
        dwfeatures; /**< dwFeatures: This value indicates what intelligent features the CCID has. */
    uint32_t dwmax_ccid_msg_len; /**< dwmax_ccid_msg_len: For extended APDU level the value */
    uint8_t  bclass_get_resp; /**< bClassGetResponse: Significant only for CCID that offers an APDU
                                 level for exchanges */
    uint8_t bclass_envelope;  /**< bClassEnvelope: Significant only for CCID that offers an extended
                                 APDU level for  exchanges */
    uint16_t wlcd_layout; /**< wlcd_layout: Number of lines and characters for the LCD display used
                            to send messages for PIN entry */
    uint8_t bpin_support; /**< bPINSupport: This value indicates what PIN support features the CCID
                             has */
    uint8_t bmax_ccid_busy_slt; /**< bMaxCCIDBusySlots: Maximum number of slots which can be
                                   simultaneously busy */
} usbd_ccid_desc_t;

/**
 * @brief USB CDC Header Functional Descriptor.
 * @param items 4.
 * @param size 5(5h).
 */
typedef struct _usbd_cdc_hdr_func_desc_t
{
    uint8_t  blength;       /**< bLength: Size of this descriptor in bytes */
    uint8_t  bdesc_type;    /**< bDescriptorType: FUNCTIONAL descriptor type */
    uint8_t  bdesc_subtype; /**< bDescriptorSubtype: Identifier (ID) of functional descriptor. */
    uint16_t bcdusb; /**< bcdUSB: USB Class Definitions for Communications Devices Spec. release */
} usbd_cdc_hdr_func_desc_t;

/**
 * @brief USB CDC Union Functional Descriptor.
 * @param items 5.
 * @param size 5(5h).
 */
typedef struct _usbd_cdc_uni_func_desc_t
{
    uint8_t blength;       /**< bLength: Size of this descriptor in bytes */
    uint8_t bdesc_type;    /**< bDescriptorType: FUNCTIONAL descriptor type */
    uint8_t bdesc_subtype; /**< bDescriptorSubtype: Identifier (ID) of functional descriptor. */
    uint8_t bctrl_if; /**< bControlInterface: The interface number of the Communications or Data
                         Class interface */
    uint8_t bsubordinate_if0; /**< bSubordinateInterface0: Interface number of first subordinate
                                 interface in the union */
} usbd_cdc_uni_func_desc_t;

/**
 * @brief USB CDC Networking Functional Descriptor.
 * @param items 8.
 * @param size 13(Dh).
 */
typedef struct _usbd_cdc_eth_func_desc_t
{
    uint8_t  blength;          /**< bLength: Size of this descriptor in bytes */
    uint8_t  bdesc_type;       /**< bDescriptorType: FUNCTIONAL descriptor type */
    uint8_t  bdesc_subtype;    /**< bDescriptorSubtype: Identifier (ID) of functional descriptor. */
    uint8_t  imac_address;     /**< iMACAddress: Index of string descriptor */
    uint32_t bmeth_statistics; /**< bmEthernetStatistics: Indicates which Ethernet statistics
                                  functions the device collects */
    uint16_t wmax_seg_size;  /**< wMaxSegmentSize: The maximum segment size that the Ethernet device
                                is capable of  supporting */
    uint16_t wnum_mc_filter; /**< wNumberMCFilters: Contains the number of multicast filters */
    uint8_t  bnum_pwr_filter; /**< bNumberPowerFilters; Contains the number of pattern filters */
} usbd_cdc_eth_func_desc_t;

/**
 * @brief USB Full Speed Composite Descriptor.
 */
typedef struct _usbd_fs_comp_cfg_desc_t
{
    usb_std_cfg_desc_t usb_cfg_desc;
#if (_USB_DEVICE_FUNCTIONLITY_ == _TWO_DISTINCT_FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
    usb_std_if_desc_t usb_if0_desc;
    usbd_hid_desc_t   fido0_desc;
    usb_std_ep_desc_t epoint1;
    usb_std_ep_desc_t epoint6;
    usb_std_if_desc_t usb_if1_desc;
    usbd_hid_desc_t   fido1_desc;
    usb_std_ep_desc_t epoint2;
    usb_std_ep_desc_t epoint8;
#else  /* _USB_DEVICE_FUNCTIONLITY_ */
    usb_std_if_desc_t usb_if0_desc;
    usbd_hid_desc_t   hid_desc;
    usb_std_ep_desc_t epoint1;
    usb_std_if_desc_t usb_if1_desc;
    usbd_hid_desc_t   fido_desc;
    usb_std_ep_desc_t epoint2;
    usb_std_ep_desc_t epoint6;
    usb_std_if_desc_t usb_if2_desc;
    usbd_ccid_desc_t  ccid_desc;
    usb_std_ep_desc_t epoint8;
    usb_std_ep_desc_t epoint7;
    usb_std_ep_desc_t epoint3;
#endif /* _USB_DEVICE_FUNCTIONLITY_ */
} usbd_fs_comp_cfg_desc_t;

/**
 * @brief USB Full Speed HID Configuration Descriptor.
 */
typedef struct _usbd_fs_hid_cfg_desc_t
{
    usb_std_cfg_desc_t usb_cfg_desc;
    usb_std_if_desc_t  usb_if_desc;
    usbd_hid_desc_t    hid_desc;
#if (_USB_DEVICE_FUNCTIONLITY_ == _HID_COMPLIANT_FIDO_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
    usb_std_ep_desc_t epoint5;
    usb_std_ep_desc_t epoint8;
#else  /* _USB_DEVICE_FUNCTIONLITY_ */
    usb_std_ep_desc_t epoint1;
#endif /* _USB_DEVICE_FUNCTIONLITY_ */
} usbd_fs_hid_cfg_desc_t;

/**
 * @brief USB Full Speed MSC Configuration Descriptor.
 */
typedef struct _usbd_fs_msc_cfg_desc_t
{
    usb_std_cfg_desc_t usb_cfg_desc;
    usb_std_if_desc_t  usb_if_desc;
    usb_std_ep_desc_t  epoint4;
    usb_std_ep_desc_t  epoint2;
} usbd_fs_msc_cfg_desc_t;

/**
 * @brief USB Full Speed CDC Configuration Descriptor.
 */
typedef struct _usbd_fs_cdc_cfg_desc_t
{
    usb_std_cfg_desc_t       usb_cfg_desc;
    usb_std_if_desc_t        usb_if0_desc;
    usbd_cdc_hdr_func_desc_t cdc_hdr_func_desc;
    usbd_cdc_uni_func_desc_t cdc_uni_func_desc;
    usbd_cdc_eth_func_desc_t cdc_eth_func_desc;
    usb_std_ep_desc_t        epoint3;
    usb_std_if_desc_t        usb_if1_desc;
    usb_std_if_desc_t        usb_if1_alt_desc;
    usb_std_ep_desc_t        epoint1;
    usb_std_ep_desc_t        epoint2;
} usbd_fs_cdc_cfg_desc_t;

/**
 * @brief USB Language ID Descriptor.
 */
typedef struct _usbd_str_lang_desc_t
{
    uint8_t  blength;    /**< bLength: Size of this descriptor in bytes */
    uint8_t  bdesc_type; /**< bDescriptorType: STRING descriptor type */
    uint16_t wlang_id;   /**< wLangId: Language ID */
} usbd_str_lang_desc_t;

/**
 * @brief USB Manufacturer Descriptor.
 */
typedef struct _usbd_str_mfr_t
{
    uint8_t blength;                    /**< bLength: Size of this descriptor in bytes */
    uint8_t bdesc_type;                 /**< bDescriptorType: STRING descriptor type */
    utf16_t utf16_str[USBD_MFR_LENGTH]; /**< Unicode characters */
} usbd_str_mfr_t;

/**
 * @brief USB Product Descriptor.
 */
typedef struct _usbd_str_prod_t
{
    uint8_t blength;                   /**< bLength: Size of this descriptor in bytes */
    uint8_t bdesc_type;                /**< bDescriptorType: STRING descriptor type */
    utf16_t utf16_str[USBD_PN_LENGTH]; /**< Unicode characters */
} usbd_str_prod_t;

/**
 * @brief USB Serial Number Descriptor.
 */
typedef struct _usbd_str_sn_t
{
    uint8_t blength;                   /**< bLength: Size of this descriptor in bytes */
    uint8_t bdesc_type;                /**< bDescriptorType: STRING descriptor type */
    utf16_t utf16_str[USBD_SN_LENGTH]; /**< Unicode characters */
} usbd_str_sn_t;

/**
 * @brief USB HID Interface Report Descriptor.
 *
 * @param usage_page_1st_byte
 * bit field | 7   6   5   4 | 3   2 | 1   0 |
 *           | bTag          | bType | bSize |
 * bSize Numeric expression specifying size of data:
 * 0 = 0 bytes
 * 1 = 1 byte
 * 2 = 2 bytes
 * 3 = 4 bytes
 * bType Numeric expression identifying type of item where:
 * 0 = Main
 * 1 = Global
 * 2 = Local
 * 3 = Reserved
 * bTag Numeric expression specifying the function of the item.
 */
typedef union _usbd_ifhid_report_t
{
    uint8_t report[46u];
    uint8_t usage_page_1st_byte;
} usbd_ifhid_report_t;

/**
 * @brief USB HID Compliant FIDO Report Descriptor.
 *
 * @param usage_page_1st_byte
 * bit field | 7   6   5   4 | 3   2 | 1   0 |
 *           | bTag          | bType | bSize |
 * bSize Numeric expression specifying size of data:
 * 0 = 0 bytes
 * 1 = 1 byte
 * 2 = 2 bytes
 * 3 = 4 bytes
 * bType Numeric expression identifying type of item where:
 * 0 = Main
 * 1 = Global
 * 2 = Local
 * 3 = Reserved
 * bTag Numeric expression specifying the function of the item.
 */
typedef union _usbd_fido_report_t
{
    uint8_t report[34u];
    uint8_t usage_page_1st_byte;
} usbd_fido_report_t;

/**
 * @brief USB CDC Interface Management Element Notification.
 */
typedef struct _usbd_if_ntfy_t
{
    uint8_t bmreq_type;
    uint8_t bnotification; /**< bNotification: Notification */
    u16_t   wvalue;
    u16_t   windex;
    u16_t   wlength;
} usbd_if_ntfy_t;

/**
 * @brief USB CDC Interface Speed Change Notification.
 */
typedef struct _usbd_ntfy_spd_t
{
    usbd_if_ntfy_t eth_spd_change;
    uint32_t dl_bit_rate; /**< dwDLBitRate: Contains the downlink bit rate, in bits per second */
    uint32_t ul_bit_rate; /**< dwULBitRate: Contains the uplink bit rate, in bits per second */
} usbd_ntfy_spd_t;

/**
 * @brief USB MSC Interface UFI Command Block Descriptor(CBD).
 */
typedef union _usbd_ufi_cbd_t
{
    uint8_t op_code;      /**< UFI command block, SCSI command wrapper */
    uint8_t scsi_cb[16u]; /**< UFI command block, SCSI command wrapper */
} usbd_ufi_cbd_t;

/**
 * @brief USB MSC Interface Command Block Wrapper(CBW).
 * @param items 7.
 * @param size 31(1Fh).
 */
typedef struct _usbd_cbw_t
{
    uint8_t        signature[4u]; /**< Signature */
    uint8_t        tag[4u];       /**< Tag */
    uint32_t       data_xfer_byte_len;
    uint8_t        flag;
    uint8_t        lun;
    uint8_t        cbd_byte_len;
    usbd_ufi_cbd_t cbd;
} usbd_cbw_t;

/**
 * @brief USB MSC Interface Command Status Wrapper(CSW).
 * @param items 4.
 * @param size 13(Dh).
 */
typedef struct _usbd_csw_t
{
    uint8_t  signature[4u]; /**< Signature */
    uint8_t  tag[4u];       /**< Tag */
    uint32_t data_residue;
    uint8_t  status;
} usbd_csw_t;
#pragma pack()

/*******************************************************************************
 * Global variables
 ******************************************************************************/

/*******************************************************************************
 * Static variables
 ******************************************************************************/
/**
 * @brief Pointer to USB device runtime state structure.
 */
static usbd_dev_if_state_t *sp_usbd_state_ptr = NULL;

/**
 * @brief USB device runtime state variable.
 */
usbd_dev_if_state_t g_usbd_dev_if_state = {0u};

static const usbd_ep_num_cfg_t s_ep_cfg[] = {
    {
        .group_id = 0u,
        .ep_num   = USBD_EP_NB1,
        .sram_id  = 3u,
    },
    {
        .group_id = 0u,
        .ep_num   = USBD_EP_NB2,
        .sram_id  = 3u,
    },
    {
        .group_id = 0u,
        .ep_num   = USBD_EP_NB3,
        .sram_id  = 3u,
    },
    {
        .group_id = 0u,
        .ep_num   = USBD_EP_NB4,
        .sram_id  = 3u,
    },
    {
        .group_id = 1u,
        .ep_num   = USBD_EP_NB5,
        .sram_id  = 1u,
    },
    {
        .group_id = 1u,
        .ep_num   = USBD_EP_NB6,
        .sram_id  = 1u,
    },
};

static const usb_std_dev_desc_t s_usb_dev_desc = {.blength        = sizeof(usb_std_dev_desc_t),
                                                  .bdesc_type     = USB_DESC_DEVICE,
                                                  .bcdusb         = USBD_STD_SPEC_REV_BCD,
                                                  .bdev_class     = USBD_DEVICE_CLASS,
                                                  .bdev_sub_class = USBD_DEVICE_SUB_CLASS,
                                                  .bdev_protocol  = USBD_DEVICE_PROTOCOL,
                                                  .bmax_pkt_size0 = USBD_CTRL_EP0_PKT_LEN,
                                                  .idvendor       = USBD_VENDOR_ID,
                                                  .idproduct      = USBD_PRODUCT_ID,
                                                  .bcddev         = USBD_RELEASE_NUMBER_BCD,
                                                  .imfr           = USBD_STR_MFR_INDEX,
                                                  .iproduct       = USBD_STR_PROD_INDEX,
                                                  .iserial_num    = USBD_STR_SN_INDEX,
                                                  .bnum_cfg       = USBD_NB_CONFIGURATION};

static const usb_std_dev_qualf_desc_t s_usbd_qualf_desc = {.blength =
                                                               sizeof(usb_std_dev_qualf_desc_t),
                                                           .bdesc_type = USB_DESC_DEVICE_QUALIFIER,
                                                           .bcdusb     = USBD_STD_SPEC_REV_BCD,
                                                           .bdev_class = USBD_DEVICE_CLASS,
                                                           .bdev_sub_class = USBD_DEVICE_SUB_CLASS,
                                                           .bdev_protocol  = USBD_DEVICE_PROTOCOL,
                                                           .bmax_pkt_size0 = USBD_CTRL_EP0_PKT_LEN,
                                                           .bnum_cfg       = USBD_NB_CONFIGURATION,
                                                           .breserved      = 0x00u};

static const usbd_fs_comp_cfg_desc_t s_usbd_fs_comp_cfg_desc = {
    .usb_cfg_desc = {.blength    = sizeof(usb_std_cfg_desc_t),
                     .bdesc_type = USB_DESC_CONFIGURATION,
                     .wtotal_len = sizeof(usbd_fs_comp_cfg_desc_t),
                     .bnum_if    = USBD_NB_INTERFACE,
                     .bcfg_value = USBD_CONF_VAL,
                     .icfg       = USBD_CONF_INDEX,
                     .bmattrib   = USBD_CONF_ATTRIB,
                     .bmax_power = USBD_MAX_POWER},
#if (_USB_DEVICE_FUNCTIONLITY_ == _TWO_DISTINCT_FIDO_SE_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
    .usb_if0_desc = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_HID_CPL_FIDO_IF0_NB,
                     .balt_setting  = USBD_HID_IF0_ALT0,
                     .bnum_ep       = USBD_HID_CPL_FIDO_IF0_NB_EP,
                     .bif_class     = USBD_HID_IF0_CLASS,
                     .bif_sub_class = USBD_HID_CPL_FIDO_IF0_SUB_CLASS,
                     .bif_protocol  = USBD_HID_CPL_FIDO_IF0_PC,
                     .iinterface    = 3u},
    .fido0_desc   = {.blength       = sizeof(usbd_hid_desc_t),
                     .bdesc_type    = USBD_DESC_CS_HID,
                     .bcdhid        = USBD_HID_SPEC_REV_BCD,
                     .bcountry_code = 0x00u,
                     .bnum_desc     = USBD_HIDDESC_NB_DESC,
                     .bdesc_type0   = USBD_DESC_CS_REPORT,
                     .wdesc_len0    = sizeof(usbd_fido_report_t)},
    .epoint1      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP1_DI,
                     .bmattrib      = USBD_EP1_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP1_SIZE,
                     .binterval     = USBD_EP1_INTERVAL},
    .epoint6      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP6_DO,
                     .bmattrib      = USBD_EP6_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP6_SIZE,
                     .binterval     = USBD_EP6_INTERVAL},
    .usb_if1_desc = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_HID_CPL_FIDO_IF1_NB,
                     .balt_setting  = USBD_HID_IF0_ALT0,
                     .bnum_ep       = USBD_HID_CPL_FIDO_IF0_ALT0_NB_EP,
                     .bif_class     = USBD_HID_IF0_CLASS,
                     .bif_sub_class = USBD_HID_CPL_FIDO_IF0_SUB_CLASS,
                     .bif_protocol  = USBD_HID_CPL_FIDO_IF0_PC,
                     .iinterface    = 4u},
    .fido1_desc   = {.blength       = sizeof(usbd_hid_desc_t),
                     .bdesc_type    = USBD_DESC_CS_HID,
                     .bcdhid        = USBD_HID_SPEC_REV_BCD,
                     .bcountry_code = 0x00u,
                     .bnum_desc     = USBD_HIDDESC_NB_DESC,
                     .bdesc_type0   = USBD_DESC_CS_REPORT,
                     .wdesc_len0    = sizeof(usbd_fido_report_t)},
    .epoint2      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP2_DI,
                     .bmattrib      = USBD_EP2_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP2_SIZE,
                     .binterval     = USBD_EP2_INTERVAL},
    .epoint8      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP8_DO,
                     .bmattrib      = USBD_EP8_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP8_SIZE,
                     .binterval     = USBD_EP8_INTERVAL}
#else  /* _USB_DEVICE_FUNCTIONLITY_ */
    .usb_if0_desc = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_HID_IF0_NB,
                     .balt_setting  = USBD_HID_IF0_ALT,
                     .bnum_ep       = USBD_HID_IF0_NB_EP,
                     .bif_class     = USBD_HID_IF0_CLASS,
                     .bif_sub_class = USBD_HID_IF0_SUB_CLASS,
                     .bif_protocol  = USBD_HID_IF0_PC,
                     .iinterface    = 0u},
    .hid_desc     = {.blength       = sizeof(usbd_hid_desc_t),
                     .bdesc_type    = USBD_DESC_CS_HID,
                     .bcdhid        = USBD_HID_SPEC_REV_BCD,
                     .bcountry_code = 0x00u,
                     .bnum_desc     = USBD_HIDDESC_NB_DESC,
                     .bdesc_type0   = USBD_DESC_CS_REPORT,
                     .wdesc_len0    = sizeof(usbd_ifhid_report_t)},
    .epoint1      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP1_DI,
                     .bmattrib      = USBD_EP1_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP1_SIZE,
                     .binterval     = USBD_EP1_INTERVAL},
    .usb_if1_desc = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_HID_CPL_FIDO_IF1_NB,
                     .balt_setting  = USBD_HID_CPL_FIDO_IF1_ALT,
                     .bnum_ep       = USBD_HID_CPL_FIDO_IF1_NB_EP,
                     .bif_class     = USBD_HID_IF1_CLASS,
                     .bif_sub_class = USBD_HID_CPL_FIDO_IF1_SUB_CLASS,
                     .bif_protocol  = USBD_HID_CPL_FIDO_IF1_PC,
                     .iinterface    = 0u},
    .fido_desc    = {.blength       = sizeof(usbd_hid_desc_t),
                     .bdesc_type    = USBD_DESC_CS_HID,
                     .bcdhid        = USBD_HID_SPEC_REV_BCD,
                     .bcountry_code = 0x00u,
                     .bnum_desc     = USBD_HIDDESC_NB_DESC,
                     .bdesc_type0   = USBD_DESC_CS_REPORT,
                     .wdesc_len0    = sizeof(usbd_fido_report_t)},
    .epoint2      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP2_DI,
                     .bmattrib      = USBD_EP2_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP2_SIZE,
                     .binterval     = USBD_EP2_INTERVAL},
    .epoint6      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP6_DO,
                     .bmattrib      = USBD_EP6_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP6_SIZE,
                     .binterval     = USBD_EP6_INTERVAL},
    .usb_if2_desc = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_CCID_IF0_NB,
                     .balt_setting  = USBD_CCID_IF0_ALT0,
                     .bnum_ep       = USBD_CCID_IF0_ALT0_NB_EP,
                     .bif_class     = USBD_CCID_IF0_CLASS,
                     .bif_sub_class = USBD_CCID_IF0_SUB_CLASS,
                     .bif_protocol  = USBD_CCID_IF0_PC,
                     .iinterface    = 0u},
    .ccid_desc    = {.blength             = sizeof(usbd_ccid_desc_t),
                     .bdesc_type          = USBD_DESC_CS_DEVICE,
                     .bcdccid             = USBD_CCID_SPEC_REV_BCD,
                     .bmax_slot_idx       = 0x00u,
                     .bvolt_support       = 0x07u,
                     .dwprotocols         = 0x00000002u,
                     .dwdefault_clk       = 3750u,
                     .dwmaximum_clk       = 3750u,
                     .bnum_clk_sptd       = 0x00u,
                     .dwdata_rate         = 19200u,
                     .dwmax_data_rate     = 115200u,
                     .bnum_data_rate_sptd = 0x00u,
                     .dwmax_ifsd          = 3062u,
                     .dwsync_protoc       = 0u,
                     .dwmechanical        = 0x00000000u,
                     .dwfeatures          = 0x000400FEu,
                     .dwmax_ccid_msg_len  = 3072u,
                     .bclass_get_resp     = 0xFFu,
                     .bclass_envelope     = 0xFFu,
                     .wlcd_layout         = 0x0000u,
                     .bpin_support        = 0x00u,
                     .bmax_ccid_busy_slt  = 0x01u},
    .epoint8      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP8_DO,
                     .bmattrib      = USBD_EP8_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP8_SIZE,
                     .binterval     = USBD_EP8_INTERVAL},
    .epoint7      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP7_DI,
                     .bmattrib      = USBD_EP7_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP7_SIZE,
                     .binterval     = USBD_EP7_INTERVAL},
    .epoint3      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP3_DI,
                     .bmattrib      = USBD_EP3_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP3_SIZE,
                     .binterval     = USBD_EP3_INTERVAL}
#endif /* _USB_DEVICE_FUNCTIONLITY_ */
};

static const usbd_fs_hid_cfg_desc_t s_usbd_fs_hid_cfg_desc = {
    .usb_cfg_desc = {.blength    = sizeof(usb_std_cfg_desc_t),
                     .bdesc_type = USB_DESC_CONFIGURATION,
                     .wtotal_len = sizeof(usbd_fs_hid_cfg_desc_t),
                     .bnum_if    = USBD_NB_INTERFACE,
                     .bcfg_value = USBD_CONF_VAL,
                     .icfg       = USBD_CONF_INDEX,
                     .bmattrib   = USBD_CONF_ATTRIB,
                     .bmax_power = USBD_MAX_POWER},
#if (_USB_DEVICE_FUNCTIONLITY_ == _HID_COMPLIANT_FIDO_FACILITY_) /* _USB_DEVICE_FUNCTIONLITY_ */
    .usb_if_desc = {.blength       = sizeof(usb_std_if_desc_t),
                    .bdesc_type    = USB_DESC_INTERFACE,
                    .bif_number    = USBD_HID_IF0_NB,
                    .balt_setting  = USBD_HID_IF0_ALT,
                    .bnum_ep       = USBD_HID_CPL_FIDO_IF0_NB_EP,
                    .bif_class     = USBD_HID_IF0_CLASS,
                    .bif_sub_class = USBD_HID_CPL_FIDO_IF0_SUB_CLASS,
                    .bif_protocol  = USBD_HID_CPL_FIDO_IF0_PC,
                    .iinterface    = USB_STR_IDX003},
    .hid_desc    = {.blength       = sizeof(usbd_hid_desc_t),
                    .bdesc_type    = USBD_DESC_CS_HID,
                    .bcdhid        = USBD_HID_SPEC_REV_BCD,
                    .bcountry_code = 0x00u,
                    .bnum_desc     = USBD_HIDDESC_NB_DESC,
                    .bdesc_type0   = USBD_DESC_CS_REPORT,
                    .wdesc_len0    = sizeof(usbd_fido_report_t)},
    .epoint5     = {.blength       = sizeof(usb_std_ep_desc_t),
                    .bdesc_type    = USB_DESC_ENDPOINT,
                    .bep_address   = USBD_EP5_DI,
                    .bmattrib      = USBD_EP5_ATTRIBUTES,
                    .wmax_pkt_size = USBD_EP5_SIZE,
                    .binterval     = USBD_EP5_INTERVAL},
    .epoint8     = {.blength       = sizeof(usb_std_ep_desc_t),
                    .bdesc_type    = USB_DESC_ENDPOINT,
                    .bep_address   = USBD_EP8_DO,
                    .bmattrib      = USBD_EP8_ATTRIBUTES,
                    .wmax_pkt_size = USBD_EP8_SIZE,
                    .binterval     = USBD_EP8_INTERVAL}
#else  /* _USB_DEVICE_FUNCTIONLITY_ */
    .usb_if_desc  = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_HID_IF0_NB,
                     .balt_setting  = USBD_HID_IF0_ALT,
                     .bnum_ep       = USBD_HID_IF0_NB_EP,
                     .bif_class     = USBD_HID_IF0_CLASS,
                     .bif_sub_class = USBD_HID_IF0_SUB_CLASS,
                     .bif_protocol  = USBD_HID_IF0_PC,
                     .iinterface    = USB_STR_IDX003},
    .hid_desc     = {.blength       = sizeof(usbd_hid_desc_t),
                     .bdesc_type    = USBD_DESC_CS_HID,
                     .bcdhid        = USBD_HID_SPEC_REV_BCD,
                     .bcountry_code = 0x00u,
                     .bnum_desc     = USBD_HIDDESC_NB_DESC,
                     .bdesc_type0   = USBD_DESC_CS_REPORT,
                     .wdesc_len0    = sizeof(usbd_ifhid_report_t)},
    .epoint1      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP1_DI,
                     .bmattrib      = USBD_EP1_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP1_SIZE,
                     .binterval     = USBD_EP1_INTERVAL}
#endif /* _USB_DEVICE_FUNCTIONLITY_ */
};

static const usbd_fs_msc_cfg_desc_t s_usbd_fs_msc_cfg_desc = {
    .usb_cfg_desc = {.blength    = sizeof(usb_std_cfg_desc_t),
                     .bdesc_type = USB_DESC_CONFIGURATION,
                     .wtotal_len = sizeof(usbd_fs_msc_cfg_desc_t),
                     .bnum_if    = USBD_NB_INTERFACE,
                     .bcfg_value = USBD_CONF_VAL,
                     .icfg       = USBD_CONF_INDEX,
                     .bmattrib   = USBD_CONF_ATTRIB,
                     .bmax_power = USBD_MAX_POWER},
    .usb_if_desc  = {.blength       = sizeof(usb_std_if_desc_t),
                     .bdesc_type    = USB_DESC_INTERFACE,
                     .bif_number    = USBD_MSC_IF0_NB,
                     .balt_setting  = USBD_MSC_IF0_ALT0,
                     .bnum_ep       = USBD_MSC_IF0_ALT0_NB_EP,
                     .bif_class     = USBD_MSC_IF0_CLASS,
                     .bif_sub_class = USBD_MSC_IF0_SUB_CLASS,
                     .bif_protocol  = USBD_MSC_IF0_PC,
                     .iinterface    = 0u},
    .epoint4      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP4_DO,
                     .bmattrib      = USBD_EP4_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP4_SIZE,
                     .binterval     = USBD_EP4_INTERVAL},
    .epoint2      = {.blength       = sizeof(usb_std_ep_desc_t),
                     .bdesc_type    = USB_DESC_ENDPOINT,
                     .bep_address   = USBD_EP2_DI,
                     .bmattrib      = USBD_EP2_ATTRIBUTES,
                     .wmax_pkt_size = USBD_EP2_SIZE,
                     .binterval     = USBD_EP2_INTERVAL}};

static const usbd_fs_cdc_cfg_desc_t s_usbd_fs_cdc_cfg_desc = {
    .usb_cfg_desc      = {.blength    = sizeof(usb_std_cfg_desc_t),
                          .bdesc_type = USB_DESC_CONFIGURATION,
                          .wtotal_len = sizeof(usbd_fs_cdc_cfg_desc_t),
                          .bnum_if    = USBD_NB_INTERFACE,
                          .bcfg_value = USBD_CONF_VAL,
                          .icfg       = USBD_CONF_INDEX,
                          .bmattrib   = USBD_CONF_ATTRIB,
                          .bmax_power = USBD_MAX_POWER},
    .usb_if0_desc      = {.blength       = sizeof(usb_std_if_desc_t),
                          .bdesc_type    = USB_DESC_INTERFACE,
                          .bif_number    = USBD_CDC_IF0_NB,
                          .balt_setting  = USBD_CDC_IF0_ALT0,
                          .bnum_ep       = USBD_CDC_IF0_ALT0_NB_EP,
                          .bif_class     = USBD_CDC_IF0_CLASS,
                          .bif_sub_class = USBD_CDC_IF0_SUB_CLASS,
                          .bif_protocol  = USBD_CDC_IF0_PC,
                          .iinterface    = 0u},
    .cdc_hdr_func_desc = {.blength       = sizeof(usbd_cdc_hdr_func_desc_t),
                          .bdesc_type    = USBD_DESC_CS_INTERFACE,
                          .bdesc_subtype = USBD_COMM_FD_HEADER,
                          .bcdusb        = USBD_CDC_SPEC_REV_BCD},
    .cdc_uni_func_desc = {.blength          = sizeof(usbd_cdc_uni_func_desc_t),
                          .bdesc_type       = USBD_DESC_CS_INTERFACE,
                          .bdesc_subtype    = USBD_COMM_FD_UNION,
                          .bctrl_if         = USBD_CICUF_NB_IF,
                          .bsubordinate_if0 = USBD_CICUF_FIRST_IF_IDX},
    .cdc_eth_func_desc = {.blength          = sizeof(usbd_cdc_eth_func_desc_t),
                          .bdesc_type       = USBD_DESC_CS_INTERFACE,
                          .bdesc_subtype    = USBD_COMM_FD_ETHERNET,
                          .imac_address     = USBD_CICEN_MAC,
                          .bmeth_statistics = USBD_CICEN_ETHER_STAT,
                          .wmax_seg_size    = USBD_CICEN_MAX_SEG_SIZE,
                          .wnum_mc_filter   = USBD_CICEN_NB_MFC_FILTER,
                          .bnum_pwr_filter  = USBD_CICEN_NB_PWR_FILTER},
    .epoint3           = {.blength       = sizeof(usb_std_ep_desc_t),
                          .bdesc_type    = USB_DESC_ENDPOINT,
                          .bep_address   = USBD_EP3_DI,
                          .bmattrib      = USBD_EP3_ATTRIBUTES,
                          .wmax_pkt_size = USBD_EP3_SIZE,
                          .binterval     = USBD_EP3_INTERVAL},
    .usb_if1_desc      = {.blength       = sizeof(usb_std_if_desc_t),
                          .bdesc_type    = USB_DESC_INTERFACE,
                          .bif_number    = USBD_CDC_IF1_NB,
                          .balt_setting  = USBD_CDC_IF1_ALT0,
                          .bnum_ep       = USBD_CDC_IF1_ALT0_NB_EP,
                          .bif_class     = USBD_CDC_IF1_CLASS,
                          .bif_sub_class = USBD_CDC_IF1_SUB_CLASS,
                          .bif_protocol  = USBD_CDC_IF1_PC,
                          .iinterface    = 0u},
    .usb_if1_alt_desc  = {.blength       = sizeof(usb_std_if_desc_t),
                          .bdesc_type    = USB_DESC_INTERFACE,
                          .bif_number    = USBD_CDC_IF1_NB,
                          .balt_setting  = USBD_CDC_IF1_ALT1,
                          .bnum_ep       = USBD_CDC_IF1_ALT1_NB_EP,
                          .bif_class     = USBD_CDC_IF1_CLASS,
                          .bif_sub_class = USBD_CDC_IF1_SUB_CLASS,
                          .bif_protocol  = USBD_CDC_IF1_PC,
                          .iinterface    = 0u},
    .epoint1           = {.blength       = sizeof(usb_std_ep_desc_t),
                          .bdesc_type    = USB_DESC_ENDPOINT,
                          .bep_address   = USBD_EP1_DI,
                          .bmattrib      = USBD_EP1_ATTRIBUTES,
                          .wmax_pkt_size = USBD_EP1_SIZE,
                          .binterval     = USBD_EP1_INTERVAL},
    .epoint2           = {.blength       = sizeof(usb_std_ep_desc_t),
                          .bdesc_type    = USB_DESC_ENDPOINT,
                          .bep_address   = USBD_EP2_DI,
                          .bmattrib      = USBD_EP2_ATTRIBUTES,
                          .wmax_pkt_size = USBD_EP2_SIZE,
                          .binterval     = USBD_EP2_INTERVAL}};

static const usbd_str_lang_desc_t s_usbd_str_lang_desc = {.blength = sizeof(usbd_str_lang_desc_t),
                                                          .bdesc_type = USB_DESC_STRING,
                                                          .wlang_id   = USBD_LANGUAGE_ID};

static const usbd_str_mfr_t s_usbd_str_mfr = {.blength    = sizeof(usbd_str_mfr_t),
                                              .bdesc_type = USB_DESC_STRING,
                                              .utf16_str  = USBD_MANUFACTURER_NAME};

static const usbd_str_prod_t s_usbd_str_prod = {.blength    = sizeof(usbd_str_prod_t),
                                                .bdesc_type = USB_DESC_STRING,
                                                .utf16_str  = USBD_PRODUCT_NAME};

static const usbd_str_sn_t s_usbd_str_sn = {.blength    = sizeof(usbd_str_sn_t),
                                            .bdesc_type = USB_DESC_STRING,
                                            .utf16_str  = USBD_SERIAL_NUMBER};

#if 0
static const usbd_str_sn_t s_usbd_str_sn2 = {.blength    = sizeof(usbd_str_sn_t),
                                      .bdesc_type = USB_DESC_STRING,
                                      .utf16_str  = USBD_2ND_SERIAL_NUMBER};
#endif

/**
 * @brief HID compliant mouse device. Report descriptor.
 *
 * @example
 *  const uint8_t HID_ReportDescriptor[] = {
 *      0x05u, 0x01u,           // HID_UsagePage ( HID_USAGE_PAGE_GENERIC_DESKTOP ),
 *      0x09u, 0x02u,           // HID_Usage ( HID_USAGE_MOUSE ),
 *      0xA1u, 0x01u,           // HID_Collection ( HID_Application ),
 *      0x09u, 0x01u,           // HID_Usage ( HID_USAGE_POINTER ),
 *      0xA1u, 0x00u,           // HID_Collection ( HID_Physical ),
 *      0x05u, 0x09u,           // HID_UsagePage ( HID_USAGE_PAGE_BUTTON ),
 *      0x19u, 0x01u,           // HID_UsageMin ( 1 ),
 *      0x29u, 0x03u,           // HID_UsageMax ( 3 ),
 *      0x15u, 0x00u,           // HID_LogicalMin ( 0 ),
 *      0x25u, 0x01u,           // HID_LogicalMax ( 1 ),
 *      0x75u, 0x01u,           // HID_ReportSize ( 1 ),
 *      0x95u, 0x08u,           // HID_ReportCount ( 8 ),
 *      0x81u, 0x02u,           // HID_Input ( HID_Data | HID_Absolute | HID_Variable ),
 *      0x05u, 0x01u,           // HID_UsagePage ( HID_USAGE_PAGE_GENERIC_DESKTOP ),
 *      0x09u, 0x30u,           // HID_Usage ( HID_USAGE_X ),
 *      0x09u, 0x31u,           // HID_Usage ( HID_USAGE_Y ),
 *      0x09u, 0x38u,           // HID_Usage ( HID_USAGE_OEM_DEFINE ),
 *      0x15u, 0x81u,           // HID_LogicalMin ( -127 ),
 *      0x25u, 0x7Fu,           // HID_LogicalMax ( 127 ),
 *      0x75u, 0x08u,           // HID_ReportSize ( 8 ),
 *      0x95u, 0x03u,           // HID_ReportCount ( 3 ),
 *      0x81u, 0x06u,           // HID_Input ( HID_Data | HID_Relative | HID_Variable ),
 *      0xC0u                   // HID_EndCollection
 *      0xC0u                   // HID_EndCollection
 *  };
 */
static const usbd_ifhid_report_t s_hid_report = {
    .report = {0x05u, 0x01u, 0x09u, 0x02u, 0xA1u, 0x01u, 0x09u, 0x01u, 0xA1u, 0x00u, 0x05u, 0x09u,
               0x19u, 0x01u, 0x29u, 0x03u, 0x15u, 0x00u, 0x25u, 0x01u, 0x75u, 0x01u, 0x95u, 0x08u,
               0x81u, 0x02u, 0x05u, 0x01u, 0x09u, 0x30u, 0x09u, 0x31u, 0x09u, 0x38u, 0x15u, 0x81u,
               0x25u, 0x7Fu, 0x75u, 0x08u, 0x95u, 0x03u, 0x81u, 0x06u, 0xC0u, 0xC0u},
};

/**
 * @brief HID compliant FIDO device. Report descriptor.
 *
 * @example
 *  const uint8_t HID_ReportDescriptor[] = {
 *      0x06u, 0xD0u, 0xF1u,    // HID_UsagePage ( FIDO_USAGE_PAGE ),
 *      0x09u, 0x01u,           // HID_Usage ( FIDO_USAGE_U2FHID / FIDO_USAGE_CTAPHID ),
 *      0xA1u, 0x01u,           // HID_Collection ( HID_Application ),
 *      0x09u, 0x20u,           // HID_Usage ( FIDO_USAGE_DATA_IN ),
 *      0x15u, 0x00u,           // HID_LogicalMin ( 0 ),
 *      0x26u, 0xFFu, 0x00u,    // HID_LogicalMaxS ( 0x00FF ),
 *      0x75u, 0x08u,           // HID_ReportSize ( 8 ),
 *      0x95u, 0x40u,           // HID_ReportCount ( HID_INPUT_REPORT_BYTES ),
 *      0x81u, 0x02u,           // HID_Input ( HID_Data | HID_Absolute | HID_Variable ),
 *      0x09u, 0x21u,           // HID_Usage ( FIDO_USAGE_DATA_OUT ),
 *      0x15u, 0x00u,           // HID_LogicalMin ( 0 ),
 *      0x26u, 0xFFu, 0x00u,    // HID_LogicalMaxS ( 0x00FF ),
 *      0x75u, 0x08u,           // HID_ReportSize ( 8 ),
 *      0x95u, 0x40u,           // HID_ReportCount ( HID_OUTPUT_REPORT_BYTES ),
 *      0x91u, 0x02u,           // HID_Output ( HID_Data | HID_Absolute | HID_Variable ),
 *      0xC0u                   // HID_EndCollection
 *  };
 */
static const usbd_fido_report_t s_fido_report = {
    .report = {0x06u, 0xD0u, 0xF1u, 0x09u, 0x01u, 0xA1u, 0x01u, 0x09u, 0x20u, 0x15u, 0x00u, 0x26u,
               0xFFu, 0x00u, 0x75u, 0x08u, 0x95u, 0x40u, 0x81u, 0x02u, 0x09u, 0x21u, 0x15u, 0x00u,
               0x26u, 0xFFu, 0x00u, 0x75u, 0x08u, 0x95u, 0x40u, 0x91u, 0x02u, 0xC0u},
};

static const usbd_if_ntfy_t s_eth_network_conn = {
    .bmreq_type           = (USB_RT_DIR_DEV2HOST | USB_RT_TYPE_CLASS | USB_RT_RECIPIENT_INTERFACE),
    .bnotification        = USBD_CSNTFY_NW_CONN,
    .wvalue.db_type.word  = USBD_NCONN_DISCONNECT,
    .windex.db_type.word  = USBD_CICUF_FIRST_IF_IDX,
    .wlength.db_type.word = 0u};

static const usbd_if_ntfy_t s_eth_resp_ava = {
    .bmreq_type           = (USB_RT_DIR_DEV2HOST | USB_RT_TYPE_CLASS | USB_RT_RECIPIENT_INTERFACE),
    .bnotification        = USBD_CSNTFY_RESP_AVA,
    .wvalue.db_type.word  = 0u,
    .windex.db_type.word  = USBD_CICUF_FIRST_IF_IDX,
    .wlength.db_type.word = 0u};

static const usbd_ntfy_spd_t s_eth_conn_spd_change = {
    .eth_spd_change =
        {
            .bmreq_type    = (USB_RT_DIR_DEV2HOST | USB_RT_TYPE_CLASS | USB_RT_RECIPIENT_INTERFACE),
            .bnotification = USBD_CSNTFY_CONN_SPD_CHANGE,
            .wvalue.db_type.word  = 0u,
            .windex.db_type.word  = USBD_CICUF_FIRST_IF_IDX,
            .wlength.db_type.word = (uint16_t)(MEMBER_SIZE(usbd_ntfy_spd_t, dl_bit_rate) +
                                               MEMBER_SIZE(usbd_ntfy_spd_t, ul_bit_rate)),
        },
    .dl_bit_rate = USBD_NCONN_BR_ZERO,
    .ul_bit_rate = USBD_NCONN_BR_ZERO};

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/
/**
 * @brief USB configuration interface reset.
 */
static void usbd_configuration_reset(void);

/**
 * @brief A task to handle all the events of USB standard request.
 */
static void usbd_ctrl_std_cmd_proc(void);

/**
 * @brief A task to handle all the events of USB class request.
 */
static void usbd_ctrl_class_cmd_proc(void);

/**
 * @brief A task to handle all the events of USB vendor request.
 */
static void usbd_ctrl_vnd_cmd_proc(void);

/**
 * @brief USB control EP0 responses stall packet.
 */
static void usbd_ctrl_ep0_set_stall(void);

/**
 * @brief USB control EP0 transfer, which implies the response from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_ctrl_ep0_wait_pkt_sent_completed(uint8_t byte_cnt);

/**
 * @brief USB control EP0 transfer, which implies the response with zero byte payload from device to
 * host.
 *
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_ctrl_ep0_wait_resp_sts_completed(void);

/**
 * @brief USB control EP0 transfer, which implies the response with zero byte payload from device to
 * host.
 *
 * @return bool The flag denotes normal completion if return true.
 */
static inline bool usbd_ctrl_ep0_wait_zero_len_pkt_sent_completed(void);

/**
 * @brief USB interrupt-in EP1 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_in_ep1_set_stall(bool b_flag);

/**
 * @brief USB interrupt-in EP1 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @param[in] b_await_xfer_done Await transfer done if the flag is asserted.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_in_ep1_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done);

/**
 * @brief USB interrupt-in EP2 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_in_ep2_set_stall(bool b_flag);

/**
 * @brief USB interrupt-in EP2 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @param[in] b_await_xfer_done Await transfer done if the flag is asserted.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_in_ep2_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done);

/**
 * @brief USB interrupt-in EP3 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_in_ep3_set_stall(bool b_flag);

/**
 * @brief USB interrupt-in EP3 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @param[in] b_await_xfer_done Await transfer done if the flag is asserted.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_in_ep3_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done);

/**
 * @brief USB interrupt-out EP4 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_out_ep4_set_stall(bool b_flag);

/**
 * @brief USB interrupt-out EP4 transfer, which implies the transfer from host to device.
 *
 * @param[out] p_byte_cnt A pointer to the variable for return the number of bytes has been received
 * from host to device.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_out_ep4_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt);

/**
 * @brief USB interrupt-in EP5 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_in_ep5_set_stall(bool b_flag);

/**
 * @brief USB interrupt-in EP5 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @param[in] b_await_xfer_done Await transfer done if the flag is asserted.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_in_ep5_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done);

/**
 * @brief USB interrupt-out EP6 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_out_ep6_set_stall(bool b_flag);

/**
 * @brief USB interrupt-out EP6 transfer, which implies the transfer from host to device.
 *
 * @param[out] p_byte_cnt A pointer to the variable for return the number of bytes has been received
 * from host to device.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_out_ep6_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt);

#if 0
/**
 * @brief USB interrupt-in EP7 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_in_ep7_set_stall(bool b_flag);

/**
 * @brief USB interrupt-in EP7 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @param[in] b_await_xfer_done Await transfer done if the flag is asserted.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_in_ep7_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done);

/**
 * @brief USB interrupt-out EP8 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_intrpt_out_ep8_set_stall(bool b_flag);

/**
 * @brief USB interrupt-out EP8 transfer, which implies the transfer from host to device.
 *
 * @param[out] p_byte_cnt A pointer to the variable for return the number of bytes has been received
 * from host to device.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_intrpt_out_ep8_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt);
#endif

/**
 * @brief USB bulk-in EP1 transfer, which implies the transfer from device to host. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[in] p_source A pointer to a buffer for transmit data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be transmit for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @param[in] byte_cnt The size in byte to transmit for a pre-defined end transfer.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep1_wait_payload_xfer_done(uint8_t *p_source,
                                                    uint8_t  sect_cnt,
                                                    uint8_t  byte_cnt);

/**
 * @brief USB bulk-in EP1 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_in_ep1_set_stall(bool b_flag);

/**
 * @brief USB bulk-in EP1 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep1_wait_pkt_sent_completed(uint8_t byte_cnt);

/**
 * @brief USB bulk-in EP2 transfer, which implies the transfer from device to host. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[in] p_source A pointer to a buffer for transmit data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be transmit for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @param[in] byte_cnt The size in byte to transmit for a pre-defined end transfer.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep2_wait_payload_xfer_done(uint8_t *p_source,
                                                    uint8_t  sect_cnt,
                                                    uint8_t  byte_cnt);

/**
 * @brief USB bulk-in EP2 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_in_ep2_set_stall(bool b_flag);

/**
 * @brief USB bulk-in EP2 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep2_wait_pkt_sent_completed(uint8_t byte_cnt);

/**
 * @brief USB bulk-in EP3 transfer, which implies the transfer from device to host. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[in] p_source A pointer to a buffer for transmit data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be transmit for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @param[in] byte_cnt The size in byte to transmit for a pre-defined end transfer.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep3_wait_payload_xfer_done(uint8_t *p_source,
                                                    uint8_t  sect_cnt,
                                                    uint8_t  byte_cnt);

/**
 * @brief USB bulk-in EP3 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_in_ep3_set_stall(bool b_flag);

/**
 * @brief USB bulk-in EP3 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep3_wait_pkt_sent_completed(uint8_t byte_cnt);

/**
 * @brief USB bulk-out EP4 transfer, which implies the transfer from host to device. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[out] p_dest A pointer to a buffer for acquire data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be acquired for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_out_ep4_wait_payload_xfer_done(uint8_t *p_dest, uint8_t sect_cnt);

/**
 * @brief USB bulk-out EP4 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_out_ep4_set_stall(bool b_flag);

/**
 * @brief USB bulk-out EP4 transfer, which implies the transfer from host to device.
 *
 * @param[out] p_byte_cnt A pointer to the variable for return the number of bytes have been
 * received from host to device.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_out_ep4_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt);

/**
 * @brief USB bulk-in EP5 transfer, which implies the transfer from device to host. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[in] p_source A pointer to a buffer for transmit data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be transmit for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @param[in] byte_cnt The size in byte to transmit for a pre-defined end transfer.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep5_wait_payload_xfer_done(uint8_t *p_source,
                                                    uint8_t  sect_cnt,
                                                    uint8_t  byte_cnt);

/**
 * @brief USB bulk-in EP5 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_in_ep5_set_stall(bool b_flag);

/**
 * @brief USB bulk-in EP5 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep5_wait_pkt_sent_completed(uint8_t byte_cnt);

/**
 * @brief USB bulk-out EP6 transfer, which implies the transfer from host to device. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[out] p_dest A pointer to a buffer for acquire data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be acquired for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_out_ep6_wait_payload_xfer_done(uint8_t *p_dest, uint8_t sect_cnt);

/**
 * @brief USB bulk-out EP6 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_out_ep6_set_stall(bool b_flag);

/**
 * @brief USB bulk-out EP6 transfer, which implies the transfer from host to device.
 *
 * @param[out] p_byte_cnt A pointer to the variable for return the number of bytes have been
 * received from host to device.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_out_ep6_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt);

#if 0
/**
 * @brief USB bulk-in EP7 transfer, which implies the transfer from device to host. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[in] p_source A pointer to a buffer for transmit data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be transmit for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @param[in] byte_cnt The size in byte to transmit for a pre-defined end transfer.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep7_wait_payload_xfer_done(uint8_t *p_source,
                                                    uint8_t  sect_cnt,
                                                    uint8_t  byte_cnt);

/**
 * @brief USB bulk-in EP7 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_in_ep7_set_stall(bool b_flag);

/**
 * @brief USB bulk-in EP7 transfer, which implies the transfer from device to host.
 *
 * @param[in] byte_cnt The number of byte to send from device to host.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_in_ep7_wait_pkt_sent_completed(uint8_t byte_cnt);

/**
 * @brief USB bulk-out EP8 transfer, which implies the transfer from host to device. The transfer
 * typically belongs to pre-defined end type.
 *
 * @param[out] p_dest A pointer to a buffer for acquire data bytes during transfer in pre-defined
 * end type.
 * @param[in] sect_cnt The length to be acquired for a pre-defined end transfer that unit is in
 * sector (typically 512 bytes).
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_out_ep8_wait_payload_xfer_done(uint8_t *p_dest, uint8_t sect_cnt);

/**
 * @brief USB bulk-out EP8 stall setting for hardcore.
 *
 * @param[in] b_flag Passthrough flag to denote requir on stall or off stall.
 */
static void usbd_bulk_out_ep8_set_stall(bool b_flag);

/**
 * @brief USB bulk-out EP8 transfer, which implies the transfer from host to device.
 *
 * @param[out] p_byte_cnt A pointer to the variable for return the number of bytes have been
 * received from host to device.
 * @return bool The flag denotes normal completion if return true.
 */
static bool usbd_bulk_out_ep8_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt);
#endif

/*******************************************************************************
 * Global functions
 ******************************************************************************/
/**
 * @brief This function is the implementation of FSUSB handler named in startup
 * code.
 */
void
FSUSB_IRQHandler(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;
    uint8_t              setup_pkt_len;
    uint32_t             status_bm;

    status_bm = fsusb_get_status_bm(p_base);
    fsusb_clear_status_bm(p_base, status_bm);
    if (status_bm & (1u << FSUSB_RESET_FLAG))
    {
        status_bm &= ~(1u << FSUSB_RESET_FLAG);
        p_state_ptr->b_bus_rst_immediately = true;
    }
    if (status_bm & (1u << FSUSB_CTRL_ARRIVAL_FLAG))
    {
        status_bm &= ~(1u << FSUSB_CTRL_ARRIVAL_FLAG);
        usb_dev_req_t *p_dev_req = &(p_state_ptr->dev_req);
        setup_pkt_len            = fsusb_get_ep0_rx_pkt_len(p_base);
        if (setup_pkt_len)
        {
            p_state_ptr->setup_pkt_len = setup_pkt_len;
            if (p_state_ptr->b_new_ctrl_setup_arrival)
            {
                p_state_ptr->b_ctrl_setup_duplication = true;
                p_dev_req                             = &(p_state_ptr->dev_req_abnormal);
            }
        }
        else
        {
            p_state_ptr->b_ctrl_setup_zero_len = true;
            p_dev_req                          = &(p_state_ptr->dev_req_abnormal);
        }
        p_state_ptr->b_new_ctrl_setup_arrival = true;
        p_dev_req->bmreq_type                 = fsusb_get_usb_dev_req_data_byte(p_base, 0u);
        p_dev_req->brequest                   = fsusb_get_usb_dev_req_data_byte(p_base, 1u);
        p_dev_req->wvalue.sb_type.lbyte       = fsusb_get_usb_dev_req_data_byte(p_base, 2u);
        p_dev_req->wvalue.sb_type.hbyte       = fsusb_get_usb_dev_req_data_byte(p_base, 3u);
        p_dev_req->windex.sb_type.lbyte       = fsusb_get_usb_dev_req_data_byte(p_base, 4u);
        p_dev_req->windex.sb_type.hbyte       = fsusb_get_usb_dev_req_data_byte(p_base, 5u);
        p_dev_req->wlength.sb_type.lbyte      = fsusb_get_usb_dev_req_data_byte(p_base, 6u);
        p_dev_req->wlength.sb_type.hbyte      = fsusb_get_usb_dev_req_data_byte(p_base, 7u);
        fsusb_clear_ep0_latched_st(p_base);
        if (!setup_pkt_len)
        {
            usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
        }
        if (255u != p_state_ptr->sema_hw_event)
        {
            osif_sema_post(&p_state_ptr->sema_hw_event);
        }
    }
    if (status_bm & (1u << FSUSB_EP1_TX_DONE_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP1_TX_DONE_FLAG);
        p_state_ptr->b_ep_sram3_job_in_que = false;
        p_state_ptr->b_ep1_sent_completed  = true;
    }
    if (status_bm & (1u << FSUSB_EP2_TX_DONE_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP2_TX_DONE_FLAG);
        p_state_ptr->b_ep_sram3_job_in_que = false;
        p_state_ptr->b_ep2_sent_completed  = true;
    }
    if (status_bm & (1u << FSUSB_EP3_TX_DONE_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP3_TX_DONE_FLAG);
        p_state_ptr->b_ep_sram3_job_in_que = false;
        p_state_ptr->b_ep3_sent_completed  = true;
    }
    if (status_bm & (1u << FSUSB_EP4_RX_RDY_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP4_RX_RDY_FLAG);
        if (!fsusb_get_ep4_stall_st(p_base) && fsusb_get_ep4_pkt_fetched_rdy(p_base))
        {
            p_state_ptr->ep4_rcvd_pkt_len = fsusb_get_ep4_rx_pkt_len(p_base);
            if (p_state_ptr->ep4_rcvd_pkt_len)
            {
                p_state_ptr->b_ep4_new_arrival = true;
                memcpy(p_state_ptr->p_ep4_buffer,
                       p_state_ptr->p_ep_shared_sram3_ptr,
                       p_state_ptr->ep4_rcvd_pkt_len);
                fsusb_clear_ep4_latched_st(p_base);
            }
        }
    }
    if (status_bm & (1u << FSUSB_EP5_TX_DONE_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP5_TX_DONE_FLAG);
        p_state_ptr->b_ep_sram1_job_in_que = false;
        p_state_ptr->b_ep5_sent_completed  = true;
    }
    if (status_bm & (1u << FSUSB_EP6_RX_RDY_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP6_RX_RDY_FLAG);
        if (!fsusb_get_ep6_stall_st(p_base) && fsusb_get_ep6_pkt_fetched_rdy(p_base))
        {
            p_state_ptr->ep6_rcvd_pkt_len = fsusb_get_ep6_rx_pkt_len(p_base);
            if (p_state_ptr->ep6_rcvd_pkt_len)
            {
                p_state_ptr->b_ep6_new_arrival = true;
                memcpy(p_state_ptr->p_ep6_buffer,
                       p_state_ptr->p_ep_shared_sram1_ptr,
                       p_state_ptr->ep6_rcvd_pkt_len);
                fsusb_clear_ep6_latched_st(p_base);
            }
        }
    }
#if 0
    if (status_bm & (1u << FSUSB_EP7_TX_DONE_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP7_TX_DONE_FLAG);
        p_state_ptr->b_ep_sram2_job_in_que = false;
        p_state_ptr->b_ep7_sent_completed  = true;
    }
    if (status_bm & (1u << FSUSB_EP8_RX_RDY_FLAG))
    {
        status_bm &= ~(1u << FSUSB_EP8_RX_RDY_FLAG);
        if (!fsusb_get_ep8_stall_st(p_base) && fsusb_get_ep8_pkt_fetched_rdy(p_base))
        {
            p_state_ptr->ep8_rcvd_pkt_len = fsusb_get_ep8_rx_pkt_len(p_base);
            if (p_state_ptr->ep8_rcvd_pkt_len)
            {
                p_state_ptr->b_ep8_new_arrival = true;
                memcpy(p_state_ptr->p_ep8_buffer,
                       p_state_ptr->p_ep_shared_sram2_ptr,
                       p_state_ptr->ep8_rcvd_pkt_len);
                fsusb_clear_ep8_latched_st(p_base);
            }
        }
    }
#endif
}

void
usbd_init(void)
{
    DEV_ASSERT(NULL == sp_usbd_state_ptr);
    usbd_dev_if_state_t      *p_state_ptr = &g_usbd_dev_if_state;
    FSUSB_t                  *p_base      = NULL;
    const usb_std_cfg_desc_t *p_cfg_desc  = NULL;
    status_t                  sts_for_sema;
    uint32_t                  source_clk_freq;

    /** Clear the state struct for this instance. */
    memset((void *)p_state_ptr, 0x00u, sizeof(usbd_dev_if_state_t));
    /** Save the runtime state structure pointer. */
    sp_usbd_state_ptr   = p_state_ptr;
    p_state_ptr->p_base = p_base = gp_fsusb_base[INST_FSUSB];
    p_state_ptr->p_irq_id        = &g_fsusb_irq_id[INST_FSUSB];
    p_state_ptr->p_clk_name      = &g_fsusb_clk_names[INST_FSUSB];

    /** Reset the PCC PLL about FSUSB, then await PLL state is locked. */
    clock_switch_pcc_freq_await_stable(*(p_state_ptr->p_clk_name));

    /* Get the LPSPI clock as configured in the clock manager */
    (void)clock_get_freq(*(p_state_ptr->p_clk_name), &source_clk_freq);
    DEV_ASSERT(0u < source_clk_freq);
    DEV_ASSERT(FEATURE_SCG_FIRC_FREQ1 == source_clk_freq);

    p_state_ptr->p_ep_shared_sram0_ptr = (uint8_t *)fsusb_get_sram0_buffer_address();
    p_state_ptr->p_ep0_buffer          = p_state_ptr->p_ep_shared_sram0_ptr;
    p_state_ptr->ep0_buf_size          = fsusb_get_sram0_buffer_size();
    p_state_ptr->p_ep_shared_sram3_ptr = (uint8_t *)fsusb_get_sram3_buffer_address();
    p_state_ptr->p_ep1_buffer          = p_state_ptr->p_ep_shared_sram3_ptr;
    p_state_ptr->ep1_buf_size          = fsusb_get_sram3_buffer_size();
    p_state_ptr->p_ep2_buffer          = p_state_ptr->p_ep_shared_sram3_ptr;
    p_state_ptr->ep2_buf_size          = fsusb_get_sram3_buffer_size();
    p_state_ptr->p_ep3_buffer          = p_state_ptr->p_ep_shared_sram3_ptr;
    p_state_ptr->ep3_buf_size          = fsusb_get_sram3_buffer_size();
    p_state_ptr->p_ep4_buffer          = p_state_ptr->p_ep_shared_sram3_ptr;
    p_state_ptr->ep4_buf_size          = fsusb_get_sram3_buffer_size();
    p_state_ptr->p_ep_shared_sram1_ptr = (uint8_t *)fsusb_get_sram1_buffer_address();
    p_state_ptr->p_ep5_buffer          = p_state_ptr->p_ep_shared_sram1_ptr;
    p_state_ptr->ep5_buf_size          = fsusb_get_sram1_buffer_size();
    p_state_ptr->p_ep6_buffer          = p_state_ptr->p_ep_shared_sram1_ptr;
    p_state_ptr->ep6_buf_size          = fsusb_get_sram1_buffer_size();
#if 0
    p_state_ptr->p_ep_shared_sram2_ptr = (uint8_t *)fsusb_get_sram2_buffer_address();
    p_state_ptr->p_ep7_buffer          = p_state_ptr->p_ep_shared_sram2_ptr;
    p_state_ptr->ep7_buf_size          = fsusb_get_sram2_buffer_size();
    p_state_ptr->p_ep8_buffer          = p_state_ptr->p_ep_shared_sram2_ptr;
    p_state_ptr->ep8_buf_size          = fsusb_get_sram2_buffer_size();
#endif
    p_state_ptr->bus_cfg_state         = USBD_DEFAULT_STATE;
    p_state_ptr->cur_cfg_val           = 0u;
    p_state_ptr->dev_feat_status       = USBD_FEAT_BUS_POWERED;
    p_cfg_desc                         = &(s_usbd_fs_hid_cfg_desc.usb_cfg_desc);
    if (p_cfg_desc->bmattrib & USB_CFG_ATTR_SELF_POWERED)
    {
        p_state_ptr->dev_feat_status |= USBD_FEAT_SELF_POWERED;
    }
    if (p_cfg_desc->bmattrib & USB_CFG_ATTR_REMOTE_WAKEUP)
    {
        p_state_ptr->dev_feat_status |= USBD_FEAT_REMOTE_WAKEUP;
    }
    p_state_ptr->b_full_spd_mode          = true;
    p_state_ptr->b_bus_power_on           = false;
    p_state_ptr->b_bus_rst_immediately    = false;
    p_state_ptr->b_new_ctrl_setup_arrival = false;
    p_state_ptr->b_ctrl_setup_duplication = false;
    p_state_ptr->b_ctrl_setup_zero_len    = false;
    p_state_ptr->b_ep_sram1_job_in_que    = false;
#if 0
    p_state_ptr->b_ep_sram2_job_in_que    = false;
#endif
    p_state_ptr->b_ep_sram3_job_in_que    = false;
    p_state_ptr->b_ep1_sent_completed     = false;
    p_state_ptr->b_ep2_sent_completed     = false;
    p_state_ptr->b_ep3_sent_completed     = false;
    p_state_ptr->b_ep4_new_arrival        = false;
    p_state_ptr->b_ep5_sent_completed     = false;
    p_state_ptr->b_ep6_new_arrival        = false;
#if 0
    p_state_ptr->b_ep7_sent_completed     = false;
    p_state_ptr->b_ep8_new_arrival        = false;
#endif
    p_state_ptr->setup_pkt_len            = 0u;
    p_state_ptr->ep4_rcvd_pkt_len         = 0u;
    p_state_ptr->ep6_rcvd_pkt_len         = 0u;
    p_state_ptr->ep8_rcvd_pkt_len         = 0u;
    p_state_ptr->sram1_job_queue[1u].ep_num = p_state_ptr->sram1_job_queue[0u].ep_num = 0xFFu;
    p_state_ptr->sram1_job_queue[1u].p_source_buffer =
        p_state_ptr->sram1_job_queue[0u].p_source_buffer = NULL;
    p_state_ptr->sram1_job_queue[1u].source_buf_len =
        p_state_ptr->sram1_job_queue[0u].source_buf_len = 0u;
#if 0
    p_state_ptr->sram2_job_queue[1u].ep_num = p_state_ptr->sram2_job_queue[0u].ep_num = 0xFFu;
    p_state_ptr->sram2_job_queue[1u].p_source_buffer =
        p_state_ptr->sram2_job_queue[0u].p_source_buffer = NULL;
    p_state_ptr->sram2_job_queue[1u].source_buf_len =
        p_state_ptr->sram2_job_queue[0u].source_buf_len = 0u;
#endif
    p_state_ptr->sram3_job_queue[1u].ep_num = p_state_ptr->sram3_job_queue[0u].ep_num = 0xFFu;
    p_state_ptr->sram3_job_queue[1u].p_source_buffer =
        p_state_ptr->sram3_job_queue[0u].p_source_buffer = NULL;
    p_state_ptr->sram3_job_queue[1u].source_buf_len =
        p_state_ptr->sram3_job_queue[0u].source_buf_len = 0u;
    p_state_ptr->hid_report_id0_idle_rate =
        0u; /* with a 4 millisecond resolution. 0 is indefinite */
    p_state_ptr->sema_hw_event = 255u;
    p_state_ptr->sema_hid      = 255u;
    sts_for_sema               = osif_sema_create(&p_state_ptr->sema_hid, 0u);
    DEV_ASSERT(STATUS_SUCCESS == sts_for_sema);
    p_state_ptr->sema_monitor = 255u;
    sts_for_sema              = osif_sema_create(&p_state_ptr->sema_monitor, 0u);
    DEV_ASSERT(STATUS_SUCCESS == sts_for_sema);
    p_state_ptr->monitor_tagged_tick = osif_get_milliseconds();
    fsusb_init(p_base);
    /** Enable the event alert gate of the FSUSB module. */
    fsusb_set_intrpt_enable_bit(p_base, FSUSB_INT_SETUP_BM | FSUSB_INT_RESET_BM);
    fsusb_set_intrpt_enable_bit(p_base, FSUSB_INT_EP1TX_BM);
    /** Enable the vector interrupt. */
    int_enable_irq(*(p_state_ptr->p_irq_id));
}

void
usbd_deinit(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    /** Disable the event alert gate of the FSUSB module. */
    fsusb_clear_intrpt_enable_bit(p_base, FSUSB_INT_ALL_BM);
    /** Disable the vector interrupt. */
    int_disable_irq(*(p_state_ptr->p_irq_id));
    /** Clear the state pointer. */
    sp_usbd_state_ptr = NULL;
}

void
usbd_install_ep4_rcv_buffer(uint8_t *p_buf, uint32_t buf_size)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    p_state_ptr->p_ep4_buffer        = p_buf;
    p_state_ptr->ep4_buf_size        = buf_size;
}

void
usbd_install_ep6_rcv_buffer(uint8_t *p_buf, uint32_t buf_size)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    p_state_ptr->p_ep6_buffer        = p_buf;
    p_state_ptr->ep6_buf_size        = buf_size;
}

#if 0
void
usbd_install_ep8_rcv_buffer(uint8_t *p_buf, uint32_t buf_size)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    p_state_ptr->p_ep8_buffer        = p_buf;
    p_state_ptr->ep8_buf_size        = buf_size;
}
#endif

void
usbd_device_reset(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t      *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t                  *p_base      = p_state_ptr->p_base;
    const usb_std_cfg_desc_t *p_cfg_desc  = NULL;

    p_state_ptr->bus_cfg_state   = USBD_DEFAULT_STATE;
    p_state_ptr->cur_cfg_val     = 0u;
    p_state_ptr->dev_feat_status = USBD_FEAT_BUS_POWERED;
    p_cfg_desc                   = &(s_usbd_fs_hid_cfg_desc.usb_cfg_desc);
    if (p_cfg_desc->bmattrib & USB_CFG_ATTR_SELF_POWERED)
    {
        p_state_ptr->dev_feat_status |= USBD_FEAT_SELF_POWERED;
    }
    if (p_cfg_desc->bmattrib & USB_CFG_ATTR_REMOTE_WAKEUP)
    {
        p_state_ptr->dev_feat_status |= USBD_FEAT_REMOTE_WAKEUP;
    }
    p_state_ptr->b_bus_power_on           = false;
    p_state_ptr->b_bus_rst_immediately    = false;
    p_state_ptr->b_new_ctrl_setup_arrival = false;
    p_state_ptr->b_ctrl_setup_duplication = false;
    p_state_ptr->b_ctrl_setup_zero_len    = false;
    p_state_ptr->b_ep_sram1_job_in_que    = false;
#if 0
    p_state_ptr->b_ep_sram2_job_in_que    = false;
#endif
    p_state_ptr->b_ep_sram3_job_in_que    = false;
    p_state_ptr->b_ep1_sent_completed     = false;
    p_state_ptr->b_ep2_sent_completed     = false;
    p_state_ptr->b_ep3_sent_completed     = false;
    p_state_ptr->b_ep4_new_arrival        = false;
    p_state_ptr->b_ep5_sent_completed     = false;
    p_state_ptr->b_ep6_new_arrival        = false;
#if 0
    p_state_ptr->b_ep7_sent_completed     = false;
    p_state_ptr->b_ep8_new_arrival        = false;
#endif
    p_state_ptr->sram1_job_queue[1u].ep_num = p_state_ptr->sram1_job_queue[0u].ep_num = 0xFFu;
    p_state_ptr->sram1_job_queue[1u].p_source_buffer =
        p_state_ptr->sram1_job_queue[0u].p_source_buffer = NULL;
    p_state_ptr->sram1_job_queue[1u].source_buf_len =
        p_state_ptr->sram1_job_queue[0u].source_buf_len = 0u;
#if 0
    p_state_ptr->sram2_job_queue[1u].ep_num = p_state_ptr->sram2_job_queue[0u].ep_num = 0xFFu;
    p_state_ptr->sram2_job_queue[1u].p_source_buffer =
        p_state_ptr->sram2_job_queue[0u].p_source_buffer = NULL;
    p_state_ptr->sram2_job_queue[1u].source_buf_len =
        p_state_ptr->sram2_job_queue[0u].source_buf_len = 0u;
#endif
    p_state_ptr->sram3_job_queue[1u].ep_num = p_state_ptr->sram3_job_queue[0u].ep_num = 0xFFu;
    p_state_ptr->sram3_job_queue[1u].p_source_buffer =
        p_state_ptr->sram3_job_queue[0u].p_source_buffer = NULL;
    p_state_ptr->sram3_job_queue[1u].source_buf_len =
        p_state_ptr->sram3_job_queue[0u].source_buf_len = 0u;
    p_state_ptr->hid_report_id0_idle_rate =
        0u; /* with a 4 millisecond resolution. 0 is indefinite */
    fsusb_init(p_base);
    fsusb_set_intrpt_enable_bit(p_base, FSUSB_INT_SETUP_BM | FSUSB_INT_RESET_BM);
    fsusb_set_intrpt_enable_bit(p_base, FSUSB_INT_EP1TX_BM);
}

bool
usbd_check_bus_rst_immediately(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    status_t             sts_for_sema;

    if (p_state_ptr->b_bus_rst_immediately)
    {
        p_state_ptr->b_bus_rst_immediately = false;
        printf("Requested to USB bus reset!\n");
        usbd_device_reset();
        sts_for_sema = osif_sema_create(&p_state_ptr->sema_hw_event, 0u);
        DEV_ASSERT(STATUS_SUCCESS == sts_for_sema);
        /** Await maximum 1ms till steady if no setup packet income. */
        sts_for_sema = osif_sema_wait(&p_state_ptr->sema_hw_event, 1u);
        osif_sema_destroy(&p_state_ptr->sema_hw_event);
        p_state_ptr->sema_hw_event = 255u;
        if (STATUS_SUCCESS == sts_for_sema)
        {
            printf("Occurring setup event after USB reset.\n");
        }
        else if (STATUS_TIMEOUT == sts_for_sema)
        {
            printf("Awaiting till steady after USB reset.\n");
        }
        else
        {
            printf("Error! %s-%d.\n", __FILE__, __LINE__);
        }
        return true;
    }
    return false;
}

bool
usbd_detect_bus_acitve(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    if (fsusb_get_bus_connect_rdy(p_base))
    {
        return true;
    }
    return false;
}
bool
usbd_check_bus_power_on(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    if (!p_state_ptr->b_bus_power_on)
    {
        if (usbd_detect_bus_acitve())
        {
            p_state_ptr->b_bus_power_on = true;
            printf("Bus power is on.\n");
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool
usbd_check_ctrl_setup_duplication(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    if (p_state_ptr->b_ctrl_setup_duplication)
    {
        p_state_ptr->b_ctrl_setup_duplication = false;
        printf("Error! %s-%d.\n", __FILE__, __LINE__);
        printf("Control setup duplication!\n");
        printf("Request: [%02X %02X %02X %02X %02X %02X %02X %02X]h.\n",
               p_state_ptr->dev_req_abnormal.bmreq_type,
               p_state_ptr->dev_req_abnormal.brequest,
               p_state_ptr->dev_req_abnormal.wvalue.sb_type.lbyte,
               p_state_ptr->dev_req_abnormal.wvalue.sb_type.hbyte,
               p_state_ptr->dev_req_abnormal.windex.sb_type.lbyte,
               p_state_ptr->dev_req_abnormal.windex.sb_type.hbyte,
               p_state_ptr->dev_req_abnormal.wlength.sb_type.lbyte,
               p_state_ptr->dev_req_abnormal.wlength.sb_type.hbyte);
        return true;
    }
    return false;
}

bool
usbd_check_ctrl_setup_zero_length(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    if (p_state_ptr->b_ctrl_setup_zero_len)
    {
        p_state_ptr->b_ctrl_setup_zero_len = false;
        printf("Error! %s-%d.\n", __FILE__, __LINE__);
        printf("Control setup zero length!\n");
        printf("Request: [%02X %02X %02X %02X %02X %02X %02X %02X]h.\n",
               p_state_ptr->dev_req_abnormal.bmreq_type,
               p_state_ptr->dev_req_abnormal.brequest,
               p_state_ptr->dev_req_abnormal.wvalue.sb_type.lbyte,
               p_state_ptr->dev_req_abnormal.wvalue.sb_type.hbyte,
               p_state_ptr->dev_req_abnormal.windex.sb_type.lbyte,
               p_state_ptr->dev_req_abnormal.windex.sb_type.hbyte,
               p_state_ptr->dev_req_abnormal.wlength.sb_type.lbyte,
               p_state_ptr->dev_req_abnormal.wlength.sb_type.hbyte);
        return true;
    }
    return false;
}

void
usbd_ctrl_pipe_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    if (p_state_ptr->b_new_ctrl_setup_arrival)
    {
        p_state_ptr->b_new_ctrl_setup_arrival = false;
        usb_dev_req_t *p_dev_req              = &(p_state_ptr->dev_req);
#if 0
        printf("Control setup arrival. Length: %Xh(%d).\n",
               p_state_ptr->setup_pkt_len,
               p_state_ptr->setup_pkt_len);
        printf("Request: [%02X %02X %02X %02X %02X %02X %02X %02X]h.\n",
               p_dev_req->bmreq_type,
               p_dev_req->brequest,
               p_dev_req->wvalue.sb_type.lbyte,
               p_dev_req->wvalue.sb_type.hbyte,
               p_dev_req->windex.sb_type.lbyte,
               p_dev_req->windex.sb_type.hbyte,
               p_dev_req->wlength.sb_type.lbyte,
               p_dev_req->wlength.sb_type.hbyte);
#endif
        if (USB_RT_TYPE_STANDARD == (p_dev_req->bmreq_type & USB_RT_TYPE_MASK))
        {
            /** USB defined standard command */
            usbd_ctrl_std_cmd_proc();
        }
        else if (USB_RT_TYPE_CLASS == (p_dev_req->bmreq_type & USB_RT_TYPE_MASK))
        {
            /** USB defined specified class command */
            usbd_ctrl_class_cmd_proc();
        }
        else if (USB_RT_TYPE_VENDOR == (p_dev_req->bmreq_type & USB_RT_TYPE_MASK))
        {
            /** Vendor defined vendor class control command */
            usbd_ctrl_vnd_cmd_proc();
        }
        else
        {
            /** Unsupport command */
            usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
        }
        /** [Start of Statement> */
        /** Todo: Registers setting if necessary. */
        /**^ <End of Statement] */
    }
}

void
usbd_intrpt_out_pipe_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              group_cntr;

#if 1
    for (group_cntr = 0u; 2u > group_cntr; group_cntr++)
    {
        if ((0u == group_cntr) && p_state_ptr->b_ep4_new_arrival)
        {
            p_state_ptr->b_ep4_new_arrival = false;
            usbd_pipe_sram3_acquire_job(
                USBD_EP_NB4, p_state_ptr->p_ep4_buffer, p_state_ptr->ep4_rcvd_pkt_len);
        }
        if ((1u == group_cntr) && p_state_ptr->b_ep6_new_arrival)
        {
            p_state_ptr->b_ep6_new_arrival = false;
            usbd_pipe_sram1_acquire_job(
                USBD_EP_NB6, p_state_ptr->p_ep6_buffer, p_state_ptr->ep6_rcvd_pkt_len);
        }
    }
#endif
#if 0
    for (group_cntr = 0u; 3u > group_cntr; group_cntr++)
    {
        if ((0u == group_cntr) && p_state_ptr->b_ep4_new_arrival)
        {
            p_state_ptr->b_ep4_new_arrival = false;
            usbd_pipe_sram3_acquire_job(
                USBD_EP_NB4, p_state_ptr->p_ep4_buffer, p_state_ptr->ep4_rcvd_pkt_len);
        }
        if ((1u == group_cntr) && p_state_ptr->b_ep6_new_arrival)
        {
            p_state_ptr->b_ep6_new_arrival = false;
            usbd_pipe_sram1_acquire_job(
                USBD_EP_NB6, p_state_ptr->p_ep6_buffer, p_state_ptr->ep6_rcvd_pkt_len);
        }
        if ((0u == group_cntr) && p_state_ptr->b_ep8_new_arrival)
        {
            p_state_ptr->b_ep8_new_arrival = false;
            usbd_pipe_sram2_acquire_job(
                USBD_EP_NB8, p_state_ptr->p_ep4_buffer, p_state_ptr->ep4_rcvd_pkt_len);
        }
    }
#endif
}

void
usbd_intrpt_in_pipe_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;
    uint8_t              group_cntr;
    uint8_t              ep_num;
    uint8_t              tmp_cntr;
    uint8_t              source_buf_len;
    uint8_t             *p_source_buf = NULL;

#if 1
    for (group_cntr = 0u; 2u > group_cntr; group_cntr++)
#endif
#if 0
    for (group_cntr = 0u; 3u > group_cntr; group_cntr++)
#endif
    {
        if (0u == group_cntr)
        {

            if (0xFFu == p_state_ptr->sram3_job_queue[0u].ep_num)
            {
                continue;
            }
            ep_num = p_state_ptr->sram3_job_queue[0u].ep_num;
            if (USBD_EP_NB1 == ep_num)
            {
                if (fsusb_get_ep1_stall_st(p_base))
                {
                    printf("EP1 stall!\n");
                    continue;
                }
            }
            else if (USBD_EP_NB2 == ep_num)
            {
                if (fsusb_get_ep2_stall_st(p_base))
                {
                    printf("EP2 stall!\n");
                    continue;
                }
            }
            else if (USBD_EP_NB3 == ep_num)
            {
                if (fsusb_get_ep3_stall_st(p_base))
                {
                    printf("EP3 stall!\n");
                    continue;
                }
            }
            else
            {
                printf("Error! %s-%d.\n", __FILE__, __LINE__);
                continue;
            }
            p_source_buf   = p_state_ptr->sram3_job_queue[0u].p_source_buffer;
            source_buf_len = p_state_ptr->sram3_job_queue[0u].source_buf_len;
            p_state_ptr->sram3_job_queue[0u].ep_num = p_state_ptr->sram3_job_queue[1u].ep_num;
            p_state_ptr->sram3_job_queue[0u].p_source_buffer =
                p_state_ptr->sram3_job_queue[1u].p_source_buffer;
            p_state_ptr->sram3_job_queue[0u].source_buf_len =
                p_state_ptr->sram3_job_queue[1u].source_buf_len;
            p_state_ptr->sram3_job_queue[1u].ep_num          = 0xFFu;
            p_state_ptr->sram3_job_queue[1u].p_source_buffer = NULL;
            p_state_ptr->sram3_job_queue[1u].source_buf_len  = 0u;
            if (NULL != p_source_buf)
            {
                memcpy(p_state_ptr->p_ep_shared_sram3_ptr, p_source_buf, source_buf_len);
            }
            if (USBD_EP_NB1 == ep_num)
            {
                usbd_intrpt_in_ep1_wait_pkt_sent_completed(source_buf_len, false);
            }
            else if (USBD_EP_NB2 == ep_num)
            {
                usbd_intrpt_in_ep2_wait_pkt_sent_completed(source_buf_len, false);
            }
            else
            {
                usbd_intrpt_in_ep3_wait_pkt_sent_completed(source_buf_len, false);
            }
        }
        else if (1u == group_cntr)
        {
            if (0xFFu == p_state_ptr->sram1_job_queue[0u].ep_num)
            {
                continue;
            }
            ep_num = p_state_ptr->sram1_job_queue[0u].ep_num;
            if (USBD_EP_NB5 == ep_num)
            {
                if (fsusb_get_ep5_stall_st(p_base))
                {
                    printf("EP5 stall!\n");
                    continue;
                }
            }
            else
            {
                printf("Error! %s-%d.\n", __FILE__, __LINE__);
                continue;
            }
            p_source_buf   = p_state_ptr->sram1_job_queue[0u].p_source_buffer;
            source_buf_len = p_state_ptr->sram1_job_queue[0u].source_buf_len;
            p_state_ptr->sram1_job_queue[0u].ep_num = p_state_ptr->sram1_job_queue[1u].ep_num;
            p_state_ptr->sram1_job_queue[0u].p_source_buffer =
                p_state_ptr->sram1_job_queue[1u].p_source_buffer;
            p_state_ptr->sram1_job_queue[0u].source_buf_len =
                p_state_ptr->sram1_job_queue[1u].source_buf_len;
            p_state_ptr->sram1_job_queue[1u].ep_num          = 0xFFu;
            p_state_ptr->sram1_job_queue[1u].p_source_buffer = NULL;
            p_state_ptr->sram1_job_queue[1u].source_buf_len  = 0u;
            if (NULL != p_source_buf)
            {
                memcpy(p_state_ptr->p_ep_shared_sram1_ptr, p_source_buf, source_buf_len);
            }
            if (USBD_EP_NB5 == ep_num)
            {
                usbd_intrpt_in_ep5_wait_pkt_sent_completed(source_buf_len, false);
            }
        }
#if 0
        else
        {
            if (0xFFu == p_state_ptr->sram2_job_queue[0u].ep_num)
            {
                continue;
            }
            ep_num = p_state_ptr->sram2_job_queue[0u].ep_num;
            if (USBD_EP_NB5 == ep_num)
            {
                if (fsusb_get_ep7_stall_st(p_base))
                {
                    printf("EP7 stall!\n");
                    continue;
                }
            }
            else
            {
                printf("Error! %s-%d.\n", __FILE__, __LINE__);
                continue;
            }
            p_source_buf   = p_state_ptr->sram2_job_queue[0u].p_source_buffer;
            source_buf_len = p_state_ptr->sram2_job_queue[0u].source_buf_len;
            p_state_ptr->sram2_job_queue[0u].ep_num = p_state_ptr->sram2_job_queue[1u].ep_num;
            p_state_ptr->sram2_job_queue[0u].p_source_buffer =
                p_state_ptr->sram2_job_queue[1u].p_source_buffer;
            p_state_ptr->sram2_job_queue[0u].source_buf_len =
                p_state_ptr->sram2_job_queue[1u].source_buf_len;
            p_state_ptr->sram2_job_queue[1u].ep_num          = 0xFFu;
            p_state_ptr->sram2_job_queue[1u].p_source_buffer = NULL;
            p_state_ptr->sram2_job_queue[1u].source_buf_len  = 0u;
            if (NULL != p_source_buf)
            {
                memcpy(p_state_ptr->p_ep_shared_sram2_ptr, p_source_buf, source_buf_len);
            }
            if (USBD_EP_NB7 == ep_num)
            {
                usbd_intrpt_in_ep7_wait_pkt_sent_completed(source_buf_len, false);
            }
        }
#endif
    }
}

void
usbd_bulk_out_pipe_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    if (USBD_CONFIGURED_STATE != p_state_ptr->bus_cfg_state)
    {
        return;
    }
    /** Todo. */
    printf("Todo! %s-%d.\n", __FILE__, __LINE__);
}

void
usbd_bulk_in_pipe_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    if (USBD_CONFIGURED_STATE != p_state_ptr->bus_cfg_state)
    {
        return;
    }
    /** Todo. */
    printf("Todo! %s-%d.\n", __FILE__, __LINE__);
}

void
usbd_status_monitor_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    osif_sema_wait(&p_state_ptr->sema_monitor, 0u);
    if (1000u <= (osif_get_milliseconds() - p_state_ptr->monitor_tagged_tick))
    {
        uint16_t u16Val;
        p_state_ptr->monitor_tagged_tick = osif_get_milliseconds();
        u16Val                           = fsusb_get_rc_trim_deviation_val(p_base);
        if (u16Val & 0x8000u)
        {
            u16Val &= (~0x8000u);
            printf("RC slow/lag: %d\n", u16Val);
        }
        else
        {
            printf("RC fast/lead: %d\n", u16Val);
        }
    }
}

void __attribute__((weak))
usbd_pipe_sram1_acquire_job(uint8_t group_ep_num, uint8_t *p_income, uint32_t byte_cnt)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    usbd_pipe_sram1_queue_job(USBD_EP_NB5, p_income, byte_cnt);
}

void
usbd_pipe_sram1_queue_job(uint8_t group_ep_num, uint8_t *p_source, uint32_t byte_cnt)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              queue_cntr;

    for (queue_cntr = 0u; 2u > queue_cntr; queue_cntr++)
    {
        if (0xFFu == p_state_ptr->sram1_job_queue[queue_cntr].ep_num)
        {
            p_state_ptr->sram1_job_queue[queue_cntr].ep_num          = group_ep_num;
            p_state_ptr->sram1_job_queue[queue_cntr].p_source_buffer = p_source;
            p_state_ptr->sram1_job_queue[queue_cntr].source_buf_len  = byte_cnt;
            break;
        }
    }
}

#if 0
void __attribute__((weak))
usbd_pipe_sram2_acquire_job(uint8_t group_ep_num, uint8_t *p_income, uint32_t byte_cnt)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    usbd_pipe_sram2_queue_job(USBD_EP_NB7, p_income, byte_cnt);
}

void
usbd_pipe_sram2_queue_job(uint8_t group_ep_num, uint8_t *p_source, uint32_t byte_cnt)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              queue_cntr;

    for (queue_cntr = 0u; 2u > queue_cntr; queue_cntr++)
    {
        if (0xFFu == p_state_ptr->sram1_job_queue[queue_cntr].ep_num)
        {
            p_state_ptr->sram2_job_queue[queue_cntr].ep_num          = group_ep_num;
            p_state_ptr->sram2_job_queue[queue_cntr].p_source_buffer = p_source;
            p_state_ptr->sram2_job_queue[queue_cntr].source_buf_len  = byte_cnt;
            break;
        }
    }
}
#endif

void __attribute__((weak))
usbd_pipe_sram3_acquire_job(uint8_t group_ep_num, uint8_t *p_income, uint32_t byte_cnt)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;

    usbd_pipe_sram3_queue_job(USBD_EP_NB1, p_income, byte_cnt);
}

void
usbd_pipe_sram3_queue_job(uint8_t group_ep_num, uint8_t *p_source, uint32_t byte_cnt)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              queue_cntr;

    for (queue_cntr = 0u; 2u > queue_cntr; queue_cntr++)
    {
        if (0xFFu == p_state_ptr->sram3_job_queue[queue_cntr].ep_num)
        {
            p_state_ptr->sram3_job_queue[queue_cntr].ep_num          = group_ep_num;
            p_state_ptr->sram3_job_queue[queue_cntr].p_source_buffer = p_source;
            p_state_ptr->sram3_job_queue[queue_cntr].source_buf_len  = byte_cnt;
            break;
        }
    }
}

/*******************************************************************************
 * Static functions
 ******************************************************************************/
static void
usbd_configuration_reset(void)
{
    usbd_dev_if_state_t      *p_state_ptr = sp_usbd_state_ptr;
    const usb_std_cfg_desc_t *p_cfg_desc  = NULL;

    p_state_ptr->bus_cfg_state   = USBD_DEFAULT_STATE;
    p_state_ptr->cur_cfg_val     = 0u;
    p_state_ptr->dev_feat_status = USBD_FEAT_BUS_POWERED;
    p_cfg_desc                   = &(s_usbd_fs_hid_cfg_desc.usb_cfg_desc);
    if (p_cfg_desc->bmattrib & USB_CFG_ATTR_SELF_POWERED)
    {
        p_state_ptr->dev_feat_status |= USBD_FEAT_SELF_POWERED;
    }
    if (p_cfg_desc->bmattrib & USB_CFG_ATTR_REMOTE_WAKEUP)
    {
        p_state_ptr->dev_feat_status |= USBD_FEAT_REMOTE_WAKEUP;
    }
}

static void
usbd_ctrl_std_cmd_proc(void)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;
    usb_dev_req_t       *p_dev_req   = &(p_state_ptr->dev_req);
    const uint8_t       *p_desc      = NULL;
    uint16_t             byte_cnt;
    uint16_t             req_len;
    uint16_t             xfer_len;

    switch (p_dev_req->brequest)
    {
        case USB_REQ_GET_STATUS:
        {
            switch (p_dev_req->bmreq_type)
            {
                case (USB_RT_DIR_DEV2HOST | USB_RT_TYPE_STANDARD | USB_RT_RECIPIENT_DEVICE):
                {
                    p_state_ptr->p_ep0_buffer[0u] = (uint8_t)p_state_ptr->dev_feat_status;
                }
                break;
                case (USB_RT_DIR_DEV2HOST | USB_RT_TYPE_STANDARD | USB_RT_RECIPIENT_INTERFACE):
                {
                    if (p_dev_req->windex.sb_type.lbyte)
                    {
                        usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                        return;
                    }
                    p_state_ptr->p_ep0_buffer[0u] = 0x00u;
                }
                break;
                case (USB_RT_DIR_DEV2HOST | USB_RT_TYPE_STANDARD | USB_RT_RECIPIENT_ENDPOINT):
                {
                    if ((USBD_EP0_DO == p_dev_req->windex.sb_type.lbyte) ||
                        (USBD_EP0_DI == p_dev_req->windex.sb_type.lbyte))
                    {
                        p_state_ptr->p_ep0_buffer[0u] = fsusb_get_ep0_stall_st(p_base);
                    }
                    else if (USBD_EP1_DI == p_dev_req->windex.sb_type.lbyte)
                    {
                        p_state_ptr->p_ep0_buffer[0u] = fsusb_get_ep1_stall_st(p_base);
                    }
                    else
                    {
                        usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                        return;
                    }
                    p_state_ptr->p_ep0_buffer[1u] = 0x00u;
                }
                break;
                default:
                {
                    usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                }
                    return;
            }
            usbd_ctrl_ep0_wait_pkt_sent_completed(2u);
        }
        break;
        case USB_REQ_CLEAR_FEATURE:
        {
            if ((USB_RT_RECIPIENT_DEVICE == (p_dev_req->bmreq_type & USB_RT_RECIPIENT_MASK)) &&
                (USB_REQ_FEAT_DEV_REMOTE_WAKEUP == p_dev_req->wvalue.sb_type.lbyte))
            {
                p_state_ptr->dev_feat_status &= ~USBD_FEAT_REMOTE_WAKEUP;
            }
            else if ((USB_RT_RECIPIENT_ENDPOINT ==
                      (p_dev_req->bmreq_type & USB_RT_RECIPIENT_MASK)) &&
                     (USB_REQ_FEAT_ENDPOINT_HALT == p_dev_req->wvalue.sb_type.lbyte))
            {
                if (USBD_EP1_DI == p_dev_req->windex.sb_type.lbyte)
                {
                    usbd_intrpt_in_ep1_set_stall(false);
                }
                else
                {
                    usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                    return;
                }
            }
            else
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            usbd_ctrl_ep0_wait_resp_sts_completed();
        }
        break;
        case USB_REQ_SET_FEATURE:
        {
            if (USB_RT_RECIPIENT_DEVICE == (p_dev_req->bmreq_type & USB_RT_RECIPIENT_MASK))
            {
                if (USB_REQ_FEAT_DEV_REMOTE_WAKEUP == p_dev_req->wvalue.sb_type.lbyte)
                {
                    p_state_ptr->dev_feat_status |= USBD_FEAT_REMOTE_WAKEUP;
#if 0
                }
                else if (USB_REQ_FEAT_TEST_MODE == p_dev_req->wvalue.sb_type.lbyte)
                {
                    if (0x01u == p_dev_req->windex.sb_type.hbyte)
                    {
                        // OP_MODE1 | TEST_J | HW_TEST_MODE
                    }
                    else if (0x02u == p_dev_req->wvalue.sb_type.hbyte)
                    {
                        // OP_MODE1 | TEST_K | HW_TEST_MODE
                    }
                    else if (0x03u == p_dev_req->wvalue.sb_type.hbyte)
                    {
                        // TEST_SE0_NAK | HW_TEST_MODE
                    }
                    else if (0x04 == p_dev_req->wvalue.sb_type.hbyte)
                    {
                        // TEST_PACKET | HW_TEST_MODE
                    }
                    else
                    {
                        usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                        return;
                    }
                    // usbd_ctrl_ep0_wait_resp_sts_completed();
                    // break;
#endif
                }
                else
                {
                    usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                    return;
                }
            }
            else if ((USB_RT_RECIPIENT_ENDPOINT ==
                      (p_dev_req->bmreq_type & USB_RT_RECIPIENT_MASK)) &&
                     (USB_REQ_FEAT_ENDPOINT_HALT == p_dev_req->wvalue.sb_type.lbyte))
            {
                if (USBD_EP1_DI == p_dev_req->windex.sb_type.lbyte)
                {
                    usbd_intrpt_in_ep1_set_stall(true);
                }
                else
                {
                    usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                    return;
                }
            }
            else
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            usbd_ctrl_ep0_wait_resp_sts_completed();
        }
        break;
        case USB_REQ_SET_ADDRESS:
        {
            if (USBD_DEFAULT_STATE == p_state_ptr->bus_cfg_state)
            {
                p_state_ptr->bus_cfg_state = USBD_ADDRESSED_STATE;
            }
            else if ((USBD_ADDRESSED_STATE == p_state_ptr->bus_cfg_state) &&
                     (0u == p_dev_req->wvalue.sb_type.lbyte))
            {
#if 1
                printf("Assign address again! Configuration value: %Xh(%d).\n",
                       p_state_ptr->cur_cfg_val,
                       p_state_ptr->cur_cfg_val);
#endif
                DEV_ASSERT(0u == p_state_ptr->cur_cfg_val);
                p_state_ptr->bus_cfg_state = USBD_DEFAULT_STATE;
            }
            else if (USBD_CONFIGURED_STATE == p_state_ptr->bus_cfg_state)
            {
#if 1
                printf("Assign address after configured!\n");
#endif
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            else
            {
#if 1
                printf("Reject to assign address!\n");
#endif
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            if (usbd_ctrl_ep0_wait_resp_sts_completed())
            {
                fsusb_set_usb_dev_addr(p_base, p_dev_req->wvalue.sb_type.lbyte);
#if 1
                printf("Assign address done. Device Address: %Xh(%d).\n",
                       p_dev_req->wvalue.sb_type.lbyte,
                       p_dev_req->wvalue.sb_type.lbyte);
#endif
            }
            else
            {
#if 1
                printf("USB handshake error!\n");
#endif
            }
        }
        break;
        case USB_REQ_GET_DESCRIPTOR:
        {
            if (USB_RT_RECIPIENT_DEVICE == (p_dev_req->bmreq_type & USB_RT_RECIPIENT_MASK))
            {
                switch (p_dev_req->wvalue.sb_type.hbyte)
                {
                    case USB_DESC_DEVICE: // 01h
                    {
                        p_desc   = (const uint8_t *)&s_usb_dev_desc;
                        byte_cnt = sizeof(s_usb_dev_desc);
                    }
                    break;
                    case USB_DESC_CONFIGURATION:      // 02h
                    case USB_DESC_OTHER_SPEED_CONFIG: // 07h
                    {
                        p_desc   = (const uint8_t *)&s_usbd_fs_hid_cfg_desc;
                        byte_cnt = sizeof(s_usbd_fs_hid_cfg_desc);
                    }
                    break;
                    case USB_DESC_STRING: // 03h
                    {
                        switch (p_dev_req->wvalue.sb_type.lbyte)
                        {
                            case USB_STR_IDX000: /**< String Index0 */
                            {
                                p_desc   = (const uint8_t *)&s_usbd_str_lang_desc;
                                byte_cnt = sizeof(s_usbd_str_lang_desc);
                            }
                            break;
                            case USB_STR_IDX001: /**< String Index1 */
                            {
                                p_desc   = (const uint8_t *)&s_usbd_str_mfr;
                                byte_cnt = sizeof(s_usbd_str_mfr);
                            }
                            break;
                            case USB_STR_IDX002: /**< String Index2 */
                            {
                                p_desc   = (const uint8_t *)&s_usbd_str_prod;
                                byte_cnt = sizeof(s_usbd_str_prod);
                            }
                            break;
                            case USB_STR_IDX003: /**< String Index3 */
                            {
                                p_desc   = (const uint8_t *)&s_usbd_str_sn;
                                byte_cnt = sizeof(s_usbd_str_sn);
                            }
                            break;
                            default:
                            {
                                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                            }
                                return;
                        }
                    }
                    break;
                    case USB_DESC_DEVICE_QUALIFIER: // 06h
                    {
                        p_desc   = (const uint8_t *)&s_usbd_qualf_desc;
                        byte_cnt = sizeof(s_usbd_qualf_desc);
                    }
                    break;
                    default:
                    {
                        usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                    }
                        return;
                }
            }
            else if (USB_RT_RECIPIENT_INTERFACE == (p_dev_req->bmreq_type & USB_RT_RECIPIENT_MASK))
            {
                switch (p_dev_req->wvalue.sb_type.hbyte)
                {
                    case USBD_DESC_CS_HID: // 21h
                    {
                        p_desc   = (const uint8_t *)&(s_usbd_fs_hid_cfg_desc.hid_desc);
                        byte_cnt = sizeof(s_usbd_fs_hid_cfg_desc.hid_desc);
                    }
                    break;
                    case USBD_DESC_CS_REPORT: // 22h
                    {
                        p_desc   = (const uint8_t *)s_hid_report.report;
                        byte_cnt = sizeof(s_hid_report);
                    }
                    break;
                    default:
                    {
                        usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                    }
                        return;
                }
            }
            else
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            if (!p_dev_req->wlength.db_type.word)
            {
                usbd_ctrl_ep0_wait_zero_len_pkt_sent_completed();
                break;
            }
            if (p_dev_req->wlength.sb_type.hbyte)
            {
                printf("Warning! %s-%d.\n", __FILE__, __LINE__);
                printf("Request length over 256 bytes! Length: %Xh(%d).\n",
                       p_dev_req->wlength.db_type.word,
                       p_dev_req->wlength.db_type.word);
            }
#if 0
            printf("Descriptor:\n");
            uartif_print_buffer(p_desc, byte_cnt);
#endif
            req_len = p_dev_req->wlength.db_type.word;
            if (byte_cnt < req_len)
            {
                req_len = byte_cnt;
            }
            for (byte_cnt = 0u, xfer_len = 0u; byte_cnt < req_len;
                 byte_cnt += xfer_len, p_desc += xfer_len)
            {
                xfer_len = req_len - byte_cnt;
                /* For end-point 0, USBD_CTRL_EP0_PKT_LEN is 40h(64). */
                if (USBD_CTRL_EP0_PKT_LEN < xfer_len)
                {
                    xfer_len = USBD_CTRL_EP0_PKT_LEN;
                }
                /** Memory copy from code area to SRAM area. */
                memcpy(p_state_ptr->p_ep0_buffer, p_desc, xfer_len);
                /** Replace bDescriptorType if request is OTHER_SPEED_CONFIGURATION. */
                if ((USB_DESC_OTHER_SPEED_CONFIG == p_dev_req->wvalue.sb_type.hbyte) && !byte_cnt)
                {
                    ((usb_std_cfg_desc_t *)p_state_ptr->p_ep0_buffer)->bdesc_type =
                        USB_DESC_OTHER_SPEED_CONFIG;
                }
                usbd_ctrl_ep0_wait_pkt_sent_completed((uint8_t)xfer_len);
            }
        }
        break;
        case USB_REQ_SET_DESCRIPTOR:
        {
            usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
        }
            return;
        case USB_REQ_GET_CONFIGURATION:
        {
            if (USBD_DEFAULT_STATE == p_state_ptr->bus_cfg_state)
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            p_state_ptr->p_ep0_buffer[0u] = p_state_ptr->cur_cfg_val;
            usbd_ctrl_ep0_wait_pkt_sent_completed(1u);
        }
        break;
        case USB_REQ_SET_CONFIGURATION:
        {
            if (USBD_DEFAULT_STATE == p_state_ptr->bus_cfg_state)
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            if (1u == p_dev_req->wvalue.sb_type.lbyte)
            {
                p_state_ptr->cur_cfg_val   = p_dev_req->wvalue.sb_type.lbyte;
                p_state_ptr->bus_cfg_state = USBD_CONFIGURED_STATE;
            }
            else if (0u == p_dev_req->wvalue.sb_type.lbyte)
            {
                p_state_ptr->cur_cfg_val   = p_dev_req->wvalue.sb_type.lbyte;
                p_state_ptr->bus_cfg_state = USBD_ADDRESSED_STATE;
                // usbd_configuration_reset();
            }
            else
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            usbd_ctrl_ep0_wait_resp_sts_completed();
        }
        break;
        case USB_REQ_GET_INTERFACE:
        {
            if (p_dev_req->windex.sb_type.lbyte)
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            p_state_ptr->p_ep0_buffer[0u] = 0u;
            usbd_ctrl_ep0_wait_pkt_sent_completed(1u);
        }
        break;
        case USB_REQ_SET_INTERFACE:
        {
            if (USBD_CONFIGURED_STATE != p_state_ptr->bus_cfg_state)
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            usbd_ctrl_ep0_wait_resp_sts_completed();
        }
        break;
        default:
        {
            usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
        }
            return;
    }
}

static void
usbd_ctrl_class_cmd_proc(void)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    usb_dev_req_t       *p_dev_req   = &(p_state_ptr->dev_req);

    switch (p_dev_req->brequest)
    {
        case USBD_REQ_GET_IDLE:
        {
            if (p_dev_req->windex.sb_type.lbyte == s_usbd_fs_hid_cfg_desc.usb_if_desc.bif_number)
            {
                p_state_ptr->p_ep0_buffer[0u] = p_state_ptr->hid_report_id0_idle_rate;
            }
            else
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            usbd_ctrl_ep0_wait_pkt_sent_completed(1u);
        }
        break;
        case USBD_REQ_SET_IDLE:
        {
            if (p_dev_req->windex.sb_type.lbyte == s_usbd_fs_hid_cfg_desc.usb_if_desc.bif_number)
            {
                p_state_ptr->hid_report_id0_idle_rate = p_dev_req->wvalue.sb_type.hbyte;
                if ((USBD_SC_BOOT == s_usbd_fs_hid_cfg_desc.usb_if_desc.bif_sub_class) &&
                    (0u == p_dev_req->wvalue.sb_type.hbyte))
                {
                    usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                    return;
                }
            }
            else
            {
                usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
                return;
            }
            usbd_ctrl_ep0_wait_resp_sts_completed();
        }
        break;
        default:
        {
            usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
        }
            return;
    }
}

static void
usbd_ctrl_vnd_cmd_proc(void)
{
    DEV_ASSERT(NULL != sp_usbd_state_ptr);
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    usb_dev_req_t       *p_dev_req   = &(p_state_ptr->dev_req);

    switch (p_dev_req->brequest)
    {
        default:
        {
            usbd_ctrl_ep0_set_stall(); /**< Return stall for command error. */
        }
            return;
    }
}

static void
usbd_ctrl_ep0_set_stall(void)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep0_stall(p_base, true);
}

static bool
usbd_ctrl_ep0_wait_pkt_sent_completed(uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep0_pkt_send(p_base, byte_cnt);
    while (false == fsusb_get_ep0_pkt_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static bool
usbd_ctrl_ep0_wait_resp_sts_completed(void)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep0_zero_send(p_base);
    while (false == fsusb_get_ep0_zero_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static inline bool
usbd_ctrl_ep0_wait_zero_len_pkt_sent_completed(void)
{
    return usbd_ctrl_ep0_wait_resp_sts_completed();
}

static void
usbd_intrpt_in_ep1_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep1_stall(p_base, b_flag);
}

static bool
usbd_intrpt_in_ep1_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    p_state_ptr->b_ep_sram3_job_in_que = true;
    fsusb_set_ep1_pkt_send(p_base, byte_cnt);
    p_state_ptr->b_ep1_sent_completed = false;
    if (b_await_xfer_done)
    {
        while (false == fsusb_get_ep1_pkt_sent_rdy(p_base))
        {
            if (p_state_ptr->b_bus_rst_immediately)
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_intrpt_in_ep2_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep2_stall(p_base, b_flag);
}

static bool
usbd_intrpt_in_ep2_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    p_state_ptr->b_ep_sram3_job_in_que = true;
    fsusb_set_ep2_pkt_send(p_base, byte_cnt);
    p_state_ptr->b_ep2_sent_completed = false;
    if (b_await_xfer_done)
    {
        while (false == fsusb_get_ep2_pkt_sent_rdy(p_base))
        {
            if (p_state_ptr->b_bus_rst_immediately)
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_intrpt_in_ep3_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep3_stall(p_base, b_flag);
}

static bool
usbd_intrpt_in_ep3_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    p_state_ptr->b_ep_sram3_job_in_que = true;
    fsusb_set_ep3_pkt_send(p_base, byte_cnt);
    p_state_ptr->b_ep3_sent_completed = false;
    if (b_await_xfer_done)
    {
        while (false == fsusb_get_ep3_pkt_sent_rdy(p_base))
        {
            if (p_state_ptr->b_bus_rst_immediately)
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_intrpt_out_ep4_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep4_stall(p_base, b_flag);
}

static bool
usbd_intrpt_out_ep4_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    *p_byte_cnt = 0u;
    while (false == fsusb_get_ep4_pkt_fetched_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    *p_byte_cnt = fsusb_get_ep4_rx_pkt_len(p_base);
    return true;
}

static void
usbd_intrpt_in_ep5_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep5_stall(p_base, b_flag);
}

static bool
usbd_intrpt_in_ep5_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    p_state_ptr->b_ep_sram1_job_in_que = true;
    fsusb_set_ep5_pkt_send(p_base, byte_cnt);
    p_state_ptr->b_ep5_sent_completed = false;
    if (b_await_xfer_done)
    {
        while (false == fsusb_get_ep5_pkt_sent_rdy(p_base))
        {
            if (p_state_ptr->b_bus_rst_immediately)
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_intrpt_out_ep6_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep6_stall(p_base, b_flag);
}

static bool
usbd_intrpt_out_ep6_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    *p_byte_cnt = 0u;
    while (false == fsusb_get_ep6_pkt_fetched_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    *p_byte_cnt = fsusb_get_ep6_rx_pkt_len(p_base);
    return true;
}

#if 0
static void
usbd_intrpt_in_ep7_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep7_stall(p_base, b_flag);
}

static bool
usbd_intrpt_in_ep7_wait_pkt_sent_completed(uint8_t byte_cnt, bool b_await_xfer_done)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    p_state_ptr->b_ep_sram2_job_in_que = true;
    fsusb_set_ep7_pkt_send(p_base, byte_cnt);
    p_state_ptr->b_ep7_sent_completed = false;
    if (b_await_xfer_done)
    {
        while (false == fsusb_get_ep7_pkt_sent_rdy(p_base))
        {
            if (p_state_ptr->b_bus_rst_immediately)
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_intrpt_out_ep8_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep8_stall(p_base, b_flag);
}

static bool
usbd_intrpt_out_ep8_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    *p_byte_cnt = 0u;
    while (false == fsusb_get_ep8_pkt_fetched_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    *p_byte_cnt = fsusb_get_ep8_rx_pkt_len(p_base);
    return true;
}
#endif

static bool
usbd_bulk_in_ep1_wait_payload_xfer_done(uint8_t *p_source, uint8_t sect_cnt, uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    DEV_ASSERT((0u == sect_cnt) ^ (0u == byte_cnt));
    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep1_buf_size;
    if (!sect_cnt)
    {
        DEV_ASSERT(64u >= byte_cnt);
        if (false == usbd_bulk_in_ep1_wait_pkt_sent_completed(byte_cnt))
        {
            return false;
        }
        return true;
    }
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_source += pkt_size)
        {
            memcpy(p_state_ptr->p_ep_shared_sram3_ptr, p_source, pkt_size);
            if (false == usbd_bulk_in_ep1_wait_pkt_sent_completed(pkt_size))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_in_ep1_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep1_stall(p_base, b_flag);
}

static bool
usbd_bulk_in_ep1_wait_pkt_sent_completed(uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep1_pkt_send(p_base, byte_cnt);
    while (false == fsusb_get_ep1_pkt_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static bool
usbd_bulk_in_ep2_wait_payload_xfer_done(uint8_t *p_source, uint8_t sect_cnt, uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    DEV_ASSERT((0u == sect_cnt) ^ (0u == byte_cnt));
    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep2_buf_size;
    if (!sect_cnt)
    {
        DEV_ASSERT(64u >= byte_cnt);
        if (false == usbd_bulk_in_ep2_wait_pkt_sent_completed(byte_cnt))
        {
            return false;
        }
        return true;
    }
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_source += pkt_size)
        {
            memcpy(p_state_ptr->p_ep_shared_sram3_ptr, p_source, pkt_size);
            if (false == usbd_bulk_in_ep2_wait_pkt_sent_completed(pkt_size))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_in_ep2_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep2_stall(p_base, b_flag);
}

static bool
usbd_bulk_in_ep2_wait_pkt_sent_completed(uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep2_pkt_send(p_base, byte_cnt);
    while (false == fsusb_get_ep2_pkt_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static bool
usbd_bulk_in_ep3_wait_payload_xfer_done(uint8_t *p_source, uint8_t sect_cnt, uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    DEV_ASSERT((0u == sect_cnt) ^ (0u == byte_cnt));
    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep3_buf_size;
    if (!sect_cnt)
    {
        DEV_ASSERT(64u >= byte_cnt);
        if (false == usbd_bulk_in_ep3_wait_pkt_sent_completed(byte_cnt))
        {
            return false;
        }
        return true;
    }
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_source += pkt_size)
        {
            memcpy(p_state_ptr->p_ep_shared_sram3_ptr, p_source, pkt_size);
            if (false == usbd_bulk_in_ep3_wait_pkt_sent_completed(pkt_size))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_in_ep3_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep3_stall(p_base, b_flag);
}

static bool
usbd_bulk_in_ep3_wait_pkt_sent_completed(uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep3_pkt_send(p_base, byte_cnt);
    while (false == fsusb_get_ep3_pkt_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static bool
usbd_bulk_out_ep4_wait_payload_xfer_done(uint8_t *p_dest, uint8_t sect_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint8_t              fetched_len;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep4_buf_size;
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_dest += pkt_size)
        {
            if (false == usbd_bulk_out_ep4_wait_pkt_rcvd_completed(&fetched_len))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_out_ep4_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep1_stall(p_base, b_flag);
}

static bool
usbd_bulk_out_ep4_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    *p_byte_cnt = 0u;
    fsusb_clear_ep4_latched_st(p_base);
    while (false == fsusb_get_ep4_pkt_fetched_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    *p_byte_cnt = fsusb_get_ep4_rx_pkt_len(p_base);
    return true;
}

static bool
usbd_bulk_in_ep5_wait_payload_xfer_done(uint8_t *p_source, uint8_t sect_cnt, uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    DEV_ASSERT((0u == sect_cnt) ^ (0u == byte_cnt));
    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep5_buf_size;
    if (!sect_cnt)
    {
        DEV_ASSERT(64u >= byte_cnt);
        if (false == usbd_bulk_in_ep5_wait_pkt_sent_completed(byte_cnt))
        {
            return false;
        }
        return true;
    }
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_source += pkt_size)
        {
            memcpy(p_state_ptr->p_ep_shared_sram1_ptr, p_source, pkt_size);
            if (false == usbd_bulk_in_ep5_wait_pkt_sent_completed(pkt_size))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_in_ep5_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep5_stall(p_base, b_flag);
}

static bool
usbd_bulk_in_ep5_wait_pkt_sent_completed(uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep5_pkt_send(p_base, byte_cnt);
    while (false == fsusb_get_ep5_pkt_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static bool
usbd_bulk_out_ep6_wait_payload_xfer_done(uint8_t *p_dest, uint8_t sect_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint8_t              fetched_len;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep6_buf_size;
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_dest += pkt_size)
        {
            if (false == usbd_bulk_out_ep6_wait_pkt_rcvd_completed(&fetched_len))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_out_ep6_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep1_stall(p_base, b_flag);
}

static bool
usbd_bulk_out_ep6_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    *p_byte_cnt = 0u;
    fsusb_clear_ep6_latched_st(p_base);
    while (false == fsusb_get_ep6_pkt_fetched_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    *p_byte_cnt = fsusb_get_ep6_rx_pkt_len(p_base);
    return true;
}

#if 0
static bool
usbd_bulk_in_ep7_wait_payload_xfer_done(uint8_t *p_source, uint8_t sect_cnt, uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    DEV_ASSERT((0u == sect_cnt) ^ (0u == byte_cnt));
    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep7_buf_size;
    if (!sect_cnt)
    {
        DEV_ASSERT(64u >= byte_cnt);
        if (false == usbd_bulk_in_ep7_wait_pkt_sent_completed(byte_cnt))
        {
            return false;
        }
        return true;
    }
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_source += pkt_size)
        {
            memcpy(p_state_ptr->p_ep_shared_sram2_ptr, p_source, pkt_size);
            if (false == usbd_bulk_in_ep7_wait_pkt_sent_completed(pkt_size))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_in_ep7_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep7_stall(p_base, b_flag);
}

static bool
usbd_bulk_in_ep7_wait_pkt_sent_completed(uint8_t byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep7_pkt_send(p_base, byte_cnt);
    while (false == fsusb_get_ep7_pkt_sent_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    return true;
}

static bool
usbd_bulk_out_ep8_wait_payload_xfer_done(uint8_t *p_dest, uint8_t sect_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    uint8_t              u8_cntr;
    uint8_t              fetched_len;
    uint16_t             pkt_size;
    uint16_t             u16_cntr;

    // pkt_size = p_state_ptr->b_full_spd_mode ? 64u : 512u;
    pkt_size = p_state_ptr->ep8_buf_size;
    for (u8_cntr = 0u; u8_cntr < sect_cnt; u8_cntr++)
    {
        for (u16_cntr = 0u; u16_cntr < 512u; u16_cntr += pkt_size, p_dest += pkt_size)
        {
            if (false == usbd_bulk_out_ep8_wait_pkt_rcvd_completed(&fetched_len))
            {
                return false;
            }
        }
    }
    return true;
}

static void
usbd_bulk_out_ep8_set_stall(bool b_flag)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    fsusb_set_ep1_stall(p_base, b_flag);
}

static bool
usbd_bulk_out_ep8_wait_pkt_rcvd_completed(uint8_t *p_byte_cnt)
{
    usbd_dev_if_state_t *p_state_ptr = sp_usbd_state_ptr;
    FSUSB_t             *p_base      = p_state_ptr->p_base;

    *p_byte_cnt = 0u;
    fsusb_clear_ep8_latched_st(p_base);
    while (false == fsusb_get_ep8_pkt_fetched_rdy(p_base))
    {
        if (p_state_ptr->b_bus_rst_immediately)
        {
            return false;
        }
    }
    *p_byte_cnt = fsusb_get_ep8_rx_pkt_len(p_base);
    return true;
}
#endif

/*** end of file ***/
