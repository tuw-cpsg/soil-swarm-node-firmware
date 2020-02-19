#ifndef UUID_H_
#define UUID_H_

#include <bluetooth/uuid.h>

/**
 * UUID (v4): 8feeXXXX-3c17-4189-8556-a293fa6b2739
 */

#define BT_UUID_BASE		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee0000, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS
 *  @brief AFarCloud Synchronization Service
 */
#define BT_UUID_ASS			BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee1801, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_GSC
 *  @brief ASS Generic Synchronization Characteristics
 */
#define BT_UUID_ASS_GSC		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2a01, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_TID
 *  @brief ASS Unix Timestamp Descriptor
 */
#define BT_UUID_ASS_UTD		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2901, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_ATD
 *  @brief ASS Ambient Temperature Descriptor
 */
#define BT_UUID_ASS_ATD		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2902, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_RHD
 *  @brief ASS Relative Humidity Descriptor
 */
#define BT_UUID_ASS_RHD		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2903, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_COS_OUTPUT_IO
 *  @brief ASS Soil Temperature Descriptor
 */
#define BT_UUID_ASS_STD		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2917, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_RSHD
 *  @brief ASS Relative Soil Humidity Descriptor
 */
#define BT_UUID_ASS_RSHD	BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2918, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_BVD
 *  @brief ASS Battery Voltage Descriptor
 */
#define BT_UUID_ASS_BVD		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2919, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

/** @def BT_UUID_ASS_BPD
 *  @brief ASS Battery Percent Descriptor
 */
#define BT_UUID_ASS_BPD		BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8fee2920, 0x3c17, 0x4189, 0x8556, 0xa293fa6b2739))

#endif /* UUID_H_ */
