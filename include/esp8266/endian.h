#ifndef _endian_h_
#define _endian_h_

#include <machine/endian.h>
#include <stdint.h>

/** Byte swap a 16 bit value.
 * @param x value to swap
 * @return byte swapped value
 */
static inline uint16_t __bswap_16(uint16_t x)
{
    return __builtin_bswap16(x);
}

/** Byte swap a 32 bit value.
 * @param x value to swap
 * @return byte swapped value
 */
static inline uint32_t __bswap_32(uint32_t x)
{
    return __builtin_bswap32(x);
}

/** Byte swap a 64 bit value.
 * @param x value to swap
 * @return byte swapped value
 */
static inline uint64_t __bswap_64(uint64_t x)
{
    return __builtin_bswap64(x);
}

#ifdef CONFIG_ENDIAN_BIG
    #define htobe16(x) (x)
    #define htole16(x) __bswap_16 (x)
    #define be16toh(x) (x)
    #define le16toh(x) __bswap_16 (x)

    #define htobe32(x) (x)
    #define htole32(x) __bswap_32 (x)
    #define be32toh(x) (x)
    #define le32toh(x) __bswap_32 (x)

    #define htobe64(x) (x)
    #define htole64(x) __bswap_64 (x)
    #define be64toh(x) (x)
    #define le64toh(x) __bswap_64 (x)
#else
    #define htobe16(x) __bswap_16 (x)
    #define htole16(x) (x)
    #define be16toh(x) __bswap_16 (x)
    #define le16toh(x) (x)

    #define htobe32(x) __bswap_32 (x)
    #define htole32(x) (x)
    #define be32toh(x) __bswap_32 (x)
    #define le32toh(x) (x)

    #define htobe64(x) __bswap_64 (x)
    #define htole64(x) (x)
    #define be64toh(x) __bswap_64 (x)
    #define le64toh(x) (x)
#endif


#endif /* _endian_h_ */
