#ifndef COMMON_HPP
#define COMMON_HPP

#define TARGET_CLOCK_FREQ	100000000 // Arty A7-100T: 100 MHz
#define TARGET_IP_ADDR		{0x0a, 0x0a, 0x0a, 0x01} // 10.10.10.1
#define TARGET_UDP_PORT		{0xcb, 0x20}		 // 52000
#define TARGET_DOMAIN_ID	0
#define TARGET_PARTICIPANT_ID	1
#define TARGET_GUID_PREFIX					\
	{0x01, 0x0f, 0x37, 0xad, 0xde, 0x09, 0x00, 0x00,	\
	 0x01, 0x00, 0x00, 0x00}

#define ENTITYID_APP_WRITER	{0x00, 0x00, 0x10, 0x03}
#define ENTITYID_APP_READER	{0x00, 0x00, 0x10, 0x04}

#define SBM_ENDIAN_LITTLE
// #define SBM_ENDIAN_BIG

#ifdef SBM_ENDIAN_LITTLE
#define S_BYTE0(x)	((x) & 0xff)
#define S_BYTE1(x)	(((x) >> 8) & 0xff)
#define L_BYTE0(x)	((x) & 0xff)
#define L_BYTE1(x)	(((x) >> 8) & 0xff)
#define L_BYTE2(x)	(((x) >> 16) & 0xff)
#define L_BYTE3(x)	(((x) >> 24) & 0xff)
#endif // SBM_ENDIAN_LITTLE

#ifdef SBM_ENDIAN_BIG
#define S_BYTE0(x)	(((x) >> 8) & 0xff)
#define S_BYTE1(x)	((x) & 0xff)
#define L_BYTE0(x)	(((x) >> 24) & 0xff)
#define L_BYTE1(x)	(((x) >> 16) & 0xff)
#define L_BYTE2(x)	(((x) >> 8) & 0xff)
#define L_BYTE3(x)	((x) & 0xff)
#endif // SBM_ENDIAN_BIG

#endif // !COMMON_HPP
