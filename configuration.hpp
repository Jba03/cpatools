#pragma once

#define CPA_ENDIAN_BIG      0
#define CPA_ENDIAN_LITTLE   1

/* TODO: correct endianness */
#define CPA_MAKE_PLATFORM(I,E) ((I<<1)|E)
#define CPA_PLATFORM_GCN      CPA_MAKE_PLATFORM(0x1, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_PS1      CPA_MAKE_PLATFORM(0x2, CPA_ENDIAN_LITTLE)
#define CPA_PLATFORM_PS2      CPA_MAKE_PLATFORM(0x3, CPA_ENDIAN_LITTLE)
#define CPA_PLATFORM_PS3      CPA_MAKE_PLATFORM(0x4, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_XBOX     CPA_MAKE_PLATFORM(0x5, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_XBOX360  CPA_MAKE_PLATFORM(0x6, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_PC       CPA_MAKE_PLATFORM(0x7, CPA_ENDIAN_LITTLE)
#define CPA_PLATFORM_OSX      CPA_MAKE_PLATFORM(0x8, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_DC       CPA_MAKE_PLATFORM(0x9, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_NDS      CPA_MAKE_PLATFORM(0xA, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_3DS      CPA_MAKE_PLATFORM(0xB, CPA_ENDIAN_BIG)
#define CPA_PLATFORM_N64      CPA_MAKE_PLATFORM(0xC, CPA_ENDIAN_BIG)

#define CPA_ENGINE_VERSION_R2 0
#define CPA_ENGINE_VERSION_R3 1

#define CPA_VERSION_ID_BIT 0x80000000
#define CPA_MAKE_VERSION(I,B,P) ((I << 9) | (B << 6) | P | CPA_VERSION_ID_BIT)
#define CPA_VERSION_R2_PC                     CPA_MAKE_VERSION(0x00, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_GCN)
#define CPA_VERSION_R2_PC_DEMO_1999_08_18     CPA_MAKE_VERSION(0x01, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_GCN)
#define CPA_VERSION_R2_PC_DEMO_1999_09_04     CPA_MAKE_VERSION(0x02, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_PS2)
#define CPA_VERSION_R2_PS1                    CPA_MAKE_VERSION(0x03, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_PS1)
#define CPA_VERSION_R2_PS2                    CPA_MAKE_VERSION(0x04, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_PS2)
#define CPA_VERSION_R2_N64                    CPA_MAKE_VERSION(0x05, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_N64)
#define CPA_VERSION_R2_NDS                    CPA_MAKE_VERSION(0x06, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_NDS)
#define CPA_VERSION_R2_3DS                    CPA_MAKE_VERSION(0x07, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_3DS)
#define CPA_VERSION_R2_DC                     CPA_MAKE_VERSION(0x08, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_DC)
#define CPA_VERSION_R2_DC_J                   CPA_MAKE_VERSION(0x09, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_DC)
#define CPA_VERSION_R2R_PS2                   CPA_MAKE_VERSION(0x0A, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_PS2)
#define CPA_VERSION_R2R_PS2_J                 CPA_MAKE_VERSION(0x0B, CPA_ENGINE_VERSION_R2, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_GCN                    CPA_MAKE_VERSION(0x0C, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_GCN)
#define CPA_VERSION_R3_GCN_DEMO               CPA_MAKE_VERSION(0x0D, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_GCN)
#define CPA_VERSION_R3_PS2                    CPA_MAKE_VERSION(0x0E, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_PS2_DEMO_2002_05_17    CPA_MAKE_VERSION(0x0F, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_PS2_DEMO_2002_08_07    CPA_MAKE_VERSION(0x10, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_PS2_DEVB_2002_09_06    CPA_MAKE_VERSION(0x11, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_PS2_DEMO_2002_10_29    CPA_MAKE_VERSION(0x12, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_PS2_DEMO_2002_12_18    CPA_MAKE_VERSION(0x13, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS2)
#define CPA_VERSION_R3_PS3                    CPA_MAKE_VERSION(0x14, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PS3)
#define CPA_VERSION_R3_PC                     CPA_MAKE_VERSION(0x15, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PC)
#define CPA_VERSION_R3_PC_DEMO_2002_10_04     CPA_MAKE_VERSION(0x16, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PC)
#define CPA_VERSION_R3_PC_DEMO_2002_10_21     CPA_MAKE_VERSION(0x17, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PC)
#define CPA_VERSION_R3_PC_DEMO_2002_12_10     CPA_MAKE_VERSION(0x18, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PC)
#define CPA_VERSION_R3_PC_DEMO_2003_01_08     CPA_MAKE_VERSION(0x19, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PC)
#define CPA_VERSION_R3_PC_DEMO_2003_01_29     CPA_MAKE_VERSION(0x1A, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_PC)
#define CPA_VERSION_R3_XBOX                   CPA_MAKE_VERSION(0x1B, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_XBOX)
#define CPA_VERSION_R3_XBOX360                CPA_MAKE_VERSION(0x1C, CPA_ENGINE_VERSION_R3, CPA_PLATFORM_XBOX360)

#define CPA_PLATFORM (CPA_VERSION & 0xF)
#define CPA_ENDIANNESS (CPA_PLATFORM & 1)
#define CPA_ENGINE_VERSION ((CPA_VERSION & 0x1FF) >> 6)

#if !(CPA_VERSION & CPA_VERSION_ID_BIT)
# error Version not set. Please define CPA_VERSION to one of CPA_VERSION_#.
#endif
