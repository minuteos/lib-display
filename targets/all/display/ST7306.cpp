/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * display/ST7306.cpp
 *
 * Driver for the Sitronix ST7306 TFT driver
 */

#include "ST7306.h"

#include <lvgl.h>

NO_INLINE void ST7306::Configure(uint32_t cmdAndLength, uint32_t data)
{
    Configure(cmdAndLength, (const char*)&data);
}

NO_INLINE void ST7306::WriteEq(uint64_t data)
{
    union {
        uint64_t d;
        uint32_t u[2];
        void ReverseNibbles()
        {
            auto u1 = ReverseNibbles(u[0]), u0 = ReverseNibbles(u[1]);
            u[0] = u0; u[1] = u1;
        }
        static uint32_t ReverseNibbles(uint32_t u)
        {
            u = __REV(u);
            return (u & 0xF0F0F0F0) >> 4 | (u & 0x0F0F0F0F) << 4;
        }
    } x = { .d = data };
    x.ReverseNibbles();
    WriteData(x);
}

void ST7306::Configure(uint32_t cmdAndLength, const char* data)
{
    WriteCommand((uint8_t)cmdAndLength);
    WriteData(Span(data, cmdAndLength >> 8));
}

ALWAYS_INLINE uint32_t expand(uint32_t a)
{
    // ____ ____ ____ ____ ABCD EFGH IJKL MNOP
    a = (a | (a << 8)) & 0x00FF00FF;
    // ____ ____ ABCD EFGH ____ ____ IJKL MNOP
    a = (a | (a << 4)) & 0x0F0F0F0F;
    // ____ ABCD ____ EFGH ____ IJKL ____ MNOP
    a = (a | (a << 2)) & 0x33333333;
    // __AB __CD __EF __GH __IJ __KL __MN __OP
    a = (a | (a << 1)) & 0x55555555;
    // _A_B _C_D _E_F _G_H _I_J _K_L _M_N _O_P
    return a;
}

OPTIMIZE_SIZE uint8_t* ST7306::FormatConvert(const uint8_t* fb, uint8_t* out, unsigned w, unsigned h, unsigned fbStride, unsigned outStride)
{
    while (h--)
    {
        // line pointers
        const uint16_t* s1 = (const uint16_t*)fb;
        fb += fbStride;
        const uint16_t* s2 = (const uint16_t*)fb;
        fb += fbStride;

        uint32_t ow;
        uint8_t* end = out + w;
        uint8_t* next = out + outStride;
        while (out < end)
        {
            // we need to turn
            //  MSB    LSB
            //   |      |
            //   ABCDEFGH abcdefgh <-- line n
            //   IJKLMNOP ijklmnop <-- line n+1
            // into
            //   AIBJCKDL EMFNGOHP aibjckdl emfngohp
            // the above is memory byte order, so the 32-bit word
            // needs to be in reverse order
            uint32_t a = *s1++, b = *s2++;
            // a = abcdefgh ABCDEFGH, b = ijklmnop IJKLMNOP
            ow = expand(b) | (expand(a) << 1);
            // ow = aibjckdl emfngohp AIBJCKDL EMFNGOHP
            ow = __REV16(ow);
            // ow = emfngohp aibjckdl EMFNGOHP AIBJCKDL
            if (out + 4 <= end)
            {
                *(uint32_t*)out = ow;
                out += 4;
            }
            else
            {
                if (out + 2 <= end)
                {
                    *(uint16_t*)out = ow;
                    ow >>= 16;
                    out += 2;
                }
                if (out < end)
                {
                    *out++ = ow;
                }
            }
        }
        out = next;
    }

    return out;
}
