#ifndef FONT8X8_H
#define FONT8X8_H

#include <cstdint>

// Bảng font 8x8 chuẩn ASCII từ 0 đến 127
// Mỗi ký tự gồm 8 byte (mỗi byte tương ứng 1 hàng 8 pixel, từ bit 0 đến bit 7)
extern uint8_t font8x8_basic[128][8];

#endif // FONT8X8_H
