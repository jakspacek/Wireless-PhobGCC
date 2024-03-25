#ifndef FONT_H
#define FONT_H

namespace ascii {
const char font[15*(32*3 - 1)] = {
0b00000000, // space
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00011000, // !
0b00111100,
0b00111100,
0b00111100,
0b00111100,
0b00111100,
0b00011000, //
0b00011000,
0b00011000,
0b00000000,
0b00000000,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b01100110, // "
0b01100110,
0b11101110,
0b11001100,
0b11001100,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // #
0b00100100,
0b00100100,
0b00100100,
0b11111111,
0b00100100,
0b00100100, //
0b00100100,
0b00100100,
0b11111111,
0b00100100,
0b00100100,
0b00100100, //
0b00000000,
0b00000000,

0b00011000, // $
0b00011000,
0b01111110,
0b11011011,
0b11011000,
0b11011000,
0b01111110, //
0b00011011,
0b00011011,
0b11011011,
0b01111110,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b01100011, // %
0b10010011,
0b10010110,
0b01100110,
0b00001100,
0b00001100,
0b00011000, //
0b00110000,
0b00110000,
0b01100110,
0b01101001,
0b11001001,
0b11000110, //
0b00000000,
0b00000000,

0b00000000, // &
0b00111000,
0b01101100,
0b01000100,
0b01100000,
0b00100000,
0b00110001, //
0b01111011,
0b11001110,
0b10000100,
0b11001110,
0b01101011,
0b00110001, //
0b00000000,
0b00000000,

0b00011000, // '
0b00011000,
0b00111000,
0b00110000,
0b00110000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000001, // (
0b00000010,
0b00000110,
0b00000100,
0b00001100,
0b00001100,
0b00001100, //
0b00001100,
0b00001100,
0b00000100,
0b00000110,
0b00000010,
0b00000001, //
0b00000000,
0b00000000,

0b10000000, // )
0b01000000,
0b01100000,
0b00100000,
0b00110000,
0b00110000,
0b00110000, //
0b00110000,
0b00110000,
0b00100000,
0b01100000,
0b01000000,
0b10000000, //
0b00000000,
0b00000000,

0b00000000, // *
0b00000000,
0b01000010,
0b01100110,
0b00100100,
0b00011000,
0b11111111, //
0b00011000,
0b00100100,
0b01100110,
0b01000010,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // +
0b00000000,
0b00000000,
0b00011000,
0b00011000,
0b00011000,
0b11111111, //
0b00011000,
0b00011000,
0b00011000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // ,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00110000,
0b00110000,
0b00110000,
0b01100000, //
0b00000000,
0b00000000,

0b00000000, // -
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01111111, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // .
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01100000,
0b01100000, //
0b00000000,
0b00000000,

0b00000110, // /
0b00000110,
0b00001100,
0b00001100,
0b00001100,
0b00011000,
0b00011000, //
0b00011000,
0b00110000,
0b00110000,
0b00110000,
0b01100000,
0b01100000, //
0b00000000,
0b00000000,

0b00011000, // 0
0b00100100,
0b01100110,
0b01000010,
0b11000111,
0b11001011,
0b11011011, //
0b11010011,
0b11100011,
0b01000010,
0b01100110,
0b00100100,
0b00011000, //
0b00000000,
0b00000000,

0b00011000, // 1
0b00111000,
0b01111000,
0b11011000,
0b00011000,
0b00011000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b11111111, //
0b00000000,
0b00000000,

0b00111100, // 2
0b01100110,
0b11000011,
0b11000011,
0b00000011,
0b00000011,
0b00000110, //
0b00001100,
0b00011000,
0b00110000,
0b01100000,
0b11000000,
0b11111111, //
0b00000000,
0b00000000,

0b00111100, // 3
0b01100110,
0b11000011,
0b11000011,
0b00000011,
0b00000010,
0b00011100, //
0b00000010,
0b00000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b00000110, // 4
0b00001110,
0b00011110,
0b00110110,
0b01100110,
0b11000110,
0b11111111, //
0b00000110,
0b00000110,
0b00000110,
0b00000110,
0b00000110,
0b00000110, //
0b00000000,
0b00000000,

0b00111110, // 5
0b00110000,
0b01100000,
0b01100000,
0b11000000,
0b11111100,
0b00000110, //
0b00000011,
0b00000011,
0b00000011,
0b00000011,
0b11000110,
0b01111100, //
0b00000000,
0b00000000,

0b00111100, // 6
0b01100110,
0b01000010,
0b11000000,
0b11000000,
0b11011100,
0b11100110, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b11111111, // 7
0b00000011,
0b00000110,
0b00000110,
0b00000100,
0b00001100,
0b00001100, //
0b00001000,
0b00011000,
0b00011000,
0b00010000,
0b00110000,
0b00110000, //
0b00000000,
0b00000000,

0b00111100, // 8
0b01100110,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b01100110,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b00111100, // 9
0b01100110,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01100111, //
0b00111011,
0b00000011,
0b00000011,
0b01000010,
0b01100110,
0b00011100, //
0b00000000,
0b00000000,

0b00000000, // :
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00110000,
0b00110000, //
0b00000000,
0b00000000,
0b00110000,
0b00110000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // ;
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00110000,
0b00110000, //
0b00000000,
0b00000000,
0b00110000,
0b00110000,
0b00110000,
0b01100000, //
0b00000000,
0b00000000,

0b00000000, // <
0b00000000,
0b00000000,
0b00000001,
0b00000110,
0b00011000,
0b01100000, //
0b10000000,
0b01100000,
0b00011000,
0b00000110,
0b00000001,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // =
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01111110,
0b00000000, //
0b00000000,
0b00000000,
0b01111110,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // >
0b00000000,
0b00000000,
0b10000000,
0b01100000,
0b00011000,
0b00000110, //
0b00000001,
0b00000110,
0b00011000,
0b01100000,
0b10000000,
0b00000000, //
0b00000000,
0b00000000,

0b00111100, // ?
0b01100110,
0b11000011,
0b11000011,
0b11000011,
0b00000110,
0b00001100, //
0b00011000,
0b00011000,
0b00000000,
0b00000000,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b00000000, // @
0b00000000,
0b00000000,
0b00000000,
0b00111100,
0b01000010,
0b10011001, //
0b10100101,
0b10100101,
0b10100101,
0b10011010,
0b01000000,
0b00111110, //
0b00000000,
0b00000000,

0b00011000, // A
0b00011000,
0b00111100,
0b00111100,
0b01100110,
0b01100110,
0b11000011, //
0b11000011,
0b11111111,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b11111000, // B
0b11001100,
0b11000110,
0b11000110,
0b11000110,
0b11001100,
0b11111100, //
0b11000110,
0b11000011,
0b11000011,
0b11000011,
0b11000110,
0b11111100, //
0b00000000,
0b00000000,

0b00111100, // C
0b01100110,
0b11000011,
0b11000011,
0b11000000,
0b11000000,
0b11000000, //
0b11000000,
0b11000000,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b11111100, // D
0b11001110,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11001110,
0b11111100, //
0b00000000,
0b00000000,

0b11111111, // E
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11111100, //
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11111111, //
0b00000000,
0b00000000,

0b11111111, // F
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11111100, //
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000, //
0b00000000,
0b00000000,

0b00111100, // G
0b01100110,
0b11000011,
0b11000011,
0b11000000,
0b11000000,
0b11000000, //
0b11001111,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b11000011, // H
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11111111, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b11111111, // I
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b11111111, //
0b00000000,
0b00000000,

0b00001111, // J
0b00000110,
0b00000110,
0b00000110,
0b00000110,
0b00000110,
0b00000110, //
0b00000110,
0b00000110,
0b00000110,
0b11000110,
0b11000110,
0b01111100, //
0b00000000,
0b00000000,

0b11000011, // K
0b11000011,
0b11000110,
0b11001100,
0b11011000,
0b11110000,
0b11100000, //
0b11110000,
0b11011000,
0b11001100,
0b11000110,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b11000000, // L
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000, //
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11111111, //
0b00000000,
0b00000000,

0b11000011, // M
0b11100111,
0b11100111,
0b11011011,
0b11011011,
0b11011011,
0b11011011, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b11000011, // N
0b11000011,
0b11100011,
0b11100011,
0b11110011,
0b11010011,
0b11011011, //
0b11001011,
0b11001111,
0b11000111,
0b11000111,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b00111100, // O
0b01100110,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b11111100, // P
0b11000110,
0b11000011,
0b11000011,
0b11000011,
0b11000110,
0b11111100, //
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000000, //
0b00000000,
0b00000000,

0b00111100, // Q
0b01100110,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b11000011,
0b11010011,
0b11011011,
0b11001110,
0b01100110,
0b00111011, //
0b00000000,
0b00000000,

0b11111100, // R
0b11000110,
0b11000011,
0b11000011,
0b11000011,
0b11000110,
0b11111000, //
0b11001100,
0b11001100,
0b11000110,
0b11000110,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b00111110, // S
0b01100011,
0b11000001,
0b11000000,
0b11000000,
0b01100000,
0b00111100, //
0b00000110,
0b00000011,
0b00000011,
0b10000011,
0b11000110,
0b01111100, //
0b00000000,
0b00000000,

0b11111111, // T
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b11000011, // U
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01111110, //
0b00000000,
0b00000000,

0b11000011, // V
0b11000011,
0b11000011,
0b01000010,
0b01000010,
0b01100110,
0b01100110, //
0b00100100,
0b00100100,
0b00100100,
0b00111100,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b11000011, // W
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b11000011,
0b11011011,
0b11011011,
0b11011011,
0b01100110,
0b01100110, //
0b00000000,
0b00000000,

0b11000011, // X
0b11000011,
0b01100110,
0b01100110,
0b00100100,
0b00011000,
0b00011000, //
0b00011000,
0b00100100,
0b01100110,
0b01100110,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b11000011, // Y
0b11000011,
0b11000011,
0b01100110,
0b01100110,
0b00100100,
0b00111100, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b11111111, // Z
0b00000011,
0b00000110,
0b00000110,
0b00001100,
0b00001100,
0b00011000, //
0b00110000,
0b00110000,
0b01100000,
0b01100000,
0b11000000,
0b11111111, //
0b00000000,
0b00000000,

0b00001111, // [
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00001100, //
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00001111, //
0b00000000,
0b00000000,

0b01100000, // backslash
0b01100000,
0b00110000,
0b00110000,
0b00110000,
0b00011000,
0b00011000, //
0b00011000,
0b00001100,
0b00001100,
0b00001100,
0b00000110,
0b00000110, //
0b00000000,
0b00000000,

0b11110000, // ]
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b00110000, //
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b11110000, //
0b00000000,
0b00000000,

0b00011000, // ^
0b00111100,
0b01100110,
0b11000011,
0b10000001,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // _
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11111111, //
0b00000000,
0b00000000,

0b00110000, // `
0b00011000,
0b00001100,
0b00000110,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000,

0b00000000, // a
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01111100,
0b11000110, //
0b00000110,
0b01111110,
0b11000110,
0b11000110,
0b11001110,
0b01111001, //
0b00000000,
0b00000000,

0b00000000, // b
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11011100,
0b11100110, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11100110,
0b11011100, //
0b00000000,
0b00000000,

0b00000000, // c
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00111110,
0b01100011, //
0b11000001,
0b11000000,
0b11000000,
0b11000001,
0b01100011,
0b00111110, //
0b00000000,
0b00000000,

0b00000000, // d
0b00000011,
0b00000011,
0b00000011,
0b00000011,
0b00111011,
0b01100111, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01100111,
0b00111011, //
0b00000000,
0b00000000,

0b00000000, // e
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00111110,
0b01100011, //
0b11000011,
0b11111111,
0b11000000,
0b11000001,
0b01100011,
0b00111110, //
0b00000000,
0b00000000,

0b00000000, // f
0b00001110,
0b00011011,
0b00011000,
0b00011000,
0b00011000,
0b11111111, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b00000000, // g
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00111011,
0b01100111, //
0b11000011,
0b11000011,
0b11000011,
0b01100111,
0b00111011,
0b00000011, //
0b01100110,
0b00111100,

0b00000000, // h
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11011100,
0b11100110, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b00000000, // i
0b00000000,
0b00011000,
0b00000000,
0b00000000,
0b00111000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00001110, //
0b00000000,
0b00000000,

0b00000000, // j
0b00000000,
0b00011000,
0b00000000,
0b00000000,
0b00111000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b11011000,
0b01110000,

0b00000000, // k
0b11000000,
0b11000000,
0b11000000,
0b11000000,
0b11000110,
0b11001100, //
0b11011000,
0b11110000,
0b11011000,
0b11001100,
0b11000110,
0b11000011, //
0b00000000,
0b00000000,

0b00000000, // l
0b00111000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00001111, //
0b00000000,
0b00000000,

0b00000000, // m
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11101110,
0b11011011, //
0b11011011,
0b11011011,
0b11011011,
0b11011011,
0b11011011,
0b11011011, //
0b00000000,
0b00000000,

0b00000000, // n
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11011100,
0b11100110, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b11000011, //
0b00000000,
0b00000000,

0b00000000, // o
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00111100,
0b01100110, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b00000000, // p
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11011100,
0b11100110, //
0b11000011,
0b11000011,
0b11000011,
0b11100110,
0b11011100,
0b11000000, //
0b11000000,
0b11000000,

0b00000000, // q
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00111011,
0b01100111, //
0b11000011,
0b11000011,
0b11000011,
0b01100111,
0b00111011,
0b00000011, //
0b00000011,
0b00000011,

0b00000000, // r
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01101111,
0b01110000, //
0b01100000,
0b01100000,
0b01100000,
0b01100000,
0b01100000,
0b01100000, //
0b00000000,
0b00000000,

0b00000000, // s
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00111110,
0b01100011, //
0b01100000,
0b00111110,
0b00000011,
0b00000011,
0b01100011,
0b00111110, //
0b00000000,
0b00000000,

0b00000000, // t
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b11111111, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00001100,
0b00000111, //
0b00000000,
0b00000000,

0b00000000, // u
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11000011,
0b11000011, //
0b11000011,
0b11000011,
0b11000011,
0b11000011,
0b01100110,
0b00111100, //
0b00000000,
0b00000000,

0b00000000, // v
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11000011,
0b11000011, //
0b01100110,
0b01100110,
0b00100100,
0b00100100,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b00000000, // w
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11000011,
0b11000011, //
0b11011011,
0b11011011,
0b11011011,
0b11011011,
0b11011011,
0b01100110, //
0b00000000,
0b00000000,

0b00000000, // x
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11000011,
0b01100110, //
0b00100100,
0b00011000,
0b00011000,
0b00100100,
0b01100110,
0b11000011, //
0b00000000,
0b00000000,

0b00000000, // y
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b11000011,
0b11000011, //
0b11000011,
0b11000011,
0b11000011,
0b01100111,
0b00111011,
0b00000011, //
0b11000110,
0b01111100,

0b00000000, // z
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01111111,
0b00000011, //
0b00000110,
0b00001100,
0b00011000,
0b00110000,
0b01100000,
0b01111111, //
0b00000000,
0b00000000,

0b00000111, // {
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00011000, //
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00001100,
0b00000111, //
0b00000000,
0b00000000,

0b00011000, // |
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000,
0b00011000, //
0b00000000,
0b00000000,

0b11100000, // }
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b00011000, //
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b00110000,
0b11100000, //
0b00000000,
0b00000000,

0b00000000, // ~
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b01100011,
0b11011011, //
0b11000110,
0b00000000,
0b00000000,
0b00000000,
0b00000000,
0b00000000, //
0b00000000,
0b00000000
};//font

}//namespace ascii


#endif //FONT_H