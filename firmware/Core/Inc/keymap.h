/*
    ┌─────────────────────────────────────────────────────┐
    │Ctrl+Shift+G│   Ctrl+F     │  7 X  │8  │9│BackSpace  │
    ├─────────────────────────────────────────────────────┤
    │   Ctrl+G   │Ctrl+Shift+F X│  4    │5 X│6│    -      │
    ├─────────────────────────────────────────────────────┤
    │     ↑      │     ←        │  1 X  │2  │3│    +      │
    ├─────────────────────────────────────────────────────┤
    │     ↓      │     →        │Shift  │0 X│.│  Enter  X │
    └─────────────────────────────────────────────────────┘
*/

//http://www2d.biglobe.ne.jp/~msyk/keyboard/layout/usbkeycode.html
uint8_t keycodes[4][6] = {
        {0x0A, 0x09, 0x5F, 0x60, 0x61, 0x2A},
        {0x0A, 0x09, 0x5C, 0x5D, 0x5E, 0x56},
        {0x52, 0x50, 0x59, 0x5A, 0x5B, 0x57},
        {0x51, 0x4F, 0xE5, 0x62, 0x63, 0x58}
};

//RGUI,RAlt,RShift,RCtrl,LGUI,LAlt,LShift,LCtrl
uint8_t modifiers[4][6] = {
    {0b00110000,0b00010000,0b00000000,0b00000000,0b00000000,0b00000000},
    {0b00010000,0b00110000,0b00000000,0b00000000,0b00000000,0b00000000},
    {0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000},
    {0b00000000,0b00000000,0b00100000,0b00000000,0b00000000,0b00000000}
};
