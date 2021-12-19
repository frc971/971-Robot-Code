# line radius (LR) depicts offset to pads lines and pad "band width"
Element(0x00 "smd chip 1206" "" "SMD_CHIP 1206" 0 0 0 25 0x00)
(
        Pad(25 25 25 50 45      "" 0x100)
        Pad(145 25 145 50 45      "" 0x100)
        ElementLine(-10 -10 180 -10 5)
        ElementLine(180 -10 180 86 5)
        ElementLine(180 86 -10 86 5)
        ElementLine(-10 86 -10 -10 5)
        Mark(85 37.5)
)
