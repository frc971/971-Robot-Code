# This are the pad dimensions from
# https://www.akm.com/akm/en/file/datasheet/CQ-3301.pdf.
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
    # The normal-sized pads.
    Pad[-6398 17106 -6398 12894 1378 2000 2178 "2" "2" "square"]
    Pad[-3839 17106 -3839 12894 1378 2000 2178 "3" "3" "square"]
    Pad[-1280 17106 -1280 12894 1378 2000 2178 "4" "4" "square"]
    Pad[1280 17106 1280 12894 1378 2000 2178 "5" "5" "square"]
    Pad[3839 17106 3839 12894 1378 2000 2178 "6" "6" "square"]
    Pad[6398 17106 6398 12894 1378 2000 2178 "7" "7" "square"]

    # The massive pads for carrying the current.
    Pad[3386 -15000 11969 -15000 5591 2000 6391 "9" "9" "square"]
    Pad[-3386 -15000 -11969 -15000 5591 2000 6391 "10" "10" "square"]

    # The big ground pads.
    Pad[-11063 15000 -11967 15000 5591 2000 6391 "1" "1" "square"]
    Pad[11063 15000 11967 15000 5591 2000 6391 "8" "8" "square"]

    ElementLine[-15944 19000 15944 19000 800]
    ElementLine[-15944 -19000 15944 -19000 800]
    ElementLine[-15944 19000 -15944 -19000 800]
    ElementLine[15944 19000 15944 -19000 800]
)
