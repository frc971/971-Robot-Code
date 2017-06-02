# This is from <http://www.st.com/web/en/resource/technical/document/datasheet/CD00000544.pdf>.
Element[0x00000000 "LM1117 DPAK" "" "TO252" 0 0 -2000 -6000 0 100 0x00000000]
(
# Pad[x1, y1, x2, y2, thickness, clearance, mask, name , pad number, flags]
  Pad[0 0 0 0
      26378 1000 27300
      "2" "2" "square"]

  Pad[-9055 23425 -9055 28937
      6300 1000 7200
      "1" "1" "square"]
  Pad[9055 23425 9055 28937
      6300 1000 7200
      "3" "3" "square"]

  ElementLine[-14200 -14500 14200 -14500 1000]
  ElementLine[14200 -14500 14200 33500 1000]
  ElementLine[14200 33500 -14200 33500 1000]
  ElementLine[-14200 33500 -14200 -14500 1000]
)
