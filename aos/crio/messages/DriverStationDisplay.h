#ifndef AOS_CRIO_DRIVER_STATION_DISPLAY_H_
#define AOS_CRIO_DRIVER_STATION_DISPLAY_H_

#include <stdarg.h>

#include "WPILib/DriverStationLCD.h"

namespace aos {

class DriverStationDisplay {
 public:
  static void Send(int line, const char *fmt, ...)
      __attribute__((format(printf, 2, 3))) {
        DriverStationLCD::Line ds_line;
        switch (line) {
          case 0:
            ds_line = DriverStationLCD::kMain_Line6;
            break;
          case 1:
            ds_line = DriverStationLCD::kUser_Line1;
            break;
          case 2:
            ds_line = DriverStationLCD::kUser_Line2;
            break;
          case 3:
            ds_line = DriverStationLCD::kUser_Line3;
            break;
          case 4:
            ds_line = DriverStationLCD::kUser_Line4;
            break;
          case 5:
            ds_line = DriverStationLCD::kUser_Line5;
            break;
          case 6:
            ds_line = DriverStationLCD::kUser_Line6;
            break;
          default:
            printf("illegal line number %hhd\n", line);
            return;
        }
        va_list args;
        va_start(args, fmt);
        DriverStationLCD::GetInstance()->VPrintfLine(ds_line, fmt, args);
        va_end(args);
        DriverStationLCD::GetInstance()->UpdateLCD();
      }
};

} // namespace aos

#endif
