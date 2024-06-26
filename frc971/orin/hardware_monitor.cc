#include <dirent.h>
#include <sys/statvfs.h>

#include "absl/flags/flag.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/orin/hardware_stats_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(bool, log_voltages, false, "If true, log voltages too.");

namespace frc971::orin {
namespace {
std::optional<std::string> ReadFileFirstLine(std::string_view file_name) {
  std::ifstream file(std::string(file_name), std::ios_base::in);
  if (!file.good()) {
    VLOG(1) << "Can't read " << file_name;
    return std::nullopt;
  }
  std::string line;
  std::getline(file, line);
  return line;
}

std::string GetHwmonNumber(const char *dir_name) {
  DIR *dirp = opendir(dir_name);
  if (!dirp) {
    VLOG(1) << "Can't open " << dir_name;
    return "";
  }
  struct dirent *directory_entry;
  while ((directory_entry = readdir(dirp)) != NULL) {
    std::string entry_name(directory_entry->d_name);
    if (entry_name.starts_with("hwmon")) {
      closedir(dirp);
      return entry_name;
    }
  }
  closedir(dirp);
  return "";
}
}  // namespace

// Periodically sends out the HardwareStats message with hardware statistics
// info.
class HardwareMonitor {
 public:
  HardwareMonitor(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        sender_(event_loop_->MakeSender<HardwareStats>("/hardware_monitor")),
        fan_hwmon_(
            GetHwmonNumber("/sys/devices/platform/39c0000.tachometer/hwmon/")),
        electrical_hwmon_(GetHwmonNumber(
            "/sys/devices/platform/c240000.i2c/i2c-1/1-0040/hwmon/")) {
    periodic_timer_ =
        event_loop_->AddTimer([this]() { PublishHardwareStats(); });
    event_loop_->OnRun([this]() {
      periodic_timer_->Schedule(event_loop_->monotonic_now(),
                                std::chrono::seconds(5));
    });
  }

 private:
  void PublishHardwareStats() {
    aos::Sender<HardwareStats>::Builder builder = sender_.MakeBuilder();
    // Iterate through all thermal zones
    std::vector<flatbuffers::Offset<ThermalZone>> thermal_zones;
    for (int zone_id = 0; zone_id < 9; zone_id++) {
      std::optional<std::string> zone_name = ReadFileFirstLine(absl::StrFormat(
          "/sys/devices/virtual/thermal/thermal_zone%d/type", zone_id));
      flatbuffers::Offset<flatbuffers::String> name_offset;
      if (zone_name) {
        name_offset = builder.fbb()->CreateString(*zone_name);
      }

      ThermalZone::Builder thermal_zone_builder =
          builder.MakeBuilder<ThermalZone>();
      thermal_zone_builder.add_id(zone_id);

      if (!name_offset.IsNull()) {
        thermal_zone_builder.add_name(name_offset);
      }

      std::optional<std::string> temperature_str =
          ReadFileFirstLine(absl::StrFormat(
              "/sys/devices/virtual/thermal/thermal_zone%d/temp", zone_id));
      uint64_t temperature = 0;
      if (temperature_str && absl::SimpleAtoi(*temperature_str, &temperature)) {
        thermal_zone_builder.add_temperature(temperature);
      }

      thermal_zones.emplace_back(thermal_zone_builder.Finish());
    }

    // Get fan speed
    std::optional<std::string> fan_speed_str = ReadFileFirstLine(
        absl::StrFormat("/sys/class/hwmon/%s/rpm", fan_hwmon_));

    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<ElectricalReading>>>
        electrical_readings_offset;
    if (absl::GetFlag(FLAGS_log_voltages)) {
      std::vector<flatbuffers::Offset<ElectricalReading>> electrical_readings;
      // Iterate through INA3221 electrical reading channels
      for (int channel = 1; channel <= 3; channel++) {
        std::optional<std::string> label = ReadFileFirstLine(absl::StrFormat(
            "/sys/class/hwmon/%s/in%d_label", electrical_hwmon_, channel));

        flatbuffers::Offset<flatbuffers::String> label_offset;
        if (label) {
          label_offset = builder.fbb()->CreateString(*label);
        }

        ElectricalReading::Builder electrical_reading_builder =
            builder.MakeBuilder<ElectricalReading>();
        electrical_reading_builder.add_channel(channel);

        if (!label_offset.IsNull()) {
          electrical_reading_builder.add_label(label_offset);
        }

        std::optional<std::string> voltage_str =
            ReadFileFirstLine(absl::StrFormat("/sys/class/hwmon/%s/in%d_input",
                                              electrical_hwmon_, channel));
        uint64_t voltage = 0;
        if (voltage_str && absl::SimpleAtoi(*voltage_str, &voltage)) {
          electrical_reading_builder.add_voltage(voltage);
        }

        std::optional<std::string> current_str = ReadFileFirstLine(
            absl::StrFormat("/sys/class/hwmon/%s/curr%d_input",
                            electrical_hwmon_, channel));
        uint64_t current = 0;
        if (current_str && absl::SimpleAtoi(*current_str, &current)) {
          electrical_reading_builder.add_current(current);
        }

        uint64_t power = voltage * current / 1000;
        if (power != 0) {
          electrical_reading_builder.add_power(power);
        }

        electrical_readings.emplace_back(electrical_reading_builder.Finish());
      }
      electrical_readings_offset =
          builder.fbb()->CreateVector(electrical_readings);
    }

    auto thermal_zone_offset = builder.fbb()->CreateVector(thermal_zones);
    HardwareStats::Builder hardware_stats_builder =
        builder.MakeBuilder<HardwareStats>();
    hardware_stats_builder.add_thermal_zones(thermal_zone_offset);
    uint64_t fan_speed = 0;
    if (fan_speed_str && absl::SimpleAtoi(*fan_speed_str, &fan_speed)) {
      hardware_stats_builder.add_fan_speed(fan_speed);
    }
    if (!electrical_readings_offset.IsNull()) {
      hardware_stats_builder.add_electrical_readings(
          electrical_readings_offset);
    }

    builder.CheckOk(builder.Send(hardware_stats_builder.Finish()));
  }

  aos::EventLoop *event_loop_;

  aos::Sender<HardwareStats> sender_;

  aos::TimerHandler *periodic_timer_;

  std::string fan_hwmon_;

  std::string electrical_hwmon_;
};

}  // namespace frc971::orin

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop shm_event_loop(&config.message());

  frc971::orin::HardwareMonitor hardware_monitor(&shm_event_loop);

  shm_event_loop.Run();

  return 0;
}
