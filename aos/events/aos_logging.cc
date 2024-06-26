#include "aos/events/aos_logging.h"

namespace aos {

void AosLogToFbs::Initialize(const std::string *name,
                             Sender<logging::LogMessageFbs> log_sender) {
  log_sender_ = std::move(log_sender);
  if (log_sender_) {
    implementation_ = std::make_shared<logging::CallbackLogImplementation>(
        [this](const logging::LogMessage &message_data) {
          aos::Sender<logging::LogMessageFbs>::Builder message =
              log_sender_.MakeBuilder();
          auto message_str = message.fbb()->CreateString(
              message_data.message, message_data.message_length);
          auto name_str = message.fbb()->CreateString(message_data.name,
                                                      message_data.name_length);

          ::aos::logging::LogMessageFbs::Builder builder =
              message.MakeBuilder<::aos::logging::LogMessageFbs>();
          builder.add_message(message_str);
          builder.add_level(
              static_cast<::aos::logging::Level>(message_data.level));
          builder.add_source_pid(message_data.source);
          builder.add_name(name_str);
          if (send_failure_counter_.failures() > 0) {
            builder.add_send_failures(send_failure_counter_.failures());
          }

          send_failure_counter_.Count(message.Send(builder.Finish()));
        },
        name);
  }
}

}  // namespace aos
