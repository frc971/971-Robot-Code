#ifndef AOS_UTIL_CONFIG_VALIDATOR_H_
#define AOS_UTIL_CONFIG_VALIDATOR_H_

#include "aos/configuration.h"  // IWYU pragma: keep
#include "aos/util/config_validator_config_generated.h"

namespace aos::util {

void ConfigIsValid(const aos::Configuration *config,
                   const ConfigValidatorConfig *validation_config_raw);
}  // namespace aos::util
#endif  // AOS_UTIL_CONFIG_VALIDATOR_H_
