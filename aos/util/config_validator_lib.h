#ifndef AOS_UTIL_CONFIG_VALIDATOR_H_
#define AOS_UTIL_CONFIG_VALIDATOR_H_

#include "aos/configuration.h"
#include "aos/util/config_validator_config_generated.h"
#include "gtest/gtest.h"
namespace aos::util {

void ConfigIsValid(const aos::Configuration *config,
                   const ConfigValidatorConfig *validation_config);
}  // namespace aos::util
#endif  // AOS_UTIL_CONFIG_VALIDATOR_H_
