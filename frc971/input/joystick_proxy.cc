#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"

int main() {
  ::aos::Init();
  ::aos::input::JoystickProxy proxy;
  proxy.Run();
  ::aos::Cleanup();
}
