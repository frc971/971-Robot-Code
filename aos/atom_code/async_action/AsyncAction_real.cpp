#include "AsyncAction.h"

using namespace aos;

volatile uint8_t AsyncActionStatics::interrupt = 0;

const int AsyncActionStatics::STOP_SIGNAL = SIGRTMIN + 4;

aos_resource_entity *AsyncActionStatics::resource_entity = NULL;

