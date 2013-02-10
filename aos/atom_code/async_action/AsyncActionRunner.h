#ifndef __AOS_ASYNC_ACTION_RUNNER_H_
#define __AOS_ASYNC_ACTION_RUNNER_H_

#include "aos/atom_code/async_action/AsyncAction.h"

namespace aos {
	class AsyncActionRunner {
		public:
			template<class T, class S> inline static int Run(AsyncAction<T, S> &action, uint8_t priority) {
				return action.Run(priority);
			}
	};
}

#endif
