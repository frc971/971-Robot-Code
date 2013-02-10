#ifndef __AOS__ASYNC_ACTION_HANDLE_H_
#define __AOS__ASYNC_ACTION_HANDLE_H_

namespace aos {

	class AsyncActionHandle {
		public:
			virtual bool IsDone() = 0;
			virtual uint16_t Join() = 0;
			virtual uint16_t Join(int32_t count) = 0;
			virtual void Stop() = 0;
			virtual void Stop(int32_t count) = 0;
	};

} // namespace aos

#endif
