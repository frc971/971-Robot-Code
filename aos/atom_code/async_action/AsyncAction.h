#ifndef _AOS_ASYNC_ACTION_H_
#define _AOS_ASYNC_ACTION_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include <string>
#include <map>
#include <type_traits>

#include "aos/aos_core.h"
#include "aos/common/type_traits.h"

class AsyncActionTest;

namespace aos {
	// S is status type T is parameter type

	template<class S> struct async_action_status {
		S status;
		uint8_t done_status; // 1 = valid status, 2 = done
		uint16_t count;
		pid_t pid;
		aos_resource_entity *resource_entity;
	};
	template<class T> struct async_action_start {
		T args;
		uint16_t count;
		aos_resource_entity *parent;
	};

	class AsyncActionRunner;
	class ResourceAction_t;

	class AsyncActionStatics {
		friend class ResourceAction_t; // a testing AsyncAction that has to mess with resource stuff
		protected:
			class stopexception : public std::exception {
				virtual const char* what() const throw() {
					return "This exception indicates that the AsyncAction was stopped. This message should never show up anywhere.";
				}
				public:
				stopexception() : std::exception() {}
			};
			class resourceexception : public std::exception {
				virtual const char* what() const throw() {
					return "This exception indicates that the AsyncAction was stopped due to resource contention. This message should never show up anywhere.";
				}
				public:
				resourceexception() : std::exception() {}
			};

			// 1 = stop current DoAction
			// 2 = stop corrent DoAction because of resource issues
			// 4 = SIGINT pending (nicely close everything down first)
			static volatile uint8_t interrupt;
			static const int STOP_SIGNAL;
			static aos_resource_entity *resource_entity;

			// throw any exceptions indicated by signals
			static inline void CheckStop(){
				if(interrupt & (0x01 | 0x04)) {
					throw stopexception();
					fprintf(stderr, "the code should never ever get here (%s: %d)\n", __FILE__, __LINE__);
				}else if(interrupt & 0x02) {
					throw resourceexception();
					fprintf(stderr, "the code should never ever get here (%s: %d)\n", __FILE__, __LINE__);
				}
			}
	};

	// S and T have to be structs
	// T is sTart and S is status
	// public functions (except for constructor) should be called on this AsyncAction
	// in processes other than the one Run ing this AsyncAction
	// vice versa for protected ones
	template<class T, class S> class AsyncAction : public AsyncActionStatics {
		static_assert(shm_ok<async_action_start<T>>::value,
                  "T must go through shared memory");
		static_assert(shm_ok<async_action_status<S>>::value,
                  "S must go through shared memory");
		friend class AsyncActionRunner;
		friend class ::AsyncActionTest;
		public:
		AsyncAction(const std::string name);

		// returns what the count will be throughout this run
		// return of 0 indicates error (didn't start)
		uint16_t Start(T &args);

		// -1 for count means use the static one
		// aka !IsRunning()
		bool IsDone(int32_t count = -1);
		// returns which one it joined
		uint16_t Join(int32_t count = -1);
		// return is whether there is an actual status or just garbage
		bool GetStatus(S &status_out, int32_t count = -1) __attribute__ ((warn_unused_result));
		// waits for a new good status
		bool GetNextStatus(S &status_out, int32_t count = -1) __attribute__ ((warn_unused_result));

		void Stop(int32_t count = -1);
		protected:
		// starts infinite loop that waits for Starts
		// returns 0 for success, negative for error
		// gets called by generated code
		int Run(uint8_t priority);

		virtual void DoAction(T &args);
		// should only be called from DoAction
		void PostStatus(S &status_in);
		void RequestResource(uint16_t resource);
		// returns whether succeeded
		bool TryRequestResource(uint16_t resource);
		void ReleaseResource(uint16_t resource);

		// called at the beginning and end of Run
		virtual void OnStart() {}
		virtual void OnEnd() {}

		// this should be the only sleep (or variant thereof) that gets called
		void Sleep(double seconds);
		private:
		aos_queue *status_queue, *start_queue;

		uint16_t local_count;
		S local_status;
		uint8_t local_done_status;
		pid_t local_pid;

		template<int (*O)(aos_resource_entity *, aos_resource *)> bool ResourceOp(uint16_t resource);
		std::map<uint16_t, uint8_t> resources;

		// for read_msg_index
		int done_index, status_index;
		uint16_t next_status_count; // uses it to figure out when to reset status_index

		// return the default if appropriate
		inline uint16_t GetCount(int32_t in){
			if(in < 0)
				return local_count;
			else
				return (uint16_t)in;
		}

		static void sig_action(int, siginfo_t *, void *);
	};

} // namespace aos

#include "AsyncAction.cpp.inc" // to make the template stuff work

#endif
