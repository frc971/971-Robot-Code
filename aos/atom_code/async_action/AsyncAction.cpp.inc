#include <sys/syscall.h>
#include <errno.h>
#include <math.h>

namespace aos {

template<class T, class S> AsyncAction<T, S>::AsyncAction(const std::string name) : local_count(0), local_done_status(0), local_pid(0), 
	done_index(0), status_index(0), next_status_count(0){
	aos_type_sig async_action_status_sig = {sizeof(async_action_status<S>), 1234, 1};
	status_queue = aos_fetch_queue(name.c_str(), &async_action_status_sig);
	aos_type_sig async_action_start_sig = {sizeof(async_action_start<T>), 4321, 1};
	start_queue = aos_fetch_queue(name.c_str(), &async_action_start_sig);
}

template<class T, class S> uint16_t AsyncAction<T, S>::Start(T &args){
	CheckStop();
	// write something to the start queue and increment the count in the status queue
	async_action_start<S> *start = (async_action_start<S> *)aos_queue_get_msg(start_queue);
	if(start == NULL)
		return 0;
	memcpy(&start->args, &args, sizeof(T));
	async_action_status<T> *status = (async_action_status<T> *)aos_queue_read_msg(status_queue, 0); // wait until one starts it if didn't already
	uint16_t r = 0;
	aos_resource_entity *entity;
	if(status == NULL)
		goto err;
	r = status->count + 1;
	local_pid = status->pid;
	start->count = r;
	entity = status->resource_entity;
	aos_queue_free_msg(status_queue, status);
	status = (async_action_status<T> *)aos_queue_get_msg(status_queue);
	status->count = r;
	status->done_status = 0;
	status->pid = local_pid;
	status->resource_entity = entity; // the resource_entity of the action, not the caller!
	start->parent = resource_entity; // if it's NULL, it'll get fixed when starting
	if(aos_queue_write_msg(status_queue, status, OVERRIDE) < 0){
		goto err;
	} else {
		status = NULL;
	}
	if(aos_queue_write_msg(start_queue, start, OVERRIDE) < 0){
		goto err;
	} else {
		start = NULL;
	}
err:
	if(start != NULL)
		aos_queue_free_msg(start_queue, start);
	if(status != NULL)
		aos_queue_free_msg(status_queue, status);
	local_count = r;
	return r;
}
template<class T, class S> void AsyncAction<T, S>::Stop(int32_t count){
	CheckStop();	
	async_action_status<S> *status = (async_action_status<S> *)aos_queue_read_msg(status_queue, PEEK | NON_BLOCK);
	if(status == NULL)
		return;
	local_pid = status->pid;
	local_count = status->count;
	aos_queue_free_msg(status_queue, status);
	if(local_count != GetCount(count)) // if it's the wrong one
		return;
	async_action_start<S> *start = (async_action_start<S> *)aos_queue_read_msg(start_queue, PEEK | NON_BLOCK);
	if(start == NULL) // if there isn't something in the start queue (aka one running)
		return;
	aos_queue_free_msg(start_queue, start);
	union sigval sival;
	sival.sival_int = 0;
	if(sigqueue(local_pid, STOP_SIGNAL, sival) < 0){ // if sending the signal failed
		fprintf(stderr, "sigqueue STOP_SIGNAL (which is %d) with pid %d failed with errno %d: ", STOP_SIGNAL, local_pid, errno);
		perror(NULL);
	}
}

template<class T, class S> bool AsyncAction<T, S>::IsDone(int32_t count){
	CheckStop();
	async_action_status<S> *status = (async_action_status<S> *)aos_queue_read_msg(status_queue, PEEK | NON_BLOCK);
	// below = there is one running && it's the right one && it's done
	bool r = status != NULL && status->count == GetCount(count) && ((status->done_status & 0x02) != 0);
	aos_queue_free_msg(status_queue, status);
	return r;
}
template<class T, class S> uint16_t AsyncAction<T, S>::Join(int32_t count_in){
	const uint16_t count = GetCount(count_in);
	if(count == 0){
		fprintf(stderr, "not joining non-existent run 0\n");
		return 0;
	}
	async_action_status<S> *status = NULL;
	done_index = 0; // the queue's message numbering might have been reset
	do {
		aos_queue_free_msg(status_queue, status);
		CheckStop();
		status = (async_action_status<S> *)aos_queue_read_msg_index(status_queue, FROM_END, &done_index);
	} while(status != NULL && (status->done_status & 0x02) == 0 && (status->count == count));
	if(status == NULL){
		fprintf(stderr, "bad news at %s: %d\n", __FILE__, __LINE__);
		return 0;
	}
	aos_queue_free_msg(status_queue, status);
	return count;
}
template<class T, class S> bool AsyncAction<T, S>::GetNextStatus(S &status_out, int32_t count_in){
	async_action_status<S> *status = NULL;
start:
	CheckStop();
	const uint16_t count = GetCount(count_in);
	if(count != next_status_count){ // reset the index if another one gets started in case the queue indexes get reset
		next_status_count = count;
		status_index = 0;
	}
	status = (async_action_status<S> *)aos_queue_read_msg_index(status_queue, FROM_END, &status_index);
	if(status == NULL)
		goto start;
	if(status->count != count){
		aos_queue_free_msg(status_queue, status);
		return false;
	}
	bool r = (status->done_status & 0x01) != 0;
	memcpy(&status_out, &status->status, sizeof(S));
	aos_queue_free_msg(status_queue, status);
	return r;
}
template<class T, class S> bool AsyncAction<T, S>::GetStatus(S &status_out, int32_t count){
	CheckStop();
	async_action_status<S> *status = (async_action_status<S> *)aos_queue_read_msg(status_queue, PEEK | NON_BLOCK);
	if(status == NULL)
		return false;
	if(status->count != GetCount(count)){
		aos_queue_free_msg(status_queue, status);
		return false;
	}
	bool r = (status->done_status & 0x01) != 0;
	memcpy(&status_out, &status->status, sizeof(S));
	aos_queue_free_msg(status_queue, status);
	return r;
}

template<class T, class S> void AsyncAction<T, S>::PostStatus(S &status_in){
	CheckStop();
	async_action_status<S> *status = (async_action_status<S> *)aos_queue_get_msg(status_queue);
	if(status == NULL)
		return;
	memcpy(&local_status, &status_in, sizeof(S));
	memcpy(&status->status, &status_in, sizeof(S));
	status->done_status = 0x01;
	local_done_status = 0x01;
	status->pid = local_pid;
	status->resource_entity = resource_entity;
	if(aos_queue_write_msg(status_queue, status, OVERRIDE) < 0){
		local_done_status = 0;
		memset(&local_status, 0x00, sizeof(S));
		aos_queue_free_msg(status_queue, status);
	}
}
template<class T, class S> template<int (*O)(aos_resource_entity *, aos_resource *)> inline bool AsyncAction<T, S>::ResourceOp(uint16_t resource){
	CheckStop();
	if(resource_entity == NULL){
		fprintf(stderr, "no started AsyncAction detected in this process\n");
		return false;
	}
	switch(O(resource_entity, aos_resource_get(resource))){
		case 0:
			break;
		case 1:
			return false;
		case -1:
			throw resourceexception();
	}
	return true;
}
template<class T, class S> void AsyncAction<T, S>::RequestResource(uint16_t resource){
	if(ResourceOp<aos_resource_request>(resource)){
		resources[resource] = 1;
	}else{
		throw resourceexception(); // if we can't get a resource, we just want to stop
	}
}
template<class T, class S> bool AsyncAction<T, S>::TryRequestResource(uint16_t resource){
	if(ResourceOp<aos_resource_request>(resource)){
		resources[resource] = 1;
		return true;
	}else{
		return false;
	}
}
template<class T, class S> void AsyncAction<T, S>::ReleaseResource(uint16_t resource){
	if(ResourceOp<aos_resource_release>(resource)){
		if(resources.erase(resource) != 1)
			fprintf(stderr, "warning: value for resource %d wasn't 1\n", resource);
	}
}

#if 0
// from gcc's (used 4.6.2) libjava/include/i386-signal.h
/* We use kernel_sigaction here because we're calling the kernel
   directly rather than via glibc.  The sigaction structure that the
   syscall uses is a different shape from the one in userland and not
   visible to us in a header file so we define it here.  */
extern "C" 
{
  struct kernel_sigaction 
  {
    void (*k_sa_sigaction)(int,siginfo_t *,void *);
    unsigned long k_sa_flags;
    void (*k_sa_restorer) (void);
    sigset_t k_sa_mask;
  };
}
#define RESTORE(name, syscall) RESTORE2 (name, syscall)
#define RESTORE2(name, syscall)			\
asm						\
  (						\
   ".text\n"					\
   ".byte 0  # Yes, this really is necessary\n" \
   "	.align 16\n"				\
   "__" #name ":\n"				\
   "	movl $" #syscall ", %eax\n"		\
   "	int  $0x80"				\
   );
/* The return code for realtime-signals.  */
RESTORE (restore_rt, __NR_rt_sigreturn)
void restore_rt (void) asm ("__restore_rt")
  __attribute__ ((visibility ("hidden")));
#define INIT_SIGNAL(signal, sigaction)				\
do								\
  {								\
    struct kernel_sigaction act;				\
    act.k_sa_sigaction = sigaction;				\
    sigemptyset (&act.k_sa_mask);				\
    act.k_sa_flags = SA_SIGINFO|0x4000000;			\
    act.k_sa_restorer = restore_rt;				\
    syscall (SYS_rt_sigaction, signal, &act, NULL, _NSIG / 8);	\
  }								\
while (0)  
/* Unblock a signal.  Unless we do this, the signal may only be sent
   once.  */
static void 
unblock_signal (int signum __attribute__ ((__unused__)))
{
  sigset_t sigs;

  sigemptyset (&sigs);
  sigaddset (&sigs, signum);
  sigprocmask (SIG_UNBLOCK, &sigs, NULL);
}
#endif

template<class T, class S> void AsyncAction<T, S>::sig_action(int signum, siginfo_t *, void *){
  // TODO really shouldn't be using stdio in here (check before changing)
	/*unblock_signal(signum);
	// MAKE_THROW_FRAME(exception)
	std::exception *exception = new std::exception;
	throw exception;*/
	fprintf(stderr, "received signal %d\n", signum);
	if(signum == STOP_SIGNAL)
		interrupt |= 0x01;
	else if(signum == RESOURCE_KILL_SIGNAL)
		interrupt |= 0x02;
	else if(signum == SIGINT || signum == SIGTERM)
		interrupt |= 0x04;
	else
		fprintf(stderr, "unknown signal %d\n", signum);
}
template<class T, class S> int AsyncAction<T, S>::Run(uint8_t priority) {
	interrupt = 0;
	struct sigaction sigact;
	sigact.sa_sigaction = sig_action;
	sigact.sa_flags = 0;
	sigaction(STOP_SIGNAL, &sigact, NULL);
	sigaction(RESOURCE_KILL_SIGNAL, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL); // kill from the command line default

	if(resource_entity != NULL){
		fprintf(stderr, "resource_entity isn't already null, which means that this is the second Run being called in this process or something (which isn't allowed...)\n");
		return -1;
	}
	resource_entity = aos_resource_entity_create(priority);

	async_action_status<S> *status = (async_action_status<S> *)aos_queue_get_msg(status_queue);
	if(status == NULL)
		return -1;
	//memset(status + offsetof(aync_action_status<S>, status->status), 0x00, sizeof(S));
	status->count = 1;
	status->done_status = 0;
	local_done_status = 0;
	local_pid = getpid();
	fprintf(stderr, "local_pid=%d STOP_SIGNAL is currently %d\n", local_pid, STOP_SIGNAL);
	status->pid = local_pid;
	status->resource_entity = resource_entity;
	// put the initial status in
	if(aos_queue_write_msg(status_queue, status, OVERRIDE) < 0){
		aos_queue_free_msg(status_queue, status);
		return -1;
	}
	
	// clear out any pending start messages
	async_action_start<T> *start = (async_action_start<T> *)aos_queue_read_msg(start_queue, NON_BLOCK);
	aos_queue_free_msg(start_queue, start);

	OnStart();

	T args;
	uint16_t current_count;
	while(true){
		interrupt = 0;
		// wait for a new start message
		start = (async_action_start<T> *)aos_queue_read_msg(start_queue, PEEK);
		if(start == NULL){
			if(interrupt & 0x04)
				break;
			continue;
		}
		memcpy(&args, &start->args, sizeof(T));
		current_count = start->count;
		if(start->parent == NULL){ // Start isn't getting called from a process with an AsyncAction implementation running in it
			start->parent = aos_resource_entity_root_get();
		}
		aos_resource_entity_set_parent(resource_entity, start->parent);
		aos_queue_free_msg(start_queue, start);
		status = (async_action_status<S> *)aos_queue_get_msg(status_queue);
		if(status == NULL){
			fprintf(stderr, "%s: %d: could not get a status message to write initial status to\n", __FILE__, __LINE__);
			continue;
		}
		status->done_status = local_done_status = 0;
		status->pid = local_pid;
		status->count = current_count;
		status->resource_entity = resource_entity;
		if(aos_queue_write_msg(status_queue, status, OVERRIDE) < 0)
			aos_queue_free_msg(status_queue, status);

		try {
			DoAction(args);
		} catch (stopexception &e) {
			fprintf(stderr, "caught stopexception\n");
		} catch (resourceexception &e) {
			fprintf(stderr, "caught resourceexception\n");
		} catch (...) {
			fprintf(stderr, "caught another exception... (bad news)\n");
		}

		start = (async_action_start<T> *)aos_queue_read_msg(start_queue, NON_BLOCK);
		if(start == NULL){
			fprintf(stderr, "somebody else consumed the start message (at %s: %d)\n", __FILE__, __LINE__);
		} else {
			aos_queue_free_msg(start_queue, start);
		}

		status = (async_action_status<S> *)aos_queue_get_msg(status_queue);
		if(status == NULL){
			fprintf(stderr, "couldn't get a message to write that we're finished in to %s: %d\n", __FILE__, __LINE__);
			continue;
		}
		memcpy(&status->status, &local_status, sizeof(S));
		status->done_status = local_done_status | 0x02;
		status->pid = local_pid;
		status->count = current_count;
		status->resource_entity = resource_entity;
		if(aos_queue_write_msg(status_queue, status, OVERRIDE) < 0)
			aos_queue_free_msg(status_queue, status);

		std::map<uint16_t, uint8_t>::iterator it;
		for(it = resources.begin(); it != resources.end(); ++it){
			ReleaseResource(it->first);
		}
		if(!resources.empty()){
			fprintf(stderr, "resources isn't empty after releasing everything in it. clearing it\n");
			resources.clear();
		}

		if(interrupt & 0x04)
			break;
	}
	OnEnd();
	return 0;
}

template<class T, class S> void AsyncAction<T, S>::Sleep(double seconds){
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts.tv_sec += (time_t)floor(seconds);
	ts.tv_nsec += (long)((seconds - floor(seconds)) * 1000000000);
	if(ts.tv_nsec > 1000000000){
		ts.tv_nsec -= 1000000000;
		ts.tv_sec += 1;
	}
	int rv;
	do {
		CheckStop();
		rv = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
	} while(rv);
}

template<class T, class S> void AsyncAction<T, S>::DoAction(__attribute__((unused)) T &args){
	fprintf(stderr, "this (at %s: %d) should never get run\n", __FILE__, __LINE__);
	*((int *)NULL) = 0;
}

} // namespace aos

