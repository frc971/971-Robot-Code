#include "TestAction.h"
#include <AsyncActionRunner.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

//void TestingAction_t::OnStart() {printf("started\n");}
//void TestingAction_t::OnEnd() {printf("ended\n");}
void TestAction_t::DoAction(double seconds, int loops) {
	printf("start of DoAction\n");
	RequestResource(test_resource1);
	bool release = loops < 0;
	if(release)
		loops *= -1;
	bool release_bad = seconds < 0;
	if(release_bad)
		seconds *= -1;
	int i;
	for(i = 0; i < loops; ++i) {
		Sleep(seconds);
		printf("posting for loop %d\n", i);
		PostStatus(i, seconds * i);
		printf("posted for loop %d\n", i);
		if(release && i > loops / 2){
			printf("releasing resource\n");
			ReleaseResource(test_resource1);
		}
		if(release_bad && i > loops / 2){
			printf("releasing resource which has not been requested\n");
			ReleaseResource(test_resource2);
		}
	}
	printf("end of DoAction\n");
}

