#ifndef VISION_RINGBUFFER_H_
#define VISION_RINGBUFFER_H_

#include <stdio.h>
#include <stdlib.h>

namespace frc971 {
namespace vision {

template<class T,class V>
class RingBuffer{
  //record class to hold sampes
	class Samples{
		public:
	  T time;
		V value;
	};  
	Samples *samples;
	int current_;
	int wraps_;
	V value_at(int index){
		return samples[index & 255].value;
	}
	T time_at(int index){
		return samples[index & 255].time;
	}
	public:
	RingBuffer(){
		current_ = 0;
		wraps_ = 0;
		samples = (Samples *)malloc(sizeof(Samples) * 256);
	}
	// Adds samples into the ringbuffer.
	void Sample(T time,V val){
		current_ += 1;
		wraps_ += current_ / 256;
		current_ = current_ % 256;
		samples[current_].time = time;
		samples[current_].value = val;
	}
	// Binary Search to find and interpolate the values.
	V ValueAt(T time){
		int start = current_ - 255;
		int end = current_;
		if(start < 0 && !wraps_){ 
			start = 0;
		}   
		int max = end;
		int min = start;
		while(end - start > 1){
			int mid = (start + end) / 2;
			Samples check = samples[mid & 255];
			if(check.time < time){
				start = mid;
				min = mid;
			}else{
				max = mid;
				end = mid;
			}   
		}
		return value_at(min) + (value_at(max) - value_at(min)) *
		((time - time_at(min)).ToSeconds()/(time_at(max) - time_at(min)).ToSeconds());
	}
};
};  // vision
};  // frc971
#endif // VISION_RINGBUFFER_H_
