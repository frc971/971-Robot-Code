/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

namespace ctre {
namespace phoenix {
namespace signals {

class MovingAverage {
private:

	int _in; //!< head ptr for ringbuffer
	int _ou; //!< tail ptr for ringbuffer
	int _cnt; //!< number of element in ring buffer
	int _cap; //!< capacity of ring buffer
	float _sum; //!< sum of all elements in ring buffer
	float * _d; //!< ring buffer
public:
	MovingAverage(int capacity) {
		_cap = capacity;
		_d = new float[_cap];
		Clear();
	}
	float Process(float input) {
		Push(input);
		return _sum / (float) _cnt;
	}
	void Clear() {
		_in = 0;
		_ou = 0;
		_cnt = 0;

		_sum = 0;
	}
	void Push(float d) {
		/* process it */
		_sum += d;

		/* if full, pop one */
		if (_cnt >= _cap)
			Pop();

		/* push new one */
		_d[_in] = d;
		if (++_in >= _cap)
			_in = 0;
		++_cnt;
	}
	void Pop() {
		/* get the oldest */
		float d = _d[_ou];

		/* process it */
		_sum -= d;

		/* pop it */
		if (++_ou >= _cap)
			_ou = 0;
		--_cnt;
	}
	//-------------- Properties --------------//
	float GetSum() {
		return _sum;
	}
	int GetCount() {
		return _cnt;
	}
};

} // namespace  Signals
} // namespace phoenix
} // namespace ctre

