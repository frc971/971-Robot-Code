package com.ctre.phoenix.schedulers;
import java.util.ArrayList;

import com.ctre.phoenix.ILoopable;

public class ConcurrentScheduler implements ILoopable {
	ArrayList<ILoopable> _loops = new ArrayList<ILoopable>();
	ArrayList<Boolean> _enabs = new ArrayList<Boolean>();

	public void add(ILoopable aLoop, boolean enable) {
		_loops.add(aLoop);
		_enabs.add(enable);
	}
	public void add(ILoopable aLoop) {
		add(aLoop, true);
	}

	public void removeAll() {
		_loops.clear();
		_enabs.clear();
	}

	public void start(ILoopable toStart) {
		for (int i = 0; i < _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			if (lp == toStart) {
				_enabs.set(i, true);
				lp.onStart();
				return;
			}
		}

	}

	public void stop(ILoopable toStop) {
		for (int i = 0; i < (int) _loops.size(); ++i) {
			ILoopable lp = (ILoopable) _loops.get(i);

			if (lp == toStop) {
				_enabs.set(i, false);
				lp.onStop();
				return;
			}
		}
	}

	public void startAll() { // All Loops

		for (ILoopable loop : _loops) {
			loop.onStart();
		}
		for (int i = 0; i < _enabs.size(); ++i) {
			_enabs.set(i,  true);
		}
	}

	public void stopAll() { // All Loops
		for (ILoopable loop : _loops) {
			loop.onStop();
		}
		for (int i = 0; i < _enabs.size(); ++i) {
			_enabs.set(i,  false);
		}
	}

	public void process() {
		for (int i = 0; i < (int) _loops.size(); ++i) {
			ILoopable loop = (ILoopable) _loops.get(i);
			boolean en = (boolean) _enabs.get(i);
			if (en) {
				loop.onLoop();
			} else {
				/* Current ILoopable is turned off, don't call onLoop for it */
			}
		}
	}

	/* ILoopable */
	public void onStart() {
		startAll();
	}

	public void onLoop() {
		process();
	}

	public void onStop() {
		stopAll();
	}

	public boolean isDone() {
		return false;
	}
}