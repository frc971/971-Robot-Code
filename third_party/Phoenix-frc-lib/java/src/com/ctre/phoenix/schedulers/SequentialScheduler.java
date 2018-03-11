package com.ctre.phoenix.schedulers;
import java.util.ArrayList;

public class SequentialScheduler implements com.ctre.phoenix.ILoopable
{
	boolean _running = false;
	ArrayList<com.ctre.phoenix.ILoopable> _loops = new ArrayList<com.ctre.phoenix.ILoopable>();
	int _periodMs;
	int _idx;
	boolean _iterated = false;

	public SequentialScheduler(int periodMs)
	{
		_periodMs = periodMs;
	}
	public void add(com.ctre.phoenix.ILoopable aLoop)
	{
		_loops.add(aLoop);
	}

	public com.ctre.phoenix.ILoopable getCurrent()
	{
		return null;
	}

	public void removeAll()
	{
		_loops.clear();
	}

	public void start()
	{
		_idx = 0;
		if(_loops.size() > 0)
			_loops.get(0).onStart();
		
		_running = true;
	}
	public void stop()
	{
		for (int i = 0; i < _loops.size(); i++)
		{
			_loops.get(i).onStop();
		}
		_running = false;
	}
	public void process()
	{
		_iterated = false;
		if (_idx < _loops.size())
		{
			if (_running)
			{
				com.ctre.phoenix.ILoopable loop = _loops.get(_idx);
				loop.onLoop();
				if (loop.isDone())
				{
					_iterated = true;
					++_idx;
					if (_idx < _loops.size()) _loops.get(_idx).onStart();
				}
			}
		}
		else
		{
			_running = false;
		}
	}
	public boolean iterated()
	{
		return _iterated;
	}
	//--- Loopable ---/
	public void onStart()
	{
		start();
	}

	public void onLoop()
	{
		process();
	}

	public boolean isDone()
	{
		/* Have to return something to know if we are done */
		if (_running == false)
			return true;
		else
			return false;
	}

	public void onStop()
	{
		stop();
	}
}