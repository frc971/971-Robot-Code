package com.ctre.phoenix.time;

public class StopWatch
{
	private long _t0 = 0;
	private long _t1 = 0;
	
	public void start()
	{
		_t0 = System.currentTimeMillis();
	}
	
	public double getDuration()
	{
		return (double)getDurationMs() / 1000;
	}
	public int getDurationMs()
	{
		_t1 = System.currentTimeMillis();
		long retval = _t1 - _t0;
		if(retval < 0)
			retval = 0;
		return (int)retval;
	}
}