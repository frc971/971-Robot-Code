package com.ctre.phoenix;

public interface ILoopable
{
	public void onStart();
	public void onLoop();
	public boolean isDone();
	public void onStop();
}