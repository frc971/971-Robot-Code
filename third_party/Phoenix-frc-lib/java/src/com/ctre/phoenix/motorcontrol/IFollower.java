package com.ctre.phoenix.motorcontrol;
import com.ctre.phoenix.motorcontrol.IMotorController;

public interface IFollower
{
	void follow(IMotorController masterToFollow);
	void valueUpdated();
}