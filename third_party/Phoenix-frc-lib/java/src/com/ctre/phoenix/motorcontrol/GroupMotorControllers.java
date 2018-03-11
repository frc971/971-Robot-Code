package com.ctre.phoenix.motorcontrol;

import java.util.*;

public class GroupMotorControllers
{
	static List<IMotorController> _ms = new ArrayList<IMotorController>();
	
	public static void register(IMotorController mc)
	{
		_ms.add(mc);
	}
	
	public static int getCount()
	{
		return _ms.size();
	}
	
	public static IMotorController get(int idx)
	{
		return _ms.get(idx);
	}
}