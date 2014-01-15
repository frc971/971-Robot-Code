package org.spartanrobotics.camera;

import aos.NativeLoader;
import aos.QueueLogHandler;
import frc971.control_loops.Piston_q;

public class QueueTest {
	
	public static void main(String[] args) {
		QueueLogHandler.UseForAll();
		NativeLoader.load("frc971_queues_so");
		Piston_q.shifters.SafeMakeWithBuilder().set(false).Send();
	}
	
}
