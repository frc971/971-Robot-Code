package org.spartanrobotics.camera;

import aos.NativeLoader;
import aos.QueueLogHandler;
import frc971.control_loops.Piston_q;

public class QueueReader {

	public static void main(String[] args) {
		QueueLogHandler.UseForAll();
		NativeLoader.load("frc971_queues_so");
		System.out.println(Piston_q.shifters.FetchLatest());
		System.out.println(Piston_q.shifters.getSet());
	}

}
