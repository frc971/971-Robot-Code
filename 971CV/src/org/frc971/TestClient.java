/**
 * 
 */
package org.frc971;

/**
 * @author daniel
 *
 */
public class TestClient extends Thread {
	public TestClient() {
		super("Test Client");
		start();
	}
	public void run() {
		String[] args = {};
		VisionTuner.main(args);
	}
}
