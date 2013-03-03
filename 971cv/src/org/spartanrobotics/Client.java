/**
 * 
 */
package org.spartanrobotics;

import java.nio.ByteBuffer;
import java.nio.channels.SocketChannel;

/**
 * @author daniel
 *
 */

/** Helper class to store data for AccepterThread. */
public class Client {
	public SocketChannel channel; //the client's socket
	/* Holds overflow data when socket's send buffer gets full, so that
	 * thread can continue running.
	 */
	public ByteBuffer toSend;
	/* Keeps track of how many times a non-blocking write operation on a socket
	 * has not written anything because it's buffer was full.
	 */
	public int failedAttempts;
}
