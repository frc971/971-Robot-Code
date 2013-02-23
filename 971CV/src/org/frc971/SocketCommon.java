/**
 * 
 */
package org.frc971;

import java.io.IOException;

import java.nio.ByteBuffer;
import java.nio.channels.SocketChannel;
import java.util.logging.Logger;

/**
 * @author daniel
 * Socket operations used by other classes
 */
public class SocketCommon {

	private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
	
	/** Reads on a SocketStream until it finds a given character sequence. */
	public static String readtoBoundary(SocketChannel sock, String boundary) {
		//reads from socket until it encounters a specific character combination
		//if boundary is null, it reads until it runs out of data
		ByteBuffer recvd = ByteBuffer.allocate(1024);
		StringBuilder sb = new StringBuilder();
		String message = "";
		try {
			int ret = 0;
			while (ret != -1) {
				ret = sock.read(recvd);
				//System.out.println(ret);
				if (ret == 0) {
					//finished receiving
					message = sb.toString();
					if (boundary == null)
						break;
				}
				else {
					for (int i = 0; i < recvd.capacity() - recvd.remaining(); i++) {
						sb.append((char)recvd.get(i));
					}
					recvd.clear();
					if (boundary != null) {
						if (sb.toString().contains(boundary)) {
							message = sb.toString();
							break;
						}
						else {
							continue;
						}
					}
				}
			}
		}
		catch (IOException e) {
			LOG.severe("Socket read failed.");
			return null;
		}
		return message;
	}
	
	/** Guarantees that large messages will be completely sent through a socket.
	 * @return Returns 0 for success, -1 for failure.
	 */
	public static int sendAll(SocketChannel sock, ByteBuffer message) {
		message.rewind();
		while (message.remaining() > 0) {
			try {
				sock.write(message);
			}
			catch (IOException e) {
				LOG.warning("Socket write failed.");
				return -1;
			}
		}
		return 0;
	}
	
	/** Overloaded method for sending a byte array. */
	public static void sendAll(SocketChannel sock, byte[] message) {
		ByteBuffer buff = ByteBuffer.wrap(message);
		sendAll(sock, buff);
	}
	
	/** Overloaded method for sending a String. */
	public static void sendAll(SocketChannel sock, String message) {
		sendAll(sock, message.getBytes());
	}
}
