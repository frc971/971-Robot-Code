/**
 * 
 */
package org.frc971;

import java.io.IOException;

import java.net.InetSocketAddress;

import java.nio.channels.ServerSocketChannel;

/**
 * @author daniel
 *
 */

/**  Serves processing results back to the atom. */
public class ResultSender {
	private static final int PORT = 9716;
	
	private ServerSocketChannel sock;
	
	AccepterThread acceptor;
	
	/** Constructor. Connects to a socket and starts the accepter thread. */
	public ResultSender() throws IOException {
		sock = ServerSocketChannel.open();
		sock.socket().bind(new InetSocketAddress(PORT));
		
		//start accepter thread
		acceptor = new AccepterThread(sock);
	}
	
	/** Sends a new message of calculated attributes to the clients.
	 * 
	 * @param azimuth is the calculated optimum azimuth for the shot.
	 * @param elevation is the calculated optimum elevation for the shot.
	 * @param range is the calculated optimum range for the shot.
	 */
	public void send(double azimuth, double elevation, double range) {
		//Formulate a message as a String similar to an HTTP header.
		if (azimuth != -1 && elevation != -1 && range != -1) {
			StringBuilder message = new StringBuilder();
			message.append("\r\n--boundarydonotcross\r\n");
			message.append("Azimuth: ");
			message.append(azimuth);
			message.append("\r\nElevation: ");
			message.append(elevation);
			message.append("\r\nRange: ");
			message.append(range);
			
			acceptor.sendtoAll(message.toString());
		}
	}
}
