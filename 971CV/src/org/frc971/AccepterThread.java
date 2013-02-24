/**
 * 
 */
package org.frc971;

/**
 * @author daniel
 * Accepts clients for data server
 */

import java.io.IOException;

import java.nio.ByteBuffer;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;

public class AccepterThread extends Thread {
	
	private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
	
	private ServerSocketChannel sock;
	
	private List<SocketChannel> connected = new ArrayList<SocketChannel>(); 
	
	/* Holds overflow data when socket's send buffer gets full, so that
	 * thread can continue running.
	 */
	private Map<SocketChannel, ByteBuffer> toSend;
	/* Keeps track of how many times a write operation on a socket
	 * has failed because it's buffer was full.
	 */
	private Map<SocketChannel, Integer> failedAttempts; //doesn't like primitive types
	
	/** Helper function to completely erase a peer from
	 *  all three lists and maps that might contain it.
	 */
	private void erasePeer(SocketChannel peer) {
		connected.remove(peer);
		toSend.remove(peer);
		failedAttempts.remove(peer);
	}
	
	/** Constructor
	 * 
	 * @param sock is the ServerSocketChannel that you want to monitor
	 */
	public AccepterThread(ServerSocketChannel sock) {
		super("Accepter Thread");
		setPriority(3); //lowish priority so Image Processor overrides it
		this.sock = sock;
		start();
	}
	
	/** Runs in separate thread. Continually accepts new connections. */
	public void run() {
		SocketChannel clientSock;
		while (true) {
			try {
				clientSock = sock.accept();
				//our writes must not block
				clientSock.configureBlocking(false);
				connected.add(clientSock);
			}
			catch (IOException e) {
				LOG.warning("Cannot serve image processing results to client:" + e.getMessage());
				Messages.warning("Cannot serve image processing results to client:" + e.getMessage());
			}
		}
	}
	
	/** Sends a message to all currently connected clients.
	 * 
	 * @param message is the message that you want to send.
	 */
	public void sendtoAll(ByteBuffer message) {
		/* Copy our connected list, so we don't have 
		 * to hold our lock forever if the writes block.
		 */
		List<SocketChannel> connectedTemp = new ArrayList<SocketChannel>();
		for (SocketChannel channel : connected) {
			connectedTemp.add(channel);
		}
		
		int result;
		for (SocketChannel conn : connectedTemp) {
			try {
				
				/** If this socket has data from the 
				 * last send operation still waiting to be
				 * sent, send this instead of our original
				 * message. Since we generally want only
				 * current data, our original message will
				 * not be missed. However, it is imperative
				 * that we finish our pending transmission, 
				 * because an incomplete transmission could
				 * leave a client thread somewhere blocking
				 * indefinitely.
				 */
				if (toSend.containsKey(conn)) {
					message = toSend.get(conn);
				}
				
				result = conn.write(message);
				
				/*if our send buffer is full, store our message away
				 * so we can try again later without halting the thread.
				 */
				if (message.remaining() > 0) {
					toSend.put(conn, message);
					//check and update our count of failed send attempts
					if (failedAttempts.containsKey(conn)) {
						int failures = failedAttempts.get(conn);
						++failures;
						if (failures >= 100) {
							//Socket has become dysfunctional
							LOG.info("Write would have blocked 100 times. Assuming peer disconect.");
							erasePeer(conn);
						}
						failedAttempts.put(conn, failures);
					}
					else {
						failedAttempts.put(conn, 1);
					}
				}
				
				if (result == -1) {
					//The write failed. This is probably because the client disconnected.
					LOG.info("Write returned -1. Client has probably disconnected.");
					erasePeer(conn);
				}
			}
			catch (IOException e) {
				//The write failed. This is probably because the client disconnected.
				LOG.info("Write threw IOException. Client has probably disconnected.");
				erasePeer(conn);
			}
		}
	}
	
	/** Overloaded sendtoAll method for byte arrays. */
	public void sendtoAll(byte[] message) {
		sendtoAll(ByteBuffer.wrap(message));
	}
	
	/** Overloaded sendtoAll method for Strings. */
	public void sendtoAll(String message) {
		sendtoAll(message.getBytes());
	}
}
