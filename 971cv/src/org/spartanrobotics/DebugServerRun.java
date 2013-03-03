package org.spartanrobotics;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;

import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.googlecode.javacv.OpenCVFrameGrabber;
import com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;

public class DebugServerRun {
	
		private final static Logger LOG = Logger.getLogger(
			DebugServerRun.class.getName());
		
		final OpenCVFrameGrabber grabber = new OpenCVFrameGrabber(-1);
		
		private ServerSocketChannel sock;
		private SocketChannel client;
	
	/** Constructs a formatted boundary header from a timestamp and content length. */	
	private ByteBuffer CreateTransmission(long content_length, double timestamp) {
		StringBuilder ret = new StringBuilder();
		ret.append("\r\n--boundarydonotcross\r\n");
        ret.append("Content-Type: image/jpeg\r\n");
        ret.append("Content-Length: ");
        ret.append(content_length);
        ret.append("\r\n");
        ret.append("X-Timestamp: ");
        ret.append(timestamp);
        ret.append("\r\n\r\n");
        return ByteBuffer.wrap(ret.toString().getBytes());
	}
	
	/** Loop that pushes a data stream to the client. */
	private void push() {
		try {
			grabber.start();
		}
		catch (Exception e) {
			LOG.severe("Could not start frame grabber.");
			return;
		}
		IplImage img;
		long content_size;
		File buff_file;
		InputStream input;
		double timestamp;
		while (true) {
			//get some image data
			try {
				img = grabber.grab();
				timestamp = System.currentTimeMillis();
				/*We buffer through /dev/shm, just to make the conversion process easier.
				 * I know this is really ugly, but it works a lot better than what
				 * I was doing before, which segfaulted.
				 */
				cvSaveImage("/dev/shm/DebugServerBuffer.jpg", img);
				buff_file = new File("/dev/shm/DebugServerBuffer.jpg");
				content_size = buff_file.length();
				int totalBytesRead = 0;
		        input = new BufferedInputStream(new FileInputStream(buff_file));
		        byte[] result = new byte[(int)content_size];
		        while(totalBytesRead < result.length){
		          int bytesRemaining = result.length - totalBytesRead;
		          //input.read() returns -1, 0, or more :
		          int bytesRead = input.read(result, totalBytesRead, bytesRemaining); 
		          if (bytesRead > 0){
		            totalBytesRead = totalBytesRead + bytesRead;
		          }
		        }
		        ByteBuffer header = CreateTransmission(content_size, timestamp);
		        ByteBuffer bbuf = ByteBuffer.wrap(result);
		        ByteBuffer to_send = ByteBuffer.allocate(header.capacity() + bbuf.capacity());
		        to_send.put(header);
		        to_send.put(bbuf);
		        to_send.rewind();
		        SocketCommon.sendAll(client, to_send);
			}
			catch (Exception e) {
				LOG.warning("Could not grab frame.");
				continue;
			}
		}
	}
	
	/** Constructor to start the server and bind it to a port. */
	public DebugServerRun(final int port) throws IOException {
		sock = ServerSocketChannel.open();
		sock.socket().bind(new InetSocketAddress(9714));
		client = sock.accept();
		client.configureBlocking(false);
		//we are now connected to our client. Wait for them to send us a header.
		LOG.info("Reading headers...");
		SocketCommon.readtoBoundary(client, "\r\n\r\n");
		//send one back
		LOG.info("Writing headers...");
		SocketCommon.sendAll(client, "donotcross\r\n");
	}
	
	/** Runs the server, and concurrently starts the vision processor with -vision flag. */
	public static void main(final String args[]) throws IOException {
		//main function for server
		
		String atomIP = null;
    	try {
    		atomIP = args[0];
    	}
    	catch (ArrayIndexOutOfBoundsException e) {
    		System.out.println("Usage: DebugServerRun [atom IP address]");
    		System.exit(0);
    	}
		
		//set logger to log everything
        LOG.setLevel(Level.ALL);
        try {
        	LogHandler handler = new LogHandler("ds_vision.log");
        	TimeFormatter formatter = new TimeFormatter();
            handler.setFormatter(formatter);
            LOG.addHandler(handler);
        }
        catch (FileNotFoundException e) {
        	System.err.println("Warning: Logging initialization failed.");
        }
        
		if (Arrays.asList(args).contains("-vision")) {
			LOG.info("Starting vision processor.");
			new TestClient(atomIP);
		}
		
		DebugServerRun server = new DebugServerRun(9714);
		server.push();
	}
}
