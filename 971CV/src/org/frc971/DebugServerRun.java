package org.frc971;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.InterruptedIOException;
import java.io.PrintWriter;

import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.GatheringByteChannel;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.googlecode.javacv.OpenCVFrameGrabber;
import com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;

public class DebugServerRun {
	
		private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
		
		final OpenCVFrameGrabber grabber = new OpenCVFrameGrabber(-1);
		
		private ServerSocketChannel sock;
		private SocketChannel client;
		
		private BufferedReader sock_in;
		private PrintWriter sock_out;
		
		private String ReadtoBoundary(String boundary) {
			//reads from socket until it encounters a specific character combination
			//if boundary is null, it reads until it runs out of data
			ByteBuffer recvd = ByteBuffer.allocate(1024);
			StringBuilder sb = new StringBuilder();
			String message = "";
			try {
				int ret = 0;
				while (ret != -1) {
					ret = client.read(recvd);
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
	private void push() {
		//push data to client
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
		        while (to_send.remaining() > 0) {
		        	client.write(to_send);
		        }
			}
			catch (Exception e) {
				LOG.warning("Could not grab frame.");
				continue;
			}
		}
	}
		
	public void Connect() throws IOException {
		client = sock.accept();
		client.configureBlocking(false);
		//sock_in = new BufferedReader(new InputStreamReader(client.socket().getInputStream()));
		//sock_out = new PrintWriter(client.socket().getOutputStream(), true);
		//we are now connected to our client. Wait for them to send us a header.
		LOG.info("Reading headers...");
		ReadtoBoundary("\r\n\r\n");
		//send one back
		LOG.info("Writing headers...");
		String ending = "donotcross\r\n";
		ByteBuffer buff = ByteBuffer.wrap(ending.getBytes());
		while (buff.remaining() > 0) {
			client.write(buff);
		}
	}
	
	public DebugServerRun(final int port) throws IOException {
		sock = ServerSocketChannel.open();
		sock.socket().bind(new InetSocketAddress(9714));
	}
	public static void main(final String args[]) throws IOException{
		//main function for server
		
		//set logger to log everything
        LOG.setLevel(Level.ALL);
        try {
        	LogHandler handler = new LogHandler("../src/org/frc971/ds_vision.log");
        	TimeFormatter formatter = new TimeFormatter();
            handler.setFormatter(formatter);
            LOG.addHandler(handler);
        }
        catch (FileNotFoundException e) {
        	System.err.println("Warning: Logging initialization failed.");
        }
        
		DebugServerRun server = new DebugServerRun(9714);
		new TestClient();
		server.Connect();
		server.push();
	}
}
