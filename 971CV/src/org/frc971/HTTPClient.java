package org.frc971;

//Author: Daniel Petti

import java.io.*;
import java.net.*;

import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;

import com.googlecode.javacv.cpp.opencv_core.IplImage;

import aos.ChannelImageGetter;
import aos.JPEGDecoder;

import java.nio.channels.SocketChannel;
import java.nio.ByteBuffer;

public class HTTPClient {
	//Connects to HTTP Server on robot and receives images
	
	private final static boolean LOCAL_DEBUG = true;
	
	private SocketChannel sock = SocketChannel.open();
	private Socket core_sock = sock.socket();
	
	private String LastBoundary = "";
	
	private BufferedReader sock_in;
	private PrintWriter sock_out;
	
	private void WriteDebug(String message) {
		//small helper function to write debug messages
		if (LOCAL_DEBUG)
			System.out.println(message);
	}
	private String ReadtoBoundary(String boundary) {
		//reads from socket until it encounters a specific character combination
		//if boundary is null, it reads until it runs out of data
		StringBuilder recvd = new StringBuilder();
		String message = "";
		try {
			core_sock.setSoTimeout(10000);
		}
		catch (SocketException e) {
			System.err.println("Warning: Could not set socket timeout.");
		}
		try {
			int ret;
			while ((ret = sock_in.read()) != -1) {
				if (ret == 0) {
					//finished receiving
					message += recvd.toString();
					recvd.setLength(0);
					if (boundary == null)
						break;
				}
				else {
					recvd.append((char)ret);
					if (boundary != null) {
						if (message.contains(boundary))
							break;
						else
							continue;
					}
				}
			}
		}
		catch (InterruptedIOException e) {
			System.err.println("Warning: Image receive timed out.");
			return null;
		}
		catch (IOException e) {
			System.err.println("Error: Socket read failed.");
			return null;
		}
		return message;
	}
	public HTTPClient() {
		//Initialize socket connection to robot
		try {
			WriteDebug("Connecting to server...");
			sock.connect(new InetSocketAddress("192.168.0.137", 9714));
			sock_in = new BufferedReader(new InputStreamReader(core_sock.getInputStream()));
			sock_out = new PrintWriter(core_sock.getOutputStream(), true);
			//Write headers
			//HTTPStreamer does not actually use the headers, so we can just write terminating chars.
			WriteDebug("Writing headers...");
			sock_out.println("\r\n\r\n");
			//Receive headers
			WriteDebug("Reading headers...");
			ReadtoBoundary("donotcross\r\n");
			WriteDebug("Now receiving data.");
		}
		catch (UnknownHostException e) {
			System.err.println("Error: Invalid host.");
			System.exit(1);
		}
		catch (IOException e) {
			System.err.println("Error: Socket IO failed.");
			System.exit(2);
		}
		
	}
	public ImageWithTimestamp GetFrame() {
		/*//read all current data from socket, in case of processing bottleneck
		WriteDebug("Emptying TCP stack...");
		String message = ReadtoBoundary(null);
		//we must end with a boundary
		int len = message.length();
		if (message.substring(len - 4, len - 1) != "\r\n\r\n") {
			WriteDebug("Boundary was not found. Waiting for boundary...");
			message += ReadtoBoundary("\r\n\r\n");
		}
		//Add the last boundary we cut off to the beginning of our message
		message = LastBoundary + message;
		//Divide our large message into string separated by boundary beginnings
		String[] cut = message.split("\r\n--");
		len = cut.length;
		//Save the newest boundary, so we can use it later.
		LastBoundary = cut[len - 1];
		//Keep only our penultimate boundary and the image after it
		//NOTE that message is missing its preceding \r\n--
		message = cut[len - 2];
		cut = message.split("\r\n\r\n");
		//NOTE that boundary is now missing its trailing \r\n\r\n
		String boundary = cut[0];
		String JPEGImage = cut[1];
		//extract timestamp in seconds
		cut = boundary.split("X-Timestamp: ");
		String time_data = cut[1];
		boundary = cut[0];
		float timestamp = Float.parseFloat(time_data);
		//extract size so we can make sure our image data is not corrupted
		cut = boundary.split("Content-Length: ");
		String size_data = cut[1];
		float size = Float.parseFloat(size_data);
		assert (size == JPEGImage.length()) : ("Fatal mismatch between actual and expected image size. Check regular expressions.");
		byte[] ImageArray = JPEGImage.getBytes();
		InputStream in = new ByteArrayInputStream(ImageArray);
		try {
			BufferedImage bImageFromConvert = ImageIO.read(in);
			ImageWithTimestamp to_return = new ImageWithTimestamp();
			to_return.image = IplImage.createFrom(bImageFromConvert);
			to_return.timestamp = timestamp;
			WriteDebug("Image processing successful.");
			return to_return;
		}
		catch (IOException e) {
			System.err.println(e.getMessage());
			return null;
		}
		*/
		//Use Brian's code to extract an image and timestamp from raw server data.
		ImageWithTimestamp final_image = new ImageWithTimestamp();
		try {
			ChannelImageGetter cgetter = new ChannelImageGetter(sock);
			ByteBuffer binary_image = cgetter.getJPEG();
			JPEGDecoder decoder = new JPEGDecoder();
			boolean sorf = decoder.decode(binary_image, final_image.image);
			if (!sorf) {
				WriteDebug("Error: JPEG decode failed.");
				return null;
			}
			final_image.timestamp = cgetter.getTimestamp();
			return final_image;
		}
		catch (IOException e) {
			WriteDebug("Error: Failed to initialize ChannelImageGetter.");
			return null;
		}
	}	
}
