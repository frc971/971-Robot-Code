package org.frc971;

//Author: Daniel Petti

import java.io.*;
import java.net.*;

import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;

import com.googlecode.javacv.cpp.opencv_core.IplImage;

import aos.ChannelImageGetter;

import java.nio.channels.SocketChannel;
import java.nio.ByteBuffer;

public class HTTPClient {
	//Connects to HTTP Server on robot and receives images
	
	private final static boolean LOCAL_DEBUG = true;
	
	private SocketChannel sock = SocketChannel.open();
	private Socket core_sock = sock.socket();
	
	
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
		//Use Brian's code to extract an image and timestamp from raw server data.
		ImageWithTimestamp final_image = new ImageWithTimestamp();
		try {
			ChannelImageGetter cgetter = new ChannelImageGetter(sock);
			ByteBuffer binary_image = cgetter.getJPEG();
			//Decode ByteBuffer into an IplImage
			InputStream in = new ByteArrayInputStream(binary_image.array());
			try {
				BufferedImage bImageFromConvert = ImageIO.read(in);
				final_image.image = IplImage.createFrom(bImageFromConvert);
				final_image.timestamp = cgetter.getTimestamp();
				WriteDebug("Image processing successful.");
				return final_image;
			}
			catch (IOException e) {
				System.err.println(e.getMessage());
				return null;
			}
			
		}
		catch (IOException e) {
			WriteDebug("Error: Failed to initialize ChannelImageGetter.");
			return null;
		}
	}	
}
