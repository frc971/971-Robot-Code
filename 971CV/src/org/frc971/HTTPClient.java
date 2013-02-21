package org.frc971;

//@author: daniel

import java.io.*;
import java.net.*;

import java.awt.image.BufferedImage;

import java.nio.channels.SocketChannel;
import java.nio.ByteBuffer;

import java.util.logging.Logger;

import javax.imageio.ImageIO;

import aos.ChannelImageGetter;

import edu.wpi.first.wpijavacv.WPIColorImage;

public class HTTPClient {
	//Connects to HTTP Server on robot and receives images
	
	private final static boolean LOCAL_DEBUG = true;
	
	private SocketChannel sock;
	private Socket core_sock;
	
	
	private BufferedReader sock_in;
	private PrintWriter sock_out;
	
	private final String ATOM_IP = "10.9.71.6";
	
	private ChannelImageGetter cgetter;
	
	private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
	
	private void WriteDebug(String message) {
		//small helper function to write debug messages
		if (LOCAL_DEBUG)
			LOG.info("LOCAL_DEBUG: " + message);
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
			LOG.warning("Could not set socket timeout.");
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
			LOG.warning("Image receive timed out.");
			return null;
		}
		catch (IOException e) {
			LOG.severe("Socket read failed.");
			return null;
		}
		return message;
	}
	public HTTPClient() {
		//Initialize socket connection to robot
		try {
			sock = SocketChannel.open();
			core_sock = sock.socket();
			WriteDebug("Connecting to server at " + ATOM_IP);
			sock.connect(new InetSocketAddress(ATOM_IP, 9714));
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
			cgetter = new ChannelImageGetter(sock);
		}
		catch (UnknownHostException e) {
			LOG.severe("Invalid host.");
			System.exit(1);
		}
		catch (IOException e) {
			LOG.severe("Socket IO failed.");
			System.exit(2);
		}
		
	}
	public ImageWithTimestamp GetFrame() {
		//Use Brian's code to extract an image and timestamp from raw server data.
		ImageWithTimestamp final_image = new ImageWithTimestamp();
		ByteBuffer binary_image = cgetter.getJPEG();
		//Decode ByteBuffer into an IplImage
		InputStream in = new ByteArrayInputStream(binary_image.array());
		try {
			BufferedImage bImageFromConvert = ImageIO.read(in);
			final_image.image = new WPIColorImage(bImageFromConvert);
			final_image.timestamp = cgetter.getTimestamp();
			WriteDebug("Image processing successful.");
			return final_image;
		}
		catch (IOException e) {
			LOG.warning("Image processing failed.");
			return null;
		}
	}	
}
