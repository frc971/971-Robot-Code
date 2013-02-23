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
	
	/** whether or not to print debug messages to stdout. */
	private final static boolean LOCAL_DEBUG = false;
	
	private SocketChannel sock;
	
	private final String ATOM_IP = "192.168.0.137";
	
	private ChannelImageGetter cgetter;
	
	private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
	
	/** Small helper method for printing debug messages to stdout. */
	private void WriteDebug(String message) {
		//small helper function to write debug messages
		if (LOCAL_DEBUG)
			LOG.info("LOCAL_DEBUG: " + message);
	}
	
	/** the constructor, initializes connection, and sets up aos getter. 
	 * @throws IOException */
	public HTTPClient() throws IOException {
		//Initialize socket connection to robot
		sock = SocketChannel.open();
		WriteDebug("Connecting to server at " + ATOM_IP);
		sock.connect(new InetSocketAddress(ATOM_IP, 9714));
		sock.configureBlocking(false);
		//Write headers
		//HTTPStreamer does not actually use the headers, so we can just write terminating chars.
		WriteDebug("Writing headers...");
		SocketCommon.sendAll(sock, "\r\n\r\n");
		//Receive headers
		WriteDebug("Reading headers...");
		SocketCommon.readtoBoundary(sock, "donotcross\r\n");
		WriteDebug("Now receiving data.");
		cgetter = new ChannelImageGetter(sock);
	}
	
	/** Grabs the most current frame from the HTTPStreamer stream.
	 * Returns a class instance with image and timestamp attributes. */
	public ImageWithTimestamp GetFrame() {
		ImageWithTimestamp final_image = new ImageWithTimestamp();
		//Use Brian's code to extract an image and timestamp from raw server data.
		ByteBuffer binary_image = cgetter.getJPEG();
		//Decode ByteBuffer into an IplImage
		byte[] b = new byte[binary_image.remaining()];
		binary_image.get(b);
		try {
			InputStream iis = new ByteArrayInputStream(b);
			BufferedImage bImageFromConvert = ImageIO.read(iis);
			final_image.image = new WPIColorImage(bImageFromConvert);
			final_image.timestamp = cgetter.getTimestamp();
			WriteDebug("Image processing successful.");
			return final_image;
		}
		catch (IOException e) {
			LOG.warning("Image processing failed: " + e.getMessage());
			return null;
		}
	}	
}
