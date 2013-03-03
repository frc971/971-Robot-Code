package org.spartanrobotics;

//@author: daniel

import java.io.*;
import java.net.*;

import java.awt.image.BufferedImage;

import javax.imageio.ImageIO;

import java.nio.channels.SocketChannel;
import java.nio.ByteBuffer;

import java.util.logging.Logger;

import aos.ChannelImageGetter;

import edu.wpi.first.wpijavacv.WPIColorImage;

public class HTTPClient implements ImageGetter {
	//Connects to HTTP Server on robot and receives images
	
	/** whether or not to print debug messages to stdout. */
	private final static boolean LOCAL_DEBUG = false;
	
	private String atomIP;
	
	private SocketChannel sock;
	
	private ChannelImageGetter cgetter;
	
	private final static Logger LOG = Logger.getLogger(
			HTTPClient.class.getName());
	
	/** Small helper method for printing debug messages to stdout. */
	private void WriteDebug(String message) {
		//small helper function to write debug messages
		if (LOCAL_DEBUG)
			LOG.info("LOCAL_DEBUG: " + message);
	}
	
	/** the constructor, initializes connection, and sets up aos getter. 
	 * @throws IOException */
	public HTTPClient(String atomIP) throws IOException {
		//Initialize socket connection to robot
		this.atomIP = atomIP;
		sock = SocketChannel.open();
		WriteDebug("Connecting to server at " + atomIP);
		sock.connect(new InetSocketAddress(atomIP, 9714));
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
	public WPIColorImage getFrame() {
		//Use Brian's code to extract an image and timestamp from raw server data.
		ByteBuffer binaryImage = cgetter.getJPEG();
		if (binaryImage == null) {
			Messages.severe("Could not parse data from robot. See the log for details.");
			return null;
		}
		//Decode ByteBuffer into an IplImage
		byte[] b = new byte[binaryImage.remaining()];
		binaryImage.get(b);
		try {
			InputStream iis = new ByteArrayInputStream(b);
			BufferedImage bImageFromConvert = ImageIO.read(iis);
			WriteDebug("Image processing successful.");
			return new WPIColorImage(bImageFromConvert);
		}
		catch (IOException e) {
			LOG.warning("Image processing failed: " + e.getMessage());
			return null;
		}
	}	
	
	/** Gets the name to display at the top of the image window. */
	public String getName() {
		return atomIP;
	}
	
	/** Gets the current image's timestamp. */
	public double getTimestamp() {
		return cgetter.getTimestamp();
	}
}
