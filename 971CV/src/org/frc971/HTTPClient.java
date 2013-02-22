package org.frc971;

//@author: daniel

import static com.googlecode.javacv.cpp.opencv_highgui.cvEncodeImage;

import java.io.*;
import java.net.*;

import java.awt.image.BufferedImage;

import java.nio.channels.FileChannel;
import java.nio.channels.SocketChannel;
import java.nio.ByteBuffer;

import java.util.Iterator;
import java.util.logging.Logger;

import javax.imageio.ImageIO;
import javax.imageio.ImageReadParam;
import javax.imageio.ImageReader;
import javax.imageio.stream.ImageInputStream;

import com.googlecode.javacv.cpp.opencv_core.CvArr;
import com.googlecode.javacv.cpp.opencv_core.CvMat;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

import aos.ChannelImageGetter;

import edu.wpi.first.wpijavacv.WPIColorImage;

public class HTTPClient {
	//Connects to HTTP Server on robot and receives images
	
	private final static boolean LOCAL_DEBUG = true;
	
	private SocketChannel sock;
	private Socket core_sock;
	
	
	private BufferedReader sock_in;
	private PrintWriter sock_out;
	
	private final String ATOM_IP = "192.168.0.137";
	
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
		ByteBuffer recvd = ByteBuffer.allocate(1024);
		StringBuilder sb = new StringBuilder();
		String message = "";
		try {
			int ret;
			while ((ret = sock.read(recvd)) != -1) {
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
	
	public HTTPClient() {
		//Initialize socket connection to robot
		try {
			sock = SocketChannel.open();
			core_sock = sock.socket();
			WriteDebug("Connecting to server at " + ATOM_IP);
			sock.connect(new InetSocketAddress(ATOM_IP, 9714));
			sock.configureBlocking(false);
			//sock_in = new BufferedReader(new InputStreamReader(core_sock.getInputStream()));
			//sock_out = new PrintWriter(core_sock.getOutputStream(), true);
			//Write headers
			//HTTPStreamer does not actually use the headers, so we can just write terminating chars.
			WriteDebug("Writing headers...");
			String ending = "\r\n\r\n";
			ByteBuffer header = ByteBuffer.wrap(ending.getBytes());
			while (header.remaining() > 0) {
				sock.write(header);
			}
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
			LOG.severe("Socket IO failed: " + e.getMessage());
			System.exit(2);
		}
		
	}
	public ImageWithTimestamp GetFrame() {
		//Use Brian's code to extract an image and timestamp from raw server data.
		ImageWithTimestamp final_image = new ImageWithTimestamp();
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
			return null;
		}
	}	
}
