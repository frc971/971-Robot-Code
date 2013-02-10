package aos;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.ReadableByteChannel;
import java.nio.channels.SocketChannel;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.googlecode.javacv.cpp.opencv_core;

/**
 * Makes implementing code that processes frames from a camera easy.
 */
public abstract class CameraProcessor {
	private static final Logger LOG = Logger.getLogger(CameraProcessor.class.getName());
	protected final ImageGetter getter;
	protected final ServableImage start = new ServableImage(ImageGetter.width, ImageGetter.height, opencv_core.IPL_DEPTH_8U, 3);
	
	/**
	 * Parses any arguments it recognizes out of {@code args} and initializes stuff appropriately.
	 * This includes using {@link QueueLogHandler} for all exceptions and {@link Thread#setDefaultUncaughtExceptionHandler}ing.
	 * @param args from {@code main}
	 */
	protected CameraProcessor(String[] args) throws UnknownHostException, IOException {
		QueueLogHandler.UseForAll();
		ReadableByteChannel channel = null;
		for (int i = 0; i < args.length; ++i) {
			final String c = args[i];
			if (c.equals("--host")) {
				String host = args[++i];
				final SocketChannel socketChannel = SocketChannel.open(new InetSocketAddress(Inet4Address.getByName(host), 9714));
				socketChannel.write(ByteBuffer.wrap(new byte[] {'\r', '\n', '\r', '\n'})); // get it past the read headers stage
				channel = socketChannel;
			} else {
				System.err.println("CameraProcessor: warning: unrecognized argument '" + c + "'. ignoring");
			}
		}
		
		if (channel != null) {
			getter = new ChannelImageGetter(channel);
		} else {
			System.out.println("creating QueueImageGetter");
			getter = new QueueImageGetter();
			System.out.println("done");
		}

		LOG.log(Level.INFO, "CameraProcessor is up");
		System.err.println("CameraProcessor is up (on stderr)");
	}
	
	protected abstract void RunIteration();
	
	protected void Run() {
		while (true) {
			if (!getter.get(start.getImage())) {
				LOG.log(Level.WARNING, "getting image failed");
				continue;
			}
			RunIteration();
			start.releaseImage();
		}
	}
}
