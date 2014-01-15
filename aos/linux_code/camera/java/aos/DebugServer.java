package aos;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.googlecode.javacv.cpp.opencv_core.IplImage;

/**
 * A server that serves {@link ServableImage}s.
 */
public class DebugServer {
	private static final String initialHeaderString = "HTTP/1.0 200 OK\r\n"
			+ "Connection: close\r\n"
			+ "Server: AOS/0.0 Vision Code Debug\r\n"
			+ "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n"
			+ "Pragma: no-cache\r\n"
			+ "Expires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"
			+ "Content-Type: multipart/x-mixed-replace; boundary=boundarydonotcross\r\n";
	private static final String intermediateHeaderFormat = "\r\n--boundarydonotcross\r\n"
			+ "Content-Type: image/bmp\r\n"
			+ "Content-Length: %d\r\n"
			+ "X-Timestamp: %f\r\n"
			+ "\r\n";
	private static final ByteBuffer initialHeader;
	private static final Pattern headerPattern;
	static {
		initialHeader = ByteBuffer.wrap(initialHeaderString.getBytes())
				.asReadOnlyBuffer();
		headerPattern = Pattern.compile("^GET ([^? ]+)(?:\\?i=(\\S*))? HTTP/.*\r\n.*$",
				Pattern.DOTALL);
	}

	private static final Logger LOG = Logger.getLogger(DebugServer.class
			.getName());
	private final ServerSocketChannel server;
	private final Collection<Client> clients = new ArrayList<Client>();
	private final Map<String, ServableImage> images = new HashMap<String, ServableImage>();
	private final Map<String, Palette> palettes = new HashMap<String, Palette>();
	private double timestamp;
	
	public static class Palette {
		private final ByteArrayOutputStream out = new ByteArrayOutputStream();
		private int entries = 0;

		/**
		 * Adds a new color. All 4 arguments are unsigned bytes.
		 * @param r red
		 * @param g green
		 * @param b blue
		 * @param a alpha (doesn't seem to work)
		 * @return this
		 */
		public Palette add(int r, int g, int b, int a) {
			out.write(b);
			out.write(g);
			out.write(r);
			out.write(a);
			++entries;
			return this;
		}
		
		private int entries() {
			return entries;
		}
		private int bytes() {
			return entries * 4;
		}
		private void writeTo(ByteBuffer buffer) {
			buffer.put(out.toByteArray());
		}
	}

	private class Client {
		private final int bmpHeaderSize;
		private int bmpType;
		private Palette palette;

		private final SocketChannel channel;
		private ServableImage image = null;
		private IplImage img;
		private int index;
		private final ByteBuffer[] buffers;
		private final ByteBuffer initial = initialHeader.duplicate();
		private final ByteBuffer read = ByteBuffer.allocate(1024);

		public Client(SocketChannel channel) throws IOException {
			this.channel = channel;
			channel.configureBlocking(false);

			if (bmpType == 3) {
				bmpHeaderSize = 122;
			} else if (bmpType == 0) {
				bmpHeaderSize = 54;
			} else {
				throw new AssertionError("unknown bmpType value " + bmpType);
			}
			// [0] gets filled in by createBmpHeader which writes the header into [1]
			// [2] gets set to the image buffer
			buffers = new ByteBuffer[] { null, ByteBuffer.allocate(2048), null };
		}

		public void register(Selector selector) throws ClosedChannelException {
			channel.register(
					selector,
					(image == null) ? SelectionKey.OP_READ
							: SelectionKey.OP_WRITE, this);
		}

		public void close() {
			if (image != null) {
				image.setDebugging(false);
			}
			if (channel != null) {
				try {
					channel.close();
				} catch (IOException e) {
					LOG.log(Level.WARNING,
							"encountered error when closing channel", e);
				}
			}
		}

		private void createBmpHeader(ByteBuffer buffer) {
			// <http://en.wikipedia.org/wiki/BMP_file_format#File_structure>
			// explains what these all are
			// signed/unsigned numbers don't matter because they'd better not be
			// that big
			final int paletteSize = (palette == null) ? 0 : palette.bytes();
			buffers[0] = ByteBuffer.wrap(String.format(intermediateHeaderFormat,
					bmpHeaderSize + paletteSize + image.imageSize(), timestamp).getBytes());
			buffer.order(ByteOrder.LITTLE_ENDIAN);
			buffer.put((byte) 'B').put((byte) 'M');
			buffer.putInt(bmpHeaderSize + paletteSize + image.imageSize());
			buffer.putInt(0); // skip ahead 4 bytes
			buffer.putInt(bmpHeaderSize + paletteSize); // offset to start of image data
			buffer.putInt(bmpHeaderSize - 14); // size of the rest of the header
			// BMPs expect image data bottom to top, so -height to fix that
			buffer.putInt(ImageGetter.width).putInt(-ImageGetter.height);
			buffer.putShort((short) 1).putShort(image.bitsPerPixel());
			buffer.putInt(bmpType);
			buffer.putInt(image.imageSize()); // number of bytes in the actual
												// image
			buffer.putInt(2835).putInt(2835); // physical resolution; don't
												// think it matters
			buffer.putInt((palette == null) ? 0 : palette.entries());
			buffer.putInt(0); // # of important colors (0 means all)
			if (palette != null) {
				palette.writeTo(buffer);
			}
			final int expected;
			if (bmpType == 0) { // BI_RGB
				expected = bmpHeaderSize + paletteSize;
			} else if (bmpType == 3) { // BI_BITFIELDS
				buffer.putInt(0x0000FF00).putInt(0x00FF0000).putInt(0xFF000000)
						.putInt(0); // RGBA bitmasks
				buffer.putInt(0x57696E20); // LCS_WINDOWS_COLOR_SPACE
				expected = bmpHeaderSize - 48;
			} else {
				throw new AssertionError("unknown bmpType value " + bmpType);
			}
			if (buffer.position() != expected) {
				throw new AssertionError(
						"header ended up the wrong size. expected "
								+ expected + " but got "
								+ buffer.position());
			}
			buffer.limit(bmpHeaderSize + paletteSize);
			buffer.rewind();
		}

		/**
		 * Does anything that this one can right now.
		 * 
		 * @return whether or not to {@link #close()} and remove this one
		 */
		public boolean run() throws IOException {
			if (image == null) {
				final int bytesRead = channel.read(read);
				final String readString = new String(read.array(), 0,
						read.position());
				LOG.log(Level.INFO, "read " + bytesRead
						+ " header bytes position=" + read.position()
						+ " string='" + readString + "'");

				final Matcher matcher = headerPattern.matcher(readString);
				if (matcher.matches()) {
					final String url = matcher.group(1);
					image = images.get(url);
					if (image == null) {
						LOG.log(Level.INFO, "couldn't find an image for url '"
								+ url + "'. dropping client");
						return true;
					} else {
						LOG.log(Level.INFO, "found an image for url '"
								+ url + "'");
					}
					palette = palettes.get(url);
					bmpType = 0; // could change this in the future
					createBmpHeader(buffers[1]);
					image.setDebugging(true);
					final String indexString = matcher.group(2);
					if (indexString != null) {
						index = Integer.parseInt(indexString);
					} else {
						index = 0;
					}
					LOG.log(Level.INFO, "using index " + index);
				} else if (!read.hasRemaining()) {
					read.flip();
					LOG.log(Level.WARNING,
							"ran out of buffer space reading the header. currently have '"
									+ readString + "'. dropping connection");
					return true;
				} else if (bytesRead == -1) {
					read.flip();
					LOG.log(Level.WARNING,
							"reached end of stream for headers without getting anything valid. currently have "
									+ read.limit()
									+ " bytes ='"
									+ readString
									+ "'. dropping connection");
					return true;
				}
			} else if (initial.hasRemaining()) {
				channel.write(initial);
			} else {
				if (buffers[2] == null) {
					img = image.getSnapshot(index);
					if (img == null) {
						return false;
					} else {
						buffers[2] = img.getByteBuffer();
						LOG.log(Level.FINE, "got " + buffers[2]
								+ " from the image");
					}
				}
				channel.write(buffers);
				boolean remaining = false;
				for (ByteBuffer c : buffers) {
					if (c.hasRemaining()) {
						remaining = true;
					}
				}
				if (!remaining) {
					for (ByteBuffer c : buffers) {
						c.rewind();
					}
					buffers[2] = null;
					image.releaseSnapshot(index, img);
				}
			}
			return false;
		}
	}

	public DebugServer(int port) throws IOException {
		server = ServerSocketChannel.open();
		server.configureBlocking(false);
		server.socket().bind(new InetSocketAddress(port), 10);
		new Thread("DebugServer") {
			@Override
			public void run() {
				try {
					loop();
				} catch (Throwable e) {
					LOG.log(Level.SEVERE,
							"trouble running the server loop", e);
					System.exit(1);
				}
			}
		}.start();
	}

	public void addImage(String name, ServableImage image) {
		if (image.bitsPerPixel() != 24) {
			throw new IllegalArgumentException("only 24-bit images are supported");
			// could support 16 and 32 bpp images by using bmpType of 3
		}
		images.put(name, image);
	}
	public void addImage(String name, ServableImage image, Palette palette) {
		if (image.bitsPerPixel() != 8) {
			throw new IllegalArgumentException("only 8-bit images are supported");
			// everything should work for 1, 2, and 4 bpp ones too except for padding etc
		}
		if (palette.entries() > (1 << image.bitsPerPixel())) {
			throw new IllegalArgumentException("too many colors in the palette");
		}
		images.put(name, image);
		palettes.put(name, palette);
	}
	/**
	 * This timestamp is written out in the debugging images.
	 * @param timestamp the current timestamp
	 */
	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

	private void loop() throws IOException {
		final Selector selector = Selector.open();
		server.register(selector, SelectionKey.OP_ACCEPT);
		while (true) {
			try {
				for (Client c : clients) {
					c.register(selector);
				}
				selector.select();
				for (final Iterator<SelectionKey> i = selector.selectedKeys()
						.iterator(); i.hasNext();) {
					final SelectionKey c = i.next();
					if (c.isAcceptable()) {
						// there's only 1 socket there that can accept
						final SocketChannel channel = server.accept();
						if (channel != null) {
							clients.add(new Client(channel));
						}
					} else {
						final Client client = (Client) c.attachment();
						try {
							if (client.run()) {
								client.close();
								clients.remove(client);
							}
						} catch (Exception e) {
							LOG.log(Level.INFO, "dropping client " + client
									+ " because it's run() threw an exception",
									e);
							client.close();
							clients.remove(client);
						}
					}
					i.remove();
				}
			} catch (IOException e) {
				LOG.log(Level.WARNING, "trouble running the server loop", e);
			}
		}
	}
}
