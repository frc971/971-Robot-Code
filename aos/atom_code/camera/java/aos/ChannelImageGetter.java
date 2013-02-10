package aos;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.ReadableByteChannel;
import java.nio.channels.SelectableChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Retrieves images from a {@link InputChannel}. Expects the images in mjpg form.
 * For now, only accepts streams formatted pretty closely to how aos::camera::HTTPStreamer does it.
 */
public class ChannelImageGetter extends JPEGImageGetter {
	/**
	 * What to multiply each length by when it needs to allocate a larger buffer to fit an image.
	 */
	private static final double extraLength = 1.2;
	
	private static final Logger LOG = Logger.getLogger(ChannelImageGetter.class
			.getName());
	private final ReadableByteChannel channel;
	private final Selector selector = Selector.open();
	private String separator = "--boundarydonotcross\r\n";
	private ByteBuffer current;
	private final ByteBuffer headerBuffer = ByteBuffer.allocateDirect(30);
	private final Map<String, String> headers = new TreeMap<String, String>(String.CASE_INSENSITIVE_ORDER);
	
	public ChannelImageGetter(ReadableByteChannel channel) throws IOException {
		this.channel = channel;
		if (channel instanceof SelectableChannel) {
			((SelectableChannel)channel).configureBlocking(false);
		}
	}

	@Override
	public ByteBuffer getJPEG() {
		try {
			if (!parseHeaders()) {
				return null;
			}
			LOG.log(Level.FINE, "parsed headers " + headers.toString());
			try {
				final int length = Integer.parseInt(headers.get("Content-Length"));
				if (current == null || current.capacity() < length) {
					LOG.log(Level.INFO, "allocating a new direct buffer of length " + length * extraLength);
					current = ByteBuffer.allocateDirect((int) (length * extraLength));
				} else {
					current.rewind();
					current.limit(length);
				}
			} catch (NumberFormatException e) {
				LOG.log(Level.WARNING, "couldn't parse '" + headers.get("Content-Length") + "' as a number");
				return null;
			}
			current.put(headerBuffer); // copy out any of the image that got buffered with the headers
			while (current.hasRemaining()) {
				channel.read(current);
			}
			current.flip();
		} catch (IOException e) {
			LOG.log(Level.WARNING, "reading the headers and/or image failed", e);
			return null;
		}
		return current;
	}
	// returns success
	private boolean parseHeaders() throws IOException {
		// Reads chunks into headerBuffer and parses out headers.
		// Looks for separator first.
		
		headerBuffer.clear();
		headers.clear();
		final byte[] separatorBytes = separator.getBytes();
		int separatorIndex = 0; // how much of the separator has been matched
		while (headerBuffer.hasRemaining() || headerBuffer.limit() < headerBuffer.capacity()) {
			if (channel instanceof SelectableChannel) {
				((SelectableChannel)channel).register(selector, SelectionKey.OP_READ);
				selector.select();
			}
			headerBuffer.limit(headerBuffer.capacity());
			channel.read(headerBuffer);
			headerBuffer.flip();
			if (separatorIndex < separatorBytes.length) {
				// make sure we don't get part of the way through
				while (headerBuffer.remaining() >= (separatorBytes.length - separatorIndex)) {
					final byte c = headerBuffer.get();
					if (separatorBytes[separatorIndex++] != c) {
						separatorIndex = 0;
					}
					if (separatorIndex == separatorBytes.length) {
						break;
					}
				}
				headerBuffer.compact();
			} else {
				int keyEnd = 0, valueStart = 0;
				boolean foundEndR = false; // found the end \r
				while (headerBuffer.hasRemaining()) {
					final byte c = headerBuffer.get();
					if (foundEndR) {
						if (c != '\n') {
							LOG.log(Level.WARNING, "found \r\n\r but no \n afterwards");
						} else {
							return true;
						}
					} else if (keyEnd == 0) {
						if (c == ':') {
							keyEnd = headerBuffer.position() - 1;
						} else if (c == '\r') {
							foundEndR = true;
						}
					} else if (valueStart == 0) {
						if (c != ' ') {
							valueStart = headerBuffer.position() - 1;
						}
					} else {
						if (c == '\r') {
							final int valueEnd = headerBuffer.position();
							final byte[] key = new byte[keyEnd];
							headerBuffer.position(0);
							headerBuffer.get(key);
							final byte[] value = new byte[valueEnd - valueStart - 1];
							headerBuffer.position(valueStart);
							headerBuffer.get(value);
							headers.put(new String(key), new String(value));
							
							headerBuffer.get(); // get the \r
							headerBuffer.get(); // get the \n
							
							headerBuffer.compact();
							headerBuffer.flip();
							
							keyEnd = valueStart = 0;
						}
					}
				}
			}
		}
		// if we got here, then it doesn't have space left and we haven't finished
		LOG.log(Level.WARNING, "got a header that was too long. headerBuffer should be made bigger");
		return false;
	}

	@Override
	public double getTimestamp() {
		if (headers.containsKey("X-Timestamp")) {
			return Double.parseDouble(headers.get("X-Timestamp"));
		} else {
			throw new UnsupportedOperationException("source stream doesn't have X-Timestamp headers");
		}
	}

}
