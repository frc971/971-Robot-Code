package aos;

/**
 * Represents an error from native code.
 */
public class NativeError extends Error {
	private static final long serialVersionUID = 7394872852984037261L;

	public NativeError() {
		super();
	}

	public NativeError(String message, Throwable cause) {
		super(message, cause);
	}

	public NativeError(String message) {
		super(message);
	}

	public NativeError(Throwable cause) {
		super(cause);
	}

}
