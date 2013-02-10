package aos;

public class NativeBufferError extends NativeError {
	private static final long serialVersionUID = -5298480149664213316L;

	public NativeBufferError() {
		super();
	}

	public NativeBufferError(String message, Throwable cause) {
		super(message, cause);
	}

	public NativeBufferError(String message) {
		super(message);
	}

	public NativeBufferError(Throwable cause) {
		super(cause);
	}

}
