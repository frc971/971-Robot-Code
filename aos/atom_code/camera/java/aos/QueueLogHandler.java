package aos;

import java.lang.Thread.UncaughtExceptionHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

/**
 * <p>Sends messages to the AOS queue-based logging system. Also sends anything that's at least a {@link Level#WARNING} to {@link System#err}.</p>
 * <p>Writes out each stack frame of exceptions as a separate line after the first one with a \t at the beginning.</p>
 */
public class QueueLogHandler extends Handler {
	private Formatter defaultFormatter;
	
	/**
	 * Sets up the logging framework to use an instance of this class for all logging and returns the newly created instance.
	 */
	public static QueueLogHandler UseForAll() {
		Natives.ensureLoaded();
		final Logger top = Logger.getLogger("");
		for (Handler c : top.getHandlers()) {
			top.removeHandler(c);
		}
		QueueLogHandler instance = new QueueLogHandler();
		top.addHandler(instance);
		top.setLevel(Level.ALL);
		
		Thread.setDefaultUncaughtExceptionHandler(new UncaughtExceptionHandler() {
			@Override
			public void uncaughtException(Thread thread, Throwable e) {
				top.log(Level.SEVERE, "uncaught exception in thread " + thread, e);
				System.exit(1);
			}
		});
		
		return instance;
	}

	@Override
	public void close() throws SecurityException {
	}
	@Override
	public void flush() {
	}

	private Formatter findFormatter() {
		final Formatter r = getFormatter();
		if (r != null) {
			return r;
		}
		if (defaultFormatter != null) {
			return defaultFormatter;
		}
		return defaultFormatter = new SimpleFormatter();
	}
	@Override
	public void publish(LogRecord record) {
		/*final StringBuilder thrownString = new StringBuilder(0);
		if (record.getThrown() != null) {
			thrownString.append(": ");
			thrownString.append(record.getThrown().toString());
			for (StackTraceElement c : record.getThrown().getStackTrace()) {
				thrownString.append(" > ");
				thrownString.append(c.getClassName());
				thrownString.append('.');
				thrownString.append(c.getMethodName());
				thrownString.append('(');
				thrownString.append(c.getFileName());
				thrownString.append(':');
				thrownString.append(c.getLineNumber());
				thrownString.append(')');
			}
		}
		Natives.LOG(record.getSourceClassName() + ": " + record.getSourceMethodName() + ": " +
				findFormatter().formatMessage(record) + thrownString, record.getLevel().intValue());*/
		if (record.getThrown() instanceof UnsatisfiedLinkError || record.getThrown().getCause() instanceof UnsatisfiedLinkError) {
			record.setThrown(new UnsatisfiedLinkError("are you running a JVM of the correct bitness?").initCause(record.getThrown()));
		}
		Natives.LOG(record.getSourceClassName() + ": " + record.getSourceMethodName() + ": " +
				findFormatter().formatMessage(record), record.getLevel().intValue());
		if (record.getThrown() != null) {
			logException(record.getThrown(), record.getLevel().intValue(), false);
		}
		
		if (record.getLevel().intValue() >= Level.WARNING.intValue()) {
			System.err.println(findFormatter().format(record));
		}
	}
	private void logException(Throwable e, int level, boolean caused_by) {
		final StringBuilder thrownString = new StringBuilder();
		if (caused_by) {
			thrownString.append("Caused by: ");
		}
		thrownString.append(e.getClass().getName());
		thrownString.append(": ");
		thrownString.append(e.getLocalizedMessage());
		Natives.LOG(thrownString.toString(), level);
		for (StackTraceElement c : e.getStackTrace()) {
			thrownString.setLength(0);
			thrownString.append("\t");
			thrownString.append(c.getClassName());
			thrownString.append('.');
			thrownString.append(c.getMethodName());
			thrownString.append('(');
			thrownString.append(c.getFileName());
			thrownString.append(':');
			thrownString.append(c.getLineNumber());
			thrownString.append(')');
			Natives.LOG(thrownString.toString(), level);
		}
		if (e.getCause() != null) {
			logException(e.getCause(), level, true);
		}
	}

}
