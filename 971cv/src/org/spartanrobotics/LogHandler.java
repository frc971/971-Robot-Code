/**
 * 
 */
package org.spartanrobotics;

import java.io.FileOutputStream;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

import java.util.logging.Handler;
import java.util.logging.LogRecord;

/**
 * @author daniel
 * 
 */

/** Logs data to custom files, using specific formatting. */
public class LogHandler extends Handler {
	
	private FileOutputStream ofstream;
	PrintWriter writer;
	
	/** Constructor for log handler. 
	 * 
	 * @param filename is the name of the file you want to log to.
	 * @throws FileNotFoundException if file cannot be opened or created.
	 */
	public LogHandler (String filename) throws FileNotFoundException {
		super();
		
		if (filename == null || filename == "") {
			filename = "logfile.log";
		}
		
		//check if file exists, and if not, create it
		ofstream = new FileOutputStream(filename);
		writer = new PrintWriter(ofstream);
		setFormatter(new TimeFormatter());
	}
	
	/*Required methods*/
	
	/** Is required by API. Writes a new message to the log.
	 * @param message is the message you want to log.
	 */
	public void publish(LogRecord message) {
		//record a message
		if (!isLoggable(message)) {
			//ensure that this message should be logged by this handler
			return;
		}
		writer.print(getFormatter().format(message)); //Formatter adds trailing \n
	}
	
	/** Is required by API. Flushes the writer. */
	public void flush() {
		writer.flush();
	}
	
	/** Is required by API. Closes logfile. */
	public void close() throws SecurityException {
		writer.close();
	}
}
