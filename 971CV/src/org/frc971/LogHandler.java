/**
 * 
 */
package org.frc971;

import java.io.FileOutputStream;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

import java.util.logging.Handler;
import java.util.logging.LogRecord;

/**
 * @author daniel
 * logs data to custom files, using specific formatting.
 */
public class LogHandler extends Handler {
	
	private FileOutputStream ofstream;
	PrintWriter writer;
	
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
	
	public void publish(LogRecord message) {
		//record a message
		if (!isLoggable(message)) {
			//ensure that this message should be logged by this handler
			return;
		}
		writer.print(getFormatter().format(message)); //Formatter adds trailing \n
	}
	public void flush() {
		writer.flush();
	}
	public void close() throws SecurityException {
		writer.close();
	}
}
