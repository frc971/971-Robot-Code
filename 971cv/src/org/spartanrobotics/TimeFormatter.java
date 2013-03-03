/**
 * 
 */
package org.spartanrobotics;

import java.util.logging.Formatter;
import java.util.logging.LogRecord;
import java.util.Date;

/**
 * @author daniel
 * 
 */

/** Formats log messages with adequate timestamp. */
public class TimeFormatter extends Formatter{
	
	/** Constructor, see Formatter. */
	public TimeFormatter() {
		super();
	}
	
	/** Format a message in the propper way.
	 * @return Includes time, name of logger, level and message.
	 */
	public String format(LogRecord message) {
		//we need to include the date and time in our message
		StringBuffer out = new StringBuffer();
		out.append("@");
		Date date = new Date(message.getMillis());
		out.append(date.toString());
		out.append(" in [");
		//add our logger's name
		out.append(message.getLoggerName());
		out.append("]: (");
		//add message level
		out.append(message.getLevel().getName());
		out.append(") ");
		//add actual message
		out.append(formatMessage(message));
		out.append("\n");
		return out.toString();
	}

}
