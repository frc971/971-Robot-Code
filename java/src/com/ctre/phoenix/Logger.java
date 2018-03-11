package com.ctre.phoenix;

public class Logger
{
	/**
	 * Logs an entry into the Phoenix DS Error/Logger stream
	 * @param code Error code to log.  If OKAY is passed, no action is taken.
	 * @param origin Origin string to send to DS/Log
	 * @return OKAY error code.
	 */
	public static ErrorCode log(ErrorCode code, String origin) {
		/* only take action if the error code is nonzero */
		if (code != ErrorCode.OK) {
			String stack = java.util.Arrays.toString(Thread.currentThread().getStackTrace());
			stack = stack.replaceAll(",", "\n");
			int errCode = code.value;
			return ErrorCode.valueOf(CTRLoggerJNI.JNI_Logger_Log(errCode, origin, stack));
		}
		/* otherwise return OK */
		return ErrorCode.OK;
	}

	//public static void close() {
	//	//CTRLoggerJNI.JNI_Logger_Close();
	//}
	//public static void open() {
	//	//CTRLoggerJNI.JNI_Logger_Open(2);
	//}
	/*
	public static String getVerbose(int code) {
		return CTRLoggerJNI.JNI_Logger_GetLong(code);
	}
	public static String getShort(int code) {
		return CTRLoggerJNI.JNI_Logger_GetShort(code);
	}
	*/
}

