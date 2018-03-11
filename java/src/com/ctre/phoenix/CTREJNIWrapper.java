package com.ctre.phoenix;

public class CTREJNIWrapper {
  static boolean libraryLoaded = false;
  
  static {
    if (!libraryLoaded) {
      try {
        System.loadLibrary("CTRE_PhoenixCCI");
      } catch (UnsatisfiedLinkError e) {
        e.printStackTrace();
        System.exit(1);
      }
      libraryLoaded = true;
    }
  }

  public static native int getPortWithModule(byte module, byte channel);

  public static native int getPort(byte channel);
}
