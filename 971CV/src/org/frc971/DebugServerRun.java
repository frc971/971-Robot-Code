package org.frc971;

import java.io.IOException;
import java.nio.channels.SocketChannel;
import java.net.InetSocketAddress;

import aos.DebugServer;
import aos.ChannelImageGetter;

public class DebugServerRun {
	public static void main(final String args[]) {
		try {
			DebugServer server = new DebugServer(9714);
			System.out.println("Debug Server running on port 9714.");
			SocketChannel client_sock = SocketChannel.open();
			client_sock.connect(new InetSocketAddress("192.168.0.137", 9714));
			ChannelImageGetter client = new ChannelImageGetter(client_sock);
			client.getJPEG();
			System.out.println(client.getTimestamp());
		}
		catch (IOException e) {
			System.out.println(e.getMessage());
		}
	}
}
