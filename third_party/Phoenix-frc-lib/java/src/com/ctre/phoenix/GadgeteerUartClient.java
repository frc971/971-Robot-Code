package com.ctre.phoenix;

public interface GadgeteerUartClient
{
	public enum GadgeteerProxyType{
		General(0), 
		Pigeon(1), 
		PC_HERO(2),
		Unknown(-1);
		private int value; private GadgeteerProxyType(int value) { this.value = value; } 
		public static GadgeteerProxyType valueOf(int value) {
			for (GadgeteerProxyType e : GadgeteerProxyType.values()) {
				if (e.value == value) {
					return e;
				}
			}
			return Unknown;
		}
	};

	public enum GadgeteerConnection	{
		NotConnected (0),
		Connecting (1),
		Connected (2),
		Unknown(-1);
		private int value; private GadgeteerConnection(int value) { this.value = value; } 
		public static GadgeteerConnection valueOf(int value) {
			for (GadgeteerConnection e : GadgeteerConnection.values()) {
				if (e.value == value) {
					return e;
				}
			}
			return Unknown;
		}
	};

	public static class GadgeteerUartStatus {
		public GadgeteerProxyType type;
		public GadgeteerConnection conn;
		public int bitrate;
		public int resetCount;
	};


	enum GadgeteerState
	{
		GadState_WaitChirp1(0),
		GadState_WaitBLInfo(1),
		GadState_WaitBitrateResp(2),
		GadState_WaitSwitchDelay(3),
		GadState_WaitChirp2(4),
		GadState_Connected_Idle(5),
		GadState_Connected_ReqChirp(6),
		GadState_Connected_RespChirp(7),
		GadState_Connected_ReqCanBus(8),
		GadState_Connected_RespCanBus(9),
		GadState_Connected_RespIsoThenChirp(10),
		GadState_Connected_RespIsoThenCanBus(11);
		private int value; private GadgeteerState(int value) { this.value = value; } 
		public static GadgeteerState valueOf(int value) {
			for (GadgeteerState e : GadgeteerState.values()) {
				if (e.value == value) {
					return e;
				}
			}
			return GadState_WaitChirp1;
		}
	};
    int getGadgeteerStatus(GadgeteerUartStatus status);
};

