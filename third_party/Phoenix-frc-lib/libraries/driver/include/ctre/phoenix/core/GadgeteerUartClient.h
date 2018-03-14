#ifndef GadgeteerUartClient_h_
#define GadgeteerUartClient_h_
#include <stdint.h>
#include <string>

class IGadgeteerUartClient
{
public:
	IGadgeteerUartClient() {}
    virtual ~IGadgeteerUartClient() {}

	enum GadgeteerProxyType
	{
		General = 0,
		Pigeon = 1,
		PC_HERO = 2,
	};
	enum GadgeteerConnection
	{
		NotConnected = 0,
		Connecting = 1,
		Connected = 2,
	};
	struct GadgeteerUartStatus {
		GadgeteerProxyType type;
		GadgeteerConnection conn;
		uint32_t bitrate;
		uint32_t resetCount;
	};

    virtual int GetGadgeteerStatus(GadgeteerUartStatus & status) = 0;

    static std::string ToString(IGadgeteerUartClient::GadgeteerProxyType gpt) {
    	std::string retval;
		switch(gpt) {
			case General: retval = "General"; break;
			case Pigeon : retval = "Pigeon"; break;
			case PC_HERO: retval = "PC_HERO"; break;
		}
		return retval;
    }
    static std::string ToString(IGadgeteerUartClient::GadgeteerConnection gc) {
    	std::string retval;
		switch(gc) {
			case NotConnected: retval = "NotConnected"; break;
			case Connecting : retval = "Connecting"; break;
			case Connected: retval = "Connected"; break;
		}
		return retval;
    }

protected:
	enum GadgeteerState
	{
		GadState_WaitChirp1 = 0,
		GadState_WaitBLInfo = 1,
		GadState_WaitBitrateResp = 2,
		GadState_WaitSwitchDelay = 3,
		GadState_WaitChirp2 = 4,
		GadState_Connected_Idle = 5,
		GadState_Connected_ReqChirp = 6,
		GadState_Connected_RespChirp = 7,
		GadState_Connected_ReqCanBus = 8,
		GadState_Connected_RespCanBus = 9,
		GadState_Connected_RespIsoThenChirp = 10,
		GadState_Connected_RespIsoThenCanBus = 11,
	};
};
#endif
