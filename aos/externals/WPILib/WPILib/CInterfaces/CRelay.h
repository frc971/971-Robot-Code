#ifndef _C_RELAY_H
#define _C_RELAY_H

typedef enum {kOff, kOn, kForward, kReverse} RelayValue;
typedef enum {kBothDirections, kForwardOnly, kReverseOnly} RelayDirection;

void InitRelay(UINT32 channel, RelayDirection direction = kBothDirections);
void InitRelayRelay(UINT8 moduleNumber, UINT32 channel, RelayDirection direction = kBothDirections);

void DeleteRelay(UINT32 channel);
void DeleteRelay(UINT8 moduleNumber, UINT32 channel);

void SetRelay(UINT32 channel, RelayValue value);
void SetRelay(UINT8 moduleNumber, UINT32 channel, RelayValue value);

typedef void *RelayObject;

RelayObject CreateRelay(UINT8 moduleNumber, UINT32 channel, RelayDirection direction = kBothDirections);
RelayObject CreateRelay(UINT32 channel, RelayDirection direction = kBothDirections);
void SetRelay(RelayObject o, RelayValue value);
void DeleteRelay(RelayObject o);

#endif

