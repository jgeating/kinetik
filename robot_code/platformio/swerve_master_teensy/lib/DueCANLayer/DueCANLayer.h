#ifndef DUE_CAN_LAYER_H
#define DUE_CAN_LAYER_H

#define CAN_BPS_1000K   1000000
#define CAN_OK          1

typedef unsigned char byte;

static byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte *cData, byte cDataLen) {
    return 0;
}

static byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen) {
    return 0;
}

static byte canInit(byte cPort, long lBaudRate) {
    return 0;
}

#endif