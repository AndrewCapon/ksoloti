#pragma once

#if ADU_TRANSFER_LOG_SIZE
typedef enum _BLType {blStartTransmit, blStartReceive, blEndTransmit, blEndReceive} BLType;
 
typedef struct _DBGLOG
{
  BLType    type;
  uint16_t  uSize;
} DBGLOG;

DBGLOG aduTransferLog[ADU_TRANSFER_LOG_SIZE] __attribute__ ((section (".sram3")));
uint16_t aduLogCount = 0;

void aduAddTransferLog(BLType type, uint16_t uSize)
{
  if(aduLogCount == 0)
    memset(aduTransferLog, 0, sizeof(aduTransferLog));

  aduTransferLog[aduLogCount].type = type;
  aduTransferLog[aduLogCount].uSize = uSize;
  aduLogCount++;
  if(aduLogCount == ADU_TRANSFER_LOG_SIZE)
    aduLogCount = 0;
}
#else
  #define aduAddTransferLog(a, b)
#endif 


#if ADU_OVERRUN_LOG_SIZE
typedef enum _LogType
{
  ltCodecCopyEnd__,
  ltFrameEndedEnd__,
  ltFrameStartedEnd,
  ltWaitingForSync_,
  ltAfterTXAdjust__
} LogType; 

typedef struct _OverrunDebug
{
  LogType  type;
  uint16_t txRingBufferUsedSize;
  uint16_t txCurrentRingBufferSize;
  uint16_t txRingBufferWriteOffset;
  uint16_t txRingBufferReadOffset;
  int16_t  sampleOffset;
  uint16_t codecFrameSampleCount;
  int16_t  codecMetricsSampleOffset;
  ADUState state;

//  uint16_t txCurrentRingBufferSize;
} __attribute__((packed)) OverrunDebug;

OverrunDebug overrunDebug[ADU_OVERRUN_LOG_SIZE]   __attribute__ ((section (".sram3")));
uint16_t uLogIndex = 0;

void AddOverunLog(LogType type)
{
  static uint16_t uMaxRingBufferSize = 0;

  overrunDebug[uLogIndex].type = type;
  overrunDebug[uLogIndex].txRingBufferUsedSize = aduState.txRingBufferUsedSize;
  overrunDebug[uLogIndex].txRingBufferWriteOffset = aduState.txRingBufferWriteOffset;
  overrunDebug[uLogIndex].txRingBufferReadOffset = aduState.txRingBufferReadOffset;
  overrunDebug[uLogIndex].sampleOffset = aduState.sampleOffset;
  overrunDebug[uLogIndex].codecFrameSampleCount = aduState.codecFrameSampleCount;
  overrunDebug[uLogIndex].codecMetricsSampleOffset = aduState.codecMetricsSampleOffset;
  overrunDebug[uLogIndex].txCurrentRingBufferSize = aduState.txCurrentRingBufferSize;
  overrunDebug[uLogIndex].state = aduState.state;
  
  if(aduState.txCurrentRingBufferSize > uMaxRingBufferSize)
    uMaxRingBufferSize = aduState.txCurrentRingBufferSize;

  uLogIndex++;
  if(uLogIndex == ADU_OVERRUN_LOG_SIZE)
    uLogIndex = 0;
}
#else
  #define AddOverunLog(a)
#endif 

