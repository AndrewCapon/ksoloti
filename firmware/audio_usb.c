
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "audio_usb.h"
#include "usb_lld.h"
#include "chevents.h"
#include "chdebug.h"
#include "usbcfg.h"
#include "analyse.h"

// do not set higher than -O1
#pragma GCC optimize ("O0")

// O0 codecCopy = 14us,  TX = 42.1us
// O1 codecCopy = 3.3us, TX = 11.5us

// Simple improvements
// O0 codecCopy = 11us,  TX = 31.0us
// O1 codecCopy = 2.9us, TX = 10.1us

// TX direct
// O0 codecCopy = 11.98us, TX = 2.6us
// O1 codecCopy = 3.14us,  TX = 2.0us


#define ADU_LOGGING 0
#define TX_DIRECT 1

// this will be ping pong buffer
// static uint16_t aduTxBuffer[AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));
// static uint16_t aduRxBuffer[2][AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));

// samples to try to keep in buffer
#define TX_RING_BUFFER_UNDERFLOW_SIZE (96)

// normal ring buffer sample size
#define TX_RING_BUFFER_NORMAL_SIZE    (96*2)

// Overflow sample size to keep USB transfers fully confined in ring buffer
// need to check this, should probably only be 2
#define TX_RING_BUFFER_OVERFLOW_SIZE  (96)

// Total allocated size in samples
#define TX_RING_BUFFER_FULL_SIZE (TX_RING_BUFFER_UNDERFLOW_SIZE + TX_RING_BUFFER_NORMAL_SIZE + TX_RING_BUFFER_OVERFLOW_SIZE)


static int16_t aduTxRingBuffer[TX_RING_BUFFER_FULL_SIZE] __attribute__ ((section (".sram3")));
#if !TX_DIRECT
static int16_t aduTxBuffer[96] __attribute__ ((section (".sram3")));
#endif
static int16_t aduRxBuffer[96] __attribute__ ((section (".sram3")));

// debugging defines
#define CODEC_METICS_MS (100)
#define EMULATE_UNDERRUN_SKIP_SAMPLE_EVERY_CODEC_FRAME (3000/(2*32))
#define ADU_TRANSFER_LOG_SIZE 0
#define CHECK_USB_DATA 1
#define ADU_OVERRUN_LOG_SIZE 4000


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


#define USE_TRANSFER_SAMPLE_SIZE 2
#define USE_TRANSFER_CHANNEL_SIZE 2
#define USE_TRANSFER_SAMPLES_MS 48

#define USE_TRANSFER_SIZE_SAMPLES (USE_TRANSFER_CHANNEL_SIZE * USE_TRANSFER_SAMPLES_MS)
#define USE_TRANSFER_SIZE_BYTES   (USE_TRANSFER_SAMPLE_SIZE * USE_TRANSFER_CHANNEL_SIZE * USE_TRANSFER_SAMPLES_MS)

extern AudioUSBDriver ADU1;

// const uint32_t aduSampleRates[] = {44100, 48000, 96000};
// #define N_SAMPLE_RATES  3
const uint32_t aduSampleRates[] = {48000};
#define N_SAMPLE_RATES  1

static uint8_t aduControlData[8];
static uint8_t aduControlChannel;





void aduEnableInput(USBDriver *usbp, bool bEnable);
void aduEnableOutput(USBDriver *usbp, bool bEnable);
void aduDataTransmitted(USBDriver *usbp, usbep_t ep); 
void aduDataReceived(USBDriver *usbp, usbep_t ep);
void aduInitiateReceiveI(USBDriver *usbp, size_t uCount);
void aduInitiateTransmitI(USBDriver *usbp, size_t uCount);
void aduResetInputBuffers(void);
void aduResetOutputBuffers(void);


/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

AduState aduState;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// Set the sample rate
static void aduSetSampleRate(USBDriver *usbp) 
{
  audio_control_cur_4_t const *pData = (audio_control_cur_4_t const *)&aduControlData[0];
  uint32_t uSampleRate = (uint32_t) pData->bCur;

  if(uSampleRate == 44100 || uSampleRate == 48000 || uSampleRate == 96000)
    aduState.currentSampleRate =  uSampleRate;

  // notify
  chSysLockFromIsr();
  chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_FORMAT);
  chSysUnlockFromIsr();

}

// Set mute
static void aduSetMute(USBDriver *usbp) 
{
  aduState.mute[aduControlChannel] = ((audio_control_cur_1_t const *)aduControlData)->bCur;
}

// Set mute
static void aduSetVolume(USBDriver *usbp) 
{
  aduState.volume[aduControlChannel] = ((audio_control_cur_2_t const *)aduControlData)->bCur;
}

bool aduHandleVolumeRequest(USBDriver *usbp, audio_control_request_t *request)
{
  // if(uLogCount < LOG_AMOUNT)
  //   memcpy(&requests[uLogCount++], request, sizeof(audio_control_request_t));
  // else
  //   uLogCount = 0;
  bool bResult = false;


  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    // Get and Set mute
    if(request->bmRequestType_bit.direction) // Get requests
    {
      audio_control_cur_1_t mute1 = { .bCur = aduState.mute[request->bChannelNumber] };
      usbSetupTransfer(usbp, (uint8_t *)&mute1, sizeof(mute1), NULL);
      bResult = true;
    }
    else // Set Requests
    {
      aduControlChannel = request->bChannelNumber;
      usbSetupTransfer(usbp, aduControlData, request->wLength, aduSetMute);
      bResult = true;

      // notify
      chSysLockFromIsr();
      chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_MUTE);
      chSysUnlockFromIsr();
    }
  }
  else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    // Get and Set volume
    if(request->bmRequestType_bit.direction) // Get requests
    {
      switch(request->bRequest)
      {
        case AUDIO_CS_REQ_RANGE:
        {
          audio_control_range_2_n_t(1) rangeVol = {
            .wNumSubRanges = 1,
            .subrange[0] = { .bMin = -VOLUME_CTRL_50_DB, VOLUME_CTRL_0_DB, 256 }
          };
          usbSetupTransfer(usbp, (uint8_t *)&rangeVol, sizeof(rangeVol), NULL);
          bResult = true;
          break;
        }

        case AUDIO_CS_REQ_CUR:
        {
          audio_control_cur_2_t curVol = { .bCur = aduState.volume[request->bChannelNumber] };
          usbSetupTransfer(usbp, (uint8_t *)&curVol, sizeof(curVol), NULL);
          bResult = true;
          break;
        }

        default: break;
      }
    }
    else // Set Requests
    {
      switch(request->bRequest)
      {
        case AUDIO_CS_REQ_CUR:
        {
          aduControlChannel = request->bChannelNumber;
          usbSetupTransfer(usbp, aduControlData, request->wLength, aduSetVolume);
          bResult = true;

          // notify
          chSysLockFromIsr();
          chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_VOLUME);
          chSysUnlockFromIsr();

          break;
        }

        default: break;
      }
    }

  } 


  return bResult;
}

bool aduHandleClockRequest(USBDriver *usbp, audio_control_request_t *request)
{
  bool bResult = false;

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if(request->bmRequestType_bit.direction) // Get requests
    {
      switch(request->bRequest)
      {
        case AUDIO_CS_REQ_CUR:
        {
          audio_control_cur_4_t curf = { (int32_t) aduState.currentSampleRate };
          usbSetupTransfer(usbp, (uint8_t *)&curf, sizeof(curf), NULL);
          bResult = true;
          break;
        }
      
        case AUDIO_CS_REQ_RANGE:
        {
          // Get sample rate range.
          audio_control_range_4_n_t(N_SAMPLE_RATES) rangef;
          rangef.wNumSubRanges = (uint16_t)N_SAMPLE_RATES;
          
          uint8_t i;
          for(i = 0; i < N_SAMPLE_RATES; i++)
          {
            rangef.subrange[i].bMin = (int32_t) aduSampleRates[i];
            rangef.subrange[i].bMax = (int32_t) aduSampleRates[i];
            rangef.subrange[i].bRes = 0;
          }
          usbSetupTransfer(usbp, (uint8_t *)&rangef, sizeof(rangef), NULL);
          bResult = true;
          break;
        }

        default : break;
      }
    }
    else // Set requests
    {
      switch(request->bRequest)
      {
        case AUDIO_CS_REQ_CUR:
        {
          usbSetupTransfer(usbp, aduControlData, request->wLength, aduSetSampleRate);
          bResult = true;
          break;
        }

        default: break;
      }
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    usbSetupTransfer(usbp, (uint8_t *)&cur_valid, sizeof(cur_valid), NULL);
    bResult = true;
  }
 
  return bResult;
}

bool aduControl(USBDriver *usbp)
{
  audio_control_request_t *acrp = (audio_control_request_t *)usbp->setup;

  if (acrp->bInterface == ITF_NUM_AUDIO_STREAMING_CONTROL) 
  {
    switch(acrp->bEntityID)
    {
      case AUDIO_FUNCTION_UNIT_ID:
      {
        return aduHandleVolumeRequest(usbp, acrp);
        break;
      }

      case UAC2_ENTITY_CLOCK:
      {
        return aduHandleClockRequest(usbp, acrp);
        break;
      }
      default: break;
    }
  }
  return false;
}


//                                       4              5               1            (3 << 8) | 2     6
bool aduSwitchInterface(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req, uint16_t wValue, uint16_t length) 
{
  bool bResult = false;
 
  if(entity == 0)
  {
    if(iface == ITF_NUM_AUDIO_STREAMING_SPEAKER)
    {
      aduEnableOutput(usbp, wValue);
      bResult = true;
    }
    else if(iface == ITF_NUM_AUDIO_STREAMING_MICROPHONE)
    {
      aduEnableInput(usbp, wValue);
      bResult = true;
    }
  }

  if(bResult)
    usbSetupTransfer(usbp, NULL, 0, NULL);

  return bResult;
}


/**
 * @brief   Notification of data removed from the input queue.
 */
static void inotify(GenericQueue *qp) 
{
}

/**
 * @brief   Notification of data inserted into the output queue.
 */
static void onotify(GenericQueue *qp) 
{
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Audio USB Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */

// Note this is never called by halInit(), the above comment is incorrect
void aduInit(void) 
{
}
 
/**
 * @brief   Initializes a audio driver
 * @details The HW dependent part of the initialization has to be performed
 *          outside, usually in the hardware initialization code.
 *
 * @param[out] adup     pointer to a @p AudioUSBDriver structure
 *
 * @init
 */
void aduObjectInit(AudioUSBDriver *adup)
{
  // default sample rate
  aduState.currentSampleRate = 48000;
  
  // default is disabled
  aduState.isOutputActive = false;
  aduState.isInputActive = false;
  
  // frame stuff
  aduResetOutputBuffers();
  aduResetInputBuffers();
  
  // set not muted
  aduState.mute[0] = 0;
  aduState.mute[1] = 0;
  aduState.mute[2] = 0;

  // set 0db volume
  aduState.volume[0] = VOLUME_CTRL_0_DB;
  aduState.volume[1] = VOLUME_CTRL_0_DB;
  aduState.volume[2] = VOLUME_CTRL_0_DB;

  adup->vmt = NULL; // none at the moment
  chEvtInit(&adup->event);
  adup->state = ADU_STOP;
  
  chIQInit(&adup->iqueue, adup->ib, AUDIO_USB_BUFFERS_SIZE, inotify, adup);
  chOQInit(&adup->oqueue, adup->ob, AUDIO_USB_BUFFERS_SIZE, onotify, adup);
}

/**
 * @brief   Configures and starts the driver.
 *
 * @param[in] bdup      pointer to a @p AudioUSBDriver object
 * @param[in] config    the Audio USB driver configuration
 *
 * @api
 */
void aduStart(AudioUSBDriver *adup, const AudioUSBConfig *config) 
{
  USBDriver *usbp = config->usbp;

  chDbgCheck(adup != NULL);

  chSysLock()
  ;
  chDbgAssert((adup->state == ADU_STOP) || (adup->state == ADU_READY),
              "aduStart(), #1 invalid state");
  usbp->in_params[config->iso_in - 1] = adup;
  usbp->out_params[config->iso_out - 1] = adup;
  adup->config = config;
  adup->state = ADU_READY;
  chSysUnlock()
  ;

}

/**
 * @brief   Stops the driver.
 * @details Any thread waiting on the driver's queues will be awakened with
 *          the message @p Q_RESET.
 *
 * @param[in] adup      pointer to a @p AudioUSBDriver object
 *
 * @api
 */
void aduStop(AudioUSBDriver *adup) 
{
  USBDriver *usbp = adup->config->usbp;

  chDbgCheck(adup != NULL);

  chSysLock()
  ;

  chDbgAssert((adup->state == ADU_STOP) || (adup->state == ADU_READY),
              "aduStop(), #1 invalid state");

  /* Driver in stopped state.*/
  usbp->in_params[adup->config->iso_in - 1] = NULL;
  usbp->out_params[adup->config->iso_out - 1] = NULL;
  adup->state = ADU_STOP;

  /* Queues reset in order to signal the driver stop to the application.*/
  chnAddFlagsI(adup, CHN_DISCONNECTED);
  chIQResetI(&adup->iqueue);
  chOQResetI(&adup->oqueue);
  chSchRescheduleS();

  chSysUnlock()
  ;
}

/**
 * @brief   USB device configured handler.
 *
 * @param[in] adup      pointer to a @p AudioUSBDriver object
 *
 * @iclass
 */
void aduConfigureHookI(AudioUSBDriver *adup) 
{
  //USBDriver *usbp = adup->config->usbp;

  chIQResetI(&adup->iqueue);
  chOQResetI(&adup->oqueue);
  chnAddFlagsI(adup, CHN_CONNECTED);
}

/**
 * @brief   Default requests hook.
 * @details Applications wanting to use the Audio USB driver can use
 *          this function as requests hook in the USB configuration.
 *          The following requests are emulated:
 *          - CDC_GET_LINE_CODING.
 *          - CDC_SET_LINE_CODING.
 *          - CDC_SET_CONTROL_LINE_STATE.
 *          .
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The hook status.
 * @retval TRUE         Message handled internally.
 * @retval FALSE        Message not handled.
 */
bool_t aduRequestsHook(USBDriver *usbp) {

  (void)usbp;
  return FALSE;
}


void aduSofHookI(AudioUSBDriver *adup)
{
  Analyse(GPIOD, 4, 1);
  Analyse(GPIOD, 4, 0);
}

void aduResetInputBuffers(void)
{

}

void aduResetOutputBuffers(void)
{
  aduState.currentFrame               = 0;
  aduState.lastOverunFrame            = 0;
  aduState.currentFrame               = 0;
  aduState.lastOverunFrame            = 0;
  aduState.sampleAdjustEveryFrame     = 0;
  aduState.sampleAdjustFrameCounter   = 0; 
  aduState.sampleOffset               = 0;
  aduState.codecMetricsSampleOffset   = 0;
  aduState.codecMetricsBlocksOkCount  = 0;
  aduState.txRingBufferWriteOffset    = 0;
  aduState.txRingBufferReadOffset     = 0;
  aduState.txRingBufferUsedSize       = 0;
  aduState.txCurrentRingBufferSize    = TX_RING_BUFFER_UNDERFLOW_SIZE + TX_RING_BUFFER_NORMAL_SIZE;
  aduState.state                      = asInit;

  memset(aduTxRingBuffer, 0, sizeof(aduTxRingBuffer));
#if !TX_DIRECT  
  memset(aduTxBuffer, 0, sizeof(aduTxBuffer));
#endif
}


void aduEnableInput(USBDriver *usbp, bool bEnable)
{
  // this is ksoloti->host
  if(bEnable != aduState.isInputActive)
  {
    aduState.isInputActive = bEnable;
    chSysLockFromIsr();
    chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_INPUT);
    if(bEnable)
      aduInitiateTransmitI(usbp, USE_TRANSFER_SIZE_BYTES);
    else
      aduResetInputBuffers();
    chSysUnlockFromIsr();
  }
}

void aduEnableOutput(USBDriver *usbp, bool bEnable)
{
  // this is host->ksoloti
  if(bEnable != aduState.isOutputActive)
  {
    aduState.isOutputActive = bEnable;
    chSysLockFromIsr();
    chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_OUTPUT);
    if(bEnable)
      aduInitiateReceiveI(usbp, USE_TRANSFER_SIZE_BYTES);
    else
      aduResetOutputBuffers();

    chSysUnlockFromIsr();
  }
}










#define CODEC_CONTROLLED 1

#if CODEC_CONTROLLED

// simplify.

// TX always transmits base on packet size, so fixed offsets, 96, 192, 288 etc

// aduCodecData handles keeping the buffer full so TX is not starved
// and doesn;t over or under flow.

// Attempts to keep two TX buffers worth of data at all times before next TX - TX_RING_BUFFER_NORMAL_SIZE

// ring buffer never changes size.
uint16_t aduSkippedSamplesStart = 0;
int16_t aduSkippedSampleValue = 0;
uint16_t aduAddedSamplesStart = 0;
int16_t aduAddedSampleValue = 0;

void aduCodecData (int32_t *in, int32_t *out)
{
  if(aduState.isOutputActive)
  {
    Analyse(GPIOB, 7, 1);

    uint16_t uLen = 32;
    uint16_t uFeedbackLen = uLen;

#if EMULATE_UNDERRUN_SKIP_SAMPLE_EVERY_CODEC_FRAME
    static uint16_t uCount = 0;
    // dont emulate if we are not in normal state
    if((uCount >= EMULATE_UNDERRUN_SKIP_SAMPLE_EVERY_CODEC_FRAME) && (aduState.state == asNormal))
    {
      uCount = 0;
      uLen = 30;
      uFeedbackLen -=2;
    }
    else
      uCount++;
#endif  

    if(aduState.state == asCodecRemove)
    {
      // remove two samples
      Analyse(GPIOB, 3, 1);
      uLen -= 2;
      aduState.state = asNormal;
      Analyse(GPIOB, 3, 0);
    } 
    else if(aduState.state == asCodecDuplicate)
    {
      // add two samples 
      Analyse(GPIOC, 7, 1);
      aduAddedSamplesStart = aduState.txRingBufferWriteOffset;
      aduAddedSampleValue = out[0]>>16;

      int u; for (u=0; u< 2; u++)
      {
        aduTxRingBuffer[aduState.txRingBufferWriteOffset] = out[u] >> 16;
        if (++(aduState.txRingBufferWriteOffset) == aduState.txCurrentRingBufferSize) 
          aduState.txRingBufferWriteOffset= 0;
      }
      aduState.state = asNormal;
      aduState.txRingBufferUsedSize+=2;
      Analyse(GPIOC, 7, 0);
    }

    int u; for (u=0; u< uLen; u++)
    {
      aduTxRingBuffer[aduState.txRingBufferWriteOffset] = out[u] >> 16;
      if (++(aduState.txRingBufferWriteOffset) == aduState.txCurrentRingBufferSize) 
        aduState.txRingBufferWriteOffset= 0;
    }

    if(uLen < 32)
    {
      aduSkippedSamplesStart = aduState.txRingBufferWriteOffset;
      aduSkippedSampleValue = out[uLen+1] >> 16;
    }
    aduState.txRingBufferUsedSize+=uLen;
    aduState.codecFrameSampleCount+=uFeedbackLen;


    AddOverunLog(ltCodecCopyEnd__);

    Analyse(GPIOB, 7, 0);
  }
}

void aduCodecFrameEnded(void)
{
  // USB clock and Codec clock will be different
  // we can get underruns and overruns.
  // also the clocks slide against each other so we can get
  // jitter in the number of codec frames received
  // per USB frame, 1, 2, 3, 0r 4.
  //
  // we want to distort the USB stream as little as possible
  // so we can't adjust the sample counts based on the codec clock
  // or on the USB clock

  // we want to try to maintain TX_RING_BUFFER_NORMAL_SIZE samples available
  // at the end of this method,

  // we must never go under USE_TRANSFER_SIZE_BYTES samples available
  // at the end of this method

  // invrement current usb frame
  aduState.currentFrame++;

  int16_t nFrameSampleOffest = (int16_t)(aduState.codecFrameSampleCount)-96;
  aduState.codecMetricsSampleOffset += nFrameSampleOffest;

  if(nFrameSampleOffest < 0)
  {
    Analyse(GPIOB, 4, 1);
    Analyse(GPIOB, 4, 0);
  }
  else if(nFrameSampleOffest > 0)
  {
    Analyse(GPIOB, 6, 1);
    Analyse(GPIOB, 6, 0);
  }

  if(0 == (aduState.currentFrame % CODEC_METICS_MS))
  {
    if(aduState.codecMetricsSampleOffset  < 0)
    {
      Analyse(GPIOD, 6, 1);
      Analyse(GPIOD, 6, 0);
    }
    else if(aduState.codecMetricsSampleOffset  > 0)
    {
      Analyse(GPIOD, 3, 1);
      Analyse(GPIOD, 3, 0);
    }

    if(aduState.codecMetricsSampleOffset != 0)
    {
      // ok we are out of sync, adjust to sync
      aduState.sampleOffset += aduState.codecMetricsSampleOffset;
      uint16_t uUseBlocks   = aduState.codecMetricsBlocksOkCount+1;

      // make recovery 1 block quicker
      // if(uUseBlocks > 1)
      //   uUseBlocks -=1;

      // calculate sample adjust counter
      aduState.sampleAdjustEveryFrame = (CODEC_METICS_MS*uUseBlocks) / ((abs(aduState.sampleOffset)>>1));
      aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame;

      // reset
      aduState.codecMetricsBlocksOkCount = 0;

      Analyse(GPIOB, 8, 0);
    }
    else
    {
      // block is ok so increment counter
      aduState.codecMetricsBlocksOkCount++;
    }

    aduState.codecMetricsSampleOffset = 0;
  }


  // we need some checks here for debugging
  if(aduState.state > asFillingUnderflow)
  {
    if(aduState.txRingBufferUsedSize < USE_TRANSFER_SIZE_SAMPLES)
    {
      Analyse(GPIOA, 9, 1);
      Analyse(GPIOA, 9, 0);
    }

    if(aduState.txRingBufferUsedSize > TX_RING_BUFFER_NORMAL_SIZE)
    {
      Analyse(GPIOG, 10, 1);
      if(aduState.txRingBufferUsedSize > TX_RING_BUFFER_NORMAL_SIZE + TX_RING_BUFFER_UNDERFLOW_SIZE)
      {
        // really bad 
        Analyse(GPIOG, 10, 0);
        Analyse(GPIOG, 10, 1);
      }
      Analyse(GPIOG, 10, 0);
    }

    uint16_t uCalcSize;
    if((aduState.txRingBufferWriteOffset < aduState.txRingBufferReadOffset))
      uCalcSize = (aduState.txRingBufferWriteOffset + TX_RING_BUFFER_NORMAL_SIZE + TX_RING_BUFFER_UNDERFLOW_SIZE) - aduState.txRingBufferReadOffset;
    else
      uCalcSize = aduState.txRingBufferWriteOffset - aduState.txRingBufferReadOffset;

    if(uCalcSize != aduState.txRingBufferUsedSize)
    {
      Analyse(GPIOG, 10, 1);
      Analyse(GPIOG, 10, 0);
    }
  }

  AddOverunLog(ltFrameEndedEnd__);
}

void aduCodecFrameStarted(void)
{
  aduState.codecFrameSampleCount = 0;
  
  if( aduState.sampleOffset != 0)
  {
    if(aduState.sampleAdjustFrameCounter == 0)
    {
      if(aduState.sampleOffset > 0)
      {
        // adjust overrun
        // chuck samples awway
        //Analyse(GPIOB, 3, 1);
        aduState.sampleOffset-=2;
        aduState.state = asCodecRemove;
        //Analyse(GPIOB, 3, 0);
      }
      else
      {
        // adjust underrun
        // duplicate sample
        //Analyse(GPIOC, 7, 1);
        aduState.sampleOffset+=2;
        aduState.state = asCodecDuplicate;
        //Analyse(GPIOC, 7, 0);
      }

      // check for finish or restert
      if(aduState.sampleOffset)
      {
        aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame;
      }
      else
      {
        aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame = 0;
      }
    }
    else
      aduState.sampleAdjustFrameCounter--;
  }
  else
    Analyse(GPIOB, 8, 1);

  AddOverunLog(ltFrameStartedEnd);
}

void aduInitiateTransmitI(USBDriver *usbp, size_t uCount)
{
  Analyse(GPIOD, 5, 1);
  
  // tell codec copy that USB frame has ended
  aduCodecFrameEnded();

  int16_t *pTxLocation = NULL;
  if(aduState.state == asInit || aduState.state == asFillingUnderflow)
  {
    // send silence
    pTxLocation = (aduTxRingBuffer + TX_RING_BUFFER_FULL_SIZE) - USE_TRANSFER_SIZE_SAMPLES;

    // wait for unflow buffer to be filled and synced
    if(aduState.txRingBufferUsedSize == TX_RING_BUFFER_NORMAL_SIZE)
      aduState.state = asNormal;
    else if(aduState.txRingBufferUsedSize > TX_RING_BUFFER_NORMAL_SIZE)
    {
      aduResetInputBuffers();
      aduResetOutputBuffers();
    }
    AddOverunLog(ltWaitingForSync_);
  }

  // transmit from buffer, increase read offset
  if(aduState.state > asFillingUnderflow)
  {
    // set transmit location
    pTxLocation = aduTxRingBuffer + aduState.txRingBufferReadOffset;

    // increase and wrap read offset
    aduState.txRingBufferReadOffset += USE_TRANSFER_SIZE_SAMPLES;
    if(aduState.txRingBufferReadOffset == TX_RING_BUFFER_UNDERFLOW_SIZE + TX_RING_BUFFER_NORMAL_SIZE)
      aduState.txRingBufferReadOffset = 0;

    // decrease buffer used size
    aduState.txRingBufferUsedSize -= USE_TRANSFER_SIZE_SAMPLES;
  }

  AddOverunLog(ltAfterTXAdjust__);

  // tell codec copy that USB frame has started
  aduCodecFrameStarted();

#if CHECK_USB_DATA
  // DEBUG test USB Data, requires USBOutputTest.axp running on Ksoloiti
  volatile int16_t tmpData[46];
  bool bOk = true;
  uint16_t u; for( u = 0; u < 46; u++)
  {
    uint32_t uDiff = abs(pTxLocation[(u*2)+2] - pTxLocation[u*2]);
    tmpData[u] = (pTxLocation[(u*2)+2] - pTxLocation[u*2]);
    if(uDiff > 300)
    {
      bOk = false;
      Analyse(GPIOA, 9, 1);
      Analyse(GPIOA, 9, 0);
    }
  }

  if(!bOk)
  {
    Analyse(GPIOA, 9, 1);
    Analyse(GPIOA, 9, 0);
  }
#endif

  // transmit USB data
  usbStartTransmitI(usbp, 3, (uint8_t *)pTxLocation, USE_TRANSFER_SIZE_BYTES);
  aduAddTransferLog(blStartTransmit, USE_TRANSFER_SIZE_BYTES);

  Analyse(GPIOD, 5, 0);
}

#else
void aduCodecData (int32_t *in, int32_t *out)
{
  uint8_t uLen = 32;

  if(aduState.isOutputActive)
  {
#if EMULATE_UNDERRUN_SKIP_SAMPLE_EVERY_CODEC_FRAME
    static uint16_t uCount = 0;
    if(uCount == EMULATE_UNDERRUN_SKIP_SAMPLE_EVERY_CODEC_FRAME)
    {
      uCount = 0;
      uLen = 30;
    }
    else
      uCount++;
#endif  
    // 2 channel, 24 bits(in 32 bits)
    Analyse(GPIOB, 7, 1);

    // add to ring buffer, add checks and optimise later
#if 0
    // not coded to use uLen as to give optimiser the best chance
    uint16_t uRemainingSamples = aduState.txCurrentRingBufferSize - aduState.txRingBufferWriteOffset;
    uint16_t *pSrc  = out;
    uint16_t *pDest = aduTxRingBuffer + aduState.txRingBufferWriteOffset;

    int u; for (u=0; u< uRemainingSamples; u++)
      *pDest++ - *pSrc++;

        
#else    
    int u; for (u=0; u< uLen; u++)
    {
      aduTxRingBuffer[aduState.txRingBufferWriteOffset] = out[u] >> 16;
      if (++(aduState.txRingBufferWriteOffset) == aduState.txCurrentRingBufferSize) 
        aduState.txRingBufferWriteOffset= 0;
    }
    aduState.txRingBufferUsedSize+=uLen;
    aduState.codecFrameSampleCount+=uLen;

    AddOverunLog(0);

    Analyse(GPIOB, 7, 0);
  }
#endif
}

void aduInitiateTransmitI(USBDriver *usbp, size_t uCount)
{
  static bool bClockOk = true;

  Analyse(GPIOD, 5, 1);

  AddOverunLog(1);

  aduState.lastTransferSize = USE_TRANSFER_SIZE_BYTES;
  aduState.currentFrame++;

  uint16_t startTxRingBufferReadOffset = aduState.txRingBufferReadOffset;

#ifdef TX_DIRECT
  int16_t *pTxLocation = NULL;
  if(aduState.state == asInit || aduState.state == asFillingUnderflow)
  {
    // send silence
    pTxLocation = (aduTxRingBuffer + TX_RING_BUFFER_FULL_SIZE) - USE_TRANSFER_SIZE_SAMPLES;

    // wait for unflow buffer to be filled
    if(aduState.txRingBufferUsedSize >= TX_RING_BUFFER_UNDERFLOW_SIZE)
      aduState.state = asNormal;
    else
      aduState.state = asFillingUnderflow;
  }
  else
  {
    // we should nexer get USB underflows now, but just incase make sure we handle it.
    size_t uSamplesUsed;
    if(aduState.txRingBufferUsedSize < USE_TRANSFER_SIZE_SAMPLES)
    {
      // USB underflow, should never see this as long as underflow buffer is big enough.
      uSamplesUsed = aduState.txRingBufferUsedSize;
      // add missing sample metrics as if we had used them
      aduState.codecFrameSampleCount += (USE_TRANSFER_SIZE_SAMPLES - uSamplesUsed);
    }
    else
      uSamplesUsed = USE_TRANSFER_SIZE_SAMPLES;

    pTxLocation = aduTxRingBuffer + aduState.txRingBufferReadOffset;
    aduState.txRingBufferReadOffset = (aduState.txRingBufferReadOffset + uSamplesUsed) % aduState.txCurrentRingBufferSize;
    aduState.txRingBufferUsedSize -= uSamplesUsed;
    if(aduState.txRingBufferReadOffset==0)
    {
      // we are synced, reset buffer length
      aduState.txCurrentRingBufferSize = TX_RING_BUFFER_UNDERFLOW_SIZE + TX_RING_BUFFER_NORMAL_SIZE;
    }

#else
  if(aduState.state == asInit || aduState.state == asFillingUnderflow)
  {
    // send full frame of silence
    uint32_t u; for(u=0; u <USE_TRANSFER_SIZE_SAMPLES; u++)
    {
      aduTxBuffer[u] = 0;
    }

    if(aduState.txRingBufferUsedSize >= TX_RING_BUFFER_UNDERFLOW_SIZE)
      aduState.state = asNormal;
    else
      aduState.state = asFillingUnderflow;
  }
  else
  {
    // probably change to TX from ring buffer
    // Stage 1 - remove from ring buffer and put in transferbuffer
    if(aduState.txRingBufferUsedSize >= USE_TRANSFER_SIZE_SAMPLES)
    {
      uint32_t u; for(u=0; u < USE_TRANSFER_SIZE_SAMPLES; u++)
      {
        aduTxBuffer[u] = aduTxRingBuffer[aduState.txRingBufferReadOffset];

        if (++(aduState.txRingBufferReadOffset) == aduState.txCurrentRingBufferSize) 
        {
          if(u != USE_TRANSFER_SIZE_SAMPLES-1)
          {
            Analyse(GPIOA, 9, 1); 
            Analyse(GPIOA, 9, 0);
          }

          aduState.txRingBufferReadOffset= 0;

        }
      }
      aduState.txRingBufferUsedSize -= USE_TRANSFER_SIZE_SAMPLES;
      if(aduState.txRingBufferReadOffset==0)
      {
        // we are synced, reset buffer length
        aduState.txCurrentRingBufferSize = TX_RING_BUFFER_UNDERFLOW_SIZE + TX_RING_BUFFER_NORMAL_SIZE;
      }
    }
    else
    {
      // this should never happen now
      // Avoids extra buffer.
      // Send full frame of silence, sending smaller packets whatever you read messes up OSX at least.
      size_t uActual = aduState.txRingBufferUsedSize;
      aduState.txRingBufferReadOffset = (aduState.txRingBufferReadOffset + uActual) % aduState.txCurrentRingBufferSize;

      uint32_t u; for(u=0; u <USE_TRANSFER_SIZE_SAMPLES; u++)
      {
        aduTxBuffer[u] = 0;
      }
      aduState.txRingBufferUsedSize -= uActual;

      // add missing sample metrics as if we had used them
      aduState.codecFrameSampleCount += (USE_TRANSFER_SIZE_SAMPLES - uActual);
    }
#endif    

    AddOverunLog(3);

    // stage 2 - If we have underrun/overrun adjust counters
    int16_t nFrameSampleOffest = (int16_t)(aduState.codecFrameSampleCount)-96;
    aduState.codecMetricsSampleOffset += nFrameSampleOffest;

    if(nFrameSampleOffest < 0)
    {
      Analyse(GPIOB, 4, 1);
      Analyse(GPIOB, 4, 0);
    }
    else if(nFrameSampleOffest > 0)
    {
      Analyse(GPIOB, 6, 1);
      Analyse(GPIOB, 6, 0);
    }

    if(0 == (aduState.currentFrame % CODEC_METICS_MS))
    {
      if(aduState.codecMetricsSampleOffset  < 0)
      {
        Analyse(GPIOD, 6, 1);
        Analyse(GPIOD, 6, 0);
      }
      else if(aduState.codecMetricsSampleOffset  > 0)
      {
        Analyse(GPIOD, 3, 1);
        Analyse(GPIOD, 3, 0);
      }

      if(aduState.codecMetricsSampleOffset != 0)
      {
        // ok we are out of sync, adjust to sync
        aduState.sampleOffset    += aduState.codecMetricsSampleOffset;
        uint16_t uUseBlocks = aduState.codecMetricsBlocksOkCount;

        // make recovery 1 block quicker
        if(uUseBlocks > 1)
          uUseBlocks -=1;

        aduState.sampleAdjustEveryFrame = (CODEC_METICS_MS*uUseBlocks) / ((abs(aduState.sampleOffset)>>1));

        aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame;
        aduState.codecMetricsBlocksOkCount = 0;


        // if(aduState.sampleOffset > 32)
        // {
        //   Analyse(GPIOG, 10, 1);
        //   Analyse(GPIOG, 10, 0);
        // }
        bClockOk = false;
        Analyse(GPIOB, 8, 0);
      }
      else
        aduState.codecMetricsBlocksOkCount++;

      aduState.codecMetricsSampleOffset = 0;
    }
    AddOverunLog(4);

    // stage 3 - handle overrun/underrun adjustments
    if( aduState.sampleOffset != 0)
    {
      if(aduState.sampleAdjustFrameCounter == 0)
      {
        if(aduState.sampleOffset > 0)
        {
          // adjust overrun
          // chuck samples awway
          Analyse(GPIOB, 3, 1);
          aduState.sampleOffset-=2;
          aduState.txRingBufferUsedSize-=2;
          //aduState.txRingBufferReadOffset = (aduState.txRingBufferReadOffset+2) % aduState.txCurrentRingBufferSize;
          aduState.txRingBufferReadOffset += 2;
          aduState.txCurrentRingBufferSize += 2;
          if(aduState.txCurrentRingBufferSize > TX_RING_BUFFER_FULL_SIZE)
          {
            Analyse(GPIOG, 10, 1);
            Analyse(GPIOG, 10, 0);
          }

          Analyse(GPIOB, 3, 0);
        }
        else
        {
          // adjust underrun
          // duplicate sample
          Analyse(GPIOC, 7, 1);
          if(aduState.txRingBufferWriteOffset == 0)
          {
            //duplicate from end
            aduTxRingBuffer[aduState.txRingBufferWriteOffset]   = aduTxRingBuffer[aduState.txCurrentRingBufferSize-2];
            aduTxRingBuffer[aduState.txRingBufferWriteOffset+1] = aduTxRingBuffer[aduState.txCurrentRingBufferSize-1];
          }
          else
          {
            aduTxRingBuffer[aduState.txRingBufferWriteOffset]   = aduTxRingBuffer[aduState.txRingBufferWriteOffset-2];
            aduTxRingBuffer[aduState.txRingBufferWriteOffset+1] = aduTxRingBuffer[aduState.txRingBufferWriteOffset-1];
          }
          aduState.txRingBufferWriteOffset = (aduState.txRingBufferWriteOffset +2) % aduState.txCurrentRingBufferSize;
          aduState.txRingBufferUsedSize+=2;
          aduState.sampleOffset+=2;
          Analyse(GPIOC, 7, 0);
        }

        // check for finish or restert
        if(aduState.sampleOffset)
        {
          aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame;
        }
        else
        {
          aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame = 0;
        }
      }
      else
        aduState.sampleAdjustFrameCounter--;
    }
    else
    {
      if(!bClockOk)
      {    
        Analyse(GPIOB, 8, 1);
        bClockOk = true;
      }
    }
    
      // Analyse(GPIOD, 5, 0);
      // Analyse(GPIOD, 5, 1);

    AddOverunLog(5);

    if(aduState.txRingBufferUsedSize == TX_RING_BUFFER_UNDERFLOW_SIZE)
    {
      // ok all data is synced and the read data is not in the first normal block, reset the offsets
      if(startTxRingBufferReadOffset >= USE_TRANSFER_SIZE_SAMPLES)
      {
        aduState.txRingBufferReadOffset = 0;
        aduState.txRingBufferWriteOffset = 0;
        aduState.txCurrentRingBufferSize = TX_RING_BUFFER_UNDERFLOW_SIZE + TX_RING_BUFFER_NORMAL_SIZE;
        Analyse(GPIOG, 10, 1);
        Analyse(GPIOG, 10, 0);
      }
    }
  }

  // reset codec fram sample count
  aduState.codecFrameSampleCount = 0;

  // transmit
#ifdef TX_DIRECT
  usbStartTransmitI(usbp, 3, (uint8_t *)pTxLocation, aduState.lastTransferSize );
#else
  usbStartTransmitI(usbp, 3, (uint8_t *)aduTxBuffer, aduState.lastTransferSize );
#endif
  aduAddTransferLog(blStartTransmit, USE_TRANSFER_SIZE_BYTES);

  Analyse(GPIOD, 5, 0);
}

#endif

/**
 * @brief   Default data transmitted callback.
 * @details The application must use this function as callback for the IN
 *          data endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 */
void aduDataTransmitted(USBDriver *usbp, usbep_t ep) 
{
  USBInEndpointState *pEpState = usbp->epc[ep]->in_state;
  volatile uint32_t uTransmittedCount = pEpState->txcnt;
  if(uTransmittedCount != aduState.lastTransferSize)
  {
    Analyse(GPIOD, 5, 1);
    Analyse(GPIOD, 5, 0);
  }    

  aduAddTransferLog(blEndTransmit, uTransmittedCount);

  if(aduState.isOutputActive)
  {
    chSysLockFromIsr();
    aduInitiateTransmitI(usbp, USE_TRANSFER_SIZE_BYTES);
    chSysUnlockFromIsr();
  }
  else
    aduResetOutputBuffers();
}

/**
 * @brief   Default data received callback.
 * @details The application must use this function as callback for the OUT
 *          data endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 */
void aduDataReceived(USBDriver *usbp, usbep_t ep) 
{
#if ADU_TRANSFER_LOG_SIZE
  USBOutEndpointState *pEpState = usbp->epc[ep]->out_state;
  volatile uint32_t uReceivedCount = pEpState->rxcnt;
  aduAddTransferLog(blEndReceive, uReceivedCount);
#endif


  if(aduState.isOutputActive)
  {
    chSysLockFromIsr();
    aduInitiateReceiveI(usbp, USE_TRANSFER_SIZE_BYTES);
    chSysUnlockFromIsr();
  }
  else
    aduResetInputBuffers();
}



void aduInitiateReceiveI(USBDriver *usbp, size_t uCount)
{
  Analyse(GPIOG, 11, 1);
//  usbStartReceiveI(usbp, 3, (uint8_t *)aduRxBuffer[uRxWrite], USE_TRANSFER_SIZE);
  usbStartReceiveI(usbp, 3, (uint8_t *)aduRxBuffer, USE_TRANSFER_SIZE_BYTES);
  aduAddTransferLog(blStartReceive, uCount);
  Analyse(GPIOG, 11, 0);
}


