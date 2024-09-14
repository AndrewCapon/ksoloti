
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "audio_usb.h"
#include "usb_lld.h"
#include "chevents.h"
#include "usbcfg.h"
#include "analyse.h"

// do not set higher than -O1
#pragma GCC optimize ("O0")

// O0 codecCopy = 14us, TX = 42.1us
// O1 codecCopy = 3.3us, TX = 11.5us

// O0 codecCopy = 11us, TX = 31.0us
// O1 codecCopy = 2.9us, TX = 10.1us

#define ADU_LOGGING 0

// this will be ping pong buffer
// static uint16_t aduTxBuffer[AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));
// static uint16_t aduRxBuffer[2][AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));
#define TX_RING_BUFFER_NORMAL_SIZE (96*2)
#define TX_RING_BUFFER_OVERFLOW_SIZE (96)
#define TX_RING_BUFFER_FULL_SIZE (TX_RING_BUFFER_NORMAL_SIZE + TX_RING_BUFFER_OVERFLOW_SIZE)

#define CODEC_METICS_MS (100)
//#define EMULATE_UNDERRUN_SKIP_SAMPLE_EVERY_CODEC_FRAME (3000/2)

static int16_t aduTxRingBuffer[TX_RING_BUFFER_FULL_SIZE] __attribute__ ((section (".sram3")));
static int16_t aduTxBuffer[96] __attribute__ ((section (".sram3")));
static int16_t aduRxBuffer[96] __attribute__ ((section (".sram3")));

#define ADU_TRANSFER_LOG_SIZE 0
#define ADU_OVERRUN_LOG_SIZE 4900

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
typedef struct _OverrunDebug
{
  uint8_t  index;
  uint16_t txRingBufferUsedSize;
  uint16_t txRingBufferWriteOffset;
  uint16_t txRingBufferReadOffset;
  int16_t  sampleOffset;
  uint16_t codecFrameSampleCount;
  int16_t  codecMetricsSampleOffset;
//  uint16_t txCurrentRingBufferSize;
} __attribute__((packed)) OverrunDebug;

OverrunDebug overrunDebug[ADU_OVERRUN_LOG_SIZE]   __attribute__ ((section (".sram3")));
uint16_t uLogIndex = 0;

void AddOverunLog(uint8_t index)
{
  overrunDebug[uLogIndex].index = index;
  overrunDebug[uLogIndex].txRingBufferUsedSize = aduState.txRingBufferUsedSize;
  overrunDebug[uLogIndex].txRingBufferWriteOffset = aduState.txRingBufferWriteOffset;
  overrunDebug[uLogIndex].txRingBufferReadOffset = aduState.txRingBufferReadOffset;
  overrunDebug[uLogIndex].sampleOffset = aduState.sampleOffset;
  overrunDebug[uLogIndex].codecFrameSampleCount = aduState.codecFrameSampleCount;
  overrunDebug[uLogIndex].codecMetricsSampleOffset = aduState.codecMetricsSampleOffset;
  
  uLogIndex++;
  if(uLogIndex == ADU_OVERRUN_LOG_SIZE)
    uLogIndex = 0;
}
#else
  #define AddOverunLogLog(a)
#endif 


//#define USE_TRANSFER_SIZE 176
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

bool __attribute__((optimize("O0"))) aduHandleVolumeRequest(USBDriver *usbp, audio_control_request_t *request)
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

bool __attribute__((optimize("O0"))) aduHandleClockRequest(USBDriver *usbp, audio_control_request_t *request)
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
  aduState.txCurrentRingBufferSize    = TX_RING_BUFFER_NORMAL_SIZE;

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















// hacky test
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
    int u; for (u=0; u< uLen; u++)
    {
      aduTxRingBuffer[aduState.txRingBufferWriteOffset] = out[u] >> 16;
      //aduState.txRingBufferWriteOffset = (aduState.txRingBufferWriteOffset + 1) % TX_RING_BUFFER_NORMAL_SIZE;
      if (++(aduState.txRingBufferWriteOffset) == TX_RING_BUFFER_NORMAL_SIZE) 
        aduState.txRingBufferWriteOffset= 0;
    }
    aduState.txRingBufferUsedSize+=uLen;
    aduState.codecFrameSampleCount+=uLen;

    AddOverunLog(0);

    Analyse(GPIOB, 7, 0);
  }
}


void aduInitiateReceiveI(USBDriver *usbp, size_t uCount)
{
  Analyse(GPIOG, 11, 1);
//  usbStartReceiveI(usbp, 3, (uint8_t *)aduRxBuffer[uRxWrite], USE_TRANSFER_SIZE);
  usbStartReceiveI(usbp, 3, (uint8_t *)aduRxBuffer, USE_TRANSFER_SIZE_BYTES);
  aduAddTransferLog(blStartReceive, uCount);
  Analyse(GPIOG, 11, 0);
}


void aduInitiateTransmitI(USBDriver *usbp, size_t uCount)
{
  static bool bClockOk = true;

  Analyse(GPIOD, 5, 1);

  AddOverunLog(1);

  aduState.lastTransferSize = USE_TRANSFER_SIZE_BYTES;
  aduState.currentFrame++;

  uint32_t uCalcUsed;
  if(aduState.txRingBufferWriteOffset < aduState.txRingBufferReadOffset)
  {
    // wrapped
    uCalcUsed = (TX_RING_BUFFER_NORMAL_SIZE+aduState.txRingBufferWriteOffset) - aduState.txRingBufferReadOffset;
  }
  else
    uCalcUsed = aduState.txRingBufferWriteOffset - aduState.txRingBufferReadOffset;

  if(uCalcUsed != aduState.txRingBufferUsedSize)
  {
    Analyse(GPIOG, 10, 1);
    Analyse(GPIOG, 10, 0);
  }

  // Analyse(GPIOD, 5, 0);
  // Analyse(GPIOD, 5, 1);

  uint8_t startTxRingBufferReadOffset = aduState.txRingBufferReadOffset;

  // probably change to TX from ring buffer
  // Stage 1 - remove from ring buffer and put in transferbuffer
  if(aduState.txRingBufferUsedSize >= USE_TRANSFER_SIZE_SAMPLES)
  {
    uint32_t u; for(u=0; u < USE_TRANSFER_SIZE_SAMPLES; u++)
    {
      aduTxBuffer[u] = aduTxRingBuffer[aduState.txRingBufferReadOffset];

      if (++(aduState.txRingBufferReadOffset) == TX_RING_BUFFER_NORMAL_SIZE) 
        aduState.txRingBufferReadOffset= 0;
    }
    aduState.txRingBufferUsedSize -= USE_TRANSFER_SIZE_SAMPLES;
  }
  else
  {
    // Real USB underrun just send any data we have, avoids extra buffer.
    aduState.lastTransferSize  = aduState.txRingBufferUsedSize;
    uint32_t u; for(u=0; u < aduState.lastTransferSize; u++)
    {
      aduTxBuffer[u] = aduTxRingBuffer[aduState.txRingBufferReadOffset];

      if (++(aduState.txRingBufferReadOffset) == TX_RING_BUFFER_NORMAL_SIZE) 
        aduState.txRingBufferReadOffset= 0;
    }
    aduState.txRingBufferUsedSize -= aduState.lastTransferSize;

    // add missing samples as if we had used thm
    aduState.codecFrameSampleCount += (USE_TRANSFER_SIZE_SAMPLES - aduState.lastTransferSize);
  }
  AddOverunLog(3);

  // Analyse(GPIOD, 5, 0);
  // Analyse(GPIOD, 5, 1);

 
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
      aduState.sampleAdjustEveryFrame = (CODEC_METICS_MS*(aduState.codecMetricsBlocksOkCount+1)) / ((abs(aduState.sampleOffset)>>1));
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
        aduState.txRingBufferReadOffset = (aduState.txRingBufferReadOffset+2) % TX_RING_BUFFER_NORMAL_SIZE;
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
          aduTxRingBuffer[aduState.txRingBufferWriteOffset]   = aduTxRingBuffer[TX_RING_BUFFER_NORMAL_SIZE-2];
          aduTxRingBuffer[aduState.txRingBufferWriteOffset+1] = aduTxRingBuffer[TX_RING_BUFFER_NORMAL_SIZE-1];
        }
        else
        {
          aduTxRingBuffer[aduState.txRingBufferWriteOffset]   = aduTxRingBuffer[aduState.txRingBufferWriteOffset-2];
          aduTxRingBuffer[aduState.txRingBufferWriteOffset+1] = aduTxRingBuffer[aduState.txRingBufferWriteOffset-1];
        }
        aduState.txRingBufferWriteOffset = (aduState.txRingBufferWriteOffset +2) % TX_RING_BUFFER_NORMAL_SIZE;
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

  if(aduState.txRingBufferUsedSize == 0)
  {
    // ok all data is synced and the read data is not in the first normal block, reset the offsets
    if(startTxRingBufferReadOffset >= USE_TRANSFER_SIZE_SAMPLES)
    {
      aduState.txRingBufferReadOffset = 0;
      aduState.txRingBufferWriteOffset = 0;
    }
  }

  aduState.codecFrameSampleCount = 0;

  if(aduState.txRingBufferUsedSize > TX_RING_BUFFER_NORMAL_SIZE)
  {
    Analyse(GPIOG, 10, 1);
    Analyse(GPIOG, 10, 0);

  }

  usbStartTransmitI(usbp, 3, (uint8_t *)aduTxBuffer, aduState.lastTransferSize );

  aduAddTransferLog(blStartTransmit, USE_TRANSFER_SIZE_BYTES);

  Analyse(GPIOD, 5, 0);
}


