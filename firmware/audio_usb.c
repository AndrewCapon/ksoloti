
#include "hal.h"
#include "audio_usb.h"
#include "usb_lld.h"
#include "chevents.h"

// do not set higher than -O1
#pragma GCC optimize ("O0")
#define ADU_LOGGING 0

// this will be ping pong buffer
// static uint16_t aduTxBuffer[AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));
// static uint16_t aduRxBuffer[2][AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));
#define TX_RING_BUFFER_SIZE (192*3)
#define CODEC_METICS_MS (1000)

static int16_t  aduTxRingBuffer[TX_RING_BUFFER_SIZE] __attribute__ ((section (".sram3")));

static uint16_t aduTxRingBufferWriteOffset = 0;
static uint16_t aduTxRingBufferReadOffset = 0;
static uint16_t aduTxRingBufferUsedSize  = 0;

static int16_t aduTxBuffer[192] __attribute__ ((section (".sram3")));
static int16_t aduRxBuffer[192] __attribute__ ((section (".sram3")));

uint16_t uCodecOffset = 0;

uint8_t uRxWrite = 0;
uint8_t uUsbTransmittingBuffer = 0;

#if ADU_LOGGING
typedef enum _BLType {blStartTransmit, blStartReceive, blEndTransmit, blEndReceive} BLType;

typedef struct _DBGLOG
{
  BLType    type;
  uint16_t  uSize;
} DBGLOG;

DBGLOG aduLog[1024] __attribute__ ((section (".sram3")));
uint16_t aduLogCount = 0;

void aduAddLog(BLType type, uint16_t uSize)
{
  if(aduLogCount == 0)
    memset(aduLog, 0, sizeof(aduLog));

  aduLog[aduLogCount].type = type;
  aduLog[aduLogCount].uSize = uSize;
  aduLogCount++;
  if(aduLogCount == 1024)
    aduLogCount = 0;
}
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

#define SWITCH_INTERFACE_DEBUG 1

#if SWITCH_INTERFACE_DEBUG
typedef struct 
{
  uint8_t iface;
  uint8_t entity;
  uint8_t req;
  uint16_t wValue;
  uint16_t length;
} SwitchDebug;


#define MAX_SWITCH_INTERFACE_DEBUG 1024
SwitchDebug switchDebug[MAX_SWITCH_INTERFACE_DEBUG]  __attribute__ ((section (".sram3")));
uint32_t nextSdIndex = 0;

void AddSwitchDebug(uint8_t iface, uint8_t entity, uint8_t req, uint16_t wValue, uint16_t length)
{
  switchDebug[nextSdIndex].iface = iface;
  switchDebug[nextSdIndex].entity = entity;
  switchDebug[nextSdIndex].req = req;
  switchDebug[nextSdIndex].wValue = wValue;
  switchDebug[nextSdIndex].length = length;
  
  nextSdIndex++;
  if(nextSdIndex == MAX_SWITCH_INTERFACE_DEBUG)
    nextSdIndex = 0;
}
#endif



#if CONTROL_MESSAGES_DEBUG
#define LOG_AMOUNT 128
uint32_t uLogCount = 0;
uint32_t uBadRequestsCount = 0;


audio_control_request_t requests[LOG_AMOUNT] __attribute__ ((section (".sram3")));
uint32_t badRequests[LOG_AMOUNT] __attribute__ ((section (".sram3")));;


#define SAMPLE_RATE_COUNT 128
typedef struct _SampleRateRequests
{
  bool     bGet;
  uint32_t uFreq;
} SampleRateRequests;
uint32_t uNextSampleRateRequestIndex = 0;

static SampleRateRequests aduSampleRateRequests[SAMPLE_RATE_COUNT]  __attribute__ ((section (".sram3")));

static void aduAddSampleRateRequest(bool bGet, uint32_t uSampleRate)
{
  aduSampleRateRequests[uNextSampleRateRequestIndex].bGet = 0;
  aduSampleRateRequests[uNextSampleRateRequestIndex].uFreq = uSampleRate;
  uNextSampleRateRequestIndex++;
  if(uNextSampleRateRequestIndex > SAMPLE_RATE_COUNT)
    uNextSampleRateRequestIndex = 0;
}
#endif



void aduEnableInput(USBDriver *usbp, bool bEnable);
void aduEnableOutput(USBDriver *usbp, bool bEnable);
void aduDataTransmitted(USBDriver *usbp, usbep_t ep); 
void aduDataReceived(USBDriver *usbp, usbep_t ep);
void aduInitiateReceiveI(USBDriver *usbp, size_t uCount);
void aduInitiateTransmitI(USBDriver *usbp, size_t uCount);


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

#if CONTROL_MESSAGES_DEBUG
  aduAddSampleRateRequest(0, uSampleRate);
#endif

  if(uSampleRate == 44100 || uSampleRate == 48000 || uSampleRate == 96000)
    aduState.currentSampleRate =  uSampleRate;

#if CONTROL_MESSAGES_DEBUG
  else
  {
    badRequests[uBadRequestsCount++] = uLogCount-1;
    // this should really not be happening!
  }
#endif

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

#if CONTROL_MESSAGES_DEBUG
  if(uLogCount < LOG_AMOUNT)
    memcpy(&requests[uLogCount++], request, sizeof(audio_control_request_t));
  else
    uLogCount = 0;
#endif

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if(request->bmRequestType_bit.direction) // Get requests
    {
      switch(request->bRequest)
      {
        case AUDIO_CS_REQ_CUR:
        {
#if CONTROL_MESSAGES_DEBUG
          // get current sample rate
          aduAddSampleRateRequest(1, aduCurrentSampleRate);
#endif
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
  // something dodgy here
#if SWITCH_INTERFACE_DEBUG
  AddSwitchDebug(iface, entity, req, wValue, length);
#endif

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
  // size_t n, maxsize;
  // AudioUSBDriver *adup = chQGetLink(qp);

  // /* If the USB driver is not in the appropriate state then transactions
  //    must not be started.*/
  // if ((usbGetDriverStateI(bdup->config->usbp) != USB_ACTIVE) ||
  //     (bdup->state != BDU_READY))
  //   return;

  // /* If there is in the queue enough space to hold at least one packet and
  //    a transaction is not yet started then a new transaction is started for
  //    the available space.*/
  // maxsize = bdup->config->usbp->epc[bdup->config->bulk_out]->out_maxsize;
  // if (!usbGetReceiveStatusI(bdup->config->usbp, bdup->config->bulk_out) &&
  //     ((n = chIQGetEmptyI(&bdup->iqueue)) >= maxsize)) {
  //   chSysUnlock();

  //   n = (n / maxsize) * maxsize;
  //   usbPrepareQueuedReceive(bdup->config->usbp,
  //                           bdup->config->bulk_out,
  //                           &bdup->iqueue, n);

  //   chSysLock();
  //   usbStartReceiveI(bdup->config->usbp, bdup->config->bulk_out);
  //}
}

/**
 * @brief   Notification of data inserted into the output queue.
 */
static void onotify(GenericQueue *qp) 
{
  // size_t n;
  // AudioUSBDriver *bdup = chQGetLink(qp);

  // /* If the USB driver is not in the appropriate state then transactions
  //    must not be started.*/
  // if ((usbGetDriverStateI(bdup->config->usbp) != USB_ACTIVE) ||
  //     (bdup->state != BDU_READY))
  //   return;

  // /* If there is not an ongoing transaction and the output queue contains
  //    data then a new transaction is started.*/
  // if (!usbGetTransmitStatusI(bdup->config->usbp, bdup->config->bulk_in) &&
  //     ((n = chOQGetFullI(&bdup->oqueue)) > 0)) {
  //   chSysUnlock();

  //   usbPrepareQueuedTransmit(bdup->config->usbp,
  //                            bdup->config->bulk_in,
  //                            &bdup->oqueue, n);

  //   chSysLock();
  //   usbStartTransmitI(bdup->config->usbp, bdup->config->bulk_in);
  // }
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
  aduState.currentFrame             = 0;
  aduState.lastOverunFrame          = 0;
  aduState.currentFrame             = 0;
  aduState.lastOverunFrame          = 0;
  aduState.sampleAdjustEveryFrame   = 0;
  aduState.sampleAdjustFrameCounter = 0; 
  aduState.sampleOffset             = 0;
  aduState.codecMetricsSampleOffset = 0;

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
  palWritePad(GPIOD, 4, 1);
  palWritePad(GPIOD, 4, 0);
}

void aduEnableInput(USBDriver *usbp, bool bEnable)
{
  // this is ksoloti->host
  if(bEnable != aduState.isInputActive)
  {
    if(bEnable)
    {
      // // lets fill the buffer;
      // uint16_t *pBuf = aduTxBuffer;
      // uint16_t u;
      // uint16_t uInc = 0xffff / 48;
      // uint16_t uVal = 0;
      
      // for(u = 0; u < 48; u++)
      // {
      //   *pBuf++ = uVal;
      //   *pBuf++ = uVal;
      //   uVal += uInc;
      // }

      chSysLockFromIsr();
      chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_INPUT);
      aduInitiateTransmitI(usbp, USE_TRANSFER_SIZE_BYTES);
      chSysUnlockFromIsr();
    }
    aduState.isInputActive = bEnable;
  }
}

void aduEnableOutput(USBDriver *usbp, bool bEnable)
{
  // this is host->ksoloti
  if(bEnable != aduState.isOutputActive)
  {
    aduState.isOutputActive = bEnable;
    if(bEnable)
    {
      //memset(aduBuffer, 0, sizeof(aduBuffer));
      chSysLockFromIsr();
      chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_OUTPUT);
      aduInitiateReceiveI(usbp, USE_TRANSFER_SIZE_BYTES);
      chSysUnlockFromIsr();
    }
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
    palWritePad(GPIOD, 5, 1);
    palWritePad(GPIOD, 5, 0);
  }    

#if ADU_LOGGING  
  aduAddLog(blEndTransmit, uTransmittedCount);
#endif

  chSysLockFromIsr();
  aduInitiateTransmitI(usbp, USE_TRANSFER_SIZE_BYTES);
  chSysUnlockFromIsr();
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
#if ADU_LOGGING
  USBOutEndpointState *pEpState = usbp->epc[ep]->out_state;
  volatile uint32_t uReceivedCount = pEpState->rxcnt;
  aduAddLog(blEndReceive, uReceivedCount);
#endif

  uRxWrite = !uRxWrite;

  chSysLockFromIsr();
  aduInitiateReceiveI(usbp, USE_TRANSFER_SIZE_BYTES);
  chSysUnlockFromIsr();
}

// hacky test
void aduCodecData (int32_t *in, int32_t *out)
{
  if(aduState.isOutputActive)
  {
    // 2 channel, 24 bits(in 32 bits)
    palWritePad(GPIOB, 7, 1);

    // add to ring buffer, add checks and optimise later
    int u; for (u=0; u< 32; u++)
    {
      aduTxRingBuffer[aduTxRingBufferWriteOffset] = out[u] >> 16;
      aduTxRingBufferWriteOffset = (aduTxRingBufferWriteOffset == TX_RING_BUFFER_SIZE) ? 0 : aduTxRingBufferWriteOffset+1;
      aduTxRingBufferUsedSize++;
    }
    aduState.codecFrameSampleCount+=32;

    palWritePad(GPIOB, 7, 0);
  }
}



void aduInitiateReceiveI(USBDriver *usbp, size_t uCount)
{
  palWritePad(GPIOG, 11, 1);
//  usbStartReceiveI(usbp, 3, (uint8_t *)aduRxBuffer[uRxWrite], USE_TRANSFER_SIZE);
  usbStartReceiveI(usbp, 3, (uint8_t *)aduRxBuffer, USE_TRANSFER_SIZE_BYTES);
#if ADU_LOGGING  
  aduAddLog(blStartReceive, uCount);
#endif
  palWritePad(GPIOG, 11, 0);
}

typedef struct _OverrunDebug
{
  uint8_t  index;
  uint16_t aduTxRingBufferUsedSize;
  uint16_t sampleOffset;
} __attribute__((packed)) OverrunDebug;

#define OVERRUN_DEBUG_SIZE (1024*2)
OverrunDebug overrunDebug[OVERRUN_DEBUG_SIZE]   __attribute__ ((section (".sram3")));
uint16_t uLogIndex = 0;

void AddLog(uint8_t index)
{
  overrunDebug[uLogIndex].aduTxRingBufferUsedSize = aduTxRingBufferUsedSize;
  overrunDebug[uLogIndex].sampleOffset = aduState.sampleOffset;
  overrunDebug[uLogIndex].index = index;

  uLogIndex++;
  if(uLogIndex == OVERRUN_DEBUG_SIZE)
    uLogIndex = 0;
}

void aduInitiateTransmitI(USBDriver *usbp, size_t uCount)
{
  palWritePad(GPIOD, 5, 1);

  aduState.lastTransferSize = USE_TRANSFER_SIZE_BYTES;
  aduState.currentFrame++;

  AddLog(1);

  // probably change to TX from ring buffer
  // Stage 1 - remove from ring buffer and put in transferbuffer
  if(aduTxRingBufferUsedSize >= USE_TRANSFER_SIZE_SAMPLES)
  {
    uint32_t u; for(u=0; u < USE_TRANSFER_SIZE_SAMPLES; u++)
    {
      aduTxBuffer[u] = aduTxRingBuffer[aduTxRingBufferReadOffset];
      aduTxRingBufferReadOffset = (aduTxRingBufferReadOffset == TX_RING_BUFFER_SIZE) ? 0 : aduTxRingBufferReadOffset+1;
      aduTxRingBufferUsedSize--;
    }
  }
  else
  {
    // underrun
    palWritePad(GPIOB, 4, 1);
    aduState.lastTransferSize  = aduTxRingBufferUsedSize;
    uint32_t u; for(u=0; u < aduState.lastTransferSize; u++)
    {
      aduTxBuffer[u] = aduTxRingBuffer[aduTxRingBufferReadOffset];
      aduTxRingBufferReadOffset = (aduTxRingBufferReadOffset == TX_RING_BUFFER_SIZE) ? 0 : aduTxRingBufferReadOffset+1;
      aduTxRingBufferUsedSize--;
    }
    palWritePad(GPIOB, 4, 0);
  }

  AddLog(2);
  // stage 2 - If we have underrun/overrun adjust counters

  // maybe do this every second, or 100ms or something to get better average
  int16_t nFrameSampleOffest = (int16_t)(aduState.codecFrameSampleCount)-96;
  aduState.codecMetricsSampleOffset += nFrameSampleOffest;

  if(nFrameSampleOffest < 0)
  {
    palWritePad(GPIOB, 4, 1);
    palWritePad(GPIOB, 4, 0);
  }
  else if(nFrameSampleOffest > 0)
  {
    palWritePad(GPIOB, 6, 1);
    palWritePad(GPIOB, 6, 0);
  }

  if(0 == (aduState.currentFrame % CODEC_METICS_MS))
  {
    if(aduState.codecMetricsSampleOffset != 0)
    {
      // ok we are out of sync, adjust to sync over next second
      aduState.sampleOffset    += aduState.codecMetricsSampleOffset;
      aduState.sampleAdjustEveryFrame = CODEC_METICS_MS / ((aduState.sampleOffset>>1)+1);
      aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame;

      palWritePad(GPIOB, 8, 0);
    }

    aduState.codecMetricsSampleOffset = 0;
  }

  // if(nFrameSampleOffest != 0)
  // {
  //   // overrun
  //   static uint32_t uLastOverrun = 0;

  //   palWritePad(GPIOB, 6, 1);
  //   uint32_t uOverrunTime = aduState.currentFrame - aduState.lastOverunFrame;
    
  //   if(uOverrunTime< 200)
  //   {
  //     palWritePad(GPIOC, 7, 1);
  //     palWritePad(GPIOC, 7, 0);
  //     if(uLastOverrun == 1)
  //     {
  //       palWritePad(GPIOC, 7, 1);
  //       palWritePad(GPIOC, 7, 0);
  //     }
  //   }
  //   uLastOverrun = uOverrunTime;

  //   aduState.lastOverunFrame = aduState.currentFrame;
  //   aduState.sampleOffset    += nFrameSampleOffest;
  //   aduState.sampleAdjustEveryFrame = uOverrunTime / ((aduState.sampleOffset>>1)+1);
  //   aduState.sampleAdjustFrameCounter = aduState.sampleAdjustEveryFrame;

  //   palWritePad(GPIOB, 6, 0);
  // }

  // stage 3 - handle overrun/underrun adjustments
  if( aduState.sampleOffset != 0)
  {
    if(aduState.sampleAdjustFrameCounter == 0)
    {
      if(aduState.sampleOffset > 0)
      {
        // adjust overrun
        // chuck samples awway
        palWritePad(GPIOB, 3, 1);
        aduState.sampleOffset-=2;
        aduTxRingBufferUsedSize-=2;
        aduTxRingBufferReadOffset = (aduTxRingBufferReadOffset+2) % TX_RING_BUFFER_SIZE;
        palWritePad(GPIOB, 3, 0);
      }
      else
      {
        // adjust underrun
        // duplicate sample
        palWritePad(GPIOC, 7, 1);
        if(aduTxRingBufferWriteOffset == 0)
        {
          //duplicate from end
          aduTxRingBuffer[aduTxRingBufferWriteOffset]   = aduTxRingBuffer[TX_RING_BUFFER_SIZE-2];
          aduTxRingBuffer[aduTxRingBufferWriteOffset+1] = aduTxRingBuffer[TX_RING_BUFFER_SIZE-1];
        }
        else
        {
          aduTxRingBuffer[aduTxRingBufferWriteOffset]   = aduTxRingBuffer[aduTxRingBufferWriteOffset-2];
          aduTxRingBuffer[aduTxRingBufferWriteOffset+1] = aduTxRingBuffer[aduTxRingBufferWriteOffset-1];
        }
        aduTxRingBufferWriteOffset = (aduTxRingBufferWriteOffset +2) % TX_RING_BUFFER_SIZE;
        aduTxRingBufferUsedSize++;
        aduState.sampleOffset++;
        palWritePad(GPIOC, 7, 0);
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
    palWritePad(GPIOB, 8, 1);
  }
  

  AddLog(3);

  aduState.codecFrameSampleCount = 0;

  usbStartTransmitI(usbp, 3, (uint8_t *)aduTxBuffer, aduState.lastTransferSize );
#if ADU_LOGGING
  aduAddLog(blStartTransmit, USE_TRANSFER_SIZE_BYTES);
#endif
  palWritePad(GPIOD, 5, 0);
}
