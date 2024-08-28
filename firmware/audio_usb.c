
#include "hal.h"
#include "audio_usb.h"
#include "usb_lld.h"
#include "chevents.h"

#define USE_TRANSFER_SIZE 176

extern AudioUSBDriver ADU1;

const uint32_t aduSampleRates[] = {44100, 48000, 96000};

#define N_SAMPLE_RATES  3

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

static uint16_t aduBuffer[AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));
static uint16_t aduBuffer2[AUDIO_USB_BUFFERS_SIZE + AUDIO_MAX_PACKET_SIZE]  __attribute__ ((section (".sram3")));


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

  palWritePad(GPIOG, 11, 1);

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

  if(bResult)
  palWritePad(GPIOG, 11, 0);

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
          //palWritePad(GPIOG, 11, 1);
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

  //palWritePad(GPIOG, 11, 0);

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

void aduEnableInput(USBDriver *usbp, bool bEnable)
{
  // this is ksoloti->host
  if(bEnable != aduState.isInputActive)
  {
    aduState.isInputActive = bEnable;
    if(bEnable)
    {
      palWritePad(GPIOG, 11, 1);
      //usbPrepareTransmit(usbp, 3, (uint8_t *)aduBuffer2, AUDIO_MAX_PACKET_SIZE);
      //usbPrepareTransmit(usbp, 3, (uint8_t *)aduBuffer2, USE_TRANSFER_SIZE);
      usbPrepareTransmit(usbp, 3, NULL, 0);
      chSysLockFromIsr();
      chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_INPUT);
      usbStartTransmitI(usbp, 3);
      chSysUnlockFromIsr();
      palWritePad(GPIOG, 11, 0);
    }
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
      palWritePad(GPIOG, 11, 1);
      //usbPrepareReceive(usbp, 3, (uint8_t *)aduBuffer, AUDIO_MAX_PACKET_SIZE);
      usbPrepareReceive(usbp, 3, (uint8_t *)aduBuffer, USE_TRANSFER_SIZE);
      //usbPrepareReceive(usbp, 3, NULL, 0);
      chSysLockFromIsr();
      chEvtBroadcastFlagsI(&ADU1.event, AUDIO_EVENT_OUTPUT);
      usbStartReceiveI(usbp, 3);
      chSysUnlockFromIsr();
      palWritePad(GPIOG, 11, 0);
    }
  }
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
  aduState.currentSampleRate = 44100;
  
  // default is disabled
  aduState.isOutputActive = false;
  aduState.isInputActive = false;
  
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
  palWritePad(GPIOG, 11, 1);
  //usbPrepareTransmit(usbp, 3, (uint8_t *)aduBuffer2, AUDIO_MAX_PACKET_SIZE);
  //usbPrepareTransmit(usbp, 3, (uint8_t *)aduBuffer2, USE_TRANSFER_SIZE);
  usbPrepareTransmit(usbp, 3, NULL, 0);
  chSysLockFromIsr();
  usbStartTransmitI(usbp, 3);
  chSysUnlockFromIsr();
  palWritePad(GPIOG, 11, 0);
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
  palWritePad(GPIOG, 11, 1);
  //usbPrepareReceive(usbp, 3, (uint8_t *)aduBuffer, AUDIO_MAX_PACKET_SIZE);
  //usbPrepareReceive(usbp, 3, (uint8_t *)aduBuffer, USE_TRANSFER_SIZE);
  usbPrepareReceive(usbp, 3, NULL, 0);
  chSysLockFromIsr();
  usbStartReceiveI(usbp, 3);
  chSysUnlockFromIsr();
  palWritePad(GPIOG, 11, 0);
}
