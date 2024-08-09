
#include "hal.h"
#include "audio_usb.h"

const uint32_t aduSampleRates[] = {44100, 48000, 96000};
uint32_t aduCurrentSampleRate  = 44100;

#define N_SAMPLE_RATES  3

static uint8_t aduControlData[8];
static uint8_t aduControlChannel;


#if CONTORL_MESSAGES_DEBUG
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

#define ADU_AUDIO_CHANNELS 2

int8_t  aduMute[ADU_AUDIO_CHANNELS + 1]   = {0, 0, 0};    // +1 for master channel 0
int16_t aduVolume[ADU_AUDIO_CHANNELS + 1] = {VOLUME_CTRL_0_DB, VOLUME_CTRL_0_DB, VOLUME_CTRL_0_DB};    // +1 for master channel 0

// Set the sample rate
static void aduSetSampleRate(USBDriver *usbp) 
{
  audio_control_cur_4_t const *pData = (audio_control_cur_4_t const *)&aduControlData[0];
  uint32_t uSampleRate = (uint32_t) pData->bCur;

#if CONTORL_MESSAGES_DEBUG
  aduAddSampleRateRequest(0, uSampleRate);
#endif

  if(uSampleRate == 44100 || uSampleRate == 48000 || uSampleRate == 96000)
    aduCurrentSampleRate =  uSampleRate;

#if CONTORL_MESSAGES_DEBUG
  else
  {
    badRequests[uBadRequestsCount++] = uLogCount-1;
    // this should really not be happening!
  }
#endif
}

// Set mute
static void aduSetMute(USBDriver *usbp) 
{
  aduMute[aduControlChannel] = ((audio_control_cur_1_t const *)aduControlData)->bCur;
}

// Set mute
static void aduSetVolume(USBDriver *usbp) 
{
  aduVolume[aduControlChannel] = ((audio_control_cur_2_t const *)aduControlData)->bCur;
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
      audio_control_cur_1_t mute1 = { .bCur = aduMute[request->bChannelNumber] };
      usbSetupTransfer(usbp, &mute1, sizeof(mute1), NULL);
      bResult = true;
    }
    else // Set Requests
    {
      aduControlChannel = request->bChannelNumber;
      usbSetupTransfer(usbp, aduControlData, request->wLength, aduSetMute);
      bResult = true;
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
          usbSetupTransfer(usbp, &rangeVol, sizeof(rangeVol), NULL);
          bResult = true;
          break;
        }

        case AUDIO_CS_REQ_CUR:
        {
          audio_control_cur_2_t curVol = { .bCur = aduVolume[request->bChannelNumber] };
          usbSetupTransfer(usbp, &curVol, sizeof(curVol), NULL);
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
          break;
        }

        default: break;
      }
    }
  } 

  palWritePad(GPIOG, 11, 0);

  return bResult;
}

bool __attribute__((optimize("O0"))) aduHandleClockRequest(USBDriver *usbp, audio_control_request_t *request)
{
  bool bResult = false;

#if CONTORL_MESSAGES_DEBUG
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
#if CONTORL_MESSAGES_DEBUG
          // get current sample rate
          aduAddSampleRateRequest(1, aduCurrentSampleRate);
#endif
          audio_control_cur_4_t curf = { (int32_t) aduCurrentSampleRate };
          usbSetupTransfer(usbp, &curf, sizeof(curf), NULL);
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
          usbSetupTransfer(usbp, &rangef, sizeof(rangef), NULL);
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
    usbSetupTransfer(usbp, &cur_valid, sizeof(cur_valid), NULL);
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

// bool aduControl(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req, uint16_t wValue, uint16_t length) 
// {
//   /* Only requests to audio control iface are supported */
//   if (iface == ITF_NUM_AUDIO_STREAMING_CONTROL) 
//   {
//     switch(entity)
//     {
//       case AUDIO_FUNCTION_UNIT_ID:
//       {
//         //return audio_volume_control(usbp, req, (wValue >> 8) & 0xFF, wValue & 0xFF, length);
//         volatile int bp = 1;
//         break;
//       }

//       case UAC2_ENTITY_CLOCK:
//       {
//         // ok we need to handle returning the clock
//         volatile int bp = 1;
//         break;
//       }
//       default: break;
//     }
//   }
//   return false;
// }
//                                       4              5               1            (3 << 8) | 2     6
bool aduSwitchInterface(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req, uint16_t wValue, uint16_t length) 
{
  bool bResult = false;

  usbSetupTransfer(usbp, NULL, 0, NULL);
  bResult = true;

  // if(entity == 0)
  // {
  //   if(iface == ITF_NUM_AUDIO_STREAMING_SPEAKER)
  //   {
  //     if(wValue == 0x0001)
  //     {
  //       // start
  //       usbSetupTransfer(usbp, NULL, 0, NULL);
  //       bResult = true;
  //     }
  //     else
  //     {
  //       // end
  //       usbSetupTransfer(usbp, NULL, 0, NULL);
  //       bResult = true;
  //     }
  //   }
  // }

  return bResult;
}
