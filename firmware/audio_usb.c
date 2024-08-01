
#include "hal.h"
#include "audio_usb.h"

const uint32_t aduSampleRates[] = {44100, 48000, 96000};
uint32_t aduCurrentSampleRate  = 44100;

#define N_SAMPLE_RATES  3

#define LOG_AMOUNT 128
uint32_t uLogCount = 0;

typedef struct request
{
  uint8_t bControlSelector;
  uint8_t bRequest;
} request;

audio_control_request_t requests[LOG_AMOUNT];

static uint8_t aduControlData[8];
static uint8_t aduControlChannel;

// Set the sample rate
static void aduSetSampleRate(USBDriver *usbp) 
{
  aduCurrentSampleRate = (uint32_t) ((audio_control_cur_4_t const *)aduControlData)->bCur;
}


bool __attribute__((optimize("O0"))) aduHandleVolumeRequest(USBDriver *usbp, audio_control_request_t *request)
{
  // if(uLogCount < LOG_AMOUNT)
  //   memcpy(&requests[uLogCount++], request, sizeof(audio_control_request_t));
  // else
  //   uLogCount = 0;

  return false;
}

bool __attribute__((optimize("O0"))) aduHandleClockRequest(USBDriver *usbp, audio_control_request_t *request)
{
  bool bResult = false;

  if(uLogCount < LOG_AMOUNT)
    memcpy(&requests[uLogCount++], request, sizeof(audio_control_request_t));
  else
    uLogCount = 0;

  palWritePad(GPIOG, 11, 1);

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if(request->bmRequestType_bit.direction) // Get requests
    {
      switch(request->bRequest)
      {
        case AUDIO_CS_REQ_CUR:
        {
          // get current sample rate
          audio_control_cur_4_t curf = { (int32_t) aduCurrentSampleRate };
          usbSetupTransfer(usbp, &curf, sizeof(curf), NULL);
          bResult = true;
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
    usbSetupTransfer(usbp, &cur_valid, sizeof(cur_valid), NULL);
    bResult = true;
  }

  palWritePad(GPIOG, 11, 0);

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
  if(entity == 0)
  {
    if(iface = ITF_NUM_AUDIO_STREAMING_SPEAKER)
    {
      if(wValue = 0x0001)
      {
        // start
        usbSetupTransfer(usbp, NULL, 0, NULL);
        bResult = true;
      }
      else
      {
        // end
        usbSetupTransfer(usbp, NULL, 0, NULL);
        bResult = true;
      }
    }
  }

  return bResult;
}
