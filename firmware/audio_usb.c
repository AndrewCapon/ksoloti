
#include "hal.h"
#include "audio_usb.h"

const uint32_t aduSampleRates[] = {44100, 48000};
uint32_t aduCurrentSampleRate  = 44100;

#define N_SAMPLE_RATES  2

bool __attribute__((optimize("O0"))) aduHandleClockRequest(USBDriver *usbp, audio_control_request_t *request)
{
  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_4_t curf = { (int32_t) aduCurrentSampleRate };
      usbSetupTransfer(usbp, &curf, sizeof(curf), NULL);
      return true;
    }
    else if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_4_n_t(N_SAMPLE_RATES) rangef;
      rangef.wNumSubRanges = (uint16_t)N_SAMPLE_RATES;
      uint8_t i;
      for(i = 0; i < N_SAMPLE_RATES; i++)
      {
        rangef.subrange[i].bMin = (int32_t) aduSampleRates[i];
        rangef.subrange[i].bMax = (int32_t) aduSampleRates[i];
        rangef.subrange[i].bRes = 0;
      }
      palWritePad(GPIOG, 11, 1);
      usbSetupTransfer(usbp, &rangef, sizeof(rangef), NULL);
      palWritePad(GPIOG, 11, 0);
      return true;
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    //return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }
  return false;

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
        //return audio_volume_control(usbp, req, (wValue >> 8) & 0xFF, wValue & 0xFF, length);
        volatile int bp = 1;
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

bool aduSwitchInterface(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req, uint16_t wValue, uint16_t length) 
{
  volatile int bp = 1;
  return false;
}
