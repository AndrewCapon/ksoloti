#pragma once

#define AVG_SIZE 100

typedef struct _moving_average_data
{
  float    average;
  float    data[AVG_SIZE];
  uint16_t idx;
  uint16_t count;
  bool     alwaysAccurate;
} moving_average_data;

void ma_init(moving_average_data *ma, bool alwaysAccurate)
{
  ma->average = 0.0f;
  ma->idx = 0;
  ma->count = 0;
  ma->alwaysAccurate = alwaysAccurate;

  uint16_t u; for(u=0; u < AVG_SIZE; u++)
    ma->data[u] = 0.0f;
}

void ma_add(moving_average_data *ma, float fValue)
{
  ma->data[ma->idx] = fValue;
  ma->idx = (ma->idx+1) % AVG_SIZE;
  ma->count ++;
  if(ma->alwaysAccurate && ma->count < AVG_SIZE)
  {
    ma->average = 0;
    uint32_t u; for (u=0; u < ma->count; u++)
      ma->average += ma->data[u]/AVG_SIZE;
  }
  else
  {
    float fRemoved = ma->data[ma->idx];
    ma->average = ma->average - (fRemoved / AVG_SIZE) + (fValue / AVG_SIZE);
  }
}

void ma_add_all(moving_average_data *ma, float fValue)
{
  ma->average = fValue;
  ma->count = AVG_SIZE;
  ma->idx = 0;
  uint32_t u; for (u=0; u < ma->count; u++)
    ma->data[u] = fValue;
}

float ma_average(moving_average_data *ma)
{
  return ma->average;
}

