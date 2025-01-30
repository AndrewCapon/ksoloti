#pragma once

#define ANALYSE_ENABLE 0

#if ANALYSE_ENABLE

#define Analyse(port, pad, bit)                                    \
  palWritePort(port, (palReadLatch(port) & ~PAL_PORT_BIT(pad)) |   \
                     (((bit) & 1U) << pad))
#else
#define Analyse(port, pad, bit)             
#endif