#ifndef _PROTOCOL_ACURITE_H_
#define _PROTOCOL_ACURITE_H_

#include "../protocol.h"

#define C2F(c) (c * 1.8 + 32)
#define F2C(f) ((f - 32 ) * 5 / 9)

#define logprintf(prio, args...) \
    {                            \
        printf(args);            \
    }
#define logprintfLn(prio, args...) \
    {                              \
        printf(args);              \
        printf("\n");              \
    }

struct protocol_t *acurite;
void acuriteInit(void);

#endif