#ifdef __cplusplus
extern "C" {
#endif
#ifndef INTENCODER_H_
#define INTENCODER_H_ 

#include "config.h"

#define HIGH 1
#define LOW  0

void encoder_init(Encoder_TypeDef encoder);
extern int en_pos1;
extern int en_pos2;
extern int en_pos3;
extern int en_pos4;
#endif //

#ifdef __cplusplus
}
#endif
