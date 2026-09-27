#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include <stdint.h>

void AppChassisInit(void);
void AppChassisRun(float dt, uint64_t time_stamp);

#endif // !__APP_CHASSIS_H
