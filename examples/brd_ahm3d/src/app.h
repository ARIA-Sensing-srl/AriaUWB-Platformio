/**
 * @file file app.h
 * @brief Application
 *
 *  Created on: Jan 23, 2024\n
 *  Author: ARIA Sensing\n
 *  Copyright: ARIA Sensing, 2023 - \n
 */


#ifndef APP_H_
#define APP_H_
#include "hydrogen_UD_v1.h"

#define DEF_APP_SERIAL_BAUD (921600UL)

/**
 * Application entry point
 * @param h handler to radar device
 */
int app_entry(HydrUDriver_t* h);


#endif /* APP_H_ */
