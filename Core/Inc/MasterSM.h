/*
 * ButtonSM.h
 *
 *  Created on: Aug 20, 2020
 *      Author: academic
 */

#ifndef INC_MASTERSM_H_
#define INC_MASTERSM_H_

typedef enum{
    BUTTON_WAIT,
    READ_ADC,
    WAIT_ADC_VALUE,
    SEND_CONTROL_LED,
    SEND_STRING,
} MasterState;

void handleMasterSM();

#endif /* INC_MASTERSM_H_ */
