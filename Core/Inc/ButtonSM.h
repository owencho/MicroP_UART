/*
 * ButtonSM.h
 *
 *  Created on: Aug 20, 2020
 *      Author: academic
 */

#ifndef INC_BUTTONSM_H_
#define INC_BUTTONSM_H_

typedef enum{
    BUTTON_WAIT,
    READ_ADC,
    WAIT_ADC_VALUE,
    SEND_CONTROL_LED,
    SEND_STRING,
} BlinkyState;

void handleButtonSM();

#endif /* INC_BUTTONSM_H_ */
