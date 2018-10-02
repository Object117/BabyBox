/*
 * stateMachine.c
 *
 *  Created on: 2018. 9. 6.
 *      Author: leejh
 */
#include"stateMachine.h"
#include"usart.h"
#include "device_led.h"
#include "device_relay.h"
#include "device_buzzer.h"

uint8_t TxBuffer1[] = "in doing_ready_state/n";
uint8_t TxBuffer2[] = "in doing_running_state/n";
uint8_t Testing[] = "in Testing/n";

USER_ACTION standby_functions = {STANDBY	,
								STANDBY_inner_door_open	,
								STANDBY_inner_door_close	,
								STANDBY_extdoor_open	,
								STANDBY_extdoor_close	,
								STANDBY_baby_in			,
								STANDBY_baby_none
};

USER_ACTION ready_functions = {READY	,
							READY_inner_door_open	,
							READY_inner_door_close	,
							READY_extdoor_open	,
							READY_extdoor_close	,
							READY_baby_in	,
							READY_baby_none
};

USER_ACTION enter_functions = {ENTER	,
						ENTER_inner_door_open	,
						ENTER_inner_door_close	,
						ENTER_extdoor_open	,
						ENTER_extdoor_close	,
						ENTER_baby_in	,
						ENTER_baby_none
};

USER_ACTION protection_functions = {PROTECTION		,
								PROTECTION_inner_door_open	,
								PROTECTION_inner_door_close		,
								PROTECTION_extdoor_open		,
								PROTECTION_extdoor_close	,
								PROTECTION_baby_in		,
								PROTECTION_baby_none
};

USER_ACTION confirm_functions = {CONFIRM		,
							CONFIRM_inner_door_open	,
							CONFIRM_inner_door_close	,
							CONFIRM_extdoor_open	,
							CONFIRM_extdoor_close	,
							CONFIRM_baby_in			,
							CONFIRM_baby_none
};

USER_ACTION exit_functions = {EXIT		,
							EXIT_inner_door_open	,
							EXIT_inner_door_close	,
							EXIT_extdoor_open	,
							EXIT_extdoor_close	,
							EXIT_baby_in	,
							EXIT_baby_none
};

USER_ACTION emergency_functions = {EMERGENCY	,
							EMER_inner_door_open	,
							EMER_inner_door_close	,
							EMER_extdoor_open	,
							EMER_extdoor_close	,
							EMER_baby_in	,
							EMER_baby_none
};

USER_ACTION recovery_functions = {RECOVERY	,
							RECOVERY_inner_door_open	,
							RECOVERY_inner_door_close	,
							RECOVERY_extdoor_open	,
							RECOVERY_extdoor_close	,
							RECOVERY_baby_in	,
							RECOVERY_baby_none
};

USER_ACTION* tStandby_state = &standby_functions;
USER_ACTION* tReady_state = &ready_functions;
USER_ACTION* tEnter_state = &enter_functions;
USER_ACTION* tProtection_state = &protection_functions;
USER_ACTION* tConfirm_state = &confirm_functions;
USER_ACTION* tExit_state = &exit_functions;
USER_ACTION* tEmergency_state = &emergency_functions;
USER_ACTION* tRecovery_state = &recovery_functions;

USER_ACTION* initialize_state(void) {
	return tStandby_state;
}

USER_ACTION* recovery_state(void) {
	return tRecovery_state;
}

USER_ACTION* change_state(void) {
	USER_ACTION* rtn;
	rtn = changeingState;		// Global value.

	return rtn;
}

/*
 * STANDBY State		(NORMAL)
 * --> Inner Door 'CLOSE'
 * --> Ext Door 'CLOSE'
 * --> Baby 'NONE'
 * __________________________________________
 */

void STANDBY_inner_door_open(void) {
//	printf("State : STANDBY\n");
}

void STANDBY_inner_door_close(void) {
	LED_OFF(RED_LED);
	LED_OFF(GREEN_LED);
	LED_ON(BLUE_LED);
	light_off();
	unlock_door();
	baby_state = BABY_NONE;		// Initialize
	Buzzer_Off(BUZZER_EXTERNAL);
}

void STANDBY_extdoor_open(void) {
	if(extdoor_status == EXT_DOOR_OPEN) {
		changeingState = tReady_state;
	}
}

void STANDBY_extdoor_close(void) {

}

void STANDBY_baby_in(void) {

}

void STANDBY_baby_none(void) {

}

/*
 * READY State		(NORMAL)
 * --> Inner Door 'CLOSE'
 * --> Ext Door 'OPEN'
 * --> Baby 'NONE'
 * __________________________________________
 */
void READY_inner_door_open(void) {
//	printf("State : READY\n");
}

void READY_inner_door_close(void) {

}

void READY_extdoor_open(void) {
	LED_OFF(RED_LED);
	LED_OFF(GREEN_LED);
	LED_ON(BLUE_LED);
	Buzzer_Off(BUZZER_EXTERNAL);
	light_on();
}

void READY_extdoor_close(void) {
	if(extdoor_status == EXT_DOOR_CLOSE) {
		changeingState = tStandby_state;
	}
}

void READY_baby_in(void) {
	if(baby_state == BABY_IN) {
		changeingState = tEnter_state;
	}
}

void READY_baby_none(void) {

}

/*
 * ENTERed Baby in box State		(NORMAL)
 * --> Inner Door 'CLOSE'
 * --> Ext Door 'OPEN'
 * --> Baby 'IN'
 * __________________________________________
 */
void ENTER_inner_door_open(void) {
//	printf("State : ENTER\n");
}

void ENTER_inner_door_close(void) {

}

void ENTER_extdoor_open(void) {

}

void ENTER_extdoor_close(void) {
	LED_OFF(RED_LED);
	LED_ON(GREEN_LED);
	LED_OFF(BLUE_LED);
	Buzzer_Off(BUZZER_EXTERNAL);
	if(extdoor_status == EXT_DOOR_CLOSE) {
		changeingState = tProtection_state;
	}
}

void ENTER_baby_in(void) {

}

void ENTER_baby_none(void) {

}

/*
 * PROTECTION in box State		(NORMAL)
 * --> Inner Door 'CLOSE'
 * --> Ext Door 'CLOSE'
 * --> Baby 'IN'
 * __________________________________________
 */
void PROTECTION_inner_door_open(void) {
//	printf("State : PROTECTION\n");
	LED_OFF(RED_LED);
	LED_ON(GREEN_LED);
	LED_OFF(BLUE_LED);
	baby_state = BABY_NONE;		// Initialize
	Buzzer_Off(BUZZER_EXTERNAL);
	lock_door();
	if(innerdoor_state == INNER_DOOR_OPEN) {
		changeingState = tConfirm_state;
	}
}

void PROTECTION_inner_door_close(void) {

}

void PROTECTION_extdoor_open(void) {
#if 1	// for the 1st TEST - ORIG ACTIVE
	if(extdoor_status == EXT_DOOR_OPEN) {
		changeingState = tEmergency_state;
	}
#endif
}

void PROTECTION_extdoor_close(void) {

}

void PROTECTION_baby_in(void) {

}

void PROTECTION_baby_none(void) {

}

/*
 * CONFIRMation in box State		(NORMAL)
 * --> Inner Door 'OPEN'
 * --> Ext Door 'CLOSE'
 * --> Baby 'IN'
 * __________________________________________
 */
void CONFIRM_inner_door_open(void) {
//	printf("State : CONFIRM\n");
}

void CONFIRM_inner_door_close(void) {
	Buzzer_Off(BUZZER_EXTERNAL);
	if(innerdoor_state == INNER_DOOR_CLOSE) {
		changeingState = tProtection_state;
	}
}

void CONFIRM_extdoor_open(void) {

}

void CONFIRM_extdoor_close(void) {

}

void CONFIRM_baby_in(void) {

}

void CONFIRM_baby_none(void) {

	if(baby_state == BABY_IN) {		// <<---- Should be modify
		changeingState = tExit_state;
	}
}

/*
 * EXIT baby in box State		(NORMAL)
 * --> Inner Door 'OPEN'
 * --> Ext Door 'CLOSE'
 * --> Baby 'NONE'
 * __________________________________________
 */
void EXIT_inner_door_open(void) {
//	printf("State : EXIT\n");
}

void EXIT_inner_door_close(void) {
	LED_OFF(RED_LED);
	LED_OFF(GREEN_LED);
	LED_ON(BLUE_LED);
	Buzzer_Off(BUZZER_EXTERNAL);
	unlock_door();
	baby_state = BABY_NONE;		// Initialize
	if(innerdoor_state == INNER_DOOR_CLOSE) {		// <<---- Should be modify
		changeingState = tStandby_state;
	}
}

void EXIT_extdoor_open(void) {

}

void EXIT_extdoor_close(void) {

}

void EXIT_baby_in(void) {

}

void EXIT_baby_none(void) {

}

/*
 * EMERGENCY State		(EXCEPTION)
 * --> Inner Door 'CLOSE'
 * --> Ext Door 'OPEN'	// FORCE
 * --> Baby 'NONE'		// hijacking??
 * __________________________________________
 */
void EMER_inner_door_open(void) {
//	printf("State : EMERGENCY !!! \n");
}

void EMER_inner_door_close(void) {

}

void EMER_extdoor_open(void) {
	LED_ON(RED_LED);
	LED_OFF(GREEN_LED);
	LED_OFF(BLUE_LED);
	Buzzer_On(BUZZER_EXTERNAL);
	baby_state = BABY_NONE;		// Initialize
}

void EMER_extdoor_close(void) {

}

void EMER_baby_in(void) {

}

void EMER_baby_none(void) {

}

/*
 * RECOVERY State		(EXCEPTION)
 * --> Inner Door 'CLOSE'
 * --> Ext Door 'OPEN'	// will CLOSE
 * --> Baby 'NONE'		//
 * __________________________________________
 */
void RECOVERY_inner_door_open(void) {
//	printf("State : RECOVERY\n");
}

void RECOVERY_inner_door_close(void) {

}

void RECOVERY_extdoor_open(void) {

}

void RECOVERY_extdoor_close(void) {
	LED_ON(RED_LED);
	LED_OFF(GREEN_LED);
	LED_ON(BLUE_LED);
	Buzzer_Off(BUZZER_EXTERNAL);
	unlock_door();
	if(extdoor_status == EXT_DOOR_CLOSE) {
		changeingState = tStandby_state;
	}
}

void RECOVERY_baby_in(void) {

}

void RECOVERY_baby_none(void) {

}
