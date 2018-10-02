/*
 * stateMachine.h
 *
 *  Created on: 2018. 9. 6.
 *      Author: leejh
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

typedef enum {
	STANDBY = 0	,
	READY		,
	ENTER		,
	PROTECTION	,
	CONFIRM		,
	EXIT		,
	EMERGENCY	,
	RECOVERY	,
	NONE_STATE
} MACHINE_STATE;

typedef enum {
	EXT_DOOR_CLOSE = 0	,
	EXT_DOOR_OPEN
} EXT_DOOR_ACTION;

typedef enum {
	INNER_DOOR_CLOSE = 0	,
	INNER_DOOR_OPEN
}INNER_DOOR_ACTION;

typedef enum {
	BABY_NONE = 0	,
	BABY_IN
}BABY_ACTION;

typedef struct action {
	MACHINE_STATE currentState;
	void (*inner_door_open)(void);
	void (*inner_door_close)(void);
	void (*extdoor_open)(void);
	void (*extdoor_close)(void);
	void (*baby_in)(void);
	void (*baby_none)(void);
} USER_ACTION;

EXT_DOOR_ACTION extdoor_status;
INNER_DOOR_ACTION innerdoor_state;
BABY_ACTION baby_state;

USER_ACTION* changeingState;

USER_ACTION* initialize_state(void);
USER_ACTION* recovery_state(void);
USER_ACTION* change_state();

void STANDBY_inner_door_open(void);
void STANDBY_inner_door_close(void);
void STANDBY_extdoor_open(void);
void STANDBY_extdoor_close(void);
void STANDBY_baby_in(void);
void STANDBY_baby_none(void);

void READY_inner_door_open(void);
void READY_inner_door_close(void);
void READY_extdoor_open(void);
void READY_extdoor_close(void);
void READY_baby_in(void);
void READY_baby_none(void);

void ENTER_inner_door_open(void);
void ENTER_inner_door_close(void);
void ENTER_extdoor_open(void);
void ENTER_extdoor_close(void);
void ENTER_baby_in(void);
void ENTER_baby_none(void);

void PROTECTION_inner_door_open(void);
void PROTECTION_inner_door_close(void);
void PROTECTION_extdoor_open(void);
void PROTECTION_extdoor_close(void);
void PROTECTION_baby_in(void);
void PROTECTION_baby_none(void);

void CONFIRM_inner_door_open(void);
void CONFIRM_inner_door_close(void);
void CONFIRM_extdoor_open(void);
void CONFIRM_extdoor_close(void);
void CONFIRM_baby_in(void);
void CONFIRM_baby_none(void);

void EXIT_inner_door_open(void);
void EXIT_inner_door_close(void);
void EXIT_extdoor_open(void);
void EXIT_extdoor_close(void);
void EXIT_baby_in(void);
void EXIT_baby_none(void);

void EMER_inner_door_open(void);
void EMER_inner_door_close(void);
void EMER_extdoor_open(void);
void EMER_extdoor_close(void);
void EMER_baby_in(void);
void EMER_baby_none(void);

void RECOVERY_inner_door_open(void);
void RECOVERY_inner_door_close(void);
void RECOVERY_extdoor_open(void);
void RECOVERY_extdoor_close(void);
void RECOVERY_baby_in(void);
void RECOVERY_baby_none(void);
#endif /* STATEMACHINE_H_ */
