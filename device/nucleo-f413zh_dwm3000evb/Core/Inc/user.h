/*
 * user.h
 *
 *  Created on: May 22, 2023
 *      Author: Johannes Arenander
 */

/*————————————————————————————————————————————————————————————*/

#ifndef INC_USER_H_
#define INC_USER_H_

/*————————————————————————————————————————————————————————————*/

#define USER_NEWLINE "\r\n"
#define USER_MODE_ANCHOR 0
#define USER_MODE_TAG 1

/*————————————————————————————————————————————————————————————*/

extern char *userPollFrame;
extern char *userResponseFrame;
extern unsigned long elapsedTimeMilliseconds;

/*————————————————————————————————————————————————————————————*/

void userButtonInterruptHandler(void);
void userDisableInterrupts(void);
void userDisableLogging(void);
void userEnableLogging(void);
void userLog(char *);
void userSetLed(int, int);
void userTimerInterruptHandler(void);
void userToggleLed(int);
int userInitializeDevice(void);
int userConfigureDevice(int);
int userCalibrateDevice(double);
int userPoll(double *);
int userRespond(void);

/*————————————————————————————————————————————————————————————*/

#endif /* INC_USER_H_ */
