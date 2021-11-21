#include "main.h"
#include "cmsis_os.h"


typedef enum {
	DIRECTION_CCW = 0,
	DIRECTION_CW = 1
} Direction;

typedef struct {
	long currentPos;
	long targetPos;
	float speed;
	float max_speed;
	float acceleration;
	long stepInverval;
	long n;
	float c0;
	float cn;
	float cmin;
	Direction direction;

} stepper_t;



void stepperInit(stepper_t* stepper);
void stepperSetSpeed(stepper_t* stepper, float speed);
float stepperGetSpeed(stepper_t* stepper);

void stepperSetMaxSpeed(stepper_t* stepper, float speed);
float stepperGetMaxSpeed(stepper_t* stepper);
void stepperSetAcceleration(stepper_t* stepper, float acceleration);

long stepperGetDistanceToGo(stepper_t* stepper);
void stepperSetCurrentPosition(stepper_t* stepper, long currentPosition);
void stepperRun(stepper_t* stepper);
void stepperStop(stepper_t* stepper);
long stepperGetTargetPosition(stepper_t* stepper);
void stepperSetAbsoluteTartePosition(stepper_t* stepper, long absolutePosition);
void stepperSetRelativeTargetPosition(stepper_t* stepper, long relativePosition);
void stepperComputeNewSpeed(stepper_t* stepper);
