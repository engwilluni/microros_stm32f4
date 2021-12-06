#include "stepper.h"


void stepperInit(stepper_t* stepper){
	stepper->currentPos = 0;
	stepper->targetPos = 0;
	stepper->speed = 0.0;
	stepper->max_speed = 2000.0;
	stepper->acceleration = 0.0;
	stepper->stepInverval = 0;
	stepper->n = 0;
	stepper->c0 = 0.0;
	stepper->cn = 0.0;
	stepper->cmin = 1.0;
	HAL_GPIO_WritePin(STEPPER_EN_GPIO_Port, STEPPER_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
}

void stepperSetSpeed(stepper_t* stepper, float speed){
	if (speed == stepper->speed) {
		return;
	}
	if (speed < -stepper->max_speed){
		speed = -stepper->max_speed;
	}
	else if (speed > stepper->max_speed) {
		speed = stepper->max_speed;
	}

	if (speed == 0.0){
		stepper->stepInverval = 0;
	}
	else {
		stepper->stepInverval = fabs(1000000.0/speed);
		stepper->direction = (speed>0.0)? DIRECTION_CW : DIRECTION_CCW;
	}
	stepper->speed = speed;
}


float stepperGetSpeed(stepper_t* stepper){
	return stepper->speed;
}


void stepperSetMaxSpeed(stepper_t* stepper, float speed){
	if (speed < 0.0){
		speed = -speed;
	}
	if (stepper->max_speed != speed){
		stepper->max_speed = speed;
		stepper->cmin = 1000000.0/speed;
		if (stepper->n > 0){
			stepper->n = (long)((stepper->speed * stepper->speed)/ (2.0 * stepper->acceleration));
			stepperComputeNewSpeed(stepper);
		}
	}

}

float stepperGetMaxSpeed(stepper_t* stepper){
	return stepper->max_speed;
}

void stepperSetAcceleration(stepper_t* stepper, float acceleration){
	if (acceleration == 0.0)
		return;
	if (acceleration < 0.0){
		acceleration = - acceleration;
	}
	if (stepper->acceleration != acceleration){
		stepper->n = stepper->n * (stepper->acceleration/acceleration);
		stepper->c0 = 0.676* sqrt(2.0 / acceleration) * 1000000.0;
		stepper->acceleration = acceleration;
		stepperComputeNewSpeed(stepper);
	}

}

long stepperGetDistanceToGo(stepper_t* stepper){
	return stepper->targetPos - stepper->currentPos;
}

void stepperSetCurrentPosition(stepper_t* stepper, long currentPosition){
	stepper->currentPos = currentPosition;
	stepper->targetPos = currentPosition;
	stepper->n = 0;
	stepper->stepInverval = 0;
	stepper->speed = 0.0;
}

void stepperRun(stepper_t* stepper){

}

void stepperStop(stepper_t* stepper){

}

long stepperGetTargetPosition(stepper_t* stepper){
	return stepper->targetPos;
}

void stepperSetAbsoluteTartePosition(stepper_t* stepper, long absolutePosition){
	stepper->targetPos = absolutePosition;
}

void stepperSetRelativeTargetPosition(stepper_t* stepper, long relativePosition){
	stepper->targetPos = stepperGetTargetPosition(stepper) + relativePosition;
}

void stepperComputeNewSpeed(stepper_t* stepper){
	long distanceTo = stepperGetDistanceToGo(stepper);
	long stepsToStop = (long)((stepper->speed * stepper->speed) / (2.0 * stepper->acceleration));

	if (distanceTo == 0 && stepsToStop <=1)
	{
		stepper->stepInverval = 0;
		stepper->speed = 0.0;
		stepper->n = 0;
		return;
	}
	if (distanceTo > 0) {
		if (stepper->n > 0){
			if ((stepsToStop >= distanceTo) || (stepper->direction == DIRECTION_CCW)){
				stepper->n = -stepsToStop;
			}

		}
		else if (stepper->n < 0 ){
			if ((stepsToStop < distanceTo) && stepper->direction == DIRECTION_CW){
				stepper->n = - stepper->n;
			}
		}
	}
	else if (distanceTo < 0) {
		if (stepper->n > 0){
			if ((stepsToStop >= -distanceTo) || stepper->direction == DIRECTION_CW){
				stepper->n = -stepsToStop;
			}
		}
		else if (stepper->n < 0){
			if ((stepsToStop < - distanceTo) && stepper->direction == DIRECTION_CCW){
				stepper->n = -stepper->n;
			}
		}
	}
	if (stepper->n == 0){
		stepper->cn = stepper->c0;
		stepper->direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	else {
		stepper->cn = stepper->cn - ((2.0 * stepper->cn)/((4.0* stepper->n)+1));
		if (stepper->cn > stepper->cmin){
			stepper->cn = stepper->cn;
		}
		else {
			stepper->cn = stepper->cmin;
		}

	}
	stepper->n++;
	stepper->stepInverval = stepper->cn;
	stepper->speed = 1000000.0/stepper->cn;
	if (stepper->direction == DIRECTION_CCW){
		stepper->speed = -stepper->speed;
	}
}
