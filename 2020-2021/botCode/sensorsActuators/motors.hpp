#ifndef MOTORS_H
#define MOTORS_H

#include "./sa_interface/motor.hpp"

/* PacBot specific motors */
class Motors{
	public:
		Motors();
		Motor* right;
		Motor* left;
};
#endif