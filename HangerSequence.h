/*
 * HangerSequence.h
 *
 *  Created on: Apr 25, 2016
 *      Author: Jacob
 */

#ifndef HANGERSEQUENCE_H_
#define HANGERSEQUENCE_H_
#include "RobotSequence.h"

class HangerSequence : public RobotSequence{
public:
	HangerSequence();
	virtual ~HangerSequence();

	void Run();
};

#endif /* HANGERSEQUENCE_H_ */
