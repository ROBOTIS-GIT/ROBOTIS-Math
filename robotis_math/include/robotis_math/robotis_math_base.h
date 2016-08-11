/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * robotis_math_base.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef ROBOTIS_MATH_ROBOTIS_MATH_BASE_H_
#define ROBOTIS_MATH_ROBOTIS_MATH_BASE_H_

#include <cmath>

namespace robotis_framework
{

#define PRINT_VAR(X) std::cout << #X << " : " << X << std::endl
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

#define DEGREE2RADIAN (M_PI / 180.0)
#define RADIAN2DEGREE (180.0 / M_PI)

inline double powDI(double a, int b)
{
	return (b == 0 ? 1 : (b > 0 ? a * powDI(a, b - 1) : 1 / powDI(a, -b)));
}

double sign(double x);

}



#endif /* ROBOTIS_MATH_ROBOTIS_MATH_BASE_H_ */
