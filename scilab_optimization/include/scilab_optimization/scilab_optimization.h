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
 * scilab_optimization.h
 *
 *  Created on: December 1, 2016
 *      Author: sch
 */

#ifndef SCILAB_OPTIMIZATION_H
#define SCILAB_OPTIMIZATION_H

#include <ros/ros.h>

#include <cmath>

#include <stdio.h>

// scilab
#include <scilab/stack-c.h>
#include <scilab/call_scilab.h>
#include <scilab/api_scilab.h>

#define SCILIB_PATH "/usr/share/scilab"

namespace robotis_framework
{

class ScilabOptimization
{
public:
  ScilabOptimization();
  ~ScilabOptimization();

  static void initialize();
  static void solveRiccatiEquation(double *K, int *row_K, int *colK,
                                   double *S, int *rowS, int *colS,
                                   double *E, double *E_img, int *rowE, int *colE,
                                   double *A, int rowA,  int colA,
                                   double *B, int rowB,  int colB,
                                   double *Q, int rowQ,  int colQ,
                                   double *R, int rowR,  int colR);

  static void terminate();


private:



};



}

#endif // SCILAB_OPTIMIZATION_H
