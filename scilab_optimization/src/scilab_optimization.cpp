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
 * scilab_optimization.cpp
 *
 *  Created on: December 1, 2016
 *      Author: sch
 */

#include "scilab_optimization/scilab_optimization.h"

namespace robotis_framework
{

ScilabOptimization::ScilabOptimization()
{

}

ScilabOptimization::~ScilabOptimization()
{

}


void ScilabOptimization::initialize()
{
#ifdef _MSC_VER
  if ( StartScilab(NULL,NULL,NULL) == FALSE )
#else
  if ( StartScilab(SCILIB_PATH,NULL,NULL) == FALSE )
#endif
  {
    ROS_WARN("Error while calling StartScilab");
  }
}

void ScilabOptimization::terminate()
{
  if ( TerminateScilab(NULL) == FALSE )
  {
    fprintf(stderr,"Error while calling TerminateScilab\n");
    return;
  }
  return;
}

void ScilabOptimization::solveRiccatiEquation(double *K, int *row_K, int *col_K,
                                              double *S, int *row_S, int *col_S,
                                              double *E, double *E_img, int *row_E, int *col_E,
                                              double *A, int row_A, int col_A,
                                              double *B, int row_B, int col_B,
                                              double *Q, int row_Q, int col_Q,
                                              double *R, int row_R, int col_R)
{
  /*
    This function calculates the optimal gain matrix K
    such that the state-feedback law  u[n] = -Kx[n]  minimizes the
    cost function

          J = Sum {x'Qx + u'Ru}

    subject to the state dynamics   x[n+1] = A*x[n] + B*u[n].

    Also calculated are the
    Riccati equation solution S and the closed-loop eigenvalues E:
                                -1
     A'SA - S - (A'SB+N)(R+B'SB) (B'SA+N') + Q = 0,   E = EIG(A-B*K).
  */

  SciErr sciErr;
  char* job;

  /****** CALCULATION ******/
  char variable_name_matrix_A[] = "A";
  char variable_name_matrix_B[] = "B";
  char variable_name_matrix_Q[] = "Q";
  char variable_name_matrix_R[] = "R";

  // Matrix A
  sciErr = createNamedMatrixOfDouble(pvApiCtx,variable_name_matrix_A,row_A,col_A, A); /* pvApiCtx is a global variable */
  if(sciErr.iErr)
    printError(&sciErr, 0);

  // Matrix B
  sciErr = createNamedMatrixOfDouble(pvApiCtx,variable_name_matrix_B,row_B,col_B, B); /* pvApiCtx is a global variable */
  if(sciErr.iErr)
    printError(&sciErr, 0);

  // Matrix Q
  sciErr = createNamedMatrixOfDouble(pvApiCtx,variable_name_matrix_Q,row_Q,col_Q, Q); /* pvApiCtx is a global variable */
  if(sciErr.iErr)
    printError(&sciErr, 0);

  // Matrix R
  sciErr = createNamedMatrixOfDouble(pvApiCtx,variable_name_matrix_R,row_R,col_R, R); /* pvApiCtx is a global variable */
  if(sciErr.iErr)
    printError(&sciErr, 0);

  // Matrix b
  job = "b = B / R * B'";
  SendScilabJob(job);

  // Matrix S
  job = "S = riccati(A,b,Q,'d','eigen')";
  SendScilabJob(job);
  //  SendScilabJob("disp(S);");

  // Matrix K
  job = "K = inv(B'*S*B+R)*(B'*S*A)";
  SendScilabJob(job);
  //  SendScilabJob("disp(K);");

  // eigenvalues E
  job = "E = spec(A-B*K)";
  SendScilabJob(job);
  //  SendScilabJob("disp(E);");

  // Read Matrix K
  int row = 0, col = 0;
  char variable_to_be_retrieved_K[] = "K";

  sciErr = readNamedMatrixOfDouble(pvApiCtx, variable_to_be_retrieved_K, &row, &col, NULL);
  if(sciErr.iErr)
    printError(&sciErr, 0);

  K = (double *)realloc(K,(row*col)*sizeof(double));

  sciErr = readNamedMatrixOfDouble(pvApiCtx, variable_to_be_retrieved_K, &row, &col, K);
  if(sciErr.iErr)
    printError(&sciErr, 0);

  *row_K = row;
  *col_K = col;

  // Read Matrix S
  char variable_to_be_retrieved_S[] = "S";

  sciErr = readNamedMatrixOfDouble(pvApiCtx, variable_to_be_retrieved_S, &row, &col, NULL);
  if(sciErr.iErr)
    printError(&sciErr, 0);

  S = (double *)realloc(S,(row*col)*sizeof(double));

  sciErr = readNamedMatrixOfDouble(pvApiCtx, variable_to_be_retrieved_S, &row, &col, S);
  if(sciErr.iErr)
    printError(&sciErr, 0);

  *row_S = row;
  *col_S = col;

  // Read Matrix E
  char variable_to_be_retrieved_E[] = "E";

  sciErr = readNamedMatrixOfDouble(pvApiCtx, variable_to_be_retrieved_E, &row, &col, NULL);
  if(sciErr.iErr)
    printError(&sciErr, 0);

  E = (double *)realloc(E,(row*col)*sizeof(double));
  E_img = (double *)realloc(E_img,(row*col)*sizeof(double));

  sciErr = readNamedComplexMatrixOfDouble(pvApiCtx, variable_to_be_retrieved_E, &row, &col, E, E_img);
  if(sciErr.iErr)
    printError(&sciErr, 0);

  *row_E = row;
  *col_E = col;

  return;
}


}
