#ifndef COM_ESTIMATION_H
#define COM_ESTIMATION_H

#include "BasicLinearAlgebra.h"
using namespace BLA;
#include "MEKF.h"
#include"Filter.h"
// Initialize module

void CoM_Estimation_Innit();

void CoM_estimation_Update(Quaternion q_est, Matrix<3,1,float> w_est,Matrix<3,1,float> weights_pos);

Matrix<3,1,float> CoM_estimation_GET_acc_est();
Matrix<3,1,float> CoM_estimation_GET_N_est();
Matrix<3,1,float> CoM_estimation_GET_CoM_est();


#endif