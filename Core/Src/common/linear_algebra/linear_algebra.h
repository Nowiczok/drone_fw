//
// Created by Michał on 08.11.2022.
//

#ifndef DRONE_CONTROLLER_FW_LINEAR_ALGEBRA_H
#define DRONE_CONTROLLER_FW_LINEAR_ALGEBRA_H

#include <stdbool.h>
#include <string.h>

void mul(float* A, float* B, bool elementWise, float* C, int row_a, int column_a, int column_b);
void tran(float* A, int row, int column);
void tran_const_in(float* A, float* A_t, int row, int column);
void add(float* A, float* B, float* C, int row_a, int column_a, int column_b);
void inv_2x2(const float* A, float* A_inv);
void sub(float* A, float* B, float* C, int row_a, int column_a, int column_b);

#endif //DRONE_CONTROLLER_FW_LINEAR_ALGEBRA_H
