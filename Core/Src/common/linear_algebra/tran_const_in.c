//
// Created by Michal on 10.11.2022.
//

#include "linear_algebra.h"

/*
 * Take the transpose of a matrix A, with the dimension row x column. The result will be A' with the dimension column x row
 */
void tran_const_in(float* A, float* A_t, int row, int column) {

    float* transpose;
    float* ptr_A = A;

    for (int i = 0; i < row; i++) {
        transpose = &A_t[i];
        for (int j = 0; j < column; j++) {
            *transpose = *ptr_A;
            ptr_A++;
            transpose += row;
        }
    }
}