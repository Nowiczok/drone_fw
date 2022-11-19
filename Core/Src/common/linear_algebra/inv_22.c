//
// Created by Michal on 11.11.2022.
//

#include "linear_algebra.h"

void inv_2x2(const float* A, float* A_inv)
{
    if(A != NULL && A_inv != NULL)
    {
        //  a  b
        //  c  d
        float det;
        det = A[2*0 + 0] * A[2*1 + 1] - A[2*0 + 1] * A[2*1 + 0];  // ad - bc
        A_inv[2*0 + 0] = A[2*1 + 1]/det;
        A_inv[2*0 + 1] = -A[2*0 + 1]/det;
        A_inv[2*1 + 0] = -A[2*1 + 0]/det;
        A_inv[2*1 + 1] = A[2*0 + 0]/det;
    }
}