#pragma once

typedef float T;

void LTLInPlace(T *matrix, const int *parentIndex, int n);
void backSubstitutionInPlace(T *matrix, T *b, const int *parentIndex, int n);
void forwardSubstitutionInPlace(T *matrix, T *b, const int *parentIndex, int n);
