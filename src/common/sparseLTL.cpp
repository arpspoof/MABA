#include "sparseLTL.h"
#include "cmath"

#define m(i, j) (matrix[(i - 1) * n + j - 1])
#define v(i) (b[i - 1])

void LTLInPlace(T *matrix, const int *parentIndex, int n)
{
    int i, j;
    for (int k = n; k > 0; k--)
    {
        m(k, k) = sqrt(m(k, k));
        j = parentIndex[k];
        while (j != 0)
        {
            m(k, j) /= m(k, k);
            j = parentIndex[j];
        }
        i = parentIndex[k];
        while (i != 0)
        {
            j = i;
            while (j != 0)
            {
                m(i, j) -= m(k, i)*m(k, j);
                j = parentIndex[j];
            }
            i = parentIndex[i];
        }
    }
}

void backSubstitutionInPlace(T *matrix, T *b, const int *parentIndex, int n)
{
    for (int i = n; i > 0; i--)
    {
        v(i) /= m(i, i);
        int j = parentIndex[i];
        while (j != 0)
        {
            v(j) -= m(i, j)*v(i);
            j = parentIndex[j];
        }
    }
}

void forwardSubstitutionInPlace(T *matrix, T *b, const int *parentIndex, int n)
{
    for (int i = 1; i <= n; i++)
    {
        int j = parentIndex[i];
        while (j != 0)
        {
            v(i) -= m(i, j) * v(j);
            j = parentIndex[j];
        }
        v(i) /= m(i, i);
    }
}
