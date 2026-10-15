#include <math.h>
#include "lu.h"

int lu_solve(const double A[LU_N][LU_N],
             const double b[LU_N],
             double x[LU_N])
{
    double U[LU_N][LU_N];
    double L[LU_N][LU_N] = {{0}};
    double pb[LU_N];
    double y[LU_N];
    int p[LU_N];

    /* Copia a matriz e inicializa a permutação de linhas. */
    for (int i = 0; i < LU_N; i++) {
        p[i] = i;
        pb[i] = b[i];
        for (int j = 0; j < LU_N; j++) {
            U[i][j] = A[i][j];
        }
        L[i][i] = 1.0;
    }

    /* Fatoração PA=LU, usando pivoteamento parcial. */
    for (int k = 0; k < LU_N; k++) {
        int pivot = k;
        double maior = fabs(U[k][k]);

        for (int i = k + 1; i < LU_N; i++) {
            if (fabs(U[i][k]) > maior) {
                maior = fabs(U[i][k]);
                pivot = i;
            }
        }

        if (maior < 1e-10) {
            return 0; /* Matriz singular ou numericamente instável. */
        }

        if (pivot != k) {
            for (int j = 0; j < LU_N; j++) {
                double tmp = U[k][j];
                U[k][j] = U[pivot][j];
                U[pivot][j] = tmp;
            }

            /* Troca apenas as colunas já calculadas de L. */
            for (int j = 0; j < k; j++) {
                double tmp = L[k][j];
                L[k][j] = L[pivot][j];
                L[pivot][j] = tmp;
            }

            int tmp_p = p[k];
            p[k] = p[pivot];
            p[pivot] = tmp_p;
        }

        for (int i = k + 1; i < LU_N; i++) {
            L[i][k] = U[i][k] / U[k][k];
            for (int j = k; j < LU_N; j++) {
                U[i][j] -= L[i][k] * U[k][j];
            }
        }
    }

    /* Aplica a mesma permutação de linhas ao vetor b. */
    for (int i = 0; i < LU_N; i++) {
        pb[i] = b[p[i]];
    }

    /* Substituição direta: L*y=P*b. */
    for (int i = 0; i < LU_N; i++) {
        double soma = pb[i];
        for (int j = 0; j < i; j++) {
            soma -= L[i][j] * y[j];
        }
        y[i] = soma / L[i][i];
    }

    /* Substituição retroativa: U*x=y. */
    for (int i = LU_N - 1; i >= 0; i--) {
        double soma = y[i];
        for (int j = i + 1; j < LU_N; j++) {
            soma -= U[i][j] * x[j];
        }
        if (fabs(U[i][i]) < 1e-10) {
            return 0;
        }
        x[i] = soma / U[i][i];
    }

    return 1;
}