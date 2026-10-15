#ifndef LU_H
#define LU_H

#define LU_N 4

/*
 * Resolve A*x=b pelo método LU com pivoteamento parcial.
 * Retorna 1 em caso de sucesso e 0 se a matriz for singular/quase singular.
 * A e b são copiados internamente; os argumentos não são modificados.
 */
int lu_solve(const double A[LU_N][LU_N],
             const double b[LU_N],
             double x[LU_N]);

#endif