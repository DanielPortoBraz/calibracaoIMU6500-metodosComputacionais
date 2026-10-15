
#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lu.h"

#define I2C_PORT        i2c0
#define I2C_SDA_PIN     0
#define I2C_SCL_PIN     1
#define MPU_ADDR        0x68

#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

#define ACCEL_SENS 16384.0
#define GYRO_SENS  131.0

#define NUM_POSICOES 6
#define NUM_AMOSTRAS 100

// Modelo: a_corrigida = c0*ax + c1*ay + c2*az + c3

static const double alvo[NUM_POSICOES][3] = {
    { 1.0,  0.0,  0.0},  // +X para cima
    {-1.0,  0.0,  0.0},  // -X para cima
    { 0.0,  1.0,  0.0},  // +Y para cima
    { 0.0, -1.0,  0.0},  // -Y para cima
    { 0.0,  0.0,  1.0},  // +Z para cima
    { 0.0,  0.0, -1.0}   // -Z para cima
};

static void escrever_reg(uint8_t reg, uint8_t valor)
{
    uint8_t dados[2] = {reg, valor};
    i2c_write_blocking(I2C_PORT, MPU_ADDR, dados, 2, false);
}

static int ler_reg(uint8_t reg, uint8_t *valor)
{
    if (i2c_write_blocking(I2C_PORT, MPU_ADDR,
                           &reg, 1, true) < 0)
        return 0;

    return i2c_read_blocking(I2C_PORT, MPU_ADDR,
                             valor, 1, false) == 1;
}

static int ler_sensor(int16_t accel[3], int16_t gyro[3])
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t dados[14];

    if (i2c_write_blocking(I2C_PORT, MPU_ADDR,
                           &reg, 1, true) < 0)
        return 0;

    if (i2c_read_blocking(I2C_PORT, MPU_ADDR,
                          dados, 14, false) != 14)
        return 0;

    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)(
            ((uint16_t)dados[2*i] << 8) | dados[2*i+1]
        );
    }

    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)(
            ((uint16_t)dados[8+2*i] << 8) | dados[9+2*i]
        );
    }

    return 1;
}

// Aguarda uma tecla pelo monitor serial e limpa a entrada pendente.
static void aguardar_tecla(void)
{
    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT) {
        // Aguarda o usuário.
    }

    sleep_ms(30);

    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
        // Descarta caracteres restantes, inclusive o Enter.
    }
}

// Coleta a média das leituras do acelerômetro, em g.
static int coletar_media(double media[3])
{
    int16_t accel[3], gyro[3];

    media[0] = 0.0;
    media[1] = 0.0;
    media[2] = 0.0;

    // Aguarda o sensor estabilizar.
    sleep_ms(500);

    // Descarta algumas leituras iniciais.
    for (int i = 0; i < 10; i++) {
        if (!ler_sensor(accel, gyro))
            return 0;

        sleep_ms(10);
    }

    for (int i = 0; i < NUM_AMOSTRAS; i++) {
        if (!ler_sensor(accel, gyro))
            return 0;

        for (int eixo = 0; eixo < 3; eixo++) {
            media[eixo] += accel[eixo] / ACCEL_SENS;
        }

        sleep_ms(10);
    }

    for (int eixo = 0; eixo < 3; eixo++) {
        media[eixo] /= NUM_AMOSTRAS;
    }

    return 1;
}

// Resolve as equações normais usando a função LU existente.
static int calibrar(double dados[NUM_POSICOES][4],
                    double coef[3][LU_N])
{
    double normal[LU_N][LU_N] = {{0}};
    double rhs[3][LU_N] = {{0}};

    // Monta X^T X e X^T y.
    for (int i = 0; i < NUM_POSICOES; i++) {
        for (int j = 0; j < LU_N; j++) {
            for (int k = 0; k < LU_N; k++) {
                normal[j][k] += dados[i][j] * dados[i][k];
            }

            for (int eixo = 0; eixo < 3; eixo++) {
                rhs[eixo][j] += dados[i][j] * alvo[i][eixo];
            }
        }
    }

    // Resolve um sistema 4x4 para cada eixo.
    for (int eixo = 0; eixo < 3; eixo++) {
        double A[LU_N][LU_N];
        double b[LU_N];
        double x[LU_N];

        for (int i = 0; i < LU_N; i++) {
            b[i] = rhs[eixo][i];

            for (int j = 0; j < LU_N; j++) {
                A[i][j] = normal[i][j];
            }
        }

        if (!lu_solve(A, b, x)) {
            printf("Erro: sistema singular no eixo %d.\n", eixo);
            return 0;
        }

        for (int j = 0; j < LU_N; j++) {
            coef[eixo][j] = x[j];
        }
    }

    return 1;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("\n=== MPU-6500 + METODO LU ===\n");

    uint8_t id;

    if (!ler_reg(REG_WHO_AM_I, &id)) {
        printf("Erro: sensor nao respondeu via I2C.\n");
        while (true) sleep_ms(1000);
    }

    printf("WHO_AM_I = 0x%02X\n", id);

    if (id != 0x70 && id != 0x68) {
        printf("Identificador inesperado. Verifique o sensor.\n");
        while (true) sleep_ms(1000);
    }

    escrever_reg(REG_PWR_MGMT_1, 0x01);
    sleep_ms(100);

    escrever_reg(REG_ACCEL_CONFIG, 0x00); // +/- 2g
    escrever_reg(REG_GYRO_CONFIG, 0x00);  // +/- 250 graus/s
    escrever_reg(REG_CONFIG, 0x03);

    sleep_ms(100);

    // Cada linha guarda ax, ay, az e o termo independente.
    double dados[NUM_POSICOES][4] = {{0}};
    double coef[3][LU_N] = {{0}};

    printf("\n=== CALIBRACAO DO ACELEROMETRO ===\n");
    printf("Serão utilizadas seis posicoes estaticas.\n");
    printf("Deixe o sensor firme em cada posicao.\n");

    const char *nomes[NUM_POSICOES] = {
        "+X para cima",
        "-X para cima",
        "+Y para cima",
        "-Y para cima",
        "+Z para cima",
        "-Z para cima"
    };

    for (int p = 0; p < NUM_POSICOES; p++) {
        printf("\nPosicao %d/6: %s\n", p + 1, nomes[p]);
        printf("Posicione o sensor e pressione Enter.\n");

        aguardar_tecla();

        double media[3];

        if (!coletar_media(media)) {
            printf("Erro na leitura I2C durante a coleta.\n");
            while (true) sleep_ms(1000);
        }

        dados[p][0] = media[0];
        dados[p][1] = media[1];
        dados[p][2] = media[2];
        dados[p][3] = 1.0;

        printf("Media: X=%.3f g, Y=%.3f g, Z=%.3f g\n",
               media[0], media[1], media[2]);
    }

    printf("\n=== RESOLUCAO PELO METODO LU ===\n");

    if (!calibrar(dados, coef)) {
        printf("Nao foi possivel calcular os coeficientes.\n");
        while (true) sleep_ms(1000);
    }

    const char *eixos[3] = {"X", "Y", "Z"};

    for (int eixo = 0; eixo < 3; eixo++) {
        printf("\nCoeficientes do eixo %s:\n", eixos[eixo]);
        printf("c0 = %.6f\n", coef[eixo][0]);
        printf("c1 = %.6f\n", coef[eixo][1]);
        printf("c2 = %.6f\n", coef[eixo][2]);
        printf("c3 = %.6f\n", coef[eixo][3]);
    }

    printf("\n=== LEITURA COM CALIBRACAO ===\n");
    printf("ACC raw: leitura original | ACC corr: corrigida\n\n");

    int16_t accel[3], gyro[3];

    while (true) {
        if (ler_sensor(accel, gyro)) {
            double a[3];

            for (int i = 0; i < 3; i++) {
                a[i] = accel[i] / ACCEL_SENS;
            }

            double corrigido[3];

            for (int eixo = 0; eixo < 3; eixo++) {
                corrigido[eixo] =
                    coef[eixo][0] * a[0] +
                    coef[eixo][1] * a[1] +
                    coef[eixo][2] * a[2] +
                    coef[eixo][3];
            }

            printf("ACC raw: %6.3f %6.3f %6.3f | ",
                   a[0], a[1], a[2]);

            printf("ACC corr: %6.3f %6.3f %6.3f | ",
                   corrigido[0], corrigido[1], corrigido[2]);

            printf("GYRO: %7.2f %7.2f %7.2f deg/s\n",
                   gyro[0] / GYRO_SENS,
                   gyro[1] / GYRO_SENS,
                   gyro[2] / GYRO_SENS);
        } else {
            printf("Falha na leitura I2C.\n");
        }

        sleep_ms(100);
    }

    return 0;
}
