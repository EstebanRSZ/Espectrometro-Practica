// Espectrometro de Barrido Angular — ESP32
// Optoelectronica UNAL Medellin, 01-2026
// Autores: Esteban Roberto Saiz, Sebastian Salazar Perez

#include <Arduino.h>
#include <math.h>

// ── Parametros geometricos ────────────────────────────────────────────────────
#define L_MM         100.0f   // distancia red→plano detector [mm]
#define D_UM         1.0f     // periodo red [µm]  (1000 lineas/mm)
#define MM_POR_PASO  0.0307f  // z=20 dientes, 2048 pasos/vuelta → zπ/2048 mm/paso

// ── Pines ────────────────────────────────────────────────────────────────────
#define PIN_IN1   26
#define PIN_IN2   27
#define PIN_IN3   14
#define PIN_IN4   12
#define PIN_ADC   34
#define PIN_HOME  25

// ── Configuracion serial ──────────────────────────────────────────────────────
#define BAUD_RATE  115200

// ── Parametros de barrido ─────────────────────────────────────────────────────
#define T_ESTAB_MS      50      // tiempo estabilizacion entre pasos [ms]
#define T_PASO_US       2000    // duracion de cada fase del paso [µs]
#define HOMING_PASOS    5000    // maximo de pasos durante homing
#define N_MUESTRAS_ADC  8       // promedio de lecturas ADC por punto
#define PASOS_OFFSET    200     // pasos desde home hasta orden 0 real

// Rango de barrido: desde violeta (~400 nm, θ≈23.6°) hasta rojo (~700 nm, θ≈44.4°)
// y_max = L * tan(44.4°) ≈ 100 * 0.975 ≈ 97.5 mm → pasos_max ≈ 97.5/0.0307 ≈ 3175
#define PASOS_MAX_SWEEP 3200

// ── Secuencia de pasos completos (half-step 8 fases) ─────────────────────────
// Orden de pines: IN1, IN2, IN3, IN4
static const uint8_t SECUENCIA[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};
static uint8_t fase_actual = 0;

// ── Estado de la maquina ──────────────────────────────────────────────────────
enum Estado { HOMING, DARK_CAL, SWEEP, END };
static Estado estado = HOMING;

static long  pasos_actuales = 0;
static float v_dark         = 0.0f;

// ── Funciones del motor ───────────────────────────────────────────────────────

static void aplicar_fase(uint8_t f) {
    uint8_t i = f & 7;
    digitalWrite(PIN_IN1, SECUENCIA[i][0]);
    digitalWrite(PIN_IN2, SECUENCIA[i][1]);
    digitalWrite(PIN_IN3, SECUENCIA[i][2]);
    digitalWrite(PIN_IN4, SECUENCIA[i][3]);
}

static void motor_apagar() {
    digitalWrite(PIN_IN1, 0);
    digitalWrite(PIN_IN2, 0);
    digitalWrite(PIN_IN3, 0);
    digitalWrite(PIN_IN4, 0);
}

// Avanza +1 paso (positivo = hacia longitudes de onda mayores)
static void paso_adelante() {
    fase_actual = (fase_actual + 1) & 7;
    aplicar_fase(fase_actual);
    delayMicroseconds(T_PASO_US);
    pasos_actuales++;
}

// Retrocede 1 paso (negativo = hacia home)
static void paso_atras() {
    fase_actual = (fase_actual - 1) & 7;
    aplicar_fase(fase_actual);
    delayMicroseconds(T_PASO_US);
    pasos_actuales--;
}

// ── Lectura ADC promediada ────────────────────────────────────────────────────

static int leer_adc() {
    long suma = 0;
    for (int i = 0; i < N_MUESTRAS_ADC; i++) {
        suma += analogRead(PIN_ADC);
        delayMicroseconds(200);
    }
    return (int)(suma / N_MUESTRAS_ADC);
}

// ── Conversion posicion → longitud de onda ────────────────────────────────────
// λ(nm) = d(µm) * sin(arctan(y_mm / L_mm)) * 1000

static float calcular_lambda(long pasos) {
    float y_mm  = (float)pasos * MM_POR_PASO;
    float theta = atan2f(y_mm, L_MM);           // rad
    float lambda_um = D_UM * sinf(theta);        // µm
    return lambda_um * 1000.0f;                  // nm
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    motor_apagar();

    pinMode(PIN_HOME, INPUT);   // pull-down externo en la placa

    // ADC: atenuacion para rango 0–3.3V (OPT101 alimentado a 5V, pero
    // la salida maxima practica con esta resistencia es ~3.3V en el rango visible)
    analogSetAttenuation(ADC_11db);   // rango 0–3.3 V → 0–4095
    analogReadResolution(12);

    delay(500);
    Serial.println("#BOOT");
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
    switch (estado) {

    // ── HOMING ───────────────────────────────────────────────────────────────
    case HOMING: {
        Serial.println("#HOMING");
        pasos_actuales = 0;

        // Retroceder hasta activar fin de carrera
        for (int i = 0; i < HOMING_PASOS; i++) {
            if (digitalRead(PIN_HOME) == HIGH) break;
            paso_atras();
        }
        motor_apagar();
        delay(200);

        // Avanzar PASOS_OFFSET pasos desde el home para posicionarse en orden 0
        for (int i = 0; i < PASOS_OFFSET; i++) {
            paso_adelante();
        }
        motor_apagar();
        pasos_actuales = 0;   // redefinir origen en orden 0

        Serial.println("#HOMING_DONE");
        estado = DARK_CAL;
        break;
    }

    // ── DARK CALIBRATION ─────────────────────────────────────────────────────
    case DARK_CAL: {
        Serial.println("#DARK_CAL_START");
        // Esperar a que el usuario apague la fuente (3 segundos)
        delay(3000);

        long suma = 0;
        for (int i = 0; i < 32; i++) {
            suma += analogRead(PIN_ADC);
            delay(10);
        }
        v_dark = (float)(suma / 32);

        Serial.print("#DARK:");
        Serial.println((int)v_dark);
        estado = SWEEP;
        break;
    }

    // ── SWEEP ─────────────────────────────────────────────────────────────────
    case SWEEP: {
        Serial.println("#SWEEP_START");
        delay(500);

        for (long p = 0; p < PASOS_MAX_SWEEP; p++) {
            paso_adelante();
            delay(T_ESTAB_MS);

            int v_raw = leer_adc();
            float lambda_nm = calcular_lambda(pasos_actuales);

            // Solo emitir datos en el rango visible util (350–750 nm)
            if (lambda_nm >= 350.0f && lambda_nm <= 750.0f) {
                Serial.print(lambda_nm, 2);
                Serial.print(',');
                Serial.println(v_raw);
            }
        }

        motor_apagar();
        Serial.println("#SWEEP_END");
        estado = END;
        break;
    }

    // ── END ───────────────────────────────────────────────────────────────────
    case END: {
        motor_apagar();
        // Esperar comando de reinicio desde el PC ('r' + Enter)
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'r' || c == 'R') {
                Serial.println("#RESET");
                estado = HOMING;
            }
        }
        delay(100);
        break;
    }

    } // switch
}
