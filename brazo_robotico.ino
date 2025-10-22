#include <Servo.h>

// ================== CONFIGURACION PROFESIONAL ==================
const int NUM_SERVOS = 4;
const int PIN_SERVOS[NUM_SERVOS] = {8, 9, 10, 11};
const String NOMBRES_SERVOS[NUM_SERVOS] = {"BASE", "ARTICULACION", "ROTACION", "PINZA"};

Servo servos[NUM_SERVOS];
int posicionesActuales[NUM_SERVOS] = {0, 0, 0, 0};

// ================== ESTRUCTURA PARA SECUENCIAS ==================
struct Movimiento {
  int servoIdx;
  int inicio;
  int fin;
  int delayMs;
  const char* mensaje;
};

// ================== SECUENCIAS DE MOVIMIENTO ==================
const Movimiento SECUENCIAS[] = {
  // Fase 1: Posicion inicial
  {3, 0, 0, 25, "Pinza: CERRADA"},
  {2, 0, 0, 25, "Rotacion: NEUTRA"},
  {1, 0, 0, 25, "Articulacion: ABAJO"},
  {0, 40, 40, 25, "Base: POSICION MEDIA"},
  
  // Fase 2: Preparacion para recoger
  {2, 0, 160, 25, "Rotacion: GIRANDO HACIA OBJETO"},
  {0, 0, 0, 25, "Base: CENTRADA"},
  {3, 0, 90, 25, "Pinza: ABIERTA"},
  {1, 0, 180, 25, "Articulacion: SUBIENDO"},
  {0, 0, 90, 25, "Base: GIRANDO A DERECHA"},
  {3, 90, 0, 25, "Pinza: CERRANDO (AGARRANDO)"},
  {0, 90, 30, 25, "Base: POSICION INTERMEDIA"},
  {1, 180, 0, 25, "Articulacion: BAJANDO (TRANSPORTE)"},
  {2, 160, 0, 25, "Rotacion: VOLVIENDO A NEUTRO"},
  
  // Fase 3: Movimiento de exhibicion
  {0, 0, 0, 25, "Base: CENTRADA"},
  {1, 0, 180, 25, "Articulacion: SUBIENDO"},
  {0, 0, 180, 25, "Base: GIRO COMPLETO IZQUIERDA"},
  {3, 0, 90, 25, "Pinza: ABIERTA (EXHIBICION)"},
  {3, 90, 0, 25, "Pinza: CERRADA"},
  {0, 180, 30, 25, "Base: VOLVIENDO A MEDIA"},
  {1, 180, 0, 25, "Articulacion: BAJANDO"}
};

const int TOTAL_MOVIMIENTOS = sizeof(SECUENCIAS) / sizeof(SECUENCIAS[0]);

// ================== FUNCIONES PROFESIONALES ==================
void inicializarSistema() {
  Serial.begin(9600);
  Serial.println("INICIALIZANDO BRAZO ROBOTICO");
  Serial.println("============================");
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(PIN_SERVOS[i]);
    servos[i].write(0);
    delay(100);
    Serial.println("OK - Servo " + NOMBRES_SERVOS[i] + " conectado en pin " + String(PIN_SERVOS[i]));
  }
  
  Serial.println("");
  Serial.println("SISTEMA LISTO - Iniciando secuencia automatica");
  Serial.println("==============================================");
}

void moverServoSuave(int servoIdx, int objetivo, int tiempoDelay, const char* mensaje) {
  int inicio = posicionesActuales[servoIdx];
  
  Serial.println("MOVIENDO - " + String(mensaje) + " | Servo: " + NOMBRES_SERVOS[servoIdx] + 
                " | " + String(inicio) + " grados -> " + String(objetivo) + " grados");
  
  if (inicio < objetivo) {
    for (int pos = inicio; pos <= objetivo; pos++) {
      servos[servoIdx].write(pos);
      delay(tiempoDelay);
    }
  } else {
    for (int pos = inicio; pos >= objetivo; pos--) {
      servos[servoIdx].write(pos);
      delay(tiempoDelay);
    }
  }
  
  posicionesActuales[servoIdx] = objetivo;
  Serial.println("COMPLETADO - Posicion: " + String(objetivo) + " grados");
}

void ejecutarMovimientoDirecto(int servoIdx, int posicion, int tiempoDelay, const char* mensaje) {
  Serial.println("POSICIONAR - " + String(mensaje) + " | Posicion: " + String(posicion) + " grados");
  servos[servoIdx].write(posicion);
  posicionesActuales[servoIdx] = posicion;
  delay(tiempoDelay);
}

void mostrarEstadoActual() {
  Serial.println("");
  Serial.println("ESTADO ACTUAL DE SERVOS:");
  Serial.println("-----------------------");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.println("  " + NOMBRES_SERVOS[i] + ": " + String(posicionesActuales[i]) + " grados");
  }
  Serial.println("-----------------------");
  Serial.println("");
}

// ================== PROGRAMA PRINCIPAL ==================
void setup() {
  inicializarSistema();
}

void loop() {
  int faseActual = 1;
  
  for (int i = 0; i < TOTAL_MOVIMIENTOS; i++) {
    Movimiento mov = SECUENCIAS[i];
    
    // Detectar cambios de fase
    if (i == 0 || i == 4 || i == 13) {
      Serial.println("");
      Serial.println("FASE " + String(faseActual) + " INICIADA");
      Serial.println("================");
      faseActual++;
    }
    
    // Ejecutar movimiento
    if (mov.inicio == mov.fin) {
      ejecutarMovimientoDirecto(mov.servoIdx, mov.inicio, mov.delayMs, mov.mensaje);
    } else {
      moverServoSuave(mov.servoIdx, mov.fin, mov.delayMs, mov.mensaje);
    }
    
    // Pequena pausa entre movimientos
    delay(100);
  }
  
  // Estado final y pausa
  mostrarEstadoActual();
  Serial.println("PAUSA FINAL: 9 segundos");
  Serial.println("REINICIANDO SECUENCIA...");
  Serial.println("");
  delay(9000);
}