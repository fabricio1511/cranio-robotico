/*
 * Cabeça InMoov - ESP32 + PCA9685 + servo de rotação da cabeça (GPIO 13)
 * Movimento em VELOCIDADE CONSTANTE (sem "pulo" direto para a posição).
 *
 * OBS: o conteúdo é Arduino (C++). A extensão .c é só para facilitar a
 * visualização na web. Para compilar na Arduino IDE, salve como .ino
 * (dentro de uma pasta com o mesmo nome do arquivo).
 *
 * ====== PROTOCOLO (linhas de texto terminadas em '\n') ======
 * Posições:  v0,v1,...,v16
 *     v0..v15 -> canais do PCA9685 (I2C)
 *     v16     -> rotação da cabeça: servo direto no GPIO 13 (50 Hz gerado
 *                pelo timer do periférico LEDC, sem biblioteca extra).
 *                Usa a MESMA unidade dos demais: ticks de 12 bits em 50 Hz
 *                (1 tick = 20000/4096 us ~ 4,88 us). Linhas antigas com 16
 *                valores continuam válidas (v16 = não mexe).
 *     0  -> NÃO mexe nesse canal
 *     >0 -> novo alvo (em ticks), cortado nos limites do canal.
 *           O servo anda até lá em velocidade constante.
 *
 * Comandos:
 *     SPD n      -> velocidade de TODOS os canais, em ticks/segundo
 *     SPD c n    -> velocidade só do canal c
 *     SYNC 1     -> todos os servos de um mesmo comando chegam juntos
 *                   (o que tem mais a andar usa a sua velocidade; os
 *                   demais são reduzidos, cada um ainda em vel. constante)
 *     SYNC 0     -> cada servo anda na sua própria velocidade (padrão)
 *     STOP       -> congela todos onde estão (mantém o sinal)
 *     OFF        -> desliga o sinal de todos (servos soltos)
 *     ?          -> mostra posições, velocidades e modo SYNC
 *     LIM        -> mostra a tabela de limites
 *
 * IMPORTANTE: ao ligar a placa (ou após OFF) a posição real do servo é
 * desconhecida. O PRIMEIRO comando de cada canal vai direto ao alvo; só
 * a partir daí o movimento passa a ser em rampa.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ====== CONFIGURAÇÕES ======
#define SDA_PIN        21
#define SCL_PIN        22
#define PCA_ADDR       0x40
#define SERVO_FREQ     50
#define NUM_PCA        16      // canais 0-15: PCA9685 (I2C)
#define NUM_SERVOS     17      // + 1 servo ligado direto ao ESP32
#define CANAL_ROTACAO  16      // índice do servo de rotação da cabeça
#define PINO_ROTACAO   13      // GPIO do servo de rotação (sem I2C)
#define LEDC_RES_BITS  16      // resolução do timer LEDC (>= 12 bits)
#define LEDC_CANAL     0       // canal LEDC (usado só no core Arduino-ESP32 2.x)
#define BAUDRATE       115200

#define INTERVALO_MS   20      // período de atualização (50 Hz = ciclo do servo)
#define VEL_PADRAO     500     // ticks por segundo (menor = mais lento)
#define VEL_MIN        5
#define VEL_MAX        3000
#define REPORTAR_FIM   0       // 1 = imprime "FIM" quando todos terminam de andar

// ====== LIMITES POR CANAL (ticks de 12 bits) ======
// Canais 2 e 6 (Testa): servo sem mecanismo -> faixa livre.
// Canal 16 (Rotação): limites PROVISÓRIOS (1,0 ms a 2,0 ms). AJUSTE conforme o
// curso mecânico real do pescoço antes de usar a faixa completa!
// Canal:                0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16
const int LIM_MIN[NUM_SERVOS] = {150, 191, 150, 225, 210, 226, 150, 251, 228, 250, 250, 271, 263, 230, 250, 271, 200};
const int LIM_MAX[NUM_SERVOS] = {321, 299, 600, 308, 327, 373, 600, 306, 355, 350, 310, 340, 312, 312, 350, 327, 400};

const char *NOMES[NUM_SERVOS] = {
  "Maxilar", "Sobrancelha Esq", "Testa Esq", "Palpebra Dir Baixo",
  "Palpebra Dir Cima", "Sobrancelha Dir", "Testa Dir", "Olho Dir E/D",
  "Olho Dir U/D", "Bochecha Dir", "Palpebra Esq Cima", "Palpebra Esq Baixo",
  "Olho Esq E/D", "Olho Esq U/D", "Bochecha Esq", "Nariz",
  "Rotacao Cabeca"
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

// ====== SAÍDA DE UM CANAL ======
// Canais 0-15 vão pelo PCA9685; o canal 16 sai pelo GPIO 13 usando o timer
// LEDC em 50 Hz. Como o ciclo tem 4096 ticks, duty = ticks << (RES - 12).
// ticks = 0 -> duty 0 -> pino em nível baixo (servo solto), igual ao PCA.
void escreverCanal(uint8_t c, int ticks) {
  if (c < NUM_PCA) {
    pwm.setPWM(c, 0, ticks);
  } else {
    uint32_t duty = (uint32_t)ticks << (LEDC_RES_BITS - 12);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PINO_ROTACAO, duty);
#else
    ledcWrite(LEDC_CANAL, duty);
#endif
  }
}

void iniciarServoRotacao() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PINO_ROTACAO, SERVO_FREQ, LEDC_RES_BITS);
  ledcWrite(PINO_ROTACAO, 0);
#else
  ledcSetup(LEDC_CANAL, SERVO_FREQ, LEDC_RES_BITS);
  ledcAttachPin(PINO_ROTACAO, LEDC_CANAL);
  ledcWrite(LEDC_CANAL, 0);
#endif
}

// ====== ESTADO DO MOVIMENTO ======
float posAtual[NUM_SERVOS];      // posição atual (ticks, com fração)
int   alvo[NUM_SERVOS];          // 0 = sem movimento pendente
bool  conhecido[NUM_SERVOS];     // false até o primeiro comando do canal
int   ultimoPWM[NUM_SERVOS];     // último valor realmente escrito no PCA9685
float velConfig[NUM_SERVOS];     // velocidade configurada (ticks/s)
float velEf[NUM_SERVOS];         // velocidade efetiva do movimento atual
bool  modoSync = false;

unsigned long ultimaAtualizacao = 0;
bool estavaMovendo = false;

char buffer[256];
size_t idx = 0;

// ====== MOVIMENTO ======
void atualizarMovimento() {
  unsigned long agora = millis();
  unsigned long dt = agora - ultimaAtualizacao;
  if (dt < INTERVALO_MS) return;
  ultimaAtualizacao = agora;
  float dts = dt / 1000.0f;

  bool movendo = false;
  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    if (alvo[c] == 0) continue;

    float dif = alvo[c] - posAtual[c];
    float passo = velEf[c] * dts;

    if (fabsf(dif) <= passo) {            // chegou
      posAtual[c] = alvo[c];
      alvo[c] = 0;
    } else {
      posAtual[c] += (dif > 0) ? passo : -passo;
      movendo = true;
    }

    int p = (int)lroundf(posAtual[c]);
    if (p != ultimoPWM[c]) {              // só escreve se mudou
      escreverCanal(c, p);
      ultimoPWM[c] = p;
    }
  }

#if REPORTAR_FIM
  if (estavaMovendo && !movendo) Serial.println("FIM");
#endif
  estavaMovendo = movendo;
}

// ====== COMANDOS ======
void soltarTodos() {
  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    escreverCanal(c, 0);
    alvo[c] = 0;
    conhecido[c] = false;
    ultimoPWM[c] = 0;
  }
  estavaMovendo = false;
  Serial.println("OK OFF");
}

void pararTudo() {
  for (uint8_t c = 0; c < NUM_SERVOS; c++) alvo[c] = 0;
  estavaMovendo = false;
  Serial.println("OK STOP");
}

void mostrarStatus() {
  Serial.print("POS ");
  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    Serial.print(conhecido[c] ? (int)lroundf(posAtual[c]) : 0);
    if (c < NUM_SERVOS - 1) Serial.print(",");
  }
  Serial.println();
  Serial.print("SPD ");
  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    Serial.print((int)lroundf(velConfig[c]));
    if (c < NUM_SERVOS - 1) Serial.print(",");
  }
  Serial.printf("  SYNC %d\n", modoSync ? 1 : 0);
}

void mostrarLimites() {
  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    Serial.printf("CH%02d %-20s %d - %d\n", c, NOMES[c], LIM_MIN[c], LIM_MAX[c]);
  }
}

void comandoVelocidade(char *args) {
  int a = 0, b = 0;
  int n = sscanf(args, "%d %d", &a, &b);
  if (n == 1) {
    if (a < VEL_MIN || a > VEL_MAX) { Serial.println("ERRO velocidade fora da faixa"); return; }
    for (uint8_t c = 0; c < NUM_SERVOS; c++) velConfig[c] = a;
    Serial.printf("OK SPD todos = %d ticks/s\n", a);
  } else if (n == 2) {
    if (a < 0 || a >= NUM_SERVOS || b < VEL_MIN || b > VEL_MAX) {
      Serial.println("ERRO canal ou velocidade invalidos");
      return;
    }
    velConfig[a] = b;
    Serial.printf("OK SPD canal %d = %d ticks/s\n", a, b);
  } else {
    Serial.println("ERRO uso: SPD n | SPD canal n");
  }
}

// ====== QUADRO DE POSIÇÕES ======
void processarFrame(char *linha) {
  int valores[NUM_SERVOS] = {0};
  int n = 0;

  char *token = strtok(linha, ",");
  while (token != NULL && n < NUM_SERVOS) {
    valores[n++] = atoi(token);
    token = strtok(NULL, ",");
  }

  bool mexe[NUM_SERVOS] = {false};
  int recebidos = 0;

  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    int v = valores[c];
    if (v <= 0) continue;                  // zero = não mexe

    bool cortado = false;
    if (v < LIM_MIN[c]) { v = LIM_MIN[c]; cortado = true; }
    if (v > LIM_MAX[c]) { v = LIM_MAX[c]; cortado = true; }
    if (cortado) Serial.printf("CH%d (%s) -> %d [LIMITADO]\n", c, NOMES[c], v);
    recebidos++;

    if (!conhecido[c]) {                   // posição real desconhecida: vai direto
      escreverCanal(c, v);
      posAtual[c] = v;
      ultimoPWM[c] = v;
      conhecido[c] = true;
      alvo[c] = 0;
      continue;
    }

    alvo[c] = v;
    mexe[c] = true;
  }

  // Define a velocidade efetiva de cada canal que recebeu alvo novo
  int canalMaior = -1;
  float maiorDist = 0;
  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    if (!mexe[c]) continue;
    float d = fabsf(alvo[c] - posAtual[c]);
    if (d > maiorDist) { maiorDist = d; canalMaior = c; }
  }

  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    if (!mexe[c]) continue;
    if (modoSync && canalMaior >= 0 && maiorDist > 0.5f) {
      float tempo = maiorDist / velConfig[canalMaior];      // segundos
      float d = fabsf(alvo[c] - posAtual[c]);
      float v = d / tempo;
      velEf[c] = (v < VEL_MIN) ? VEL_MIN : v;
    } else {
      velEf[c] = velConfig[c];
    }
  }

  if (recebidos > 0) {
    estavaMovendo = true;
    ultimaAtualizacao = millis();          // evita "salto" no primeiro passo
  }
  Serial.printf("OK %d\n", recebidos);
}

void processarLinha(char *linha) {
  while (*linha == ' ') linha++;
  if (linha[0] == '\0') return;

  if (strncasecmp(linha, "OFF", 3) == 0)  { soltarTodos(); return; }
  if (strncasecmp(linha, "STOP", 4) == 0) { pararTudo(); return; }
  if (strncasecmp(linha, "LIM", 3) == 0)  { mostrarLimites(); return; }
  if (linha[0] == '?')                    { mostrarStatus(); return; }
  if (strncasecmp(linha, "SPD", 3) == 0)  { comandoVelocidade(linha + 3); return; }
  if (strncasecmp(linha, "SYNC", 4) == 0) {
    modoSync = (atoi(linha + 4) != 0);
    Serial.printf("OK SYNC %d\n", modoSync ? 1 : 0);
    return;
  }

  processarFrame(linha);
}

// ====== SETUP / LOOP ======
void setup() {
  Serial.begin(BAUDRATE);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);                   // I2C rápido: 16 canais cabem no período
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  iniciarServoRotacao();                   // GPIO 13, 50 Hz via timer LEDC
  delay(10);

  for (uint8_t c = 0; c < NUM_SERVOS; c++) {
    posAtual[c] = 0;
    alvo[c] = 0;
    conhecido[c] = false;
    ultimoPWM[c] = 0;
    velConfig[c] = VEL_PADRAO;
    velEf[c] = VEL_PADRAO;
  }

  Serial.println();
  Serial.println("PRONTO - v0,...,v16 (0 = nao mover; v16 = rotacao cabeca) | SPD | SYNC | STOP | OFF | ? | LIM");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      buffer[idx] = '\0';
      processarLinha(buffer);
      idx = 0;
    } else if (idx < sizeof(buffer) - 1) {
      buffer[idx++] = ch;
    } else {
      idx = 0;
      Serial.println("ERRO linha muito longa");
    }
  }

  atualizarMovimento();
}