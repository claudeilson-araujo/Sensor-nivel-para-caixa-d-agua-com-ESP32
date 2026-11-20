/*
  Nível das caixas d'água + controle de bombas (ESP32, Arduino IDE)
  Funciona sozinho: sem Wi-Fi, sem Home Assistant. Tudo aparece no display OLED.

  Bibliotecas (Gerenciador de Bibliotecas): "Adafruit SSD1306" e "Adafruit GFX Library"
  Placa: "ESP32 Dev Module" (pacote "esp32" da Espressif)

  IMPORTANTE: o pino ECHO do JSN-SR04T entrega 5 V. Use um divisor de tensão
  (1 kΩ em série + 2 kΩ para o GND) antes de entrar no ESP32.

  Os relés acionam a BOBINA de um contator (nunca a bomba direto).
  Mantenha as boias elétricas em série com a bobina como proteção independente.

  Botões (ligados entre o pino e o GND):
    - Toque curto: liga/desliga o modo AUTOMÁTICO da bomba
    - Segurar 3 s: apaga a falha por tempo excedido da bomba
  Modo automático desligado = bomba desligada.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================================
// CONFIGURAÇÃO
// =====================================================================

// Calibração: distância do sensor até a água, em cm
//   CHEIA = água no nível máximo | VAZIA = nível mínimo útil
const float CX1_CHEIA  = 30.0,  CX1_VAZIA  = 120.0;
const float CX2_CHEIA  = 30.0,  CX2_VAZIA  = 120.0;
const float CIST_CHEIA = 30.0,  CIST_VAZIA = 180.0;

// Regras de operação
const float    NIVEL_LIGA     = 30;                 // % da caixa: liga abaixo disso
const float    NIVEL_DESLIGA  = 95;                 // % da caixa: desliga acima disso
const float    CIST_NIVEL_MIN = 15;                 // % da cisterna: abaixo disso, bomba bloqueada
const uint32_t TEMPO_MAX_MS   = 30UL * 60UL * 1000UL;  // funcionamento contínuo máximo (30 min)
const uint32_t PAUSA_MIN_MS   = 2UL * 60UL * 1000UL;   // pausa mínima entre partidas (2 min)
const bool     AUTO_PADRAO    = false;              // estado do modo automático ao ligar

// Pinos
const uint8_t PIN_SDA = 21, PIN_SCL = 27;           // display OLED (I2C)
const uint8_t PIN_RELE1 = 22, PIN_RELE2 = 23;       // módulo relé (ativo em nível baixo)
const uint8_t PIN_BTN1 = 32, PIN_BTN2 = 33;         // botões (para o GND)

// =====================================================================
// SENSORES
// =====================================================================

struct Sensor {
  uint8_t trig, echo;
  float cheia, vazia;
  float amostras[5];
  uint8_t idx;
  float nivel;  // em %, NAN se sem leitura válida
};

enum { CX1 = 0, CX2 = 1, CIST = 2 };

Sensor sensores[3] = {
  {25, 26, CX1_CHEIA,  CX1_VAZIA,  {NAN, NAN, NAN, NAN, NAN}, 0, NAN},
  {18, 19, CX2_CHEIA,  CX2_VAZIA,  {NAN, NAN, NAN, NAN, NAN}, 0, NAN},
  {16, 17, CIST_CHEIA, CIST_VAZIA, {NAN, NAN, NAN, NAN, NAN}, 0, NAN},
};

// Mediana das últimas amostras válidas -> nível em %
float calcularNivel(const Sensor &s) {
  float v[5];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (!isnan(s.amostras[i])) v[n++] = s.amostras[i];
  }
  if (n < 3) return NAN;  // poucas leituras válidas

  for (uint8_t i = 1; i < n; i++) {  // ordenação simples
    float x = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  float dist = v[n / 2];
  float p = (s.vazia - dist) / (s.vazia - s.cheia) * 100.0f;
  return constrain(p, 0.0f, 100.0f);
}

// Uma leitura do ultrassônico (bloqueia no máximo ~30 ms)
void amostrar(Sensor &s) {
  digitalWrite(s.trig, LOW);
  delayMicroseconds(4);
  digitalWrite(s.trig, HIGH);
  delayMicroseconds(15);
  digitalWrite(s.trig, LOW);

  unsigned long d = pulseIn(s.echo, HIGH, 30000UL);
  float cm = NAN;
  if (d > 0) {
    cm = d * 0.0343f / 2.0f;
    if (cm < 20.0f || cm > 450.0f) cm = NAN;  // fora da faixa do JSN-SR04T
  }
  s.amostras[s.idx] = cm;
  s.idx = (s.idx + 1) % 5;
  s.nivel = calcularNivel(s);
}

// =====================================================================
// BOMBAS
// =====================================================================

struct Bomba {
  uint8_t pino;
  bool ligada;
  bool autom;
  bool falha;
  uint32_t inicio;
  uint32_t fim;
  uint8_t botao;
  uint32_t btnDesde;
  bool btnAtivo;
};

Bomba bombas[2] = {
  {PIN_RELE1, false, AUTO_PADRAO, false, 0, 0, PIN_BTN1, 0, false},
  {PIN_RELE2, false, AUTO_PADRAO, false, 0, 0, PIN_BTN2, 0, false},
};

void ajustarRele(Bomba &b, bool liga) {
  if (liga == b.ligada) return;
  b.ligada = liga;
  digitalWrite(b.pino, liga ? LOW : HIGH);
  if (liga) b.inicio = millis(); else b.fim = millis();
}

bool cisternaOk() {
  float c = sensores[CIST].nivel;
  return !isnan(c) && c >= CIST_NIVEL_MIN;
}

// Lógica de controle (chamada a cada 5 s)
void controlar() {
  uint32_t agora = millis();
  bool cistOk = cisternaOk();

  for (uint8_t i = 0; i < 2; i++) {
    Bomba &b = bombas[i];
    float n = sensores[i].nivel;

    if (b.ligada) {
      // Desliga se: cisterna baixa, automático desligado, sensor sem leitura ou caixa cheia
      bool desligar = !cistOk || !b.autom || isnan(n) || n >= NIVEL_DESLIGA;
      if (agora - b.inicio > TEMPO_MAX_MS) {  // tempo máximo excedido
        desligar = true;
        b.falha = true;
      }
      if (desligar) ajustarRele(b, false);
    } else if (b.autom && !b.falha && cistOk && !isnan(n) && n <= NIVEL_LIGA &&
               (agora - b.fim) > PAUSA_MIN_MS) {
      ajustarRele(b, true);
    }
  }
}

void lerBotoes() {
  uint32_t agora = millis();
  for (uint8_t i = 0; i < 2; i++) {
    Bomba &b = bombas[i];
    bool apertado = digitalRead(b.botao) == LOW;
    if (apertado && !b.btnAtivo) {
      b.btnAtivo = true;
      b.btnDesde = agora;
    } else if (!apertado && b.btnAtivo) {
      b.btnAtivo = false;
      uint32_t dur = agora - b.btnDesde;
      if (dur >= 3000) b.falha = false;        // segurou 3 s: apaga falha
      else if (dur >= 50) b.autom = !b.autom;  // toque curto: alterna automático
    }
  }
}

// =====================================================================
// DISPLAY (sem acentos: a fonte padrão não tem)
// =====================================================================

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void linhaNivel(int y, const char *nome, float n, bool ligada, bool autom, bool temBomba) {
  display.setCursor(0, y);
  display.print(nome);

  display.setCursor(24, y);
  if (isnan(n)) {
    display.print("--");
  } else {
    display.print((int) round(n));
    display.print("%");
  }

  display.drawRect(54, y, 44, 8, SSD1306_WHITE);
  if (!isnan(n)) display.fillRect(54, y, (int) (n * 0.44f), 8, SSD1306_WHITE);

  if (temBomba) {
    if (ligada) { display.setCursor(102, y); display.print("ON"); }
    if (autom)  { display.setCursor(120, y); display.print("A"); }
  }
}

void desenhar() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  linhaNivel(0,  "Cx1",  sensores[CX1].nivel,  bombas[0].ligada, bombas[0].autom, true);
  linhaNivel(14, "Cx2",  sensores[CX2].nivel,  bombas[1].ligada, bombas[1].autom, true);
  linhaNivel(28, "Cist", sensores[CIST].nivel, false, false, false);

  display.setCursor(0, 44);
  if (bombas[0].falha)      display.print("FALHA BOMBA 1");
  else if (bombas[1].falha) display.print("FALHA BOMBA 2");
  else if (!cisternaOk())   display.print("CISTERNA BAIXA");
  else                      display.print("Sistema OK");

  display.setCursor(0, 56);
  display.print("B1:");
  display.print(bombas[0].autom ? "AUTO" : "OFF");
  display.print(" B2:");
  display.print(bombas[1].autom ? "AUTO" : "OFF");

  display.display();
}

// =====================================================================
// SETUP / LOOP
// =====================================================================

void setup() {
  Serial.begin(115200);

  // Relés começam desligados (módulo ativo em nível baixo)
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(bombas[i].pino, OUTPUT);
    digitalWrite(bombas[i].pino, HIGH);
    pinMode(bombas[i].botao, INPUT_PULLUP);
  }
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(sensores[i].trig, OUTPUT);
    pinMode(sensores[i].echo, INPUT);
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display OLED nao encontrado (endereco 0x3C). Seguindo sem display.");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();
}

void loop() {
  static uint32_t tAmostra = 0, tControle = 0, tTela = 0;
  static uint8_t prox = 0;
  uint32_t agora = millis();

  lerBotoes();

  if (agora - tAmostra >= 200) {  // um sensor por vez, em rodízio
    tAmostra = agora;
    amostrar(sensores[prox]);
    prox = (prox + 1) % 3;
  }
  if (agora - tControle >= 5000) {
    tControle = agora;
    controlar();
  }
  if (agora - tTela >= 500) {
    tTela = agora;
    desenhar();
  }
}
