// =============================================================================
// VERSA3_MEGA.ino  –  ARDUINO MEGA 2560  (COLETOR DE DADOS / SLAVE BT)
// =============================================================================
//
// OBJETIVO:
//   Coletar dados brutos dos sensores e enviar via Bluetooth (HC-06 Slave)
//   para o ESP32 CYD (Master BT). O CYD realiza todos os cálculos matemáticos.
//   O Mega também controla relés, motor PWM e alarmes.
//
// ARQUITETURA (v4.1 HC-06 Slave):
//   - Mega lê sensores → envia VALORES BRUTOS via Serial1 → HC-06 (Slave BT)
//   - ESP32 CYD (Master) conecta ao HC-06 usando o MAC salvo na NVS
//   - ESP32 CYD recebe CSV bruto, converte para grandezas físicas e exibe
//
// ─────────────────────────────────────────────────────────────────────────────
// PINAGEM COMPLETA  (Mega 2560)
// ─────────────────────────────────────────────────────────────────────────────
//  Pino  │ Função                         │ Observação
// ───────┼────────────────────────────────┼─────────────────────────────────────
//  D2    │ SENSOR RPM  (SJA12-10N1)       │ INT0  –  FALLING  (NPN open-collector)
//         │                                │  Ligar saída do sensor ao D2 via
//         │                                │  resistor pull-up 10kΩ para +5V.
//  D11   │ PWM Motor (controlador BLDC)   │ analogWrite 0-255
//  D12   │ LED / buzzer de alerta         │ HIGH = alerta ativo
//  A0    │ Tensão bateria B1 (principal)  │ Divisor 220kΩ + 10kΩ → 0-5V
//  A1    │ Tensão bateria B2 (reserva)    │ idem
//  A2    │ Sensor de chuva (digital)      │ LOW = chuva (INPUT_PULLUP)
//  A3    │ Tensão bateria B3 (sistema 12V)│ Divisor 220kΩ + 10kΩ → 0-5V
//  A4    │ Corrente (ACS758LCB-050B)      │ Saída analógica: 2.5V=0A, 40mV/A
//  D22   │ Relé  1 – Motor 1x/3x         │ HIGH = ativado
//  D23   │ Relé  2 – Limitador veloc.     │
//  D24   │ Relé  3 – Seleção bat. B1/B2  │
//  D25   │ Relé  4 – Marcha ré            │
//  D26   │ Relé  5 – Velocidade nível 1   │
//  D27   │ Relé  6 – Velocidade nível 2   │
//  D28   │ Relé  7 – Luz interna          │
//  D29   │ Relé  8 – Carregamento Solar B1│
//  D30   │ Relé  9 – Carregamento Solar B2│
//  D31   │ Relé 10 – Ventilador           │
//  D32   │ Relé 11 – Seta Esquerda        │
//  D33   │ Relé 12 – Seta Direita         │
//  D34   │ Relé 13 – Proteção B3          │
//  D35   │ Relé 14 – Freio                │
//  D36   │ Relé 15 – Farol                │
//  TX1/RX1 (pinos 18/19) → HC-06 Bluetooth SLAVE (9600 baud) → ESP32 CYD (Master)
//  USB Serial (pinos 0/1)→ Monitor serial / debug (115200 baud)
//
// HC-06 CONFIGURAÇÃO (via AT commands com baud 9600, sem jumper):
//   AT+NAME=VERSA3_MEGA_HC06  → define o nome visível ao fazer scan
//   AT+PIN=1234               → senha de pareamento (deve bater com o ESP32)
//   AT+BAUD4                  → garante 9600 baud (padrão de fábrica)
//   Nota: O HC-06 NÃO precisa de configuração de ROLE (é sempre Slave).
//   Após configurar, reiniciar o módulo; ele ficará em modo slave aguardando.
//   O ESP32 CYD irá detectar o nome "VERSA3_MEGA_HC06" no scan e conectar.
//
// PROTOCOLO BT (Mega → CYD) v4.0 — CSV BRUTO:
//   Linha principal (a cada 500ms):
//     <rawB1>,<rawB2>,<rawB3>,<pulsos>,<chuva>,<setas>,<rawCorr>\n
//     rawB1   = ADC A0 (0-1023) bateria B1
//     rawB2   = ADC A1 (0-1023) bateria B2
//     rawB3   = ADC A3 (0-1023) bateria B3 sistema
//     pulsos  = pulsos RPM desde o último envio (unsigned long)
//     chuva   = 0=seco | 1=chuva
//     setas   = 0=off | 1=esq | 2=dir | 3=hazard
//     rawCorr = ADC A4 (0-1023) sensor corrente ACS758
//
//   Linhas de alerta/evento (quando detectado):
//     ALERT,<tipo>   – SUBTENSAO | CORRENTE | CHUVA | ROUBO | FREIO
//     CLEAR,<tipo>   – quando condição de alerta é resolvida
//     KEYOFF         – chave de 48V desligada (debounce 2s)
//     KEYON          – chave de 48V religada
//     FAROL,<0|1>    – estado do relé 15 (botão físico de farol)
//     RADAR,<esq>,<dir> – distância traseira (quando R4=ré ativa)
//     READY          – Mega inicializado (enviado no setup)
//     ACK:R,<n>,<e>  – confirmação de comando de relé
//
// DIVISOR DE TENSÃO (48V ou 72V → 0-5V para ADC):
//   R1 = 220 kΩ  (entre bateria+ e pino analógico)
//   R2 =  10 kΩ  (entre pino analógico e GND)
//   Vout = Vbat × R2 / (R1 + R2) = Vbat × 10/230 ≈ Vbat × 0,04348
//   Vbat = Vadc × (VREF / ADC_MAX) / DIVISOR_RAZAO
//   ESP32 usa: vBx = (rawBx / 1023.0) * 5.0 / 0.04348
//
// SENSOR RPM – SJA12-10N1:
//   Sensor magnético indutivo, 3 fios, saída NPN open-collector.
//   Fios: Marrom = VCC (10-30V DC), Azul = GND, Preto = Saída NPN.
//   Circuito de interface: saída preta → resistor 10kΩ pull-up 5V → D2 do Mega.
//   PULSOS_POR_VOLTA: número de magnetos fixados na roda/eixo (ajustar conforme instalação).
//   ESP32 recalcula RPM e velocidade usando circRodaCM salvo na NVS.
//
// COMANDOS DE SIMULAÇÃO (bancada via Monitor Serial):
//   SIM_ROUBO       → alarmeDisparado = true  (bloqueia motor)
//   SIM_RESET_ALARM → alarmeDisparado = false (reset alarme)
//   SIM_CHUVA       → chuvaDetectada = true
//   SIM_BATT        → imprime STATUS:BATT_LOW
//   SIM_RESET_ODO   → zera odômetro e grava EEPROM
//   CHAVE:1 / CHAVE:0
//   VEL:0 / VEL:1 / VEL:2 / VEL:3
//
// EEPROM (persistência entre resets):
//   Addr 0-3 (4 bytes) → odometroMetros (unsigned long)
//   Addr 4   (1 byte)  → magic byte 0xA5 (ODO válido)
//   Addr 5-8 (4 bytes) → diametroRodaM (float)
//   Addr 9   (1 byte)  → magic byte 0xB6 (diâmetro válido)
//   Salvo a cada EEPROM_SAVE_INTERVAL_MS (30 s) se valor mudou.
//
// COMANDOS RECONHECIDOS (via BT do CYD ou USB):
//   CRUISE,ON,<vel> / CRUISE,OFF
//   PAS,<imans>
//   CURVA,<0|1|2>
//   BALANCO,<0-100>
//   LIMVEL,<kmh>
//   STANDBY,<0|1>
//   REBOOT_CLEAN       (reinicia o Mega via asm)
//   RESET_ALARM        (cancela alarme de roubo)
//   TESTE_ROUBO        (simula alarme de roubo)
//   TESTE_CHUVA        (simula chuva)
//   R,<n>,<0|1>[*XX]  (controla relé n com checksum XOR opcional)
//   C,R,<cm>           (define circunferência da roda em cm, salva EEPROM)
// =============================================================================

#include <Arduino.h>
#include <EEPROM.h>

// BAUDRATE ────────────────────────────────────────────────────────────────────
#define SERIAL_BAUD     115200  // USB (debug / monitor)  — alta velocidade para diagnóstico
#define SERIAL_CYD_BAUD 9600    // Serial1 → HC-06 Slave (baud padrão de fábrica do módulo)
// BT_WATCHDOG_TIMEOUT_MS: watchdog do Mega deve ser maior que o BT_WATCHDOG_MS do ESP32 (3000ms)
// para que o ESP32 detecte a queda primeiro e tente reconectar antes de o Mega reiniciar Serial1.
#define BT_WATCHDOG_TIMEOUT_MS  4000UL

// Debounce do sensor RPM: ignora pulsos com intervalo menor que este valor (µs).
// 3ms é seguro até 20.000 RPM com 1 ímã (período mínimo = 3ms).
#define RPM_DEBOUNCE_US  3000UL
#define PULSOS_POR_VOLTA   1       // Nº de magnetos na roda (SJA12-10N1)
#define DIAMETRO_RODA_DEFAULT 0.55f // Diâmetro padrão (m) usado na 1ª vez (EEPROM virgem)
#define TENSAO_SISTEMA     48.0f   // 48V ou 72V (para alertas de bateria)
#define TENSAO_ALERTA_PERC 0.875f  // Alerta quando bateria < 87,5% da tensão nominal
#define RPM_TIMEOUT_US     600000UL // 600ms sem pulso → RPM = 0
#define RPM_JANELA_MS      100UL   // Janela de cálculo de RPM (ms)
#define TELE_INTERVALO_MS  500UL   // Intervalo de envio de telemetria ao CYD (ms)
#define MOTOR_RAMP_STEP    5       // Passo de rampa PWM por ciclo (suaviza aceleração)

// ─── EEPROM – PERSISTÊNCIA DO ODÔMETRO E DIÂMETRO DA RODA ───────────────────
#define EEPROM_ADDR_ODO          0   // Endereço: odômetro (4 bytes, unsigned long)
#define EEPROM_MAGIC_ADDR        4   // Endereço: magic byte de dados válidos (odômetro)
#define EEPROM_MAGIC_VAL      0xA5   // Valor magic = EEPROM inicializada
#define EEPROM_ADDR_DIAM         5   // Endereço: diâmetro da roda (4 bytes, float)
#define EEPROM_DIAM_MAGIC_ADDR   9   // Endereço: magic byte para diâmetro
#define EEPROM_DIAM_MAGIC_VAL 0xB6   // Valor magic = diâmetro salvo é válido
#define EEPROM_SAVE_INTERVAL_MS  30000UL  // Intervalo mínimo entre gravações (30 s)

// ─── PINAGEM ──────────────────────────────────────────────────────────────────
static const uint8_t P_RPM          = 2;   // INT0 – SJA12-10N1
static const uint8_t P_MOTOR_PWM    = 11;  // PWM motor BLDC
static const uint8_t P_LED_ALERTA   = 12;  // LED/buzzer de alarme
static const uint8_t P_BATT_PRINC   = A0;  // Divisor tensão bateria principal (B1)
static const uint8_t P_BATT_RESERVA = A1;  // Divisor tensão bateria reserva (B2)
static const uint8_t P_SENSOR_CHUVA = A2;  // Sensor de chuva (INPUT_PULLUP)
static const uint8_t P_BATT_B3      = A3;  // Divisor tensão bateria sistema 12V (B3)
static const uint8_t P_CORRENTE     = A4;  // Sensor de corrente ACS758 (saída analógica)

// ─── SENSOR DE CORRENTE ACS758LCB-050B ───────────────────────────────────────
// Tensão de offset (sem corrente) = VCC/2 = 2.5V → ADC = 512 (@ 5V VREF, 10-bit)
// Sensibilidade: 40mV/A (para modelo 50A bidirecional)
// Corrente = (Vadc - 2.5V) / 0.040 V/A
#define ACS758_SENSITIVIDADE  0.040f  // V/A
#define ACS758_OFFSET_V       2.5f    // Tensão de offset (0A)

// Relés – mapeamento idêntico ao CYD (pinos 22-36, R1-R15)
static const uint8_t PINO_RELE[15] = {
    22, // Relé  1 – Motor 1x/3x
    23, // Relé  2 – Limitador velocidade
    24, // Relé  3 – Seleção bateria B1/B2
    25, // Relé  4 – Marcha ré
    26, // Relé  5 – Velocidade nível 1
    27, // Relé  6 – Velocidade nível 2
    28, // Relé  7 – Luz interna
    29, // Relé  8 – Carregamento Solar B1
    30, // Relé  9 – Carregamento Solar B2
    31, // Relé 10 – Ventilador
    32, // Relé 11 – Seta Esquerda
    33, // Relé 12 – Seta Direita
    34, // Relé 13 – Proteção B3
    35, // Relé 14 – Freio
    36  // Relé 15 – Farol
};
static const uint8_t NUM_RELES = sizeof(PINO_RELE) / sizeof(PINO_RELE[0]);

// ─── CONSTANTES DE CÁLCULO ────────────────────────────────────────────────────
// diametroRodaM e circunferenciaM são variáveis (não constantes) porque o valor
// pode ser configurado pelo CYD e precisa ser persistido na EEPROM.
float diametroRodaM     = DIAMETRO_RODA_DEFAULT;  // Diâmetro da roda em metros
float circunferenciaM   = 3.14159265f * DIAMETRO_RODA_DEFAULT; // Recomputado no setup
static const float DIVISOR_RAZAO       = 10.0f / (220.0f + 10.0f); // R2/(R1+R2)
static const float VREF                = 5.0f;
static const int   ADC_MAX             = 1023;
static const float TENSAO_ALERTA_V     = TENSAO_SISTEMA * TENSAO_ALERTA_PERC;

// ─── VARIÁVEIS GLOBAIS ────────────────────────────────────────────────────────

// RPM / velocidade
volatile unsigned long g_pulsos         = 0;   // Contador de pulsos (ISR)
volatile unsigned long g_ultimoPulsoUs  = 0;   // Timestamp do último pulso (µs)
static  unsigned long  g_lastRpmCalcMs  = 0;
float  rpmAtual        = 0.0f;
float  velocidadeKmh   = 0.0f;
unsigned long odometroMetros = 0UL;

// Bateria
float bateriaVoltagem  = 0.0f;
float bateriaReservaV  = 0.0f;
float bateriaB3V       = 0.0f;   // Tensão da bateria de sistema 12V (B3)
float correnteA        = 0.0f;   // Corrente de tração medida pelo ACS758 (A)

// Estado geral
bool ignitionOn        = false;
bool chuvaDetectada    = false;
volatile bool alarmeDisparado     = false;
volatile bool alarmePiscaEstado   = false;

// KEYOFF/KEYON: detecta queda de tensão da chave de 48V
// Quando a chave é desligada, B1 cai rapidamente abaixo de KEYOFF_VOLTAGE_THRESHOLD.
#define KEYOFF_VOLTAGE_THRESHOLD 5.0f   // V – abaixo disto = chave desligada
static bool chaveEraLigada = false;     // Rastreia transição ligado→desligado
static unsigned long tKeyDebounce = 0;  // Debounce de 2s para evitar falso trigger
#define KEYOFF_DEBOUNCE_MS 2000UL       // 2 segundos sem tensão = realmente desligada

// Farol: rastreia estado para enviar FAROL,x ao CYD apenas na mudança
static bool farolEstadoAnterior = false;

// ─── WATCHDOG BT ─────────────────────────────────────────────────────────────
unsigned long g_tBtHeartbeat = 0;  // Timestamp do último BT_OK recebido do ESP32
bool g_btConectado = false;        // true = ESP32 respondeu recentemente com BT_OK

// Controle de velocidade (0=off, 1=lento, 2=médio, 3=máximo)
uint8_t nivelVelocidade   = 0;
uint8_t pwmAtual          = 0;  // PWM atual (com rampa)

// Temporização
static unsigned long ultimaTelemMs = 0;

// ─── ISR – SENSOR RPM ─────────────────────────────────────────────────────────
void contarPulsoRPM() {
    unsigned long agora = micros();
    // Debounce por software: ignora pulsos com intervalo < RPM_DEBOUNCE_US (3ms)
    if (agora - g_ultimoPulsoUs > RPM_DEBOUNCE_US) {
        g_pulsos++;
        g_ultimoPulsoUs = agora;
    }
}

// ─── PERSISTÊNCIA EEPROM – ODÔMETRO ──────────────────────────────────────────
// Salva odometroMetros a cada EEPROM_SAVE_INTERVAL_MS se o valor mudou.
// EEPROM.put() só escreve bytes que diferem → protege a vida útil da memória.
void salvarOdometro() {
    static unsigned long ultimoSalvamentoMs = 0;
    static unsigned long odoSalvo           = 0xFFFFFFFFUL; // força 1ª escrita
    unsigned long agora = millis();
    if ((agora - ultimoSalvamentoMs) < EEPROM_SAVE_INTERVAL_MS) return;
    ultimoSalvamentoMs = agora;  // Actualiza o timer independentemente — evita polling constante
    if (odometroMetros == odoSalvo) return;  // Nada mudou: não escreve na flash
    EEPROM.put(EEPROM_ADDR_ODO, odometroMetros);
    EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
    odoSalvo = odometroMetros;
    Serial.print(F("EEPROM: ODO salvo = ")); Serial.print(odometroMetros); Serial.println(F("m"));
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
    // Porta USB para debug / monitor serial (115200 baud)
    Serial.begin(SERIAL_BAUD);
    // Porta Serial1 → HC-06 Bluetooth SLAVE (9600 baud padrão de fábrica)
    // O HC-06 é sempre Slave: aguarda o ESP32 CYD (Master) iniciar a conexão.
    // Após parear via scan no CYD, a comunicação é transparente (UART via BT).
    Serial1.begin(SERIAL_CYD_BAUD);
    Serial1.setTimeout(100);  // Timeout de 100ms para readBytesUntil

    // Motor PWM
    pinMode(P_MOTOR_PWM, OUTPUT);
    analogWrite(P_MOTOR_PWM, 0);

    // Sensor RPM (SJA12-10N1 NPN → INPUT_PULLUP, borda de descida)
    pinMode(P_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(P_RPM), contarPulsoRPM, FALLING);

    // Sensores analógicos (ADC – não precisam de pinMode, mas declarados para clareza)
    pinMode(P_SENSOR_CHUVA, INPUT_PULLUP);
    // P_BATT_PRINC, P_BATT_RESERVA, P_BATT_B3, P_CORRENTE → modo INPUT por padrão (ADC)

    // LED/buzzer de alerta
    pinMode(P_LED_ALERTA, OUTPUT);
    digitalWrite(P_LED_ALERTA, LOW);

    // Relés – todos desligados na inicialização (LOW = relé aberto)
    for (uint8_t i = 0; i < NUM_RELES; i++) {
        pinMode(PINO_RELE[i], OUTPUT);
        digitalWrite(PINO_RELE[i], LOW);
    }

    // Restaurar odômetro da EEPROM
    if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VAL) {
        EEPROM.get(EEPROM_ADDR_ODO, odometroMetros);
        Serial.print(F("EEPROM: ODO restaurado = ")); Serial.print(odometroMetros); Serial.println(F("m"));
    } else {
        odometroMetros = 0UL;
        Serial.println(F("EEPROM: primeiro uso - ODO = 0m"));
    }

    // Restaurar diâmetro da roda da EEPROM
    if (EEPROM.read(EEPROM_DIAM_MAGIC_ADDR) == EEPROM_DIAM_MAGIC_VAL) {
        EEPROM.get(EEPROM_ADDR_DIAM, diametroRodaM);
        circunferenciaM = 3.14159265f * diametroRodaM;
        Serial.print(F("EEPROM: Diametro restaurado = ")); Serial.print(diametroRodaM, 4); Serial.println(F("m"));
    } else {
        diametroRodaM   = DIAMETRO_RODA_DEFAULT;
        circunferenciaM = 3.14159265f * diametroRodaM;
        Serial.print(F("EEPROM: Diametro padrao = ")); Serial.print(diametroRodaM, 4); Serial.println(F("m"));
    }

    Serial.println(F("VERSA3_MEGA v4.1 HC-06 Slave iniciado."));
    // Informa CYD que Mega está pronto (CYD aguarda este token no boot)
    Serial1.println(F("READY"));
}

// ─── CÁLCULO DE RPM, VELOCIDADE E ODÔMETRO ───────────────────────────────────
void calcularRPM() {
    unsigned long agora = millis();
    unsigned long intervalo = agora - g_lastRpmCalcMs;
    
    // CORREÇÃO: Atualiza o timestamp mesmo se a janela for menor que o esperado
    // Isso evita que o RPM fique travado em valores antigos
    if (intervalo < RPM_JANELA_MS) {
        // Se o intervalo for muito pequeno, ainda assim atualizamos o timestamp
        // para evitar loop infinito de janelas "estagnadas"
        if (intervalo < 10) {  // Menos de 10ms é considerado erro/glitch
            return;
        }
        g_lastRpmCalcMs = agora - (RPM_JANELA_MS / 2);  // Ajusta para meio período
        intervalo = RPM_JANELA_MS;
    }

    // Leitura atômica do contador de pulsos
    noInterrupts();
    unsigned long pulsos = g_pulsos;
    g_pulsos = 0;
    unsigned long ultimoPulsoUs = g_ultimoPulsoUs;
    interrupts();

    g_lastRpmCalcMs = agora;

    if (pulsos > 0) {
        // RPM = (pulsos / PULSOS_POR_VOLTA) / (intervalo_ms / 60000)
        rpmAtual = (pulsos * 60000.0f) / ((float)intervalo * PULSOS_POR_VOLTA);
        velocidadeKmh = (rpmAtual * circunferenciaM * 60.0f) / 1000.0f;

        // Odômetro: distância percorrida nesta janela (em metros)
        float distM = velocidadeKmh * 1000.0f / 3600.0f * (intervalo / 1000.0f);
        odometroMetros += (unsigned long)distM;

        // DEBUG RPM: loga sempre que detecta pulsos (sensor em funcionamento)
        // Consolidado em um único Serial.println para minimizar overhead na hot path.
        {
            char dbgBuf[64];
            snprintf(dbgBuf, sizeof(dbgBuf), "[DBG RPM] pulsos=%lu int=%lums rpm=%.0f kmh=%.1f",
                pulsos, intervalo, rpmAtual, velocidadeKmh);
            Serial.println(dbgBuf);
        }
    } else {
        // Se nenhum pulso chegou e já passou o timeout → RPM = 0
        if ((micros() - ultimoPulsoUs) > RPM_TIMEOUT_US) {
            rpmAtual = 0.0f;
            velocidadeKmh = 0.0f;
        }
    }
}

// ─── LEITURA DE BATERIA E CORRENTE ───────────────────────────────────────────
void lerBateria() {
    int rawP = analogRead(P_BATT_PRINC);
    bateriaVoltagem = (rawP / (float)ADC_MAX) * VREF / DIVISOR_RAZAO;

    int rawR = analogRead(P_BATT_RESERVA);
    bateriaReservaV = (rawR / (float)ADC_MAX) * VREF / DIVISOR_RAZAO;

    // B3: bateria do sistema 12V (mesmo divisor de tensão 220k/10k)
    int rawB3 = analogRead(P_BATT_B3);
    bateriaB3V = (rawB3 / (float)ADC_MAX) * VREF / DIVISOR_RAZAO;

    // Corrente: ACS758LCB-050B → (Vadc - 2.5V) / 0.040 V/A
    int rawI = analogRead(P_CORRENTE);
    float vI = (rawI / (float)ADC_MAX) * VREF;
    correnteA = (vI - ACS758_OFFSET_V) / ACS758_SENSITIVIDADE;
    if(correnteA < 0.0f) correnteA = 0.0f;  // Não exibe corrente negativa (descarga apenas)

    // DEBUG sensores: loga ADC bruto + tensão calculada a cada ~5s (50 × 100ms)
    // Permite verificar se o divisor de tensão e a leitura ADC estão corretos.
    {
        static unsigned long dbgBatCount = 0;
        dbgBatCount++;
        if (dbgBatCount % 50 == 0) {
            Serial.print(F("[DBG ADC] B1="));   Serial.print(rawP);
            Serial.print(F("("));              Serial.print(bateriaVoltagem, 1); Serial.print(F("V)"));
            Serial.print(F(" B2="));           Serial.print(rawR);
            Serial.print(F("("));              Serial.print(bateriaReservaV, 1); Serial.print(F("V)"));
            Serial.print(F(" B3="));           Serial.print(rawB3);
            Serial.print(F("("));              Serial.print(bateriaB3V, 1);    Serial.print(F("V)"));
            Serial.print(F(" I="));            Serial.print(rawI);
            Serial.print(F("("));              Serial.print(correnteA, 2);     Serial.println(F("A)"));
        }
    }
}

// ─── CONTROLE DO MOTOR COM RAMPA PWM ─────────────────────────────────────────
void controlarMotor() {
    // Se alarme ou ignição desligada: cortar motor imediatamente
    if (alarmeDisparado || !ignitionOn) {
        pwmAtual = 0;
        analogWrite(P_MOTOR_PWM, 0);
        return;
    }

    uint8_t alvo;
    switch (nivelVelocidade) {
        case 1:  alvo = 72;  break;  // ~28% – lento
        case 2:  alvo = 160; break;  // ~63% – médio
        case 3:  alvo = 255; break;  // 100% – máximo
        default: alvo = 0;   break;
    }

    // Rampa suave para evitar picos de corrente
    if (pwmAtual < alvo) {
        pwmAtual = (uint8_t)min((int)pwmAtual + MOTOR_RAMP_STEP, (int)alvo);
    } else if (pwmAtual > alvo) {
        pwmAtual = (uint8_t)max((int)pwmAtual - MOTOR_RAMP_STEP, (int)alvo);
    }

    analogWrite(P_MOTOR_PWM, pwmAtual);
}

// ─── PROCESSAMENTO DE ALERTAS ────────────────────────────────────────────────
void processarAlertas() {
    unsigned long agora = millis();

    // ALARME CRÍTICO (roubo / bloqueio): pisca LED, motor já cortado em controlarMotor()
    if (alarmeDisparado) {
        static unsigned long tBlink = 0;
        if (agora - tBlink >= 400UL) {
            alarmePiscaEstado = !alarmePiscaEstado;
            tBlink = agora;
            digitalWrite(P_LED_ALERTA, alarmePiscaEstado ? HIGH : LOW);
        }
    } else {
        // CORREÇÃO: Garante que o LED seja desligado quando não houver alarme
        digitalWrite(P_LED_ALERTA, LOW);
        alarmePiscaEstado = false;
    }

    // CHUVA: Envia ALERT,CHUVA via serial (CYD exibe overlay).
    // NÃO usa relé 8 (que agora é Solar B1) — evita conflito de função.
    static bool chuvaAnterior = false;
    if (chuvaDetectada && !chuvaAnterior) {
        Serial1.println(F("ALERT,CHUVA"));
    } else if (!chuvaDetectada && chuvaAnterior) {
        Serial1.println(F("CLEAR,CHUVA"));
    }
    chuvaAnterior = chuvaDetectada;

    // FAROL: Envia FAROL,x ao CYD apenas quando o estado muda.
    // O relé 15 (pino 36) controla o farol. O CYD usa este status para
    // a lógica farolManualLigado (prioridade sobre LDR automático).
    bool farolAtual = (digitalRead(PINO_RELE[14]) == HIGH);  // R15 = índice 14
    if (farolAtual != farolEstadoAnterior) {
        Serial1.print(F("FAROL,"));
        Serial1.println(farolAtual ? 1 : 0);
        farolEstadoAnterior = farolAtual;
    }

    // BATERIA BAIXA: envia ALERT,SUBTENSAO (tipo reconhecido pelo CYD)
    // CORREÇÃO: Removeu-se a condição > 5.0f que impedia alertas de bateria crítica
    static unsigned long tBattAlert = 0;
    if (bateriaVoltagem < TENSAO_ALERTA_V) {
        if (agora - tBattAlert >= 5000UL) {
            Serial1.println(F("ALERT,SUBTENSAO"));
            Serial.println(F("ALERTA: Bateria B1 baixa!"));
            tBattAlert = agora;
        }
    }
    if (bateriaReservaV < TENSAO_ALERTA_V) {
        if (agora - tBattAlert >= 5000UL) {
            Serial1.println(F("ALERT,SUBTENSAO"));
            Serial.println(F("ALERTA: Bateria B2 baixa!"));
            tBattAlert = agora;
        }
    }

    // CORRENTE EXCESSIVA: > 45A → avisa CYD
    static unsigned long tCorrAlert = 0;
    if (correnteA > 45.0f && (agora - tCorrAlert >= 3000UL)) {
        Serial1.println(F("ALERT,CORRENTE"));
        tCorrAlert = agora;
    }

    // KEYOFF / KEYON: detecta transição da chave de 48V (via tensão B1)
    // Quando a chave é desligada, B1 cai abaixo de KEYOFF_VOLTAGE_THRESHOLD.
    // Aguarda KEYOFF_DEBOUNCE_MS consecutivos antes de confirmar (evita falso trigger).
    bool chaveAtualLigada = (bateriaVoltagem > KEYOFF_VOLTAGE_THRESHOLD);
    if (chaveAtualLigada && !chaveEraLigada) {
        // Transição OFF→ON: informa CYD imediatamente
        Serial1.println(F("KEYON"));
        Serial.println(F("Chave: ON (KEYON enviado)"));
        chaveEraLigada = true;
        tKeyDebounce = 0;
    } else if (!chaveAtualLigada && chaveEraLigada) {
        // Tensão caiu: inicia debounce
        if (tKeyDebounce == 0) {
            tKeyDebounce = agora;
        } else if ((agora - tKeyDebounce) >= KEYOFF_DEBOUNCE_MS) {
            // Confirmado OFF após debounce
            Serial1.println(F("KEYOFF"));
            Serial.println(F("Chave: OFF (KEYOFF enviado)"));
            chaveEraLigada = false;
            tKeyDebounce = 0;
        }
    } else {
        tKeyDebounce = 0;  // Reset debounce se tensão voltou antes do timeout
    }
}

// ─── PROCESSADOR DE COMANDO DE RELÉ (com checksum XOR) ───────────────────────
// Não usa a classe String para evitar fragmentação de heap no AVR (8KB SRAM).
// Formato esperado: R,<num>,<estado>*<XX>
void processarComandoRelay(char *cmd) {
    // Verificar checksum se presente (posição do '*')
    char *starPos = strchr(cmd, '*');
    if (starPos != NULL) {
        uint8_t chkCalc = 0;
        for (char *p = cmd; p < starPos; p++) chkCalc ^= (uint8_t)(*p);
        uint8_t chkRecv = (uint8_t)strtol(starPos + 1, NULL, 16);
        if (chkCalc != chkRecv) {
            Serial1.println(F("ERR:CHECKSUM"));
            return;
        }
        *starPos = '\0';  // Trunca checksum para o parsing
    }

    // Parse: R,<num>,<estado>
    char *tok = strtok(cmd, ",");   // "R"
    if (tok == NULL) return;
    tok = strtok(NULL, ",");        // "<num>"
    if (tok == NULL) return;
    int num = atoi(tok);
    tok = strtok(NULL, ",");        // "<estado>"
    if (tok == NULL) return;
    int estado = atoi(tok);

    if (num >= 1 && num <= (int)NUM_RELES) {
        digitalWrite(PINO_RELE[num - 1], estado ? HIGH : LOW);
        Serial1.print(F("ACK:R,"));
        Serial1.print(num);
        Serial1.print(',');
        Serial1.println(estado);
    }
}

// ─── ENVIO DE COMANDO DE RELÉ COM CHECKSUM ────────────────────────────────────
void enviarComandoRelay(int numero, int estado) {
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "R,%d,%d", numero, estado);

    uint8_t chk = 0;
    for (int i = 0; cmd[i] != '\0'; i++) chk ^= (uint8_t)cmd[i];

    char finalCmd[64];
    snprintf(finalCmd, sizeof(finalCmd), "%s*%02X\n", cmd, chk);
    Serial1.print(finalCmd);
}

// ─── PROCESSAMENTO DE COMANDOS SERIAIS (USB + CYD) ───────────────────────────
// Não usa a classe String. Usa char buffer + strtok/strcmp para parsing.
void processarComandosSerial() {
    for (int porta = 0; porta < 2; porta++) {
        Stream &s = (porta == 0) ? (Stream&)Serial : (Stream&)Serial1;
        if (!s.available()) continue;

        char buf[96];
        int n = s.readBytesUntil('\n', buf, sizeof(buf) - 1);
        buf[n] = '\0';
        // Remove CR se presente
        if (n > 0 && buf[n-1] == '\r') buf[--n] = '\0';
        if (n == 0) continue;

        // ── STATUS ──────────────────────────────────────────────────────────
        if (strcmp(buf, "?STATUS") == 0) {
            Stream &resp = (porta == 0) ? (Stream&)Serial : (Stream&)Serial1;
            resp.print(F("STATUS:READY,CHAVE,"));
            resp.print(ignitionOn ? 1 : 0);
            resp.print(F(",BATT,"));    resp.print(bateriaVoltagem, 1);
            resp.print(F(",BATTR,"));   resp.print(bateriaReservaV, 1);
            resp.print(F(",BATTB3,"));  resp.print(bateriaB3V, 1);
            resp.print(F(",AMP,"));     resp.print(correnteA, 1);
            resp.print(F(",ODO,"));     resp.print(odometroMetros);
            resp.print(F(",RPM,"));     resp.print((int)rpmAtual);
            resp.print(F(",VEL,"));     resp.println(velocidadeKmh, 1);

        // ── IGNIÇÃO ─────────────────────────────────────────────────────────
        } else if (strcmp(buf, "CHAVE:1") == 0) {
            ignitionOn = true;
            Serial.println(F("Ignicao: ON"));
            Serial1.println(F("ACK:CHAVE_ON"));

        } else if (strcmp(buf, "CHAVE:0") == 0) {
            ignitionOn = false;
            nivelVelocidade = 0;
            EEPROM.put(EEPROM_ADDR_ODO, odometroMetros);
            EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
            Serial.println(F("Ignicao: OFF (ODO salvo)"));
            Serial1.println(F("ACK:CHAVE_OFF"));

        // ── NÍVEL DE VELOCIDADE ──────────────────────────────────────────────
        } else if (strcmp(buf, "VEL:0") == 0) { nivelVelocidade = 0; }
        else if  (strcmp(buf, "VEL:1") == 0) { nivelVelocidade = 1; }
        else if  (strcmp(buf, "VEL:2") == 0) { nivelVelocidade = 2; }
        else if  (strcmp(buf, "VEL:3") == 0) { nivelVelocidade = 3; }

        // ── CRUISE CONTROL ───────────────────────────────────────────────────
        // Formato: CRUISE,ON,<vel>\n  ou  CRUISE,OFF\n
        // NOTA v4.0: O cruise control é uma funcionalidade PLACEHOLDER.
        //   O comando é reconhecido e confirmado (ACK), mas NÃO mantém a
        //   velocidade alvo. Implementação PID completa é trabalho futuro.
        //   Para usar: conecte um sensor de velocidade no Mega e implemente
        //   um laço PID usando rpmAtual como feedback e P_MOTOR_PWM como saída.
        else if (strncmp(buf, "CRUISE,", 7) == 0) {
            char tmp[96]; memcpy(tmp, buf, n+1);
            strtok(tmp, ",");           // "CRUISE"
            char *estado = strtok(NULL, ",");
            if (estado != NULL) {
                if (strcmp(estado, "ON") == 0) {
                    char *velStr = strtok(NULL, ",");
                    float velAlvo = velStr ? atof(velStr) : 0.0f;
                    Serial.print(F("[CRUISE] ON: alvo ")); Serial.print(velAlvo); Serial.println(F("km/h (PID pendente)"));
                    Serial1.println(F("ACK:CRUISE_ON"));
                } else {
                    Serial.println(F("[CRUISE] OFF"));
                    Serial1.println(F("ACK:CRUISE_OFF"));
                }
            }

        // ── PAS (Pedal Assist) ────────────────────────────────────────────────
        // Formato: PAS,<imans>\n
        } else if (strncmp(buf, "PAS,", 4) == 0) {
            int imans = atoi(buf + 4);
            Serial.print(F("PAS imans: ")); Serial.println(imans);
            Serial1.println(F("ACK:PAS_OK"));

        // ── CURVA DE SENSIBILIDADE ────────────────────────────────────────────
        } else if (strncmp(buf, "CURVA,", 6) == 0) {
            int curva = atoi(buf + 6);
            Serial.print(F("Curva: ")); Serial.println(curva);

        // ── BALANÇO DE TRAÇÃO ─────────────────────────────────────────────────
        } else if (strncmp(buf, "BALANCO,", 8) == 0) {
            int balanco = atoi(buf + 8);
            Serial.print(F("Balanco: ")); Serial.println(balanco);

        // ── LIMITADOR DE VELOCIDADE ───────────────────────────────────────────
        } else if (strncmp(buf, "LIMVEL,", 7) == 0) {
            int lim = atoi(buf + 7);
            Serial.print(F("LimVel: ")); Serial.print(lim); Serial.println(F("km/h"));

        // ── STANDBY ───────────────────────────────────────────────────────────
        } else if (strncmp(buf, "STANDBY,", 8) == 0) {
            int est = atoi(buf + 8);
            if (est == 1) {
                // CYD entrou em standby → desliga periféricos não essenciais
                Serial.println(F("Standby: ON"));
            } else {
                Serial.println(F("Standby: OFF"));
            }

        // ── A_ON / A_OFF (obsoleto desde v4.0 BT-Only — áudio A2DP removido) ────
        // Mantido por retrocompatibilidade: não executa nenhuma ação de hardware.
        } else if (strcmp(buf, "A_ON") == 0) {
            Serial.println(F("[INFO] A_ON recebido (sem acao - audio removido v4.0)"));

        } else if (strcmp(buf, "A_OFF") == 0) {
            Serial.println(F("[INFO] A_OFF recebido (sem acao - audio removido v4.0)"));

        // ── REBOOT_CLEAN ──────────────────────────────────────────────────────
        } else if (strcmp(buf, "REBOOT_CLEAN") == 0) {
            Serial.println(F("Reboot solicitado pelo CYD"));
            Serial1.println(F("ACK:REBOOT"));
            delay(100);
            // Reboot via watchdog (asm volatile)
            asm volatile ("jmp 0");

        // ── RESET_ALARM ───────────────────────────────────────────────────────
        } else if (strcmp(buf, "RESET_ALARM") == 0) {
            alarmeDisparado = false;
            Serial.println(F("Alarme resetado pelo CYD"));
            Serial1.println(F("ACK:RESET_ALARM"));

        // ── TESTE_ROUBO ───────────────────────────────────────────────────────
        } else if (strcmp(buf, "TESTE_ROUBO") == 0) {
            alarmeDisparado = true;
            Serial1.println(F("ALERT,ROUBO"));
            Serial.println(F("SIM: alarme de roubo ativado pelo CYD"));

        // ── TESTE_CHUVA ───────────────────────────────────────────────────────
        } else if (strcmp(buf, "TESTE_CHUVA") == 0) {
            Serial1.println(F("ALERT,CHUVA"));
            Serial.println(F("SIM: chuva simulada pelo CYD"));

        // ── SYNC (parâmetro genérico) ─────────────────────────────────────────
        } else if (strncmp(buf, "SYNC,", 5) == 0) {
            Serial.print(F("SYNC: ")); Serial.println(buf + 5);

        // ── COMANDOS DE SIMULAÇÃO (Monitor Serial) ────────────────────────────
        } else if (strcmp(buf, "SIM_ROUBO") == 0) {
            alarmeDisparado = true;
            Serial.println(F("SIM: alarme disparado"));
            Serial1.println(F("SIM:ROUBO_ATIVO"));

        } else if (strcmp(buf, "SIM_RESET_ALARM") == 0) {
            alarmeDisparado = false;
            Serial.println(F("SIM: alarme resetado"));

        } else if (strcmp(buf, "SIM_CHUVA") == 0) {
            chuvaDetectada = true;
            Serial.println(F("SIM: chuva ativada"));

        } else if (strcmp(buf, "SIM_BATT") == 0) {
            Serial1.println(F("ALERT,SUBTENSAO"));
            Serial.println(F("SIM: bateria baixa"));

        } else if (strcmp(buf, "SIM_RESET_ODO") == 0) {
            odometroMetros = 0UL;
            EEPROM.put(EEPROM_ADDR_ODO, odometroMetros);
            EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
            Serial.println(F("SIM: odometro zerado e EEPROM atualizada"));

        // ── BT_OK (heartbeat do ESP32 CYD) ───────────────────────────────────
        // O ESP32 envia "BT_OK\n" a cada ~2s para confirmar que está ativo.
        // Alimenta o watchdog local e marca a conexão como ativa.
        } else if (strcmp(buf, "BT_OK") == 0) {
            g_tBtHeartbeat = millis();
            g_btConectado = true;

        // ── RELÉ (R,<num>,<estado>[*<xx>]) ───────────────────────────────────
        } else if (strncmp(buf, "R,", 2) == 0) {
            processarComandoRelay(buf);

        // ── CIRCUNFERÊNCIA DA RODA ────────────────────────────────────────────
        // Formato: C,R,<cm>\n
        } else if (strncmp(buf, "C,R,", 4) == 0) {
            // Faixa válida para triciclo: 100-260cm (roda 12" a 30")
            float circCm = atof(buf + 4);
            if (circCm >= 100.0f && circCm <= 260.0f) {
                diametroRodaM   = (circCm / 100.0f) / 3.14159265f;
                circunferenciaM = circCm / 100.0f;
                EEPROM.put(EEPROM_ADDR_DIAM, diametroRodaM);
                EEPROM.update(EEPROM_DIAM_MAGIC_ADDR, EEPROM_DIAM_MAGIC_VAL);
                Serial.print(F("EEPROM: Diametro atualizado = ")); Serial.print(diametroRodaM, 4); Serial.println(F("m"));
                Serial1.println(F("ACK:CIRC_OK"));
            } else {
                Serial1.println(F("ERR:CIRC_INVALIDA"));
            }
        }
    }
}

// ─── ENVIO DE TELEMETRIA PARA O CYD (PROTOCOLO CSV BRUTO) ────────────────────
// MUDANÇA v4.0 (BT-Only):
//   Formato anterior: protocolos múltiplos com prefixos (V,, SPD,, SETA,)
//   Formato novo: CSV com valores BRUTOS do ADC — ESP32 CYD realiza todos os cálculos.
//
//   Protocolo: <rawB1>,<rawB2>,<rawB3>,<pulsos>,<chuva>,<setas>,<rawCorr>\n
//     rawB1   = leitura bruta ADC A0  (0-1023) – bateria principal B1
//     rawB2   = leitura bruta ADC A1  (0-1023) – bateria reserva B2
//     rawB3   = leitura bruta ADC A3  (0-1023) – bateria sistema 12V B3
//     pulsos  = contador de pulsos RPM desde o último envio (unsigned long)
//     chuva   = 0 (seco) ou 1 (chuva detectada)
//     setas   = 0 (off) | 1 (esq) | 2 (dir) | 3 (ambas / hazard)
//     rawCorr = leitura bruta ADC A4  (0-1023) – sensor corrente ACS758
//
//   O ESP32 CYD usa as constantes MEGA_VREF_V, MEGA_ADC_MAX e MEGA_DIV_RAZAO
//   para converter raw→tensão real (V), e circRodaCM para calcular velocidade.
//
// INTERVALO: TELE_INTERVALO_MS (500ms). A janela de pulsos coincide com este intervalo.
//
// RETROCOMPATIBILIDADE: Textos de alerta (ALERT,, CLEAR,, KEYOFF, KEYON, etc.)
//   continuam sendo enviados como linhas de texto separadas — o CYD detecta CSV
//   pela presença do primeiro campo numérico, e alertas pelo token de texto.
void enviarTelemetria() {
    unsigned long agora = millis();
    if (agora - ultimaTelemMs < TELE_INTERVALO_MS) return;
    ultimaTelemMs = agora;

    // Lê valores brutos do ADC (sem conversão — ESP32 CYD converte para grandezas físicas).
    // analogRead() retorna 0-1023 para tensão 0-5V no pino analógico do Mega 2560.
    int rawB1   = analogRead(P_BATT_PRINC);    // A0: divisor 220k/10k para B1
    int rawB2   = analogRead(P_BATT_RESERVA);  // A1: divisor 220k/10k para B2
    int rawB3   = analogRead(P_BATT_B3);       // A3: divisor 220k/10k para B3
    int rawCorr = analogRead(P_CORRENTE);      // A4: ACS758LCB-050B (0A=512@5V)

    // Leitura atômica do contador de pulsos RPM (ISR usa volatile).
    // noInterrupts() evita corrida de dados: ISR pode incrementar g_pulsos
    // no meio de uma leitura de 32-bit no AVR (não atômica por padrão).
    noInterrupts();
    unsigned long pulsos = g_pulsos;  // Copia o contador acumulado desde o último envio
    g_pulsos = 0;                     // Reseta para a próxima janela de medição
    interrupts();

    // Status do sensor de chuva: INPUT_PULLUP → LOW = molhado, HIGH = seco.
    int chuva = (digitalRead(P_SENSOR_CHUVA) == LOW) ? 1 : 0;

    // Setas: lê estado dos relés R11 (esq) e R12 (dir) diretamente dos pinos de saída.
    // HIGH = relé ativo = seta piscando. Codificação:
    //   0 = off, 1 = esquerda, 2 = direita, 3 = hazard (ambas piscam = emergência)
    bool setaEsq = (digitalRead(PINO_RELE[10]) == HIGH);  // R11 – índice 10
    bool setaDir = (digitalRead(PINO_RELE[11]) == HIGH);  // R12 – índice 11
    int  setas   = 0;
    if      (setaEsq && setaDir) setas = 3;
    else if (setaDir)            setas = 2;
    else if (setaEsq)            setas = 1;

    // Monta e envia CSV bruto em uma única linha.
    // Usar char[] + snprintf evita a classe String (fragmentação de heap no AVR 8KB SRAM).
    // "lu" = unsigned long para pulsos; "%d" para inteiros; "\n" termina o pacote.
    char buf[64];
    snprintf(buf, sizeof(buf), "%d,%d,%d,%lu,%d,%d,%d\n",
             rawB1, rawB2, rawB3, pulsos, chuva, setas, rawCorr);
    Serial1.print(buf);  // Envia ao HC-06 (Serial1 → Bluetooth → ESP32 CYD)

    // DEBUG: Loga o CSV enviado a cada ~5s (10 pacotes × 500ms)
    // Permite rastrear se o Mega está gerando dados corretos.
    // Formato: [DBG TX] rawB1 rawB2 rawB3 pulsos chuva setas rawCorr
    {
        static unsigned long dbgEnvioCount = 0;
        dbgEnvioCount++;
        if (dbgEnvioCount % 10 == 0) {
            Serial.print(F("[DBG TX] CSV #"));
            Serial.print(dbgEnvioCount);
            Serial.print(F(" rawB1="));  Serial.print(rawB1);
            Serial.print(F(" rawB2="));  Serial.print(rawB2);
            Serial.print(F(" rawB3="));  Serial.print(rawB3);
            Serial.print(F(" pulsos=")); Serial.print(pulsos);
            Serial.print(F(" chuva="));  Serial.print(chuva);
            Serial.print(F(" setas="));  Serial.print(setas);
            Serial.print(F(" rawCorr=")); Serial.println(rawCorr);
            // Log das grandezas físicas calculadas no Mega (para comparar com o ESP32)
            Serial.print(F("[DBG TX] RPM="));    Serial.print(rpmAtual, 0);
            Serial.print(F(" kmh="));            Serial.print(velocidadeKmh, 1);
            Serial.print(F(" BT="));             Serial.println(g_btConectado ? F("ON") : F("OFF"));
        }
    }
}

// ─── WATCHDOG BT DO MEGA ─────────────────────────────────────────────────────
// Monitora se o ESP32 (Master) está respondendo com BT_OK a cada ~2s.
// Se o heartbeat parar, considera que a conexão BT caiu e reinicia a Serial1.
// Com HC-06 (Slave): ao reiniciar Serial1, o HC-06 volta a aguardar nova
// conexão do ESP32. O ESP32 detectará a queda pelo seu próprio watchdog (3s)
// e tentará reconectar usando o MAC salvo na NVS.
void verificarWatchdogBT() {
    if (g_tBtHeartbeat == 0) return;  // Ainda não recebeu o primeiro BT_OK
    if (millis() - g_tBtHeartbeat > BT_WATCHDOG_TIMEOUT_MS) {
        g_btConectado = false;
        Serial.println(F("BT: timeout - reiniciando Serial1 (HC-06 aguardará nova conexão)"));
        Serial1.end();
        delay(1000);  // HC-06 precisa de 500-1000ms após perda de serial para voltar ao estado pairable
        Serial1.begin(SERIAL_CYD_BAUD);
        g_tBtHeartbeat = millis(); // Reseta timer para não repetir imediatamente
    }
}

// ─── LOOP PRINCIPAL ───────────────────────────────────────────────────────────
void loop() {
    // 1. Leitura do sensor de chuva (LOW = chuva, pois INPUT_PULLUP)
    chuvaDetectada = (digitalRead(P_SENSOR_CHUVA) == LOW);

    // 2. Cálculo de RPM e velocidade
    calcularRPM();

    // 3. Leitura das tensões de bateria
    lerBateria();

    // 4. Processamento de comandos seriais recebidos
    processarComandosSerial();

    // 5. Watchdog BT: reinicia Serial1 se ESP32 parou de responder
    verificarWatchdogBT();

    // 6. Controle do motor (com rampa)
    controlarMotor();

    // 7. Processamento de alertas (alarme, chuva, bateria baixa)
    processarAlertas();

    // 8. Envio de telemetria periódica para o CYD
    enviarTelemetria();

    // 9. Persistência do odômetro na EEPROM
    salvarOdometro();

    // Ciclo de 50ms para resposta adequada e não sobrecarregar a serial
    delay(50);
}