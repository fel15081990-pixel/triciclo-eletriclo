// =============================================================================
// VERSA 3 - COMPUTADOR DE BORDO PARA VEÍCULOS ELÉTRICOS
// Firmware do Display ESP32 CYD (Cheap Yellow Display)
// Versao: 4.1 (HC-06 Slave no Mega, ESP32 CYD como BT Master com scan)
// =============================================================================
//
// *****************************************************************************
// *         MANUAL DE MONTAGEM DO CYD - PASSO-A-PASSO (PARA LEIGOS)          *
// *****************************************************************************
//
// MATERIAIS:
//   - 1x ESP32 CYD (Cheap Yellow Display) com tela TFT 320x240 e touch
//   - Fios Dupont femea-femea (pelo menos 6 fios)
//   - 1x Resistor 1k ohm e 1x Resistor 2k ohm (protecao 3.3V)
//   - 1x Sensor LDR (foto-resistor) + 1x Resistor 10k ohm (pull-down)
//
// ---------------------------------------------------------------------------
// PASSO 1: LIGACAO BLUETOOTH COM O ARDUINO MEGA (v4.1 HC-06 Master)
// ---------------------------------------------------------------------------
//   O CYD "conversa" com o Mega EXCLUSIVAMENTE via Bluetooth Serial (SPP).
//   NÃO HÁ FIOS SERIAIS entre o ESP32 e o Mega — a comunicação é wireless.
//
//   HARDWARE NECESSÁRIO:
//     - Arduino Mega 2560 + módulo HC-06 (SLAVE, sempre aguarda conexão)
//     - ESP32 CYD (BluetoothSerial como MASTER, nome "VERSA3_CYD")
//
//   CONEXÃO HC-06 AO MEGA:
//     HC-06 VCC   → Mega 5V (ou 3.3V dependendo do módulo)
//     HC-06 GND   → Mega GND
//     HC-06 TXD   → Mega RX1 (pino 19)
//     HC-06 RXD   → Mega TX1 (pino 18)
//
//   CONFIGURAÇÃO DO HC-06 (via AT commands em 9600 baud SEM jumper):
//     AT+NAME=VERSA3_MEGA_HC06  → nome visível no scan do CYD
//     AT+PIN=1234               → PIN de pareamento
//     AT+BAUD4                  → confirma 9600 baud (padrão de fábrica)
//     Nota: HC-06 NÃO possui comando AT+ROLE (é sempre Slave por hardware).
//
//   FLUXO AUTOMÁTICO (v4.1):
//     1. CYD liga → carrega MAC do HC-06 salvo na NVS
//     2. Se MAC salvo → SerialBT.connect(mac) automaticamente
//     3. Se sem MAC → usuário usa PAREAMENTO BT para scan e seleção
//     4. Mega envia CSV a cada 500ms; CYD recebe, converte e exibe
//     5. Se conexão cair: CYD watchdog (3s) tenta SerialBT.connect(mac)
//     6. Menu BLUETOOTH → "Buscar Dispositivos" refaz o scan quando necessário
//
// *****************************************************************************
// *                    FIM DO MANUAL DE MONTAGEM DO CYD                       *
// *****************************************************************************
//
// ===================== VISÃO GERAL DO PROJETO ================================
// Este firmware controla a interface gráfica (IHM) de um computador de bordo
// para veículos elétricos. Ele roda num módulo ESP32 CYD (Cheap Yellow Display)
// com tela TFT 320x240 pixels e touch capacitivo integrado.
//
// O ESP32 CYD se comunica via UART (Serial 115200 baud) com um Arduino Mega
// 2560, que é o controlador principal do veículo (relés, sensores, motores).
//
// ===================== HARDWARE ENVOLVIDO ====================================
//
// 1. ESP32 CYD (ESTE FIRMWARE):
//    - SoC: ESP32-WROOM-32 (dual-core 240MHz, 520KB SRAM, 4MB Flash)
//    - Display: TFT ILI9341 320x240px, touch XPT2046 (SPI)
//    - Biblioteca gráfica: LVGL (Light and Versatile Graphics Library)
//    - Armazenamento: NVS (Non-Volatile Storage) via Preferences.h
//    - Pino 21: Controle de backlight (HIGH = ligado, LOW = desligado)
//    - Pino 36: Sensor LDR (farol automático, leitura analógica 0-4095)
//    - Comunicação: Bluetooth SPP (BluetoothSerial MASTER) → HC-06 no Mega
//    - BT Watchdog: 3s sem dados → tenta SerialBT.connect(savedMac)
//    - BT Manual: Menu PAREAMENTO BT → scan de dispositivos → selecionar HC-06
//
// 2. ARDUINO MEGA 2560 (firmware separado: VERSA3_MEGA.ino):
//    - Microcontrolador ATmega2560 (16MHz, 8KB SRAM, 256KB Flash)
//    - HC-06 na Serial1 (pinos 18/19) como SLAVE Bluetooth
//    - 15 relés nos pinos digitais 22 a 36
//    - Sensores conectados ao Mega:
//      * Divisor de tensão: Resistores 220kΩ + 10kΩ (1/4W, 1%)
//        Fator de divisão = (220k + 10k) / 10k = 23.0
//        Permite medir até ~75V em pinos analógicos (5V máx no ADC Mega)
//      * Sensor de corrente: ACS758LCB-050B (50A, efeito Hall)
//        Sensibilidade: 40mV/A, offset: 2.5V (bidirecional)
//        Mede APENAS corrente de tração (Baterias B1 e B2)
//      * GPS: Módulo Neo-6M V2 (Serial1 do Mega, 9600 baud)
//      * Temperatura/Umidade: DHT11 (digital, leitura a cada 2s)
//      * Chuva: Sensor MingWu (saída digital + analógica, 3.3-5V)
//      * Ultrassônico: JSN-SR04T 2.0 (prova d'água, pisca LED)
//      * Velocidade: Sensor magnético 2 fios (INT0, pino 2 do Mega)
//      * PAS (Pedal Assist): Sensor magnético (INT1, pino 3 do Mega)
//        Configurável de 5 a 24 ímãs via menu
//      * Buzzer: Módulo passivo 5V (alerta sonoro)
//
// 3. ESP32-S3 (firmware separado: VERSA3_S3_REMOTE.ino):
//    - Terminal portátil de controle remoto
//    - Comunicação RF: Módulo HC-12 433MHz (alcance até 1000m)
//    - Pacotes com sync header 0xAA, 0x55
//
// ===================== MAPA DOS 15 RELÉS =====================================
// Relé  Pino(Mega) Função                      Variável CYD
// R1    22         Motor 1x3/3x3               modo3x3
// R2    23         Limitador de velocidade      limitadorLigado
// R3    24         Seleção bateria (B1/B2)      bateriaEmUso
// R4    25         Marcha ré                    modoReLigado
// R5    26         Velocidade nível 1           velocidadeAtual
// R6    27         Velocidade nível 2           velocidadeAtual
// R7    28         Luz interna (acessório)      (standby desliga)
// R8    29*        Carregamento solar B1        solarLigado
// R9    30*        Carregamento solar B2        solarLigado
// R10   31         Ventilador (acessório)       (standby desliga)
// R11   32         Seta esquerda                (controle manual)
// R12   33         Seta direita                 (controle manual)
// R13   34         Proteção bateria B3 (<11.5V) (automático)
// R14   35         Freio (sinal)                (sensor Mega)
// R15   36         Farol (acessório)            (standby desliga)
// * R8/R9 são controlados DIRETAMENTE pelo CYD nos pinos 26/27
//
// ===================== PROTOCOLO SERIAL (MEGA ↔ CYD) =========================
// Mega → CYD:  V,<B1>,<B2>,<B3>,<Amp>\n   (telemetria de energia)
//              READY\n                       (confirmação boot limpo)
//              STANDBY,<0|1>\n              (modo economia)
//              ALERT,<tipo>\n               (alerta furto/temperatura)
// CYD → Mega:  R,<num>,<estado>\n           (comando de relé)
//              RESET\n                       (reset geral de emergência)
//              REBOOT_CLEAN\n               (verificar integridade)
//              C,R,<valor>\n                (circunferência da roda)
//              PAS,<imas>\n                 (configuração PAS)
//              CURVA,<idx>\n                (curva de aceleração)
//              BALANCO,<val>\n              (balanço motores)
//              CRUISE,ON,<vel>\n / CRUISE,OFF\n
//              TESTE_ROUBO\n               (simula alerta de roubo - modo teste)
//              TESTE_CHUVA\n               (simula alerta de chuva - modo teste)
//              RESET_ALARM\n               (limpa todos os alertas de teste)
//
// ===================== MÓDULOS IMPLEMENTADOS =================================
// Módulo A: Central de Diagnóstico e Reset de Relés (menu MANUTENÇÃO)
// Módulo B: Sistema HC-12 RF (receptor de comandos remotos no Mega)
// Módulo C: Interface Cruise Control (0-100 km/h) e PAS (0-30 ímãs)
// Módulo D: Monitoramento de Energia (B1/B2 tração, B3 sistema fixo 12V)
// Módulo E: Standby Manual (botão físico no Mega via Serial, wake-on-touch/motion)
// Módulo F: Splash Screen de inicialização (NÃO bloqueia, sem teste boot)
// Módulo G: Preço por Quilômetro (telemetria dual-battery, custo/km por bat/força/tração)
// Módulo H: Radar de Proximidade Traseiro (auto-ativação por R4 / marcha ré)
// Módulo I: Carrossel de Alarmes (ciclo de 3s, prioridade para setas/emergência)
// Módulo J: REMOVIDO (v4.0) – Bluetooth A2DP Sink de áudio eliminado.
//           O BT agora é EXCLUSIVO para dados de sensores (SPP/BluetoothSerial).
//           Todos os pinos GPIO 25/26 ficam livres (não usar para relés).
// Módulo K: Balanço de Tração (-5/+5, feedback visual, conversão para Mega)
// Módulo L: REMOVIDO (RTC/Data/Hora eliminados - Seção 1 refatoração)
//
// ===================== NOVAS MELHORIAS (v3.3 - ESTABILIDADE) ==================
// 1.  Balanço de tração: escala -5 a +5 (antes 0-100), com feedback visual
// 2.  Cruise control: slider 0-100 km/h (antes 10-40), botão VERDE/VERMELHO
// 3.  PAS: roller 0-30 (antes slider 5-24), com subtítulo explicativo
// 4.  Radar automático: auto-switch para RADAR quando R4 ativa, auto-retorno
// 5.  Ciclo de alarmes: carrossel de 3s para alertas secundários
// 6.  Standby manual: timer automático REMOVIDO, controle via botão físico no Mega
// 7.  Wake-on-touch: tela acorda ao toque enquanto backlight desligado
// 8.  PRECO_POR_KM: custo/km por bateria, força e modo tração (NVS + mutex)
// 9.  Submenus: Carregadores (B1/B2), Preço Energia, Histórico por bat/força
// 10. Menu Manutenção: container scrollável (Agenda de Revisão visível)
// 11. TESTE_AUTOMATICO: botão Home removido, Reset reposicionado
// 12. Scroll: LV_SCROLL_SNAP_NONE em listas (TESTE_ALERTAS, TESTE_MANUAL, etc.)
// 13. Interface gráfica LVGL com temas claro/escuro
// 14. Documentação exaustiva: cabeçalhos de função, dicionário de variáveis
// 15. Auditoria completa: 10 itens verificados linha por linha
//
// ===================== MELHORIAS DE ESTABILIDADE (v3.3) =======================
// 16. DUAL-CORE: Core 1 exclusivo LVGL, Core 0 para telemetria (FreeRTOS)
// 17. WDT PREVENTION: vTaskDelay(5ms/10ms) em ambos os cores
// 18. NVS DIRTY CHECK: salvarDadosVeiculo/salvarConfiguracoes so escrevem se mudou
// 19. SPI MUTEX: xSemaphore protege barramento compartilhado display/touch
// 20. I2S COMPAT: compilacao condicional para Core 3.x e 2.0.17
// 21. VOLATILE: variaveis compartilhadas entre cores marcadas como volatile
// 22. DOCUMENTACAO: cada bloco comentado com O QUE FAZ, PINOS, OTIMIZACAO, WDT
//
// ===================== REGRAS DE CÓDIGO ======================================
// PROIBIDO: Uso da classe String (usa char arrays + snprintf)
// PROIBIDO: snprintf com %f no AVR (usa dtostrf - só afeta MEGA)
// Comentários: 100% em português
// Persistência: Preferences.h (NVS do ESP32) para configs e km
// =============================================================================

#include <LVGL_CYD.h>       // Biblioteca wrapper do LVGL para o CYD
#include <Preferences.h>    // Armazenamento não volátil (NVS) do ESP32
#include "BluetoothSerial.h" // Bluetooth SPP nativo do ESP32 (Master → HC-06 Slave)
#include "esp_gap_bt_api.h"  // ESP-IDF GAP: discovery (scan), autenticação, PIN reply

// =============================================================================
// FREERTOS DUAL-CORE - INCLUDES E CONFIGURACAO
// =============================================================================
// O QUE FAZ: Inclui os headers do FreeRTOS para gerenciamento de tarefas e
//   semaforos. O ESP32 possui 2 nucleos (Core 0 e Core 1) que podem executar
//   tarefas simultaneamente. Sem separacao adequada, uma operacao bloqueante
//   (ex: Serial.readBytesUntil) travaria o LVGL e causaria WDT reset.
//
// ARQUITETURA DUAL-CORE IMPLEMENTADA:
//   Core 1 (App Core): loop() do Arduino - EXCLUSIVO para LVGL
//     * lv_timer_handler() a cada 5ms (renderizacao + touch)
//     * Atualizacao de labels, overlays, splash screen
//     * Prioridade: 1 (padrao do Arduino loop)
//     * vTaskDelay(5ms) a cada iteracao para alimentar o WDT
//
//   Core 0 (Pro Core): tarefaTelemetriaFunc() - FreeRTOS task dedicada
//     * Leitura Serial do Mega (lerDadosUno)
//     * Envio de comandos de reles (enviarStatusReles)
//     * Controle solar, standby, farol hibrido
//     * Calculos de economia e relogio RTC
//     * Salvamento periodico na NVS (com dirty check)
//     * Prioridade: 2 (alta, mas nao interfere no LVGL)
//     * vTaskDelay(10ms) a cada iteracao para alimentar o WDT
//
// PINOS/HARDWARE: Nenhum pino extra. Usa recursos internos do ESP32.
// OTIMIZACAO: Elimina travamentos por operacoes bloqueantes no loop principal.
// WDT: vTaskDelay() em ambas as tasks alimenta o watchdog automaticamente.
// =============================================================================
#include <freertos/FreeRTOS.h>   // Kernel FreeRTOS (tasks, scheduler)
#include <freertos/task.h>       // xTaskCreatePinnedToCore, vTaskDelay
#include <freertos/semphr.h>     // xSemaphoreCreateMutex, xSemaphoreTake/Give

// ---------------------------------------------------------------------------
// NOTA v4.0 (BT-Only): O Bluetooth A2DP (áudio) foi completamente removido.
// O ESP32 CYD usa EXCLUSIVAMENTE o BluetoothSerial (SPP) para comunicação
// de dados com o Arduino Mega via HC-05. Os pinos GPIO 25 e 26 (DAC de
// áudio) ficam livres mas NÃO são usados para relés ou outros propósitos.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// PINOS DO ESP32 CYD
// ---------------------------------------------------------------------------
// Seção 5: ANÁLISE DE CONFLITOS DE PINAGEM CONCLUÍDA
// Pin 21 (Backlight): NÃO conflita com SPI (SPI usa 18/23/19/15/33)
// Pin 26 (DAC2 Audio): RESOLVIDO em v3.2 - R8 movido para serial ao Mega
// Pin 27 (PSRAM CS no ESP32-WROVER): RESOLVIDO em v3.1 - R9 movido para serial ao Mega
// MUDANÇA v3.1/v3.2:
//   O Relé 8 (Solar B1) era controlado diretamente pelo GPIO 26 do CYD.
//   Com A2DP Sink, GPIO 25/26 são usados pelo DAC interno de áudio.
//   O Relé 9 (Solar B2) era controlado pelo GPIO 27.
//   No ESP32-WROVER, GPIO27 = PSRAM CS: escrita neste pino corromperia a PSRAM.
//   Ambos R8 e R9 agora são controlados via protocolo serial ao Mega.
//
// GPIO 25 = DAC1 (canal esquerdo de áudio) — NÃO USAR PARA OUTRO FIM
// GPIO 26 = DAC2 (canal direito de áudio)  — NÃO USAR PARA OUTRO FIM
// GPIO 27 = PSRAM CS (ESP32-WROVER)        — NÃO USAR PARA OUTRO FIM
#define PIN_BACKLIGHT 21     // GPIO21: Backlight do display (Módulo E: standby)

// =============================================================================
// ESTRUTURAS DE DADOS
// =============================================================================
// Telemetria: armazena dados de consumo energético por fonte/motor.
// Cada índice do array dados[5] corresponde a:
//   [0] Bateria Principal (B1)
//   [1] Bateria Reserva (B2)
//   [2] Motor 1
//   [3] Motor 2
//   [4] Extra/Livre
// Todos os campos podem ser zerados individualmente pelo usuário
// via long-press na tela de detalhes de consumo.
struct Telemetria {
    float kmTotal;                // Distância total percorrida nesta fonte (km)
    float kmPorConsumo;           // Quilômetros por ampere-hora (eficiência)
    float eficiencia;             // Km/A - razão distância/consumo
    uint32_t tempoSegundos;       // Tempo total de uso acumulado (segundos)
    float picoAmperagem;          // Maior corrente registrada (Amperes)
    float consumoMedioAmperagem;  // Média de consumo de corrente (Amperes)
};

// CalibPoint: ponto de calibração do touch screen.
// 5 pontos são coletados: 4 cantos + centro da tela.
// Usados para calcular scaleX, scaleY, offsetX, offsetY.
struct CalibPoint { int x; int y; };

// Limites: tensão mínima e máxima de uma bateria.
// Calculados com base na química (Chumbo/Lítio/LiFePO4) e número de células.
struct Limites { float vMin; float vMax; };

// ConfigBateria: configuração individual para cada uma das 3 baterias.
// Armazenada na NVS (namespace "cyd_storage") para persistência.
struct ConfigBateria {
    int sistema;       // Tensão nominal do sistema (12, 24, 36, 48, 60, 72V)
    float fatorCalib;  // Fator de calibração = V_multímetro / V_bruta_ADC
    int quimica;       // 0=Chumbo-Ácido, 1=Lítio-íon, 2=LiFePO4
    int capacidade;    // Capacidade em Ah (1 a 50)
};

// =============================================================================
// ENUMERAÇÃO DE PÁGINAS (NAVEGAÇÃO DA INTERFACE)
// =============================================================================
// Cada valor representa uma tela diferente do menu.
// A navegação lateral (botões < >) percorre as páginas 0..13.
// Submenus são acessados pelo botão central e navegados com VOLTAR/HOME.
// As páginas dos módulos A e C ficam acessíveis apenas via submenu.
enum Pagina {
    PRINCIPAL, RE, LIMITADOR, VELOCIDADE, MOTOR, VOLTAGEM, RODA,
    BATERIA, KM, CONSUMO, ACIONAR, CALIBRAGEM, SOLAR,
    // Sub-páginas originais
    CONFIG_BAT, AJUSTE_ESPECIFICO, SEL_VOLTAGEM, SEL_QUIMICA, SEL_CAPACIDADE, CALIBRAR_V,
    SEL_BATERIA_ATIVA,
    // Telas integradas do módulo de Consumo
    MENU_PRINCIPAL_CONS, MENU_MOTORES_CONS, SUBMENU_DETALHES_CONS,
    // Telas integradas do módulo Solar
    MENU_SOLAR, SOLAR_TOGGLE, SOLAR_AJUSTE,
    // MÓDULO A: Diagnóstico e Testes
    MANUTENCAO_MENU, TESTE_AUTOMATICO, TESTE_MANUAL,
    // MÓDULO C: Condução e Cruise Control
    CONDUCAO_MENU, CRUISE_CONTROL, CONFIG_PAS, CONFIG_SENSIBILIDADE,
    CONFIG_TRAVA,           // Submenu: Trava (switch liga/desliga)
    CONFIG_LIMITE_VEL,      // Submenu: Limitador de velocidade (roller 0-80 km/h)
    CONFIG_TEMAS,           // Submenu: Alternância Tema Claro/Escuro
    // NOVAS TELAS: Alertas, Data/Hora, Bluetooth, Música
    TESTE_ALERTAS,         // Submenu de teste de alertas e ícones
    // CONFIG_DATETIME removida (Seção 1: sem RTC/Data/Hora)
    AGENDA_MANUTENCAO,     // Agenda de manutenção programada
    // RADAR DE PROXIMIDADE TRASEIRO (HC-SR04 / JSN-SR04T)
    RADAR_PROXIMIDADE,      // Gauge visual de distância traseira
    // MÓDULO G: PREÇO POR QUILÔMETRO (substituiu Economia/Carga)
    PRECO_POR_KM,           // Menu principal: Carregadores, Preço Energia, Histórico
    PRECO_CARREGADORES,     // Submenu: Carregador B1 / Carregador B2
    PRECO_CARREG_EDIT,      // Roller 0-1000W para editar potência do carregador
    PRECO_ENERGIA,          // Roller 0.0-10.0 R$/kWh
    PRECO_HIST_BAT,         // Histórico: seleção de bateria (B1/B2)
    PRECO_HIST_FORCA,       // Histórico: seleção de força (F1/F2/F3)
    PRECO_HIST_RESULT,      // Histórico: resultado custo/km por tração
    // PAREAMENTO BLUETOOTH (v4.0 BT-Only)
    PAREAMENTO_BT,          // Menu de pareamento manual Bluetooth (watchdog / reconexão)
    BLUETOOTH_MENU,         // Tela de status Bluetooth (conectado/desconectado, reconexão forçada)
    // MENU HODÔMETRO: Nova tela Home (painel velocímetro estilo automotivo)
    MENU_HODOMETRO
};

Pagina paginaAtual = MENU_HODOMETRO;  // Inicia no Hodômetro (nova Home)
bool emSubMenu = false;

// =============================================================================
// SISTEMA DE NAVEGAÇÃO CORRIGIDO
// =============================================================================
// BUG CORRIGIDO: Antes, a navegação usava (Pagina)(index % totalPaginas)
// que mapeava index 13 → CONFIG_BAT (enum 13) em vez de MANUTENCAO_MENU.
// Agora usa um array explícito de páginas navegáveis.
const Pagina navPaginas[] = {
    MENU_HODOMETRO, PRINCIPAL, RE, LIMITADOR, VELOCIDADE, MOTOR, VOLTAGEM, RODA,
    BATERIA, KM, CONSUMO, ACIONAR, CALIBRAGEM, SOLAR,
    MANUTENCAO_MENU, CONDUCAO_MENU, RADAR_PROXIMIDADE,
    PRECO_POR_KM
};
const int totalNavPaginas = 18;

// Labels do menu (indexados pela posição em navPaginas[], NÃO pelo enum!)
const char* menuLabels[] = {
    "HODOMETRO", "PAINEL PRINCIPAL", "MODO RE", "LIMITADOR VEL.", "NIVEL VELOCIDADE",
    "SELECAO MOTOR", "ENERGIA ATUAL", "CIRCUNF. RODA (CM)", "CONFIGURACAO BATERIA",
    "QUILOMETRAGEM", "DADOS CONSUMO", "ACIONAR RELES", "CALIBRAGEM TOUCH", "PAINEL SOLAR",
    "MANUTENCAO/TESTES", "CONDUCAO", "RADAR TRASEIRO", "PRECO POR KM"
};

// Retorna o índice de navegação para uma dada página
// Usado para exibir o label correto na interface
int obterNavIndex(Pagina p) {
    for(int i = 0; i < totalNavPaginas; i++) {
        if(navPaginas[i] == p) return i;
    }
    return 0;
}

// =============================================================================
// VARIÁVEIS GLOBAIS - ESTADO DO VEÍCULO
// =============================================================================
// Estas variáveis refletem o estado atual dos relés e configurações.
// São sincronizadas com o Arduino Mega via comandos Serial R,<num>,<estado>\n.
bool modoReLigado = false;      // R4: Marcha ré ativa (pino 25 do Mega)
bool modo3x3 = false;            // R1: Motor 3x3 (pino 22 do Mega)
bool limitadorLigado = false;    // R2: Limitador de velocidade (pino 23 do Mega)
int bateriaEmUso = 1;            // R3: 1=B1 (principal), 2=B2 (reserva) (pino 24 do Mega)
int velocidadeAtual = 1;         // R5/R6: Nível 1/2/3 (pinos 26/27 do Mega)

// --- Dados de telemetria recebidos do Mega via Serial ---
// Formato: V,<B1>,<B2>,<B3>,<Amp>\n
// B1/B2: tensão das baterias de tração (divisor 220kΩ/10kΩ, fator 23.0)
// B3: tensão da bateria de sistema fixa 12V (mesma fórmula)
// Amp: corrente de tração medida pelo ACS758LCB-050B (50A, 40mV/A)
float circRodaCM = 196.0;        // Circunferência da roda em CM (salva NVS) — 196cm = roda 26" típica
volatile float vB1 = 0, vB2 = 0, vB3 = 0;  // DUAL-CORE: volatile pois Core 0 escreve, Core 1 le // Tensões das 3 baterias (Volts)
volatile float ampere = 0;                 // DUAL-CORE: volatile - atualizado pela telemetria (Core 0)                // Corrente de tração (Amperes, só B1+B2)
volatile float kmh = 0;                    // DUAL-CORE: volatile - atualizado pela telemetria (Core 0)                   // Velocidade atual (km/h, sensor magnético)
volatile float autonomia = 0;              // DUAL-CORE: volatile - calculado pela telemetria (Core 0)             // Autonomia estimada (km)
float kmTotal = 0, kmParcial = 0; // Quilometragem total e parcial (salvas NVS)
volatile int rpm = 0;                      // DUAL-CORE: volatile - atualizado pela telemetria (Core 0)                     // Rotações por minuto da roda

// --- Configuração de baterias ---
int batEdicao = 0;               // Índice da bateria sendo editada (0/1/2)
float voltagemReferenciaMultimetro = 0.0; // Valor do multímetro p/ calibração
ConfigBateria confBat[3];        // Configuração das 3 baterias (NVS)

// Pinos analógicos do ESP32 CYD para leitura direta de tensão
// NOTA: A maior parte da telemetria vem do Mega via Serial.
// Estes pinos são usados apenas para leitura local de backup.
const int pinoVoltagem = 34;     // GPIO34: ADC1_CH6 (somente leitura, input-only)
const int pinoCorrente = 35;     // GPIO35: ADC1_CH7 (somente leitura, input-only)

// --- Calibração do touch screen ---
// 5 pontos: 4 cantos + centro. Usados para calcular escala e offset.
CalibPoint pontos[5];            // Coordenadas raw dos 5 toques de calibração
int etapa = 0;                   // Etapa atual da calibração (0-5)
float scaleX = 1.0, scaleY = 1.0; // Fator de escala X/Y (calc. na calibração)
int offsetX = 0, offsetY = 0;   // Offset X/Y (compensação de centro)
Preferences prefs;               // Objeto NVS (Preferences) do ESP32

// --- Ponteiros para labels do painel principal ---
// Criados em montarInterface() quando paginaAtual == PRINCIPAL.
// ATENÇÃO: São NULL durante a splash screen! Não acessar antes de montarInterface().
// BUG CORRIGIDO: atualizarDadosPrincipais() agora só executa quando !splashScreenAtiva.
lv_obj_t * label_baterias, * label_dados, * label_status, * label_km, * label_rodape, * label_info, * calib_marker;

// --- Ponteiros para labels do MENU HODÔMETRO (nova tela Home) ---
// Criados em montarInterface() quando paginaAtual == MENU_HODOMETRO.
lv_obj_t * hodo_lbl_velocidade = NULL;  // Centro: velocidade grande
lv_obj_t * hodo_lbl_autonomia = NULL;   // Sup. esquerdo: autonomia estimada
lv_obj_t * hodo_lbl_energia = NULL;     // Sup. direito: voltagem + %
lv_obj_t * hodo_lbl_km = NULL;          // Inf. esquerdo: ODO + TRIP
lv_obj_t * hodo_lbl_motor = NULL;       // Inf. direito: modo motor + limitador + vel

// --- Sistema de Temas (Claro/Escuro) ---
bool temaEscuro = true;                 // true = Dark Mode (default), false = Light Mode
// --- Modo Disfarce (Filtro Visual de Velocidade) ---
// FUNCIONAMENTO:
//   Se velocidade real < limiteDisfarce → visor mostra velocidade real (acompanha aceleração/desaceleração)
//   Se velocidade real >= limiteDisfarce → visor TRAVA no limiteDisfarce
//   Exemplo: limiteDisfarce=35, real=50 → visor mostra 35
//            limiteDisfarce=35, real=20 → visor mostra 20 (normal)
// IMPORTANTE: O Modo Disfarce NÃO afeta o Relé R2 (Limitador Físico).
//   São funções INDEPENDENTES. O disfarce é APENAS visual.
bool modoDisfarce = false;              // true = filtro visual ativo
int limiteDisfarce = 35;                // Velocidade máxima visível no visor (km/h)

// --- Limitador de Velocidade FÍSICO (Relé R2 no Mega) ---
// FUNCIONAMENTO:
//   Quando limitadorLigado==true: envia R,2,0 ao Mega (R2 acionado = corte real)
//   Também envia LIMVEL,<valor> para o Mega definir o corte físico.
//   limiteVelocidade define o valor em km/h do corte real no hardware.
// DIFERENÇA DO DISFARCE: Este REALMENTE limita o veículo via hardware.
int limiteVelocidade = 25;              // Limite físico em km/h (enviado ao Mega)

// Fontes LVGL usadas na interface
// montserrat_22: texto padrão de menus e informações
// montserrat_28: títulos e valores destacados
#define FONTE_MENU_PRINCIPAL &lv_font_montserrat_22
#define FONTE_GRANDE &lv_font_montserrat_28

// Globais do módulo de Consumo (originais)
int itemSelecionado = 0;
Telemetria dados[5]; // 0:Principal, 1:Reserva, 2:Motor1, 3:Motor2, 4:Extra/Livre

// Variáveis do módulo Solar (originais)
bool solarLigado = false;
float ajustePorcentagem = 50.0;
static bool estadoAnteriorRele8 = false;
static bool estadoAnteriorRele9 = false;

// =============================================================================
// MÓDULO A: DIAGNÓSTICO E RESET - VARIÁVEIS
// =============================================================================
// O teste automático sequencia os 15 relés (1 por segundo), ligando e
// desligando cada um. AVISO: o veículo deve estar em cavaletes!
// O teste manual permite controlar cada relé individualmente via switch.
bool testeAutomaticoAtivo = false;    // true = sequência de teste em andamento
int testeAutomaticoIndice = 0;        // Relé atual sendo testado (1-15)
unsigned long testeAutomaticoTimer = 0; // Timestamp do último avanço de relé

// Descrições dos 15 relés (usadas na interface de teste manual e automático)
// Cada string mapeia para o relé correspondente no Arduino Mega (pinos 22-36)
const char* nomeReles[15] = {
  "R1: Motor 1x3/3x3",
  "R2: Limitador",
  "R3: Bateria Uso",
  "R4: Marcha Re",
  "R5: Velocidade 1",
  "R6: Velocidade 2",
  "R7: Luz Interna",
  "R8: Solar B1",
  "R9: Solar B2",
  "R10: Ventilador",
  "R11: Seta Esquerda",
  "R12: Seta Direita",
  "R13: Protecao B3",
  "R14: Freio",
  "R15: Farol"
};

bool estadoRelesTeste[15] = {false};

// =============================================================================
// MÓDULO C: CONDUÇÃO - VARIÁVEIS
// =============================================================================
// Cruise Control: mantém velocidade constante usando PID no Mega.
// PAS (Pedal Assist System): sensor magnético com 5 a 24 ímãs no pedal.
// Curva de sensibilidade: define como o acelerador responde ao PAS.
// Balanço: distribuição de potência entre motores (frente/traseiro).
bool cruiseAtivo = false;           // true = piloto automático ativo
float cruiseVelocidadeAlvo = 25.0;  // Velocidade alvo do cruise (0-100 km/h)
int pasImans = 12;                  // Número de ímãs do sensor PAS (0-30)
int curvaSensibilidade = 1;         // 0=Suave, 1=Linear, 2=Agressiva
// ---------------------------------------------------------------------------
// BALANÇO DE TRAÇÃO (ESCALA -5 a +5)
// ---------------------------------------------------------------------------
//   -5 = 100% traseiro, 0 = equilibrado (50/50), +5 = 100% dianteiro
//   Conversão para o Mega (que espera 0-100): valorMega = (balanco + 5) * 10
//   Exemplo: -5 -> 0, 0 -> 50, +5 -> 100
int balancoTracao = 0;              // -5 a +5: distribuição frente/traseiro

// =============================================================================
// MÓDULO E: STANDBY - VARIÁVEIS
// =============================================================================
volatile bool modoStandby = false;          // DUAL-CORE: volatile - alterado por Serial (Core 0) e UI (Core 1)
volatile bool alertaCritico = false;        // DUAL-CORE: volatile - set por Serial (Core 0)
bool wakeupTouchBlock = false;              // Seção 2: bloqueia clique acidental ao acordar
unsigned long wakeupTouchBlockTime = 0;     // Seção 2: timestamp do wake-up

// --- Standby 10 minutos: desliga periféricos se velocidade == 0 por 10 min ---
unsigned long standbyTimerInicio = 0;  // millis() quando velocidade ficou 0
bool standbyTimerAtivo = false;         // true = contando 10 min
bool farolManualLigado = false;         // Status do botão físico do farol (Mega)
bool farolLDRAtivo = false;             // true = LDR detectou escuro
const int PIN_LDR = 36;                // GPIO36: ADC para sensor de luz (LDR)
const int STANDBY_TIMEOUT_MS = 600000; // 10 minutos em ms

// =============================================================================
// TIMER 20s POS-CHAVE (KEY-OFF) - Lógica de desligamento suave
// =============================================================================
// Quando a chave de 48V é desligada (tensão B1 cai abaixo de ~5V no divisor),
// o CYD detecta a queda e inicia um timer de 20 segundos.
// Durante esses 20s, aparece um diálogo LVGL perguntando:
//   "Deseja manter o sistema ligado?"
//   [SIM] → sistema fica ativo, botão DESLIGAR MANUAL aparece no menu
//   [NÃO] ou timeout → envia comando STANDBY,1 ao Mega, desliga backlight
// NOTA: A leitura de tensão vem do Mega via Serial (variável vB1).
bool chaveDesligadaDetectada = false;   // true = tensão B1 caiu (chave off)
bool timerKeyOffAtivo = false;          // true = contando os 20 segundos
unsigned long timerKeyOffInicio = 0;    // millis() quando a queda foi detectada
bool sistemaKeepAlive = false;          // true = utilizador escolheu SIM (manter ligado)
bool dialogoKeyOffVisivel = false;      // true = diálogo LVGL está na tela
lv_obj_t * dialogoKeyOffObj = NULL;     // ponteiro para o diálogo LVGL
lv_obj_t * lblTimerKeyOff = NULL;       // ponteiro para o label de contagem regressiva no diálogo
#define KEYOFF_TIMEOUT_MS 20000         // 20 segundos para decidir
#define KEYOFF_VOLTAGE_THRESHOLD 5.0    // Abaixo disto = chave desligada

// =============================================================================
// MODO TESTE: Variáveis de controle
// =============================================================================
// Quando o utilizador entra no menu TESTE_ALERTAS e aciona "Teste Roubo"
// ou "Teste Chuva", o CYD envia o comando real ao Mega (TESTE_ROUBO ou
// TESTE_CHUVA). O Mega propaga para todos os dispositivos (CYD + S3).
// Ao sair do menu de teste, o CYD envia RESET_ALARM ao Mega para limpar.
bool modoTesteAtivo = false;  // true = estamos dentro do menu de testes

// =============================================================================
// SISTEMA DE ALERTAS (OVERLAY TRANSPARENTE)
// =============================================================================
// Alertas são exibidos em lv_layer_top() para serem visíveis em qualquer tela.
// Cada alerta tem uma flag booleana. Quando true, o overlay é mostrado.
// O overlay é semi-transparente (bg_opa 180) com texto/ícone centralizado.
volatile bool alertaSetaEsq = false;       // DUAL-CORE: volatile - set por Serial (Core 0), lido pelo overlay (Core 1)       // Seta esquerda piscando
volatile bool alertaSetaDir = false;       // DUAL-CORE: volatile       // Seta direita piscando
volatile bool alertaEmergencia = false;    // DUAL-CORE: volatile    // Ambas setas piscando (alerta/emergência)
volatile bool alertaChuva = false;         // DUAL-CORE: volatile         // Pista escorregadia
volatile bool alertaTemperatura = false;   // DUAL-CORE: volatile   // Alta temperatura motor/bateria
volatile bool alertaCorrenteExc = false;   // DUAL-CORE: volatile   // Corrente excessiva (ACS758 > limite)
volatile bool alertaFalhaFreio = false;    // DUAL-CORE: volatile    // Falha no sistema de freio
volatile bool alertaBatCritica = false;    // DUAL-CORE: volatile    // Bateria crítica (<5%)
volatile bool alertaSubtensao = false;     // DUAL-CORE: volatile     // Subtensão no sistema
volatile bool alertaCruiseOn = false;      // DUAL-CORE: volatile      // Cruise control ativo (indicador)
unsigned long alertaPiscaTimer = 0;  // Timer para piscar setas
bool alertaPiscaEstado = false;      // Estado atual do pisca (on/off)
lv_obj_t * overlayAlerta = NULL;     // Container overlay no layer_top
lv_obj_t * overlayLabel = NULL;      // Label do overlay

// =============================================================================
// AGENDA DE MANUTENÇÃO POR KM (RTC/Data/Hora REMOVIDOS - Seção 1)
// =============================================================================
// Sistema focado apenas em telemetria e controle. Manutenção por KM apenas.
float manutKmAlvo = 0;            // Km para próxima manutenção (0=desativado)
bool manutAlertaMostrado = false;  // Evita mostrar alerta repetidamente

// =============================================================================
// RADAR DE PROXIMIDADE TRASEIRO - DUAL SENSOR (2x JSN-SR04T)
// =============================================================================
// O Arduino Mega mede a distância com DOIS sensores ultrassônicos (Esq/Dir).
// O Mega envia via Serial no formato: RADAR,<esq>,<dir>\n
// O CYD exibe visual automotivo com dois arcos (lv_arc) paralelos.
// Zonas de cor dinâmicas:
//   VERDE  (>100cm): Livre
//   AMARELO (40-100cm): Atenção
//   VERMELHO (<40cm): Perigo
// O radar só aparece se radarHabilitado == true E marcha ré (R4) ativa.
// radarHabilitado é persistido na NVS via Preferences.h.
volatile int radarDistanciaEsq = 0;        // DUAL-CORE: volatile - distância esquerda (cm)
volatile int radarDistanciaDir = 0;        // DUAL-CORE: volatile - distância direita (cm)
volatile bool radarAtivo = true;            // DUAL-CORE: volatile - Seção 3: Radar "Sempre Ativo" por padrão
bool radarHabilitado = true;               // Persistido na NVS - switch no menu de config
unsigned long radarUltimaLeitura = 0; // millis() da última leitura recebida
#define RADAR_TIMEOUT_MS 2000       // Se não receber dados em 2s, considera inativo

// =============================================================================
// DUAL-CORE: HANDLES DE TAREFAS E SINCRONIZAÇÃO
// =============================================================================
// O QUE FAZ: Armazena os handles (ponteiros) para as tarefas FreeRTOS e o
//   semáforo mutex de proteção do barramento SPI.
//
// TASK HANDLE: Permite monitorar, suspender ou deletar a tarefa de telemetria.
//   Se tarefaTelemetria == NULL, a tarefa não foi criada (erro no setup).
//
// SPI MUTEX: O display ILI9341 e o touch XPT2046 compartilham o mesmo
//   barramento SPI. O mutex garante que apenas um acesse o SPI por vez.
//   Sem mutex: display escrevendo pixels + touch lendo coordenadas = dados
//   corrompidos, tela piscando ou touch não responsivo.
//   Com mutex: cada operação adquire exclusividade antes de acessar o SPI.
//
// PINOS/HARDWARE: SPI (CLK=18, MOSI=23, MISO=19, CS_TFT=15, CS_TOUCH=33)
// OTIMIZAÇÃO: Mutex tem overhead mínimo (~1us por take/give)
// WDT: xSemaphoreTake com timeout de 50ms evita deadlock
// =============================================================================
TaskHandle_t tarefaTelemetria = NULL;      // Handle da tarefa de telemetria (Core 0)
SemaphoreHandle_t spiMutex = NULL;         // Mutex para proteção do barramento SPI

// Bluetooth SPP: CYD atua como MASTER, inicia conexão com HC-06 (Slave) no Mega.
// Nome local anunciado no modo Master: BT_MASTER_NAME ("VERSA3_CYD").
BluetoothSerial SerialBT;

// =============================================================================
// BLUETOOTH MASTER — COMUNICAÇÃO COM HC-06 (v4.1 HC-06 Master)
// =============================================================================
// MUDANÇA v4.1: O ESP32 CYD passa a ser o MASTER Bluetooth.
//   O HC-06 no Mega é sempre SLAVE — aguarda a conexão do CYD.
//
// FLUXO:
//   1. Boot → carrega MAC do HC-06 da NVS ("bt_config"/"hc06_mac")
//   2. Se MAC salvo → SerialBT.connect(mac) → envia dados
//   3. Se sem MAC → usuário abre PAREAMENTO_BT → busca dispositivos (scan)
//   4. Usuário seleciona "VERSA3_MEGA_HC06" → MAC salvo na NVS → conecta
//   5. Watchdog (3s): conexão perdida → SerialBT.connect(mac) automaticamente
//
// THREAD SAFETY:
//   - GAP callback registrado depois de SerialBT.begin() para não conflitar
//   - pendingBtScan / pendingBtConnect: Core 1 seta, Core 0 executa
//   - btDeviceList protegido por btDeviceMutex (GAP callback + Core 1 lêem)
//   - volatile em todas as variáveis compartilhadas entre cores
//
// PINOS: Bluetooth interno do ESP32 (sem pinos externos)
// WDT: SerialBT.connect() pode demorar até 5s; tratado com vTaskDelay.
// =============================================================================
#define BT_WATCHDOG_MS          3000UL  // 3s sem dados = conexão perdida
#define BT_HEARTBEAT_INTERVAL_MS 2000UL  // 2s entre heartbeats enviados ao Mega
#define BT_RESTART_DELAY_MS      300UL  // ms de pausa entre end() e begin()
#define BT_CONNECT_TIMEOUT_MS   5000UL  // Timeout para SerialBT.connect() (ms)
// BT_INQUIRY_DURATION_UNITS: duração do scan em unidades de 1,28s (0x0A = 12,8s ≈ 10s)
#define BT_INQUIRY_DURATION_UNITS 0x0A  // 10 × 1,28s = 12,8 segundos de scan
#define BT_MASTER_NAME    "VERSA3_CYD"       // Nome local do ESP32 no modo Master
#define BT_TARGET_NAME    "VERSA3_MEGA_HC06" // Nome do HC-06 no Mega (AT+NAME)
#define BT_TARGET_PIN     "1234"             // PIN padrão do HC-06 (AT+PIN)
#define MAX_BT_DEVICES    10                 // Máximo de dispositivos no scan
// MAC do HC-06 gravado no módulo (padrão de fábrica — sobrescrito pela NVS se o utilizador
// associar outro módulo via menu PAREAMENTO BT). Formato: 6 bytes little-endian.
// Para alterar: aceder a PAREAMENTO BT, fazer scan e selecionar o dispositivo.
#define BT_DEFAULT_MAC    {0x98, 0xDA, 0x60, 0x0B, 0x5F, 0x92}

// ── Variáveis de estado da conexão BT ────────────────────────────────────────
volatile bool           btConectado      = false;  // true = HC-06 conectado e enviando CSV
volatile unsigned long  btUltimaRecepMs  = 0;      // millis() do último CSV recebido
volatile bool           btReconectando   = false;  // true = tentando reconectar
volatile unsigned long  btReconectandoMs = 0;      // quando iniciou a reconexão
volatile bool           btConectando     = false;  // true = tentativa de connect em curso
lv_obj_t *              btMenuStatusLabel = NULL;  // Label de status na tela BLUETOOTH_MENU

// Flag para reconexão thread-safe: Core 1 seta, Core 0 executa
volatile bool           pendingReconexaoBT = false;

// ── Scanning / Discovery ─────────────────────────────────────────────────────
volatile bool           btScanAtivo        = false;  // true = inquiry BT em andamento
volatile bool           pendingBtScan      = false;  // Core 1 seta → Core 0 inicia scan
volatile bool           pendingBtScanStop  = false;  // Core 1 seta → Core 0 para scan
volatile bool           pendingBtConnect   = false;  // Core 1 seta → Core 0 conecta ao MAC pendente
volatile bool           pendingRefreshBtList = false; // Core 0/callback seta → Core 1 atualiza lista LVGL

// Lista de dispositivos encontrados no scan (acesso protegido por btDeviceMutex)
struct BtDeviceInfo {
    uint8_t mac[6];   // Endereço MAC Bluetooth (6 bytes)
    char    name[33]; // Nome do dispositivo (null-terminated, 32 chars + \0)
    bool    valid;    // true = entrada válida
};
SemaphoreHandle_t   btDeviceMutex = NULL;        // Protege btDeviceList entre GAP callback e Core 1
BtDeviceInfo        btDeviceList[MAX_BT_DEVICES]; // Dispositivos encontrados no scan
volatile int        btDeviceCount  = 0;           // Número de dispositivos encontrados
int                 btSelectedDeviceIdx = -1;     // Índice selecionado pelo utilizador no LVGL (-1 = nenhum)

// ── MAC do HC-06 alvo (persistido na NVS) ────────────────────────────────────
uint8_t             btTargetMac[6]   = {0};   // MAC do HC-06 (carregado da NVS)
volatile bool       btTargetMacSaved = false; // true = MAC válido na NVS
uint8_t             btPendingMac[6]  = {0};   // MAC a conectar (set pelo UI, lido pelo Core 0)

// Constantes de conversão: raw ADC do Mega → grandeza física no CYD
// O Mega 2560 tem ADC de 10 bits (0-1023) com VREF = 5V.
// O divisor de tensão usa R1=220kΩ e R2=10kΩ → Razão = 10/(220+10) = 0,04348
// Vbat = (raw / MEGA_ADC_MAX) × MEGA_VREF_V / MEGA_DIV_RAZAO
#define MEGA_VREF_V     5.0f                         // Tensão de referência ADC do Mega (5V)
#define MEGA_ADC_MAX    1023.0f                      // Resolução 10-bit: valores 0 a 1023
#define MEGA_DIV_RAZAO  (10.0f / (220.0f + 10.0f))  // R2/(R1+R2) = 0,04348
// Sensor de corrente ACS758LCB-050B (mesmo modelo do Mega):
// Corrente = (Vadc - 2.5V) / 0.040 V/A  →  em raw: corrente = ((raw/1023)*5 - 2.5) / 0.040
#define MEGA_ACS758_OFFSET_V  2.5f   // Tensão de saída sem corrente (0A = 2.5V = raw 512)
#define MEGA_ACS758_SENS_VA   0.040f // Sensibilidade: 40mV por Ampere
// Número de ímãs/dentes no sensor RPM (mesmo valor do Mega PULSOS_POR_VOLTA)
#define CYD_PULSOS_POR_VOLTA  1      // Ajustar conforme instalação (1 ímã por volta = padrão)

// =============================================================================
// NVS DIRTY CHECK: ÚLTIMOS VALORES SALVOS NA MEMÓRIA NÃO VOLÁTIL
// =============================================================================
// O QUE FAZ: Armazena uma cópia dos últimos valores escritos na NVS.
//   Antes de cada escrita, compara o valor atual com o último salvo.
//   Se forem iguais, PULA a escrita (economia de ~99% das operações).
//
// POR QUE: A flash NVS do ESP32 suporta ~100.000 ciclos de escrita por setor.
//   Sem dirty check: escrita a cada 30s = ~2880 escritas/dia = flash degradada em ~35 dias
//   Com dirty check: escrita APENAS quando valor muda = ~10-50 escritas/dia = flash dura anos
//
// VALORES MONITORADOS:
//   circRodaCM: muda RARAMENTE (só quando usuário ajusta no menu)
//   kmTotal: muda A CADA PULSO DO SENSOR (mas NVS só salva a cada 30s)
//   kmParcial: muda junto com kmTotal
//   Configs gerais: mudam APENAS quando usuário altera no menu
//
// INICIALIZAÇÃO: -1.0 força a primeira escrita após o boot (valor inválido)
// OTIMIZAÇÃO: Reduz desgaste da flash NVS em ~99%
// =============================================================================
static float lastSavedCircRodaCM = -1.0f;   // Último valor salvo de circunferência
static float lastSavedKmTotal = -1.0f;      // Último valor salvo de km total
static float lastSavedKmParcial = -1.0f;    // Último valor salvo de km parcial
static int lastSavedVelocidade = -1;        // Último valor salvo de velocidade
static int lastSavedBateriaEmUso = -1;      // Última bateria em uso salva
static float lastSavedScaleX = -999.0f;     // Última calibração X salva
static float lastSavedScaleY = -999.0f;     // Última calibração Y salva

// =============================================================================
// MÓDULO G: PREÇO POR QUILÔMETRO - VARIÁVEIS GLOBAIS
// =============================================================================
// Sistema de telemetria com rastreamento cruzado: Bateria × Força × Tração
// Duas baterias (B1/B2), três forças (F1/F2/F3), dois modos (1x3/3x3)
// Total: 2 × 3 × 2 = 12 acumuladores de km independentes
//
// MUTEX: telemetriaMutex protege TODAS as variáveis abaixo contra acesso
//   simultâneo entre Core 0 (escrita) e Core 1 (leitura para UI).
//   Core 0 chama xSemaphoreTake(telemetriaMutex) antes de escrever.
//   Core 1 chama xSemaphoreTake(telemetriaMutex) antes de ler para desenhar.
//
// NVS KEYS: Todas ≤15 chars (limite ESP32 NVS). Namespace: "precokm"
//   potB1 (5), potB2 (5), precoKwh (8), km_b1f1_1x3 (11), etc.
// =============================================================================
SemaphoreHandle_t telemetriaMutex = NULL;  // OBJETIVO 4: Mutex dual-core

// Configuração de carregadores (1 por bateria — OBJETIVO 4: isolamento)
float potCarregB1 = 500.0;    // Potência carregador B1 em Watts (NVS: "potB1")
float potCarregB2 = 500.0;    // Potência carregador B2 em Watts (NVS: "potB2")
float precoKWh = 0.75;        // Preço do kWh em R$ (NVS: "precoKwh")

// Detecção automática de carga (Core 0)
bool cargaDetectada = false;       // true = carga em andamento detectada
int cargaBateria = 1;              // Qual bateria está carregando (1=B1, 2=B2)
float cargaVoltaInicio = 0.0;      // Voltagem no início da detecção
float cargaVoltaAnterior = 0.0;    // Voltagem da leitura anterior (delta)
int cargaPercInicio = 0;           // % no início da carga
unsigned long cargaTempoInicio = 0; // millis() quando carga iniciou
float cargaTempoH = 0.0;          // Tempo decorrido de carga em horas
int cargaGanhoPerc = 0;           // Ganho % durante carga
float custoUltimaCarga = 0.0;     // Custo da última sessão de carga (R$)
float custoProjCarga100 = 0.0;    // Projeção de custo para 0→100% (R$)

// Km acumulados por [bateria][força][tração] — 12 combinações
// Índices: bat=0(B1)/1(B2), forca=0(F1)/1(F2)/2(F3), trac=0(1x3)/1(3x3)
float kmAcum[2][3][2] = {{{0}}};  // Todos iniciam em 0, carregados da NVS

// Custo por km calculado — mesmo layout [bat][forca][trac]
float custoKm[2][3][2] = {{{0}}}; // R$/km por combinação

// Porcentagem de bateria consumida por modo (Fix 4: necessário para fórmula correta)
// Acumula total de % gasta em cada combinação bat/força/tração
float percGasto[2][3][2] = {{{0}}}; // Total % consumida por combinação
static int percAnterior = -1;       // Última porcentagem lida (para calcular delta)

// Variável de navegação para submenus do Preço por Km
int precoEditBat = 0;              // Bateria selecionada no submenu (0=B1, 1=B2)
int precoEditForca = 0;            // Força selecionada no submenu (0=F1, 1=F2, 2=F3)

// =============================================================================
// SISTEMA DE ALARMES - CICLO DE 3 SEGUNDOS (CARROSSEL)
// =============================================================================
// Quando há múltiplos alarmes ativos, eles alternam a cada 3 segundos.
// EXCEÇÃO: Setas (L/R) e Emergência têm PRIORIDADE TOTAL e interrompem o ciclo.
// Variáveis da máquina de estados do carrossel:
int indexAlertaAtual = 0;                // Índice do alerta sendo exibido no ciclo
unsigned long ultimoTempoTroca = 0;      // millis() da última troca de alerta no ciclo
#define ALERTA_CICLO_MS 3000             // 3 segundos entre trocas no carrossel

// =============================================================================
// RADAR AUTOMÁTICO - VARIÁVEIS DE CONTROLE
// =============================================================================
// Quando R4 (marcha ré) é detectada, o sistema salva a página atual e
// muda automaticamente para RADAR_PROXIMIDADE. Ao desengatar R4, retorna.
Pagina paginaAnteriorRadar = MENU_HODOMETRO;  // Página salva antes do auto-switch
bool radarAutoAtivado = false;           // true = trocou para radar automaticamente
bool modoReAnterior = false;             // Estado anterior de modoReLigado (anti-flicker)

// DUAL-CORE: Flag para solicitar montarInterface() a partir do Core 0.
// Core 0 (telemetria) NÃO pode chamar montarInterface() diretamente pois
// LVGL não é thread-safe. Seta esta flag; Core 1 (loop) chama montarInterface()
// na próxima iteração segura.
volatile bool pendingMontarInterface = false;

// =============================================================================
// MÓDULO F: SPLASH SCREEN - VARIÁVEIS
// =============================================================================
// splashScreenAtiva: controla se a tela de splash está sendo exibida.
//   - Inicia true no boot. O loop() incrementa splashProgress a cada 100ms.
//   - Quando splashProgress >= 100, muda para false e carrega montarInterface().
//   - NÃO depende de nenhuma variável de "sistema pronto" (sistemaReady removida).
//   - A verificação de integridade dos relés foi movida para o menu Manutenção.
bool splashScreenAtiva = true;   // true = splash visível, false = interface normal
int splashProgress = 0;          // 0-100: progresso da barra na splash screen

// =============================================================================
// DECLARAÇÕES ANTECIPADAS
// =============================================================================
void montarInterface();
void criar_menu_hodometro(lv_obj_t * cont);
void atualizar_menu_hodometro();
void controlarRelesSolar();
void salvarConfiguracoesSolar();
void salvarDadosVeiculo();
void enviarComandoRele(int numero, int estado);
void enviarResetGeral();
void salvarPrecoKm();
void carregarPrecoKm();
void toggleStandbyManual();
void tarefaTelemetriaFunc(void *pvParameters);  // DUAL-CORE: Tarefa Core 0
void detectarCargaAuto();                        // Detecção automática de carga
void acumularKmPorModo();                        // Acumula km por bat/força/tração
void calcularCustoKm();                          // Calcula R$/km por combinação
void aplicar_tema(bool escuro);                  // Aplica tema claro/escuro globalmente
void desligarManual();                           // Desliga sistema manualmente pelo botao keepalive
void atualizarStatusBT();                        // Atualiza label de status na tela BLUETOOTH_MENU
void salvarMacBT(const uint8_t mac[6]);          // Persiste MAC do HC-06 na NVS
void carregarMacBT();                            // Restaura MAC do HC-06 da NVS

// =============================================================================
// BLUETOOTH GAP CALLBACK — DISCOVERY + AUTENTICAÇÃO
// =============================================================================
// Registrado APÓS SerialBT.begin() para não conflitar com o init interno da lib.
// Trata eventos de:
//   ESP_BT_GAP_DISC_RES_EVT        — novo dispositivo encontrado durante scan
//   ESP_BT_GAP_DISC_STATE_CHANGED  — scan iniciado/terminado
//   ESP_BT_GAP_AUTH_CMPL_EVT       — autenticação concluída (par OK ou falhou)
//   ESP_BT_GAP_PIN_REQ_EVT         — HC-06 pede PIN (responde com BT_TARGET_PIN)
//
// THREAD SAFETY: Este callback é chamado em uma task interna do BT stack
//   (não no Core 0 nem no Core 1). Apenas escreve em variáveis volatile e
//   no btDeviceList protegido por btDeviceMutex.
// =============================================================================
static void btGapEventHandler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        // ── Novo dispositivo encontrado durante inquiry ──────────────────────
        case ESP_BT_GAP_DISC_RES_EVT:
            if (btScanAtivo) {
                // GAP callback roda em contexto de task (não ISR) → usar xSemaphoreTake
                if (xSemaphoreTake(btDeviceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (btDeviceCount < MAX_BT_DEVICES) {
                        BtDeviceInfo &dev = btDeviceList[btDeviceCount];
                        memcpy(dev.mac, param->disc_res.bda, 6);
                        dev.name[0] = '\0';
                        dev.valid   = true;
                        // Extrai nome do EIR/CoD properties
                        for (int i = 0; i < param->disc_res.num_prop; i++) {
                            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                                int len = param->disc_res.prop[i].len;
                                if (len > 32) len = 32;
                                memcpy(dev.name, (char*)param->disc_res.prop[i].val, len);
                                dev.name[len] = '\0';
                                break;
                            }
                        }
                        btDeviceCount++;
                        pendingRefreshBtList = true;
                    }
                    xSemaphoreGive(btDeviceMutex);
                }
            }
            break;

        // ── Início/fim do scan ───────────────────────────────────────────────
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                Serial.println("[BT] Scan iniciado");
            } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                btScanAtivo = false;
                pendingRefreshBtList = true;
                Serial.println("[BT] Scan concluído");
            }
            break;

        // ── Autenticação concluída ───────────────────────────────────────────
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                Serial.printf("[BT] Autenticado: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                    param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                    param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
            } else {
                Serial.printf("[BT] Falha autenticação: %d\n", (int)param->auth_cmpl.stat);
            }
            break;

        // ── Pedido de PIN (HC-06 sempre usa PIN fixo) ────────────────────────
        case ESP_BT_GAP_PIN_REQ_EVT:
            {
                const char* pin = BT_TARGET_PIN;
                uint8_t pinLen = (uint8_t)strlen(pin);
                esp_bt_pin_code_t pinCode;
                memset(pinCode, 0, sizeof(pinCode));
                memcpy(pinCode, pin, pinLen);
                esp_bt_gap_pin_reply(param->pin_req.bda, true, pinLen, pinCode);
                Serial.printf("[BT] PIN enviado para HC-06\n");
            }
            break;

        default:
            break;
    }
}

// =============================================================================
// PERSISTÊNCIA - SALVAMENTO DE DADOS DE CONSUMO
// =============================================================================
// Salva os dados de telemetria de um índice específico (0-4) na NVS.
// Namespace: "telemetria", chave: "tel_<idx>".
// Chamada após zerar dados individuais na tela de detalhes.
// NOTA SOBRE MEMORY: Cada chamada abre e fecha o namespace NVS.
// Não há risco de vazamento de memória pois prefs.end() é sempre chamado.
void salvarDados(int idx) {
    if (idx < 0 || idx >= 5) return;
    prefs.begin("telemetria", false);
    char key[12]; 
    snprintf(key, sizeof(key), "tel_%d", idx);
    prefs.putBytes(key, &dados[idx], sizeof(Telemetria));
    prefs.end();
}

void carregarDados() {
    prefs.begin("telemetria", true);
    for(int i = 0; i < 5; i++) {
        char key[12];
        snprintf(key, sizeof(key), "tel_%d", i);
        if (prefs.getBytes(key, &dados[i], sizeof(Telemetria)) == 0) {
            dados[i] = {0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f};
        }
    }
    prefs.end();
}

// =============================================================================
// PERSISTÊNCIA DO MAC DO HC-06 (NVS namespace "bt_config")
// =============================================================================
// salvarMacBT: grava os 6 bytes do MAC + flag de validade.
//   Chamado quando o utilizador seleciona o dispositivo no BLUETOOTH_MENU/PAREAMENTO_BT.
// carregarMacBT: restaura na inicialização; chamado em carregarConfiguracoes().
// =============================================================================
void salvarMacBT(const uint8_t mac[6]) {
    memcpy(btTargetMac, mac, 6);
    btTargetMacSaved = true;
    prefs.begin("bt_config", false);
    prefs.putBytes("hc06_mac", mac, 6);
    prefs.putBool("hc06_saved", true);
    prefs.end();
    Serial.printf("[BT] MAC salvo: %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void carregarMacBT() {
    prefs.begin("bt_config", true);
    bool saved = prefs.getBool("hc06_saved", false);
    if (saved && prefs.getBytes("hc06_mac", btTargetMac, 6) == 6) {
        btTargetMacSaved = true;
        Serial.printf("[BT] MAC carregado da NVS: %02X:%02X:%02X:%02X:%02X:%02X\n",
            btTargetMac[0], btTargetMac[1], btTargetMac[2],
            btTargetMac[3], btTargetMac[4], btTargetMac[5]);
    } else {
        // NVS vazia: usa o MAC padrão gravado no firmware (BT_DEFAULT_MAC).
        // Isso garante auto-connect imediato sem exigir menu de scan na primeira vez.
        // O utilizador pode sobrescrever via PAREAMENTO BT se trocar o módulo HC-06.
        const uint8_t defaultMac[6] = BT_DEFAULT_MAC;
        memcpy(btTargetMac, defaultMac, 6);
        btTargetMacSaved = true;
        Serial.printf("[BT] MAC padrao (BT_DEFAULT_MAC): %02X:%02X:%02X:%02X:%02X:%02X\n",
            btTargetMac[0], btTargetMac[1], btTargetMac[2],
            btTargetMac[3], btTargetMac[4], btTargetMac[5]);
        Serial.println("[BT] Para associar outro modulo: use PAREAMENTO BT → scan");
    }
    prefs.end();
}

// =============================================================================
// EVENTO PARA ZERAR DADOS DE CONSUMO (callback LVGL)
// =============================================================================
// Callback registrado nos botões da lista de detalhes de consumo.
// Ativado por LONG_PRESS (segurar por ~400ms) para evitar toques acidentais.
// O user_data contém o tipo de dado a zerar (0=kmTotal, 1=kmPorConsumo, etc).
// Após zerar, salva na NVS e recria a interface para atualizar o display.
static void evento_zerar_especifico(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    int tipoDado = (int)(intptr_t)lv_event_get_user_data(e);
    if(code == LV_EVENT_LONG_PRESSED) {
        switch(tipoDado) {
            case 0: dados[itemSelecionado].kmTotal = 0.0f; break;
            case 1: dados[itemSelecionado].kmPorConsumo = 0.0f; break;
            case 2: dados[itemSelecionado].eficiencia = 0.0f; break;
            case 3: dados[itemSelecionado].tempoSegundos = 0; break;
            case 4: dados[itemSelecionado].picoAmperagem = 0.0f; break;
            case 5: dados[itemSelecionado].consumoMedioAmperagem = 0.0f; break;
        }
        salvarDados(itemSelecionado);
        montarInterface();
    }
}

// =============================================================================
// INTERCEPTAÇÃO DE TOQUE (CALIBRAÇÃO DO TOUCH SCREEN)
// =============================================================================
// Callback registrado no dispositivo de entrada (indev) do LVGL.
// Intercepta TODOS os eventos de toque (LV_EVENT_ALL) para aplicar
// a correção de calibração (scaleX/scaleY/offsetX/offsetY).
//
// FÓRMULA DE CORREÇÃO:
//   X_corrigido = (X_raw * scaleX) + offsetX
//   Y_corrigido = (Y_raw * scaleY) + offsetY
//   Resultado limitado a [0, 319] x [0, 239] (resolução do display)
//
// scaleX/scaleY são calculados durante a calibração de 5 pontos.
// Sem calibração, scale = 1.0 e offset = 0 (passthrough direto).
void meu_feedback_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_READY) {
        lv_indev_data_t * data = (lv_indev_data_t *)lv_event_get_param(e);
        if (data && data->state == LV_INDEV_STATE_PR) {
            data->point.x = (data->point.x * scaleX) + offsetX;
            data->point.y = (data->point.y * scaleY) + offsetY;
            data->point.x = constrain(data->point.x, 0, 319);
            data->point.y = constrain(data->point.y, 0, 239);
        }
    }
}

// =============================================================================
// FUNÇÕES DE CÁLCULO - LEITURA DE TENSÃO E BATERIA
// =============================================================================

// ---------------------------------------------------------------------------
// lerVoltagemBruta()
// ---------------------------------------------------------------------------
// Lê a tensão diretamente do pino analógico do ESP32 CYD (GPIO34).
// CIRCUITO: Divisor de tensão com resistores 220kΩ + 10kΩ (1/4W, 1%).
//
// CÁLCULO DO DIVISOR:
//   V_pino = V_bateria * R2 / (R1 + R2)
//   V_pino = V_bateria * 10k / (220k + 10k)
//   V_pino = V_bateria * 10 / 230 = V_bateria / 23.0
//
// CÁLCULO INVERSO (pino → bateria):
//   V_bateria = V_pino * 23.0
//   V_pino = ADC_raw * (3.3V / 4095)  [ADC 12 bits do ESP32]
//
// FAIXA DE MEDIÇÃO:
//   Máximo: 3.3V * 23.0 = 75.9V (cobre até sistema de 72V)
//   Resolução: 3.3V / 4095 * 23.0 ≈ 0.0185V por step
//
// FILTRO: Valores < 0.3V são considerados ruído e retornam 0.0.
// NOTA: Esta leitura é de backup. A telemetria principal vem do Mega.
// ---------------------------------------------------------------------------
float lerVoltagemBruta() {
    int leitura = analogRead(pinoVoltagem);      // 0-4095 (12 bits)
    float vPino = leitura * (3.3 / 4095.0);      // Converte para Volts no pino
    float vCalculada = vPino * 23.0;              // Aplica fator do divisor
    return (vCalculada < 0.3) ? 0.0 : vCalculada; // Filtro de ruído
}

// ---------------------------------------------------------------------------
// obterVoltagemReal(idx)
// ---------------------------------------------------------------------------
// Retorna a tensão real da bateria especificada.
// Para B1 (idx=0) e B2 (idx=1): usa dados recebidos do Mega via Serial.
// Para B3 (idx=2): lê localmente via ADC e aplica fator de calibração.
// O fator de calibração corrige diferenças entre ADC e multímetro real.
// ---------------------------------------------------------------------------
float obterVoltagemReal(int idx) {
    if(idx == 0) return vB1;  // Bateria principal (do Mega)
    if(idx == 1) return vB2;  // Bateria reserva (do Mega)
    return lerVoltagemBruta() * confBat[idx].fatorCalib; // B3 local + calibração
}

// ---------------------------------------------------------------------------
// obterLimitesBateria(idx)
// ---------------------------------------------------------------------------
// Calcula tensão mínima e máxima da bateria com base na química.
// CHUMBO (ÁCIDO): 10.5V - 13.8V por bloco de 12V
// LÍTIO-ÍON:      3.0V - 4.2V por célula (3.7V nominal)
// LiFePO4:        2.5V - 3.6V por célula (3.2V nominal)
// O número de células em série é deduzido da tensão nominal do sistema.
// Exemplo: Sistema 48V Lítio = 48/3.7 ≈ 13 células em série
//          Min = 13 * 3.0V = 39.0V, Max = 13 * 4.2V = 54.6V
// ---------------------------------------------------------------------------
Limites obterLimitesBateria(int idx) {
    float vMinCell = 0, vMaxCell = 0;
    int numSeries = 1;
    switch (confBat[idx].quimica) {
        case 0: vMinCell = 10.5; vMaxCell = 13.8; numSeries = confBat[idx].sistema / 12; break;
        case 1: vMinCell = 3.0; vMaxCell = 4.2; numSeries = (confBat[idx].sistema == 12) ? 3 : (int)(confBat[idx].sistema / 3.7); break;
        case 2: vMinCell = 2.5; vMaxCell = 3.6; numSeries = (confBat[idx].sistema == 12) ? 4 : (int)(confBat[idx].sistema / 3.2); break;
    }
    if (numSeries < 1) numSeries = 1;
    return { vMinCell * (float)numSeries, vMaxCell * (float)numSeries };
}

// ---------------------------------------------------------------------------
// calcularPorcentagem(idx, vAtual)
// ---------------------------------------------------------------------------
// Calcula a porcentagem de carga da bateria (0-100%) usando interpolação linear.
// FÓRMULA: % = ((V_atual - V_min) / (V_max - V_min)) * 100
// Limitado a [0, 100] por clamping manual (ESP32 não tem constrain do Arduino).
// Valores abaixo de 1.0V retornam 0% (bateria desconectada).
// ---------------------------------------------------------------------------
int calcularPorcentagem(int idx, float vAtual) {
    Limites lim = obterLimitesBateria(idx);
    if (vAtual < 1.0) return 0;                    // Bateria desconectada
    float range = lim.vMax - lim.vMin;
    if (range <= 0) return 0;                       // Configuração inválida
    int perc = (int)((vAtual - lim.vMin) / range * 100.0);
    if (perc < 0) perc = 0;
    if (perc > 100) perc = 100;
    return perc;
}

// =============================================================================
// MÓDULO D: CONTROLE DOS RELÉS SOLARES (R8 e R9)
// =============================================================================
// Controla os relés de carregamento solar diretamente pelos GPIOs do CYD.
// R8 (GPIO26): Carregamento solar da bateria B1
// R9 (GPIO27): Carregamento solar da bateria B2
//
// LÓGICA:
// - Solar só é ativado se solarLigado == true (toggle do usuário)
// - Solar só ativa se a porcentagem de B3 > ajustePorcentagem (corte)
// - R8 só carrega B1 se B1 NÃO está em uso (evita carga durante descarga)
// - R9 só carrega B2 se B2 NÃO está em uso
//
// SEGURANÇA: B3 é a bateria de sistema fixa 12V. Se B3 estiver baixa,
// o solar não ativa (prioriza manter o sistema ligado).
//
// Log via Serial quando o estado do relé muda (para diagnóstico).
// ---------------------------------------------------------------------------
void controlarRelesSolar() {
    bool novoEstadoRele8 = false;
    bool novoEstadoRele9 = false;

    // MÓDULO D: Usa dados reais do sistema (B3 é sistema, não tração)
    float percB3 = (float)calcularPorcentagem(2, vB3);

    if (solarLigado && percB3 > ajustePorcentagem) {
        if (bateriaEmUso != 1) novoEstadoRele8 = true;
        if (bateriaEmUso != 2) novoEstadoRele9 = true;
    }

    // R8 (Solar B1): controlado via serial ao Mega.
    if (novoEstadoRele8 != estadoAnteriorRele8) {
        enviarComandoRele(8, novoEstadoRele8 ? 1 : 0);
        Serial.print("[SOLAR] Rele 8: ");
        Serial.println(novoEstadoRele8 ? "LIGADO" : "DESLIGADO");
        estadoAnteriorRele8 = novoEstadoRele8;
    }

    // R9 (Solar B2): controlado via serial ao Mega.
    // CORRIGIDO: Removido digitalWrite(RELE_9_PIN) — GPIO27 no ESP32-WROVER é
    // compartilhado com PSRAM (CS), causava corrupção de memória no acesso PSRAM.
    // Controle feito APENAS via protocolo serial com o Mega (R,9,x).
    if (novoEstadoRele9 != estadoAnteriorRele9) {
        enviarComandoRele(9, novoEstadoRele9 ? 1 : 0);
        Serial.print("[SOLAR] Rele 9: ");
        Serial.println(novoEstadoRele9 ? "LIGADO" : "DESLIGADO");
        estadoAnteriorRele9 = novoEstadoRele9;
    }
}

// =============================================================================
// PERSISTÊNCIA - SALVAMENTO DE CONFIGURAÇÕES NA MEMÓRIA NÃO VOLÁTIL
// =============================================================================
// O ESP32 usa NVS (Non-Volatile Storage) via Preferences.h.
// Cada grupo de dados tem seu próprio namespace para organização:
//   "veiculo"    - calibração touch, roda, km, velocidade, bateria ativa
//   "cyd_storage" - configuração individual das 3 baterias + solar
//   "conducao"   - PAS, curva de sensibilidade, balanço motores
//   "telemetria" - dados de consumo por fonte/motor
//
// ATENÇÃO SOBRE DESGASTE DA FLASH:
// A NVS do ESP32 usa flash SPI com ~100.000 ciclos de escrita por setor.
// Para evitar desgaste prematuro:
//   - Configs gerais: salvam a cada 60 segundos (loop)
//   - Dados críticos (km): salvam a cada 30 segundos (loop)
//   - Dados de consumo: salvam apenas quando o usuário zera um campo
//   - Alterações de config: salvam imediatamente (ação do usuário)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// salvarDadosVeiculo() - COM DIRTY CHECK (OTIMIZAÇÃO NVS)
// ---------------------------------------------------------------------------
// O QUE FAZ: Salva dados críticos do veículo na NVS APENAS se houver mudança.
// OTIMIZAÇÃO NVS: Compara cada valor com o último salvo antes de escrever.
//   Se NENHUM valor mudou, retorna imediatamente (zero escritas na flash).
//   Reduz desgaste da flash NVS em ~99% comparado com escrita incondicional.
// PINOS/HARDWARE: Flash SPI interna do ESP32 (mesma flash do firmware)
// WDT: Operação rápida (~2ms se escrever, ~0ms se pular). Não causa WDT.
// ---------------------------------------------------------------------------
void salvarDadosVeiculo() {
    // DIRTY CHECK: Verifica se algum valor mudou desde a última escrita
    bool mudou = false;
    if(circRodaCM != lastSavedCircRodaCM) mudou = true;
    if(kmTotal != lastSavedKmTotal) mudou = true;
    if(kmParcial != lastSavedKmParcial) mudou = true;
    
    // OTIMIZAÇÃO NVS: Se nada mudou, não escreve (economia de ~99% das escritas)
    if(!mudou) return;
    
    prefs.begin("veiculo", false);
    prefs.putFloat("roda", circRodaCM);   // Circunferência da roda em CM
    prefs.putFloat("kmT", kmTotal);       // Quilometragem total acumulada
    prefs.putFloat("kmP", kmParcial);     // Quilometragem parcial (trip)
    prefs.end();
    
    // Atualiza valores de referência para próxima comparação
    lastSavedCircRodaCM = circRodaCM;
    lastSavedKmTotal = kmTotal;
    lastSavedKmParcial = kmParcial;
    
    // Log apenas quando realmente escreve (debug)
    Serial.println("[NVS] Dados veiculo salvos (dirty check: valores alterados)");
}

// ---------------------------------------------------------------------------
// salvarConfiguracoes() - COM DIRTY CHECK (OTIMIZAÇÃO NVS)
// ---------------------------------------------------------------------------
// O QUE FAZ: Salva TODAS as configurações do veículo na NVS.
// OTIMIZAÇÃO NVS: Verifica se houve mudança antes de abrir o namespace.
//   Se nenhum valor mudou, retorna sem escrever (economia de flash).
// Inclui: calibração touch (scaleX/Y, offsetX/Y), circunferência da roda,
// velocidade atual, km total/parcial, bateria em uso.
// Chamada a cada 60s pela tarefa de telemetria (Core 0) e após mudanças.
// WDT: Operação rápida (~5ms se escrever). Não causa WDT.
// ---------------------------------------------------------------------------
void salvarConfiguracoes() {
    // DIRTY CHECK: Verifica se algum valor relevante mudou
    bool mudou = false;
    if(scaleX != lastSavedScaleX || scaleY != lastSavedScaleY) mudou = true;
    if(velocidadeAtual != lastSavedVelocidade) mudou = true;
    if(bateriaEmUso != lastSavedBateriaEmUso) mudou = true;
    if(circRodaCM != lastSavedCircRodaCM) mudou = true;
    if(kmTotal != lastSavedKmTotal) mudou = true;
    if(kmParcial != lastSavedKmParcial) mudou = true;
    
    // OTIMIZAÇÃO NVS: Se nada mudou, não escreve
    if(!mudou) return;
    
    prefs.begin("veiculo", false);
    prefs.putFloat("scX", scaleX); prefs.putFloat("scY", scaleY);
    prefs.putInt("offX", offsetX); prefs.putInt("offY", offsetY);
    prefs.putFloat("roda", circRodaCM);   // Circunferência roda (persistente)
    prefs.putInt("vel", velocidadeAtual);
    prefs.putFloat("kmT", kmTotal);       // km total (persistente)
    prefs.putFloat("kmP", kmParcial);     // km parcial (persistente)
    prefs.putInt("batUso", bateriaEmUso);
    prefs.end();
    
    // Atualiza valores de referência
    lastSavedScaleX = scaleX;
    lastSavedScaleY = scaleY;
    lastSavedVelocidade = velocidadeAtual;
    lastSavedBateriaEmUso = bateriaEmUso;
    lastSavedCircRodaCM = circRodaCM;
    lastSavedKmTotal = kmTotal;
    lastSavedKmParcial = kmParcial;
    
    Serial.println("[NVS] Configuracoes salvas (dirty check: valores alterados)");
}

// Salva a configuração da bateria em edição (sistema, fator, química, capacidade)
// Namespace: "cyd_storage", chave: "bat_<idx>"
void salvarConfigBateria() {
    prefs.begin("cyd_storage", false);
    char key[12]; 
    snprintf(key, sizeof(key), "bat_%d", batEdicao);
    prefs.putBytes(key, &confBat[batEdicao], sizeof(ConfigBateria));
    prefs.end();
}

// Salva o estado do carregamento solar (liga/desliga e porcentagem de corte)
// Namespace: "cyd_storage"
void salvarConfiguracoesSolar() {
    prefs.begin("cyd_storage", false);
    prefs.putBool("solar_on", solarLigado);
    prefs.putFloat("solar_perc", ajustePorcentagem);
    prefs.end();
}

// ---------------------------------------------------------------------------
// salvarConfigsConducao() - MÓDULO C
// ---------------------------------------------------------------------------
// salvarConfigsConducao()
// ---------------------------------------------------------------------------
// O QUE É: Salva configurações de condução na NVS (namespace "conducao").
// O QUE NÃO É: Não envia as configurações para o Mega (isso é feito separadamente).
// CHAVES NVS:
//   pas_imans: número de ímãs do sensor PAS (0-30, 0=desativado)
//   curva: índice da curva de aceleração (0=Suave, 1=Linear, 2=Agressiva)
//   balanco: distribuição de potência (-5 a +5, 0=equilibrado)
//            Nota: chave NVS mantém nome "balanco" para compatibilidade
// MANUTENÇÃO FUTURA: Se adicionar novo parâmetro de condução, incluir aqui
//   e também em enviarConfigsConducao() para sincronizar com o Mega.
// ---------------------------------------------------------------------------
void salvarConfigsConducao() {
    prefs.begin("conducao", false);
    prefs.putInt("pas_imans", pasImans);
    prefs.putInt("curva", curvaSensibilidade);
    prefs.putInt("balanco", balancoTracao);
    prefs.putBool("radar_on", radarHabilitado);
    prefs.putBool("modo_disf", modoDisfarce);
    prefs.putInt("lim_disf", limiteDisfarce);       // Limite visual do disfarce
    prefs.putInt("lim_vel", limiteVelocidade);       // Limite físico do R2
    prefs.putBool("tema_esc", temaEscuro);
    prefs.end();
}

// ---------------------------------------------------------------------------
// salvarPrecoKm() - MÓDULO G
// ---------------------------------------------------------------------------
// O QUE É: Persiste os parâmetros de preço/km na NVS (namespace "precokm").
// CHAVES NVS (todas ≤15 chars - OBJETIVO 4):
//   potB1 (5), potB2 (5): potência dos carregadores
//   precoKwh (8): preço do kWh
//   custoUltCg (10): custo última carga
//   custoPrj100 (11): projeção custo 100%
//   km_bXfY_Zx3 (11): km acumulados por combinação
// ---------------------------------------------------------------------------
void salvarPrecoKm() {
    Preferences p;
    p.begin("precokm", false);
    p.putFloat("potB1", potCarregB1);
    p.putFloat("potB2", potCarregB2);
    p.putFloat("precoKwh", precoKWh);
    p.putFloat("custoUltCg", custoUltimaCarga);
    p.putFloat("custoPrj100", custoProjCarga100);
    // Salva 12 acumuladores de km: km_bXfY_Zx3
    char key[16];
    for(int b = 0; b < 2; b++) {
        for(int f = 0; f < 3; f++) {
            for(int t = 0; t < 2; t++) {
                snprintf(key, sizeof(key), "km_b%df%d_%dx3", b+1, f+1, t==0?1:3);
                p.putFloat(key, kmAcum[b][f][t]);
            }
        }
    }
    // Salva custos calculados
    for(int b = 0; b < 2; b++) {
        for(int f = 0; f < 3; f++) {
            for(int t = 0; t < 2; t++) {
                snprintf(key, sizeof(key), "ck_b%df%d_%dx3", b+1, f+1, t==0?1:3);
                p.putFloat(key, custoKm[b][f][t]);
            }
        }
    }
    // Salva % consumida por modo (Fix 4: chave "pc_bXfY_Zx3", 11 chars)
    for(int b = 0; b < 2; b++) {
        for(int f = 0; f < 3; f++) {
            for(int t = 0; t < 2; t++) {
                snprintf(key, sizeof(key), "pc_b%df%d_%dx3", b+1, f+1, t==0?1:3);
                p.putFloat(key, percGasto[b][f][t]);
            }
        }
    }
    p.end();
}

// ---------------------------------------------------------------------------
// carregarPrecoKm() - MÓDULO G
// ---------------------------------------------------------------------------
// O QUE É: Restaura parâmetros de preço/km da NVS ao ligar.
// ---------------------------------------------------------------------------
void carregarPrecoKm() {
    Preferences p;
    p.begin("precokm", true);
    potCarregB1 = p.getFloat("potB1", 500.0);
    potCarregB2 = p.getFloat("potB2", 500.0);
    precoKWh = p.getFloat("precoKwh", 0.75);
    custoUltimaCarga = p.getFloat("custoUltCg", 0.0);
    custoProjCarga100 = p.getFloat("custoPrj100", 0.0);
    char key[16];
    for(int b = 0; b < 2; b++) {
        for(int f = 0; f < 3; f++) {
            for(int t = 0; t < 2; t++) {
                snprintf(key, sizeof(key), "km_b%df%d_%dx3", b+1, f+1, t==0?1:3);
                kmAcum[b][f][t] = p.getFloat(key, 0.0);
            }
        }
    }
    for(int b = 0; b < 2; b++) {
        for(int f = 0; f < 3; f++) {
            for(int t = 0; t < 2; t++) {
                snprintf(key, sizeof(key), "ck_b%df%d_%dx3", b+1, f+1, t==0?1:3);
                custoKm[b][f][t] = p.getFloat(key, 0.0);
            }
        }
    }
    // Carrega % consumida por modo (Fix 4)
    for(int b = 0; b < 2; b++) {
        for(int f = 0; f < 3; f++) {
            for(int t = 0; t < 2; t++) {
                snprintf(key, sizeof(key), "pc_b%df%d_%dx3", b+1, f+1, t==0?1:3);
                percGasto[b][f][t] = p.getFloat(key, 0.0);
            }
        }
    }
    p.end();
}

// ---------------------------------------------------------------------------
// detectarCargaAuto() - MÓDULO G (Core 0)
// ---------------------------------------------------------------------------
// O QUE É: Detecta início/fim de carga pela subida contínua de voltagem.
// REGRA: Se a voltagem subiu >0.5V desde a última leitura E veículo parado
//        (kmh < 0.5), considera que a bateria está carregando.
// AO FIM DA CARGA: calcula custo da sessão e projeção 0→100%.
// ISOLAMENTO (OBJETIVO 4): usa bateriaEmUso para selecionar potCarregB1/B2.
// MUTEX: protege escrita em custoUltimaCarga e custoProjCarga100.
// ---------------------------------------------------------------------------
void detectarCargaAuto() {
    // Seleciona voltagem da bateria em uso
    float vAtual = (bateriaEmUso == 1) ? vB1 : vB2;
    int batIdx = (bateriaEmUso == 1) ? 0 : 1;
    int percAtual = calcularPorcentagem(batIdx, vAtual);

    if(!cargaDetectada) {
        // Detecção de INÍCIO: voltagem subiu >0.5V e veículo parado
        if(cargaVoltaAnterior > 0 && (vAtual - cargaVoltaAnterior) > 0.5 && kmh < 0.5) {
            cargaDetectada = true;
            cargaBateria = bateriaEmUso;
            cargaVoltaInicio = vAtual;
            cargaPercInicio = percAtual;
            cargaTempoInicio = millis();
            cargaGanhoPerc = 0;
            cargaTempoH = 0.0;
        }
    } else {
        // Carga em andamento — atualiza métricas
        cargaGanhoPerc = percAtual - cargaPercInicio;
        cargaTempoH = (float)(millis() - cargaTempoInicio) / 3600000.0;

        // Detecção de FIM: voltagem caiu (carregador desconectado) ou veículo moveu
        if((cargaVoltaAnterior - vAtual) > 0.3 || kmh > 1.0) {
            cargaDetectada = false;

            // Calcula custo da sessão
            // Potência do carregador correto (OBJETIVO 4: isolamento por bateria)
            float potW = (cargaBateria == 1) ? potCarregB1 : potCarregB2;

            // Energia consumida = potência × tempo (kWh)
            float energiaKWh = (cargaTempoH * potW) / 1000.0;

            // Custo desta sessão = energia × preço
            float custoSessao = energiaKWh * precoKWh;

            // Projeção para 100%: se ganhou X% custou Y, 100% custa Y×(100/X)
            float proj100 = 0.0;
            if(cargaGanhoPerc > 0) {
                proj100 = custoSessao * (100.0 / (float)cargaGanhoPerc);
            }

            // Mutex para escrita segura (OBJETIVO 4)
            if(xSemaphoreTake(telemetriaMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                custoUltimaCarga = custoSessao;
                custoProjCarga100 = proj100;
                xSemaphoreGive(telemetriaMutex);
            }

            // Persiste na NVS
            salvarPrecoKm();
        }
    }
    // Atualiza voltagem anterior para próxima comparação
    cargaVoltaAnterior = vAtual;
}

// ---------------------------------------------------------------------------
// acumularKmPorModo() - MÓDULO G (Core 0)
// ---------------------------------------------------------------------------
// O QUE É: Acumula km rodados na combinação correta [bateria][força][tração].
// RASTREAMENTO: Usa bateriaEmUso (1/2), velocidadeAtual (1/2/3), modo3x3 (bool)
//   para determinar o índice no array kmAcum[2][3][2].
// DELTA: Calcula diferença de kmTotal desde última chamada.
// MUTEX (OBJETIVO 4): protege escrita em kmAcum[][][].
// NVS: salva a cada 60s (controlado pelo caller em tarefaTelemetriaFunc).
// ---------------------------------------------------------------------------
static float kmAcumAnterior = 0.0;  // Último kmTotal registrado para delta

void acumularKmPorModo() {
    float kmAtual = kmTotal;

    // Inicializa na primeira chamada
    if(kmAcumAnterior == 0.0 && kmAtual > 0) {
        kmAcumAnterior = kmAtual;
        return;
    }

    float delta = kmAtual - kmAcumAnterior;
    if(delta <= 0.0) {
        kmAcumAnterior = kmAtual;
        return;  // Sem avanço ou reset de km
    }

    // Mapeia para índices do array
    int batIdx = (bateriaEmUso == 1) ? 0 : 1;       // 0=B1, 1=B2
    int forcaIdx = velocidadeAtual - 1;               // 0=F1, 1=F2, 2=F3
    if(forcaIdx < 0) forcaIdx = 0;
    if(forcaIdx > 2) forcaIdx = 2;
    int tracIdx = modo3x3 ? 1 : 0;                   // 0=1x3, 1=3x3

    // Calcula delta de porcentagem consumida (Fix 4)
    int batIdx0 = (bateriaEmUso == 1) ? 0 : 1;
    int percAtual = calcularPorcentagem(batIdx0, (bateriaEmUso == 1) ? vB1 : vB2);
    float deltPerc = 0.0;
    if(percAnterior >= 0 && percAtual < percAnterior) {
        deltPerc = (float)(percAnterior - percAtual); // % que caiu
    }
    percAnterior = percAtual;

    // Mutex para escrita segura (OBJETIVO 4)
    if(xSemaphoreTake(telemetriaMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        kmAcum[batIdx][forcaIdx][tracIdx] += delta;
        if(deltPerc > 0.0) {
            percGasto[batIdx][forcaIdx][tracIdx] += deltPerc;
        }
        xSemaphoreGive(telemetriaMutex);
    }

    kmAcumAnterior = kmAtual;
}

// ---------------------------------------------------------------------------
// calcularCustoKm() - MÓDULO G (Core 0)
// ---------------------------------------------------------------------------
// O QUE É: Calcula o R$/km para cada combinação bat/força/tração.
// FÓRMULA CORRIGIDA (Fix 4):
//   custoKm = custoProjCarga100 * percGasto / (100 * kmAcum)
//   Exemplo: 30km rodados, bateria caiu 50%, custo carga 100% = R$10
//   custoKm = 10 * 50 / (100 * 30) = R$ 0,1667/km
//   Equivale a: autonomia estimada = 30 * 100/50 = 60km, custo/km = 10/60
// MUTEX (OBJETIVO 4): protege leitura de kmAcum/percGasto e escrita de custoKm.
// ---------------------------------------------------------------------------
void calcularCustoKm() {
    if(xSemaphoreTake(telemetriaMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for(int b = 0; b < 2; b++) {
            float custoBase = custoProjCarga100;

            for(int f = 0; f < 3; f++) {
                for(int t = 0; t < 2; t++) {
                    // Fix 4: Fórmula correta usando % consumida e km rodados
                    if(kmAcum[b][f][t] > 0.1 && percGasto[b][f][t] > 0.1 && custoBase > 0.0) {
                        custoKm[b][f][t] = (custoBase * percGasto[b][f][t]) / (100.0 * kmAcum[b][f][t]);
                    } else {
                        custoKm[b][f][t] = 0.0;
                    }
                }
            }
        }
        xSemaphoreGive(telemetriaMutex);
    }
}

// ---------------------------------------------------------------------------
// carregarConfiguracoes()
// ---------------------------------------------------------------------------
// Carrega TODAS as configurações da NVS ao iniciar o sistema.
// Se uma chave não existir (primeiro boot), usa valores padrão:
//   - Touch: scale=1.0, offset=0 (sem correção)
//   - Roda: 15cm circunferência
//   - Velocidade: nível 1
//   - Bateria: sistema 48V, fator 1.0, Lítio, 10Ah
//   - Solar: desligado, corte 50%
//   - PAS: 12 ímãs, curva Linear, balanço 50%
// ---------------------------------------------------------------------------
void carregarConfiguracoes() {
    prefs.begin("veiculo", true);
    scaleX = prefs.getFloat("scX", 1.0); scaleY = prefs.getFloat("scY", 1.0);
    offsetX = prefs.getInt("offX", 0); offsetY = prefs.getInt("offY", 0);
    circRodaCM = prefs.getFloat("roda", 196.0); // 196cm = roda 26" típica
    velocidadeAtual = prefs.getInt("vel", 1);
    kmTotal = prefs.getFloat("kmT", 0.0);
    kmParcial = prefs.getFloat("kmP", 0.0);
    bateriaEmUso = prefs.getInt("batUso", 1);
    prefs.end();

    prefs.begin("cyd_storage", true);
    for(int i = 0; i < 3; i++) {
        char key[12];
        snprintf(key, sizeof(key), "bat_%d", i);
        if (prefs.getBytes(key, &confBat[i], sizeof(ConfigBateria)) == 0) {
            confBat[i] = {48, 1.0, 1, 10};
        }
    }
    solarLigado = prefs.getBool("solar_on", false);
    ajustePorcentagem = prefs.getFloat("solar_perc", 50.0);
    prefs.end();
    
    // MÓDULO C: Carrega configs de condução
    prefs.begin("conducao", true);
    pasImans = prefs.getInt("pas_imans", 0);  // Seção 5: Default 0 (desativado) - corrige bug "0/Desativado"
    curvaSensibilidade = prefs.getInt("curva", 1);
    balancoTracao = prefs.getInt("balanco", 0);
    radarHabilitado = prefs.getBool("radar_on", true);  // Default: habilitado
    modoDisfarce = prefs.getBool("modo_disf", false);
    limiteDisfarce = prefs.getInt("lim_disf", 35);        // Default: 35 km/h visual
    limiteVelocidade = prefs.getInt("lim_vel", 25);       // Default: 25 km/h físico
    temaEscuro = prefs.getBool("tema_esc", true);  // Default: Dark Mode
    prefs.end();

    // Carrega o MAC do HC-06 salvo de um pareamento anterior
    carregarMacBT();
    
    // DIRTY CHECK: Inicializa valores de referência com os dados carregados
    // Isto garante que a primeira comparação após o boot NÃO dispare escrita
    // desnecessária (os dados acabaram de ser lidos da NVS, não mudaram).
    lastSavedCircRodaCM = circRodaCM;
    lastSavedKmTotal = kmTotal;
    lastSavedKmParcial = kmParcial;
    lastSavedVelocidade = velocidadeAtual;
    lastSavedBateriaEmUso = bateriaEmUso;
    lastSavedScaleX = scaleX;
    lastSavedScaleY = scaleY;
}

// =============================================================================
// MÓDULO A: FUNÇÕES DE COMUNICAÇÃO SERIAL COM ARDUINO MEGA
// =============================================================================
// REGRA OBRIGATÓRIA: Proibido usar classe String!
// Todos os comandos são montados com char arrays + snprintf().
//
// enviarComandoRele(numero, estado)
// Envia comando para ligar/desligar um relé específico no Mega.
// Formato serial: R,<numero>,<estado>\n
//   <numero>: 1-15 (corresponde aos relés R1-R15, pinos 22-36 do Mega)
//   <estado>: 0=desligado (LOW), 1=ligado (HIGH)
// Exemplo: "R,10,1\n" liga o Ventilador (R10, pino 31)
// ---------------------------------------------------------------------------
void enviarComandoRele(int numero, int estado) {
    char cmd[48];
    // Seção 6: Adiciona checksum simples (XOR de todos os bytes antes do *)
    snprintf(cmd, sizeof(cmd), "R,%d,%d", numero, estado);
    uint8_t chk = 0;
    for(int i = 0; cmd[i] != '\0'; i++) chk ^= (uint8_t)cmd[i];
    char final_cmd[48];
    snprintf(final_cmd, sizeof(final_cmd), "%s*%02X\n", cmd, chk);
    SerialBT.print(final_cmd);
}

// ---------------------------------------------------------------------------
// enviarResetGeral()
// ---------------------------------------------------------------------------
// Envia RESET\n para o Mega, que desliga TODOS os 15 relés (pinos 22-36 LOW).
// Também reseta o estado local dos switches de teste manual.
// Usado pelo botão RESET GERAL (laranja) nos menus de manutenção.
// SEGURANÇA: Garante que nenhum relé fique ligado após o reset.
// ---------------------------------------------------------------------------
void enviarResetGeral() {
    SerialBT.print("RESET\n");
    
    // Reseta estados locais
    for(int i=0; i<15; i++) {
        estadoRelesTeste[i] = false;
    }
    
    // Para teste automático se ativo
    testeAutomaticoAtivo = false;
}

// =============================================================================
// SINCRONIZAÇÃO DE ESTADO DOS RELÉS (CYD → MEGA)
// =============================================================================
// enviarStatusReles()
// Envia o estado atual de todos os relés controlados pelo CYD para o Mega.
// Chamada a cada 500ms no loop() e após qualquer mudança de estado.
// Relés sincronizados: R1 (motor), R2 (limitador), R3 (bateria),
//   R4 (marcha ré), R5/R6 (velocidade 1/2/3)
// R8/R9 (solar) são controlados diretamente pelos GPIOs do CYD.
//
// ANÁLISE DE BUG (TIMING):
// Esta função envia 5-6 comandos Serial em sequência rápida.
// A 115200 baud, cada comando (~10 bytes) leva ~0.9ms.
// Total: ~5ms para todos os comandos. Não bloqueia o loop.
// O Mega processa os comandos na ordem de chegada (buffer Serial 64 bytes).
// ---------------------------------------------------------------------------
// Seção 6: Envia pacote de sincronização para Mega e Remote
void enviarSyncParam(const char* param, float valor) {
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "SYNC,%s,%.2f\n", param, valor);
    SerialBT.print(cmd);
}

void enviarStatusReles() {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "R,1,%d\n", modo3x3 ? 1 : 0);
    SerialBT.print(cmd);
    
    // R2 (Limitador Físico): Controlado APENAS por limitadorLigado.
    // O Modo Disfarce NÃO interfere no R2 (são funções independentes).
    // limitadorLigado==true → R2=0 (relé acionado = corte real no Mega)
    // limitadorLigado==false → R2=1 (relé desligado = potência total)
    snprintf(cmd, sizeof(cmd), "R,2,%d\n", limitadorLigado ? 0 : 1);
    SerialBT.print(cmd);
    // Envia o valor do limite físico ao Mega (só quando limitador ativo)
    if(limitadorLigado) {
        snprintf(cmd, sizeof(cmd), "LIMVEL,%d\n", limiteVelocidade);
        SerialBT.print(cmd);
    }
    
    snprintf(cmd, sizeof(cmd), "R,3,%d\n", (bateriaEmUso == 2) ? 1 : 0);
    SerialBT.print(cmd);
    
    snprintf(cmd, sizeof(cmd), "R,4,%d\n", modoReLigado ? 1 : 0);
    SerialBT.print(cmd);
    
    if (velocidadeAtual == 1) { 
        SerialBT.print("R,5,1\n"); 
        SerialBT.print("R,6,0\n"); 
    }
    else if (velocidadeAtual == 2) { 
        SerialBT.print("R,5,0\n"); 
        SerialBT.print("R,6,0\n"); 
    }
    else if (velocidadeAtual == 3) { 
        SerialBT.print("R,5,0\n"); 
        SerialBT.print("R,6,1\n"); 
    }
}

// Envia a circunferência da roda para o Mega
// Formato: C,R,<valor>\n (usado para cálculo de velocidade/km no Mega)
void enviarConfigRoda() {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "C,R,%.2f\n", circRodaCM);
    SerialBT.print(cmd);
}

// ---------------------------------------------------------------------------
// enviarConfigsConducao() - MÓDULO C
// ---------------------------------------------------------------------------
// Envia as 3 configurações de condução para o Arduino Mega:
//   PAS,<imas>\n    - número de ímãs do sensor PAS
//   CURVA,<idx>\n   - curva de aceleração (0/1/2)
//   BALANCO,<val>\n - distribuição de potência (0-100)
// O Mega aplica estas configurações no controle do acelerador e PID.
// ---------------------------------------------------------------------------
void enviarConfigsConducao() {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "PAS,%d\n", pasImans);
    SerialBT.print(cmd);
    
    snprintf(cmd, sizeof(cmd), "CURVA,%d\n", curvaSensibilidade);
    SerialBT.print(cmd);
    
    // Converte escala -5/+5 para 0-100 para o Mega
    // Fórmula: valorMega = (balancoTracao + 5) * 10
    // -5 -> 0 (100% traseiro), 0 -> 50 (equilibrado), +5 -> 100 (100% dianteiro)
    int valorMegaBalanco = (balancoTracao + 5) * 10;
    snprintf(cmd, sizeof(cmd), "BALANCO,%d\n", valorMegaBalanco);
    SerialBT.print(cmd);
}

// =============================================================================
// NAVEGAÇÃO ENTRE PÁGINAS (BOTÕES LATERAIS)
// =============================================================================
// Callback dos botões de navegação lateral (< e >) e submenu (VOLTAR/HOME).
// user_data indica a ação: 0=anterior, 1=próxima, 2=HOME, 3=VOLTAR.
// As páginas circulam de 0 a totalPaginas-1 (14 páginas).
// Ao navegar, emSubMenu é resetado para false (volta à visão geral).
// ---------------------------------------------------------------------------
static void nav_event_cb(lv_event_t * e) {
    int acao = (int)(intptr_t)lv_event_get_user_data(e);
    if (acao == 2) { paginaAtual = MENU_HODOMETRO; emSubMenu = false; }
    else if (acao == 3) { emSubMenu = false; }
    else {
        // BUG CORRIGIDO: Usa navPaginas[] para mapear corretamente
        // Antes usava (Pagina)(index % totalPaginas) que mapeava
        // index 13 → CONFIG_BAT em vez de MANUTENCAO_MENU
        int navIdx = obterNavIndex(paginaAtual);
        if(acao == 1) navIdx = (navIdx + 1) % totalNavPaginas;
        else navIdx = (navIdx - 1 + totalNavPaginas) % totalNavPaginas;
        paginaAtual = navPaginas[navIdx];
        emSubMenu = false;
    }
    montarInterface();
}

// =============================================================================
// ATUALIZAÇÃO DO PAINEL PRINCIPAL (TELA INICIAL)
// =============================================================================
// Atualiza os textos do painel principal com dados de telemetria em tempo real.
// Chamada a cada 500ms pelo loop() (APENAS quando !splashScreenAtiva).
//
// DADOS EXIBIDOS:
//   Linha 1: Tensões B1/B2/B3 com porcentagem de carga
//   Linha 2: Corrente de tração (Amperes) e velocidade (km/h)
//   Linha 3: Estado dos relés (Motor, Limitador, Velocidade, Ré)
//   Linha 4: Quilometragem total/parcial + cruise control (se ativo)
//   Linha 5: Autonomia estimada e RPM
//
// LÓGICA DE TROCA AUTOMÁTICA DE BATERIA:
//   Se B1 < 10% e B2 >= 10% → troca para B2
//   Se B2 < 10% e B1 >= 10% → troca para B1
//   Após troca: atualiza relés e salva config na NVS.
//
// ATENÇÃO: Só atualiza se paginaAtual == PRINCIPAL.
// Os labels (label_baterias, etc) devem existir (criados em montarInterface).
// BUG CORRIGIDO: Não é chamada durante splash (labels seriam NULL = crash).
// ---------------------------------------------------------------------------
void atualizarDadosPrincipais() {
    // Protege leitura de variáveis compartilhadas entre cores (Core 0 escreve, Core 1 lê)
    if (xSemaphoreTake(telemetriaMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        int p1 = calcularPorcentagem(0, vB1);
        int p2 = calcularPorcentagem(1, vB2);
        int p3 = calcularPorcentagem(2, vB3);
        // Copia valores voláteis para uso local
        float localV1 = vB1;
        float localV2 = vB2;
        float localV3 = vB3;
        float localAmp = ampere;
        float localKmH = kmh;
        float localKmTotal = kmTotal;
        float localKmParcial = kmParcial;
        int localBateriaEmUso = bateriaEmUso;
        xSemaphoreGive(telemetriaMutex);

    bool mudouBat = false;
    if (bateriaEmUso == 1 && p1 < 10 && p2 >= 10) { bateriaEmUso = 2; mudouBat = true; }
    else if (bateriaEmUso == 2 && p2 < 10 && p1 >= 10) { bateriaEmUso = 1; mudouBat = true; }

    if(mudouBat) { enviarStatusReles(); salvarConfiguracoes(); }

    // Atualiza hodômetro se estiver ativo
    if (paginaAtual == MENU_HODOMETRO) { atualizar_menu_hodometro(); return; }
    
    if (paginaAtual != PRINCIPAL) return;
    const char* tagB1 = (bateriaEmUso == 1) ? "#FF0000 B1#" : "B1";
    const char* tagB2 = (bateriaEmUso == 2) ? "#FF0000 B2#" : "B2";

    // Baterias sem relógio (RTC removido - Seção 1)
    lv_label_set_text_fmt(label_baterias, "B1:#0000FF %.1fV# #00FF00 %d%%#  B2:#0000FF %.1fV# #00FF00 %d%%#\nB3:#0000FF %.1fV# #00FF00 %d%%#  %s %s",
                          vB1, p1, vB2, p2, vB3, p3, (bateriaEmUso == 1 ? tagB1 : ""), (bateriaEmUso == 2 ? tagB2 : ""));
    // MODO DISFARCE (filtro visual): aplica cap na velocidade exibida
    //   Se real < limiteDisfarce → mostra real. Se real >= limiteDisfarce → trava no limite.
    float kmhExibida = kmh;
    if(modoDisfarce && kmh > (float)limiteDisfarce) {
        kmhExibida = (float)limiteDisfarce;
    }
    
    // MÓDULO D: Exibe corrente + velocidade (filtrada pelo disfarce se ativo)
    // Velocidade em AZUL para destaque visual durante condução
    lv_label_set_text_fmt(label_dados, "#0000FF %.1f# A    #0000FF %.1f KM/H#", ampere, kmhExibida);
    
    // Status: Motor + Limitador Físico + Disfarce Visual + Velocidade + Ré
    lv_label_set_text_fmt(label_status, "MOT:%s  LIM:%s  %sV:#0000FF %d#  RE:%s", 
                          modo3x3?"#0000FF 3x3#":"#0000FF 1x3#", 
                          limitadorLigado?"#FF8800 L#":"#FF0000 D#",
                          modoDisfarce?"#FF0000 DF# ":" ",
                          velocidadeAtual, 
                          modoReLigado?"#00FF00 ON#":"#FF0000 OFF#");
    
    // MÓDULO C: Adiciona indicador de cruise control
    if(cruiseAtivo) {
        lv_label_set_text_fmt(label_km, "T: %.1f KM    P: #0000FF %.1f# KM  #00FF00 CRUISE: %.0f#", 
                              kmTotal, kmParcial, cruiseVelocidadeAlvo);
    } else {
        lv_label_set_text_fmt(label_km, "T: %.1f KM    P: #0000FF %.1f# KM", kmTotal, kmParcial);
    }
    
    lv_label_set_text_fmt(label_rodape, "AUT: #0000FF %.1f KM#    #FF0000 %d RPM#", autonomia, rpm);
}

// =============================================================================
// MENU HODÔMETRO: NOVA TELA HOME (PAINEL VELOCÍMETRO)
// =============================================================================
// criar_menu_hodometro(): Monta o layout visual no container passado.
//   - Velocidade central grande (montserrat_28 escalada 2x via transform_zoom)
//   - Autonomia (sup. esquerdo), Energia (sup. direito)
//   - ODO/TRIP (inf. esquerdo), Motor/Lim/Vel (inf. direito)
// atualizar_menu_hodometro(): Atualiza valores e cor da velocidade.
//   - Azul (0-32 km/h), Vermelho (>32 km/h)
// ---------------------------------------------------------------------------
void criar_menu_hodometro(lv_obj_t * cont) {
    // Fundo dinâmico: preto (escuro) ou branco (claro)
    lv_color_t corFundo = temaEscuro ? lv_color_hex(0x000000) : lv_color_hex(0xFFFFFF);
    lv_color_t corTexto = temaEscuro ? lv_color_white() : lv_color_black();
    lv_obj_set_style_bg_color(cont, corFundo, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
    
    // --- VELOCIDADE CENTRAL GIGANTE (apenas número, sem "km/h") ---
    // montserrat_28 escalada ~3.5x via transform_zoom (896/256 ≈ 3.5x → ~98px efetivos)
    hodo_lbl_velocidade = lv_label_create(cont);
    lv_label_set_text(hodo_lbl_velocidade, "0");
    lv_obj_set_style_text_font(hodo_lbl_velocidade, FONTE_GRANDE, 0);
    lv_obj_set_style_transform_zoom(hodo_lbl_velocidade, 896, 0);
    lv_obj_set_style_text_color(hodo_lbl_velocidade, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_align(hodo_lbl_velocidade, LV_ALIGN_CENTER, 0, -5);
    
    // --- TOPO ESQUERDO: Autonomia ---
    hodo_lbl_autonomia = lv_label_create(cont);
    lv_label_set_text(hodo_lbl_autonomia, "Est. -- km");
    lv_obj_set_style_text_font(hodo_lbl_autonomia, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hodo_lbl_autonomia, corTexto, 0);
    lv_obj_set_width(hodo_lbl_autonomia, 140);
    lv_label_set_long_mode(hodo_lbl_autonomia, LV_LABEL_LONG_CLIP);
    lv_obj_align(hodo_lbl_autonomia, LV_ALIGN_TOP_LEFT, 8, 8);
    
    // --- TOPO DIREITO: Voltagem + % ---
    hodo_lbl_energia = lv_label_create(cont);
    lv_label_set_recolor(hodo_lbl_energia, true);
    lv_label_set_text(hodo_lbl_energia, "-- V  --%");
    lv_obj_set_style_text_font(hodo_lbl_energia, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hodo_lbl_energia, corTexto, 0);
    lv_obj_set_width(hodo_lbl_energia, 140);
    lv_label_set_long_mode(hodo_lbl_energia, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(hodo_lbl_energia, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(hodo_lbl_energia, LV_ALIGN_TOP_RIGHT, -8, 8);
    
    // --- BASE ESQUERDA: ODO + TRIP ---
    hodo_lbl_km = lv_label_create(cont);
    lv_label_set_text(hodo_lbl_km, "ODO: 0 km\nTRIP: 0 km");
    lv_obj_set_style_text_font(hodo_lbl_km, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hodo_lbl_km, corTexto, 0);
    lv_obj_set_width(hodo_lbl_km, 150);
    lv_label_set_long_mode(hodo_lbl_km, LV_LABEL_LONG_CLIP);
    lv_obj_align(hodo_lbl_km, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    
    // --- BASE DIREITA: Motor + Limitador + Velocidade ---
    hodo_lbl_motor = lv_label_create(cont);
    lv_label_set_recolor(hodo_lbl_motor, true);
    lv_label_set_text(hodo_lbl_motor, "1x3  D  V1");
    lv_obj_set_style_text_font(hodo_lbl_motor, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hodo_lbl_motor, corTexto, 0);
    lv_obj_set_width(hodo_lbl_motor, 150);
    lv_label_set_long_mode(hodo_lbl_motor, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(hodo_lbl_motor, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(hodo_lbl_motor, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    
    // Atualiza com valores reais
    atualizar_menu_hodometro();
}

// ---------------------------------------------------------------------------
// atualizar_menu_hodometro() - Atualiza valores e cores em tempo real
// Chamada a cada 500ms pelo loop() (igual a atualizarDadosPrincipais)
// ---------------------------------------------------------------------------
void atualizar_menu_hodometro() {
    if(paginaAtual != MENU_HODOMETRO) return;
    if(hodo_lbl_velocidade == NULL) return;
    
    // Velocidade central: inteiro para visual limpo
    // MODO DISFARCE (filtro visual):
    //   Se real < limiteDisfarce → mostra velocidade real (acompanha aceleração/desaceleração)
    //   Se real >= limiteDisfarce → trava no limiteDisfarce (cap visual)
    //   Exemplo: limiteDisfarce=35, real=50 → mostra 35
    //            limiteDisfarce=35, real=20 → mostra 20
    int velReal = (int)(kmh + 0.5);
    int velExibida = velReal;
    if(modoDisfarce && velReal > limiteDisfarce) {
        velExibida = limiteDisfarce;  // Cap visual: trava no limite configurado
    }
    lv_label_set_text_fmt(hodo_lbl_velocidade, "%d", velExibida);
    // Cor dinâmica baseada no valor EXIBIDO: Azul (<=32 km/h), Vermelho (>32 km/h)
    if(velExibida > 32)
        lv_obj_set_style_text_color(hodo_lbl_velocidade, lv_palette_main(LV_PALETTE_RED), 0);
    else
        lv_obj_set_style_text_color(hodo_lbl_velocidade, lv_palette_main(LV_PALETTE_BLUE), 0);
    
    // Autonomia (topo esquerdo)
    lv_label_set_text_fmt(hodo_lbl_autonomia, "Est. %.0f km", autonomia);
    
    // Energia: voltagem + % da bateria em uso (topo direito)
    float vAtual = (bateriaEmUso == 1) ? vB1 : vB2;
    int batIdx = (bateriaEmUso == 1) ? 0 : 1;
    int perc = calcularPorcentagem(batIdx, vAtual);
    // Recolor: cores brilhantes que funcionam em ambos os temas
    lv_label_set_text_fmt(hodo_lbl_energia, "#2196F3 %.1fV#  #4CAF50 %d%%#", vAtual, perc);
    
    // ODO + TRIP (base esquerda) — inteiros para leitura limpa
    lv_label_set_text_fmt(hodo_lbl_km, "ODO: %d km\nTRIP: %d km", (int)kmTotal, (int)kmParcial);
    
    // Motor + Limitador Físico + Disfarce + Velocidade (base direita)
    if(modoDisfarce) {
        lv_label_set_text_fmt(hodo_lbl_motor, "%s  %s  #F44336 DF#  #2196F3 V%d#",
                              modo3x3 ? "#2196F3 3x3#" : "1x3",
                              limitadorLigado ? "#FF8800 L#" : "#F44336 D#",
                              velocidadeAtual);
    } else {
        lv_label_set_text_fmt(hodo_lbl_motor, "%s  %s  #2196F3 V%d#",
                              modo3x3 ? "#2196F3 3x3#" : "1x3",
                              limitadorLigado ? "#FF8800 L#" : "#F44336 D#",
                              velocidadeAtual);
    }
}

// ---------------------------------------------------------------------------
// aplicar_tema() - Sistema de Temas Claro/Escuro
// ---------------------------------------------------------------------------
// Salva na NVS, recria a interface com as cores atualizadas.
// As telas usam temaEscuro para selecionar fundo/texto dinamicamente.
// REGRA: Valores monetários (*R$ X,XX*) sempre ficam em Azul independentemente do tema.
// ---------------------------------------------------------------------------
void aplicar_tema(bool escuro) {
    temaEscuro = escuro;
    salvarConfigsConducao();
    montarInterface();
}

// =============================================================================
// MÓDULO F: SPLASH SCREEN (INICIALIZAÇÃO LIMPA)
// =============================================================================
// A splash screen é montada UMA ÚNICA VEZ no setup().
// O loop() apenas incrementa splashProgress e atualiza o valor da barra.
// NÃO chama montarSplashScreen() repetidamente (causava tela branca piscando).
//
// FLUXO DE BOOT SIMPLIFICADO:
//   1. setup() chama montarSplashScreen() uma vez
//   2. loop() incrementa splashProgress a cada 100ms (+5 por ciclo = ~2s total)
//   3. Quando splashProgress >= 100, chama montarInterface() e encerra splash
//   4. NÃO há bloqueio por sistemaReady (removido)
//   5. NÃO há teste automático de relés no boot (movido para menu Manutenção)
//   6. NÃO há botão Pular (não é necessário, boot nunca trava)
//
// LAYOUT DA TELA:
//   [centro-superior] "Computador de Bordo" (montserrat_28, branco)
//   [centro]          "Veiculos Eletricos"  (montserrat_22, ciano)
//   [inferior]        Barra de progresso fina (6px, ciano sobre cinza)
//   [rodapé]          "feito por (Felipe Albano)" (montserrat_14, cinza)
//
// PREVENÇÃO DE MEMORY LEAK:
//   splashBar é um ponteiro global. Ao sair da splash, é setado NULL.
//   A chamada lv_obj_clean() dentro de montarInterface() destrói todos os
//   objetos da splash de uma vez (LVGL gerencia memória internamente).
//   Não há objetos criados dinamicamente fora da árvore LVGL.
// ---------------------------------------------------------------------------

// Ponteiro global para a barra de progresso da splash.
// Armazenado para que o loop() possa atualizar o valor SEM recriar a tela.
// Quando a splash termina, é setado NULL para evitar acesso a objeto destruído.
lv_obj_t * splashBar = NULL;

// ---------------------------------------------------------------------------
// montarSplashScreen()
// ---------------------------------------------------------------------------
// Monta a tela de splash UMA ÚNICA VEZ (chamada apenas no setup).
// Cria os objetos LVGL estáticos: título, subtítulo, barra, assinatura.
// O ponteiro da barra (splashBar) fica disponível para atualização no loop().
//
// IMPORTANTE: Esta função NÃO deve ser chamada no loop().
// Se chamada repetidamente, causaria:
//   - lv_obj_clean() destruiria todos os objetos a cada frame
//   - Tela branca piscando (objetos recriados antes de serem renderizados)
//   - Consumo desnecessário de CPU e memória
// ---------------------------------------------------------------------------
void montarSplashScreen() {
    // Obtém a tela ativa do LVGL (a tela raiz, sempre existe)
    lv_obj_t * scr = lv_scr_act();

    // Remove TODOS os objetos filhos da tela (limpeza total)
    // Isto é seguro pois é chamado apenas uma vez no setup()
    lv_obj_clean(scr);
    
    // Fundo preto sólido para visual limpo e profissional
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // ===== TÍTULO PRINCIPAL: "Computador de Bordo" =====
    // Fonte montserrat_28 (maior fonte padrão LVGL ativada)
    // Cor branca sobre fundo preto = alto contraste, visual premium
    // Centralizado horizontalmente, levemente acima do centro vertical
    lv_obj_t * titulo = lv_label_create(scr);
    lv_label_set_text(titulo, "Computador de Bordo");
    lv_obj_set_style_text_font(titulo, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(titulo, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(titulo, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(titulo, LV_ALIGN_CENTER, 0, -30);

    // ===== SUBTÍTULO: "Veiculos Eletricos" =====
    // Fonte montserrat_22, cor ciano (0x00FFFF) para destaque visual
    // Posicionado logo abaixo do título, centralizado
    lv_obj_t * subtitulo = lv_label_create(scr);
    lv_label_set_text(subtitulo, "Veiculos Eletricos");
    lv_obj_set_style_text_font(subtitulo, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(subtitulo, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_text_align(subtitulo, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(subtitulo, LV_ALIGN_CENTER, 0, 10);
    
    // ===== BARRA DE PROGRESSO (fina, moderna, parte inferior) =====
    // Altura: 6px (fina e elegante, não compete com os textos)
    // Largura: 280px (deixa margem de 20px de cada lado em tela de 320px)
    // Fundo: cinza escuro (0x222222) - trilho da barra
    // Indicador: ciano (0x00FFFF) - preenchimento animado da esquerda p/ direita
    // Bordas arredondadas (radius 3px) tanto no fundo quanto no indicador
    // Posição: 50px acima do fundo da tela (espaço para assinatura abaixo)
    splashBar = lv_bar_create(scr);
    lv_obj_set_size(splashBar, 280, 6);
    lv_obj_align(splashBar, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_set_style_bg_color(splashBar, lv_color_hex(0x222222), 0);
    lv_obj_set_style_bg_color(splashBar, lv_color_hex(0x00FFFF), LV_PART_INDICATOR);
    lv_obj_set_style_radius(splashBar, 3, 0);
    lv_obj_set_style_radius(splashBar, 3, LV_PART_INDICATOR);
    lv_bar_set_value(splashBar, splashProgress, LV_ANIM_ON);

    // ===== ASSINATURA: "feito por (Felipe Albano)" =====
    // Fonte montserrat_14 (pequena e discreta), cor cinza claro
    // Posicionada centralizada abaixo da barra de progresso, perto do fundo
    lv_obj_t * assinatura = lv_label_create(scr);
    lv_label_set_text(assinatura, "feito por (Felipe Albano)");
    lv_obj_set_style_text_font(assinatura, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(assinatura, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(assinatura, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(assinatura, LV_ALIGN_BOTTOM_MID, 0, -15);
}

// =============================================================================
// MÓDULO A: TESTE AUTOMÁTICO DE RELÉS (SEQUENCIADOR)
// =============================================================================
// Chamada a cada iteração do loop(). Se testeAutomaticoAtivo == false,
// retorna imediatamente (custo zero de CPU).
//
// SEQUÊNCIA:
//   1. Aguarda 1 segundo (millis - testeAutomaticoTimer < 1000)
//   2. Desliga o relé anterior (enviarComandoRele com estado 0)
//   3. Avança para o próximo relé (testeAutomaticoIndice++)
//   4. Se passou de R15 (>15), encerra o teste
//   5. Caso contrário, liga o novo relé e atualiza a interface
//
// A interface mostra o nome do relé ativo e uma barra de progresso.
// O botão RESET GERAL pode interromper o teste a qualquer momento.
//
// ANÁLISE DE BUG (LOOP INFINITO):
// Não há risco de loop infinito. O índice é incrementado a cada 1s e
// o teste termina quando índice > 15 (máximo 15 iterações = 15 segundos).
// ---------------------------------------------------------------------------
void processarTesteAutomatico() {
    if(!testeAutomaticoAtivo) return;
    
    unsigned long agora = millis();
    if(agora - testeAutomaticoTimer < 3000) return;  // 3 segundos por relé (tempo para ouvir estalo)
    
    // Desliga relé anterior
    if(testeAutomaticoIndice > 0) {
        enviarComandoRele(testeAutomaticoIndice, 0);
    }
    
    // Próximo relé
    testeAutomaticoIndice++;
    
    if(testeAutomaticoIndice > 15) {
        // Fim do teste
        testeAutomaticoAtivo = false;
        testeAutomaticoIndice = 0;
        montarInterface();
        return;
    }
    
    // Liga relé atual
    enviarComandoRele(testeAutomaticoIndice, 1);
    testeAutomaticoTimer = agora;
    
    // Atualiza interface
    montarInterface();
}

// =============================================================================
// INTERFACE GRÁFICA PRINCIPAL (UNIFICADA COM TODOS OS MÓDULOS)
// =============================================================================
// ---------------------------------------------------------------------------
// atualizarStatusBT — Atualiza o label de status na tela BLUETOOTH_MENU
// Chamada no loop() a cada 500ms. Se a tela não estiver ativa (btMenuStatusLabel
// == NULL), retorna imediatamente sem fazer nada.
// ---------------------------------------------------------------------------
void atualizarStatusBT() {
    if (btMenuStatusLabel == NULL) return;
    if (btConectado) {
        lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_OK "  CONECTADO");
        lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_GREEN), 0);
    } else if (btConectando || btReconectando) {
        lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_REFRESH "  Conectando...");
        lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_ORANGE), 0);
    } else if (btScanAtivo) {
        lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_REFRESH "  Buscando dispositivos...");
        lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_YELLOW), 0);
    } else {
        lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_CLOSE "  DESCONECTADO");
        lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_RED), 0);
    }
}

// Função central que monta/remonta toda a interface gráfica.
// Chamada sempre que a página muda ou dados precisam ser atualizados.
//
// FLUXO:
//   1. lv_obj_clean(scr) - Remove todos os objetos anteriores
//   2. Verifica se é tela de calibração (caso especial, retorna após montar)
//   3. Cria container principal (320x240px, sem borda, sem scroll)
//   4. Se paginaAtual == PRINCIPAL: cria labels de telemetria
//   5. Se !emSubMenu: cria botão de entrada no submenu
//   6. Se emSubMenu: monta a tela específica do submenu atual
//   7. Adiciona botões de navegação (VOLTAR/HOME ou </>)
//
// ANÁLISE DE MEMORY LEAK:
// lv_obj_clean(scr) destrói TODOS os objetos filhos da tela.
// O LVGL gerencia memória internamente (pool de objetos).
// Não há malloc/free manual — sem risco de leak desde que:
//   - Não se guarde ponteiros para objetos após lv_obj_clean()
//   - Ponteiros globais (splashBar, label_baterias, etc) sejam
//     sempre recriados ou setados NULL após clean
//
// ATENÇÃO: Esta função é chamada com frequência (a cada mudança de tela).
// O custo de lv_obj_clean + recriação é aceitável para telas estáticas.
// Para animações (splash, teste automático), usar ponteiros para atualizar.
// ---------------------------------------------------------------------------
void montarInterface() {
    lv_obj_t * scr = lv_scr_act();
    lv_obj_clean(scr);

    // Null out hodômetro labels (destruídos por lv_obj_clean acima)
    hodo_lbl_velocidade = NULL;
    hodo_lbl_autonomia = NULL;
    hodo_lbl_energia = NULL;
    hodo_lbl_km = NULL;
    hodo_lbl_motor = NULL;
    btMenuStatusLabel = NULL;  // Destruído por lv_obj_clean; reseta para evitar acesso a objeto inválido

    // --- Tela especial de Calibragem do Touch (mantida) ---
    if (paginaAtual == CALIBRAGEM && emSubMenu) {
        etapa = 0; scaleX = 1.0; scaleY = 1.0; offsetX = 0; offsetY = 0;
        label_info = lv_label_create(scr); lv_obj_center(label_info);
        lv_label_set_text(label_info, "Toque no PONTO VERMELHO\nCanto Superior Esquerdo");

        calib_marker = lv_obj_create(scr);
        lv_obj_set_size(calib_marker, 20, 20);
        lv_obj_set_style_bg_color(calib_marker, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_radius(calib_marker, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(calib_marker, 0, 0);
        lv_obj_align(calib_marker, LV_ALIGN_TOP_LEFT, 0, 0);

        lv_obj_add_event_cb(scr, [](lv_event_t* e){
            lv_indev_t * indev = lv_indev_get_act();
            lv_point_t point; lv_indev_get_point(indev, &point);
            pontos[etapa].x = point.x; pontos[etapa].y = point.y;
            etapa++;
            if (etapa == 1) { lv_label_set_text(label_info, "Canto Superior Direito"); lv_obj_align(calib_marker, LV_ALIGN_TOP_RIGHT, 0, 0); }
            else if (etapa == 2) { lv_label_set_text(label_info, "Canto Inferior Esquerdo"); lv_obj_align(calib_marker, LV_ALIGN_BOTTOM_LEFT, 0, 0); }
            else if (etapa == 3) { lv_label_set_text(label_info, "Canto Inferior Direito"); lv_obj_align(calib_marker, LV_ALIGN_BOTTOM_RIGHT, 0, 0); }
            else if (etapa == 4) { lv_label_set_text(label_info, "Centro da Tela"); lv_obj_align(calib_marker, LV_ALIGN_CENTER, 0, 0); }
            else if (etapa == 5) {
                float dX = (float)(pontos[1].x - pontos[0].x); float dY = (float)(pontos[2].y - pontos[0].y);
                if(dX != 0) scaleX = 320.0 / dX; if(dY != 0) scaleY = 240.0 / dY;
                offsetX = 160 - (pontos[4].x * scaleX); offsetY = 120 - (pontos[4].y * scaleY);
                salvarConfiguracoes(); ESP.restart();
            }
        }, LV_EVENT_PRESSED, NULL);
        return;
    }

    // --- Container principal ---
    lv_obj_t * cont = lv_obj_create(scr);
    lv_obj_set_size(cont, 320, 240);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    // --- SISTEMA DE TEMAS: Aplica fundo e cor de texto padrão ao container ---
    lv_color_t corFundo = temaEscuro ? lv_color_hex(0x000000) : lv_color_hex(0xFFFFFF);
    lv_color_t corTexto = temaEscuro ? lv_color_white() : lv_color_black();
    lv_obj_set_style_bg_color(cont, corFundo, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(cont, corTexto, 0);

    // Ajuste de preenchimento
    if (paginaAtual == MENU_PRINCIPAL_CONS || paginaAtual == MENU_MOTORES_CONS || paginaAtual == SUBMENU_DETALHES_CONS) {
        lv_obj_set_style_pad_all(cont, 5, 0);
    }

    // ======================== MENU HODÔMETRO (NOVA HOME) ========================
    if (paginaAtual == MENU_HODOMETRO) {
        criar_menu_hodometro(cont);
    }
    // ======================== PAINEL PRINCIPAL (ANTIGO) ========================
    else if (paginaAtual == PRINCIPAL) {
        // Fix 2: Labels com largura fixa + LONG_CLIP para evitar ghosting/sobreposição
        //   Sem largura fixa, ao atualizar texto, LVGL pode não limpar pixels antigos
        //   se o novo texto for menor que o anterior. LONG_CLIP garante recorte.
        // Tarefa 2: Margem esquerda aumentada (8px) para evitar corte de valores
        // como 50.0 A ou 99.9 KM/H na borda da tela (largura reduzida para 296px)
        label_baterias = lv_label_create(cont); lv_label_set_recolor(label_baterias, true); lv_obj_set_style_text_font(label_baterias, FONTE_MENU_PRINCIPAL, 0);
        lv_obj_set_width(label_baterias, 296); lv_label_set_long_mode(label_baterias, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_pad_left(label_baterias, 8, 0);
        lv_obj_align(label_baterias, LV_ALIGN_TOP_MID, 0, 5);
        label_dados = lv_label_create(cont); lv_label_set_recolor(label_dados, true); lv_obj_set_style_text_font(label_dados, FONTE_MENU_PRINCIPAL, 0);
        lv_obj_set_width(label_dados, 296); lv_label_set_long_mode(label_dados, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_pad_left(label_dados, 8, 0);
        lv_obj_align(label_dados, LV_ALIGN_TOP_MID, 0, 60);
        label_status = lv_label_create(cont);
        lv_label_set_recolor(label_status, true); lv_obj_set_style_text_font(label_status, FONTE_MENU_PRINCIPAL, 0);
        lv_obj_set_width(label_status, 296); lv_label_set_long_mode(label_status, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_pad_left(label_status, 8, 0);
        lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 105);
        label_km = lv_label_create(cont); lv_label_set_recolor(label_km, true); lv_obj_set_style_text_font(label_km, FONTE_MENU_PRINCIPAL, 0);
        lv_obj_set_width(label_km, 296); lv_label_set_long_mode(label_km, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_pad_left(label_km, 8, 0);
        lv_obj_align(label_km, LV_ALIGN_TOP_MID, 0, 150);
        label_rodape = lv_label_create(cont); lv_label_set_recolor(label_rodape, true); lv_obj_set_style_text_font(label_rodape, FONTE_MENU_PRINCIPAL, 0);
        lv_obj_set_width(label_rodape, 296); lv_label_set_long_mode(label_rodape, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_pad_left(label_rodape, 8, 0);
        lv_obj_align(label_rodape, LV_ALIGN_TOP_MID, 0, 195);
        atualizarDadosPrincipais();
        
        // --- BOTÃO DESLIGAR MANUAL (aparece quando keep-alive está ativo) ---
        // Quando o utilizador escolheu SIM no diálogo de 20s (chave desligada),
        // o sistema fica ativo mas aparece este botão para desligar manualmente.
        // O botão é vermelho com ícone de power, posicionado no rodapé com margem.
        if(sistemaKeepAlive) {
            lv_obj_t * btnDesligar = lv_btn_create(cont);
            lv_obj_set_size(btnDesligar, 260, 36);
            lv_obj_align(btnDesligar, LV_ALIGN_BOTTOM_MID, 0, -8);
            lv_obj_set_style_bg_color(btnDesligar, lv_color_hex(0x880000), 0);
            lv_obj_set_style_border_color(btnDesligar, lv_color_hex(0xFF4444), 0);
            lv_obj_set_style_border_width(btnDesligar, 2, 0);
            lv_obj_add_event_cb(btnDesligar, [](lv_event_t*e){
                desligarManual();
            }, LV_EVENT_CLICKED, NULL);
            lv_obj_t * lblDesl = lv_label_create(btnDesligar);
            lv_label_set_text(lblDesl, LV_SYMBOL_POWER " DESLIGAR MANUAL");
            lv_obj_center(lblDesl);
        }
    }
    // ======================== BOTÃO DE ENTRADA NO SUBMENU ========================
    else if (!emSubMenu) {
        lv_obj_t * btn_entrar = lv_btn_create(cont);
        lv_obj_set_size(btn_entrar, 240, 60); lv_obj_center(btn_entrar);
        lv_obj_add_event_cb(btn_entrar, [](lv_event_t*e){
            if(paginaAtual == BATERIA) paginaAtual = CONFIG_BAT;
            else if(paginaAtual == VOLTAGEM) paginaAtual = SEL_BATERIA_ATIVA;
            else if(paginaAtual == CONSUMO) paginaAtual = MENU_PRINCIPAL_CONS;
            else if(paginaAtual == SOLAR) paginaAtual = MENU_SOLAR;
            else if(paginaAtual == ACIONAR) paginaAtual = MANUTENCAO_MENU;
            // BUG CORRIGIDO: Navegação direta para Manutenção/Condução
            // Quando navPaginas[] aponta diretamente para estas páginas,
            // não precisa mudar paginaAtual (já está correto)
            else if(paginaAtual == MANUTENCAO_MENU) { /* ja correto */ }
            else if(paginaAtual == CONDUCAO_MENU) { /* ja correto */ }
            else if(paginaAtual == RADAR_PROXIMIDADE) { /* ja correto */ }
            else if(paginaAtual == PRECO_POR_KM) { /* ja correto */ }
            emSubMenu = true; montarInterface();
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_t * lbl = lv_label_create(btn_entrar); 
        lv_label_set_text(lbl, menuLabels[obterNavIndex(paginaAtual)]); 
        lv_obj_center(lbl);
    }
    // ======================== TELAS DE SUBMENU ========================
    else {
        // ====================================================================================
        // MÓDULO A: MANUTENÇÃO / TESTES
        // ====================================================================================
        if (paginaAtual == MANUTENCAO_MENU) {
            // Container de rolagem vertical para evitar que Agenda de Revisão
            // fique coberta pelos botões Voltar/Home fixos no rodapé.
            // Ocupa 80% da tela (deixa espaço para rodapé).
            lv_obj_t * cont_menu = lv_obj_create(cont);
            lv_obj_set_size(cont_menu, LV_PCT(100), LV_PCT(80));
            lv_obj_align(cont_menu, LV_ALIGN_TOP_MID, 0, 0);
            lv_obj_set_flex_flow(cont_menu, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(cont_menu, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_set_scroll_dir(cont_menu, LV_DIR_VER);
            lv_obj_set_scroll_snap_y(cont_menu, LV_SCROLL_SNAP_NONE);
            lv_obj_add_flag(cont_menu, LV_OBJ_FLAG_SCROLL_MOMENTUM);
            lv_obj_set_style_border_width(cont_menu, 0, 0);
            lv_obj_set_style_bg_opa(cont_menu, 0, 0);
            lv_obj_set_style_pad_gap(cont_menu, 8, 0);

            // Opção 1: Verificação de integridade manual
            lv_obj_t * b0 = lv_btn_create(cont_menu);
            lv_obj_set_size(b0, 260, 45);
            lv_label_set_text(lv_label_create(b0), LV_SYMBOL_REFRESH " VERIFICAR INTEGRIDADE");
            lv_obj_add_event_cb(b0, [](lv_event_t* e){
                SerialBT.print("REBOOT_CLEAN\n");
                lv_obj_t * scr = lv_scr_act();
                lv_obj_t * msgbox = lv_msgbox_create(scr);
                lv_obj_set_size(msgbox, 260, 100);
                lv_obj_center(msgbox);
                lv_obj_t * lbl_msg = lv_label_create(msgbox);
                lv_label_set_text(lbl_msg, "Comando enviado ao Mega.\nAguardando READY...");
                lv_obj_set_style_text_align(lbl_msg, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_center(lbl_msg);
            }, LV_EVENT_CLICKED, NULL);

            lv_obj_t * b1 = lv_btn_create(cont_menu);
            lv_obj_set_size(b1, 260, 45);
            lv_label_set_text(lv_label_create(b1), LV_SYMBOL_PLAY " TESTE AUTOMATICO");
            lv_obj_add_event_cb(b1, [](lv_event_t* e){ 
                paginaAtual = TESTE_AUTOMATICO; montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);

            lv_obj_t * b2 = lv_btn_create(cont_menu);
            lv_obj_set_size(b2, 260, 45);
            lv_label_set_text(lv_label_create(b2), LV_SYMBOL_EDIT " TESTE MANUAL");
            lv_obj_add_event_cb(b2, [](lv_event_t* e){ 
                paginaAtual = TESTE_MANUAL; montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);

            lv_obj_t * b3 = lv_btn_create(cont_menu);
            lv_obj_set_size(b3, 260, 45);
            lv_label_set_text(lv_label_create(b3), LV_SYMBOL_WARNING " TESTE ALERTAS/ICONES");
            lv_obj_add_event_cb(b3, [](lv_event_t* e){ 
                paginaAtual = TESTE_ALERTAS; montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);

            lv_obj_t * b4 = lv_btn_create(cont_menu);
            lv_obj_set_size(b4, 260, 45);
            lv_label_set_text(lv_label_create(b4), LV_SYMBOL_BELL " AGENDA DE REVISAO");
            lv_obj_add_event_cb(b4, [](lv_event_t* e){ 
                paginaAtual = AGENDA_MANUTENCAO; montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);

            // Botão PAREAMENTO BT (v4.1 Master): acesso direto ao menu de conexão Bluetooth
            lv_obj_t * b5 = lv_btn_create(cont_menu);
            lv_obj_set_size(b5, 260, 45);
            // Cor verde se conectado, laranja se conectando/reconectando, vermelho se desconectado
            lv_color_t corBt = btConectado ? lv_palette_main(LV_PALETTE_GREEN) :
                               ((btConectando || btReconectando) ? lv_palette_main(LV_PALETTE_ORANGE) :
                                lv_palette_main(LV_PALETTE_RED));
            lv_obj_set_style_bg_color(b5, corBt, 0);
            lv_obj_t * lbl_bt = lv_label_create(b5);
            lv_label_set_text_fmt(lbl_bt, LV_SYMBOL_BLUETOOTH " BLUETOOTH: %s",
                                  btConectado ? "CONECTADO" :
                                  ((btConectando || btReconectando) ? "CONECTANDO..." : "DESCONECTADO"));
            lv_obj_add_event_cb(b5, [](lv_event_t* e){ 
                paginaAtual = BLUETOOTH_MENU; montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);
            
            // Botão DESLIGAR TELA removido da interface.
            // O controle de backlight agora é feito via botão físico no Mega,
            // que envia TELA,0 (desligar) e TELA,1 (ligar) via Serial.
        }
        else if (paginaAtual == TESTE_AUTOMATICO) {
            // Mensagem de aviso
            if(!testeAutomaticoAtivo) {
                lv_obj_t * titulo = lv_label_create(cont);
                lv_label_set_text(titulo, "TESTE AUTOMATICO");
                lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 10);
                
                lv_obj_t * aviso = lv_label_create(cont);
                lv_label_set_text(aviso, "AVISO:\nCertifique-se que o veiculo\nesta em cavaletes.");
                lv_obj_set_style_text_color(aviso, lv_color_hex(0xFF0000), 0);
                lv_obj_align(aviso, LV_ALIGN_CENTER, 0, -20);
                
                lv_obj_t * btn_iniciar = lv_btn_create(cont);
                lv_obj_set_size(btn_iniciar, 200, 60);
                lv_obj_align(btn_iniciar, LV_ALIGN_BOTTOM_MID, 0, -40);
                lv_obj_set_style_bg_color(btn_iniciar, lv_palette_main(LV_PALETTE_GREEN), 0);
                
                lv_obj_add_event_cb(btn_iniciar, [](lv_event_t*e){
                    testeAutomaticoAtivo = true;
                    testeAutomaticoIndice = 0;
                    testeAutomaticoTimer = millis();
                    montarInterface();
                }, LV_EVENT_CLICKED, NULL);
                
                lv_label_set_text(lv_label_create(btn_iniciar), "INICIAR TESTE");
            } else {
                // Teste em andamento
                lv_obj_t * titulo = lv_label_create(cont);
                lv_label_set_text(titulo, "TESTE EM ANDAMENTO");
                lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 10);
                
                lv_obj_t * info = lv_label_create(cont);
                lv_label_set_text_fmt(info, "Ativando %s", nomeReles[testeAutomaticoIndice-1]);
                lv_obj_set_style_text_font(info, FONTE_GRANDE, 0);
                lv_obj_align(info, LV_ALIGN_CENTER, 0, -20);
                
                // Barra de progresso
                lv_obj_t * bar = lv_bar_create(cont);
                lv_obj_set_size(bar, 280, 30);
                lv_obj_align(bar, LV_ALIGN_CENTER, 0, 30);
                lv_bar_set_value(bar, (testeAutomaticoIndice * 100) / 15, LV_ANIM_ON);
                
                lv_obj_t * progresso = lv_label_create(cont);
                lv_label_set_text_fmt(progresso, "%d / 15", testeAutomaticoIndice);
                lv_obj_align(progresso, LV_ALIGN_CENTER, 0, 65);
            }
            
            // BOTÃO RESET GERAL (sempre visível)
            lv_obj_t * btn_reset = lv_btn_create(scr);
            lv_obj_set_size(btn_reset, 140, 50);
            lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_MID, 0, -10);
            lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0xFF8800), 0);
            
            lv_obj_add_event_cb(btn_reset, [](lv_event_t*e){
                enviarResetGeral();
                paginaAtual = MANUTENCAO_MENU;
                emSubMenu = true;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            
            lv_obj_t * lbl_reset = lv_label_create(btn_reset);
            lv_label_set_text(lbl_reset, "RESET GERAL");
            lv_obj_center(lbl_reset);
        }
        else if (paginaAtual == TESTE_MANUAL) {
            // BUG CORRIGIDO: Reset Geral sobrepunha botões dos relés.
            // Agora: Voltar (esq), Reset Geral (dir, no lugar do Home).
            // Lista ocupa mais espaço vertical (160px em vez de 140px).
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, "TESTE MANUAL - RELES");
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 5);
            
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 290, 160);
            lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 25);
            
            // Cria switch para cada relé (R1-R15)
            for(int i=0; i<15; i++) {
                lv_obj_t * item = lv_list_add_btn(list, NULL, nomeReles[i]);
                
                lv_obj_t * sw = lv_switch_create(item);
                lv_obj_align(sw, LV_ALIGN_RIGHT_MID, -10, 0);
                
                if(estadoRelesTeste[i]) {
                    lv_obj_add_state(sw, LV_STATE_CHECKED);
                }
                
                lv_obj_add_event_cb(sw, [](lv_event_t*e){
                    lv_obj_t * sw_obj = (lv_obj_t*)lv_event_get_target(e);
                    int idx = (int)(intptr_t)lv_event_get_user_data(e);
                    bool estado = lv_obj_has_state(sw_obj, LV_STATE_CHECKED);
                    
                    estadoRelesTeste[idx] = estado;
                    enviarComandoRele(idx+1, estado ? 1 : 0);
                }, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)i);
            }
            // Nota: O botão Reset Geral é adicionado no rodapé (lado direito)
            // no lugar do botão Home. O botão Voltar permanece na esquerda.
        }

        // ====================================================================================
        // TESTE ALERTAS / ÍCONES
        // ====================================================================================
        // Permite testar manualmente cada tipo de alerta visual (overlay) e
        // enviar comandos de simulação ao Mega (TESTE_ROUBO, TESTE_CHUVA).
        // Ao sair (botão VOLTAR), todos os alertas são limpos via RESET_ALARM.
        else if (paginaAtual == TESTE_ALERTAS) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, "TESTE ALERTAS / ICONES");
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 5);

            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 290, 165);
            lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 25);

            // Botão: Simular Roubo
            lv_obj_t * bR = lv_list_add_btn(list, LV_SYMBOL_WARNING, "SIMULAR ROUBO");
            lv_obj_add_event_cb(bR, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaEmergencia = true;
                alertaCritico = true;
                SerialBT.print("TESTE_ROUBO\n");
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Chuva
            lv_obj_t * bC = lv_list_add_btn(list, LV_SYMBOL_WARNING, "SIMULAR CHUVA");
            lv_obj_add_event_cb(bC, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaChuva = true;
                SerialBT.print("TESTE_CHUVA\n");
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Temperatura
            lv_obj_t * bT = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "SIMULAR TEMPERATURA");
            lv_obj_add_event_cb(bT, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaTemperatura = true;
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Corrente Excessiva
            lv_obj_t * bI = lv_list_add_btn(list, LV_SYMBOL_CHARGE, "SIMULAR CORRENTE EXC.");
            lv_obj_add_event_cb(bI, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaCorrenteExc = true;
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Subtensão
            lv_obj_t * bS = lv_list_add_btn(list, LV_SYMBOL_BATTERY_EMPTY, "SIMULAR SUBTENSAO");
            lv_obj_add_event_cb(bS, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaSubtensao = true;
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Falha Freio
            lv_obj_t * bF = lv_list_add_btn(list, LV_SYMBOL_STOP, "SIMULAR FALHA FREIO");
            lv_obj_add_event_cb(bF, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaFalhaFreio = true;
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Seta Esquerda
            lv_obj_t * bSE = lv_list_add_btn(list, LV_SYMBOL_LEFT, "SIMULAR SETA ESQ.");
            lv_obj_add_event_cb(bSE, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaSetaEsq = true; alertaSetaDir = false;
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Simular Seta Direita
            lv_obj_t * bSD = lv_list_add_btn(list, LV_SYMBOL_RIGHT, "SIMULAR SETA DIR.");
            lv_obj_add_event_cb(bSD, [](lv_event_t*e){
                modoTesteAtivo = true;
                alertaSetaEsq = false; alertaSetaDir = true;
            }, LV_EVENT_CLICKED, NULL);

            // Botão: Limpar Todos os Alertas
            lv_obj_t * bLimpar = lv_btn_create(cont);
            lv_obj_set_size(bLimpar, 150, 30);
            lv_obj_align(bLimpar, LV_ALIGN_BOTTOM_MID, 0, -5);
            lv_obj_set_style_bg_color(bLimpar, lv_color_hex(0xFF4444), 0);
            lv_label_set_text(lv_label_create(bLimpar), "LIMPAR TODOS");
            lv_obj_add_event_cb(bLimpar, [](lv_event_t*e){
                alertaEmergencia = false; alertaTemperatura = false;
                alertaChuva = false; alertaCorrenteExc = false;
                alertaFalhaFreio = false; alertaSubtensao = false;
                alertaSetaEsq = false; alertaSetaDir = false;
                alertaCruiseOn = false; alertaBatCritica = false;
                alertaCritico = false;
                modoTesteAtivo = false;
                SerialBT.print("RESET_ALARM\n");
            }, LV_EVENT_CLICKED, NULL);
        }

        // ====================================================================================
        // AGENDA DE MANUTENÇÃO POR KM
        // ====================================================================================
        // Permite definir o km alvo para a próxima manutenção.
        // Quando kmTotal >= manutKmAlvo, aparece alerta na tela home.
        // manutKmAlvo = 0 desativa a agenda.
        else if (paginaAtual == AGENDA_MANUTENCAO) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, "AGENDA DE MANUTENCAO");
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 5);

            // Informação do km atual
            lv_obj_t * lbl_atual = lv_label_create(cont);
            lv_label_set_text_fmt(lbl_atual, "Km atual: %.0f km", kmTotal);
            lv_obj_set_style_text_font(lbl_atual, &lv_font_montserrat_14, 0);
            lv_obj_align(lbl_atual, LV_ALIGN_TOP_MID, 0, 30);

            // Informação do km alvo
            lv_obj_t * lbl_alvo = lv_label_create(cont);
            if (manutKmAlvo > 0) {
                lv_label_set_text_fmt(lbl_alvo, "Proximo servico: %.0f km", manutKmAlvo);
            } else {
                lv_label_set_text(lbl_alvo, "Agenda: DESATIVADA");
            }
            lv_obj_set_style_text_font(lbl_alvo, &lv_font_montserrat_14, 0);
            lv_obj_align(lbl_alvo, LV_ALIGN_TOP_MID, 0, 50);

            // Roller para definir km alvo (0 = desativar, 500 a 10000 de 500 em 500)
            lv_obj_t * lbl_def = lv_label_create(cont);
            lv_label_set_text(lbl_def, "Definir alvo:");
            lv_obj_set_style_text_font(lbl_def, &lv_font_montserrat_14, 0);
            lv_obj_align(lbl_def, LV_ALIGN_CENTER, -60, -20);

            lv_obj_t * roller_manut = lv_roller_create(cont);
            char optsManut[512] = "0 (Desativar)";
            for (int km = 500; km <= 10000; km += 500) {
                char tmp[16]; snprintf(tmp, sizeof(tmp), "\n%d km", km);
                strncat(optsManut, tmp, sizeof(optsManut) - strlen(optsManut) - 1);
            }
            lv_roller_set_options(roller_manut, optsManut, LV_ROLLER_MODE_NORMAL);
            lv_roller_set_visible_row_count(roller_manut, 3);
            lv_obj_set_size(roller_manut, 130, 90);
            lv_obj_align(roller_manut, LV_ALIGN_CENTER, 50, -10);
            // Seleciona o índice correspondente ao valor atual
            int idxAtual = 0;
            if (manutKmAlvo > 0) idxAtual = (int)(manutKmAlvo / 500.0f);
            if (idxAtual > 20) idxAtual = 20;
            lv_roller_set_selected(roller_manut, idxAtual, LV_ANIM_OFF);

            lv_obj_add_event_cb(roller_manut, [](lv_event_t*e){
                lv_obj_t * r = (lv_obj_t*)lv_event_get_target(e);
                int sel = lv_roller_get_selected(r);
                manutKmAlvo = (sel == 0) ? 0 : (float)(sel * 500);
                manutAlertaMostrado = false;  // Permite novo alerta no novo alvo
                // Persiste na NVS (namespace "manut" = mesmo usado em carregarAgendaManutencao)
                Preferences p2; p2.begin("manut", false);
                p2.putFloat("kmAlvo", manutKmAlvo); p2.end();
            }, LV_EVENT_VALUE_CHANGED, NULL);

            // Botão: RESETAR AGENDA (manutenção feita)
            lv_obj_t * btn_reset = lv_btn_create(cont);
            lv_obj_set_size(btn_reset, 200, 35);
            lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_MID, 0, -5);
            lv_obj_set_style_bg_color(btn_reset, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_label_set_text(lv_label_create(btn_reset), "MANUTENCAO FEITA");
            lv_obj_add_event_cb(btn_reset, [](lv_event_t*e){
                manutKmAlvo = 0;
                manutAlertaMostrado = false;
                Preferences p2; p2.begin("manut", false);
                p2.putFloat("kmAlvo", 0.0f); p2.end();
                paginaAtual = MANUTENCAO_MENU;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        // ====================================================================================
        // PAREAMENTO BLUETOOTH (v4.1 HC-06 Master) — Scan e seleção de dispositivo
        // ====================================================================================
        // Exibe a lista de dispositivos BT encontrados durante o scan.
        // O utilizador toca num dispositivo para conectar e salvar o MAC.
        // Botão "Buscar" inicia o inquiry BT (Core 0 executa; resultados chegam via GAP callback).
        else if (paginaAtual == PAREAMENTO_BT) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, LV_SYMBOL_BLUETOOTH " PAREAMENTO BT (MASTER)");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_14, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 3);

            // Status da conexão
            lv_obj_t * lbl_status = lv_label_create(cont);
            if (btConectado) {
                lv_label_set_text(lbl_status, LV_SYMBOL_OK " CONECTADO ao HC-06");
                lv_obj_set_style_text_color(lbl_status, lv_palette_main(LV_PALETTE_GREEN), 0);
            } else if (btConectando || btReconectando) {
                lv_label_set_text(lbl_status, LV_SYMBOL_REFRESH " Conectando...");
                lv_obj_set_style_text_color(lbl_status, lv_palette_main(LV_PALETTE_ORANGE), 0);
            } else if (btScanAtivo) {
                lv_label_set_text(lbl_status, LV_SYMBOL_REFRESH " Buscando dispositivos...");
                lv_obj_set_style_text_color(lbl_status, lv_palette_main(LV_PALETTE_YELLOW), 0);
            } else {
                lv_label_set_text(lbl_status, LV_SYMBOL_CLOSE " DESCONECTADO");
                lv_obj_set_style_text_color(lbl_status, lv_palette_main(LV_PALETTE_RED), 0);
            }
            lv_obj_set_style_text_font(lbl_status, &lv_font_montserrat_12, 0);
            lv_obj_align(lbl_status, LV_ALIGN_TOP_MID, 0, 24);

            // MAC salvo na NVS
            lv_obj_t * lbl_mac = lv_label_create(cont);
            if (btTargetMacSaved) {
                char macStr[20];
                snprintf(macStr, sizeof(macStr), "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                    btTargetMac[0], btTargetMac[1], btTargetMac[2],
                    btTargetMac[3], btTargetMac[4], btTargetMac[5]);
                lv_label_set_text(lbl_mac, macStr);
                lv_obj_set_style_text_color(lbl_mac, lv_palette_main(LV_PALETTE_GREEN), 0);
            } else {
                lv_label_set_text(lbl_mac, "MAC: nao associado");
                lv_obj_set_style_text_color(lbl_mac, lv_color_hex(0xAAAAAA), 0);
            }
            lv_obj_set_style_text_font(lbl_mac, &lv_font_montserrat_12, 0);
            lv_obj_align(lbl_mac, LV_ALIGN_TOP_MID, 0, 42);

            // ── Lista de dispositivos encontrados ─────────────────────────────
            // Cada linha: toque conecta ao dispositivo e salva o MAC na NVS.
            // Lista scrollável com até MAX_BT_DEVICES entradas.
            lv_obj_t * list_dev = lv_list_create(cont);
            lv_obj_set_size(list_dev, 295, 100);
            lv_obj_align(list_dev, LV_ALIGN_TOP_MID, 0, 62);
            lv_obj_set_scroll_snap_y(list_dev, LV_SCROLL_SNAP_NONE);

            int devCount = 0;
            if (xSemaphoreTake(btDeviceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                devCount = btDeviceCount;
                xSemaphoreGive(btDeviceMutex);
            }

            if (devCount == 0) {
                lv_obj_t * lbl_none = lv_list_add_text(list_dev,
                    btScanAtivo ? "Buscando..." : "Nenhum dispositivo.\nPressione BUSCAR.");
            } else {
                for (int di = 0; di < devCount && di < MAX_BT_DEVICES; di++) {
                    // Cria cópia local para evitar race condition com o callback BT
                    char devName[33]; uint8_t devMac[6];
                    if (xSemaphoreTake(btDeviceMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                        strncpy(devName, btDeviceList[di].name, 32); devName[32] = '\0';
                        memcpy(devMac, btDeviceList[di].mac, 6);
                        xSemaphoreGive(btDeviceMutex);
                    } else { continue; }

                    char lblText[64];
                    if (devName[0] != '\0') {
                        snprintf(lblText, sizeof(lblText), "%s", devName);
                    } else {
                        snprintf(lblText, sizeof(lblText), "%02X:%02X:%02X:%02X:%02X:%02X",
                            devMac[0], devMac[1], devMac[2], devMac[3], devMac[4], devMac[5]);
                    }

                    // Destaque se é o dispositivo alvo ou o que já está associado
                    bool isTarget = (btTargetMacSaved && memcmp(devMac, btTargetMac, 6) == 0);
                    lv_obj_t * btnDev = lv_list_add_btn(list_dev,
                        isTarget ? LV_SYMBOL_OK : LV_SYMBOL_BLUETOOTH, lblText);
                    if (isTarget) {
                        lv_obj_set_style_bg_color(btnDev, lv_palette_darken(LV_PALETTE_GREEN, 3), 0);
                    }

                    // Ao tocar: para scan, salva MAC e solicita conexão ao Core 0
                    lv_obj_add_event_cb(btnDev, [](lv_event_t* ev) {
                        // Recupera o índice do dispositivo pelo user_data
                        int idx = (int)(intptr_t)lv_event_get_user_data(ev);
                        if (idx < 0 || idx >= MAX_BT_DEVICES) return;
                        uint8_t mac[6];
                        bool ok = false;
                        if (xSemaphoreTake(btDeviceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            if (btDeviceList[idx].valid) {
                                memcpy(mac, btDeviceList[idx].mac, 6);
                                ok = true;
                            }
                            xSemaphoreGive(btDeviceMutex);
                        }
                        if (!ok) return;
                        // Para o scan se estiver ativo
                        if (btScanAtivo) pendingBtScanStop = true;
                        // Preenche MAC pendente e solicita conexão ao Core 0
                        memcpy(btPendingMac, mac, 6);
                        pendingBtConnect = true;
                        btConectando = true;
                        btConectado  = false;
                        Serial.printf("[BT UI] Dispositivo selecionado idx=%d\n", idx);
                        montarInterface();
                    }, LV_EVENT_CLICKED, (void*)(intptr_t)di);
                }
            }

            // ── Botões de ação ────────────────────────────────────────────────
            // Botão BUSCAR: inicia inquiry BT (10s), preenche lista
            lv_obj_t * btn_scan = lv_btn_create(cont);
            lv_obj_set_size(btn_scan, 140, 36);
            lv_obj_align(btn_scan, LV_ALIGN_BOTTOM_LEFT, 5, -5);
            lv_obj_set_style_bg_color(btn_scan,
                btScanAtivo ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_BLUE), 0);
            lv_obj_t * lbl_scan = lv_label_create(btn_scan);
            lv_label_set_text(lbl_scan, btScanAtivo ? LV_SYMBOL_CLOSE " PARAR" : LV_SYMBOL_REFRESH " BUSCAR");
            lv_obj_set_style_text_font(lbl_scan, &lv_font_montserrat_12, 0);
            lv_obj_center(lbl_scan);
            lv_obj_add_event_cb(btn_scan, [](lv_event_t* e) {
                if (btScanAtivo) {
                    pendingBtScanStop = true;
                } else {
                    pendingBtScan = true;
                }
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);

            // Botão RECONECTAR: usa o MAC salvo
            lv_obj_t * btn_rec = lv_btn_create(cont);
            lv_obj_set_size(btn_rec, 140, 36);
            lv_obj_align(btn_rec, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
            lv_obj_set_style_bg_color(btn_rec,
                btTargetMacSaved ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
            lv_obj_t * lbl_rec = lv_label_create(btn_rec);
            lv_label_set_text(lbl_rec, LV_SYMBOL_PLAY " RECONECTAR");
            lv_obj_set_style_text_font(lbl_rec, &lv_font_montserrat_12, 0);
            lv_obj_center(lbl_rec);
            if (btTargetMacSaved) {
                lv_obj_add_event_cb(btn_rec, [](lv_event_t* e) {
                    btConectado  = false;
                    btConectando = true;
                    pendingReconexaoBT = true;
                    Serial.println("[BT UI] Reconectar ao MAC salvo (PAREAMENTO_BT)");
                    montarInterface();
                }, LV_EVENT_CLICKED, NULL);
            } else {
                lv_obj_add_state(btn_rec, LV_STATE_DISABLED);  // Sem MAC salvo: botão inativo
            }
        }
        // ====================================================================================
        // BLUETOOTH_MENU — Status rápido da ligação e MAC associado
        // ====================================================================================
        // Mostra: status (conectado/conectando/desconectado), MAC do HC-06 associado,
        // botão de reconexão forçada e atalho para PAREAMENTO_BT (scan de dispositivos).
        else if (paginaAtual == BLUETOOTH_MENU) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, LV_SYMBOL_BLUETOOTH " Bluetooth (MASTER)");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_14, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 5);

            // Label de status — guardado em btMenuStatusLabel para atualização periódica
            btMenuStatusLabel = lv_label_create(cont);
            if (btConectado) {
                lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_OK "  CONECTADO");
                lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_GREEN), 0);
            } else if (btConectando || btReconectando) {
                lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_REFRESH "  Conectando...");
                lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_ORANGE), 0);
            } else {
                lv_label_set_text(btMenuStatusLabel, LV_SYMBOL_CLOSE "  DESCONECTADO");
                lv_obj_set_style_text_color(btMenuStatusLabel, lv_palette_main(LV_PALETTE_RED), 0);
            }
            lv_obj_set_style_text_font(btMenuStatusLabel, &lv_font_montserrat_14, 0);
            lv_obj_align(btMenuStatusLabel, LV_ALIGN_TOP_MID, 0, 28);

            // Nome do dispositivo alvo (HC-06)
            lv_obj_t * lbl_dev = lv_label_create(cont);
            lv_label_set_text_fmt(lbl_dev, "Alvo: %s", BT_TARGET_NAME);
            lv_obj_set_style_text_font(lbl_dev, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(lbl_dev, lv_color_hex(0xAAAAAA), 0);
            lv_obj_align(lbl_dev, LV_ALIGN_TOP_MID, 0, 50);

            // MAC salvo na NVS
            lv_obj_t * lbl_mac = lv_label_create(cont);
            if (btTargetMacSaved) {
                char macStr[20];
                snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                    btTargetMac[0], btTargetMac[1], btTargetMac[2],
                    btTargetMac[3], btTargetMac[4], btTargetMac[5]);
                lv_label_set_text_fmt(lbl_mac, "MAC: %s", macStr);
                lv_obj_set_style_text_color(lbl_mac, lv_palette_main(LV_PALETTE_GREEN), 0);
            } else {
                lv_label_set_text(lbl_mac, "MAC: nao associado");
                lv_obj_set_style_text_color(lbl_mac, lv_palette_main(LV_PALETTE_RED), 0);
            }
            lv_obj_set_style_text_font(lbl_mac, &lv_font_montserrat_12, 0);
            lv_obj_align(lbl_mac, LV_ALIGN_TOP_MID, 0, 66);

            // Último dado recebido
            lv_obj_t * lbl_tempo = lv_label_create(cont);
            if (btUltimaRecepMs > 0) {
                unsigned long segsAtras = (millis() - btUltimaRecepMs) / 1000UL;
                lv_label_set_text_fmt(lbl_tempo, "Ultimo CSV: %lus atras", segsAtras);
            } else {
                lv_label_set_text(lbl_tempo, "Aguardando dados...");
            }
            lv_obj_set_style_text_font(lbl_tempo, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(lbl_tempo, lv_color_hex(0xAAAAAA), 0);
            lv_obj_align(lbl_tempo, LV_ALIGN_TOP_MID, 0, 82);

            // Botão: Forçar Reconexão ao MAC salvo
            lv_obj_t * btn_recon = lv_btn_create(cont);
            lv_obj_set_size(btn_recon, 210, 36);
            lv_obj_align(btn_recon, LV_ALIGN_CENTER, 0, 18);
            lv_obj_set_style_bg_color(btn_recon,
                btTargetMacSaved ? lv_palette_main(LV_PALETTE_BLUE) : lv_palette_main(LV_PALETTE_GREY), 0);
            lv_obj_t * lbl_recon = lv_label_create(btn_recon);
            lv_label_set_text(lbl_recon, LV_SYMBOL_REFRESH " Forcar Reconexao");
            lv_obj_set_style_text_font(lbl_recon, &lv_font_montserrat_12, 0);
            lv_obj_center(lbl_recon);
            if (btTargetMacSaved) {
                lv_obj_add_event_cb(btn_recon, [](lv_event_t* e) {
                    btConectado  = false;
                    btReconectando = true;
                    btReconectandoMs = millis();
                    btUltimaRecepMs  = 0;
                    pendingReconexaoBT = true;
                    Serial.println("[BT] Reconexao forcada solicitada pelo BLUETOOTH_MENU");
                    montarInterface();
                }, LV_EVENT_CLICKED, NULL);
            }

            // Botão: Ir para PAREAMENTO BT (scan de dispositivos)
            lv_obj_t * btn_par = lv_btn_create(cont);
            lv_obj_set_size(btn_par, 210, 36);
            lv_obj_align(btn_par, LV_ALIGN_CENTER, 0, 60);
            lv_obj_set_style_bg_color(btn_par, lv_palette_main(LV_PALETTE_TEAL), 0);
            lv_obj_t * lbl_par = lv_label_create(btn_par);
            lv_label_set_text(lbl_par, LV_SYMBOL_LIST " Buscar Dispositivos");
            lv_obj_set_style_text_font(lbl_par, &lv_font_montserrat_12, 0);
            lv_obj_center(lbl_par);
            lv_obj_add_event_cb(btn_par, [](lv_event_t* e) {
                paginaAtual = PAREAMENTO_BT;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        // ====================================================================================
        else if (paginaAtual == CONDUCAO_MENU) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 280, 160);
            lv_obj_center(list);

            lv_obj_t * b1 = lv_list_add_btn(list, LV_SYMBOL_GPS, "PILOTO AUTOMATICO");
            lv_obj_add_event_cb(b1, [](lv_event_t* e){ 
                paginaAtual = CRUISE_CONTROL; 
                montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);

            lv_obj_t * b2 = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "CONFIG. PAS");
            lv_obj_add_event_cb(b2, [](lv_event_t* e){ 
                paginaAtual = CONFIG_PAS; 
                montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);
            
            lv_obj_t * b3 = lv_list_add_btn(list, LV_SYMBOL_EDIT, "SENSIBILIDADE");
            lv_obj_add_event_cb(b3, [](lv_event_t* e){ 
                paginaAtual = CONFIG_SENSIBILIDADE; 
                montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);
            
            // Switch Radar Habilitado (persiste na NVS)
            lv_obj_t * contRadar = lv_obj_create(list);
            lv_obj_set_size(contRadar, 260, 40);
            lv_obj_set_style_bg_opa(contRadar, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(contRadar, 0, 0);
            lv_obj_set_style_pad_all(contRadar, 0, 0);
            lv_obj_clear_flag(contRadar, LV_OBJ_FLAG_SCROLLABLE);
            
            lv_obj_t * lblRadar = lv_label_create(contRadar);
            lv_label_set_text(lblRadar, LV_SYMBOL_EYE_OPEN " RADAR TRASEIRO");
            lv_obj_align(lblRadar, LV_ALIGN_LEFT_MID, 5, 0);
            
            lv_obj_t * swRadar = lv_switch_create(contRadar);
            lv_obj_align(swRadar, LV_ALIGN_RIGHT_MID, -5, 0);
            if(radarHabilitado) lv_obj_add_state(swRadar, LV_STATE_CHECKED);
            else lv_obj_clear_state(swRadar, LV_STATE_CHECKED);
            lv_obj_add_event_cb(swRadar, [](lv_event_t* e){
                lv_obj_t * sw = (lv_obj_t*)lv_event_get_target(e);
                radarHabilitado = lv_obj_has_state(sw, LV_STATE_CHECKED);
                salvarConfigsConducao();
            }, LV_EVENT_VALUE_CHANGED, NULL);
            
            // Botão MODO DISFARCE (velocidade fictícia)
            lv_obj_t * b4 = lv_list_add_btn(list, LV_SYMBOL_EYE_CLOSE, "MODO DISFARCE");
            lv_obj_add_event_cb(b4, [](lv_event_t* e){ 
                paginaAtual = CONFIG_TRAVA; 
                montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);
            
            // Botão LIMITADOR DE VELOCIDADE
            lv_obj_t * b5 = lv_list_add_btn(list, LV_SYMBOL_WARNING, "LIMITADOR VEL.");
            lv_obj_add_event_cb(b5, [](lv_event_t* e){ 
                paginaAtual = CONFIG_LIMITE_VEL; 
                montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);
            
            // Botão TEMAS (Claro/Escuro)
            lv_obj_t * b6 = lv_list_add_btn(list, LV_SYMBOL_IMAGE, "TEMAS");
            lv_obj_add_event_cb(b6, [](lv_event_t* e){ 
                paginaAtual = CONFIG_TEMAS; 
                montarInterface(); 
            }, LV_EVENT_CLICKED, NULL);
        }
        // ====================================================================================
        // TELA PILOTO AUTOMÁTICO (CRUISE CONTROL) - 0 a 60 km/h (lv_roller)
        // ====================================================================================
        // Seção 1: Ajustado de slider 0-100 para lv_roller 0-60 km/h
        // ! PERIGO ! O cruise control NÃO deve ser usado em descidas ou piso molhado.
        // O Arduino Mega possui kill-switch no freio que desativa o cruise se acionado.
        // ====================================================================================
        else if (paginaAtual == CRUISE_CONTROL) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, "PILOTO AUTOMATICO");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 5);
            
            // Rótulo "Velocidade Alvo"
            lv_obj_t * lbl_alvo = lv_label_create(cont);
            lv_label_set_text(lbl_alvo, "Velocidade Alvo:");
            lv_obj_align(lbl_alvo, LV_ALIGN_TOP_MID, 0, 30);
            
            // Valor atual grande em ciano
            lv_obj_t * lbl_vel = lv_label_create(cont);
            lv_obj_set_style_text_font(lbl_vel, FONTE_GRANDE, 0);
            lv_obj_set_style_text_color(lbl_vel, lv_color_hex(0x00FFFF), 0);
            lv_label_set_text_fmt(lbl_vel, "%.0f km/h", cruiseVelocidadeAlvo);
            lv_obj_align(lbl_vel, LV_ALIGN_TOP_MID, 0, 50);
            
            // Roller 0-60 km/h (Seção 1: substituiu slider 0-100)
            lv_obj_t * roller_cruise = lv_roller_create(cont);
            char cruiseOpts[256] = "";
            for(int i = 0; i <= 60; i++) {
                char n[5]; snprintf(n, sizeof(n), "%d", i);
                strcat(cruiseOpts, n);
                if(i < 60) strcat(cruiseOpts, "\n");
            }
            lv_roller_set_options(roller_cruise, cruiseOpts, LV_ROLLER_MODE_NORMAL);
            lv_roller_set_visible_row_count(roller_cruise, 5);
            lv_obj_set_size(roller_cruise, 100, 110);
            lv_obj_align(roller_cruise, LV_ALIGN_CENTER, 0, -5);
            int cruiseIdx = (int)cruiseVelocidadeAlvo;
            if(cruiseIdx > 60) cruiseIdx = 60;
            lv_roller_set_selected(roller_cruise, cruiseIdx, LV_ANIM_OFF);
            
            lv_obj_add_event_cb(roller_cruise, [](lv_event_t*e){
                lv_obj_t * r = (lv_obj_t*)lv_event_get_target(e);
                cruiseVelocidadeAlvo = (float)lv_roller_get_selected(r);
                if(cruiseAtivo) {
                    char cmd[32];
                    snprintf(cmd, sizeof(cmd), "CRUISE,ON,%.0f\n", cruiseVelocidadeAlvo);
                    SerialBT.print(cmd);
                }
            }, LV_EVENT_VALUE_CHANGED, NULL);
            
            // Botão ATIVAR/DESATIVAR com mudança de cor
            lv_obj_t * btn_cruise = lv_btn_create(cont);
            lv_obj_set_size(btn_cruise, 220, 50);
            lv_obj_align(btn_cruise, LV_ALIGN_CENTER, 0, 50);
            
            if(cruiseAtivo) {
                // ATIVO: botão VERMELHO com texto "DESATIVAR"
                lv_obj_set_style_bg_color(btn_cruise, lv_palette_main(LV_PALETTE_RED), 0);
                lv_obj_t * lblCr = lv_label_create(btn_cruise);
                lv_label_set_text(lblCr, LV_SYMBOL_POWER " DESATIVAR");
                lv_obj_center(lblCr);
            } else {
                // INATIVO: botão VERDE com texto "ATIVAR"
                lv_obj_set_style_bg_color(btn_cruise, lv_palette_main(LV_PALETTE_GREEN), 0);
                lv_obj_t * lblCr = lv_label_create(btn_cruise);
                lv_label_set_text(lblCr, LV_SYMBOL_POWER " ATIVAR");
                lv_obj_center(lblCr);
            }
            
            lv_obj_add_event_cb(btn_cruise, [](lv_event_t*e){
                cruiseAtivo = !cruiseAtivo;
                alertaCruiseOn = cruiseAtivo;
                char cmd[32];
                if(cruiseAtivo) {
                    snprintf(cmd, sizeof(cmd), "CRUISE,ON,%.0f\n", cruiseVelocidadeAlvo);
                } else {
                    snprintf(cmd, sizeof(cmd), "CRUISE,OFF\n");
                }
                SerialBT.print(cmd);
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        // ====================================================================================
        // TELAS ORIGINAIS DO MÓDULO DE CONSUMO (mantidas integralmente)
        // ====================================================================================
        else if (paginaAtual == MENU_PRINCIPAL_CONS) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 280, 160);
            lv_obj_center(list);

            const char* nomes[] = {"BATERIA PRINCIPAL", "BATERIA RESERVA", "MOTORES"};
            const char* icones[] = {LV_SYMBOL_BATTERY_FULL, LV_SYMBOL_BATTERY_2, LV_SYMBOL_CHARGE};
            for(int i = 0; i < 3; i++) {
                lv_obj_t * b = lv_list_add_btn(list, icones[i], nomes[i]);
                lv_obj_add_event_cb(b, [](lv_event_t* e){
                    int idx = (int)(intptr_t)lv_event_get_user_data(e);
                    if(idx == 2) {
                        paginaAtual = MENU_MOTORES_CONS;
                    } else {
                        itemSelecionado = idx;
                        paginaAtual = SUBMENU_DETALHES_CONS;
                    }
                    montarInterface();
                }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
            }
        }
        else if (paginaAtual == MENU_MOTORES_CONS) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 280, 160);
            lv_obj_center(list);

            lv_obj_t * b1 = lv_list_add_btn(list, LV_SYMBOL_CHARGE, "MOTOR 1");
            lv_obj_add_event_cb(b1, [](lv_event_t* e){ itemSelecionado = 2; paginaAtual = SUBMENU_DETALHES_CONS; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            lv_obj_t * b2 = lv_list_add_btn(list, LV_SYMBOL_CHARGE, "MOTOR 2");
            lv_obj_add_event_cb(b2, [](lv_event_t* e){ itemSelecionado = 3; paginaAtual = SUBMENU_DETALHES_CONS; montarInterface(); }, LV_EVENT_CLICKED, NULL);
        }
        else if (paginaAtual == SUBMENU_DETALHES_CONS) {
            lv_obj_t * titulo = lv_label_create(cont);
            const char* nomes_titulos[] = {"PRINCIPAL", "RESERVA", "MOTOR 1", "MOTOR 2"};
            if (itemSelecionado >= 0 && itemSelecionado < 4) {
                lv_label_set_text_fmt(titulo, LV_SYMBOL_EDIT " %s", nomes_titulos[itemSelecionado]);
            }
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, -5);

            lv_obj_t * list_dados = lv_list_create(cont);
            lv_obj_set_size(list_dados, 290, 130);
            lv_obj_align(list_dados, LV_ALIGN_TOP_MID, 0, 25);

            char buf[64];

            // 1. Tempo Total
            int h = dados[itemSelecionado].tempoSegundos / 3600;
            int m = (dados[itemSelecionado].tempoSegundos % 3600) / 60;
            int s = (dados[itemSelecionado].tempoSegundos % 60);
            snprintf(buf, sizeof(buf), "Tempo: %02dh %02dm %02ds", h, m, s);
            lv_obj_t * bT = lv_list_add_btn(list_dados, LV_SYMBOL_REFRESH, buf);
            lv_obj_add_event_cb(bT, evento_zerar_especifico, LV_EVENT_ALL, (void*)3);
            // 2. Pico de Amperagem
            snprintf(buf, sizeof(buf), "Pico Amperagem: %.2f A", dados[itemSelecionado].picoAmperagem);
            lv_obj_t * bP = lv_list_add_btn(list_dados, LV_SYMBOL_UP, buf);
            lv_obj_add_event_cb(bP, evento_zerar_especifico, LV_EVENT_ALL, (void*)4);
            // 3. Consumo Médio
            snprintf(buf, sizeof(buf), "Consumo Med.: %.2f A", dados[itemSelecionado].consumoMedioAmperagem);
            lv_obj_t * bM = lv_list_add_btn(list_dados, LV_SYMBOL_LOOP, buf);
            lv_obj_add_event_cb(bM, evento_zerar_especifico, LV_EVENT_ALL, (void*)5);
            // 4. Quilometragem por Ampere
            snprintf(buf, sizeof(buf), "Km por Ampere: %.2f Km/A", dados[itemSelecionado].eficiencia);
            lv_obj_t * bE = lv_list_add_btn(list_dados, LV_SYMBOL_CHARGE, buf);
            lv_obj_add_event_cb(bE, evento_zerar_especifico, LV_EVENT_ALL, (void*)2);
            // 5. Distancia Total
            snprintf(buf, sizeof(buf), "Dist. Total: %.2f Km", dados[itemSelecionado].kmTotal);
            lv_obj_t * bD = lv_list_add_btn(list_dados, LV_SYMBOL_GPS, buf);
            lv_obj_add_event_cb(bD, evento_zerar_especifico, LV_EVENT_ALL, (void*)0);
        }

        // ====================================================================================
        // TELAS ORIGINAIS DO MÓDULO SOLAR (mantidas integralmente)
        // ====================================================================================
        else if (paginaAtual == MENU_SOLAR) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 280, 160);
            lv_obj_center(list);

            lv_obj_t * b1 = lv_list_add_btn(list, LV_SYMBOL_POWER, "CARREG. SOLAR");
            lv_obj_add_event_cb(b1, [](lv_event_t* e){ paginaAtual = SOLAR_TOGGLE; montarInterface(); }, LV_EVENT_CLICKED, NULL);

            lv_obj_t * b2 = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "AJUSTE PORCENTAGEM");
            lv_obj_add_event_cb(b2, [](lv_event_t* e){ paginaAtual = SOLAR_AJUSTE; montarInterface(); }, LV_EVENT_CLICKED, NULL);
        }
        else if (paginaAtual == SOLAR_TOGGLE) {
            lv_obj_t * info = lv_label_create(cont);
            lv_label_set_text(info, "SISTEMA DE CARREGAMENTO:");
            lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 10);

            lv_obj_t * btn_toggle = lv_btn_create(cont);
            lv_obj_set_size(btn_toggle, 200, 70);
            lv_obj_align(btn_toggle, LV_ALIGN_CENTER, 0, 15);

            if (solarLigado) {
                lv_obj_set_style_bg_color(btn_toggle, lv_palette_main(LV_PALETTE_GREEN), 0);
            } else {
                lv_obj_set_style_bg_color(btn_toggle, lv_palette_main(LV_PALETTE_RED), 0);
            }

            lv_obj_t * lbl = lv_label_create(btn_toggle);
            lv_label_set_text(lbl, solarLigado ? LV_SYMBOL_POWER " LIGADO" : LV_SYMBOL_POWER " DESLIGADO");
            lv_obj_center(lbl);

            lv_obj_add_event_cb(btn_toggle, [](lv_event_t*e){
                solarLigado = !solarLigado;
                salvarConfiguracoesSolar();
                Serial.println(solarLigado ? "[SOLAR] Carregamento Solar LIGADO pelo utilizador." : "[SOLAR] Carregamento Solar DESLIGADO pelo utilizador.");
                controlarRelesSolar();
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        else if (paginaAtual == SOLAR_AJUSTE) {
            lv_obj_t * info = lv_label_create(cont);
            lv_label_set_text(info, "CORTE SOLAR (%):");
            lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 5);

            lv_obj_t * roller = lv_roller_create(cont);
            const char * opcoes = "0\n5\n10\n15\n20\n25\n30\n35\n40\n45\n50\n55\n60\n65\n70\n75\n80\n85\n90\n95\n100";
            lv_roller_set_options(roller, opcoes, LV_ROLLER_MODE_NORMAL);
            lv_roller_set_visible_row_count(roller, 3);
            lv_obj_align(roller, LV_ALIGN_CENTER, 0, 15);

            uint16_t index_inicial = (uint16_t)(ajustePorcentagem / 5.0);
            lv_roller_set_selected(roller, index_inicial, LV_ANIM_OFF);

            lv_obj_add_event_cb(roller, [](lv_event_t*e){
                lv_obj_t * r = (lv_obj_t *)lv_event_get_target(e);
                ajustePorcentagem = lv_roller_get_selected(r) * 5.0;
                salvarConfiguracoesSolar();
                char logMsg[64];
                snprintf(logMsg, sizeof(logMsg), "[SOLAR] Ajuste Porcentagem alterado para: %.0f%%\n", ajustePorcentagem);
                Serial.print(logMsg);
                controlarRelesSolar();
            }, LV_EVENT_VALUE_CHANGED, NULL);
        }

        // ====================================================================================
        // TELA QUILOMETRAGEM (mantida do original)
        // ====================================================================================
        else if (paginaAtual == KM) {
            lv_obj_t * lbl_tit = lv_label_create(cont);
            lv_label_set_text(lbl_tit, "RESET DE QUILOMETRAGEM"); lv_obj_align(lbl_tit, LV_ALIGN_TOP_MID, 0, 10);
            lv_obj_t * btn_p = lv_btn_create(cont); lv_obj_set_size(btn_p, 130, 80); lv_obj_align(btn_p, LV_ALIGN_CENTER, -70, 0);
            lv_obj_t * lbl_p = lv_label_create(btn_p); lv_label_set_text_fmt(lbl_p, "PARCIAL\n%.1f km", kmParcial); lv_obj_center(lbl_p);
            lv_obj_add_event_cb(btn_p, [](lv_event_t* e){
                if(lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
                    kmParcial = 0.0; salvarConfiguracoes(); Serial.println("C,KP,0"); montarInterface();
                }
            }, LV_EVENT_ALL, NULL);
            lv_obj_t * btn_t = lv_btn_create(cont); lv_obj_set_size(btn_t, 130, 80); lv_obj_align(btn_t, LV_ALIGN_CENTER, 70, 0);
            lv_obj_t * lbl_t = lv_label_create(btn_t);
            lv_label_set_text_fmt(lbl_t, "TOTAL\n%.1f km", kmTotal); lv_obj_center(lbl_t);
            lv_obj_add_event_cb(btn_t, [](lv_event_t* e){
                if(lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
                    kmTotal = 0.0; salvarDadosVeiculo(); Serial.println("C,KT,0"); montarInterface();
                }
            }, LV_EVENT_ALL, NULL);
        }

        // ====================================================================================
        // TELA SELEÇÃO BATERIA ATIVA (mantida do original)
        // ====================================================================================
        else if (paginaAtual == SEL_BATERIA_ATIVA) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 260, 140); lv_obj_center(list);
            lv_obj_t * b1 = lv_list_add_btn(list, LV_SYMBOL_BATTERY_FULL, "BATERIA PRINCIPAL (B1)");
            lv_obj_add_event_cb(b1, [](lv_event_t*e){ bateriaEmUso = 1; enviarStatusReles(); salvarConfiguracoes(); paginaAtual = MENU_HODOMETRO; emSubMenu = false; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            if(bateriaEmUso == 1) lv_obj_set_style_bg_color(b1, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_obj_t * b2 = lv_list_add_btn(list, LV_SYMBOL_BATTERY_2, "BATERIA RESERVA (B2)");
            lv_obj_add_event_cb(b2, [](lv_event_t*e){ bateriaEmUso = 2; enviarStatusReles(); salvarConfiguracoes(); paginaAtual = MENU_HODOMETRO; emSubMenu = false; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            if(bateriaEmUso == 2) lv_obj_set_style_bg_color(b2, lv_palette_main(LV_PALETTE_GREEN), 0);
        }

        // ====================================================================================
        // TELAS CONFIGURAÇÃO DE BATERIA (mantidas do original)
        // ====================================================================================
        else if (paginaAtual == CONFIG_BAT) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 260, 140); lv_obj_center(list);
            for(int i=0; i<3; i++) {
                char bname[20];
                snprintf(bname, sizeof(bname), "Bateria %d", i+1);
                lv_obj_t * b = lv_list_add_btn(list, LV_SYMBOL_BATTERY_FULL, bname);
                lv_obj_add_event_cb(b, [](lv_event_t*e){ batEdicao = (int)(intptr_t)lv_event_get_user_data(e); voltagemReferenciaMultimetro = obterVoltagemReal(batEdicao); paginaAtual = AJUSTE_ESPECIFICO; montarInterface(); }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
            }
        }
        else if (paginaAtual == AJUSTE_ESPECIFICO) {
            Limites lim = obterLimitesBateria(batEdicao);
            lv_obj_t * info = lv_label_create(cont); lv_label_set_recolor(info, true);
            lv_label_set_text_fmt(info, "ATUAL: #0000ff %.1fV# (%d%%)\n#ff0000 MIN: %.1fV# | #00ff00 MAX: %.1fV#", obterVoltagemReal(batEdicao), calcularPorcentagem(batEdicao, obterVoltagemReal(batEdicao)), lim.vMin, lim.vMax);
            lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 5);
            lv_obj_t * list = lv_list_create(cont); lv_obj_set_size(list, 260, 120); lv_obj_align(list, LV_ALIGN_CENTER, 0, 10);
            char bufS[30];
            snprintf(bufS, sizeof(bufS), "SISTEMA (%dV)", confBat[batEdicao].sistema); 
            lv_obj_add_event_cb(lv_list_add_btn(list, LV_SYMBOL_EDIT, bufS), [](lv_event_t*e){ paginaAtual = SEL_VOLTAGEM; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            const char * nQ[] = {"CHUMBO", "LITIO", "LIFEPO4"}; 
            char bufQ[30]; 
            snprintf(bufQ, sizeof(bufQ), "QUIMICA (%s)", nQ[confBat[batEdicao].quimica]);
            lv_obj_add_event_cb(lv_list_add_btn(list, LV_SYMBOL_CHARGE, bufQ), [](lv_event_t*e){ paginaAtual = SEL_QUIMICA; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            char bufC[30]; 
            snprintf(bufC, sizeof(bufC), "CAPACIDADE (%d Ah)", confBat[batEdicao].capacidade);
            lv_obj_add_event_cb(lv_list_add_btn(list, LV_SYMBOL_SD_CARD, bufC), [](lv_event_t*e){ paginaAtual = SEL_CAPACIDADE; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            lv_obj_add_event_cb(lv_list_add_btn(list, LV_SYMBOL_REFRESH, "CALIBRAR VOLTAGEM"), [](lv_event_t*e){ paginaAtual = CALIBRAR_V; montarInterface(); }, LV_EVENT_CLICKED, NULL);
        }
        else if (paginaAtual == CALIBRAR_V) {
            // BUG CORRIGIDO: Botões Voltar/Home/Confirmar estavam sobrepostos.
            // Agora: Voltar (esq), Confirmar (dir VERDE, foreground), +/- (centro)
            lv_obj_t * tit_cal = lv_label_create(cont);
            lv_label_set_text(tit_cal, "CALIBRAR VOLTAGEM");
            lv_obj_align(tit_cal, LV_ALIGN_TOP_MID, 0, 5);
            
            lv_obj_t * dV = lv_label_create(cont);
            lv_obj_set_style_text_font(dV, FONTE_GRANDE, 0);
            lv_label_set_text_fmt(dV, "%.1f V", voltagemReferenciaMultimetro);
            lv_obj_align(dV, LV_ALIGN_CENTER, 0, -20);
            
            lv_obj_t * bL = lv_btn_create(cont);
            lv_obj_set_size(bL, 60, 50);
            lv_obj_align(bL, LV_ALIGN_LEFT_MID, 10, -20);
            lv_obj_add_event_cb(bL, [](lv_event_t*e){ voltagemReferenciaMultimetro -= 0.1; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            lv_label_set_text(lv_label_create(bL), "-");
            
            lv_obj_t * bM = lv_btn_create(cont);
            lv_obj_set_size(bM, 60, 50);
            lv_obj_align(bM, LV_ALIGN_RIGHT_MID, -10, -20);
            lv_obj_add_event_cb(bM, [](lv_event_t*e){ voltagemReferenciaMultimetro += 0.1; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            lv_label_set_text(lv_label_create(bM), "+");
            
            // Botão CONFIRMAR: verde, lado direito do rodapé, na frente
            // Criado POR ÚLTIMO para garantir z-index superior
            lv_obj_t * bOk = lv_btn_create(scr);  // No scr (não cont) para evitar sobreposição
            lv_obj_set_size(bOk, 150, 50);
            lv_obj_align(bOk, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
            lv_obj_set_style_bg_color(bOk, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_obj_move_foreground(bOk);  // Garante que fica na frente
            lv_obj_add_event_cb(bOk, [](lv_event_t*e){
                float bruto = lerVoltagemBruta();
                if (bruto > 0.1) confBat[batEdicao].fatorCalib = voltagemReferenciaMultimetro / bruto;
                salvarConfigBateria();
                paginaAtual = AJUSTE_ESPECIFICO;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            lv_obj_t * lbl_ok = lv_label_create(bOk);
            lv_label_set_text(lbl_ok, LV_SYMBOL_OK " CONFIRMAR");
            lv_obj_center(lbl_ok);
        }
        else if (paginaAtual == SEL_CAPACIDADE) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 260, 180); lv_obj_center(list);
            for(int i = 1; i <= 50; i++) {
                char txt[15];
                snprintf(txt, sizeof(txt), "%d Ah", i); lv_obj_t * b = lv_list_add_btn(list, NULL, txt); if(confBat[batEdicao].capacidade == i) lv_obj_set_style_bg_color(b, lv_palette_main(LV_PALETTE_GREEN), 0);
                lv_obj_add_event_cb(b, [](lv_event_t* e){ confBat[batEdicao].capacidade = (int)(intptr_t)lv_event_get_user_data(e); salvarConfigBateria(); paginaAtual = AJUSTE_ESPECIFICO; montarInterface(); }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
            }
        }
        else if (paginaAtual == SEL_VOLTAGEM) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 260, 180); lv_obj_center(list);
            int volts[] = {12, 24, 36, 48, 60, 72};
            for(int v : volts) {
                char txt[15];
                snprintf(txt, sizeof(txt), "%d Volts", v); lv_obj_t * b = lv_list_add_btn(list, NULL, txt); if(confBat[batEdicao].sistema == v) lv_obj_set_style_bg_color(b, lv_palette_main(LV_PALETTE_BLUE), 0);
                lv_obj_add_event_cb(b, [](lv_event_t* e){ confBat[batEdicao].sistema = (int)(intptr_t)lv_event_get_user_data(e); salvarConfigBateria(); paginaAtual = AJUSTE_ESPECIFICO; montarInterface(); }, LV_EVENT_CLICKED, (void*)(intptr_t)v);
            }
        }
        else if (paginaAtual == SEL_QUIMICA) {
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 260, 180); lv_obj_center(list);
            const char * ops[] = {"CHUMBO", "LITIO", "LIFEPO4"};
            for(int i=0; i<3; i++) {
                lv_obj_t * b = lv_list_add_btn(list, LV_SYMBOL_CHARGE, ops[i]);
                if(confBat[batEdicao].quimica == i) lv_obj_set_style_bg_color(b, lv_palette_main(LV_PALETTE_ORANGE), 0);
                lv_obj_add_event_cb(b, [](lv_event_t* e){ confBat[batEdicao].quimica = (int)(intptr_t)lv_event_get_user_data(e); salvarConfigBateria(); paginaAtual = AJUSTE_ESPECIFICO; montarInterface(); }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
            }
        }

        // ====================================================================================
        // RADAR DE PROXIMIDADE TRASEIRO - VISUAL AUTOMOTIVO DUAL SENSOR
        // ====================================================================================
        // Tela dedicada ao radar de ré com dois arcos (lv_arc) paralelos:
        //   Arco Esquerdo = sensor traseiro esquerdo
        //   Arco Direito  = sensor traseiro direito
        // Zonas de cor dinâmicas:
        //   VERDE  (>100cm): Livre
        //   AMARELO (40-100cm): Atenção
        //   VERMELHO (<40cm): Perigo
        // Distância numérica (cm) abaixo de cada arco.
        // Dados recebidos do Mega via Serial: RADAR,<esq>,<dir>\n
        // Só exibe se radarHabilitado == true (NVS) E R4 ativa.
        else if (paginaAtual == RADAR_PROXIMIDADE) {
            // Título
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, "RADAR TRASEIRO");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, lv_color_hex(0x00FFFF), 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            if(!radarHabilitado) {
                lv_obj_t * lbl_off = lv_label_create(cont);
                lv_label_set_text(lbl_off, "RADAR DESABILITADO\n\nAtive nas configuracoes\nde conducao");
                lv_obj_set_style_text_align(lbl_off, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_set_style_text_color(lbl_off, lv_color_hex(0x888888), 0);
                lv_obj_center(lbl_off);
            } else if(!radarAtivo) {
                lv_obj_t * lbl_inativo = lv_label_create(cont);
                lv_label_set_text(lbl_inativo, "RADAR INATIVO\n\nLigue a marcha re (R4)\npara ativar o sensor");
                lv_obj_set_style_text_align(lbl_inativo, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_set_style_text_color(lbl_inativo, lv_color_hex(0x888888), 0);
                lv_obj_center(lbl_inativo);
            } else {
                // Função auxiliar inline para calcular cor de zona
                // VERDE (>100cm) | AMARELO (40-100cm) | VERMELHO (<40cm)
                auto corPorDistancia = [](int dist) -> lv_color_t {
                    if(dist < 40) return lv_color_hex(0xFF0000);       // VERMELHO: perigo
                    else if(dist <= 100) return lv_color_hex(0xFFFF00); // AMARELO: atenção
                    else return lv_color_hex(0x00FF00);                 // VERDE: livre
                };
                
                auto textoPorDistancia = [](int dist) -> const char* {
                    if(dist < 40) return "PERIGO!";
                    else if(dist <= 100) return "ATENCAO";
                    else return "LIVRE";
                };
                
                // ---- Labels ESQ / DIR no topo ----
                lv_obj_t * lblEsqTit = lv_label_create(cont);
                lv_label_set_text(lblEsqTit, "ESQ");
                lv_obj_set_style_text_color(lblEsqTit, lv_color_hex(0xAAAAAA), 0);
                lv_obj_align(lblEsqTit, LV_ALIGN_TOP_LEFT, 35, 25);
                
                lv_obj_t * lblDirTit = lv_label_create(cont);
                lv_label_set_text(lblDirTit, "DIR");
                lv_obj_set_style_text_color(lblDirTit, lv_color_hex(0xAAAAAA), 0);
                lv_obj_align(lblDirTit, LV_ALIGN_TOP_RIGHT, -35, 25);
                
                // ---- ARCO ESQUERDO ----
                int distEsq = radarDistanciaEsq;
                lv_color_t corEsq = corPorDistancia(distEsq);
                
                lv_obj_t * arcoEsq = lv_arc_create(cont);
                lv_obj_set_size(arcoEsq, 110, 110);
                lv_obj_align(arcoEsq, LV_ALIGN_CENTER, -55, -5);
                lv_arc_set_rotation(arcoEsq, 135);
                lv_arc_set_bg_angles(arcoEsq, 0, 270);
                lv_arc_set_range(arcoEsq, 0, 400);
                lv_obj_clear_flag(arcoEsq, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_style_arc_width(arcoEsq, 12, LV_PART_MAIN);
                lv_obj_set_style_arc_width(arcoEsq, 12, LV_PART_INDICATOR);
                lv_obj_set_style_arc_color(arcoEsq, temaEscuro ? lv_color_hex(0x333333) : lv_color_hex(0xCCCCCC), LV_PART_MAIN);
                lv_obj_set_style_arc_color(arcoEsq, corEsq, LV_PART_INDICATOR);
                lv_obj_set_style_bg_opa(arcoEsq, LV_OPA_TRANSP, LV_PART_KNOB);
                lv_obj_set_style_pad_all(arcoEsq, 0, LV_PART_KNOB);
                int valEsq = 400 - distEsq;
                if(valEsq < 0) valEsq = 0;
                if(valEsq > 400) valEsq = 400;
                lv_arc_set_value(arcoEsq, valEsq);
                
                // Distância numérica dentro do arco esquerdo
                lv_obj_t * lblDistEsq = lv_label_create(arcoEsq);
                lv_obj_set_style_text_font(lblDistEsq, &lv_font_montserrat_22, 0);
                lv_obj_set_style_text_color(lblDistEsq, corEsq, 0);
                char bufEsq[12];
                snprintf(bufEsq, sizeof(bufEsq), "%dcm", distEsq);
                lv_label_set_text(lblDistEsq, bufEsq);
                lv_obj_center(lblDistEsq);
                
                // ---- ARCO DIREITO ----
                int distDir = radarDistanciaDir;
                lv_color_t corDir = corPorDistancia(distDir);
                
                lv_obj_t * arcoDir = lv_arc_create(cont);
                lv_obj_set_size(arcoDir, 110, 110);
                lv_obj_align(arcoDir, LV_ALIGN_CENTER, 55, -5);
                lv_arc_set_rotation(arcoDir, 135);
                lv_arc_set_bg_angles(arcoDir, 0, 270);
                lv_arc_set_range(arcoDir, 0, 400);
                lv_obj_clear_flag(arcoDir, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_style_arc_width(arcoDir, 12, LV_PART_MAIN);
                lv_obj_set_style_arc_width(arcoDir, 12, LV_PART_INDICATOR);
                lv_obj_set_style_arc_color(arcoDir, temaEscuro ? lv_color_hex(0x333333) : lv_color_hex(0xCCCCCC), LV_PART_MAIN);
                lv_obj_set_style_arc_color(arcoDir, corDir, LV_PART_INDICATOR);
                lv_obj_set_style_bg_opa(arcoDir, LV_OPA_TRANSP, LV_PART_KNOB);
                lv_obj_set_style_pad_all(arcoDir, 0, LV_PART_KNOB);
                int valDir = 400 - distDir;
                if(valDir < 0) valDir = 0;
                if(valDir > 400) valDir = 400;
                lv_arc_set_value(arcoDir, valDir);
                
                // Distância numérica dentro do arco direito
                lv_obj_t * lblDistDir = lv_label_create(arcoDir);
                lv_obj_set_style_text_font(lblDistDir, &lv_font_montserrat_22, 0);
                lv_obj_set_style_text_color(lblDistDir, corDir, 0);
                char bufDir[12];
                snprintf(bufDir, sizeof(bufDir), "%dcm", distDir);
                lv_label_set_text(lblDistDir, bufDir);
                lv_obj_center(lblDistDir);
                
                // ---- Labels de zona abaixo dos arcos ----
                lv_obj_t * lblZonaEsq = lv_label_create(cont);
                lv_obj_set_style_text_color(lblZonaEsq, corEsq, 0);
                lv_label_set_text(lblZonaEsq, textoPorDistancia(distEsq));
                lv_obj_align(lblZonaEsq, LV_ALIGN_BOTTOM_LEFT, 20, -25);
                
                lv_obj_t * lblZonaDir = lv_label_create(cont);
                lv_obj_set_style_text_color(lblZonaDir, corDir, 0);
                lv_label_set_text(lblZonaDir, textoPorDistancia(distDir));
                lv_obj_align(lblZonaDir, LV_ALIGN_BOTTOM_RIGHT, -20, -25);
            }
        }

        // ====================================================================================
        // ====================================================================================
        // MÓDULO G: PREÇO POR QUILÔMETRO - MENU PRINCIPAL
        // ====================================================================================
        else if (paginaAtual == PRECO_POR_KM) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, LV_SYMBOL_CHARGE " PRECO POR KM");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 280, 150);
            lv_obj_center(list);
            
            lv_obj_t * b1 = lv_list_add_btn(list, LV_SYMBOL_CHARGE, "CARREGADORES");
            lv_obj_add_event_cb(b1, [](lv_event_t* e){
                paginaAtual = PRECO_CARREGADORES;
                emSubMenu = true;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            
            lv_obj_t * b2 = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "PRECO DA ENERGIA");
            lv_obj_add_event_cb(b2, [](lv_event_t* e){
                paginaAtual = PRECO_ENERGIA;
                emSubMenu = true;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            
            lv_obj_t * b3 = lv_list_add_btn(list, LV_SYMBOL_LIST, "HISTORICO");
            lv_obj_add_event_cb(b3, [](lv_event_t* e){
                paginaAtual = PRECO_HIST_BAT;
                emSubMenu = true;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            
            // Status de carga (se detectada)
            if(cargaDetectada) {
                lv_obj_t * lblCarga = lv_label_create(cont);
                lv_label_set_recolor(lblCarga, true);
                char cBuf[80];
                snprintf(cBuf, sizeof(cBuf), "#00FF00 CARREGANDO B%d: +%d%% em %.1fh#", cargaBateria, cargaGanhoPerc, cargaTempoH);
                lv_label_set_text(lblCarga, cBuf);
                lv_obj_align(lblCarga, LV_ALIGN_BOTTOM_MID, 0, -65);
            }
        }
        
        // ====================================================================================
        // MÓDULO G: SUBMENU CARREGADORES (B1 / B2)
        // ====================================================================================
        else if (paginaAtual == PRECO_CARREGADORES) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, LV_SYMBOL_CHARGE " CARREGADORES");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            lv_obj_t * list = lv_list_create(cont);
            lv_obj_set_size(list, 280, 120);
            lv_obj_center(list);
            
            // Botão Carregador B1
            lv_obj_t * bB1 = lv_list_add_btn(list, LV_SYMBOL_BATTERY_FULL, "CARREGADOR BAT 1");
            char lb1[32]; snprintf(lb1, sizeof(lb1), "%.0f W", potCarregB1);
            lv_obj_t * lblB1val = lv_label_create(bB1);
            lv_label_set_text(lblB1val, lb1);
            lv_obj_set_style_text_color(lblB1val, lv_color_hex(0x0055FF), 0);
            lv_obj_add_event_cb(bB1, [](lv_event_t* e){
                precoEditBat = 0;
                paginaAtual = PRECO_CARREG_EDIT;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            
            // Botão Carregador B2
            lv_obj_t * bB2 = lv_list_add_btn(list, LV_SYMBOL_BATTERY_FULL, "CARREGADOR BAT 2");
            char lb2[32]; snprintf(lb2, sizeof(lb2), "%.0f W", potCarregB2);
            lv_obj_t * lblB2val = lv_label_create(bB2);
            lv_label_set_text(lblB2val, lb2);
            lv_obj_set_style_text_color(lblB2val, lv_color_hex(0x0055FF), 0);
            lv_obj_add_event_cb(bB2, [](lv_event_t* e){
                precoEditBat = 1;
                paginaAtual = PRECO_CARREG_EDIT;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        
        // ====================================================================================
        // MÓDULO G: EDITOR DE POTÊNCIA DO CARREGADOR (lv_roller 0-1000W, step 10)
        // ====================================================================================
        else if (paginaAtual == PRECO_CARREG_EDIT) {
            lv_obj_t * titulo = lv_label_create(cont);
            char tBuf[40];
            snprintf(tBuf, sizeof(tBuf), "CARREGADOR BAT %d", precoEditBat + 1);
            lv_label_set_text(titulo, tBuf);
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            // Valor atual
            float potAtual = (precoEditBat == 0) ? potCarregB1 : potCarregB2;
            lv_obj_t * lblAtual = lv_label_create(cont);
            char aBuf[32]; snprintf(aBuf, sizeof(aBuf), "Atual: %.0f W", potAtual);
            lv_label_set_text(lblAtual, aBuf);
            lv_obj_set_style_text_color(lblAtual, lv_color_hex(0x0055FF), 0);
            lv_obj_align(lblAtual, LV_ALIGN_TOP_MID, 0, 25);
            
            // Roller 0-1000W, step 10
            lv_obj_t * roller = lv_roller_create(cont);
            static char opts[5000]; opts[0] = '\0';
            for(int i = 0; i <= 1000; i += 10) {
                char n[8]; snprintf(n, sizeof(n), "%d W", i);
                strcat(opts, n);
                if(i < 1000) strcat(opts, "\n");
            }
            lv_roller_set_options(roller, opts, LV_ROLLER_MODE_NORMAL);
            lv_roller_set_visible_row_count(roller, 5);
            lv_obj_set_size(roller, 120, 110);
            lv_obj_align(roller, LV_ALIGN_CENTER, 0, -5);
            int selIdx = (int)(potAtual / 10.0);
            if(selIdx > 100) selIdx = 100;
            lv_roller_set_selected(roller, selIdx, LV_ANIM_OFF);
            
            lv_obj_add_event_cb(roller, [](lv_event_t*e){
                lv_obj_t * r = (lv_obj_t*)lv_event_get_target(e);
                float val = (float)(lv_roller_get_selected(r) * 10);
                if(precoEditBat == 0) potCarregB1 = val;
                else potCarregB2 = val;
                salvarPrecoKm();
            }, LV_EVENT_VALUE_CHANGED, NULL);
            
            // Botão Salvar
            lv_obj_t * btnSalvar = lv_btn_create(cont);
            lv_obj_set_size(btnSalvar, 200, 40);
            lv_obj_align(btnSalvar, LV_ALIGN_BOTTOM_MID, 0, -65);
            lv_obj_set_style_bg_color(btnSalvar, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_label_set_text(lv_label_create(btnSalvar), LV_SYMBOL_OK " SALVAR");
            lv_obj_add_event_cb(btnSalvar, [](lv_event_t*e){
                salvarPrecoKm();
                paginaAtual = PRECO_CARREGADORES;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        
        // ====================================================================================
        // MÓDULO G: PREÇO DA ENERGIA (lv_roller 0.0-10.0, step 0.1)
        // ====================================================================================
        else if (paginaAtual == PRECO_ENERGIA) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, LV_SYMBOL_SETTINGS " PRECO DA ENERGIA");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            // Valor atual
            lv_obj_t * lblAtual = lv_label_create(cont);
            char aBuf[40]; snprintf(aBuf, sizeof(aBuf), "Atual: R$ %.2f / kWh", precoKWh);
            lv_label_set_text(lblAtual, aBuf);
            lv_obj_set_style_text_color(lblAtual, lv_color_hex(0x0055FF), 0);
            lv_obj_align(lblAtual, LV_ALIGN_TOP_MID, 0, 25);
            
            // Roller 0.0 - 10.0, step 0.1
            lv_obj_t * roller = lv_roller_create(cont);
            static char opts[3000]; opts[0] = '\0';
            for(int i = 0; i <= 100; i++) {
                char n[12]; snprintf(n, sizeof(n), "R$ %.1f", i * 0.1);
                strcat(opts, n);
                if(i < 100) strcat(opts, "\n");
            }
            lv_roller_set_options(roller, opts, LV_ROLLER_MODE_NORMAL);
            lv_roller_set_visible_row_count(roller, 5);
            lv_obj_set_size(roller, 140, 110);
            lv_obj_align(roller, LV_ALIGN_CENTER, 0, -5);
            int selIdx = (int)(precoKWh * 10.0 + 0.5);
            if(selIdx > 100) selIdx = 100;
            lv_roller_set_selected(roller, selIdx, LV_ANIM_OFF);
            
            lv_obj_add_event_cb(roller, [](lv_event_t*e){
                lv_obj_t * r = (lv_obj_t*)lv_event_get_target(e);
                precoKWh = lv_roller_get_selected(r) * 0.1;
                salvarPrecoKm();
            }, LV_EVENT_VALUE_CHANGED, NULL);
            
            // Botão Salvar
            lv_obj_t * btnSalvar = lv_btn_create(cont);
            lv_obj_set_size(btnSalvar, 200, 40);
            lv_obj_align(btnSalvar, LV_ALIGN_BOTTOM_MID, 0, -65);
            lv_obj_set_style_bg_color(btnSalvar, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_label_set_text(lv_label_create(btnSalvar), LV_SYMBOL_OK " SALVAR");
            lv_obj_add_event_cb(btnSalvar, [](lv_event_t*e){
                salvarPrecoKm();
                paginaAtual = PRECO_POR_KM;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        
        // ====================================================================================
        // MÓDULO G: HISTÓRICO - SELEÇÃO DE BATERIA
        // ====================================================================================
        else if (paginaAtual == PRECO_HIST_BAT) {
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text(titulo, LV_SYMBOL_LIST " HISTORICO");
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            lv_obj_t * lblSel = lv_label_create(cont);
            lv_label_set_text(lblSel, "Selecione a bateria:");
            lv_obj_set_style_text_color(lblSel, corTexto, 0);
            lv_obj_align(lblSel, LV_ALIGN_TOP_MID, 0, 30);
            
            lv_obj_t * btnB1 = lv_btn_create(cont);
            lv_obj_set_size(btnB1, 200, 50);
            lv_obj_align(btnB1, LV_ALIGN_CENTER, 0, -20);
            lv_obj_set_style_bg_color(btnB1, lv_palette_main(LV_PALETTE_BLUE), 0);
            lv_label_set_text(lv_label_create(btnB1), LV_SYMBOL_BATTERY_FULL " BATERIA 1");
            lv_obj_add_event_cb(btnB1, [](lv_event_t* e){
                precoEditBat = 0;
                paginaAtual = PRECO_HIST_FORCA;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            
            lv_obj_t * btnB2 = lv_btn_create(cont);
            lv_obj_set_size(btnB2, 200, 50);
            lv_obj_align(btnB2, LV_ALIGN_CENTER, 0, 40);
            lv_obj_set_style_bg_color(btnB2, lv_palette_main(LV_PALETTE_TEAL), 0);
            lv_label_set_text(lv_label_create(btnB2), LV_SYMBOL_BATTERY_FULL " BATERIA 2");
            lv_obj_add_event_cb(btnB2, [](lv_event_t* e){
                precoEditBat = 1;
                paginaAtual = PRECO_HIST_FORCA;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
        }
        
        // ====================================================================================
        // MÓDULO G: HISTÓRICO - SELEÇÃO DE FORÇA
        // ====================================================================================
        else if (paginaAtual == PRECO_HIST_FORCA) {
            lv_obj_t * titulo = lv_label_create(cont);
            char tBuf[40];
            snprintf(tBuf, sizeof(tBuf), "BATERIA %d - FORCA", precoEditBat + 1);
            lv_label_set_text(titulo, tBuf);
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 2);
            
            for(int f = 0; f < 3; f++) {
                lv_obj_t * btn = lv_btn_create(cont);
                lv_obj_set_size(btn, 200, 40);
                lv_obj_align(btn, LV_ALIGN_CENTER, 0, -30 + f * 45);
                lv_color_t cores[] = {
                    lv_palette_main(LV_PALETTE_GREEN),
                    lv_palette_main(LV_PALETTE_ORANGE),
                    lv_palette_main(LV_PALETTE_RED)
                };
                lv_obj_set_style_bg_color(btn, cores[f], 0);
                char fBuf[16]; snprintf(fBuf, sizeof(fBuf), "FORCA %d", f + 1);
                lv_label_set_text(lv_label_create(btn), fBuf);
                lv_obj_add_event_cb(btn, [](lv_event_t* e){
                    precoEditForca = (int)(intptr_t)lv_event_get_user_data(e);
                    paginaAtual = PRECO_HIST_RESULT;
                    montarInterface();
                }, LV_EVENT_CLICKED, (void*)(intptr_t)f);
            }
        }
        
        // ====================================================================================
        // MÓDULO G: HISTÓRICO - RESULTADO CUSTO/KM POR TRAÇÃO
        // Layout: alinhamento relativo com lv_obj_align_to (sem posicionamento fixo)
        // Cada label posicionado abaixo do anterior com 10px de espaçamento
        // Descrições em PRETO, valores em AZUL com formato *R$ [valor]*
        // ====================================================================================
        else if (paginaAtual == PRECO_HIST_RESULT) {
            // --- Título (preto, centralizado) ---
            lv_obj_t * titulo = lv_label_create(cont);
            lv_label_set_text_fmt(titulo, "B%d / FORCA %d", precoEditBat + 1, precoEditForca + 1);
            lv_obj_set_style_text_font(titulo, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(titulo, corTexto, 0);
            lv_obj_align(titulo, LV_ALIGN_TOP_MID, 0, 5);
            
            // Lê dados protegidos por mutex
            float ck1x3 = 0, ck3x3 = 0;
            if(telemetriaMutex != NULL && xSemaphoreTake(telemetriaMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                ck1x3 = custoKm[precoEditBat][precoEditForca][0];
                ck3x3 = custoKm[precoEditBat][precoEditForca][1];
                xSemaphoreGive(telemetriaMutex);
            }
            
            // Margem esquerda: 10px da borda para centralizar levemente à esquerda
            const lv_coord_t marginLeft = 10;
            
            // --- Linha 1: Descrição (preto) ---
            lv_obj_t * lbl1x3Desc = lv_label_create(cont);
            lv_label_set_text(lbl1x3Desc, "Preco por KM Modo X1:");
            lv_obj_set_style_text_color(lbl1x3Desc, corTexto, 0);
            lv_obj_set_style_text_font(lbl1x3Desc, &lv_font_montserrat_14, 0);
            lv_obj_align_to(lbl1x3Desc, titulo, LV_ALIGN_OUT_BOTTOM_LEFT, marginLeft, 10);
            
            // --- Linha 1: Valor (azul) com formato *R$ [valor]* ---
            lv_obj_t * lbl1x3Val = lv_label_create(cont);
            char buf1[32];
            if(ck1x3 > 0.0001) snprintf(buf1, sizeof(buf1), "*R$ %.4f*", ck1x3);
            else snprintf(buf1, sizeof(buf1), "*R$ 0,00*");
            lv_label_set_text(lbl1x3Val, buf1);
            lv_obj_set_style_text_color(lbl1x3Val, lv_color_hex(0x0055FF), 0);
            lv_obj_set_style_text_font(lbl1x3Val, &lv_font_montserrat_14, 0);
            lv_obj_set_width(lbl1x3Val, 280);
            lv_label_set_long_mode(lbl1x3Val, LV_LABEL_LONG_CLIP);
            lv_obj_align_to(lbl1x3Val, lbl1x3Desc, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
            
            // --- Linha 2: Descrição (preto) ---
            lv_obj_t * lbl3x3Desc = lv_label_create(cont);
            lv_label_set_text(lbl3x3Desc, "Preco por KM Modo 3x3:");
            lv_obj_set_style_text_color(lbl3x3Desc, corTexto, 0);
            lv_obj_set_style_text_font(lbl3x3Desc, &lv_font_montserrat_14, 0);
            lv_obj_align_to(lbl3x3Desc, lbl1x3Val, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
            
            // --- Linha 2: Valor (azul) ---
            lv_obj_t * lbl3x3Val = lv_label_create(cont);
            char buf3[32];
            if(ck3x3 > 0.0001) snprintf(buf3, sizeof(buf3), "*R$ %.4f*", ck3x3);
            else snprintf(buf3, sizeof(buf3), "*R$ 0,00*");
            lv_label_set_text(lbl3x3Val, buf3);
            lv_obj_set_style_text_color(lbl3x3Val, lv_color_hex(0x0055FF), 0);
            lv_obj_set_style_text_font(lbl3x3Val, &lv_font_montserrat_14, 0);
            lv_obj_set_width(lbl3x3Val, 280);
            lv_label_set_long_mode(lbl3x3Val, LV_LABEL_LONG_CLIP);
            lv_obj_align_to(lbl3x3Val, lbl3x3Desc, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
            
            // --- Linha 3: Descrição (preto) ---
            lv_obj_t * lblUltDesc = lv_label_create(cont);
            lv_label_set_text(lblUltDesc, "Ultima Carga:");
            lv_obj_set_style_text_color(lblUltDesc, corTexto, 0);
            lv_obj_set_style_text_font(lblUltDesc, &lv_font_montserrat_14, 0);
            lv_obj_align_to(lblUltDesc, lbl3x3Val, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
            
            // --- Linha 3: Valor (azul) ---
            lv_obj_t * lblUltVal = lv_label_create(cont);
            char ultBuf[32]; snprintf(ultBuf, sizeof(ultBuf), "*R$ %.2f*", custoUltimaCarga);
            lv_label_set_text(lblUltVal, ultBuf);
            lv_obj_set_style_text_color(lblUltVal, lv_color_hex(0x0055FF), 0);
            lv_obj_set_style_text_font(lblUltVal, &lv_font_montserrat_14, 0);
            lv_obj_set_width(lblUltVal, 280);
            lv_label_set_long_mode(lblUltVal, LV_LABEL_LONG_CLIP);
            lv_obj_align_to(lblUltVal, lblUltDesc, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
        }

        // ====================================================================================
        // TELAS GENÉRICAS: RE, LIMITADOR, VELOCIDADE, MOTOR, RODA (mantidas do original)
        // ====================================================================================
        else {
            lv_obj_t * tit = lv_label_create(cont);
            lv_label_set_text(tit, menuLabels[(int)paginaAtual]); lv_obj_align(tit, LV_ALIGN_TOP_MID, 0, 10);
            if (paginaAtual == RODA) {
                // Nota: O original usava String aqui - REFATORADO para char array
                lv_obj_t * roller = lv_roller_create(cont);
                char rodaOpts[2000] = "";
                for(int i=0; i<=300; i++) {
                    char num[6];
                    snprintf(num, sizeof(num), "%d", i);
                    strcat(rodaOpts, num);
                    if(i<300) strcat(rodaOpts, "\n");
                }
                lv_roller_set_options(roller, rodaOpts, LV_ROLLER_MODE_NORMAL);
                lv_roller_set_selected(roller, (uint16_t)circRodaCM, LV_ANIM_OFF);
                lv_obj_center(roller);
                // Ao alterar circunferência da roda, salva imediatamente na memória não volátil
                lv_obj_add_event_cb(roller, [](lv_event_t * e){ circRodaCM = (float)lv_roller_get_selected((lv_obj_t *)lv_event_get_target(e)); enviarConfigRoda(); salvarDadosVeiculo(); }, LV_EVENT_VALUE_CHANGED, NULL);
            }
            if (paginaAtual == RE) {
                lv_obj_t * btn_re = lv_btn_create(cont);
                lv_obj_set_size(btn_re, 150, 80); lv_obj_center(btn_re);
                lv_obj_set_style_bg_color(btn_re, modoReLigado ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED), 0);
                lv_obj_add_event_cb(btn_re, [](lv_event_t* e){ modoReLigado = !modoReLigado; enviarStatusReles(); montarInterface(); }, LV_EVENT_CLICKED, NULL);
                lv_obj_t * lbl_re = lv_label_create(btn_re);
                lv_label_set_text(lbl_re, modoReLigado ? "RE: LIGADO" : "RE: DESLIG."); lv_obj_center(lbl_re);
            }
            if (paginaAtual == VELOCIDADE) {
                lv_obj_t * c_v = lv_obj_create(cont);
                lv_obj_set_size(c_v, 280, 100); lv_obj_center(c_v); lv_obj_set_flex_flow(c_v, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(c_v, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_border_width(c_v, 0, 0); lv_obj_set_style_bg_opa(c_v, 0, 0);
                for(int i = 1; i <= 3; i++) {
                    lv_obj_t * b_v = lv_btn_create(c_v);
                    lv_obj_set_size(b_v, 70, 70); if (velocidadeAtual == i) lv_obj_set_style_bg_color(b_v, lv_palette_main(LV_PALETTE_GREEN), 0);
                    lv_obj_t * l_v = lv_label_create(b_v); lv_label_set_text_fmt(l_v, "%d", i); lv_obj_center(l_v);
                    lv_obj_add_event_cb(b_v, [](lv_event_t * e) { velocidadeAtual = (int)(intptr_t)lv_event_get_user_data(e); enviarStatusReles(); salvarConfiguracoes(); montarInterface(); }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
                }
            }
            if (paginaAtual == LIMITADOR) {
                // LIMITADOR FÍSICO (R2): Botão grande ON/OFF + indicador de valor
                lv_obj_t * btn_lim = lv_btn_create(cont);
                lv_obj_set_size(btn_lim, 200, 70);
                lv_obj_align(btn_lim, LV_ALIGN_CENTER, 0, -20);
                if(limitadorLigado) {
                    lv_obj_set_style_bg_color(btn_lim, lv_palette_main(LV_PALETTE_ORANGE), 0);
                    lv_obj_set_style_border_color(btn_lim, lv_color_hex(0xFF4400), 0);
                    lv_obj_set_style_border_width(btn_lim, 3, 0);
                } else {
                    lv_obj_set_style_bg_color(btn_lim, lv_palette_main(LV_PALETTE_GREY), 0);
                }
                lv_obj_t * lbl_lim = lv_label_create(btn_lim);
                lv_obj_set_style_text_font(lbl_lim, FONTE_GRANDE, 0);
                if(limitadorLigado) {
                    lv_label_set_text_fmt(lbl_lim, "LIMIT: %d km/h", limiteVelocidade);
                } else {
                    lv_label_set_text(lbl_lim, "LIMITADOR: OFF");
                }
                lv_obj_center(lbl_lim);
                lv_obj_add_event_cb(btn_lim, [](lv_event_t * e){
                    limitadorLigado = !limitadorLigado;
                    enviarStatusReles();  // R2 + LIMVEL ao Mega
                    salvarConfiguracoes();
                    montarInterface();
                }, LV_EVENT_CLICKED, NULL);
                
                // Indicador de Modo Disfarce (visual) se ativo
                if(modoDisfarce) {
                    lv_obj_t * lbl_disf = lv_label_create(cont);
                    lv_label_set_recolor(lbl_disf, true);
                    lv_label_set_text_fmt(lbl_disf, "#FF0000 DISFARCE: max %d km/h#", limiteDisfarce);
                    lv_obj_set_style_text_font(lbl_disf, &lv_font_montserrat_14, 0);
                    lv_obj_align(lbl_disf, LV_ALIGN_CENTER, 0, 30);
                }
            }
            if (paginaAtual == MOTOR) {
                lv_obj_t * b_m = lv_btn_create(cont);
                lv_obj_set_size(b_m, 180, 80); lv_obj_center(b_m); lv_obj_set_style_bg_color(b_m, modo3x3 ? lv_palette_main(LV_PALETTE_BLUE) : lv_palette_main(LV_PALETTE_BROWN), 0);
                lv_obj_add_event_cb(b_m, [](lv_event_t* e){ modo3x3 = !modo3x3; enviarStatusReles(); montarInterface(); }, LV_EVENT_CLICKED, NULL);
                lv_obj_t * l_m = lv_label_create(b_m);
                lv_label_set_text(l_m, modo3x3 ? "MOTOR: 3x3" : "MOTOR: 1x3"); lv_obj_center(l_m);
            }
        }
        
        // --- BOTÕES DE NAVEGAÇÃO DE SUBMENU (VOLTAR / HOME ou RESET) ---
        lv_obj_t * btn_back = lv_btn_create(scr);
        lv_obj_set_size(btn_back, 100, 50); lv_obj_align(btn_back, LV_ALIGN_BOTTOM_LEFT, 15, -15);
        lv_obj_add_event_cb(btn_back, [](lv_event_t* e){
            // Navegação do módulo A (Manutenção/Testes)
            if(paginaAtual == TESTE_AUTOMATICO || paginaAtual == TESTE_MANUAL || paginaAtual == AGENDA_MANUTENCAO
               || paginaAtual == PAREAMENTO_BT || paginaAtual == BLUETOOTH_MENU) {
                paginaAtual = MANUTENCAO_MENU;
            }
            // Seção 3: Ao sair de TESTE_ALERTAS, desliga alertas manuais
            // e envia RESET_ALARM ao Mega para limpar alertas em TODOS os dispositivos
            // (CYD local + ESP32-S3 remoto via RF). Isto garante que ao sair do
            // menu de teste, o telemóvel para de apitar e fecha o Card de Roubo.
            else if(paginaAtual == TESTE_ALERTAS) {
                // Reset de todos os alertas ativados manualmente no teste
                alertaSetaEsq = false;
                alertaSetaDir = false;
                alertaEmergencia = false;
                alertaChuva = false;
                alertaTemperatura = false;
                alertaCorrenteExc = false;
                alertaFalhaFreio = false;
                alertaBatCritica = false;
                alertaSubtensao = false;
                alertaCruiseOn = false;
                alertaCritico = false; // Mega propaga RESET_ALARM ao S3
                // Isto limpa o Card de Roubo, para o buzzer e apaga LEDs no S3
                SerialBT.print("RESET_ALARM\n");
                modoTesteAtivo = false;  // Sai do modo teste
                // Alertas reais de sensores serão reativados pela telemetria
                paginaAtual = MANUTENCAO_MENU;
            }
            // Navegação do Radar de Proximidade (volta para a página anterior ou MENU_HODOMETRO)
            else if(paginaAtual == RADAR_PROXIMIDADE) {
                paginaAtual = MENU_HODOMETRO;
                emSubMenu = false;
            }
            else if(paginaAtual == MANUTENCAO_MENU) {
                // Volta para navegação principal
                paginaAtual = MENU_HODOMETRO;
                emSubMenu = false;
            }
            // Navegação do módulo C (Condução) — inclui novos submenus
            else if(paginaAtual == CRUISE_CONTROL || paginaAtual == CONFIG_PAS || paginaAtual == CONFIG_SENSIBILIDADE
                    || paginaAtual == CONFIG_TRAVA || paginaAtual == CONFIG_LIMITE_VEL || paginaAtual == CONFIG_TEMAS) {
                paginaAtual = CONDUCAO_MENU;
            }
            else if(paginaAtual == CONDUCAO_MENU) {
                paginaAtual = MENU_HODOMETRO;
                emSubMenu = false;
            }
            // Módulo G: Navegação interna Preço por Km
            else if(paginaAtual == PRECO_HIST_RESULT) paginaAtual = PRECO_HIST_FORCA;
            else if(paginaAtual == PRECO_HIST_FORCA) paginaAtual = PRECO_HIST_BAT;
            else if(paginaAtual == PRECO_HIST_BAT) paginaAtual = PRECO_POR_KM;
            else if(paginaAtual == PRECO_CARREG_EDIT) paginaAtual = PRECO_CARREGADORES;
            else if(paginaAtual == PRECO_CARREGADORES) paginaAtual = PRECO_POR_KM;
            else if(paginaAtual == PRECO_ENERGIA) paginaAtual = PRECO_POR_KM;
            else if(paginaAtual == PRECO_POR_KM) {
                paginaAtual = MENU_HODOMETRO;
                emSubMenu = false;
            }
            // Navegação original (Solar, Consumo, Bateria, etc.)
            else if(paginaAtual == SOLAR_TOGGLE || paginaAtual == SOLAR_AJUSTE) paginaAtual = MENU_SOLAR;
            else if(paginaAtual == MENU_SOLAR) { paginaAtual = SOLAR; emSubMenu = false; }
            else if(paginaAtual == SUBMENU_DETALHES_CONS && (itemSelecionado == 2 || itemSelecionado == 3)) paginaAtual = MENU_MOTORES_CONS;
            else if(paginaAtual == MENU_MOTORES_CONS || paginaAtual == SUBMENU_DETALHES_CONS) paginaAtual = MENU_PRINCIPAL_CONS;
            else if(paginaAtual == MENU_PRINCIPAL_CONS) { paginaAtual = CONSUMO; emSubMenu = false; }
            else if(paginaAtual == CONFIG_BAT) { paginaAtual = BATERIA; emSubMenu = false; }
            else if(paginaAtual == SEL_BATERIA_ATIVA) { paginaAtual = VOLTAGEM; emSubMenu = false; }
            else if(paginaAtual == AJUSTE_ESPECIFICO) paginaAtual = CONFIG_BAT;
            else if(paginaAtual == CALIBRAR_V || paginaAtual == SEL_VOLTAGEM || paginaAtual == SEL_QUIMICA || paginaAtual == SEL_CAPACIDADE) paginaAtual = AJUSTE_ESPECIFICO;
            else { emSubMenu = false; }
            montarInterface();
        }, LV_EVENT_CLICKED, NULL);
        lv_label_set_text(lv_label_create(btn_back), "VOLTAR");

        // No TESTE_MANUAL e TESTE_AUTOMATICO: substitui Home por Reset Geral (laranja)
        // Na CALIBRAR_V: sem botão Home (Confirmar verde já ocupa direita)
        if(paginaAtual == TESTE_MANUAL || paginaAtual == TESTE_AUTOMATICO) {
            lv_obj_t * btn_reset_m = lv_btn_create(scr);
            lv_obj_set_size(btn_reset_m, 120, 50);
            lv_obj_align(btn_reset_m, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
            lv_obj_set_style_bg_color(btn_reset_m, lv_color_hex(0xFF8800), 0);
            lv_obj_add_event_cb(btn_reset_m, [](lv_event_t*e){
                enviarResetGeral();
                for(int i=0; i<15; i++) estadoRelesTeste[i] = false;
                montarInterface();
            }, LV_EVENT_CLICKED, NULL);
            lv_label_set_text(lv_label_create(btn_reset_m), "RESET");
        }
        else if(paginaAtual != CALIBRAR_V) {
            // Botão HOME padrão (não exibido na calibração para evitar sobreposição)
            lv_obj_t * btn_home = lv_btn_create(scr);
            lv_obj_set_size(btn_home, 100, 50);
            lv_obj_align(btn_home, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
            lv_obj_add_event_cb(btn_home, [](lv_event_t* e){ paginaAtual = MENU_HODOMETRO; emSubMenu = false; montarInterface(); }, LV_EVENT_CLICKED, NULL);
            lv_label_set_text(lv_label_create(btn_home), "HOME");
        }
    }

    // --- BOTÕES DE NAVEGAÇÃO LATERAL (fora de submenu) ---
    if (!emSubMenu) {
        lv_obj_t * bl = lv_btn_create(scr);
        lv_obj_set_size(bl, 40, 80); lv_obj_align(bl, LV_ALIGN_LEFT_MID, 0, 0); lv_obj_set_style_bg_opa(bl, LV_OPA_20, 0); lv_obj_add_event_cb(bl, nav_event_cb, LV_EVENT_CLICKED, (void*)0);
        lv_obj_t * bd = lv_btn_create(scr);
        lv_obj_set_size(bd, 40, 80); lv_obj_align(bd, LV_ALIGN_RIGHT_MID, 0, 0); lv_obj_set_style_bg_opa(bd, LV_OPA_20, 0); lv_obj_add_event_cb(bd, nav_event_cb, LV_EVENT_CLICKED, (void*)1);
    }
}

// =============================================================================
// MÓDULO D: PROCESSAMENTO DE DADOS SERIAIS DO ARDUINO MEGA
// =============================================================================
// Lê dados do buffer Serial e processa conforme o protocolo definido.
// Chamada a cada iteração do loop() (não bloqueante).
//
// PROTOCOLO DE ENTRADA (Mega → CYD):
//   "V,<B1>,<B2>,<B3>,<Amp>\n" - Telemetria de energia
//     B1/B2: tensão das baterias de tração (medidas pelo divisor 220kΩ/10kΩ)
//     B3: tensão da bateria de sistema 12V (mesmo divisor)
//     Amp: corrente de tração medida pelo ACS758LCB-050B (50A, 40mV/A)
//
//   "READY\n" - Mega completou inicialização (apenas log, não bloqueia boot)
//
//   "STANDBY,<0|1>\n" - Modo economia (0=normal, 1=standby)
//     Quando standby: desliga backlight (GPIO21 LOW) para economizar energia
//     Quando normal: liga backlight (GPIO21 HIGH)
//
//   "ALERT,<tipo>\n" - Alerta crítico (furto ou temperatura)
//     Se em standby: pisca tela uma vez (200ms) para chamar atenção
//
// PARSING: Usa strtok() + atof()/atoi() (sem classe String!).
// Buffer de 128 bytes é suficiente para qualquer mensagem do protocolo.
//
// ANÁLISE DE BUG (TIMING):
// delay(200) no handler de ALERT bloqueia o loop por 200ms.
// Durante esse tempo, o LVGL não processa toques.
// Impacto: mínimo (alerta é raro e o usuário não interage em standby).
// Sugestão futura: substituir por lógica não bloqueante com millis().
//
// ANÁLISE DE BUG (BUFFER OVERFLOW):
// readBytesUntil lê no máximo 127 bytes (sizeof-1). Seguro.
// strtok opera no próprio buffer (sem alocação dinâmica). Seguro.
// NULL checks em todos os tokens antes de atof(). Seguro.
// ---------------------------------------------------------------------------
void lerDadosUno() {
    if (SerialBT.available()) {
        char buffer[128];
        int bytesLidos = SerialBT.readBytesUntil('\n', buffer, sizeof(buffer)-1);
        buffer[bytesLidos] = '\0';
        
        // DEBUG: Log do pacote bruto recebido (cada 50 pacotes para não poluir o Serial)
        {
            static volatile unsigned long dbgContPacotes = 0;
            dbgContPacotes++;
            if (dbgContPacotes % 50 == 0) {
                Serial.printf("[DBG RX] Pacote #%lu (%d bytes): '%s'\n",
                    dbgContPacotes, bytesLidos, buffer);
            }
        }
        
        // Seção 6: Validação de checksum (se presente)
        // Formato com checksum: DADOS*XX\n onde XX = XOR hex
        char* chkPos = strchr(buffer, '*');
        if(chkPos != NULL) {
            uint8_t chkRecebido = (uint8_t)strtol(chkPos + 1, NULL, 16);
            *chkPos = '\0';  // Remove checksum do buffer para parsing
            uint8_t chkCalculado = 0;
            for(int ci = 0; buffer[ci] != '\0'; ci++) chkCalculado ^= (uint8_t)buffer[ci];
            if(chkRecebido != chkCalculado) {
                Serial.printf("[DBG CHK] Checksum INVALIDO: recv=0x%02X calc=0x%02X buf='%s'\n",
                    chkRecebido, chkCalculado, buffer);
                return;  // Descarta pacote com checksum inválido
            }
        }
        
        // =================================================================
        // PROTOCOLO v4.0 BT-Only: CSV BRUTO
        // =================================================================
        // Detecta pacote CSV bruto pelo primeiro caractere numérico.
        // Formato: <rawB1>,<rawB2>,<rawB3>,<pulsos>,<chuva>,<setas>[,<rawCorr>]\n
        // Todos os outros pacotes (alertas, comandos) começam com letra.
        // =================================================================
        if(buffer[0] >= '0' && buffer[0] <= '9') {
            // Pacote CSV bruto do Mega — actualiza watchdog
            btUltimaRecepMs = millis();   // Alimenta o BT watchdog (5s sem dados = reconectar)
            btConectado = true;            // Marca link como activo
            btReconectando = false;        // Cancela eventual flag de reconexão
            
            // Parse dos 7 campos CSV com strtok (sem String, sem sscanf)
            char buf2[128];
            strncpy(buf2, buffer, sizeof(buf2)-1);
            buf2[sizeof(buf2)-1] = '\0';
            
            char* tok = strtok(buf2, ",");
            int rawB1 = tok ? atoi(tok) : 0;   // ADC A0 – bateria B1
            
            tok = strtok(NULL, ",");
            int rawB2 = tok ? atoi(tok) : 0;   // ADC A1 – bateria B2
            
            tok = strtok(NULL, ",");
            int rawB3 = tok ? atoi(tok) : 0;   // ADC A3 – bateria B3 sistema 12V
            
            tok = strtok(NULL, ",");
            unsigned long pulsos = tok ? (unsigned long)atol(tok) : 0; // Pulsos RPM janela 500ms
            
            tok = strtok(NULL, ",");
            int chuvaRaw = tok ? atoi(tok) : 0; // 0=seco | 1=chuva

            tok = strtok(NULL, ",");
            int setasRaw = tok ? atoi(tok) : 0; // 0=off | 1=esq | 2=dir | 3=hazard

            tok = strtok(NULL, ",");
            int rawCorr = tok ? atoi(tok) : 512; // ADC A4 – corrente ACS758 (0A = ~512)
            
            // ── CONVERSÃO RAW → GRANDEZAS FÍSICAS ──────────────────────────
            // O Mega APENAS envia ADC bruto. Todo o cálculo físico é feito aqui.
            //
            // Tensão: Vbat = (raw / MEGA_ADC_MAX) × MEGA_VREF_V / MEGA_DIV_RAZAO
            //   Exemplo raw=412 (48V sistema): (412/1023)*5 / 0.04348 ≈ 46.3V
            vB1 = (rawB1 / MEGA_ADC_MAX) * MEGA_VREF_V / MEGA_DIV_RAZAO;
            vB2 = (rawB2 / MEGA_ADC_MAX) * MEGA_VREF_V / MEGA_DIV_RAZAO;
            vB3 = (rawB3 / MEGA_ADC_MAX) * MEGA_VREF_V / MEGA_DIV_RAZAO;

            // Corrente: Iadc = (Vadc - 2.5V) / 0.040 V/A
            //   Vadc = (rawCorr / 1023.0) × 5.0
            //   Corrente negativa (regeneração) é zerada — exibe só descarga
            {
                float vI = (rawCorr / MEGA_ADC_MAX) * MEGA_VREF_V;
                ampere = (vI - MEGA_ACS758_OFFSET_V) / MEGA_ACS758_SENS_VA;
                if(ampere < 0.0f) ampere = 0.0f;
            }

            // ── CÁLCULO DE RPM, VELOCIDADE E ODÔMETRO ──────────────────────
            // O Mega conta pulsos de janela em janela (500ms padrão) e envia
            // o total. O CYD recalcula RPM e velocidade usando circRodaCM.
            //
            // RPM = (pulsos / CYD_PULSOS_POR_VOLTA) / (intervalo_ms / 60000)
            //     = pulsos × 60000 / (intervalo_ms × CYD_PULSOS_POR_VOLTA)
            //
            // Velocidade (km/h) = RPM × Circunferência_m × 60 / 1000
            //   circRodaCM é salvo na NVS e pode ser ajustado pelo usuário.
            //
            // Odômetro: acumula a distância percorrida em cada janela de 500ms.
            {
                static unsigned long tUltPacote = 0;  // Timestamp do pacote anterior
                unsigned long agora = millis();
                unsigned long intervalo_ms = (tUltPacote > 0) ? (agora - tUltPacote) : 500;
                if(intervalo_ms > 2000) intervalo_ms = 500; // Limita se houve pausa longa
                tUltPacote = agora;

                if(pulsos > 0 && intervalo_ms > 0) {
                    // RPM calculado a partir dos pulsos da janela
                    float rpmCalc = (pulsos * 60000.0f) /
                                    ((float)intervalo_ms * (float)CYD_PULSOS_POR_VOLTA);
                    rpm = (int)rpmCalc;

                    // Velocidade: km/h = RPM × circunferência_em_metros × 60 / 1000
                    // circRodaCM em cm → divide por 100 para converter para metros
                    kmh = (rpmCalc * (circRodaCM / 100.0f) * 60.0f) / 1000.0f;
                    if(kmh < 0.0f) kmh = 0.0f;  // Proteção contra valor negativo
                    if(kmh > 150.0f) kmh = 0.0f; // Filtra leitura espúria (>150km/h inválido)

                    // Odômetro: distância = velocidade × tempo
                    // distKm = kmh × (intervalo_ms / 3.600.000ms/h)
                    float distKm = kmh * ((float)intervalo_ms / 3600000.0f);
                    kmTotal += distKm;  // Acumula na variável global (salva na NVS a cada 30s)
                } else {
                    // Sem pulsos nesta janela: veículo parado
                    rpm = 0;
                    kmh = 0.0f;
                }
            }

            // ── STATUS SENSORES DIGITAIS ────────────────────────────────────
            // Chuva: o Mega usa INPUT_PULLUP → 1 = chuva, 0 = seco
            // Atualiza a variável local; o alerta será exibido pelo overlay de alertas.
            alertaChuva = (chuvaRaw == 1);

            // Setas: 0=off, 1=esq, 2=dir, 3=hazard (ambas piscam = emergência)
            alertaSetaEsq = (setasRaw == 1 || setasRaw == 3);
            alertaSetaDir = (setasRaw == 2 || setasRaw == 3);
            if(setasRaw == 3) alertaEmergencia = true; // Hazard = emergência visual
            else if(setasRaw == 0) alertaEmergencia = false; // Hazard OFF = limpa flag

            // DEBUG: Loga CSV parseado a cada ~5s (10 pacotes × 500ms = 5s)
            // Permite rastrear o fluxo sensor→CSV→display no Monitor Serial do ESP32.
            {
                static unsigned long dbgCsvCount = 0;
                dbgCsvCount++;
                if (dbgCsvCount % 10 == 0) {
                    Serial.printf("[DBG CSV] #%lu rawB1=%d rawB2=%d rawB3=%d pulsos=%lu chuva=%d setas=%d rawCorr=%d\n",
                        dbgCsvCount, rawB1, rawB2, rawB3, pulsos, chuvaRaw, setasRaw, rawCorr);
                    Serial.printf("[DBG CALC] vB1=%.2fV vB2=%.2fV vB3=%.2fV A=%.2fA RPM=%d kmh=%.1f\n",
                        vB1, vB2, vB3, ampere, rpm, kmh);
                }
            }
            
            return; // Pacote CSV processado — sai sem tentar parse de texto
        }
        
        // =================================================================
        // PROTOCOLO DE TEXTO: alertas, comandos e eventos especiais
        // Apenas linhas que NÃO começam com dígito chegam aqui.
        // =================================================================
        
        // Parse usando strtok (SEM STRING!)
        char* token = strtok(buffer, ",");
        
        if(token != NULL) {
            // Protocolo de voltagem e corrente (legado v3.x — substituído por CSV bruto no v4.0)
            // Mantido para retrocompatibilidade com Mega rodando firmware anterior ao v4.0.
            if(strcmp(token, "V") == 0) {
                btUltimaRecepMs = millis(); // Pacote de texto também alimenta o watchdog BT
                btConectado = true;
                token = strtok(NULL, ",");
                if(token) vB1 = atof(token);
                
                token = strtok(NULL, ",");
                if(token) vB2 = atof(token);
                
                token = strtok(NULL, ",");
                if(token) vB3 = atof(token);
                
                token = strtok(NULL, ",");
                if(token) ampere = atof(token);  // Apenas tração (B1+B2)
            }
            // MÓDULO E: Comando de Standby
            else if(strcmp(token, "STANDBY") == 0) {
                token = strtok(NULL, ",");
                if(token) {
                    int estado = atoi(token);
                    modoStandby = (estado == 1);
                    
                    if(modoStandby) {
                        // Desliga backlight
                        digitalWrite(PIN_BACKLIGHT, LOW);
                    } else {
                        // Liga backlight
                        digitalWrite(PIN_BACKLIGHT, HIGH);
                    }
                }
            }
            // MÓDULO F: Confirmação de sistema pronto (Mega respondeu READY)
            // Nota: sistemaReady foi removido. O READY do Mega agora é apenas
            // informativo (log). O boot do CYD NÃO depende mais desta mensagem.
            // Se a splash ainda estiver ativa quando READY chegar, ela avançará
            // normalmente pelo progresso (não trava mais).
            else if(strcmp(token, "READY") == 0) {
                Serial.println("[INFO] Mega confirmou READY");
                btUltimaRecepMs = millis(); // READY também indica que a ligação BT está activa
                btConectado = true;
            }
            // Alertas críticos (furto, temperatura, etc.)
            // O Mega envia ALERT,<tipo> quando detecta condição perigosa.
            // tipos: ROUBO, TEMP, CHUVA, CORRENTE, FREIO, SUBTENSAO
            else if(strcmp(token, "ALERT") == 0) {
                alertaCritico = true;
                token = strtok(NULL, ",");
                if(token != NULL) {
                    if(strcmp(token, "ROUBO") == 0) alertaEmergencia = true;
                    else if(strcmp(token, "TEMP") == 0) alertaTemperatura = true;
                    else if(strcmp(token, "CHUVA") == 0) alertaChuva = true;
                    else if(strcmp(token, "CORRENTE") == 0) alertaCorrenteExc = true;
                    else if(strcmp(token, "FREIO") == 0) alertaFalhaFreio = true;
                    else if(strcmp(token, "SUBTENSAO") == 0) alertaSubtensao = true;
                }
                
                // Se em standby, pisca tela uma vez (não bloqueante no futuro)
                if(modoStandby) {
                    digitalWrite(PIN_BACKLIGHT, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    digitalWrite(PIN_BACKLIGHT, LOW);
                }
            }
            // FAROL: Status do botão físico de farol no Mega
            // Formato: FAROL,<0|1>\n
            // 0 = botão desligado (LDR controla), 1 = botão ligado (farol forçado)
            else if(strcmp(token, "FAROL") == 0) {
                token = strtok(NULL, ",");
                if(token) farolManualLigado = (atoi(token) == 1);
            }
            // SETA: Setas de direção ativadas pelo Mega
            // Formato: SETA,<ESQ|DIR|AMBAS|OFF>\n
            else if(strcmp(token, "SETA") == 0) {
                token = strtok(NULL, ",");
                if(token) {
                    if(strcmp(token, "ESQ") == 0) { alertaSetaEsq = true; alertaSetaDir = false; }
                    else if(strcmp(token, "DIR") == 0) { alertaSetaEsq = false; alertaSetaDir = true; }
                    else if(strcmp(token, "AMBAS") == 0) { alertaSetaEsq = true; alertaSetaDir = true; alertaEmergencia = true; }
                    else { alertaSetaEsq = false; alertaSetaDir = false; alertaEmergencia = false; }
                }
            }
            // CRUISE: Status do cruise control
            // Formato: CRUISE,<ON|OFF>,<velocidade>\n
            else if(strcmp(token, "CRUISE") == 0) {
                token = strtok(NULL, ",");
                if(token) {
                    if(strcmp(token, "ON") == 0) {
                        alertaCruiseOn = true;
                    } else {
                        alertaCruiseOn = false;
                    }
                }
            }
            // CLEAR: Limpa um alerta específico
            // Formato: CLEAR,<tipo>\n
            else if(strcmp(token, "CLEAR") == 0) {
                token = strtok(NULL, ",");
                if(token) {
                    if(strcmp(token, "ROUBO") == 0) alertaEmergencia = false;
                    else if(strcmp(token, "TEMP") == 0) alertaTemperatura = false;
                    else if(strcmp(token, "CHUVA") == 0) alertaChuva = false;
                    else if(strcmp(token, "CORRENTE") == 0) alertaCorrenteExc = false;
                    else if(strcmp(token, "FREIO") == 0) alertaFalhaFreio = false;
                    else if(strcmp(token, "SUBTENSAO") == 0) alertaSubtensao = false;
                    else if(strcmp(token, "ALL") == 0) {
                        // Limpa TODOS os alertas (usado pelo RESET_ALARM)
                        alertaEmergencia = false; alertaTemperatura = false;
                        alertaChuva = false; alertaCorrenteExc = false;
                        alertaFalhaFreio = false; alertaSubtensao = false;
                        alertaSetaEsq = false; alertaSetaDir = false;
                        alertaCruiseOn = false; alertaBatCritica = false;
                        alertaCritico = false;
                    }
                }
            }
            // RESET_ALARM: Comando recebido do Mega para limpar TODOS os alertas
            // Isto é enviado quando o utilizador sai do menu de teste no CYD,
            // ou quando o Mega quer forçar limpeza de alertas em todos os ecrãs.
            // Equivale a CLEAR,ALL mas é um comando dedicado para clareza.
            else if(strcmp(token, "RESET_ALARM") == 0) {
                alertaEmergencia = false; alertaTemperatura = false;
                alertaChuva = false; alertaCorrenteExc = false;
                alertaFalhaFreio = false; alertaSubtensao = false;
                alertaSetaEsq = false; alertaSetaDir = false;
                alertaCruiseOn = false; alertaBatCritica = false;
                alertaCritico = false;
                modoTesteAtivo = false;
            }
            // Formato: KEYOFF\n
            // O CYD inicia o timer de 20s e mostra diálogo de keep-alive
            else if(strcmp(token, "KEYOFF") == 0) {
                if(!sistemaKeepAlive && !timerKeyOffAtivo) {
                    chaveDesligadaDetectada = true;
                    timerKeyOffAtivo = true;
                    timerKeyOffInicio = millis();
                }
            }
            // KEYON: Mega informa que a chave de 48V foi religada
            // Formato: KEYON\n
            // Cancela o timer de 20s e fecha o diálogo
            else if(strcmp(token, "KEYON") == 0) {
                chaveDesligadaDetectada = false;
                timerKeyOffAtivo = false;
                sistemaKeepAlive = false;
                dialogoKeyOffVisivel = false;
                if(dialogoKeyOffObj != NULL) {
                    lblTimerKeyOff = NULL;
                    lv_obj_del(dialogoKeyOffObj);
                    dialogoKeyOffObj = NULL;
                }
            }
            // RADAR: Distância dos dois sensores ultrassônicos traseiros
            // Formato: RADAR,<esq>,<dir>\n
            // O Mega envia a cada 100ms quando R4 (marcha ré) está ativa.
            // Quando R4 é desligada, envia RADAR,0,0 uma vez.
            // O CYD exibe visual automotivo com dois arcos na tela RADAR_PROXIMIDADE.
            else if(strcmp(token, "RADAR") == 0) {
                token = strtok(NULL, ",");
                if(token) {
                    radarDistanciaEsq = atoi(token);
                    token = strtok(NULL, ",");
                    if(token) radarDistanciaDir = atoi(token);
                    radarUltimaLeitura = millis();
                    
                    // Se ambas distâncias == 0, R4 foi desligada
                    bool reAtiva = (radarDistanciaEsq > 0 || radarDistanciaDir > 0);
                    radarAtivo = reAtiva;
                    
                    // ==========================================================
                    // RADAR AUTOMÁTICO: Troca de página ao engatar/desengatar ré
                    // ==========================================================
                    // DUAL-CORE: Esta função roda no Core 0 (tarefa de telemetria).
                    // montarInterface() manipula LVGL e SPI — só pode rodar no Core 1.
                    // Usamos a flag pendingMontarInterface para sinalizar ao Core 1.
                    if(radarHabilitado) {
                        if(reAtiva && !modoReAnterior && !radarAutoAtivado) {
                            paginaAnteriorRadar = paginaAtual;
                            paginaAtual = RADAR_PROXIMIDADE;
                            emSubMenu = true;
                            radarAutoAtivado = true;
                            pendingMontarInterface = true;  // Core 1 irá chamar montarInterface()
                        }
                        else if(!reAtiva && modoReAnterior && radarAutoAtivado) {
                            paginaAtual = paginaAnteriorRadar;
                            // Páginas que são nível raiz (não têm submenu)
                            emSubMenu = (paginaAnteriorRadar != MENU_HODOMETRO &&
                                         paginaAnteriorRadar != PRINCIPAL &&
                                         paginaAnteriorRadar != RE &&
                                         paginaAnteriorRadar != LIMITADOR &&
                                         paginaAnteriorRadar != VELOCIDADE &&
                                         paginaAnteriorRadar != MOTOR &&
                                         paginaAnteriorRadar != VOLTAGEM &&
                                         paginaAnteriorRadar != KM &&
                                         paginaAnteriorRadar != BATERIA &&
                                         paginaAnteriorRadar != RODA &&
                                         paginaAnteriorRadar != SOLAR &&
                                         paginaAnteriorRadar != ACIONAR &&
                                         paginaAnteriorRadar != CONDUCAO_MENU &&
                                         paginaAnteriorRadar != MANUTENCAO_MENU &&
                                         paginaAnteriorRadar != PAREAMENTO_BT);
                            radarAutoAtivado = false;
                            pendingMontarInterface = true;  // Core 1 irá chamar montarInterface()
                        }
                    }
                    modoReAnterior = reAtiva;
                }
            }
            // SPD: Velocidade, RPM e odometro do Mega
            // Formato: SPD,<rpm>,<velocidade_kmh>,<odometro_metros>\n
            else if(strcmp(token, "SPD") == 0) {
                token = strtok(NULL, ",");
                if(token) rpm = atoi(token);
                
                token = strtok(NULL, ",");
                if(token) kmh = atof(token);
                
                token = strtok(NULL, ",");
                if(token) {
                    float odoMetros = atof(token);
                    // Converte metros para km e acumula
                    float novoKmTotal = odoMetros / 1000.0;
                    if(novoKmTotal > kmTotal) {
                        float delta = novoKmTotal - kmTotal;
                        kmParcial += delta;
                        kmTotal = novoKmTotal;
                    }
                }
            }
            // TELA: Comando do botão físico no Mega para ligar/desligar backlight
            // Formato: TELA,<0|1>\n  (0=desliga, 1=liga)
            // O Mega envia este comando quando o botão físico é pressionado/solto.
            else if(strcmp(token, "TELA") == 0) {
                token = strtok(NULL, ",");
                if(token) {
                    int estado = atoi(token);
                    if(estado == 0) {
                        digitalWrite(PIN_BACKLIGHT, LOW);   // Desliga backlight
                        modoStandby = true;
                    } else {
                        digitalWrite(PIN_BACKLIGHT, HIGH);  // Liga backlight
                        modoStandby = false;
                    }
                }
            }
        }
    }
}

// =============================================================================
// SISTEMA DE ALERTAS - OVERLAY TRANSPARENTE (lv_layer_top)
// =============================================================================
// Desenha alertas em lv_layer_top() para serem visíveis em QUALQUER tela.
// Fundo semi-transparente (bg_opa 180), texto/ícone centralizado.
// Chamada a cada iteração do loop() - verifica flags e atualiza overlay.
//
// COMO FUNCIONA:
//   1. Verifica todas as flags de alerta (alertaSetaEsq, alertaChuva, etc.)
//   2. Se alguma flag ativa, cria/atualiza container no lv_layer_top()
//   3. O overlay é semi-transparente e não bloqueia interação
//   4. Setas e emergência piscam a cada 500ms
//   5. Se nenhuma flag ativa, remove o overlay
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// atualizarOverlayAlertas()
// ---------------------------------------------------------------------------
// O QUE É: Gerencia o overlay transparente de alertas no lv_layer_top().
// O QUE NÃO É: Não controla relés ou hardware - apenas exibe informações.
// LÓGICA:
//   1. PRIORIDADE TOTAL: Setas (L/R) e Emergência ficam FIXAS, sem ciclar
//   2. CARROSSEL: Outros alarmes (bateria, corrente, temp, chuva, freio)
//      alternam a cada 3 segundos usando máquina de estados com millis()
//   3. O overlay é semi-transparente e não bloqueia interação com a tela
// ---------------------------------------------------------------------------
void atualizarOverlayAlertas() {
    // Verifica se há algum alerta ativo
    bool temAlerta = alertaSetaEsq || alertaSetaDir || alertaEmergencia ||
                     alertaChuva || alertaTemperatura || alertaCorrenteExc ||
                     alertaFalhaFreio || alertaBatCritica || alertaSubtensao ||
                     alertaCruiseOn;
    
    // Pisca setas a cada 500ms
    if(millis() - alertaPiscaTimer > 500) {
        alertaPiscaEstado = !alertaPiscaEstado;
        alertaPiscaTimer = millis();
    }
    
    lv_obj_t * layer = lv_layer_top();
    
    if(!temAlerta) {
        if(overlayAlerta != NULL) {
            lv_obj_del(overlayAlerta);
            overlayAlerta = NULL;
            overlayLabel = NULL;
        }
        indexAlertaAtual = 0;
        return;
    }
    
    // Cria overlay se não existir
    // LAYOUT FIX: Overlay tem 280px de largura (320 - 2*20 margem lateral)
    // Posicionado em TOP_MID com y=8 para margem de respiro do topo.
    // Altura 48px para não cobrir os botões de navegação (BOTTOM, y=-15).
    // O overlay NÃO bloqueia cliques na tela debaixo (CLICKABLE desativado).
    if(overlayAlerta == NULL) {
        overlayAlerta = lv_obj_create(layer);
        lv_obj_set_size(overlayAlerta, 280, 48);
        lv_obj_align(overlayAlerta, LV_ALIGN_TOP_MID, 0, 8);
        lv_obj_set_style_bg_color(overlayAlerta, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(overlayAlerta, 180, 0);
        lv_obj_set_style_border_color(overlayAlerta, lv_color_hex(0xFF0000), 0);
        lv_obj_set_style_border_width(overlayAlerta, 2, 0);
        lv_obj_set_style_radius(overlayAlerta, 8, 0);
        lv_obj_set_style_pad_all(overlayAlerta, 4, 0);
        lv_obj_clear_flag(overlayAlerta, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(overlayAlerta, LV_OBJ_FLAG_CLICKABLE);
        
        overlayLabel = lv_label_create(overlayAlerta);
        lv_obj_set_style_text_color(overlayLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_align(overlayLabel, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(overlayLabel, 268);  // 280 - 2*6 padding
        lv_obj_center(overlayLabel);
        lv_label_set_recolor(overlayLabel, true);
    }
    
    char alertaBuf[128] = "";
    
    // =====================================================================
    // REGRA DE PRIORIDADE MÁXIMA: Setas e Emergência NÃO entram no ciclo
    // Ficam FIXAS enquanto estiverem ativas (segurança viária)
    // =====================================================================
    if(alertaEmergencia) {
        if(alertaPiscaEstado) {
            snprintf(alertaBuf, sizeof(alertaBuf), "#FF0000 " LV_SYMBOL_WARNING " EMERGENCIA " LV_SYMBOL_WARNING "#");
        } else {
            alertaBuf[0] = '\0';
        }
        lv_obj_set_style_bg_opa(overlayAlerta, alertaPiscaEstado ? 200 : 80, 0);
    }
    else if(alertaSetaEsq || alertaSetaDir) {
        // Setas têm prioridade: ficam fixas (piscando)
        if(alertaSetaEsq && alertaSetaDir) {
            if(alertaPiscaEstado)
                snprintf(alertaBuf, sizeof(alertaBuf), "#FFAA00 " LV_SYMBOL_LEFT " ALERTA " LV_SYMBOL_RIGHT "#");
        }
        else if(alertaSetaEsq && alertaPiscaEstado) {
            snprintf(alertaBuf, sizeof(alertaBuf), "#FFAA00 " LV_SYMBOL_LEFT " SETA ESQUERDA#");
        }
        else if(alertaSetaDir && alertaPiscaEstado) {
            snprintf(alertaBuf, sizeof(alertaBuf), "#FFAA00 SETA DIREITA " LV_SYMBOL_RIGHT "#");
        }
        lv_obj_set_style_bg_opa(overlayAlerta, alertaPiscaEstado ? 200 : 80, 0);
    }
    else {
        // =====================================================================
        // CARROSSEL DE ALARMES: Alterna a cada 3 segundos entre alarmes ativos
        // =====================================================================
        // Monta array dos alarmes secundários que estão ativos
        struct AlertaInfo { bool ativo; const char* texto; };
        AlertaInfo filaAlertas[] = {
            {alertaFalhaFreio,  "#FF0000 " LV_SYMBOL_CLOSE " FALHA DE FREIO!#"},
            {alertaCorrenteExc, "#FF4400 " LV_SYMBOL_CHARGE " CORRENTE EXCESSIVA!#"},
            {alertaTemperatura, "#FF4400 " LV_SYMBOL_WARNING " TEMPERATURA ALTA!#"},
            {alertaBatCritica,  "#FF0000 " LV_SYMBOL_BATTERY_EMPTY " BATERIA CRITICA!#"},
            {alertaSubtensao,   "#FF4400 " LV_SYMBOL_BATTERY_1 " SUBTENSAO!#"},
            {alertaChuva,       "#00AAFF " LV_SYMBOL_WARNING " PISO MOLHADO#"},
            {alertaCruiseOn,    "#00FF00 " LV_SYMBOL_GPS " CRUISE ATIVO#"}
        };
        const int totalTiposAlerta = 7;
        
        // Conta quantos estão ativos
        int ativosCount = 0;
        int ativosIdx[7];
        for(int i = 0; i < totalTiposAlerta; i++) {
            if(filaAlertas[i].ativo) {
                ativosIdx[ativosCount] = i;
                ativosCount++;
            }
        }
        
        if(ativosCount > 0) {
            // Cicla a cada ALERTA_CICLO_MS (3 segundos)
            if(millis() - ultimoTempoTroca >= ALERTA_CICLO_MS) {
                indexAlertaAtual = (indexAlertaAtual + 1) % ativosCount;
                ultimoTempoTroca = millis();
            }
            // Garante que o índice está dentro do range
            if(indexAlertaAtual >= ativosCount) indexAlertaAtual = 0;
            
            snprintf(alertaBuf, sizeof(alertaBuf), "%s", filaAlertas[ativosIdx[indexAlertaAtual]].texto);
        }
        lv_obj_set_style_bg_opa(overlayAlerta, 180, 0);
    }
    
    if(overlayLabel != NULL) {
        lv_label_set_text(overlayLabel, alertaBuf);
    }
}

// atualizarRelogio() e carregarConfigsDateTime() REMOVIDOS (Seção 1: sem RTC)

// Carrega agenda de manutenção da NVS (Seção 1: apenas KM)
void carregarAgendaManutencao() {
    Preferences prefs;
    prefs.begin("manut", true);
    manutKmAlvo = prefs.getFloat("kmAlvo", 0);
    prefs.end();
}

// Verifica se é hora de manutenção (apenas por KM - Seção 1: data removida)
void verificarAgendaManutencao() {
    if(manutAlertaMostrado) return;
    
    bool alertar = false;
    
    // Verifica apenas por km (Seção 1: agendamento por data removido)
    if(manutKmAlvo > 0 && kmTotal >= manutKmAlvo) {
        alertar = true;
    }
    
    if(alertar) {
        manutAlertaMostrado = true;
        // Mostra alerta overlay de manutenção
        lv_obj_t * layer = lv_layer_top();
        lv_obj_t * alert = lv_obj_create(layer);
        lv_obj_set_size(alert, 300, 80);
        lv_obj_align(alert, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(alert, lv_color_hex(0x002266), 0);
        lv_obj_set_style_bg_opa(alert, 220, 0);
        lv_obj_set_style_border_color(alert, lv_color_hex(0x00AAFF), 0);
        lv_obj_set_style_border_width(alert, 2, 0);
        lv_obj_set_style_radius(alert, 10, 0);
        
        lv_obj_t * lbl = lv_label_create(alert);
        lv_label_set_text(lbl, LV_SYMBOL_BELL " LEMBRETE:\nRealizar Manutencao Programada!");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(lbl);
        
        // Botão para fechar o alerta
        lv_obj_t * btnFechar = lv_btn_create(alert);
        lv_obj_set_size(btnFechar, 40, 25);
        lv_obj_align(btnFechar, LV_ALIGN_TOP_RIGHT, -5, 5);
        lv_obj_set_style_bg_color(btnFechar, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_add_event_cb(btnFechar, [](lv_event_t*e){
            lv_obj_t * parent = lv_obj_get_parent((lv_obj_t*)lv_event_get_target(e));
            lv_obj_del(parent);
        }, LV_EVENT_CLICKED, NULL);
        lv_label_set_text(lv_label_create(btnFechar), LV_SYMBOL_CLOSE);
    }
}

// =============================================================================
// STANDBY 10 MINUTOS + FAROL HÍBRIDO (LDR + BOTÃO MANUAL)
// =============================================================================
// STANDBY: Se velocidade == 0 por 10 minutos, desliga periféricos e R15 (farol).
// Reset: Se botão manual do farol for toggleado, timer reinicia.
// Se velocidade > 0, timer zera automaticamente.
//
// FAROL HÍBRIDO:
// 1. Sensor LDR no ESP32 CYD (GPIO36): detecta escuro/claro
// 2. Botão físico no Mega (pino digital): prioridade TOTAL
//    Se botão LIGADO → R15 ON independente do LDR
//    Se botão DESLIGADO → LDR decide (escuro=ON, claro=OFF)
// 3. Status do farol atualizado via serial CSV do Mega
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// processarStandby()
// ---------------------------------------------------------------------------
// O QUE É: Gerencia o modo standby (desligamento da tela) do veículo.
// O QUE NÃO É: NÃO desliga automaticamente por tempo. O desligamento é
//   feito APENAS pelo botão físico no Mega (comando TELA,0/1 via Serial).
// LÓGICA:
//   1. Se velocidade > 0 e estava em standby → acorda automaticamente
//   2. Se toque detectado enquanto tela apagada → acorda (wake-on-touch)
//   3. NÃO há timer de inatividade (removido conforme solicitação)
//
// ! PERIGO ! O standby NÃO desliga os motores ou freios.
// Apenas desliga o backlight e periféricos não essenciais.
// ---------------------------------------------------------------------------
void processarStandby() {
    // Wake-on-motion: se velocidade > 0, sai do standby automaticamente
    if(kmh > 0.5 && modoStandby) {
        modoStandby = false;
        digitalWrite(PIN_BACKLIGHT, HIGH);  // Pino 21: Liga backlight
        SerialBT.print("STANDBY,0\n");       // Informa Mega: sair de standby
    }
    
    // Wake-on-touch: detecta toque enquanto tela apagada (Seção 2)
    // Se pino 21 LOW (backlight desligado), qualquer toque reativa HIGH imediatamente
    // SEM processar o clique no botão debaixo (prevenção de cliques acidentais)
    if(modoStandby) {
        lv_indev_t * indev = lv_indev_get_next(NULL);
        if(indev != NULL) {
            lv_point_t point;
            lv_indev_get_point(indev, &point);
            if(lv_indev_get_state(indev) == LV_INDEV_STATE_PRESSED) {
                modoStandby = false;
                wakeupTouchBlock = true;               // Seção 2: bloqueia clique acidental
                wakeupTouchBlockTime = millis();        // Seção 2: timer anti-click
                digitalWrite(PIN_BACKLIGHT, HIGH);      // Pino 21: Liga backlight
                SerialBT.print("STANDBY,0\n");
            }
        }
    }
    
    // NOTA: NÃO há timer automático de standby.
    // O desligamento é feito APENAS pelo botão físico no Mega
    // (comando Serial "TELA,0" para desligar, "TELA,1" para ligar).
}

// ---------------------------------------------------------------------------
// toggleStandbyManual()
// ---------------------------------------------------------------------------
// Inverte o estado do standby manualmente.
// NOTA: O botão virtual foi removido da interface. Esta função é mantida
// para uso interno (ex: wake-on-motion). O controle principal de tela
// agora é feito pelo botão físico no Mega (comando TELA,0/1 via Serial).
// ---------------------------------------------------------------------------
void toggleStandbyManual() {
    modoStandby = !modoStandby;
    if(modoStandby) {
        // Desliga periféricos
        enviarComandoRele(15, 0);  // R15: Farol OFF (pino 36 do Mega)
        enviarComandoRele(7, 0);   // R7: Luz interna OFF (pino 28 do Mega)
        enviarComandoRele(10, 0);  // R10: Ventilador OFF (pino 31 do Mega)
        digitalWrite(PIN_BACKLIGHT, LOW);   // Pino 21: Tela OFF
        SerialBT.print("STANDBY,1\n");       // Informa Mega
    } else {
        digitalWrite(PIN_BACKLIGHT, HIGH);  // Pino 21: Tela ON
        SerialBT.print("STANDBY,0\n");       // Informa Mega
    }
}

// =============================================================================
// TIMER 20s POS-CHAVE (KEY-OFF) - Diálogo de keep-alive
// =============================================================================
// O QUE FAZ: Quando o Mega detecta que a chave de 48V foi desligada (tensão B1
//   cai abaixo do limiar), envia KEYOFF ao CYD. O CYD mostra um diálogo LVGL
//   perguntando "Deseja manter o sistema ligado?" com botões SIM e NÃO.
//   - SIM: sistema fica ativo (sistemaKeepAlive=true), botão DESLIGAR MANUAL
//     aparece no menu principal.
//   - NÃO ou timeout 20s: envia STANDBY,1 ao Mega e desliga backlight.
// NOTA: A detecção da tensão é feita pelo Mega (divisor 220k/10k no pino A0).
//   O CYD apenas reage ao comando KEYOFF/KEYON recebido pela Serial.
// =============================================================================
void processarTimerKeyOff() {
    // Só processa se o timer estiver ativo
    if(!timerKeyOffAtivo) return;
    
    // Se o diálogo ainda não foi criado, cria agora
    if(!dialogoKeyOffVisivel) {
        dialogoKeyOffVisivel = true;
        
        // Cria diálogo overlay no layer_top (visível em qualquer tela)
        lv_obj_t * layer = lv_layer_top();
        dialogoKeyOffObj = lv_obj_create(layer);
        lv_obj_set_size(dialogoKeyOffObj, 280, 140);
        lv_obj_align(dialogoKeyOffObj, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(dialogoKeyOffObj, lv_color_hex(0x001133), 0);
        lv_obj_set_style_bg_opa(dialogoKeyOffObj, 240, 0);
        lv_obj_set_style_border_color(dialogoKeyOffObj, lv_color_hex(0xFF8800), 0);
        lv_obj_set_style_border_width(dialogoKeyOffObj, 3, 0);
        lv_obj_set_style_radius(dialogoKeyOffObj, 12, 0);
        lv_obj_set_style_pad_all(dialogoKeyOffObj, 10, 0);
        lv_obj_clear_flag(dialogoKeyOffObj, LV_OBJ_FLAG_SCROLLABLE);
        
        // Título do diálogo
        lv_obj_t * lblTitulo = lv_label_create(dialogoKeyOffObj);
        lv_label_set_text(lblTitulo, LV_SYMBOL_POWER " CHAVE DESLIGADA");
        lv_obj_set_style_text_color(lblTitulo, lv_color_hex(0xFF8800), 0);
        lv_obj_set_style_text_font(lblTitulo, &lv_font_montserrat_16, 0);
        lv_obj_align(lblTitulo, LV_ALIGN_TOP_MID, 0, 2);
        
        // Mensagem
        lv_obj_t * lblMsg = lv_label_create(dialogoKeyOffObj);
        lv_label_set_text(lblMsg, "Manter sistema ligado?");
        lv_obj_set_style_text_color(lblMsg, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_align(lblMsg, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(lblMsg, LV_ALIGN_TOP_MID, 0, 28);
        
        // Timer visual (conta regressiva) — ponteiro global para atualização eficiente
        lblTimerKeyOff = lv_label_create(dialogoKeyOffObj);
        lv_label_set_text(lblTimerKeyOff, "Desliga em: 20s");
        lv_obj_set_style_text_color(lblTimerKeyOff, lv_color_hex(0xAAAA00), 0);
        lv_obj_align(lblTimerKeyOff, LV_ALIGN_TOP_MID, 0, 50);
        
        // Botão SIM (verde) - mantém sistema ligado
        lv_obj_t * btnSim = lv_btn_create(dialogoKeyOffObj);
        lv_obj_set_size(btnSim, 110, 40);
        lv_obj_align(btnSim, LV_ALIGN_BOTTOM_LEFT, 5, -5);
        lv_obj_set_style_bg_color(btnSim, lv_color_hex(0x006600), 0);
        lv_obj_add_event_cb(btnSim, [](lv_event_t*e){
            // Utilizador escolheu SIM → manter sistema ligado
            sistemaKeepAlive = true;
            timerKeyOffAtivo = false;
            dialogoKeyOffVisivel = false;
            if(dialogoKeyOffObj != NULL) {
                lblTimerKeyOff = NULL;
                lv_obj_del(dialogoKeyOffObj);
                dialogoKeyOffObj = NULL;
            }
            // Botão DESLIGAR MANUAL aparecerá no menu principal
            // (verificado em montarInterface → PRINCIPAL)
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_t * lblS = lv_label_create(btnSim);
        lv_label_set_text(lblS, LV_SYMBOL_OK " SIM");
        lv_obj_center(lblS);
        
        // Botão NAO (vermelho) - desliga sistema
        lv_obj_t * btnNao = lv_btn_create(dialogoKeyOffObj);
        lv_obj_set_size(btnNao, 110, 40);
        lv_obj_align(btnNao, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
        lv_obj_set_style_bg_color(btnNao, lv_color_hex(0x660000), 0);
        lv_obj_add_event_cb(btnNao, [](lv_event_t*e){
            // Utilizador escolheu NÃO → desligar sistema
            timerKeyOffAtivo = false;
            dialogoKeyOffVisivel = false;
            sistemaKeepAlive = false;
            if(dialogoKeyOffObj != NULL) {
                lblTimerKeyOff = NULL;
                lv_obj_del(dialogoKeyOffObj);
                dialogoKeyOffObj = NULL;
            }
            // Envia comando de standby ao Mega
            SerialBT.print("STANDBY,1\n");
            modoStandby = true;
            digitalWrite(PIN_BACKLIGHT, LOW);
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_t * lblN = lv_label_create(btnNao);
        lv_label_set_text(lblN, LV_SYMBOL_CLOSE " NAO");
        lv_obj_center(lblN);
    }
    
    // Atualiza o timer visual (conta regressiva)
    if(dialogoKeyOffObj != NULL) {
        unsigned long elapsed = millis() - timerKeyOffInicio;
        int restante = (int)((KEYOFF_TIMEOUT_MS - elapsed) / 1000);
        if(restante < 0) restante = 0;
        
        // Atualiza label do timer usando ponteiro global (seguro, não depende de índice)
        if(lblTimerKeyOff != NULL) {
            char buf[32];
            snprintf(buf, sizeof(buf), "Desliga em: %ds", restante);
            lv_label_set_text(lblTimerKeyOff, buf);
        }
        
        // Timeout: se passou 20s sem resposta, desliga automaticamente
        if(elapsed >= KEYOFF_TIMEOUT_MS) {
            timerKeyOffAtivo = false;
            dialogoKeyOffVisivel = false;
            sistemaKeepAlive = false;
            lblTimerKeyOff = NULL;  // Ponteiro global deve ser zerado junto com o obj
            lv_obj_del(dialogoKeyOffObj);
            dialogoKeyOffObj = NULL;
            // Desliga sistema
            SerialBT.print("STANDBY,1\n");
            modoStandby = true;
            digitalWrite(PIN_BACKLIGHT, LOW);
        }
    }
}

// =============================================================================
// desligarManual() - Chamada pelo botão DESLIGAR MANUAL no menu principal
// =============================================================================
// O QUE FAZ: Quando o utilizador manteve o sistema ligado (SIM no diálogo),
//   aparece um botão DESLIGAR MANUAL no menu. Ao clicar, esta função é chamada
//   para desligar o sistema de forma limpa.
// =============================================================================
void desligarManual() {
    sistemaKeepAlive = false;
    chaveDesligadaDetectada = false;
    timerKeyOffAtivo = false;
    SerialBT.print("STANDBY,1\n");
    modoStandby = true;
    digitalWrite(PIN_BACKLIGHT, LOW);
}

void processarFarolHibrido() {
    // Prioridade 1: Botão físico do Mega
    // farolManualLigado é atualizado via serial (FAROL,<0|1>)
    if(farolManualLigado) {
        // Botão manual LIGADO: R15 sempre ON, independente do LDR
        enviarComandoRele(15, 1);
        return;
    }
    
    // Prioridade 2: Sensor LDR (GPIO36)
    // LDR + resistor pull-down: mais luz = mais tensão
    // Limiar: < 1000 (de 0-4095) = escuro
    int ldrValor = analogRead(PIN_LDR);
    bool escuro = (ldrValor < 1000);
    
    if(escuro && !farolLDRAtivo) {
        farolLDRAtivo = true;
        enviarComandoRele(15, 1);  // Liga farol
    }
    else if(!escuro && farolLDRAtivo) {
        farolLDRAtivo = false;
        enviarComandoRele(15, 0);  // Desliga farol
    }
}

// =============================================================================
// ANÁLISE DE BUGS E RISCOS (VARREDURA COMPLETA DO CÓDIGO)
// =============================================================================
//
// 1. MEMORY LEAKS (LVGL):
//    RISCO: Nenhum identificado.
//    - Todos os objetos LVGL são filhos da tela (scr) ou de containers.
//    - lv_obj_clean(scr) dentro de montarInterface() destrói toda a árvore.
//    - O LVGL usa pool interno de memória — não há malloc/free manual.
//    - Ponteiros globais (splashBar, label_baterias, etc) são recriados
//      a cada chamada de montarInterface() ou setados NULL após limpeza.
//    - VERIFICADO: Nenhum lv_obj_create() sem parent (seria leak).
//    - VERIFICADO: Nenhum lv_mem_alloc() manual no código.
//
// 2. TIMING SERIAL (CONFLITOS):
//    RISCO BAIXO: delay(200) no handler de ALERT bloqueia loop.
//    - Durante os 200ms, o LVGL não processa toques nem animações.
//    - Impacto: mínimo (alerta é raro, standby = sem interação do usuário).
//    - SUGESTÃO: Substituir por lógica millis() não bloqueante no futuro.
//    - enviarStatusReles() envia 5-6 comandos sequenciais (~5ms total).
//      A 115200 baud, buffer do Mega (64 bytes) absorve sem problema.
//    - readBytesUntil() tem timeout padrão de 1000ms (Serial.setTimeout).
//      Se o Mega não enviar \n, o CYD bloqueia por 1 segundo.
//      RISCO: Se o Mega travar, o CYD fica lento (mas não trava).
//      CORREÇÃO POSSÍVEL: Serial.setTimeout(100) no setup.
//
// 3. LOOPS INFINITOS:
//    RISCO: Nenhum identificado.
//    - O loop() tem vTaskDelay(5ms) que garante ~200 iterações/segundo.
//    - processarTesteAutomatico() termina após R15 (15 iterações de 1s).
//    - A splash screen termina após splashProgress >= 100 (~2 segundos).
//    - Nenhum while() ou for() sem condição de saída no código.
//
// 4. PONTEIROS NULL (CRASH):
//    BUG CORRIGIDO: atualizarDadosPrincipais() era chamada durante a splash,
//    quando label_baterias/label_dados/etc ainda não existiam (= NULL).
//    lv_label_set_text_fmt() com ponteiro NULL → crash do ESP32.
//    FIX: Adicionada guarda (!splashScreenAtiva) antes do bloco de 500ms.
//
// 5. BOOT TRAVADO:
//    BUG CORRIGIDO: O boot dependia de sistemaReady (resposta READY do Mega).
//    Se o Mega não respondesse (cabo desconectado, Mega travado), o CYD
//    ficava preso na splash screen indefinidamente.
//    FIX: sistemaReady removida. Boot avança quando splashProgress >= 100.
//
// 6. SPLASH PISCANDO:
//    BUG CORRIGIDO: montarSplashScreen() era chamada a cada iteração do loop,
//    causando lv_obj_clean() + recriação de todos os objetos a cada frame.
//    Resultado: tela branca piscando e botão PULAR não responsivo (destruído
//    antes do LVGL processar o evento de clique).
//    FIX: montarSplashScreen() chamada UMA VEZ no setup(). O loop() apenas
//    atualiza o valor de splashBar via ponteiro.
//
// 7. DESGASTE DA FLASH NVS:
//    RISCO CONTROLADO: Salvamentos periódicos (30s/60s) ao invés de contínuos.
//    Com 100.000 ciclos de escrita por setor NVS:
//    - A cada 30s = 2880 escritas/dia = ~35 dias de uso contínuo 24h por setor.
//    - O NVS do ESP32 faz wear leveling interno, distribuindo entre setores.
//    - Na prática: anos de vida útil em uso normal (veículo ~8h/dia).
//
// 8. BUFFER OVERFLOW SERIAL:
//    RISCO: Nenhum. readBytesUntil lê no máximo sizeof(buffer)-1 = 127 bytes.
//    strtok opera no próprio buffer stack (sem alocação dinâmica).
//    Todos os snprintf usam sizeof(cmd) para limitar escrita.
//
// =============================================================================

// =============================================================================
// SETUP - INICIALIZAÇÃO DO SISTEMA
// =============================================================================
// Ordem de inicialização (IMPORTANTE para segurança):
//   1. Serial (comunicação com Mega) - deve ser a primeira
//   1b. SPI Mutex - ANTES do LVGL (proteção do barramento)
//   2. LVGL/Display - inicializa o driver gráfico e touch
//   3. ADC - resolução de 12 bits para leituras analógicas
//   4. Pinos GPIO - relés solares e backlight em estado seguro
//   5. Carregar configs da NVS - restaura configurações salvas
//   6. Bluetooth A2DP Sink + AVRCP
//   7. Splash Screen - tela visual enquanto o sistema carrega
//   8. DUAL-CORE: Cria tarefa de telemetria no Core 0 (FreeRTOS)
//
// NOTA: O REBOOT_CLEAN NÃO é mais enviado automaticamente no boot.
// A verificação de integridade foi movida para o menu Manutenção.
// Motivo: o envio no boot bloqueava o sistema se o Mega não respondesse
// READY a tempo (ou nunca respondesse, ex: cabo desconectado).
// =============================================================================
void setup() {
    // 1. Inicia comunicação serial USB para debug (Monitor Serial)
    Serial.begin(115200);

    // 1b. Inicia Bluetooth SPP em modo MASTER (v4.1 HC-06 Master).
    // O ESP32 CYD agora é o MASTER e inicia a conexão com o HC-06 (Slave) no Mega.
    //
    // FLUXO DE CONEXÃO:
    //   1. carregarConfiguracoes() já restaurou btTargetMac (MAC do HC-06 ou BT_DEFAULT_MAC)
    //   2. btTargetMacSaved é sempre true (MAC padrão hardcoded como fallback)
    //   3. pendingBtConnect agendado → Core 0 executa SerialBT.connect(mac)
    //   4. BT watchdog (3s) monitora continuidade e tenta reconexão automática
    //
    // MITIGAÇÃO ASSERT_WARN (ESP-IDF BT stack):
    //   O ASSERT_WARN ocorre quando a stack BT recebe comandos antes de estar pronta.
    //   Solução: delay de 600ms após SerialBT.begin() antes de qualquer operação GAP.
    //   Referência: IDF bug #6971 — stack BT precisa de tempo para inicializar ACL.
    SerialBT.begin(BT_MASTER_NAME, true);  // Master mode: CYD busca HC-06
    SerialBT.setTimeout(100);
    // ASSERT_WARN mitigation: aguarda stack BT estabilizar.
    // 600ms escolhido empiricamente: margem de segurança acima do tempo de init
    // do ACL layer do ESP-IDF BT controller (~400ms medidos em ESP-IDF v4.x).
    // Sem este delay, esp_bt_gap_register_callback() e SerialBT.connect() podem
    // receber eventos HCI antes de o ACL estar pronto → ASSERT_WARN no log.
    delay(600);
    btUltimaRecepMs = 0;  // Watchdog não dispara antes do primeiro pacote

    // Cria mutex para proteger btDeviceList entre GAP callback e Core 1 (LVGL)
    btDeviceMutex = xSemaphoreCreateMutex();
    if (btDeviceMutex == NULL) {
        Serial.println("[ERRO] Falha ao criar btDeviceMutex!");
    }

    // Registra GAP callback APÓS SerialBT.begin() + delay para tratar discovery + PIN.
    // O callback btGapEventHandler lida com: scan results, auth, PIN do HC-06.
    esp_bt_gap_register_callback(btGapEventHandler);

    // Ativa segurança SSP apenas para GAP (não conflita com SPP do SerialBT)
    // Modo: ESP_BT_SP_IOCAP_NONE — sem teclado/display (aceita qualquer PIN via cb)
    esp_bt_io_cap_t ioCap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &ioCap, sizeof(uint8_t));

    // Auto-connect movido para passo 5b (após carregarConfiguracoes que carrega o MAC)
    Serial.printf("[BT] SerialBT iniciado como Master '%s'\n", BT_MASTER_NAME);

    // 1b. PROTEÇÃO SPI: Cria mutex ANTES de inicializar o LVGL
    // O mutex deve existir antes de qualquer operação SPI (display/touch).
    // xSemaphoreCreateMutex() cria um mutex binário inicialmente "disponível".
    // Se a criação falhar (sem memória), spiMutex fica NULL e o código
    // continua sem proteção (graceful degradation, não trava).
    spiMutex = xSemaphoreCreateMutex();
    if(spiMutex == NULL) {
        Serial.println("[ERRO] Falha ao criar mutex SPI! Memoria insuficiente.");
    } else {
        Serial.println("[SPI] Mutex criado com sucesso");
    }
    
    // OBJETIVO 4: Mutex para variáveis de telemetria (Core 0 escreve, Core 1 lê)
    telemetriaMutex = xSemaphoreCreateMutex();
    if(telemetriaMutex == NULL) {
        Serial.println("[ERRO] Falha ao criar mutex Telemetria!");
    } else {
        Serial.println("[DUAL-CORE] Mutex Telemetria criado com sucesso");
    }

    // 2. Inicializa o display CYD com LVGL (orientação USB à esquerda)
    // Configura: driver ILI9341 (SPI), touch XPT2046, LVGL tick/timer
    LVGL_CYD::begin(USB_LEFT);

    // 3. ADC do ESP32 com resolução de 12 bits (0-4095 para 0-3.3V)
    // Usado para leitura direta de tensão no pino 34 (pinoVoltagem)
    analogReadResolution(12);

    // 4. Configura pinos GPIO para estado seguro na partida
    // NOTA v3.1: RELE_9_PIN (GPIO27) removido — compartilha com PSRAM CS no ESP32-WROVER
    //   O relé solar R9 é controlado via serial ao Mega: enviarComandoRele(9, estado)
    // NOTA v3.2: RELE_8_PIN (GPIO26) removido — agora é DAC de áudio (A2DP Sink)
    //   O relé solar R8 é controlado via serial ao Mega: enviarComandoRele(8, estado)
    pinMode(PIN_BACKLIGHT, OUTPUT); // GPIO21: Backlight do display
    digitalWrite(PIN_BACKLIGHT, HIGH); // Tela ligada para mostrar splash

    // 5. Carrega configurações salvas na memória não volátil (NVS)
    // IMPORTANTE: Deve ser chamado ANTES do auto-connect BT (passo 5b),
    // pois carregarConfiguracoes() chama carregarMacBT() que preenche btTargetMac.
    // BUG CORRIGIDO: Antes, carregarConfiguracoes() era chamada DEPOIS do
    // memcpy(btPendingMac, btTargetMac), fazendo o auto-connect usar MAC {0}.
    carregarConfiguracoes();
    carregarDados();  // Carrega dados de telemetria/consumo salvos
    // carregarConfigsDateTime() REMOVIDA (Seção 1: sem RTC)
    carregarAgendaManutencao(); // Restaura agenda de manutenção
    carregarPrecoKm(); // Restaura carregadores B1/B2, preço kWh, km acumulados
    
    // 5b. Agenda auto-connect ao HC-06 usando MAC carregado da NVS
    // btTargetMac já foi preenchido por carregarMacBT() dentro de carregarConfiguracoes()
    memcpy(btPendingMac, btTargetMac, 6);
    pendingBtConnect = true;
    btConectando = true;
    Serial.printf("[BT] Auto-connect agendado para MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        btTargetMac[0], btTargetMac[1], btTargetMac[2],
        btTargetMac[3], btTargetMac[4], btTargetMac[5]);

    // 5c. Configura pino LDR (sensor de luminosidade para farol automático)
    // GPIO36 (VP): ADC1_CH0, leitura analógica 0-4095
    // LDR com resistor pull-down: mais luz = mais tensão
    pinMode(PIN_LDR, INPUT);

    // Log de inicialização (debug via Serial Monitor)
    Serial.println("--- SISTEMA VERSA 3 CYD DISPLAY ---");
    Serial.printf("Solar: %s | Corte: %.0f%%\n", solarLigado ? "LIGADO" : "DESLIGADO", ajustePorcentagem);
    // Data/Hora log removido (Seção 1: sem RTC)

    // Aplica estado dos relés solares conforme configuração carregada
    controlarRelesSolar();
    
    // 6. Registra callback de calibração do touch (intercepta coordenadas)
    // Aplica scale/offset calculados na calibração para corrigir toques
    lv_indev_t * indev = lv_indev_get_next(NULL);
    if(indev) lv_indev_add_event_cb(indev, meu_feedback_cb, LV_EVENT_ALL, NULL);
    
    // 7. MÓDULO F: Monta a splash screen UMA ÚNICA VEZ
    // A barra de progresso será atualizada no loop() via ponteiro splashBar
    // O sistema avança automaticamente para montarInterface() em ~2 segundos
    montarSplashScreen();

    // =====================================================================
    // 8. DUAL-CORE: Cria tarefa de telemetria no Core 0
    // =====================================================================
    // O QUE FAZ: Cria uma tarefa FreeRTOS dedicada à telemetria no Core 0.
    //   O Core 1 (este core, onde roda o loop()) fica exclusivo para LVGL.
    //
    // PARÂMETROS DO xTaskCreatePinnedToCore:
    //   tarefaTelemetriaFunc: função a executar (loop infinito de telemetria)
    //   "Telemetria": nome da tarefa (visível no debug/stack trace)
    //   8192: tamanho do stack em bytes (8KB - suficiente para Serial+NVS)
    //   NULL: parâmetro passado à função (não utilizado)
    //   2: prioridade (2 = alta, permite resposta rápida a dados seriais)
    //   &tarefaTelemetria: ponteiro para armazenar o handle da tarefa
    //   0: Core 0 (Pro Core) - dedicado a telemetria
    //
    // NOTA: Se a criação falhar (ex: sem memória para stack), o sistema
    //   continua funcionando mas SEM telemetria (display mostra zeros).
    //   Log de erro é impresso para diagnóstico via Serial Monitor.
    //
    // WDT: A tarefa inclui vTaskDelay(10ms) que alimenta o watchdog.
    // MEMÓRIA: 8KB de stack é suficiente para:
    //   - Buffer Serial: 128 bytes
    //   - Variáveis locais de strtok/atof: ~200 bytes
    //   - Chamadas NVS (Preferences): ~2KB (abre/fecha namespace)
    //   - Margem de segurança: ~5.5KB
    // =====================================================================
    BaseType_t resultado = xTaskCreatePinnedToCore(
        tarefaTelemetriaFunc,   // Função da tarefa
        "Telemetria",           // Nome (debug)
        8192,                   // Stack: 8KB
        NULL,                   // Parâmetro (não usado)
        2,                      // Prioridade: 2 (alta)
        &tarefaTelemetria,      // Handle de saída
        0                       // Core 0 (Pro Core)
    );
    
    if(resultado != pdPASS) {
        Serial.println("[ERRO] Falha ao criar tarefa Telemetria no Core 0!");
        Serial.println("[ERRO] Sistema opera SEM telemetria.");
    } else {
        Serial.println("[DUAL-CORE] Tarefa Telemetria criada no Core 0 (prioridade 2, stack 8KB)");
    }
    
    Serial.println("[DUAL-CORE] Core 1: LVGL (loop) | Core 0: Telemetria (task)");
    Serial.printf("[DUAL-CORE] Heap livre: %d bytes\n", ESP.getFreeHeap());
}

// =============================================================================
// TAREFA DE TELEMETRIA - CORE 0 (FreeRTOS Task)
// =============================================================================
// O QUE FAZ: Executa todas as operações que NÃO são LVGL em um núcleo separado.
//   Isto libera o Core 1 exclusivamente para o LVGL (renderização + touch),
//   eliminando travamentos causados por operações bloqueantes (Serial, NVS).
//
// PINOS/HARDWARE:
//   - Bluetooth SPP (SerialBT): CYD como MASTER → HC-06 Slave no Mega
//   - GPIO 36 (PIN_LDR): leitura do sensor de luminosidade
//   - GPIO 21 (PIN_BACKLIGHT): controle do backlight em standby
//   - Flash NVS: salvamento periódico de configurações
//   - GPIO 25/26: DAC de áudio (A2DP) — não usar para relés
//   - GPIO 27: PSRAM CS no WROVER — não usar para relés
//
// OTIMIZAÇÃO:
//   - Serial.readBytesUntil() pode bloquear até 100ms (setTimeout no setup)
//     No Core 0, esse bloqueio NÃO afeta o LVGL (que roda no Core 1)
//   - Salvamento NVS com dirty check: só escreve quando valores mudam
//   - Intervalos escalonados: relés 500ms, standby 2s, NVS 30-60s
//
// WDT: vTaskDelay(10ms) a cada iteração garante que o watchdog do Core 0
//   seja alimentado. Sem esse delay, o Core 0 consumiria 100% da CPU e
//   o watchdog resetaria o ESP32 após ~5 segundos.
//
// STACK: 8192 bytes (8KB) - suficiente para buffers Serial (128B) + NVS
// PRIORIDADE: 2 (alta, mas menor que tarefas de sistema do ESP-IDF)
// =============================================================================
void tarefaTelemetriaFunc(void *pvParameters) {
    // Timers locais para intervalos escalonados
    unsigned long t_reles = 0;       // Envio de status de relés: 500ms
    unsigned long t_standby = 0;     // Standby + farol: 2000ms
    unsigned long t_save = 0;        // Configs gerais NVS: 60000ms
    unsigned long t_veiculo = 0;     // Dados críticos NVS: 30000ms
    unsigned long t_dbg_bt = 0;      // Debug BT status: 5000ms
    
    Serial.println("[DUAL-CORE] Tarefa Telemetria iniciada no Core 0");
    
    // Loop infinito da tarefa (nunca retorna)
    for(;;) {
        // =====================================================================
        // DEBUG BT STATUS PERIÓDICO (a cada 5s)
        // =====================================================================
        // Permite rastrear o estado da conexão BT no Monitor Serial sem poluir.
        // Útil para diagnosticar ASSERT_WARN, timeouts e perdas de pacotes.
        // =====================================================================
        if (millis() - t_dbg_bt > 5000UL) {
            t_dbg_bt = millis();
            Serial.printf("[DBG BT] conectado=%d reconectando=%d conectando=%d scanAtivo=%d\n",
                (int)btConectado, (int)btReconectando, (int)btConectando, (int)btScanAtivo);
            if (btUltimaRecepMs > 0) {
                unsigned long segsAgo = (millis() - btUltimaRecepMs) / 1000UL;
                Serial.printf("[DBG BT] Ultimo pacote: %lus atras | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    segsAgo,
                    btTargetMac[0], btTargetMac[1], btTargetMac[2],
                    btTargetMac[3], btTargetMac[4], btTargetMac[5]);
            } else {
                Serial.println("[DBG BT] Aguardando primeiro pacote CSV do Mega...");
            }
        }
        // ─────────────────────────────────────────────────────────────────────
        // LEITURA SERIAL DO ARDUINO MEGA (NÃO BLOQUEANTE COM TIMEOUT 100ms)
        // O QUE FAZ: Lê dados de telemetria, alertas e comandos do Mega.
        // WDT: Serial.readBytesUntil tem timeout de 100ms (configurado no setup).
        //   Mesmo se bloquear por 100ms, o vTaskDelay no final alimenta o WDT.
        // OTIMIZAÇÃO: Chamada a cada iteração (~10ms) para não perder dados.
        // ─────────────────────────────────────────────────────────────────────
        lerDadosUno();
        
        // =====================================================================
        // BT WATCHDOG: RECONEXÃO AUTOMÁTICA (MASTER MODE)
        // =====================================================================
        // O QUE FAZ: Se nenhum pacote CSV foi recebido por BT_WATCHDOG_MS (3s),
        //   considera que a conexão com o HC-06 caiu.
        //   Como somos Master, tentamos SerialBT.connect(savedMac) para reconectar.
        //
        // ASSERT_WARN mitigation: SerialBT.begin() tem delay de 600ms embutido
        //   (vTaskDelay após begin) para deixar a stack BT estabilizar antes de
        //   chamar SerialBT.connect(). Sem este delay, o IDF BT controller emite
        //   ASSERT_WARN quando recebe HCI antes do ACL estar pronto.
        //
        // btUltimaRecepMs == 0 na partida → não dispara watchdog até o 1º pacote.
        // btConectando == true → já há uma tentativa em curso; não duplicar.
        // =====================================================================
        if(btUltimaRecepMs > 0 && !btReconectando && !btConectando &&
           (millis() - btUltimaRecepMs > BT_WATCHDOG_MS)) {
            
            btConectado = false;
            btReconectando = true;
            btReconectandoMs = millis();
            
            Serial.println("[BT] Watchdog: 3s sem dados — tentando reconectar ao HC-06...");
            
            btConectando = true;
            SerialBT.end();
            // ASSERT_WARN mitigation: aguarda stack BT liberar recursos + estabilizar
            vTaskDelay(pdMS_TO_TICKS(BT_RESTART_DELAY_MS + 400));
            SerialBT.begin(BT_MASTER_NAME, true);  // Re-inicia como Master
            SerialBT.setTimeout(100);
            // Delay adicional pós-begin para evitar ASSERT_WARN na stack BT
            vTaskDelay(pdMS_TO_TICKS(600));
            esp_bt_gap_register_callback(btGapEventHandler);  // Re-registra GAP callback
            // Tenta conectar ao MAC (BT_DEFAULT_MAC ou MAC salvo na NVS)
            if (SerialBT.connect(btTargetMac)) {
                Serial.printf("[BT] Reconexão OK para MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                    btTargetMac[0], btTargetMac[1], btTargetMac[2],
                    btTargetMac[3], btTargetMac[4], btTargetMac[5]);
                btReconectando = false;
                btConectando   = false;
            } else {
                Serial.println("[BT] Falha reconexão — aguardando proxima janela de watchdog (3s)");
                btReconectando = false;
                btConectando   = false;
            }
        }
        // =====================================================================
        // RECONEXÃO FORÇADA PELO USUÁRIO (pendingReconexaoBT) — MASTER MODE
        // =====================================================================
        // Core 1 (callback LVGL) seta pendingReconexaoBT = true.
        // Este bloco (Core 0) executa o restart + connect do SerialBT com segurança.
        // =====================================================================
        if(pendingReconexaoBT) {
            pendingReconexaoBT = false;  // Consome a flag imediatamente
            Serial.println("[BT] Executando reconexão forçada (solicitada pelo Core 1)...");
            btConectando = true;
            SerialBT.end();
            vTaskDelay(pdMS_TO_TICKS(BT_RESTART_DELAY_MS + 400));
            SerialBT.begin(BT_MASTER_NAME, true);  // Re-inicia como Master
            SerialBT.setTimeout(100);
            // Delay pós-begin: mitiga ASSERT_WARN na stack BT (IDF ACL init ~400ms)
            vTaskDelay(pdMS_TO_TICKS(600));
            esp_bt_gap_register_callback(btGapEventHandler);
            Serial.printf("[BT] Forçar Reconexão ao MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                btTargetMac[0], btTargetMac[1], btTargetMac[2],
                btTargetMac[3], btTargetMac[4], btTargetMac[5]);
            if (SerialBT.connect(btTargetMac)) {
                Serial.println("[BT] Reconexão forçada OK");
            } else {
                Serial.println("[BT] Reconexão forçada falhou — aguarda watchdog (3s)");
            }
            btConectando = false;
        }

        // =====================================================================
        // CONEXÃO A NOVO DISPOSITIVO (pendingBtConnect — seleção no PAREAMENTO_BT
        // ou auto-connect no boot via BT_DEFAULT_MAC)
        // =====================================================================
        if(pendingBtConnect) {
            pendingBtConnect = false;
            btConectando = true;
            btConectado  = false;
            Serial.printf("[BT] Conectando ao MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                btPendingMac[0], btPendingMac[1], btPendingMac[2],
                btPendingMac[3], btPendingMac[4], btPendingMac[5]);
            // Reinicia BT para limpar conexão anterior (se existir)
            SerialBT.end();
            vTaskDelay(pdMS_TO_TICKS(BT_RESTART_DELAY_MS + 400));
            SerialBT.begin(BT_MASTER_NAME, true);
            SerialBT.setTimeout(100);
            // Delay pós-begin: mitiga ASSERT_WARN (IDF BT ACL init time)
            vTaskDelay(pdMS_TO_TICKS(600));
            esp_bt_gap_register_callback(btGapEventHandler);
            if (SerialBT.connect(btPendingMac)) {
                // Salva MAC na NVS para auto-connect futuros
                salvarMacBT(btPendingMac);
                Serial.println("[BT] Conectado e MAC salvo na NVS");
                btReconectando = false;
            } else {
                Serial.println("[BT] Falha ao conectar — MAC agendado para watchdog");
            }
            btConectando = false;
            pendingRefreshBtList = true;  // Atualiza UI
        }

        // =====================================================================
        // SCAN DE DISPOSITIVOS BT (pendingBtScan — botão no PAREAMENTO_BT)
        // =====================================================================
        if(pendingBtScan) {
            pendingBtScan = false;
            // Garante que não está conectado antes de fazer scan
            SerialBT.disconnect();
            vTaskDelay(pdMS_TO_TICKS(300));
            // Limpa lista de dispositivos
            if (xSemaphoreTake(btDeviceMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                memset(btDeviceList, 0, sizeof(btDeviceList));
                btDeviceCount = 0;
                xSemaphoreGive(btDeviceMutex);
            }
            btScanAtivo = true;
            // Inquiry de 10 segundos (BT_INQUIRY_DURATION_UNITS unidades de 1.28s)
            esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, BT_INQUIRY_DURATION_UNITS, 0);
            if (err == ESP_OK) {
                Serial.println("[BT] Scan de dispositivos iniciado (10s)...");
            } else {
                Serial.printf("[BT] Falha ao iniciar scan: 0x%X\n", err);
                btScanAtivo = false;
            }
        }

        if(pendingBtScanStop && btScanAtivo) {
            pendingBtScanStop = false;
            esp_bt_gap_cancel_discovery();
            Serial.println("[BT] Scan interrompido pelo utilizador");
        }
        // =====================================================================
        // O QUE FAZ: Sincroniza estado dos relés com o Mega e verifica solar.
        // WDT: Cada envio serial leva ~0.9ms (10 bytes a 115200 baud).
        //   Total: ~5ms para 6 comandos. Não causa WDT.
        // =====================================================================
        if(millis() - t_reles > 500) {
            if(!splashScreenAtiva) {
                enviarStatusReles();
                controlarRelesSolar();
            }
            
            // atualizarRelogio() REMOVIDO (Seção 1: sem RTC)
            
            // Alertas automáticos baseados em dados de telemetria
            // ACS758 max 50A: corrente > 45A = alerta
            if(ampere > 45.0f) alertaCorrenteExc = true;
            else alertaCorrenteExc = false;
            
            // Subtensão: usa limites dinâmicos da configuração de bateria (não hardcoded 36V)
            // Limiar = vMin da bateria B1 (índice 0) + margem de 10% do range
            {
                Limites limB1 = obterLimitesBateria(0);
                Limites limB2 = obterLimitesBateria(1);
                float threshB1 = limB1.vMin + (limB1.vMax - limB1.vMin) * 0.10f;
                float threshB2 = limB2.vMin + (limB2.vMax - limB2.vMin) * 0.10f;
                if ((vB1 < threshB1 && vB1 > 5.0f) || (vB2 < threshB2 && vB2 > 5.0f))
                    alertaSubtensao = true;
                else
                    alertaSubtensao = false;
            }
            
            // Bateria de sistema B3 crítica: < 11V (12V nominal)
            if(vB3 < 11.0f && vB3 > 5.0f) alertaBatCritica = true;
            else alertaBatCritica = false;
            
            // Módulo G: Acumula km por bat/força/tração
            acumularKmPorModo();

            // Calcular autonomia estimada baseada na % restante e consumo histórico.
            // Fórmula: autonomia = kmPorPorc * percAtual
            //   kmPorPorc = km rodados desde a última carga / % consumida
            // Se não houver histórico suficiente, usa estimativa simplificada:
            //   autonomia = (percAtual / 100) * rangeMax (aprox 60km para 48V/10Ah)
            {
                int batIdx = (bateriaEmUso == 1) ? 0 : 1;
                float vAtual = (bateriaEmUso == 1) ? vB1 : vB2;
                int percAtual = calcularPorcentagem(batIdx, vAtual);
                float kmPorPercCalc = 0.0f;
                // Tenta calcular km/% do acumulador atual (força F1, modo 1x3 como base)
                if (percGasto[batIdx][0][0] > 1.0f && kmAcum[batIdx][0][0] > 0.1f) {
                    kmPorPercCalc = kmAcum[batIdx][0][0] / percGasto[batIdx][0][0];
                }
                if (kmPorPercCalc > 0.1f) {
                    autonomia = kmPorPercCalc * (float)percAtual;
                } else {
                    // Estimativa grosseira: 1% ≈ 0.6km (60km full para 48V/10Ah típico)
                    autonomia = (float)percAtual * 0.6f;
                }
                // Limita a valor razoável
                if (autonomia < 0.0f) autonomia = 0.0f;
                if (autonomia > 999.0f) autonomia = 999.0f;
            }
            
            // Timeout do radar de proximidade
            if(radarAtivo && (millis() - radarUltimaLeitura > RADAR_TIMEOUT_MS)) {
                radarAtivo = false;
                radarDistanciaEsq = 0;
                radarDistanciaDir = 0;
            }
            
            t_reles = millis();
        }
        
        // =====================================================================
        // STANDBY E FAROL HÍBRIDO (INTERVALO: 2000ms)
        // =====================================================================
        // O QUE FAZ: Gerencia modo standby (tela desligada) e farol automático.
        // WDT: analogRead (LDR) leva ~100us. enviarComandoRele ~1ms. Seguro.
        // OTIMIZAÇÃO: Intervalo de 2s evita sobrecarga de comandos serial.
        // =====================================================================
        if(!splashScreenAtiva && (millis() - t_standby > BT_HEARTBEAT_INTERVAL_MS)) {
            processarStandby();
            if(!modoStandby) processarFarolHibrido();
            // Heartbeat para o Mega: confirma que o CYD está ativo e a ligação BT funciona.
            // Só envia se conectado para não causar erro de escrita com SerialBT desconectado.
            if (btConectado) {
                SerialBT.print("BT_OK\n");
            }
            t_standby = millis();
        }
        
        // =====================================================================
        // PERSISTÊNCIA: CONFIGS GERAIS NA NVS (INTERVALO: 60000ms = 1 MINUTO)
        // =====================================================================
        // O QUE FAZ: Salva calibração, velocidade, bateria ativa na NVS.
        // OTIMIZAÇÃO NVS: salvarConfiguracoes() tem dirty check interno.
        //   Se nada mudou, retorna em ~0ms (sem escrita na flash).
        //   Se mudou, escrita leva ~5ms. Flash suporta ~100k ciclos/setor.
        // WDT: Escrita NVS leva no máximo ~10ms. Seguro.
        // =====================================================================
        if(millis() - t_save > 60000) {
            salvarConfiguracoes();
            calcularCustoKm();   // Módulo G: Recalcula custo/km
            detectarCargaAuto(); // Módulo G: Detecta carga por diferencial de voltagem
            t_save = millis();
        }
        
        // =====================================================================
        // PERSISTÊNCIA: DADOS CRÍTICOS NA NVS (INTERVALO: 30000ms = 30s)
        // =====================================================================
        // O QUE FAZ: Salva kmTotal e circRodaCM — dados que NÃO podem ser
        //   perdidos em caso de desligamento inesperado (queda de energia).
        // OTIMIZAÇÃO NVS: salvarDadosVeiculo() tem dirty check interno.
        //   Em uso normal, kmTotal muda constantemente mas a escrita real
        //   só ocorre a cada 30s (compromisso segurança vs vida útil flash).
        //   Se o veículo está parado, kmTotal não muda e NVS não escreve.
        // WDT: Escrita NVS ~2ms. Seguro.
        // =====================================================================
        if(millis() - t_veiculo > 30000) {
            salvarDadosVeiculo();
            t_veiculo = millis();
        }
        
        // =====================================================================
        // WDT PREVENTION: vTaskDelay OBRIGATÓRIO
        // =====================================================================
        // O QUE FAZ: Libera o processador por 10ms para o scheduler FreeRTOS.
        // POR QUE 10ms: Compromisso entre responsividade serial (~100 bytes a
        //   115200 baud levam ~0.9ms, buffer de 64 bytes não transborda em 10ms)
        //   e economia de energia (CPU idle durante o delay).
        // WDT: Este é o ponto principal de alimentação do watchdog do Core 0.
        //   Sem este delay, o watchdog reseta o ESP32 em ~5 segundos.
        //   O scheduler do FreeRTOS automaticamente "alimenta" o WDT quando
        //   uma tarefa cede o processador via vTaskDelay.
        // =====================================================================
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // NOTA: Este ponto NUNCA é alcançado (loop infinito).
    // Se por algum motivo a tarefa sair do loop, deleta-se automaticamente.
    vTaskDelete(NULL);
}

// =============================================================================
// LOOP PRINCIPAL - CORE 1 (EXCLUSIVO PARA LVGL)
// =============================================================================
// O QUE FAZ: Executa APENAS operações de interface gráfica (LVGL) no Core 1.
//   Todas as operações de telemetria, serial, NVS e GPIO foram movidas para
//   a tarefa tarefaTelemetriaFunc() no Core 0.
//
// ARQUITETURA DUAL-CORE:
//   ANTES (single-core): loop() fazia TUDO — LVGL, Serial, NVS, GPIO, cálculos
//     Problema: Serial.readBytesUntil() bloqueava por até 100ms, travando LVGL
//     Resultado: tela não respondia a toques, animações travavam, WDT reset
//
//   DEPOIS (dual-core): loop() faz APENAS LVGL
//     Core 1 (loop): lv_timer_handler, UI updates, overlays, splash
//     Core 0 (task): Serial, relés, solar, standby, NVS, cálculos
//     Resultado: LVGL sempre responsivo, zero travamentos, zero WDT resets
//
// PROTEÇÃO SPI:
//   O lv_timer_handler() acessa o SPI para:
//     1. Enviar pixels ao display ILI9341 (SPI write)
//     2. Ler coordenadas do touch XPT2046 (SPI read)
//   O mutex spiMutex garante que nenhuma outra operação SPI ocorra
//   simultaneamente (ex: se futuro módulo SD card for adicionado ao SPI).
//
// WDT PREVENTION:
//   vTaskDelay(5ms) no final de cada iteração:
//     1. Alimenta o watchdog do Core 1 (evita reset por timeout ~5s)
//     2. Libera CPU para tarefas de sistema (WiFi, BT stack)
//     3. Limita taxa a ~200 iterações/s (ideal para LVGL)
//     4. Economiza energia (Core 1 idle durante 5ms)
//
// PINOS/HARDWARE: SPI (display + touch), nenhum GPIO direto
// OTIMIZAÇÃO: Core 1 dedicado = latência de toque < 5ms (imperceptível)
// =============================================================================
void loop() {
    // =====================================================================
    // 1. LVGL: Renderização + Touch (PROTEGIDO POR MUTEX SPI)
    // =====================================================================
    // O QUE FAZ: Processa a fila de renderização do LVGL e lê o touch.
    // PROTEÇÃO SPI: O mutex garante acesso exclusivo ao barramento SPI
    //   durante toda a operação do LVGL (display write + touch read).
    //   Timeout de 50ms: se o mutex não estiver disponível em 50ms,
    //   pula esta iteração (não trava). Situação rara (mutex quase sempre livre).
    // DEVE ser chamado a cada ~5ms para responsividade adequada.
    // Se não for chamado, toques não são detectados e animações travam.
    // =====================================================================
    // =====================================================================
    // Seção 2: Anti-click acidental ao acordar do standby.
    // O wakeupTouchBlock bloqueia APENAS o processamento de touch (input device).
    // O lv_timer_handler() é chamado SEMPRE para manter rendering e animações fluindo.
    // Após 300ms, o bloqueio é removido e toques voltam a ser aceitos.
    // =====================================================================
    if(wakeupTouchBlock && (millis() - wakeupTouchBlockTime > 300)) {
        wakeupTouchBlock = false;
    }

    // lv_timer_handler() processa rendering + input. É chamado sempre
    // (o bloqueio de touch é feito pelo driver de input, não aqui).
    if(spiMutex != NULL) {
        if(xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(spiMutex);
        }
    } else {
        lv_timer_handler();
    }

    // =====================================================================
    // Processar flag pendingMontarInterface (enviada pelo Core 0 via lerDadosUno).
    // LVGL não é thread-safe: montarInterface() só pode ser chamada no Core 1.
    // =====================================================================
    if(pendingMontarInterface && !splashScreenAtiva) {
        pendingMontarInterface = false;
        montarInterface();
    }

    // =====================================================================
    // Processar flag pendingRefreshBtList (enviada pelo GAP callback ou Core 0).
    // Atualiza a tela PAREAMENTO_BT com novos dispositivos encontrados no scan.
    // =====================================================================
    if(pendingRefreshBtList && !splashScreenAtiva) {
        pendingRefreshBtList = false;
        if (paginaAtual == PAREAMENTO_BT || paginaAtual == BLUETOOTH_MENU) {
            montarInterface();
        }
    }

    // =====================================================================
    // 2. MÓDULO F: Controle da Splash Screen
    // =====================================================================
    // O QUE FAZ: Incrementa a barra de progresso e transita para interface.
    // NOTA: Roda no Core 1 pois manipula objetos LVGL (splashBar, montarInterface).
    //   LVGL NÃO é thread-safe — TODAS as chamadas LVGL devem ser no mesmo core.
    // WDT: Operação rápida (~0.1ms). Não causa WDT.
    // =====================================================================
    if(splashScreenAtiva) {
        static unsigned long ultimoSplashUpdate = 0;
        if(millis() - ultimoSplashUpdate > 100) {
            splashProgress += 5;  // +5% a cada 100ms (20 steps total)
            if(splashProgress > 100) splashProgress = 100;
            
            // Atualiza APENAS o valor da barra (ponteiro armazenado no setup)
            // NULL check: protege contra acesso a objeto já destruído
            if(splashBar != NULL) {
                lv_bar_set_value(splashBar, splashProgress, LV_ANIM_ON);
            }
            
            // Transição para interface principal quando barra completa
            // NÃO depende de sistemaReady (removido — boot nunca trava)
            if(splashProgress >= 100) {
                splashScreenAtiva = false;  // Desativa flag da splash
                splashBar = NULL;           // Limpa ponteiro (será destruído por lv_obj_clean)
                montarInterface();          // Carrega a interface principal
            }
            ultimoSplashUpdate = millis();
        }
        // IMPORTANTE: NÃO usa return aqui.
        // O lv_timer_handler() no início do loop DEVE executar mesmo durante
        // a splash para que o LVGL processe animações da barra de progresso.
    }
    
    // =====================================================================
    // 3. MÓDULO A: Teste Automático de Relés (sequenciador)
    // =====================================================================
    // O QUE FAZ: Processa a sequência de teste de relés.
    // NOTA: Roda no Core 1 pois chama montarInterface() (LVGL).
    // Se testeAutomaticoAtivo == false, retorna imediatamente (custo zero).
    // WDT: Não bloqueante (verifica millis e retorna).
    // =====================================================================
    processarTesteAutomatico();

    // =====================================================================
    // 4. Atualização Periódica da Interface (INTERVALO: 500ms)
    // =====================================================================
    // O QUE FAZ: Atualiza labels de telemetria, overlays e indicadores.
    // NOTA: APENAS operações LVGL. Telemetria (serial, relés, NVS) está no Core 0.
    // CORREÇÃO: Só executa quando a splash NÃO está ativa.
    //   Durante a splash, label_baterias/label_status/etc são NULL.
    //   Chamar atualizarDadosPrincipais() com labels NULL = crash.
    // WDT: Operações LVGL são rápidas (~2ms). Não causa WDT.
    // =====================================================================
    static uint32_t t = 0;

    if (!splashScreenAtiva && (millis() - t > 500)) {
        // Operações LVGL de atualização de dados na tela
        atualizarDadosPrincipais();  // Atualiza textos de tensão, km, status
        atualizarOverlayAlertas();   // Overlay de alertas no lv_layer_top()
        processarTimerKeyOff();      // Timer 20s pós-chave (diálogo keep-alive)
        atualizarStatusBT();         // Atualiza label de status na tela BLUETOOTH_MENU
        
        // Auto-refresh do radar: redesenha gauge se recebendo dados
        if(paginaAtual == RADAR_PROXIMIDADE && emSubMenu && radarAtivo) {
            montarInterface();
        }
        
        t = millis();
    }
    
    // =====================================================================
    // 5. Verificação de Manutenção (INTERVALO: 30s)
    // =====================================================================
    // O QUE FAZ: Verifica se é hora de manutenção e mostra overlay LVGL.
    // NOTA: Roda no Core 1 pois cria objetos LVGL (overlay de lembrete).
    // WDT: Verificação rápida (~0.1ms). Não causa WDT.
    // =====================================================================
    static uint32_t t_manut = 0;
    if(!splashScreenAtiva && (millis() - t_manut > 30000)) {
        verificarAgendaManutencao();
        t_manut = millis();
    }

    // =====================================================================
    // 6. WDT PREVENTION: vTaskDelay OBRIGATÓRIO (substitui delay(5))
    // =====================================================================
    // O QUE FAZ: Libera o Core 1 por 5ms para o scheduler FreeRTOS.
    // POR QUE 5ms (e não delay(5)):
    //   delay() do Arduino é implementado como busy-wait em algumas versões,
    //   o que NÃO alimenta o watchdog nem libera CPU para tarefas de sistema.
    //   vTaskDelay() é a forma correta no FreeRTOS:
    //     1. Alimenta o watchdog timer do Core 1
    //     2. Permite que tarefas de sistema (WiFi, BT) executem
    //     3. Coloca o core em idle (economia de energia)
    //     4. Retorna após exatamente 5ms (precisão do tick do FreeRTOS)
    // TAXA RESULTANTE: ~200 iterações/segundo (ideal para LVGL a cada 5ms)
    // ENERGIA: Core 1 consome ~0W durante o idle (vs ~0.1W em busy-wait)
    // =====================================================================
    vTaskDelay(pdMS_TO_TICKS(5));
}
