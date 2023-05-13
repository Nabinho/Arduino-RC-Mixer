/**********************************************************************************************************************
 * V1.0 - @Nabinho
 * 13/05/2023
 *
 * Codigo de mixagem de canais de receptor de radio controle com placas Arduino.
 * Baseado para o controle de motores DC com a DFRobot Romeo.
**********************************************************************************************************************/

// --------------------------------------------------------------------------------------------------------------------

// Definicao para DEBUG
// Descomente para debug no monitor serial
//#define DEBUG

// Variaveis dos pinos para leitura dos canais
const int PINO_THR = 2;
const int PINO_STR = 3;

// Variaveis dos pinos para controle dos motores
const uint8_t PINO_ENA = 5;
const uint8_t PINO_DIRA = 4;
const uint8_t PINO_ENB = 6;
const uint8_t PINO_DIRB = 7;

// Variaveis para o canal THR
unsigned long tempo_atual_THR = 0;
unsigned long tempo_antes_THR = 0;
int tempo_alta_THR = 0;
float tempo_alta_THR_float;

// Variaveis para o canal STR
unsigned long tempo_atual_STR = 0;
unsigned long tempo_antes_STR = 0;
int tempo_alta_STR = 0;
float tempo_alta_STR_float;

// Variaveis dos limites de sinais de PPM de saida
const int PPM_CENTRAL = 0;
const int PPM_MAXIMO = 255;
const int PPM_MINIMO = -255;

// Variaveis dos sinais de acionamento dos drivers
int PPM_esquerda = 0;
int PPM_direita = 0;
int ultimo_PPM_esquerda;
int ultimo_PPM_direita;
int soma_maxima = 120;
int soma_minima = -120;
float soma_esquerda;
float soma_direita;
const float TEMPO_MINIMO_PPM = 0.90;
const float TEMPO_MAXIMO_PPM = 2.10;

// Variaveis para temporizacao do Fail-Safe
unsigned long tempo_ultimo_dado = 0;
unsigned long tempo_failsafe = 0;
const int INTERVALO_FAILSAFE = 1000;

// Pino conectado ao LED BUILTIN da Vespa
const int PINO_LED = 13;

// --------------------------------------------------------------------------------------------------------------------

void setup() {

  // Inicia comunicacao serial em 115200 bps
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("<------- VESPA HOBBY V1.0 ------->");
#endif

  // Configura o pino conectado ao LED BUILTIN  da Vespa como saída
  pinMode(PINO_LED, OUTPUT);

  // Configura os pinos de controle dos motores como saida
  pinMode(PINO_ENA, OUTPUT);
  pinMode(PINO_DIRA, OUTPUT);
  pinMode(PINO_ENB, OUTPUT);
  pinMode(PINO_DIRB, OUTPUT);

  // Inicia os motores parados
  acionaMotorA(PPM_CENTRAL);
  acionaMotorB(PPM_CENTRAL);

  // Configura os pinos de leitura dos canais como entradas
  pinMode(PINO_THR, INPUT);
  pinMode(PINO_STR, INPUT);

  // Configuracao das interrupcoes dos canais
  attachInterrupt(digitalPinToInterrupt(PINO_THR), mede_THR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_STR), mede_STR, CHANGE);
}

// --------------------------------------------------------------------------------------------------------------------

void loop() {

  // Conversao da duracao dos pulsos de 'long' para 'float' e de 'us' para 'ms'
  tempo_alta_THR_float = float(tempo_alta_THR);
  tempo_alta_STR_float = float(tempo_alta_STR);
  tempo_alta_THR_float = tempo_alta_THR_float / 1000.0;
  tempo_alta_STR_float = tempo_alta_STR_float / 1000.0;

  // Verifica se a duracao do pulso de ambos os canais esta dentro da faixa limite
  if (tempo_alta_THR_float > TEMPO_MINIMO_PPM && tempo_alta_THR_float < TEMPO_MAXIMO_PPM && tempo_alta_STR_float > TEMPO_MINIMO_PPM && tempo_alta_STR_float < TEMPO_MAXIMO_PPM) {

    // Atualiza a contagem de tempo do ultimo pulso recebido
    tempo_ultimo_dado = millis();

    // Acende o LED BUILTIN para indicar o recebimento de dados do receptor
    digitalWrite(PINO_LED, HIGH);

    // Calcula a diferenca dos canais para o controle de cada driver
    soma_direita = (tempo_alta_THR_float - tempo_alta_STR_float) * 200.0;
    soma_esquerda = (tempo_alta_THR_float + tempo_alta_STR_float - 3) * 200.0;

    // Mapeia o resultado do calculo para os valores maximo e minimo de acionamento dos drivers
    PPM_direita = map(soma_direita, soma_minima, soma_maxima, PPM_MINIMO, PPM_MAXIMO);
    PPM_esquerda = map(soma_esquerda, soma_minima, soma_maxima, PPM_MINIMO, PPM_MAXIMO);

    // Limita os valores de controle dentro dos valores maximo e minimo
    PPM_direita = constrain(PPM_direita, PPM_MINIMO, PPM_MAXIMO);
    PPM_esquerda = constrain(PPM_esquerda, PPM_MINIMO, PPM_MAXIMO);

    // Verifica se o novo dado e diferente do ultimo recebido
    if (PPM_esquerda != ultimo_PPM_esquerda || PPM_direita != ultimo_PPM_direita) {
      // Aciona os canais com os sinais mapeados
      acionaMotorA(PPM_esquerda);
      acionaMotorB(PPM_direita);
    }

    // Atualiza as variaveis dos ultimos valores recebidos
    ultimo_PPM_esquerda = PPM_esquerda;
    ultimo_PPM_direita = PPM_direita;

    // Se DEBUG definido
#ifdef DEBUG
    // Exibe informacoes dos canais e dos calculos para monitoramento
    Serial.println("------------ CANAL THR ------------");
    Serial.print("TEMPO EM ALTA (ms): ");
    Serial.print(tempo_alta_THR_float);
    Serial.println("");
    Serial.println("------------ CANAL STR ------------");
    Serial.print("TEMPO EM ALTA (ms): ");
    Serial.print(tempo_alta_STR_float);
    Serial.println("");
    Serial.println("--- RESULTADO DE SOMA DOS CANAIS ---");
    Serial.print("SOMATORIA ESUQUERDA: ");
    Serial.print(soma_esquerda);
    Serial.print(" | SOMATORIA DIREITA: ");
    Serial.print(soma_direita);
    Serial.println("");
    Serial.println("------- SINAIS PPM MAPEADOS -------");
    Serial.print("ESUQERDA: ");
    Serial.print(PPM_esquerda);
    Serial.print(" | DIREITA: ");
    Serial.print(PPM_direita);
    Serial.println("");
#endif

  }
  // Caso nao haja novos dados validos dos canais
  else {

    // Zera as variaveis que armazenam os tempos dos canais convertidas para 'float'
    tempo_alta_THR_float = 0;
    tempo_alta_STR_float = 0;

    // Apaga o LED BUILTIN para indicar que não há sinal do receptor
    digitalWrite(PINO_LED, LOW);
  }

  // Caso o sinal de um dos canais ezteja zerado
  if (tempo_alta_THR_float == 0.0 || tempo_alta_STR_float == 0.0) {
    // Atualiza contagem de tempo do Fail-Safe
    tempo_failsafe = millis();
    // Verifica se a diferenca entre o tempo de Fail-Safe o tempo do ultimo dado valido recebido e maior do que o intervalor de acionamento do Fail-Safe
    if ((tempo_failsafe - tempo_ultimo_dado) > INTERVALO_FAILSAFE) {

      // Mantem os motores parados ate o recebimento de sinais ser restaurado
      acionaMotorA(PPM_CENTRAL);
      acionaMotorB(PPM_CENTRAL);

      // Se DEBUG definido
#ifdef DEBUG
      // Alerta que o Fail-Safe foi ativado
      Serial.println("FAILSAFE ATIVADO!!!");
#endif
    }
  }
}

// --------------------------------------------------------------------------------------------------------------------

// Funcao para medir o tempo do sinal do canal "THR"
void mede_THR() {

  // Atualiza a contagem de tempo atual
  tempo_atual_THR = micros();

  // Verifica se a mudança de sinal do canal foi de "HIGH" para "LOW"
  if (digitalRead(PINO_THR) == LOW) {
    // Calcula a duracao do pulso em nivel logico alto do canal
    tempo_alta_THR = tempo_atual_THR - tempo_antes_THR;
  }

  //  Atualiza a contagem de tempo anterior
  tempo_antes_THR = tempo_atual_THR;
}

// --------------------------------------------------------------------------------------------------------------------

// Funcao para medir o tempo do sinal do canal "STR"
void mede_STR() {

  // Atualiza a contagem de tempo atual
  tempo_atual_STR = micros();

  // Verifica se a mudança de sinal do canal foi de "HIGH" para "LOW"
  if (digitalRead(PINO_STR) == LOW) {
    // Calcula a duracao do pulso em nivel logico alto do canal
    tempo_alta_STR = tempo_atual_STR - tempo_antes_STR;
  }

  //  Atualiza a contagem de tempo anterior
  tempo_antes_STR = tempo_atual_STR;
}

// --------------------------------------------------------------------------------------------------------------------

// Funcao de controle do motor A (esquerda)
void acionaMotorA(int velocidade) {
  if (velocidade < 0) {
    digitalWrite(PINO_DIRA, LOW);
  } else {
    digitalWrite(PINO_DIRA, HIGH);
  }
  analogWrite(PINO_ENA, abs(velocidade));
}

// --------------------------------------------------------------------------------------------------------------------

// Funcao de controle do motor B (direita)
void acionaMotorB(int velocidade) {
  if (velocidade < 0) {
    digitalWrite(PINO_DIRB, LOW);
  } else {
    digitalWrite(PINO_DIRB, HIGH);
  }
  analogWrite(PINO_ENB, abs(velocidade));
}

// --------------------------------------------------------------------------------------------------------------------
