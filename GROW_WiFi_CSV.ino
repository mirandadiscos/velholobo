/* PROJETO GROW - ESP32 classico / WROOM + Wi-Fi, CSV, TelnetStream e ArduinoOTA
   Mantenha GrowRede.h e GrowRegistro.h na mesma pasta deste .ino.
   Bibliotecas: HX711 por Bogdan Necula; WiFiManager por tzapu; TelnetStream por jandrassy; ArduinoOTA (nativa do ESP32).
   Build 2.0.2: Estrutura original mantida com validação de peso + umidade < 30%.
*/
#include <Arduino.h>
#include <driver/gpio.h>
#include <HX711.h>
#include <Preferences.h>
#include <TelnetStream.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <stddef.h>
#include "GrowRede.h"

const uint8_t PINO_BOMBA=2, PINO_DOUT=21, PINO_SCK=22, PINO_UMIDADE=34;
const float MOLHADO_MV=800.0f, SECO_MV=2200.0f;
const float UMIDADE_INICIAR=30.0f, UMIDADE_PARAR=85.0f;
const unsigned long LOG_MS=500, ADC_MS=10, PESO_TIMEOUT=1000, ADC_TIMEOUT=500;
const unsigned long REGISTRO_MS_DESLIGADA=10000, REGISTRO_MS_LIGADA=1000;
const uint32_t ASSINATURA=0x47525731, VERSAO=1;

struct Ajustes {
  uint32_t assinatura, versao;
  float fator;
  int32_t tara;
  float doseG;
  uint32_t checksum;
};
static_assert(sizeof(Ajustes)==24, "Layout de memoria inesperado");
uint32_t calcularChecksum(const Ajustes &v);
bool salvarAjustes(Ajustes v);
bool registroValido(const Ajustes &v);

Ajustes ajustes={ASSINATURA,VERSAO,0.0f,0,0.0f,0};
bool modoPlotter=false;
bool telnetIniciado=false;
bool otaIniciado=false;
bool telnetLogContinuo=false; // Desativado por padrao no Telnet para manter o PuTTY limpo

#define INFO_PRINTF(...) do { \
  if (!modoPlotter) { \
    Serial.printf(__VA_ARGS__); \
    if (telnetIniciado && WiFi.status() == WL_CONNECTED) { \
      TelnetStream.printf(__VA_ARGS__); \
    } \
  } \
} while (0)

#define INFO_PRINTLN(...) do { \
  if (!modoPlotter) { \
    Serial.println(__VA_ARGS__); \
    if (telnetIniciado && WiFi.status() == WL_CONNECTED) { \
      TelnetStream.print(__VA_ARGS__); \
      TelnetStream.print("\r\n"); \
    } \
  } \
} while (0)

#define INFO_PRINT(...) do { \
  if (!modoPlotter) { \
    Serial.print(__VA_ARGS__); \
    if (telnetIniciado && WiFi.status() == WL_CONNECTED) { \
      TelnetStream.print(__VA_ARGS__); \
    } \
  } \
} while (0)

HX711 balanca;
Preferences memoria;
bool memoriaOK=false, bombaLigada=false, bloqueado=false, rearmado=true;
const char *motivo="INICIALIZANDO";
double pesoInicioG=0.0, pesoAlvoG=0.0, pesoPosRegaG=0.0;

long pesos[5]={};
uint8_t posPeso=0, nPeso=0;
uint32_t umidades[25]={}, somaMv=0;
uint8_t posAdc=0, nAdc=0;
unsigned long ultimoPeso=0, ultimoAdc=0, ultimoLog=0, ultimoRegistro=0, ultimoAvisoConfirmacao=0;

char entrada=0, coleta=0;
char numero[40];
uint8_t nNumero=0, nColeta=0;
bool numeroLongo=false, taraPronta=false;
int32_t taraCandidata=0;
int64_t somaColeta=0;
double massaConhecida=0.0;
unsigned long inicioColeta=0;

bool aguardandoConfirmacao = false;

bool calibrada() { return isfinite(ajustes.fator) && fabs(ajustes.fator)>=0.000001f; }
bool emAjuste() { return entrada || coleta || taraPronta; }
bool pesoDisponivel() { return calibrada() && nPeso==5 && millis()-ultimoPeso<=PESO_TIMEOUT; }
bool adcDisponivel() { return nAdc==25 && millis()-ultimoAdc<=ADC_TIMEOUT; }
float tensaoMv() { return nAdc ? static_cast<float>(somaMv)/nAdc : 0.0f; }
float umidadeRelativa() {
  float u=100.0f*(SECO_MV-tensaoMv())/(SECO_MV-MOLHADO_MV);
  return u<0 ? 0 : (u>100 ? 100 : u);
}
double pesoAtualG() {
  int64_t soma=0;
  for (uint8_t i=0;i<5;++i) soma+=pesos[i];
  return (static_cast<double>(soma)/5-ajustes.tara)/ajustes.fator;
}

uint32_t calcularChecksum(const Ajustes &v) {
  uint32_t h=2166136261u;
  const uint8_t *p=reinterpret_cast<const uint8_t*>(&v);
  for (size_t i=0;i<offsetof(Ajustes,checksum);++i) h=(h^p[i])*16777619u;
  return h;
}
bool registroValido(const Ajustes &v) {
  return v.assinatura==ASSINATURA && v.versao==VERSAO
    && v.checksum==calcularChecksum(v) && isfinite(v.fator)
    && (v.fator==0 || fabs(v.fator)>=0.000001f)
    && v.tara>-8388607 && v.tara<8388607
    && isfinite(v.doseG) && v.doseG>=0 && v.doseG<=100000;
}

void rele(bool ligar) {
  bombaLigada=ligar;
  digitalWrite(PINO_BOMBA,ligar ? LOW : HIGH);
}
void desligar(const char *razao) {
  rele(false);
  motivo=razao;
}
void bloquear(const char *razao) {
  desligar(razao);
  if (!bloqueado) INFO_PRINTF("EVENTO: bomba BLOQUEADA: %s\r\n",razao);
  bloqueado=true;
  aguardandoConfirmacao = false;
  entrada=coleta=0;
  taraPronta=false;
  nNumero=0;
  numeroLongo=false;
}
bool salvarAjustes(Ajustes v) {
  v.assinatura=ASSINATURA; v.versao=VERSAO;
  v.checksum=calcularChecksum(v);
  if (!memoriaOK || memoria.putBytes("ajustes",&v,sizeof(v))!=sizeof(v)) {
    bloquear("ERRO_AO_SALVAR");
    return false;
  }
  ajustes=v;
  INFO_PRINTLN("EVENTO: ajustes salvos na memoria interna.");
  return true;
}
void mostrarAjustes() {
  INFO_PRINTF("CONFIG: calibrada=%s | fator=%.8f | tara=%ld | dose=%.1f g\r\n",
    calibrada()?"SIM":"NAO",ajustes.fator,static_cast<long>(ajustes.tara),ajustes.doseG);
}
void ajuda() {
  INFO_PRINTLN("\r\nGROW: inicia umidade <30%; para por peso inicial+dose ou umidade >=85%.");
  INFO_PRINTLN("Primeira vez: t vazio; c 1500 + Enter com 1500 g; d 500 + Enter para dose +500 g.");
  INFO_PRINTLN("Salva automaticamente. Reinicios recuperam calibracao, tara e dose.");
  INFO_PRINTLN("w: configurar Wi-Fi/servidor | n: estado da rede e memoria | f: inicializar arquivos");
  INFO_PRINTLN("0/q: parar e bloquear | a: retomar | p: configuracao | g: grafico | m: alternar logs no Telnet | h: ajuda");
}

void mostrarRede() {
  INFO_PRINTF("REDE: dispositivo=%s | Wi-Fi=%s | Portal=%s | HTTP=%d\r\n",
    growDispositivo,growWifi.load()?"CONECTADO":"DESCONECTADO",
    growPortal.load()?"ABERTO (192.168.4.1)":"FECHADO",growHttp.load());
  INFO_PRINTF("HISTORICO: %s | pendentes=%lu | limite=%.0f KiB | livres=%.0f KiB | perdas neste boot=%lu\r\n",
    growEstadoDisco(),static_cast<unsigned long>(growPendentes.load()),
    growCapacidade.load()/1024.0,growBytesLivres.load()/1024.0,
    static_cast<unsigned long>(growPerdidos.load()));
  INFO_PRINTLN("AP: GROW-Config-<ID> | senha inicial grow12345 | w abre por 3 minutos.");
  INFO_PRINTLN("HTTP 200=ultimo envio confirmado; 403=chave errada; -1001=URL/chave invalidas; -1002=erro NVS.");
}

void verificarServicosRede() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!telnetIniciado) {
      TelnetStream.begin();
      telnetIniciado = true;
      INFO_PRINTLN("EVENTO: Servico Telnet iniciado com sucesso.");
    }
    if (!otaIniciado) {
      ArduinoOTA.setHostname("esp32-grow");
      ArduinoOTA.onStart([]() {
        INFO_PRINTLN("OTA: Inicio de atualizacao sem fio...");
      });
      ArduinoOTA.onEnd([]() {
        INFO_PRINTLN("\r\nOTA: Atualizacao concluida com sucesso!");
      });
      ArduinoOTA.onError([](ota_error_t error) {
        INFO_PRINTF("OTA Erro [%u]\r\n", error);
      });
      ArduinoOTA.begin();
      otaIniciado = true;
      INFO_PRINTLN("EVENTO: Servico ArduinoOTA pronto para uploads via Wi-Fi.");
    }
    ArduinoOTA.handle();
  } else {
    telnetIniciado = false;
    otaIniciado = false;
  }
}

void iniciarColeta(char tipo) {
  desligar("CALIBRANDO");
  coleta=tipo; entrada=0; nColeta=0; somaColeta=0; inicioColeta=millis();
}
void receberColeta(long bruto) {
  if (!coleta) return;
  ++nColeta;
  if (nColeta<=3) return;
  somaColeta+=bruto;
  if (nColeta<23) return;
  double media=static_cast<double>(somaColeta)/20;
  char tipo=coleta; coleta=0;
  if (tipo=='T') {
    taraCandidata=static_cast<int32_t>(lround(media));
    taraPronta=true;
    motivo="AGUARDA_MASSA_CALIBRACAO";
    INFO_PRINTLN("EVENTO: tara capturada. Coloque a massa conhecida e envie c 1500 + Enter.");
  } else {
    double delta=media-taraCandidata;
    float fator=static_cast<float>(delta/massaConhecida);
    if (fabs(delta)<1 || !isfinite(fator) || fabs(fator)<0.000001f) {
      bloquear("CALIBRACAO_INVALIDA"); return;
    }
    Ajustes novo=ajustes; novo.fator=fator; novo.tara=taraCandidata;
    if (!salvarAjustes(novo)) return;
    taraPronta=false; rearmado=true; motivo="PRONTO";
    mostrarAjustes();
    INFO_PRINTLN("EVENTO: calibracao concluida; nao precisa repetir no proximo reinicio.");
  }
}
void iniciarNumero(char tipo) {
  if (tipo=='C' && !taraPronta) {
    INFO_PRINTLN("Primeiro envie t com a plataforma vazia."); return;
  }
  desligar("AJUSTANDO");
  coleta=0; entrada=tipo; nNumero=0; numeroLongo=false;
  INFO_PRINTLN(tipo=='C' ? "Informe massa conhecida em gramas + Enter:" : "Informe dose adicional em gramas + Enter:");
}
void aplicarNumero() {
  if (entrada=='F') {
    numero[nNumero]='\0';
    if (strcmp(numero,"12345")==0 && growDisco.load()==GROW_DISCO_SEM_FS) {
      growPedirFormato.store(true);
      INFO_PRINTLN("Inicializacao solicitada. Aguarde e envie n para conferir.");
    } else INFO_PRINTLN("Inicializacao cancelada. Historico existente nao foi apagado.");
    entrada=0; motivo="PRONTO"; return;
  }
  numero[nNumero]='\0';
  for (char *p=numero;*p;++p) if (*p==',') *p='.';
  char *fim=nullptr;
  double valor=strtod(numero,&fim);
  while (*fim && isspace(static_cast<unsigned char>(*fim))) ++fim;
  if (fim==numero || *fim || !isfinite(valor) || valor<0 || valor>100000) {
    INFO_PRINTLN("Valor invalido. Use gramas, maior que zero, ate 100000."); return;
  }
  if (valor==0) { bloquear("PARADA_USUARIO"); return; }
  if (entrada=='C') {
    massaConhecida=valor; iniciarColeta('C');
  } else {
    Ajustes novo=ajustes; novo.doseG=static_cast<float>(valor);
    if (!salvarAjustes(novo)) return;
    entrada=0; motivo="PRONTO";
    INFO_PRINTF("EVENTO: dose definida em +%.1f g para a proxima rega.\r\n",ajustes.doseG);
  }
}

void atenderSerial() {
  bool temSerial = Serial.available() > 0;
  bool temTelnet = telnetIniciado && (WiFi.status() == WL_CONNECTED) && (TelnetStream.available() > 0);

  while (temSerial || temTelnet) {
    char c = temSerial ? Serial.read() : TelnetStream.read();
    
    if ((static_cast<unsigned char>(c) < 32 || static_cast<unsigned char>(c) > 126) && c != '\r' && c != '\n') {
      temSerial = Serial.available() > 0;
      temTelnet = telnetIniciado && (WiFi.status() == WL_CONNECTED) && (TelnetStream.available() > 0);
      continue;
    }

    if (aguardandoConfirmacao) {
      if (c == 'y' || c == 'Y') {
        aguardandoConfirmacao = false;
        double peso = pesoAtualG();
        pesoInicioG = peso;
        pesoAlvoG = pesoInicioG + ajustes.doseG;
        motivo = "IRRIGANDO";
        rele(true);
        INFO_PRINTF("EVENTO: rega AUTORIZADA e iniciada | base=%.1f g | dose=%.1f g | alvo=%.1f g\r\n",pesoInicioG,ajustes.doseG,pesoAlvoG);
      } else if (c == 'n' || c == 'N') {
        aguardandoConfirmacao = false;
        INFO_PRINTLN("EVENTO: rega NEGADA pelo usuario. O sistema perguntara novamente no proximo ciclo de leitura.");
      } else if (c != '\r' && c != '\n') {
        INFO_PRINTF("ALERTA: Solo seco (%.1f%%). Rega PENDENTE de confirmacao!\r\n", umidadeRelativa());
        INFO_PRINTLN("Envie 'Y' para autorizar a rega ou 'N' para cancelar.");
      }
      continue;
    }
    
    if (entrada) {
      if (c=='q' || c=='Q') bloquear("PARADA_USUARIO");
      else if (c=='\n' || c=='\r') {
        if (numeroLongo) INFO_PRINTLN("Numero longo demais. Tente novamente.");
        else if (nNumero) aplicarNumero();
        nNumero=0; numeroLongo=false;
      } else if (!numeroLongo) {
        if (nNumero<sizeof(numero)-1) numero[nNumero++]=c;
        else numeroLongo=true;
      }
      continue;
    }
    if (isspace(static_cast<unsigned char>(c))) continue;
    switch (tolower(static_cast<unsigned char>(c))) {
      case '0': case 'q': bloquear("PARADA_USUARIO"); break;
      case 't':
        taraPronta=false; iniciarColeta('T');
        INFO_PRINTLN("EVENTO: capturando tara; plataforma vazia e parada."); break;
      case 'c': iniciarNumero('C'); break;
      case 'd': iniciarNumero('D'); break;
      case 'a':
        if (!calibrada() || ajustes.doseG<=0 || emAjuste()
            || !pesoDisponivel() || !adcDisponivel() || tensaoMv()<300 || tensaoMv()>2800) {
          INFO_PRINTLN("Conclua ajustes e confira os sensores antes de retomar.");
        } else { bloqueado=false; rearmado=true; motivo="RETOMADO"; }
        break;
      case 'w':
        growPedirPortal.store(true);
        INFO_PRINTLN("Abrindo GROW-Config-<ID>. Senha grow12345; pagina 192.168.4.1 por 3 minutos."); break;
      case 'n': mostrarRede(); break;
      case 'f':
        if (growDisco.load()!=GROW_DISCO_SEM_FS) {
          INFO_PRINTLN("Formatacao recusada: somente volume nao montado pode ser inicializado.");
        } else {
          desligar("INICIALIZANDO_ARQUIVOS"); entrada='F'; nNumero=0; numeroLongo=false;
          INFO_PRINTLN("ATENCAO: apaga SOMENTE a particao de arquivos. Calibracao NVS e preservada.");
          INFO_PRINTLN("Somente na primeira instalacao, digite 12345 + Enter; q cancela.");
        }
        break;
      case 'g': modoPlotter=true; break;
      case 'm': 
        telnetLogContinuo = !telnetLogContinuo;
        INFO_PRINTF("EVENTO: Logs continuos no Telnet %s.\r\n", telnetLogContinuo ? "ATIVADOS" : "DESATIVADOS");
        break;
      case 'p': mostrarAjustes(); break;
      case 'h': ajuda(); break;
      default: INFO_PRINTLN("Comando desconhecido. Envie h."); break;
    }

    temSerial = Serial.available() > 0;
    temTelnet = telnetIniciado && (WiFi.status() == WL_CONNECTED) && (TelnetStream.available() > 0);
  }
}

void atualizarSensores() {
  unsigned long agora=millis();
  if (coleta && agora-inicioColeta>6000) bloquear("TIMEOUT_CALIBRACAO");
  if (agora-ultimoAdc>=ADC_MS) {
    uint32_t mv=analogReadMilliVolts(PINO_UMIDADE);
    if (nAdc==25) somaMv-=umidades[posAdc]; else ++nAdc;
    umidades[posAdc]=mv; somaMv+=mv; posAdc=(posAdc+1)%25; ultimoAdc=agora;
  }
  if (balanca.is_ready()) {
    long bruto=balanca.read();
    if (bruto<=-8388607L || bruto>=8388607L) {
      nPeso=0; bloquear("HX711_SATURADO"); return;
    }
    pesos[posPeso]=bruto; posPeso=(posPeso+1)%5;
    if (nPeso<5) ++nPeso;
    ultimoPeso=agora; receberColeta(bruto);
  }
}

void controlarRega() {
  if (bloqueado || emAjuste() || !calibrada() || ajustes.doseG<=0) return;
  if (!pesoDisponivel() || !adcDisponivel()) return;
  float u=umidadeRelativa(); double peso=pesoAtualG();
  
  if (bombaLigada) {
    if (u>=UMIDADE_PARAR || peso>=pesoAlvoG) {
      desligar(u>=UMIDADE_PARAR ? "UMIDADE_85" : "PESO_ALVO");
      rearmado=false;
      pesoPosRegaG = peso;
      INFO_PRINTF("EVENTO: rega concluida por %s | peso=%.1f g | umidade=%.1f%%\r\n",motivo,peso,u);
    }
  } else {
    // REARME COM UMIDADE < 30%
    if (u < UMIDADE_INICIAR) rearmado=true;
    
    if (rearmado && u < UMIDADE_INICIAR) {
      // VALIDAÇÃO DE PESO INTEGRALMENTE MANTIDA
      if (pesoPosRegaG == 0.0 || peso <= pesoPosRegaG - (ajustes.doseG * 0.90)) {
        if (!aguardandoConfirmacao) {
          aguardandoConfirmacao = true;
          motivo="AGUARDANDO_CONFIRMACAO";
          ultimoAvisoConfirmacao = millis();
          INFO_PRINTF("ALERTA: Solo seco (%.1f%%). A rega e necessaria.\r\n", u);
          INFO_PRINTLN("Deseja iniciar a rega agora? Envie 'Y' para confirmar ou 'N' para cancelar.");
        } else {
          if (millis() - ultimoAvisoConfirmacao >= 5000) {
            ultimoAvisoConfirmacao = millis();
            INFO_PRINTF("ALERTA: Solo seco (%.1f%%). Rega PENDENTE de autorizacao!\r\n", u);
            INFO_PRINTLN("Envie 'Y' para confirmar a rega ou 'N' para cancelar.");
          }
        }
      } else {
        if (!aguardandoConfirmacao) motivo="AGUARDA_QUEDA_PESO";
      }
    }
  }
}

void verificarFalhas() {
  if (bloqueado || emAjuste()) return;
  if (nPeso==5 && millis()-ultimoPeso>PESO_TIMEOUT) {
    nPeso=0; bloquear("HX711_SEM_LEITURA");
  }
  if (nAdc==25 && (millis()-ultimoAdc>ADC_TIMEOUT || tensaoMv()<300 || tensaoMv()>2800))
    bloquear("SENSOR_UMIDADE_INVALIDO");
}

void logSerial() {
  if (millis()-ultimoLog < LOG_MS) return;
  ultimoLog=millis();

  if (modoPlotter) {
    if (pesoDisponivel() && adcDisponivel() && tensaoMv()>=300 && tensaoMv()<=2800) {
      Serial.print("Peso_g:"); Serial.print(pesoAtualG(),1);
      Serial.print("\tUmidade_pct:"); Serial.println(umidadeRelativa(),1);
    }
    return;
  }

  char buf[256];
  char strPeso[16], strUmid[32];
  
  if (pesoDisponivel()) snprintf(strPeso, sizeof(strPeso), "%.1fg", pesoAtualG());
  else strcpy(strPeso, "INDISP");

  if (adcDisponivel()) snprintf(strUmid, sizeof(strUmid), "%.1f%%(%.2fV)", umidadeRelativa(), tensaoMv()/1000.0f);
  else strcpy(strUmid, "INDISP");

  snprintf(buf, sizeof(buf),
    "[%.1fs] P:%s | U:%s | Bomba:%s | St:%s | Wi-Fi:%s\r\n",
    millis()/1000.0, strPeso, strUmid, bombaLigada?"ON":"OFF", motivo, growWifi.load()?"OK":"OFF");

  Serial.print(buf);

  if (telnetIniciado && WiFi.status() == WL_CONNECTED && telnetLogContinuo) {
    TelnetStream.print(buf);
  }
}

void setup() {
  gpio_set_level(static_cast<gpio_num_t>(PINO_BOMBA),HIGH);
  pinMode(PINO_BOMBA,OUTPUT); rele(false);
  Serial.begin(115200);
  
  pinMode(PINO_UMIDADE,INPUT);
  analogReadResolution(12); analogSetPinAttenuation(PINO_UMIDADE,ADC_11db);
  balanca.begin(PINO_DOUT,PINO_SCK);
  memoriaOK=memoria.begin("grow-v2",false);
  Ajustes salvo={};
  if (memoriaOK && memoria.getBytesLength("ajustes")==sizeof(salvo)
      && memoria.getBytes("ajustes",&salvo,sizeof(salvo))==sizeof(salvo)
      && registroValido(salvo)) ajustes=salvo;
  delay(500);
  motivo=calibrada()?"PRONTO":"FALTA_CALIBRACAO";
  if (!memoriaOK) bloquear("MEMORIA_INDISPONIVEL");
  
  growIniciarRede();
  ajuda();
  mostrarAjustes();
}

void loop() {
  verificarServicosRede();
  atenderSerial();
  verificarFalhas();
  atualizarSensores();
  verificarFalhas();
  controlarRega();
  
  unsigned long intervaloRegistro = bombaLigada ? REGISTRO_MS_LIGADA : REGISTRO_MS_DESLIGADA;
  if (millis() - ultimoRegistro >= intervaloRegistro) {
    ultimoRegistro = millis();
    growRegistrar(pesoDisponivel(),pesoDisponivel()?pesoAtualG():0,
      adcDisponivel() && tensaoMv()>=300 && tensaoMv()<=2800,
      tensaoMv(),umidadeRelativa(),bombaLigada);
  }

  logSerial();
  delay(1);
}