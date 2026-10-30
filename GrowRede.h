#pragma once
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager, por tzapu.
#include <HTTPClient.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>
#include <algorithm>
#include <vector>
#include <time.h>
#include <math.h>
#include <string.h>
#include "GrowRegistro.h"

// A rede usa uma tarefa separada. Nenhum HTTP ou arquivo no loop da bomba.
// Wi-Fi de 2,4 GHz para ESP32 classico. HTTP apenas na rede local.
const char *GROW_AP_SENHA="grow12345"; // Troque antes de distribuir o produto.
const uint32_t GROW_HISTORICO_MS=30000;
const uint32_t GROW_LIMITE_BYTES=1024u*1024u;
const size_t GROW_POR_ARQUIVO=64;
enum { GROW_DISCO_INICIANDO, GROW_DISCO_OK, GROW_DISCO_SEM_FS,
       GROW_DISCO_CHEIO, GROW_DISCO_ERRO, GROW_DISCO_CORROMPIDO };

std::atomic<uint32_t> growPendentes{0}, growBytesLivres{0}, growCapacidade{0};
std::atomic<uint32_t> growPerdidos{0}; // Contador desde este boot.
std::atomic<int> growDisco{GROW_DISCO_INICIANDO}, growHttp{0};
std::atomic<bool> growWifi{false}, growPortal{false}, growPedirPortal{false};
std::atomic<bool> growPedirFormato{false};
QueueHandle_t growFila=nullptr;
uint64_t growBoot=0;
uint32_t growSequencia=0, growUltimaAmostra=0;
char growDispositivo[13]={};

struct GrowConfigRede {
  uint32_t assinatura;
  char url[128];
  char chave[65];
  uint32_t checksum;
};
inline uint32_t growConfigCRC(const GrowConfigRede &c) {
  uint32_t h=2166136261u;
  const uint8_t *p=reinterpret_cast<const uint8_t*>(&c);
  for (size_t i=0;i<offsetof(GrowConfigRede,checksum);++i) h=(h^p[i])*16777619u;
  return h;
}

// Somente a tarefa de rede acessa estes objetos e o sistema de arquivos.
class GrowComunicacao {
  Preferences prefs;
  WiFiManager wm;
  WiFiManagerParameter campoUrl{"servidor", "Servidor: http://IP-DO-PC:8765/dados", "", 128};
  WiFiManagerParameter campoChave{"chave", "Chave exibida pelo servidor Windows", "", 65};
  GrowConfigRede config{};
  std::vector<uint32_t> arquivos;
  uint32_t proximoArquivo=1, ultimoEnvio=0, ultimaReconexao=0;
  bool fsOK=false, prefsOK=false, salvarCampos=false, horarioIniciado=false;
  char nomeAP[32]={};

  void caminho(uint32_t id, char *dest) {
    snprintf(dest,24,"/fila/q%08lx.bin",static_cast<unsigned long>(id));
  }
  void espaco() {
    if (!fsOK) return;
    const size_t total=LittleFS.totalBytes(), usado=LittleFS.usedBytes();
    growBytesLivres.store(total>usado ? total-usado : 0);
    // Usa no maximo 1 MiB de payload e reserva 25% do volume ao filesystem.
    growCapacidade.store(std::min(static_cast<uint32_t>(total*3/4),GROW_LIMITE_BYTES));
  }
  bool montar() {
    // NUNCA formatar automaticamente: uma falha nao deve apagar o historico.
    fsOK=LittleFS.begin(false);
    arquivos.clear(); growPendentes.store(0);
    if (!fsOK) { growDisco.store(GROW_DISCO_SEM_FS); return false; }
    if (!LittleFS.exists("/fila") && !LittleFS.mkdir("/fila")) {
      growDisco.store(GROW_DISCO_ERRO); return false;
    }
    File dir=LittleFS.open("/fila");
    if (!dir || !dir.isDirectory()) { growDisco.store(GROW_DISCO_ERRO); return false; }
    for (File f=dir.openNextFile();f;f=dir.openNextFile()) {
      const char *nome=strrchr(f.name(),'/'); nome=nome?nome+1:f.name();
      unsigned long id=0; int fim=0;
      if (sscanf(nome,"q%8lx.bin%n",&id,&fim)==1 && fim==13 && nome[fim]=='\0')
        arquivos.push_back(static_cast<uint32_t>(id));
      f.close();
    }
    dir.close(); std::sort(arquivos.begin(),arquivos.end());
    proximoArquivo=arquivos.empty()?1:arquivos.back()+1;
    // Confere CRC e tamanho antes de enviar ou continuar a fila apos reboot.
    for (uint32_t id:arquivos) {
      char nome[24]; caminho(id,nome); File f=LittleFS.open(nome,FILE_READ);
      if (!f || f.size()%sizeof(GrowRegistro) || f.size()>GROW_POR_ARQUIVO*sizeof(GrowRegistro)) {
        growDisco.store(GROW_DISCO_CORROMPIDO); return false;
      }
      size_t quantidade=f.size()/sizeof(GrowRegistro);
      for (size_t i=0;i<quantidade;++i) {
        GrowRegistro r{};
        if (f.read(reinterpret_cast<uint8_t*>(&r),sizeof(r))!=sizeof(r) || !growRegistroValido(r)) {
          f.close(); growDisco.store(GROW_DISCO_CORROMPIDO); return false;
        }
      }
      f.close(); growPendentes.fetch_add(quantidade);
    }
    growDisco.store(GROW_DISCO_OK); espaco(); return true;
  }
  bool gravar(const GrowRegistro &r) {
    int estado=growDisco.load();
    if (!fsOK || (estado!=GROW_DISCO_OK && estado!=GROW_DISCO_CHEIO)) return false;
    espaco();
    if ((growPendentes.load()+1)*sizeof(r)>growCapacidade.load() || growBytesLivres.load()<16384) {
      growDisco.store(GROW_DISCO_CHEIO); return false; // Preserva os antigos.
    }
    uint32_t id=proximoArquivo;
    if (!arquivos.empty()) {
      char ultimo[24]; caminho(arquivos.back(),ultimo);
      File f=LittleFS.open(ultimo,FILE_READ);
      if (!f) { growDisco.store(GROW_DISCO_ERRO); return false; }
      if (f.size()<GROW_POR_ARQUIVO*sizeof(r)) id=arquivos.back();
      f.close();
    }
    char nome[24]; caminho(id,nome);
    File f=LittleFS.open(nome,FILE_APPEND);
    if (!f) { growDisco.store(GROW_DISCO_ERRO); return false; }
    size_t escritos=f.write(reinterpret_cast<const uint8_t*>(&r),sizeof(r));
    f.flush(); f.close();
    // Reabre para conferir o registro antes de contabiliza-lo como gravado.
    f=LittleFS.open(nome,FILE_READ); GrowRegistro conferido{};
    bool ok=escritos==sizeof(r) && f && f.size()>=sizeof(r)
      && f.size()%sizeof(r)==0 && f.seek(f.size()-sizeof(r))
      && f.read(reinterpret_cast<uint8_t*>(&conferido),sizeof(r))==sizeof(r)
      && memcmp(&r,&conferido,sizeof(r))==0 && growRegistroValido(conferido);
    f.close();
    if (!ok) { growDisco.store(GROW_DISCO_ERRO); return false; }
    if (id==proximoArquivo) { arquivos.push_back(id); ++proximoArquivo; }
    growPendentes.fetch_add(1); growDisco.store(GROW_DISCO_OK); espaco(); return true;
  }
  void salvarConfiguracao() {
    salvarCampos=false;
    GrowConfigRede nova{}; nova.assinatura=0x47524E31;
    strlcpy(nova.url,campoUrl.getValue(),sizeof(nova.url));
    strlcpy(nova.chave,campoChave.getValue(),sizeof(nova.chave));
    // Servidor local: URL completa terminando em /dados; nao segue redirects.
    String url(nova.url); url.trim();
    if (!url.startsWith("http://") || !url.endsWith("/dados") || strlen(nova.chave)<16) {
      growHttp.store(-1001); return;
    }
    strlcpy(nova.url,url.c_str(),sizeof(nova.url));
    nova.checksum=growConfigCRC(nova);
    if (!prefsOK || prefs.putBytes("config",&nova,sizeof(nova))!=sizeof(nova)) {
      growHttp.store(-1002); return;
    }
    config=nova; growHttp.store(0);
  }
  void enviar() {
    if (arquivos.empty() || !config.url[0] || !config.chave[0]) return;
    const int estado=growDisco.load();
    if (estado!=GROW_DISCO_OK && estado!=GROW_DISCO_CHEIO) return;
    char nome[24]; caminho(arquivos.front(),nome); File f=LittleFS.open(nome,FILE_READ);
    if (!f) { growDisco.store(GROW_DISCO_ERRO); return; }
    size_t n=f.size()/sizeof(GrowRegistro);
    if (f.size()%sizeof(GrowRegistro) || n>GROW_POR_ARQUIVO) {
      f.close(); growDisco.store(GROW_DISCO_CORROMPIDO); return;
    }
    if (!n) { // Arquivo vazio deixado por uma interrupcao antes da gravacao.
      f.close();
      if (LittleFS.remove(nome)) arquivos.erase(arquivos.begin());
      return;
    }
    String json; json.reserve(n*210+64);
    json="{\"dispositivo\":\""; json+=growDispositivo; json+="\",\"registros\":[";
    char ultimoID[48]={};
    for (size_t i=0;i<n;++i) {
      GrowRegistro r{};
      if (f.read(reinterpret_cast<uint8_t*>(&r),sizeof(r))!=sizeof(r) || !growRegistroValido(r)) {
        f.close(); growDisco.store(GROW_DISCO_CORROMPIDO); return;
      }
      char boot[17], peso[32], adc[48], item[280];
      snprintf(boot,sizeof(boot),"%08lx%08lx",static_cast<unsigned long>(r.boot>>32),static_cast<unsigned long>(static_cast<uint32_t>(r.boot)));
      snprintf(ultimoID,sizeof(ultimoID),"%s-%lu",boot,static_cast<unsigned long>(r.sequencia));
      if ((r.flags&GROW_PESO_OK) && isfinite(r.pesoG)) snprintf(peso,sizeof(peso),"%.3f",r.pesoG);
      else strlcpy(peso,"null",sizeof(peso));
      if (r.flags&GROW_ADC_OK) snprintf(adc,sizeof(adc),"\"mv\":%u,\"umidade10\":%u",r.tensaoMv,r.umidade10);
      else strlcpy(adc,"\"mv\":null,\"umidade10\":null",sizeof(adc));
      snprintf(item,sizeof(item),"%s{\"boot\":\"%s\",\"seq\":%lu,\"epoch\":%lu,\"ligado_s\":%lu,\"peso_g\":%s,%s,\"bomba\":%s}",
        i?",":"",boot,static_cast<unsigned long>(r.sequencia),static_cast<unsigned long>(r.epoch),
        static_cast<unsigned long>(r.ligadoS),peso,adc,(r.flags&GROW_BOMBA)?"true":"false");
      json+=item;
    }
    f.close(); json+="]}";
    WiFiClient cliente; HTTPClient http;
    http.setConnectTimeout(1500); http.setTimeout(5000); http.setReuse(false);
    if (!http.begin(cliente,String(config.url))) { growHttp.store(-1003); return; }
    http.addHeader("Content-Type","application/json"); http.addHeader("X-Grow-Key",String(config.chave));
    int codigo=http.POST(json); growHttp.store(codigo);
    String resposta=(codigo==200 && http.getSize()>=0 && http.getSize()<100)?http.getString():String("");
    resposta.trim(); http.end();
    String esperado="OK "; esperado+=ultimoID;
    // Um HTTP 200 generico nao basta: exige a confirmacao deste lote.
    if (codigo==200 && resposta==esperado) {
      if (!LittleFS.remove(nome)) { growDisco.store(GROW_DISCO_ERRO); return; }
      arquivos.erase(arquivos.begin()); growPendentes.fetch_sub(n);
      growDisco.store(GROW_DISCO_OK); espaco();
    }
  }
public:
  void iniciar() {
    snprintf(nomeAP,sizeof(nomeAP),"GROW-Config-%.6s",growDispositivo+6);
    prefsOK=prefs.begin("grow-net",false);
    GrowConfigRede salva{};
    if (prefsOK && prefs.getBytesLength("config")==sizeof(salva)
        && prefs.getBytes("config",&salva,sizeof(salva))==sizeof(salva)
        && salva.assinatura==0x47524E31 && salva.checksum==growConfigCRC(salva)
        && memchr(salva.url,0,sizeof(salva.url)) && memchr(salva.chave,0,sizeof(salva.chave))) config=salva;
    campoUrl.setValue(config.url,128); campoChave.setValue(config.chave,65);
    wm.setDebugOutput(false); // Nao poluir o Plotter nem imprimir credenciais.
    wm.setConfigPortalBlocking(false); wm.setConfigPortalTimeout(180);
    wm.setConnectTimeout(8); wm.setSaveParamsCallback([this](){salvarCampos=true;});
    wm.setSaveConfigCallback([this](){salvarCampos=true;});
    wm.addParameter(&campoUrl); wm.addParameter(&campoChave);
    montar();
    WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true);
    wm.autoConnect(nomeAP,GROW_AP_SENHA);
    ultimoEnvio=millis()-30000u;
  }
  void passo() {
    wm.process();
    if (salvarCampos) salvarConfiguracao();
    if (growPedirPortal.exchange(false) && !wm.getConfigPortalActive()) {
      campoUrl.setValue(config.url,128); campoChave.setValue(config.chave,65);
      wm.startConfigPortal(nomeAP,GROW_AP_SENHA);
    }
    // Formatar exige comando explicito e somente se o volume nao esta montado.
    if (growPedirFormato.exchange(false) && !fsOK) {
      if (LittleFS.format()) montar(); else growDisco.store(GROW_DISCO_ERRO);
    }
    GrowRegistro r{};
    // Enquanto aguarda inicializacao do filesystem, conserva a fila em RAM.
    if (fsOK) while (xQueueReceive(growFila,&r,0)==pdTRUE) if (!gravar(r)) growPerdidos.fetch_add(1);
    bool conectado=WiFi.status()==WL_CONNECTED;
    growWifi.store(conectado); growPortal.store(wm.getConfigPortalActive());
    if (conectado && !horarioIniciado) {
      configTime(0,0,"pool.ntp.org","time.google.com"); horarioIniciado=true;
    }
    if (!conectado && !wm.getConfigPortalActive() && millis()-ultimaReconexao>=30000) {
      ultimaReconexao=millis(); WiFi.reconnect();
    }
    uint32_t intervalo=(growHttp.load()==200)?500u:30000u;
    if (conectado && !wm.getConfigPortalActive() && millis()-ultimoEnvio>=intervalo) {
      ultimoEnvio=millis(); enviar();
    }
  }
};

inline void growTarefaRede(void *) {
  GrowComunicacao rede; rede.iniciar();
  for (;;) { rede.passo(); vTaskDelay(pdMS_TO_TICKS(20)); }
}
inline void growIniciarRede() {
  uint64_t mac=ESP.getEfuseMac();
  snprintf(growDispositivo,sizeof(growDispositivo),"%012llX",static_cast<unsigned long long>(mac&0xFFFFFFFFFFFFULL));
  growBoot=(static_cast<uint64_t>(esp_random())<<32)|esp_random();
  growFila=xQueueCreate(16,sizeof(GrowRegistro));
  if (!growFila || xTaskCreate(growTarefaRede,"grow-rede",12288,nullptr,1,nullptr)!=pdPASS) {
    growDisco.store(GROW_DISCO_ERRO);
    if (growFila) { vQueueDelete(growFila); growFila=nullptr; }
  }
  growUltimaAmostra=millis();
}
inline void growRegistrar(bool pesoOK, double peso, bool adcOK, float mv, float umidade, bool bomba) {
  if (millis()-growUltimaAmostra<GROW_HISTORICO_MS) return;
  growUltimaAmostra=millis();
  GrowRegistro r{}; r.boot=growBoot; r.sequencia=++growSequencia; r.versao=1;
  time_t agora=time(nullptr); r.epoch=agora>=1704067200?static_cast<uint32_t>(agora):0;
  r.ligadoS=static_cast<uint32_t>(esp_timer_get_time()/1000000LL);
  if (pesoOK && isfinite(peso)) { r.flags|=GROW_PESO_OK; r.pesoG=static_cast<float>(peso); }
  if (adcOK) { r.flags|=GROW_ADC_OK; r.tensaoMv=static_cast<uint16_t>(lroundf(mv)); r.umidade10=static_cast<uint16_t>(lroundf(umidade*10)); }
  if (bomba) r.flags|=GROW_BOMBA;
  r.crc=growCRC(r);
  if (!growFila || xQueueSend(growFila,&r,0)!=pdTRUE) growPerdidos.fetch_add(1);
}
inline const char *growEstadoDisco() {
  switch (growDisco.load()) {
    case GROW_DISCO_INICIANDO: return "INICIANDO";
    case GROW_DISCO_OK: return "OK";
    case GROW_DISCO_SEM_FS: return "SEM_FS: f + 12345 para inicializar (apaga particao de arquivos)";
    case GROW_DISCO_CHEIO: return "CHEIO: registros novos nao estao sendo salvos";
    case GROW_DISCO_CORROMPIDO: return "CORROMPIDO: preservado, sincronizacao interrompida";
    default: return "ERRO: historico indisponivel";
  }
}
