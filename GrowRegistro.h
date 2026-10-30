#pragma once
#include <stdint.h>
#include <stddef.h>

// Formato interno: 32 bytes. O CSV e criado pelo servidor Windows.
struct GrowRegistro {
  uint64_t boot;
  uint32_t epoch, ligadoS, sequencia;
  float pesoG;
  uint16_t tensaoMv, umidade10;
  uint8_t flags, versao;
  uint16_t crc;
};
static_assert(sizeof(GrowRegistro)==32, "Registro deve ocupar 32 bytes");
enum { GROW_PESO_OK=1, GROW_ADC_OK=2, GROW_BOMBA=4 };

inline uint16_t growCRC(const GrowRegistro &r) {
  uint16_t crc=0xFFFF;
  const uint8_t *p=reinterpret_cast<const uint8_t*>(&r);
  for (size_t i=0;i<offsetof(GrowRegistro,crc);++i) {
    crc^=static_cast<uint16_t>(p[i])<<8;
    for (uint8_t b=0;b<8;++b) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;
  }
  return crc;
}
inline bool growRegistroValido(const GrowRegistro &r) {
  return r.versao==1 && r.sequencia>0 && r.crc==growCRC(r);
}
