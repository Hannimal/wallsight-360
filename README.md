# WallSight 360

**Mapeia os contornos da tua divisão com LiDAR e acompanha movimento com mmWave.**

> Radar de interior experimental, construído com ESP32-S3, LiDAR LD19, mmWave LD2450 e ecrã TFT. Deteta *superfícies candidatas a paredes*, guarda o mapa e apresenta até três alvos mmWave em tempo real.

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--S3-orange)](https://platformio.org/) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

**Versão 0.2 · PlatformIO · Protótipo experimental · ESP32-S3 DevKitC-1 N8R8 + LDROBOT LD19 + HLK-LD2450 + TFT SPI ILI9341 320×240**

Projeto autónomo para mapear os contornos visíveis de uma divisão, extrair **segmentos retos candidatos a paredes**, guardar o mapa na memória flash e sobrepor até três alvos detetados pelo radar mmWave. O LiDAR é estacionário: não existe SLAM, deslocamento do sensor, classificação de paredes garantida ou deteção humana mmWave a 360°.

## 1. Materiais

| Material | Qtd. | Observações |
|---|---:|---|
| ESP32-S3 DevKitC-1 **N8R8** | 1 | 8 MB flash / 8 MB PSRAM |
| LDROBOT **LD19** com cabo de 4 vias | 1 | LD19 genuíno; alimentação 5 V; saída UART TTL 3,3 V, 230400 baud |
| Hi-Link **HLK-LD2450** | 1 | Alimentação 5 V; UART TTL 3,3 V, 256000 baud |
| ILI9341 **SPI** 2,8" 320×240 | 1 | Escolher módulo SPI com lógica 3,3 V (não variante paralela) |
| Botão momentâneo normalmente aberto | 1 | Uma extremidade ao GPIO47, outra ao GND |
| Fonte 5 V / 2 A e repartição de 5 V | 1 | Alimentação estável; proteger contra ligação invertida |
| Fios, conectores, caixa não metálica | — | Inclui cabo LD19 apropriado |

Atenção: o nome ILI9341 refere-se ao controlador, e módulos com o mesmo nome podem ter **VCC/LED/backlight diferentes**. Confirmar o esquema do módulo comprado, sobretudo antes de alimentar o LED e VCC. NUNCA ligar 5 V a entradas GPIO do ESP32.

## 2. Ligações

| Origem | Destino | Observações |
|---|---|---|
| LD19 TX | ESP32 GPIO15 | UART2 RX, apenas receção |
| LD19 PWM | **Em aberto (Hi-Z)** | LD19 genuíno: controlo interno do motor; verificar variante |
| LD19 P5V | Alimentação 5 V | Ramal de 5 V regulados |
| LD19 GND | GND comum | — |
| LD2450 TX | ESP32 GPIO4 | UART1 RX |
| LD2450 RX | ESP32 GPIO5 | UART1 TX, 3,3 V |
| LD2450 5V | Alimentação 5 V | Ramal de 5 V regulados |
| LD2450 GND | GND comum | — |
| TFT SCK / CLK | ESP32 GPIO12 | SPI clock |
| TFT SDI / SDA / MOSI | ESP32 GPIO11 | SPI data |
| TFT CS | ESP32 GPIO10 | — |
| TFT DC / RS | ESP32 GPIO9 | — |
| TFT RST | ESP32 GPIO8 | — |
| TFT VCC | 3,3 V **se compatível** | Confirmar módulo exato |
| TFT GND | GND comum | — |
| TFT LED / BL | Conforme documentação do TFT | Alguns módulos requerem limitação de corrente |
| TFT MISO / SDO | Sem ligação | Não é necessário para este firmware |
| Botão | GPIO47 + GND | INPUT_PULLUP, sem resistor externo |

**Alimentação:** usar uma única fonte 5 V regulada e uma topologia de distribuição adequada (USB-C ao ESP32 e 5 V aos sensores). Não usar várias fontes de 5 V em paralelo sem isolamento e não tentar alimentar os dois sensores pelo pino 3V3 do ESP32. Todos os GND devem estar comuns.

**Montagem:** fixar o LD19 horizontalmente, com visão livre de 360°. Fixar o LD2450 na mesma base e apontar o seu eixo frontal para a direção de 0° definida para o mapa. Idealmente colocar o LiDAR à altura das superfícies verticais que queres medir, sem pernas de mesas/sofás a obstruir o plano.

Ver `esquema_ligacoes.svg`.

## 3. PlatformIO — compilar, flash e monitor série

**Todo o projeto de firmware é PlatformIO**, não requer abrir ficheiros `.ino` nem configurar bibliotecas manualmente na Arduino IDE. O firmware utiliza Arduino como *framework*, mas a compilação, gestão de dependências e upload são feitos pelo PlatformIO.

Estrutura:

```text
wallsight-360/
├── platformio.ini         # Placa, flash, PSRAM, bibliotecas e monitor
├── src/
│   └── main.cpp           # Firmware principal
├── include/
│   ├── radar_protocol.h   # Protocolo LD19 e LD2450
│   └── wall_map.h         # Mapeamento e persistência
├── tests/
│   ├── protocol_tests.cpp # Testes C++ sem hardware
│   └── map_tests.cpp
├── esquema_ligacoes.svg
└── README.md
```

No VS Code, instalar a extensão **PlatformIO IDE**, abrir esta pasta como projeto e executar **Build** / **Upload** / **Monitor**, ou no terminal:

```bash
pio run -e esp32s3-n8r8
pio run -e esp32s3-n8r8 -t upload
pio device monitor -b 115200
```

A configuração assume **ESP32-S3-WROOM-1 N8R8** (8 MB flash QIO + 8 MB PSRAM OPI), com `board_build.arduino.memory_type = qio_opi`, flags de PSRAM e monitor pela **porta USB nativa**. Se usares a porta USB-UART em vez da USB nativa, ajusta `ARDUINO_USB_CDC_ON_BOOT=0`. Pode ser necessário selecionar explicitamente `upload_port` / `monitor_port` para a tua porta.

Dependências `Adafruit GFX Library`, `Adafruit ILI9341` e `Adafruit BusIO` são instaladas automaticamente pelo PlatformIO através de `lib_deps`.

**Estado:** os testes de lógica C++ foram executados sem hardware. Não foi possível executar `pio run` neste ambiente, pois o PlatformIO CLI não está instalado. A compilação ESP32-S3 e os testes com sensores e ecrã reais permanecem por validar.

## 4. Interface — modos

### SCAN (primeira utilização)

- Se não houver mapa guardado, o equipamento arranca em `SCAN`.
- O LiDAR recolhe distâncias e distribui as medições por **360 setores de 1°**.
- O firmware só aceita uma nova superfície depois de observações compatíveis em várias passagens; as alterações em paredes já conhecidas exigem mais observações para serem aceites.
- Alvos detetados pelo mmWave são excluídos do mapa numa área de aproximadamente 50 cm, **após calibração**.
- Segmentos retos com comprimento mínimo de 85 cm, continuidade e desvio geométrico limitado aparecem a cor clara. São **candidatos a paredes**, também podendo ser armários ou mobiliário.
- Deixar o sensor imóvel e esperar que apareça uma proporção significativa dos pontos estáveis. **Pressionar uma vez o botão:** guarda mapa na flash e muda para `LIVE`.

### LIVE

- O LiDAR continua ligado e a transmitir, mas o mapa guardado deixa de ser alterado.
- Até três alvos mmWave são desenhados sobre o mesmo referencial XY, ajustado pela calibração angular e offsets.
- **Pressionar uma vez:** volta a `SCAN`, preservando o mapa atual para melhoramento posterior.
- **Pressionar e manter 2,5 segundos:** elimina o mapa guardado e começa um novo `SCAN`.
- Quando o equipamento reinicia com um mapa válido guardado, arranca em `LIVE`.

**Monitor série:** `s` = guardar mapa / alternar modo; `n` = apagar e recomeçar; `e` = exportar os **360 valores de distância (mm)** em JSON entre `MAP_JSON_BEGIN` e `MAP_JSON_END`. A exportação inclui pontos de superfícies, não polígonos editáveis.

**Atenção à memória flash:** o firmware guarda apenas quando o utilizador pede para congelar o mapa, evitando escritas repetidas. Se se cortar energia durante SCAN antes de guardar, as alterações não guardadas desaparecem.

## 5. Calibração dos sensores

No início de `src/main.cpp`:

```cpp
constexpr float LIDAR_ROTATION_DEG=0.0f;
constexpr float RADAR_ROTATION_DEG=0.0f;
constexpr float RADAR_X_SIGN=1.0f;
constexpr float RADAR_X_OFFSET_MM=0.0f;
constexpr float RADAR_Y_OFFSET_MM=0.0f;
constexpr float DISPLAY_RANGE_MM=6000.f;
```

**Procedimento:**
1. Colocar sensor numa sala com uma parede frontal reconhecível, e marcar fisicamente a frente do módulo LD2450.
2. Confirmar que a parede em frente aparece na parte superior do ecrã. Se não, ajustar `LIDAR_ROTATION_DEG`.
3. Fazer um percurso da esquerda para a direita na frente do radar. Se o alvo andar ao contrário no ecrã, trocar `RADAR_X_SIGN` de `1.0` para `-1.0`.
4. Ajustar `RADAR_ROTATION_DEG` até os alvos e o eixo frontal terem a mesma orientação.
5. Medir a distância entre os centros dos sensores e preencher os dois offsets em mm.
6. **Apagar o mapa antigo após alterar a rotação do LiDAR** (`n`), porque as medições anteriores estavam noutro referencial.

A distância de ecrã padrão é 6 metros **desde o centro até à margem circular**, embora o LD19 possa medir mais longe. Ajustar `DISPLAY_RANGE_MM` se a divisão for maior; segmentos fora do círculo não são mostrados.

## 6. Como se estima uma parede

1. Leitura e validação CRC dos frames LiDAR LD19 (47 bytes, 12 amostras).
2. Conversão ângulo/distância → coordenadas (x,y) no plano do LiDAR.
3. Agrupamento temporal em setores angulares, com tolerância para variações de medição.
4. Rejeição de alterações rápidas e de pontos sobre alvos conhecidos pelo LD2450.
5. Agrupamento de pontos vizinhos próximos e aproximadamente colineares (`getSegments`).
6. Traçado de linhas candidatas no ecrã. Guarda-se o mapa polar, **não um modelo arquitetónico**.

**Limitações inevitáveis da versão 2:**
- O LiDAR vê a **primeira superfície refletora** em cada direção. Não descobre automaticamente paredes escondidas atrás de móveis.
- Um sofá/armário estável pode ser considerado uma superfície que parece uma parede.
- O sistema não distingue quartos conectados através de portas nem combina diferentes posições de observação, pois não inclui SLAM.
- Vidros, espelhos, objetos pretos e luz solar intensa podem degradar medições.
- O HLK-LD2450 é um radar frontal (campo ~120°), não deteta pessoas em todos os 360°, e pode deixar de reportar pessoas imóveis.
- A associação entre pontos de um LiDAR 2D e pessoas é **aproximada**; não é reconhecimento, identificação nem contagem fiável de ocupantes.

Para planta arquitetónica completa, o próximo nível exige um sistema móvel com odometria e SLAM (p. ex. Raspberry Pi + ROS 2), ou várias estações fixas registadas geometricamente.

## 7. Testes de processamento sem hardware

```sh
g++ -std=c++11 -O2 -Wall -Wextra -Werror -pedantic tests/protocol_tests.cpp -o /tmp/protocol_tests
/tmp/protocol_tests
g++ -std=c++11 -O2 -Wall -Wextra -Werror -pedantic tests/map_tests.cpp -o /tmp/map_tests
/tmp/map_tests
```

Os testes verificam CRC do LD19, descodificação LD2450 (incluindo o sinal não convencional das coordenadas), extração de segmentos numa sala retangular simulada, resistência a uma alteração transitória e checksum da persistência. **Não substituem testes reais de hardware e calibração.**

## 8. Solução de problemas

| Problema | Verificar |
|---|---|
| LD19 `--` | TX→GPIO15, 5V, GND, cabos, baud 230400, PWM conforme a variante |
| RADR `--` | TX→GPIO4, 5V, GND, baud 256000, modo de reporte LD2450 |
| TFT negro / branco | Interface realmente SPI ILI9341, CS/DC/RST, alimentação/LED e 3.3V |
| Poucos pontos estáveis | Obstáculos móveis, superfícies refletoras, nível de instalação, intensidade, distância |
| Paredes e alvos desalinhados | Rotação e offsets; reiniciar SCAN após mudar rotação LiDAR |
| Linhas retas em armários | Limitação do algoritmo de superfícies — não é segmentação semântica |
| Botão não reage | Confirmar contacto entre GPIO47 e GND, sem aplicar 5V |
| Mapa some após reinício | Fazer clique curto para guardar; verificar mensagens de NVS no Serial |
| Interface lenta ou com flicker | Framebuffer pode não caber na memória; reduzir frequência e resolução ou mudar para LVGL/TFT_eSPI |

## 9. Referências técnicas

- LD19 Development Manual — https://core-electronics.com.au/attachments/localcontent/LD19_Development_Manual_v2_7895434727a.pdf
- LD19 (Waveshare Wiki) — https://www.waveshare.com/wiki/DTOF_LIDAR_LD19
- LD19 datasheet — https://raw.githubusercontent.com/May-DFRobot/DFRobot/master/LD19_Datasheet_v2.5.pdf
- LD2450 Serial Communication Protocol — https://assets.super.so/79c0d2a8-d37a-438f-8fbe-c44778f3b0dd/files/ee2aafb0-f93e-47cd-b722-ad0f9e76b6f7/LD2450_Serial_Communication_Protocol_V1.02_EN.pdf
- ESP32-S3 UART (Espressif) — https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/uart.html
- ESP32-S3 DevKitC-1 (Espressif) — https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html
- Adafruit ILI9341 — https://github.com/adafruit/Adafruit_ILI9341

## 10. Licença e estado do projeto

Distribuído sob licença [MIT](LICENSE). O firmware é um **protótipo experimental**, não é um dispositivo de segurança nem um detetor garantido de ocupação. Os testes nativos de protocolo e processamento foram executados com sucesso; compilação PlatformIO e validação com sensores físicos permanecem pendentes.

### Publicação inicial no GitHub

O repositório deve ser criado como **público**, com o nome `wallsight-360`, sem inicializar README, licença ou `.gitignore` no GitHub (já estão incluídos). Para publicar esta pasta diretamente a partir do terminal local, depois de configurar o GitHub CLI (`gh auth login`):

```bash
cd wallsight-360
git init -b main
git add .
git commit -m "Initial release: WallSight 360 PlatformIO prototype"
gh repo create wallsight-360 --public --source . --remote origin --push
```
