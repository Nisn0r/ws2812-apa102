# STM32G030 — WS2812 → APA102

## Objectif
Décoder un signal d'entrée WS2812 et le retransmettre en APA102.

## 📖 Référence matérielle : [STM32G030-reference.md](STM32G030-reference.md)
**À consulter avant de rouvrir les PDF constructeur.** Regroupe tout ce qui a déjà
été extrait du RM0454 et de la datasheet DS12991 : périphériques réellement
présents sur le G030 (la doc couvre toute la famille), piège du prefetch flash,
PWM Input Mode et règles des flags de capture, carte des registres TIM3, DMA
burst (`DCR`/`DMAR`), table DMAMUX, SPI, brochage TSSOP20 et fonctions
alternatives, pièges de débogage, et les **timings WS2812 réellement mesurés**.

## Contexte
Projet initialement tenté sur ATtiny202 (abandonné : trop contraint en broches et
en cycles CPU pour décoder WS2812 et sortir en SPI simultanément). Repris sur
STM32G030 spécifiquement pour son DMA configurable et une horloge plus rapide —
les deux failles qui avaient fait échouer les architectures ATtiny202 (CPU devant
réagir par bit, faute de marge de cycles).

## SystemClock_Config (main.c)
Ne pas modifier sans justification solide : HSI 16MHz → PLLM/1 → PLLN×8 (VCO
128MHz) → PLLR/2 → SYSCLK 64MHz, AHB/APB1 en DIV1. C'est cette configuration qui
fait tourner l'ALU et tous les bus internes (AHB, APB, CPU) à 64MHz.

Contient aussi `__HAL_FLASH_PREFETCH_BUFFER_ENABLE()` : `FLASH_ACR.PRFTEN` est
**désactivé par défaut au reset** (RM0454 §3.7.1, reset `0x00040600`), et ni
`HAL_RCC_ClockConfig` ni `FLASH_SetLatency` ne l'activent. À 64MHz avec
`FLASH_LATENCY_2`, chaque franchissement de ligne d'instruction coûte sinon les
2 wait states en entier.

---

## Architecture actuelle

Toute la configuration (GPIO, TIM3, DMA, SPI) est faite directement dans `main()`
— pas de fonctions `MX_*_Init` séparées ; les handles HAL sont des variables
locales à `main()`, valides tant que `main()` ne retourne pas (jamais).

```
WS2812 (PA6) ─► TIM3 PWM Input Mode ─► DMA1_CH1 (burst) ─► capture[]  (ping-pong)
                                                              │
                                                      décodage logiciel
                                                     (boucle while(1), sans IRQ)
                                                              │
                                                   pixel complet, au fil de l'eau
                                                              │
                                                    SPI1 ─► APA102 (PA1/PA2)
```

Un seul canal DMA est utilisé (capture). La sortie se fait par écriture directe
dans `SPI1->DR`, sans DMA ni buffer de trame.

### 1. Capture — TIM3 en PWM Input Mode, entrée PA6
- **PA6** = `TIM3_CH1` en AF1. PA0 avait été envisagé puis écarté : aucune
  fonction TIM3 sur ce boîtier. Une seule broche physique suffit, CH2 lit le
  même signal en interne (`TIM_ICSELECTION_INDIRECTTI`).
- **CH1** (front montant, direct) → CCR1 = période, et réarme le compteur à 0
  (slave mode `Reset` sur `TI1FP1`) — **réarmement matériel**, sans CPU.
- **CH2** (front descendant, indirect) → CCR2 = durée du niveau haut.
- **Pas d'IRQ**, pas de NVIC : `HAL_TIM_IC_Start` (jamais `_IT`).

### 2. Transfert — DMA1 canal 1 en **mode burst**
`TIM3->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_2TRANSFERS` (= `0x10D`).
Seul `CC2DE` est activé : chaque front descendant déclenche une rafale de
2 transferts lus via le registre virtuel `TIM3->DMAR`, redirigés vers CCR1 puis
CCR2.

**Pourquoi le burst et pas deux canaux** : CC1 et CC2 sont des événements à des
instants différents. Avec deux canaux séparés, l'appariement période/largeur
dépendrait de l'état du signal au démarrage du timer (front montant ou descendant
en premier) et resterait **décalé en permanence** en mode circulaire. Le burst
appariage les deux valeurs par construction, quel que soit l'instant de démarrage
— ce qui compte ici car on ne maîtrise pas quand le signal est présenté au MCU.

Buffer entrelacé : `capture[2*i]` = période du bit i, `capture[2*i+1]` = largeur.

### 3. Décodage — ping-pong, dans la boucle `while(1)`
Buffer circulaire de 2 chunks de 24 bits. Les flags **Half Transfer** et
**Transfer Complete** (pollés, pas d'IRQ) indiquent quelle moitié est prête ; on
la décode pendant que le DMA remplit l'autre. Budget : ~30 µs (1920 cycles) pour
24 bits, au lieu des ~75 cycles/bit du polling.

Un chunk **n'est pas aligné sur un pixel**, et n'a pas à l'être : c'est la
resynchronisation sur période allongée qui place les frontières.

Règles de décodage (`WS2812_DecodeChunk`) :
- `période > WS2812_PERIOD_RESET` → fin de trame : `APA102_EndFrame()` clôture la
  trame précédente, `APA102_StartFrame()` ouvre la suivante. Le bit courant
  appartient déjà à la nouvelle trame.
- `période > WS2812_PERIOD_NEW_PIXEL` → séparateur inter-pixel ou inter-triplet :
  ce bit ouvre un nouveau pixel. **C'est ce mécanisme qui resynchronise la trame**
  et corrige toute dérive, plutôt qu'un comptage modulo 24 qui dériverait
  silencieusement.
- `période < WS2812_PERIOD_MIN` → glitch, capture ignorée.
- 8 bits → `pixel[byteIndex]` (0=G, 1=R, 2=B) ; 3 octets → `APA102_SendPixel()`,
  émis immédiatement.

### 4. Sortie — SPI1, émission **en continu**, sans DMA ni buffer
- **SCK = PA1, MOSI = PA2** (AF0, broches 8 et 9 du TSSOP20, adjacentes).
- **Half-duplex émission seule** (`BIDIMODE`/`BIDIOE`) : l'APA102 n'a pas de
  ligne de retour, et surtout `SPI1_MISO` tombe sur **PA6**, déjà pris par
  l'entrée WS2812.
- fPCLK/8 = **8 MHz**. Un pixel (4 octets) part en 4 µs, contre ~28 µs que dure
  un pixel WS2812.

**Pourquoi le streaming est possible ici** : contrairement au WS2812 — où la
valeur du bit est codée par une *durée*, donc avec un timing strict — l'APA102
est cadencé par sa propre horloge SCK. Il n'y a **aucune contrainte de temps
entre deux octets** : on peut pousser un pixel, attendre 30 µs, pousser le
suivant. D'où :
- **aucune limite sur le nombre de LEDs**, et pas besoin de le connaître à
  l'avance (il n'est pas connu sur ce montage) ;
- **pas de buffer de trame** en RAM ;
- **pas de canal DMA** en sortie — le FIFO 32 bits du SPI vaut exactement un
  pixel APA102, l'attente sur `TXE` est marginale.

Séquence : start frame (32 bits à 0) au reset → chaque pixel dès qu'il est
décodé → end frame au reset suivant. `frameActive` empêche d'émettre des pixels
orphelins au démarrage, tant qu'aucun reset n'a été vu.

L'**end frame est calculée** : `ledCount/16` octets (la spec demande au moins
n/2 coups d'horloge pour propager les données au bout de la chaîne), minimum 4.
Elle s'adapte donc seule à la longueur réelle.

⚠️ **`SPI1->DR` doit être écrit en accès 8 bits** (`*(volatile uint8_t *)&SPI1->DR`).
Un accès 16/32 bits déclenche le *data packing* du FIFO et émet **deux** trames.
Voir référence §8.

- Témoin : un pulse sur **PA4** (LED) par trame.

## Seuils WS2812 — issus de relevés réels, pas des valeurs nominales
Mesurés à l'oscillo et dans le buffer, sur le contrôleur utilisé (tick = 15,6 ns) :

| Grandeur | Ticks | Constante |
|---|---|---|
| Largeur bit 0 | 22-23 | — |
| Largeur bit 1 | **59-60** | seuil `WS2812_BIT_THRESHOLD = 40` |
| Période normale | 74-75 | — |
| Séparateur inter-pixel | ~139 | `WS2812_PERIOD_NEW_PIXEL = 100` |
| Séparateur inter-triplet | > 600 | idem |
| Reset de fin de trame | > 3200 | `WS2812_PERIOD_RESET = 3200` |

⚠️ Le bit 1 est à **59-60 ticks**, pas ~45 comme le laisserait croire la valeur
nominale de 0,7 µs — d'où un seuil à 40 et non 32. Le contrôleur groupe les
pixels par **3** et envoie ~60 pixels avant reset.

---

## Points ouverts / à vérifier
- **Non validé sur matériel** : la chaîne DMA burst + décodage + sortie APA102
  compile mais n'a pas encore tourné. Seule la capture DMA burst a été vérifiée
  au débogueur (buffer régulier, pulse PA4 stable).
- **Ordre des octets APA102** (`0xE0|luminosité, B, G, R`) écrit de mémoire, pas
  depuis un datasheet relu. Si les couleurs sortent permutées, c'est dans
  `APA102_SendPixel`.
- **Reset > 1,02 ms** : le compteur 16 bits sans prescaler reboucle au-delà, un
  gap trop long serait lu comme une valeur faussement petite. Non observé ;
  parade = `Prescaler = 1` (cf. référence §10).
- **Chaînes très longues** : le reset est détecté sur le *premier bit de la trame
  suivante*, donc end frame + start frame sont émises pendant que la nouvelle
  trame défile déjà. À ~60 LEDs cela fait 8 octets = 8 µs, très en deçà des 28 µs
  d'un pixel. Mais vers 500 LEDs l'end frame atteint ~32 octets (~32 µs) et
  dépasserait le budget d'un chunk, avec risque de retard sur le buffer de
  capture. Dans ce cas, repasser la sortie en DMA.

---

## Leçons apprises (approche polling, abandonnée)
Une implémentation entièrement en polling CPU a été tentée puis **abandonnée
après mesure**. Elle perdait des bits par overcapture (`CC2OF`), donnant des
pixels corrompus (`1,240,1` au lieu de `0,255,0`). À conserver :

1. **Le polling par bit n'est pas viable à 800 kbit/s sur ce MCU.** Mesure au
   pin dédié + oscillo : **1,36 µs par bit** (~87 cycles) pour une période de
   1,17 µs (~75 cycles), et **1,84 µs** sur le bit terminal d'un octet. C'est un
   dépassement structurel, pas une marge à grappiller.
2. **Mesurer, ne pas estimer.** L'estimation sur désassemblage donnait ~55-60
   cycles ; la réalité était ~87. Les stalls flash sur branchements coûtent bien
   plus que le comptage d'instructions ne le suggère.
3. **`-Ofast` / `-O3` dégradent** ce type de chemin chaud (code plus gros et plus
   branchu → plus de lignes flash traversées → plus de stalls). Ne pas y revenir
   comme piste d'optimisation. `CFLAGS` reste `-Og -g3`.
4. **Les appels HAL ne sont pas inlinés entre unités de compilation** sous `-Og`
   (`HAL_TIM_ReadCapturedValue`, `HAL_GPIO_TogglePin`…). Dans un chemin chaud,
   utiliser l'accès registre direct.
5. **Effacer un flag rc_w0** : `TIM3->SR = ~MASK`, **jamais** `&= ~MASK` — le
   second est un load-modify-store qui peut effacer un flag levé entre le load et
   le store. C'est l'idiome de `__HAL_TIM_CLEAR_FLAG`.
6. **RM0454 §16.3.5** : lire les `CCRx` *avant* de tester/effacer `CCxOF`, pour ne
   pas rater une overcapture survenue entre les deux.
7. Hypothèse réfutée en cours de route : les flags `CCxIF` ne sont **pas** retardés
   jusqu'au front de l'autre canal ; chaque canal lève le sien immédiatement et
   indépendamment.

---

## Build, flash & debug
- `Makefile` : liste explicite et minimale de fichiers HAL (pas de wildcard).
  `stm32g0xx_hal_tim.c`, `_tim_ex.c` et `_spi.c` ont dû être ajoutés au fil des
  périphériques utilisés — **chaque `stm32g0xx_hal_xxx.c` doit être ajouté
  manuellement à `HAL_SOURCES`**.
- `CFLAGS` : `-Og -g3` (voir leçon 3 ci-dessus).
- **Sonde J-Link (Segger) via OpenOCD**, pas de ST-Link, pas de CubeProgrammer.
  `make flash` flashe le `.elf` (symboles inclus) avec verify + reset.

  Le dossier de configs OpenOCD vit **hors du dépôt** et doit contenir :
  - `interface/jlink.cfg` — `adapter driver jlink`, plus `adapter serial <n°>`
    si plusieurs sondes sont connectées
  - `target/stm32g0x.cfg` — `transport select swd` forcé (le G0 ne fait que du
    SWD), plus un hook `reset-init` qui remonte la PLL à 64 MHz côté sonde

  Son emplacement n'est pas codé en dur : `OPENOCD_SCRIPTS ?= ../OpenOCD` dans le
  Makefile, surchargeable sans le modifier :

  ```sh
  make flash OPENOCD_SCRIPTS=/chemin/vers/mes/configs
  ```
- **Debug VS Code** : extension *Cortex-Debug*, `servertype: openocd`,
  `gdbPath` vers `gdb-multiarch`, `runToEntryPoint: main`, et les deux mêmes
  fichiers de config. Cortex-Debug fonctionne aussi avec J-Link GDB Server. Le
  `.vscode/` n'est pas suivi (chemins locaux) — cette description tient lieu de
  documentation du montage s'il faut le recréer.

## Suivi de version
**Dépôt git public.** Ces deux notes (`CLAUDE.md`, `STM32G030-reference.md`) sont
suivies et donc publiées : elles doivent rester exemptes de tout chemin local,
nom d'utilisateur, numéro de série de sonde ou identifiant personnel. Les
chemins absolus ont été retirés du Makefile au profit de `$(SRC_DIR)` et de
`OPENOCD_SCRIPTS ?=` surchargeable.

`.gitignore` couvre :
- `/build/` — artefacts régénérés par `make`.
- `.claude/settings.local.json` — réglages propres à la machine.
- `.vscode/` — chemins locaux (`gdbPath`, emplacement des configs OpenOCD).
- `factory_firmware.bin` — binaire tiers, non régénérable, hors sujet du source.

**Sauvegardes manuelles** : la convention `*.old` / `*.bak` héritée du projet
ATtiny202 est rendue inutile par git — ne plus en créer, faire des commits.

Avant tout nouveau fichier commité, vérifier qu'il ne contient ni chemin absolu,
ni nom d'utilisateur, ni numéro de série de sonde. Contrôle rapide :

```sh
git ls-files -co --exclude-standard | xargs grep -nI "/home/\|~/\|adapter serial"
```

## Fichiers annexes
`stm32g0xx_hal_msp.c` contient le MSP TIM3 (GPIO PA6 + horloge, pas de NVIC) et
le MSP SPI1 (PA1/PA2 en AF0), plus le MSP global par défaut. `stm32g0xx_it.c`
n'a **aucun handler custom** — pas d'IRQ du tout dans ce projet, tout est en
polling de flags ou en DMA. `system_stm32g0xx.c`, `syscalls.c`, `sysmem.c`
restent du boilerplate STM32CubeMX/CMSIS inchangé.
