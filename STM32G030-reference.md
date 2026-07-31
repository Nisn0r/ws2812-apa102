# STM32G030 — notes matérielles de référence

Faits extraits de la documentation constructeur au fil du projet, pour éviter de
relire les PDF à chaque session. Sources :

- **RM0454** — *STM32G0x0 advanced Arm-based 32-bit MCUs* — Reference manual (ST)
- **DS12991** — *STM32G030x6/x8* — Datasheet (ST)
- En-têtes CMSIS/HAL du package STM32CubeG0 (chemin défini par `SDK_SRC` dans le
  `Makefile`)
- Schéma de la carte : *WeAct STM32G0xx/C0xx Core Board V1.0*

Tout ce qui suit a été vérifié dans ces sources (numéro de §/table donné à chaque
fois). Ce qui n'a pas été vérifié est explicitement marqué comme tel.

---

## 1. La cible : STM32G030F6

- Cortex-M0+, **32 Ko flash / 8 Ko RAM** (d'après `STM32G030xx_FLASH.ld`).
- Boîtier TSSOP20 (suffixe `F`).
- **Pas de cache de données** sur Cortex-M0+ (RM0454 §3.3.5).

### Périphériques réellement présents
Relevé dans `stm32g030xx.h` (CMSIS) :

> `DMA1`, `I2C1`, `I2C2`, `SPI1`, `SPI2`, `TIM1`, `TIM3`, `TIM14`, `TIM16`,
> `TIM17`, `USART1`, `USART2`

**Absents** (mais présents dans la doc, qui couvre toute la famille G0x0) :
`TIM4`, `TIM6`, `TIM7`, `TIM15`, `SPI3`, `I2C3`, `USART3` à `USART6`, `DMA2`.

⚠️ Piège récurrent : le chapitre 16 du RM0454 s'intitule « General-purpose timers
**(TIM3/TIM4)** » et la Table 39 (DMAMUX) liste des périphériques inexistants ici.
Toujours recouper avec la liste ci-dessus.

---

## 2. Accélération flash — le piège du prefetch

`FLASH_ACR`, offset 0x000, **reset value `0x0004 0600`** (RM0454 §3.7.1) :

| Bit | Nom | État au reset | Remarque |
|---|---|---|---|
| 8 | `PRFTEN` | **0 — désactivé** | à activer à la main |
| 9 | `ICEN` | **1 — activé** | cache d'instructions déjà actif |
| 11 | `ICRST` | 0 | reset du cache (écriture seulement si `ICEN=0`) |
| 2:0 | `LATENCY` | 0 | wait states |

**Le point important** : `PRFTEN` est à 0 au reset, et **ni `HAL_RCC_ClockConfig`
ni `FLASH_SetLatency` ne l'activent**. À 64 MHz avec `FLASH_LATENCY_2`, chaque
franchissement de ligne d'instruction coûte alors les 2 wait states en entier.

```c
__HAL_FLASH_PREFETCH_BUFFER_ENABLE();   /* SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN) */
```

Détails RM0454 §3.3.5 :
- Une lecture flash fournit **64 bits** (2 instructions 32 bits ou 4 en 16 bits).
- Le prefetch n'a d'intérêt qu'à partir d'1 wait state.
- Sur **branchement** (code non séquentiel), si la cible n'est ni dans la ligne
  courante ni dans la ligne préchargée : pénalité ≥ nombre de wait states.
- Le cache d'instructions retient **2 lignes de 64 bits** (16 octets), politique
  LRU. Utile surtout pour les boucles.

> Conséquence observée sur ce projet : `-Ofast` **dégrade** les performances du
> chemin chaud (code plus gros et plus branchu → plus de lignes traversées →
> plus de stalls). Voir CLAUDE.md.

---

## 3. Timers — PWM Input Mode

### Disponibilité
Le mode « PWM input » existe sur :
- **TIM1** (RM0454 §15.3.8) — présent sur G030
- **TIM3**/TIM4 (§16.3.6) — seul TIM3 existe ici
- TIM15 (§19.4.7) — absent du G030

Il n'existe **pas** de mode « Pulse Width Measurement » dédié (équivalent du
TCB0 FRQPW de l'ATtiny202) ; le PWM Input Mode en est l'équivalent fonctionnel,
avec en plus un **réarmement matériel** du compteur.

### Contrainte structurelle
> « The PWM input mode can be used only with the TIMx_CH1/TIMx_CH2 signals due to
> the fact that only TI1FP1 and TI2FP2 are connected to the slave mode
> controller. » (§16.3.6, note 1)

→ **CH1/CH2 uniquement**, jamais CH3/CH4.

### Principe (§16.3.6)
Deux signaux ICx mappés sur la **même** entrée TIx, actifs sur des fronts
opposés ; l'un des TIxFP sert de trigger au slave mode controller configuré en
mode **Reset**.

Pour mesurer période + rapport cyclique sur TI1 :

| Étape | Registre | Valeur |
|---|---|---|
| Entrée de CCR1 | `CCMR1.CC1S` | `01` (TI1) |
| Polarité TI1FP1 | `CCER.CC1P/CC1NP` | `0`/`0` (front montant) |
| Entrée de CCR2 | `CCMR1.CC2S` | `10` (TI1 aussi) |
| Polarité TI1FP2 | `CCER.CC2P/CC2NP` | `1`/`0` (front descendant) |
| Trigger | `SMCR.TS` | `00101` (TI1FP1) |
| Slave mode | `SMCR.SMS` | `100` (Reset) |
| Captures | `CCER.CC1E/CC2E` | `1`/`1` |

Résultat : **CCR1 = période**, **CCR2 = durée du niveau haut**. Une seule broche
physique est nécessaire (CH2 lit le même signal en interne via
`TIM_ICSELECTION_INDIRECTTI`).

### Flags de capture (§16.3.5)
Le PWM input mode « reprend la même procédure » que l'input capture ; les règles
de flags s'appliquent donc telles quelles :

- `CCxIF` est levé **immédiatement et indépendamment** à la transition du canal
  concerné. Il n'y a **aucun** délai jusqu'au front de l'autre canal.
- `CCxIF` s'efface **en lisant `CCRx`** ou en écrivant 0.
- `CCxOF` (overcapture) est levé si une capture survient alors que `CCxIF` était
  déjà haut. Il s'efface **uniquement par écriture de 0** (pas par lecture).
- Recommandation explicite du RM :
  > « In order to handle the overcapture, it is recommended to read the data
  > **before** the overcapture flag. »

**Idiome d'effacement** (registre rc_w0) :
```c
TIM3->SR = (uint32_t)~TIM_SR_CC2OF;   /* OK — idiome de __HAL_TIM_CLEAR_FLAG */
TIM3->SR &= (uint32_t)~TIM_SR_CC2OF;  /* NON — load-modify-store, peut effacer
                                         un flag levé entre le load et le store */
```
Écrire des 1 sur les autres bits d'un registre rc_w0 est un no-op garanti par le
matériel, donc la première forme est sans risque.

---

## 4. Carte des registres TIM3 (offsets)

| Offset | Registre | | Offset | Registre |
|---|---|---|---|---|
| 0x00 | `CR1` | | 0x24 | `CNT` |
| 0x04 | `CR2` | | 0x28 | `PSC` |
| 0x08 | `SMCR` | | 0x2C | `ARR` |
| 0x0C | `DIER` | | 0x30 | `RCR` |
| 0x10 | `SR` | | **0x34** | **`CCR1`** |
| 0x14 | `EGR` | | **0x38** | **`CCR2`** |
| 0x18 | `CCMR1` | | 0x3C | `CCR3` |
| 0x1C | `CCMR2` | | 0x40 | `CCR4` |
| 0x20 | `CCER` | | **0x48** | **`DCR`** |
| | | | **0x4C** | **`DMAR`** |

`DIER` (§16.4.4) contient `CC1DE`/`CC2DE` : requêtes **DMA** par canal, sur une
ligne matérielle **distincte** des interruptions `CC1IE`/`CC2IE`. Utiliser le DMA
ne sollicite donc pas le NVIC.

---

## 5. DMA burst des timers (§16.3.20)

> « The TIMx timers have the capability to generate **multiple DMA requests upon
> a single event**. […] it can also be used to **read several registers in a
> row**, at regular intervals. »

Un seul événement déclencheur produit une rafale de N transferts, tous dirigés
vers/depuis le **registre virtuel `TIMx_DMAR`**, qui redirige chaque accès vers
un registre réel.

### `TIMx_DCR` (offset 0x48, §16.4.20)
- `DBL[12:8]` — nombre de transferts, **encodé n−1** :
  `00000` = 1 transfert, `00001` = 2 transferts, … `10001` = 18.
- `DBA[4:0]` — adresse de base, **offset depuis `TIMx_CR1` en mots de 32 bits**
  (`00000` = CR1, `00001` = CR2, …).

### `TIMx_DMAR` (offset 0x4C, §16.4.21)
```
adresse réelle = (adresse TIMx_CR1) + (DBA + index) × 4
```
`index` va de 0 à DBL, géré automatiquement par le transfert DMA.
Champ `DMAB[15:0]` → **accès 16 bits** (half-word).

### Application à ce projet (CCR1 + CCR2 en une rafale)
```c
TIM3->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_2TRANSFERS;  /* = 0x10D */
```
- `TIM_DMABASE_CCR1` = `0x0D` = 13 → 13 × 4 = 0x34 = offset de CCR1 ✓
- `TIM_DMABURSTLENGTH_2TRANSFERS` = `0x100` → DBL = 1 → 2 transferts ✓

Déclenché par **CC2** (front descendant) : à cet instant CCR1 contient la période
et CCR2 la durée haute qui vient d'être capturée — les deux appartiennent au même
bit, **appariés par construction**. C'est ce qui évite le décalage permanent que
provoquerait l'usage de deux canaux DMA distincts (CC1 et CC2 étant des
événements à des instants différents, l'appariement dépendrait de l'état du
signal au démarrage du timer).

---

## 6. DMA et DMAMUX

- **DMA1 à 5 canaux** sur STM32G030xx, **pas de DMA2** (RM0454 Table 33).
  (7 canaux sur G050/G070, 7+5 sur G0B0.)
- DMAMUX : 4 générateurs de requêtes, 23 entrées de trigger, 23 entrées de
  synchro, jusqu'à 55 entrées de requêtes périphériques.

### Entrées DMAMUX utiles (Table 39 — **liste famille**, filtrée aux périphériques du G030)

| # | Ressource | | # | Ressource |
|---|---|---|---|---|
| 1-4 | `dmamux_req_gen0..3` | | 32 | **`TIM3_CH1`** |
| 5 | `ADC` | | 33 | **`TIM3_CH2`** |
| 10-11 | `I2C1_RX` / `I2C1_TX` | | 34-35 | `TIM3_CH3` / `TIM3_CH4` |
| 12-13 | `I2C2_RX` / `I2C2_TX` | | 36-37 | `TIM3_TRIG` / `TIM3_UP` |
| 16-17 | `SPI1_RX` / **`SPI1_TX`** | | 44-46 | `TIM16_CH1` / `COM` / `UP` |
| 18-19 | `SPI2_RX` / **`SPI2_TX`** | | 47-49 | `TIM17_CH1` / `COM` / `UP` |
| 20-25 | `TIM1_CH1..CH4`, `TRIG_COM`, `UP` | | 50-51 | `USART1_RX` / `USART1_TX` |
| | | | 52-53 | `USART2_RX` / `USART2_TX` |

En pratique on passe par les macros HAL `DMA_REQUEST_TIM3_CH2`,
`DMA_REQUEST_SPI1_TX`, etc., qui encodent ces numéros.

---

## 7. Brochage — fonctions alternatives (DS12991 Table 13)

### Boîtier TSSOP20 (STM32G030F6, Figure 4)
Les broches **réellement sorties** sur ce boîtier :

| Pin | Signal | | Pin | Signal |
|---|---|---|---|---|
| 1 | PB7/PB8 | | 11 | **PA4** *(LED témoin)* |
| 2 | PB9/PC14-OSC32_IN | | 12 | PA5 |
| 3 | PC15-OSC32_OUT | | 13 | **PA6** *(entrée WS2812)* |
| 4 | VDD/VDDA | | 14 | PA7 |
| 5 | VSS/VSSA | | 15 | PB0/PB1/PB2/PA8 |
| 6 | NRST | | 16 | PA11[PA9] |
| 7 | PA0 | | 17 | PA12[PA10] |
| 8 | **PA1** *(SPI1_SCK)* | | 18 | PA13 |
| 9 | **PA2** *(SPI1_MOSI)* | | 19 | PA15/PA14-BOOT0 |
| 10 | PA3 | | 20 | PB3/PB4/PB5/PB6 |

Plusieurs GPIO sont **fusionnés sur une même broche** (pins 1, 2, 15, 19, 20) —
attention avant de choisir une broche « libre » dans ces groupes.

### TIM3 — broches possibles
| Signal | Broches | AF |
|---|---|---|
| `TIM3_CH1` | **PA6**, PB4, PC6 | AF1 |
| `TIM3_CH2` | PA7, PB5, PC7 | AF1 |

⚠️ **PA0 n'a aucune fonction TIM3.** Ses seules AF sont `SPI2_SCK` (AF0) et
`USART2_CTS` (AF1). C'est ce qui a fait écarter PA0 comme entrée WS2812.

### Port A — extrait pertinent
| Broche | AF0 | AF1 | AF2 | AF4 | AF5 | AF6 |
|---|---|---|---|---|---|---|
| PA0 | `SPI2_SCK` | `USART2_CTS` | — | — | — | — |
| PA1 | `SPI1_SCK`/`I2S1_CK` | `USART2_RTS_DE_CK` | — | — | — | `I2C1_SMBA` |
| PA2 | `SPI1_MOSI`/`I2S1_SD` | `USART2_TX` | — | — | — | — |
| PA3 | `SPI2_MISO` | `USART2_RX` | — | — | — | — |
| PA4 | `SPI1_NSS`/`I2S1_WS` | `SPI2_MOSI` | — | `TIM14_CH1` | — | — |
| PA5 | `SPI1_SCK`/`I2S1_CK` | — | — | — | — | — |
| **PA6** | `SPI1_MISO`/`I2S1_MCK` | **`TIM3_CH1`** | `TIM1_BKIN` | — | `TIM16_CH1` | — |
| PA7 | `SPI1_MOSI`/`I2S1_SD` | `TIM3_CH2` | `TIM1_CH1N` | `TIM14_CH1` | `TIM17_CH1` | — |
| PA8 | `MCO` | `SPI2_NSS` | `TIM1_CH1` | — | — | — |
| PA9 | `MCO` | `USART1_TX` | `TIM1_CH2` | `SPI2_MISO` | — | `I2C1_SCL` |
| PA10 | `SPI2_MOSI` | `USART1_RX` | `TIM1_CH3` | — | `TIM17_BKIN` | `I2C1_SDA` |
| PA11 | `SPI1_MISO`/`I2S1_MCK` | `USART1_CTS` | `TIM1_CH4` | — | `TIM1_BKIN2` | `I2C2_SCL` |

Autres broches SPI2 relevées : `PB10` = `SPI2_SCK`, `PB11` = `SPI2_MOSI`.

### SPI1 — récapitulatif des broches
| Signal | Broches possibles | AF |
|---|---|---|
| `SPI1_SCK` | **PA1** (pin 8), PA5 (pin 12) | AF0 |
| `SPI1_MOSI` | **PA2** (pin 9), PA7 (pin 14) | AF0 |
| `SPI1_MISO` | PA6 (pin 13) ⚠️, PA11 | AF0 |
| `SPI1_NSS` | PA4 (pin 11) | AF0 |

⚠️ `SPI1_MISO` tombe sur **PA6**, déjà occupé par `TIM3_CH1` (entrée WS2812) —
raison de plus d'utiliser le SPI en half-duplex émission seule pour l'APA102.

**Choix retenu dans ce projet : SCK = PA1, MOSI = PA2** (broches 8 et 9,
adjacentes, toutes deux en AF0, aucun conflit avec PA4 ni PA6).

---

## 8. SPI (RM0454 §27)

### Instances disponibles (Table 129)
`SPI1`/`I2S1` et `SPI2` sont présents sur le G030. `SPI3` et `I2S2` sont
**réservés au STM32G0B0xx**.

| Caractéristique | Valeur |
|---|---|
| Taille de mot | **4 à 16 bits** (`DS[3:0]` dans `CR2`) |
| FIFO Rx/Tx | 32 bits |
| CRC matériel | oui |
| Modes NSSP & TI | oui |

Horloge : `SPI1` est sur `RCC_APBENR2`, `SPI2` sur `RCC_APBENR1`. Le G0 n'a qu'un
seul domaine APB → **PCLK = 64 MHz** avec la configuration du projet.

### Bits utiles
**`SPI_CR1`**
- `BR[5:3]` — diviseur d'horloge :

  | BR | Rapport | @64 MHz |
  |---|---|---|
  | `000` | fPCLK/2 | 32 MHz |
  | `001` | fPCLK/4 | 16 MHz |
  | `010` | **fPCLK/8** | **8 MHz** ← retenu pour l'APA102 |
  | `011` | fPCLK/16 | 4 MHz |
  | `100`…`111` | /32 … /256 | 2 MHz … 250 kHz |

- `MSTR` (bit 2) — 1 = maître
- `BIDIMODE` / `BIDIOE` — half-duplex ; `BIDIOE=1` force le sens **sortie**
  (seul MOSI est utilisé, ce qu'il faut pour l'APA102 qui n'a pas de retour)
- `SSM` / `SSI` — gestion logicielle du NSS
- `CPOL` / `CPHA` — mode 0 (`0`/`0`) pour l'APA102 (échantillonnage sur front
  montant)

**`SPI_CR2`**
- `DS[11:8]` — taille de mot : `0111` = 8 bits
- `FRXTH` (bit 12) — seuil RXNE : 1 = 1/4 de FIFO (8 bits)
- `TXDMAEN` (bit 1) — « a DMA request is generated whenever the TXE flag is set »
- `RXDMAEN` (bit 0) — idem sur RXNE

### ⚠️ Data packing — piège sur l'écriture de `SPIx_DR`
RM0454 §27.5, section *Data packing* :

> « When the data frame size fits into one byte (less than or equal to 8 bits),
> data packing is used **automatically when any read or write 16-bit access is
> performed** on the SPIx_DR register. […] **Two data frames are sent** after the
> single 16-bit access. »

Autrement dit, avec `DS = 8 bits`, la largeur de l'accès C détermine le nombre de
trames émises :

```c
*(volatile uint8_t *)&SPI1->DR = value;   /* OK  — 1 octet emis */
SPI1->DR = value;                         /* NON — accès 32 bits : data packing,
                                             2 trames émises */
```

Le champ `DR` étant déclaré `__IO uint32_t` dans le CMSIS, l'écriture naïve est
un accès 32 bits : **le cast 8 bits est obligatoire**. Symptôme sinon : chaque
octet est émis en double.

Le RM ajoute que pour une séquence d'un nombre **impair** de trames, il suffit
d'écrire la dernière en accès 8 bits.

### FIFO d'émission
32 bits, soit **4 octets** en mode 8 bits — exactement la taille d'une trame LED
APA102. `TXE` signale que le FIFO a de la place ; l'écriture d'un pixel entier
est donc quasi non bloquante, ce qui permet de se passer de DMA en sortie.

### Entrées DMAMUX
`SPI1_RX` = 16, **`SPI1_TX` = 17**, `SPI2_RX` = 18, `SPI2_TX` = 19.
(Non utilisées actuellement : la sortie APA102 écrit directement dans `DR`.)

### Débit dans ce projet
Émission **en continu**, pixel par pixel : 4 octets à 8 MHz = **4 µs**, à
comparer aux ~28 µs que dure un pixel WS2812 en entrée. L'APA102 n'ayant aucune
contrainte de temps entre octets (il est cadencé par SCK, contrairement au
WS2812 dont le bit est codé par une durée), rien n'oblige à bufferiser la trame
entière — et le nombre de LEDs n'a donc pas à être connu à l'avance.

---

## 9. Débogage — piège à connaître

RM0454 §16.3.21 : quand le cœur est arrêté (breakpoint/halt), le compteur du
timer **continue ou s'arrête selon le bit `DBG_TIMx_STOP` du DBGMCU** (§29.9.2).
À garder en tête en inspectant des buffers de capture au débogueur : selon la
configuration, le flux peut continuer d'être capturé pendant l'arrêt.

Le `target/stm32g0x.cfg` d'OpenOCD utilisé ici positionne à l'examine-end :
- `DBGMCU_CR |= DBG_STANDBY | DBG_STOP`
- `DBGMCU_APB1_FZ |= DBG_IWDG_STOP | DBG_WWDG_STOP` (gel des watchdogs au halt)

Elle contient aussi un hook `reset-init` qui reconfigure la PLL à 64 MHz côté
sonde — indépendant du `SystemClock_Config` applicatif.

---

## 10. Timings WS2812 — **relevés réels** sur le contrôleur utilisé

À 64 MHz sans prescaler, 1 tick = **15,6 ns**.

Valeurs **mesurées à l'oscillo et lues dans le buffer de capture** (pas les
valeurs nominales du datasheet WS2812, qui diffèrent sensiblement ici) :

| Grandeur | Ticks mesurés | Durée |
|---|---|---|
| Largeur bit 0 | **22-23** | ~0,35 µs |
| Largeur bit 1 | **59-60** | ~0,93 µs |
| Période d'un bit | **74-75** | ~1,17 µs |
| Séparateur inter-pixel | **~139** | ~2,17 µs |
| Séparateur inter-triplet | **> 600** | > 9 µs |
| Reset de fin de trame | > 3200 | > 50 µs |

> ⚠️ Le bit 1 est à **59-60 ticks**, pas ~45 comme le laisserait supposer la
> valeur nominale de 0,7 µs. Le seuil de discrimination est donc placé à **40**
> (milieu de 23 et 59), et non 32. Le contrôleur groupe les pixels par **3**,
> avec un séparateur inter-triplet plus long, et envoie ~60 pixels avant reset.

**Limite connue** : le compteur est en 16 bits sans prescaler, soit une plage de
65536 ticks = **1,02 ms**. Si le gap de reset dépassait cette durée, `CCR1`
reboucle et la valeur lue serait faussement petite. Non observé à ce jour ; la
parade serait `Prescaler = 1` (tick 31,2 ns, plage 2,05 ms — le bit 0 tomberait à
11 ticks et le bit 1 à 30, encore largement discriminables).

Budget CPU correspondant : **~75 cycles par bit** à 64 MHz — c'est cette
contrainte qui a condamné l'approche par polling (mesurée à 1,36 µs/bit, soit
~87 cycles). Voir CLAUDE.md pour l'historique.
