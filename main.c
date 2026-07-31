#include "stm32g0xx.h"

/* Prototypes */
static void SystemClock_Config(void);
static void WS2812_DecodeChunk(const uint16_t *chunk);
static void APA102_StartFrame(void);
static void APA102_SendPixel(uint8_t g, uint8_t r, uint8_t b);
static void APA102_EndFrame(uint32_t ledCount);

/*
 * Decodage WS2812 via TIM3 en PWM Input Mode sur PA6 (TIM3_CH1) :
 * CH1 (direct, front montant) capture la periode du bit courant dans CCR1 et
 * reinitialise le compteur (slave mode Reset) ; CH2 (indirect, front
 * descendant) capture la duree du niveau haut dans CCR2, qui distingue un
 * bit 0 d'un bit 1.
 *
 * Seuils issus de RELEVES REELS a l'oscillo sur le controleur utilise
 * (a 64MHz sans prescaler, 1 tick = 15.6ns) :
 *   largeur bit 0 ......... 22-23 ticks  (~0.35us)
 *   largeur bit 1 ......... 59-60 ticks  (~0.93us)
 *   periode normale ....... 74-75 ticks  (~1.17us)
 *   separateur inter-pixel ~139 ticks    (periode allongee, sert de synchro)
 *   separateur inter-triplet > 600 ticks (le controleur groupe par 3 pixels)
 *   reset de fin de trame . > 50us       (~60 pixels par trame)
 *
 * Le seuil de bit est place au milieu de 23 et 59 -> 40 (marge symetrique de
 * ~18 ticks de chaque cote). Une periode hors gabarit haut marque le debut
 * d'un nouveau pixel : c'est ce qui resynchronise la trame et corrige toute
 * derive, plutot qu'un simple comptage modulo 24.
 */
#define WS2812_BIT_THRESHOLD      40U   /* > seuil -> bit 1 */
#define WS2812_PERIOD_MIN         40U   /* en dessous : glitch, capture ignoree */
#define WS2812_PERIOD_NEW_PIXEL  100U   /* au dessus : debut d'un nouveau pixel */
#define WS2812_PERIOD_RESET     3200U   /* au dessus (>50us) : fin de trame */

/*
 * Capture par DMA en mode burst (remplace le polling CPU par bit, mesure a
 * 1.36us/bit pour un budget de 1.25us -> depassement structurel, cf CLAUDE.md).
 *
 * Le DMA ne decode rien : il recopie des half-words. La valeur du bit est
 * portee par la VALEUR du half-word (largeur d'impulsion en ticks) ; le
 * decodage reste logiciel mais se fait a posteriori, sans echeance par bit.
 *
 * Mode burst (RM0454 16.3.20) : un SEUL evenement declencheur (CC2, front
 * descendant) lance une rafale de 2 transferts lus via le registre virtuel
 * TIM3->DMAR, redirige vers CCR1 puis CCR2 (DBA = CCR1, DBL = 2 transferts).
 * A cet instant CCR1 contient la periode (capturee au dernier front montant)
 * et CCR2 la duree du niveau haut qui vient d'etre capturee : les deux
 * appartiennent au meme bit, **apparies par construction**. C'est ce qui evite
 * le decalage permanent entre deux buffers separes si le timer demarre pendant
 * une phase haute du signal.
 *
 * Buffer entrelace : capture[2*i] = periode du bit i, capture[2*i+1] = largeur.
 *
 * PING-PONG : le buffer fait 2 chunks de 24 bits. Le DMA tourne en circulaire ;
 * le flag Half Transfer signale que la 1re moitie est pleine, Transfer Complete
 * la 2e. Le CPU decode une moitie pendant que le DMA remplit l'autre, ce qui
 * marche quelle que soit la longueur de la trame (~60 pixels ici) sans avoir a
 * tout bufferiser. Budget : ~30us (1920 cycles) pour decoder 24 bits, contre
 * les 80 cycles/bit du polling.
 *
 * Note : un chunk n'est PAS aligne sur un pixel, et ce n'est pas necessaire —
 * c'est la resynchronisation sur la periode allongee qui place les frontieres.
 */
#define WS2812_CHUNK_BITS   24U
#define WS2812_CAPTURE_BITS (WS2812_CHUNK_BITS * 2U)
#define WS2812_CAPTURE_LEN  (WS2812_CAPTURE_BITS * 2U)

static uint16_t capture[WS2812_CAPTURE_LEN];

/*
 * Sortie APA102 : emission EN CONTINU, pixel par pixel, au fil du decodage.
 *
 * Contrairement au WS2812 (ou le bit est code par la duree du niveau haut, donc
 * avec un timing strict), l'APA102 est cadence par sa propre horloge SCK : il
 * n'y a AUCUNE contrainte de temps entre deux octets. On peut donc pousser
 * chaque pixel des qu'il est decode, au lieu de bufferiser la trame entiere.
 *
 * Consequence : plus aucune limite sur le nombre de LEDs, et plus de gros
 * buffer en RAM. Le nombre de pixels de la trame d'entree n'a pas besoin
 * d'etre connu a l'avance.
 *
 * Format APA102 : start frame (32 bits a 0), puis 4 octets par LED
 * (0xE0|luminosite5bits, B, G, R), puis end frame.
 * Le WS2812 fournit G,R,B -> reordonnancement en B,G,R ici.
 */
#define APA102_BRIGHTNESS 0x1FU   /* 5 bits, 0x1F = maximum */

/* frameActive : une start frame a ete emise, les pixels qui suivent lui
   appartiennent. Reste a 0 tant qu'aucun reset WS2812 n'a ete vu, pour ne pas
   emettre de pixels orphelins au demarrage (on ignore la trame en cours). */
static uint8_t  frameActive = 0;
static uint32_t ledCount    = 0;   /* LEDs emises dans la trame courante */

/* Etat du decodage bit -> octet -> pixel */
static uint8_t rxByte    = 0;
static uint8_t bitCount  = 0;
static uint8_t byteIndex = 0;   /* 0=G, 1=R, 2=B */
static uint8_t pixel[3]  = {0};

int main(void)
{
    HAL_Init();

    /**
     * System clock initialization
     */
    SystemClock_Config();

    /**
     * GPIO initialization
     */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    /* Configure GPIO pin : PA4 (LED temoin) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_IC_InitTypeDef sConfigIC = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_HandleTypeDef htim3 = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&htim3);

    /* CH1 : front montant, capture directe -> periode */
    sConfigIC.ICPolarity  = TIM_ICPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

    /* CH2 : front descendant, capture indirecte du meme signal -> duree du niveau haut */
    sConfigIC.ICPolarity  = TIM_ICPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

    /* Slave mode Reset : le compteur repart de 0 a chaque front montant sur TI1FP1 */
    sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig);

    /*
     * Mode burst : DBA = CCR1 (offset 0x34 / 4 = 13), DBL = 2 transferts.
     * Chaque acces au registre virtuel TIM3->DMAR est redirige vers
     * CCR1 puis CCR2 (RM0454 16.4.21 : adresse = CR1 + (DBA + index) * 4).
     */
    TIM3->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_2TRANSFERS;

    /*
     * Un seul canal DMA, source = TIM3->DMAR (adresse fixe), destination = le
     * buffer entrelace (incrementee), half-word de chaque cote, circulaire.
     * La requete DMA (CC2DE) est une ligne materielle distincte de
     * l'interruption (CC2IE) : le NVIC n'est pas sollicite.
     */
    __HAL_RCC_DMA1_CLK_ENABLE();

    DMA_HandleTypeDef hdmaCapture = {0};
    hdmaCapture.Instance                 = DMA1_Channel1;
    hdmaCapture.Init.Request             = DMA_REQUEST_TIM3_CH2;
    hdmaCapture.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdmaCapture.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaCapture.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaCapture.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdmaCapture.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdmaCapture.Init.Mode                = DMA_CIRCULAR;
    hdmaCapture.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdmaCapture);

    /* Armer le DMA AVANT de lancer le timer, pour ne manquer aucune capture */
    HAL_DMA_Start(&hdmaCapture, (uint32_t)&TIM3->DMAR, (uint32_t)capture, WS2812_CAPTURE_LEN);

    /*
     * Sortie APA102 : SPI1 en emission seule (half-duplex, BIDIMODE=1/BIDIOE=1
     * -> seul MOSI est utilise, l'APA102 n'a pas de ligne de retour).
     * SCK = PA1, MOSI = PA2 (AF0, broches 8 et 9 du TSSOP20, adjacentes) —
     * configure dans HAL_SPI_MspInit (stm32g0xx_hal_msp.c).
     * fPCLK/8 = 8MHz : 4 octets (un pixel) partent en 4us, a comparer aux ~28us
     * que dure un pixel WS2812. Pas de DMA en sortie : le FIFO 32 bits du SPI
     * absorbe un pixel entier, l'ecriture est quasi non bloquante.
     * Mode 0 (CPOL=0, CPHA=0) : l'APA102 echantillonne sur front montant.
     */
    SPI_HandleTypeDef hspi1 = {0};
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_1LINE;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);

    /* BIDIOE : en half-duplex, forcer le sens sortie.
       Pas de TXDMAEN : les octets sont ecrits directement dans DR (voir
       APA102_WriteByte), ce qui libere un canal DMA. */
    SPI1->CR1 |= SPI_CR1_BIDIOE;
    __HAL_SPI_ENABLE(&hspi1);

    /* Seul CC2 declenche la rafale (CC1 n'a pas besoin de sa propre requete) */
    __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC2);

    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);

    while (1)
    {
        /*
         * Ping-pong : Half Transfer -> la 1re moitie du buffer est pleine,
         * Transfer Complete -> la 2e. On decode la moitie qui vient d'etre
         * remplie pendant que le DMA ecrit dans l'autre.
         */
        if (DMA1->ISR & DMA_ISR_HTIF1)
        {
            DMA1->IFCR = DMA_IFCR_CHTIF1;
            WS2812_DecodeChunk(&capture[0]);
        }

        if (DMA1->ISR & DMA_ISR_TCIF1)
        {
            DMA1->IFCR = DMA_IFCR_CTCIF1;
            WS2812_DecodeChunk(&capture[WS2812_CHUNK_BITS * 2U]);
        }
    }
}

/**
  * @brief Decode un chunk de WS2812_CHUNK_BITS bits depuis le buffer de capture.
  *        chunk[2*i] = periode du bit i, chunk[2*i+1] = duree du niveau haut.
  */
static void WS2812_DecodeChunk(const uint16_t *chunk)
{
    for (uint32_t i = 0; i < WS2812_CHUNK_BITS; i++)
    {
        uint16_t period = chunk[2U * i];
        uint16_t width  = chunk[2U * i + 1U];

        if (period > WS2812_PERIOD_RESET)
        {
            /* Reset : la trame precedente s'acheve. On la cloture (end frame),
               puis on ouvre la suivante (start frame). Ce bit appartient deja
               a la nouvelle trame. */
            if (frameActive)
                APA102_EndFrame(ledCount);

            APA102_StartFrame();
            frameActive = 1;
            ledCount    = 0;
            bitCount    = 0;
            byteIndex   = 0;

            /* Temoin : un pulse sur PA4 par trame */
            GPIOA->ODR ^= GPIO_PIN_4;
            GPIOA->ODR ^= GPIO_PIN_4;
        }
        else if (period > WS2812_PERIOD_NEW_PIXEL)
        {
            /* Separateur inter-pixel ou inter-triplet : ce bit ouvre un pixel */
            bitCount = 0;
            byteIndex = 0;
        }
        else if (period < WS2812_PERIOD_MIN)
        {
            /* Trop court pour etre un bit valide : glitch, on resynchronise */
            bitCount = 0;
            byteIndex = 0;
            continue;
        }

        rxByte <<= 1;
        if (width > WS2812_BIT_THRESHOLD)
            rxByte |= 1U;

        if (++bitCount == 8U)
        {
            bitCount = 0;
            pixel[byteIndex] = rxByte;
            rxByte = 0;

            if (++byteIndex == 3U)
            {
                byteIndex = 0;
                /* Pixel complet : emis immediatement, sans attendre la fin de
                   la trame. Ignore tant qu'aucune start frame n'a ete emise. */
                if (frameActive)
                {
                    APA102_SendPixel(pixel[0], pixel[1], pixel[2]);
                    ledCount++;
                }
            }
        }
    }
}

/**
  * @brief Pousse un octet sur SPI1 vers l'APA102.
  *
  * L'ecriture DOIT etre un acces 8 bits : sur un acces 16/32 bits, le FIFO
  * applique automatiquement le "data packing" (RM0454 27.5, section Data
  * packing) et emettrait deux trames au lieu d'une.
  *
  * Le FIFO d'emission fait 32 bits, soit un pixel APA102 entier : l'attente sur
  * TXE est en pratique tres courte.
  */
static inline void APA102_WriteByte(uint8_t value)
{
    while ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
    }
    *(volatile uint8_t *)&SPI1->DR = value;
}

/**
  * @brief Start frame APA102 : 32 bits a 0.
  */
static void APA102_StartFrame(void)
{
    for (uint32_t i = 0; i < 4U; i++)
        APA102_WriteByte(0x00U);
}

/**
  * @brief Emet un pixel. Entree au format WS2812 (G, R, B), sortie APA102
  *        (0xE0|luminosite, B, G, R).
  */
static void APA102_SendPixel(uint8_t g, uint8_t r, uint8_t b)
{
    APA102_WriteByte(0xE0U | APA102_BRIGHTNESS);
    APA102_WriteByte(b);
    APA102_WriteByte(g);
    APA102_WriteByte(r);
}

/**
  * @brief End frame APA102.
  *
  * Elle doit fournir au moins ledCount/2 coups d'horloge supplementaires pour
  * propager les donnees jusqu'au bout de la chaine, soit ledCount/16 octets.
  * Minimum 4 octets (32 bits), la valeur usuelle pour les chaines courtes.
  */
static void APA102_EndFrame(uint32_t ledCount_)
{
    uint32_t bytes = (ledCount_ + 15U) / 16U;

    if (bytes < 4U)
        bytes = 4U;

    while (bytes--)
        APA102_WriteByte(0xFFU);
}

/**
  * @brief Configuration d'horloge minimale
  *
  * Cette configuration utilise l'horloge HSI interne à 16 MHz.
  */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
    /* Prefetch desactive par defaut au reset (RM0454 3.7.1, FLASH_ACR=0x00040600).
       A 64MHz avec FLASH_LATENCY_2 (2 wait states), chaque franchissement de ligne
       d'instruction de 64 bits coute alors les 2 wait states en entier : c'est ce
       qui faisait deborder le budget de ~80 cycles/bit de la boucle de polling
       WS2812, malgre un corps de boucle court. */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}