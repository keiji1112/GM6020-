/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */


#include "mbed.h"
#include "CAN.h"
DigitalOut LED(LED1);
CAN can1(PB_8,PB_9,1000*1000);//PB_8が受信側(CAN1_RD) PB_9が送信側(CAN1_TD)
//CAN can1(PD_0, PD_1, 125000); f767用ピン
Timer t;
int16_t shift_data_robo ;
int16_t shift_data_msg ;


short target;
const float gain_p = 0.1f;
const float gain_d = 0.1f;
float pre_diff = 0.0f;
float differential = 0.0f;

//pulopo
//constexprはコンパイル時に値が確定して、定数として計算される
constexpr int NUM_OF_STICK = 4;  // スティックのチャンネル数
constexpr int NUM_OF_SWITCH = 6; // スイッチのチャンネル数
constexpr int NUM_OF_CH = NUM_OF_STICK + NUM_OF_SWITCH; // プロポの合計のチャンネル数
constexpr int PROPO_MAINBUF_SIZE = 25; // propoMainBufとpropoMainDataの要素数

//ユーザー定義型スイッチの状態を３つに分けている
enum swichDataEnum {
    switchOff,     // スイッチOFF
    switchNeutral, // スイッチニュートラル
    switchOn,      // スイッチON
};

float stickData[NUM_OF_STICK] = {0}; // プロポのスティックの受信データを格納(0:1ch 1:2ch 2:3ch 3:4ch)
swichDataEnum switchData[NUM_OF_SWITCH]; // プロポのスイッチの受信データを格納(0:swichA 1:swichB 2:swichC 3:swichD 4:swichE 5:swichF)

uint8_t propoMainBuf[PROPO_MAINBUF_SIZE]; // DMAでペリフェラルによりデータが書き込まれるメモリ
//DMAを使っているのでwhile文で読み込む必要はない

uint8_t propoMainData[PROPO_MAINBUF_SIZE]; // propoMainBufをコピーして使う
//while文の中で元のデータをいじるときに使う

void EncodePulopoDataBySbus(float stickData[4], swichDataEnum switchData[6],const uint8_t sBusData[25]);
//生データを使いやすいデータに変える関数

UART_HandleTypeDef huart4;
//UARTのいろいろな設定を保持できる構造体
DMA_HandleTypeDef hdma_uart4_rx;
//受信に使うDMAのいろいろな設定を保持できる構造体

static void MX_UART4_Init(void);

static void MX_UART4_DMA_Init(void);

int MX_UART4_DeInit(void);

int MX_DMA_UART4_DeInit(void);

int MX_GPIO_DeInit(void);

void Error_Handler(void);
int main()
{
    t.start(); 
    LED=1;
    //送信
    CANMessage robo;
    robo.id = 0x1ff;
    robo.len = 8;
    //robo.data[0] = -15;
    robo.data[1] = 0;
    
     //受信
     CANMessage msg;
     msg.id =0x205;
     msg.len =8;
    MX_UART4_DMA_Init();
    MX_UART4_Init();
    //printf("HAL_UART_Receive_DMA() return %d\r\n",HAL_UART_Receive_DMA(&huart4, propoMainBuf,PROPO_MAINBUF_SIZE)); // プロポと通信開始(SerialDMA)
  
    while (1) {
    

        if (propoMainBuf[0] == 0x0f) // もしスタートバイトが配列の1つ目にあるなら
        {
            for (int i = 0; i < PROPO_MAINBUF_SIZE; i++) // 受信機からのデータを取得
            {
            propoMainData[i] = propoMainBuf[i];
            }
            EncodePulopoDataBySbus(stickData, switchData, propoMainData);
        }
        else {
            // // こちらがDMAをリセットして再度初期化する何か
            // /*DMAとUARTとGPIOを初期化解除*/
            printf("MX_DMA_UART4_DeInit() return %d\r\n", MX_DMA_UART4_DeInit()); // おそらく必要ない？
            // pc.printf("MX_GPIO_DeInit() return %d\r\n",MX_GPIO_DeInit());
            printf("MX_UART4_DeInit() return %d\r\n", MX_UART4_DeInit());

            // /*DMAとUARTとGPIOを再度初期化*/
            MX_UART4_DMA_Init();
            MX_UART4_Init();
            printf("HAL_UART_Receive_DMA() return %d\r\n",HAL_UART_Receive_DMA(&huart4, propoMainBuf, PROPO_MAINBUF_SIZE)); // プロポと通信開始(SerialDMA)
        }


        //スティックデータ 
        //printf("LX:%+4f LY:%+4f RX:%+4f RY:%+4f\r\n",stickData[3],stickData[2],stickData[0],stickData[1]);
        

     //送信
     //pd制御
       if(std::abs(stickData[2])>0.006){
    target= 180*stickData[2];
       }
       else target=0;
    float diff = target -shift_data_msg; //目標値との偏差をとる
    differential = (diff - pre_diff)/1000; //微分
    pre_diff = diff;

    robo.data[0] +=gain_p * diff + gain_d * differential;//偏差にゲインをかけて操作量に変換する
    //robo.data[0] = 50*stickData[2];
     can1.write(robo);
     shift_data_robo =(robo.data[0]<<8)+robo.data[1];
    
    //受信
    can1.read(msg);
     shift_data_msg =(msg.data[2]<<8)+msg.data[3];//速度

    //printf("%d %d %d %d %d\n", msg.data[2]<<8,msg.data[2],msg.data[3],(msg.data[2]<<8)+msg.data[3],shift_data_msg);

    printf("rpm:%d stickData[2]:%f  \n",shift_data_msg,stickData[2]);
     
    }
     while(t.read_us() <= 1000){};
        t.stop();
        t.reset();
     
}



void EncodePulopoDataBySbus(float stickData[4], swichDataEnum switchData[6], const uint8_t sBusData[25]) {
    //stickData配列のアドレス、switchData配列のアドレス、生データの配列のアドレス（おそらくただの引数として使っている）

    bool PropoMainDataBit[PROPO_MAINBUF_SIZE * 8];//8は多分１byteのこと
    int CH_data[16];//16個のチャンネル、4個はスティック、6個はボタン

    //生データをbitごとに分割して格納している
    for (int i = 0; i < PROPO_MAINBUF_SIZE - 1; i++) {
        PropoMainDataBit[i * 8 + 0] = sBusData[i + 1] & 0b00000001;
        PropoMainDataBit[i * 8 + 1] = sBusData[i + 1] & 0b00000010;
        PropoMainDataBit[i * 8 + 2] = sBusData[i + 1] & 0b00000100;
        PropoMainDataBit[i * 8 + 3] = sBusData[i + 1] & 0b00001000;
        PropoMainDataBit[i * 8 + 4] = sBusData[i + 1] & 0b00010000;
        PropoMainDataBit[i * 8 + 5] = sBusData[i + 1] & 0b00100000;
        PropoMainDataBit[i * 8 + 6] = sBusData[i + 1] & 0b01000000;
        PropoMainDataBit[i * 8 + 7] = sBusData[i + 1] & 0b10000000;
    }

    //16個のチャンネルに11bitずつ格納
    for (int i = 0; i < 16; i++) {
        CH_data[i] = 0;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 0] << 0;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 1] << 1;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 2] << 2;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 3] << 3;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 4] << 4;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 5] << 5;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 6] << 6;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 7] << 7;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 8] << 8;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 9] << 9;
        CH_data[i] = CH_data[i] | PropoMainDataBit[i * 11 + 10] << 10;
        // printf("%d,", CH_data[i]); // デバック用
    }
    // printf("\r\n"); // デバック用

    // 入力信号を-1~+1に変換
    stickData[0] = (CH_data[0] - 1027.0) / (1688.0 - 374.0) * 2;
    stickData[1] = (CH_data[1] - 1024.0) / (1680.0 - 368.0) * 2;
    stickData[2] = (CH_data[2] - 1025.0) / (1680.0 - 412.0) * 2;
    stickData[3] = (CH_data[3] - 1027.0) / (1673.0 - 360.0) * 2;
    // printf("%f,%f,%f,%f\r\n", stickData[0], stickData[1], stickData[2], stickData[3]);

    //                         CH5   CH6   CH7   CH8   CH9  CH10
    const int OFF_LIMIT[6] = {1500, 1500, 1500, 1500, 1500, 1500}; // スイッチOFFとする上限
    //                       CH5  CH6  CH7  CH8  CH9  CH10
    const int ON_LIMIT[6] = {500, 500, 500, 500, 500, 500}; // スイッチONとする下限

    //ボタンのチャンネル６個の判断
    //0から2047の間で、500より小さいとON、1500より大きいとOFF、それ以外はニュートラル
    for (int i = 0; i < 6; i++) {
        if (CH_data[i + 4] < ON_LIMIT[i]) {
            switchData[i] = switchOn;
        }
        else if (CH_data[i + 4] > OFF_LIMIT[i]) {
            switchData[i] = switchOff;
        }
        else {
            switchData[i] = switchNeutral;
        }
    }
}

//staticをつけると他のファイルから呼び出せなくなる
static void MX_UART4_Init(void) {

    /* USER CODE BEGIN UART4_Init 0 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    //UART4を使う

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 100000;
    huart4.Init.WordLength = UART_WORDLENGTH_9B;
    huart4.Init.StopBits = UART_STOPBITS_2;
    huart4.Init.Parity = UART_PARITY_EVEN;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    //huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    //huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    //huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;

    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
}

//staticをつけると他のファイルから呼び出せなくなる
static void MX_UART4_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

    /* DMA interrupt init */
    /* DMA1_Stream6_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 4, 4);//この行消したらDMA使えた
    // HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);//この行消したらDMA使えた
   // printf("MX_UART4_DMA_Init() end\r\n");
}

int MX_UART4_DeInit(void) {
    if (HAL_UART_DeInit(&huart4) != HAL_OK) {
        return 1;
    }
    return 0;
}

int MX_DMA_UART4_DeInit(void) {
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
    if (HAL_DMA_DeInit(&hdma_uart4_rx) != HAL_OK) {
        return 1;
    }
    return 0;
}

int MX_GPIO_DeInit(void) {
    HAL_GPIO_DeInit(GPIOC, 10);
    HAL_GPIO_DeInit(GPIOC, 11);
    return 0;
}

void Error_Handler(void) {
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {};
}