/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/** \brief Variables SPI
 */

uint8_t bufftx[10] = "Hello!!\n\r";
uint8_t spiData[2];
char POWER_CTL = 0x2D;  /**< Power Control Register*/
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; /**< X-Axis Data 0*/
char DATAX1 = 0x33; /**< X-Axis Data 1*/
char DATAY0 = 0x34; /**< Y-Axis Data 0*/
char DATAY1 = 0x35; /**< Y-Axis Data 1*/
char DATAZ0 = 0x36; /**< Z-Axis Data 0*/
char DATAZ1 = 0x37; /**< Z-Axis Data 1*/
char values[10];    /**< This buffer will array_joy_xhold values read from the ADXL345 registers.*/

/** \brief Variables para el almacenamiento de los valores
 */

#define N 5					//Tamaño del buffer de datos
#define D 3						//Datos guardados Joy Stick(X,Y), Pie(X)
#define H 10					//Margen por encima de los maximos

int mx[D][N];
int data_csv[16];

#define USB_HID_SCAN_NEXT 0x01
#define USB_HID_SCAN_PREV 0x02
#define USB_HID_STOP      0x04
#define USB_HID_EJECT     0x08
#define USB_HID_PAUSE     0x10
#define USB_HID_MUTE      0x20
#define USB_HID_VOL_UP    0x40
#define USB_HID_VOL_DEC   0x80

// USB keyboard codes
#define USB_HID_MODIFIER_LEFT_CTRL   0x01
#define USB_HID_MODIFIER_LEFT_SHIFT  0x02
#define USB_HID_MODIFIER_LEFT_ALT    0x04
#define USB_HID_MODIFIER_LEFT_GUI    0x08 // (Win/Apple/Meta)
#define USB_HID_MODIFIER_RIGHT_CTRL  0x10
#define USB_HID_MODIFIER_RIGHT_SHIFT 0x20
#define USB_HID_MODIFIER_RIGHT_ALT   0x40
#define USB_HID_MODIFIER_RIGHT_GUI   0x80

#define KEY_NONE 0x00 // No key pressed
#define KEY_ERR_OVF 0x01 //  Keyboard Error Roll Over - used for all slots if too many keys are pressed ("Phantom key")
// 0x02 //  Keyboard POST Fail
// 0x03 //  Keyboard Error Undefined
#define KEY_A 0x04 // Keyboard a and A
#define KEY_B 0x05 // Keyboard b and B
#define KEY_C 0x06 // Keyboard c and C
#define KEY_D 0x07 // Keyboard d and D
#define KEY_E 0x08 // Keyboard e and E
#define KEY_F 0x09 // Keyboard f and F
#define KEY_G 0x0a // Keyboard g and G
#define KEY_H 0x0b // Keyboard h and H
#define KEY_I 0x0c // Keyboard i and I
#define KEY_J 0x0d // Keyboard j and J
#define KEY_K 0x0e // Keyboard k and K
#define KEY_L 0x0f // Keyboard l and L
#define KEY_M 0x10 // Keyboard m and M
#define KEY_N 0x11 // Keyboard n and N
#define KEY_O 0x12 // Keyboard o and O
#define KEY_P 0x13 // Keyboard p and P
#define KEY_Q 0x14 // Keyboard q and Q
#define KEY_R 0x15 // Keyboard r and R
#define KEY_S 0x16 // Keyboard s and S
#define KEY_T 0x17 // Keyboard t and T
#define KEY_U 0x18 // Keyboard u and U
#define KEY_V 0x19 // Keyboard v and V
#define KEY_W 0x1a // Keyboard w and W
#define KEY_X 0x1b // Keyboard x and X
#define KEY_Y 0x1c // Keyboard y and Y
#define KEY_Z 0x1d // Keyboard z and Z

#define KEY_1 0x1e // Keyboard 1 and !
#define KEY_2 0x1f // Keyboard 2 and @
#define KEY_3 0x20 // Keyboard 3 and #
#define KEY_4 0x21 // Keyboard 4 and $
#define KEY_5 0x22 // Keyboard 5 and %
#define KEY_6 0x23 // Keyboard 6 and ^
#define KEY_7 0x24 // Keyboard 7 and &
#define KEY_8 0x25 // Keyboard 8 and *
#define KEY_9 0x26 // Keyboard 9 and (
#define KEY_0 0x27 // Keyboard 0 and )

#define KEY_COMA 0x36 // Keyboard , and <
#define KEY_DOT 0x37 // Keyboard . and >


#define KEY_ENTER 0x28 // Keyboard Return (ENTER)
#define KEY_ESC 0x29 // Keyboard ESCAPE
#define KEY_BACKSPACE 0x2a // Keyboard DELETE (Backspace)
#define KEY_TAB 0x2b // Keyboard Tab
#define KEY_SPACE 0x2c // Keyboard Spacebar



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	
struct keyboardHID_t {
		uint8_t id;
		uint8_t modifiers;
		uint8_t key1;
		uint8_t key2;
		uint8_t key3;
};

void keyboard(uint8_t modificador, uint8_t letra){
	  struct keyboardHID_t keyboardHID;
		keyboardHID.id = 1;
		keyboardHID.modifiers = modificador;
		keyboardHID.key1 = letra;
		keyboardHID.key2 = 0;
		keyboardHID.key3 = 0;
    USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
		HAL_Delay(30);

		keyboardHID.id = 1;
		keyboardHID.modifiers = 0;
		keyboardHID.key1 = 0;
		keyboardHID.key2 = 0;
		keyboardHID.key3 = 0;
    USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
    HAL_Delay(30);
}
	
PUTCHAR_PROTOTYPE
{

  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/** \brief Configura los dispositivos adxl 
 *
 *
 * \param uint_16 CS dispositivo seleccionado
 * \return devuelve un int con el valor leido
 */
void init_SPI(uint16_t CS){
	
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_SET);
	HAL_Delay(10);
	
	//Write operation
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_RESET);
	spiData[0] = DATA_FORMAT;
	spiData[1] = 0x01;
	HAL_SPI_Transmit(&hspi1,spiData,2,10);
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_SET);
	
	//Write operation
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_RESET);
	spiData[0] = POWER_CTL;
	spiData[1] = 0x08;
	HAL_SPI_Transmit(&hspi1,spiData,2,10);
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_SET);
	HAL_Delay(10);
	
}
	/** \brief Lee el dato que se encuentra en la posicion de memoria que especifiacmos
 *
 *
 * \param char address
 					char DATAX0 = 0x32; 
					char DATAX1 = 0x33; 
					char DATAY0 = 0x34; 
					char DATAY1 = 0x35; 
					char DATAZ0 = 0x36; 
					char DATAZ1 = 0x37; 
 * \param uint_16 CS dispositivo seleccionado
 * \return devuelve un int con el valor leido
 */
int spi_read(char address, uint16_t CS){
	uint8_t spiData1[2];
	
	spiData1[0] = 0x80 | address;
	
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1,spiData1,1,10);
	HAL_SPI_Receive(&hspi1,&spiData1[1],1,100);
	HAL_GPIO_WritePin(GPIOA,CS,GPIO_PIN_SET);
	
	//printf("DATAX0 = %d\t\t",spiData1[1]);
	
	return spiData1[1];
}
/** \brief Lee los seis datos que envia el adxl345 
 *
 *
 * \param uint_16 CS dispositivo seleccionado
 * \return devuelve un int con el valor leido
 */
void adxl_read(uint16_t CS, int pos[]){
	int values[6];
	for(int i=0;i<6;i++){
		values[i]= spi_read(DATAX0+i,CS);
		//printf("%x\t",values[i]);
	}
	
	int pos_16[3];
	pos_16[0] = (values[1] << 8) | values[0];  //< The X value is stored in values[0] and values[1].*/
  pos_16[1] = ((int)values[3] << 8) | (int)values[2];  //< The Y value is stored in values[2] and values[3].*/
  pos_16[2] = ((int)values[5] << 8) | (int)values[4];  //< The Z value is stored in values[4] and values[5].*/

	pos[0] = -entero16_to32(pos_16[0]);
	pos[1] = -entero16_to32(pos_16[1]);
	pos[2] = entero16_to32(pos_16[2]);
	
	for(int i=0;i<3;i++){
		//printf("%4d\t",pos[i]);
	}
	
}
/** \brief Convierte un numero en complemento a 2 de 16 bits en uno de 32, en el caso de que el bit numero 15 se 1
 *					completaremos los proximos 16 bits con ffff de esta manera extenderemos el signo.
 *
 *
 * \param uint_16 CS dispositivo seleccionado
 * \return devuelve un int con el valor leido
 */

int entero16_to32(int c){
	
			int sol;

    if((0x00008000&c) == 0x00008000){
        //printf("Se detecto numero negativo\n\r");
        sol = 0xFFFF0000|c;

    }
    //printf("%x\t%d\n\r",c,c);
    return sol;
		
}

void add_value(int *p,int x){

    for(int i=N-1; i>=0; i--){       //IMPOTANTE
        *(p+i) = *(p+i-1);
    }
    *p =  x;
}

 /** \brief Necesario para ejecutar QSORT y poder ordenar el vector
  */
int cmp (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}
void media_min_max(int x[][N],int c, int sol[]){


    int aux_v[N];
    for(int i=0;i<D;i++){                   //Pasamos una fila de la matriz a un vector auxiliar
        for(int j=0;j<N;j++){
            aux_v[j]=x[i][j];
        }

        qsort(aux_v,N,sizeof(aux_v[0]),cmp);    //Ordenamos el vector auxiliar
        int sum=0;
        for(int k=c; k<N-c; k++){             //Sumamos todos los componentes del vector sin contar los max o min
            sum += aux_v[k];                //en funcion del c que es la cantidad de numeros que excluimos
        }
        sol[i] = sum/(N-(2*c));
    }
}

void estabilizador(int mx[][N],int input[], int output[],int excluir){

      for(int i=0;i<D;i++)                      //A?adimos los valores leidos al la primera columna de la matriz
            add_value(&mx[i][0],input[i]);

        media_min_max(mx,excluir,output);


  }

int selector_pie(int x){
	const int npp=4;
	static int max_pie = -140;
	static int min_pie = 100;
	static int paso_pie;
	int sol;
	if((x<min_pie || x>max_pie) && x != 0){

		if(x<min_pie)
			min_pie = x;
		if(x>max_pie)
			max_pie = x;

		if(max_pie == min_pie)
				paso_pie = 100;
		else
				paso_pie = (max_pie-min_pie + 5)/npp;


        //scanf("%s", str1);
	}
	
  sol = (x-min_pie)/paso_pie;
    
	data_csv[11] = x;
	data_csv[12] = sol;
	data_csv[13] = max_pie;
	data_csv[14] = min_pie;
	data_csv[15] = paso_pie;
	return sol;
}

int selector_joy(int x,int y){
	const int npp=3;
	static int max_joy[2] = {-100,-100};
	static int min_joy[2] = {100,100};
	static int paso_joy[2];
	int sx,sy;
	int sol;
	
	if(x<min_joy[0] || x>max_joy[0]){

		if(x<min_joy[0])
			min_joy[0] = x;
		if(x>max_joy[0])
			max_joy[0] = x;

        if(max_joy[0] == min_joy[0])
            paso_joy[0] = 100;
        else
            paso_joy[0] = (max_joy[0] + H - min_joy[0])/npp;
	}
	sx = (x-min_joy[0])/paso_joy[0];
	
	if(y<min_joy[1] || y>max_joy[1]){

		if(y<min_joy[1])
			min_joy[1] = y;
		if(y>max_joy[1])
			max_joy[1] = y;

        if(max_joy[1] == min_joy[1])
            paso_joy[1] = 100;
        else
            paso_joy[1] = (max_joy[1] + H - min_joy[1])/npp;


        //scanf("%s", str1);
	}
    sy = (y-min_joy[1])/paso_joy[1];
	
	data_csv[0] = x;
	data_csv[1] = sx;
	data_csv[2] = max_joy[0];
	data_csv[3] = min_joy[0];
	data_csv[4] = paso_joy[0];
	data_csv[5] = y;
	data_csv[6] = sy;
	data_csv[7] = max_joy[1];
	data_csv[8] = min_joy[1];
	data_csv[9] = paso_joy[1];
	
	
	
	//printf("\fDato X= %3d  sel = %3d  max = %3d  min = %3d  paso = %3d",x,sx,max_joy[0],min_joy[0], paso_joy[0]);
	//printf("Dato Y= %3d   seleccion en Y = %3d      ",y,sy);

	sol = sx+3*sy;
	data_csv[10] = sol;
	//printf("   Pos = %d      \n\r",sol);

	return sol;
}

int selector_letra(int joy, int pie){
    int num_letra = joy * 4 + pie;
		return num_letra;
}

void greeting(){
	printf("\f\nStarting");
	for(int i = 0; i < 20; i++){
		printf(",");
		HAL_Delay(200);
		
		int out_adxl_4[3];
		adxl_read(GPIO_PIN_4,out_adxl_4);
			
		//printf("\tGPIO_PIN_5\t");
		int out_adxl_5[3];
		adxl_read(GPIO_PIN_5,out_adxl_5);
		int out_adxl_mix[3];
		out_adxl_mix[0] = out_adxl_4[0];
		out_adxl_mix[1] = out_adxl_4[1];
		out_adxl_mix[2] = out_adxl_5[0];
		
		//printf("\n\r");
		int adxl_fitrada[3];
		estabilizador(mx,out_adxl_mix,adxl_fitrada,2);
	}
	printf("\n");
	HAL_Delay(200);
	printf("OK\n");
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	uint8_t cadena_letras[] = {KEY_BACKSPACE, KEY_BACKSPACE, KEY_BACKSPACE, KEY_BACKSPACE,    KEY_A,KEY_B,KEY_C,KEY_D,    KEY_E,KEY_F,KEY_G,KEY_H,    KEY_Y,KEY_Z,KEY_DOT,KEY_COMA,    KEY_SPACE, KEY_SPACE, KEY_ENTER, KEY_ENTER,    KEY_I,KEY_J,KEY_K,KEY_L,  KEY_U,KEY_V,KEY_W,KEY_X,    KEY_Q,KEY_R,KEY_S,KEY_T,    KEY_M,KEY_N,KEY_O,KEY_P};


	// HID Media
  struct mediaHID_t {
    uint8_t id;
    uint8_t keys;
  };

  struct mediaHID_t mediaHID;
  mediaHID.id = 2;
  mediaHID.keys = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	
	init_SPI(GPIO_PIN_4);
	init_SPI(GPIO_PIN_5);
	
	int out_adxl_mix[3];
	int adxl_fitrada[3];
	
	greeting();
	bool old_value = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//lectura_SPI();
		//printf("GPIO_PIN_4\t");
		int out_adxl_4[3];
		adxl_read(GPIO_PIN_4,out_adxl_4);
			
		//printf("\tGPIO_PIN_5\t");
		int out_adxl_5[3];
		adxl_read(GPIO_PIN_5,out_adxl_5);
				
		out_adxl_mix[0] = out_adxl_4[0];
		out_adxl_mix[1] = out_adxl_4[1];
		out_adxl_mix[2] = out_adxl_5[0];
		
		//printf("\n\r");
		
		estabilizador(mx,out_adxl_mix,adxl_fitrada,2);
		/*for(int i = 0; i<3; i++){
			printf("%4d\t",adxl_fitrada[i]);
		}
		
		*/

		int seleccion_joy = selector_joy(adxl_fitrada[0],adxl_fitrada[1]);
		int seleccion_pie = selector_pie(-adxl_fitrada[2]);
		//printf("%4d, %4d, %4d\t\t",out_adxl_4[0],out_adxl_4[1],out_adxl_4[2]);
		//printf("%4d, %4d, %4d\n\r",out_adxl_5[0],out_adxl_5[1],out_adxl_5[2]);
		printf("%4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\n\r",data_csv[0],data_csv[1],data_csv[2],data_csv[3],data_csv[4],data_csv[5],data_csv[6],data_csv[7],data_csv[8],data_csv[9],data_csv[10],data_csv[11],data_csv[12],data_csv[13],data_csv[14],data_csv[15]);
		//printf("\fDato X= %3d  sel = %3d  max = %3d  min = %3d  paso = %3d  Pos = %3d",data_csv[0],data_csv[1],data_csv[2],data_csv[3], data_csv[4],data_csv[5]);
	
		//selector_pie(adxl_fitrada[0]);
		//printf("\n\r");
		
		bool value = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);

		if(!value && old_value){

			//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
			keyboard(0x00,cadena_letras[selector_letra(seleccion_joy,seleccion_pie)]);

		}

		old_value = value;
		
		HAL_Delay(10);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
