 /******************************************************************************
  * @file    main.c
  * @author  FreeRTOS nnd@isep.ipp.pt
  * @version V1.2
  * @date    16/12/2021
  * @brief   SISTR FreeRTOS Mutex/Semaphores/Queues project
  ******************************************************************************/

// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"

#include "semphr.h" //Semafores e Mutex
#include "queue.h"	//Filas de mensagens

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define pergunta2_TASK_PRIORITY	( tskIDLE_PRIORITY + 1) //Prioridade Task relativa à pergunta 2

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_PERIOD_MS )
/* The rate at LCD is refreshed. */
#define mainLCD_DELAY			( ( TickType_t ) 2000 / portTICK_PERIOD_MS )
/* The rate at which the message is sent to the USART. */
#define mainUSART_DELAY			( ( TickType_t ) 1000 / portTICK_PERIOD_MS )

 /* Configure RCC clock at 72 MHz */
static void prvSetupRCC( void );

 /* Configure GPIO. */
static void prvSetupGPIO( void );

/* Simple LED toggle task. */
static void prvLedPisca( void *pvParameters );

//Pergunta 2
static void prvPergunta2( void *pvParameters );

//Pergunta 6
static void prvPergunta6( void *pvParameters );

static void prvPrintI2CData( void *pvParameters );

void pvrI2C_init(uint16_t buffer_depth_i2c );


/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);
/***************************************/

void prvUSART2_Init (uint32_t baudrate, uint16_t buffer_depth_send, uint16_t buffer_depth_receive);
char prvUSART_GetChar();
void prvUSART_PutChar(char put_char);
void prvUSART_PutString(char *put_string, uint8_t string_length);
void USART_Flush();
void USART_Close();

void joystick_setup(void);
char joystick_read();
void joystick_flush();

/* Task 1 handle variable. */
TaskHandle_t HandleTask1;
TaskHandle_t HandleTask2;
TaskHandle_t HandleTask3;
TaskHandle_t HandleTask4;
TaskHandle_t HandleTask5;

struct structI2C {
		 int16_t X;
		 int16_t Y;
		 int16_t Z;
	};

SemaphoreHandle_t xSemaphore; /* Global variable. --> Semaphore*/

volatile QueueHandle_t xUSART_Send; /* Global variable. --> Fila de mensagens - Dados a enviar pela USART */

volatile QueueHandle_t xUSART_Receive; /* Global variable. --> Fila de mensagens - Dados a receber pela USART */

volatile QueueHandle_t xButton_State; /* Global variable. --> Fila de mensagens - Informa qual foi botão pressionado*/

QueueHandle_t xQueueI2C_Data; /* Global variable. --> Fila de mensagens - Valores do I2C*/

QueueHandle_t xOpcao_Menu; /* Global variable. --> Fila de mensagens - Valores do I2C*/

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();

    prvUSART2_Init(115200,40,10);
    joystick_setup();

    pvrI2C_init(20);

    lcd_init ();

    i2c_send(0x2D, 0b00001000); 	//ADXL345 - measure mode
	i2c_send(0x31, 0b00001000);		//ADXL345 - data-format

	 xSemaphore = xSemaphoreCreateBinary();

	xOpcao_Menu = xQueueCreate( 2, sizeof( uint8_t ) ); //Criar Fila de Mensagens

	/* Create the tasks */
 	xTaskCreate( prvLedPisca, "LED_PISCA", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask1 );
	xTaskCreate( prvPrintI2CData, "Print_I2C_Data", configMINIMAL_STACK_SIZE+512, NULL, pergunta2_TASK_PRIORITY, &HandleTask4 );
 	xTaskCreate( prvPergunta6, "Pergunta6", configMINIMAL_STACK_SIZE, NULL, pergunta2_TASK_PRIORITY, &HandleTask5 );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/


static void prvLedPisca( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY ); //1s
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6)));
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/

static void prvPrintI2CData( void *pvParameters )
{
	TickType_t xLastExecutionTime;

	xLastExecutionTime = xTaskGetTickCount();

	struct structI2C I2C_Data;
	char aux[30];
	uint8_t opcao=0;
	char button;
	char recebido;


	for (;;)
	{
		if (xQueueReceive(xOpcao_Menu,&opcao, ( TickType_t ) 20 / portTICK_PERIOD_MS ) == pdTRUE)
		{
			switch(opcao)
			{
			case 1:
				i2c_receive();
				xQueueReceive(xQueueI2C_Data,&I2C_Data,(TickType_t) portMAX_DELAY );
				sprintf (aux, "%d %d %d \r\n", I2C_Data.X,I2C_Data.Y,I2C_Data.Z);
				prvUSART_PutString( aux, strlen(aux));
				break;
			case 2:
				lcd_draw_fillrect(0, 0, 127, 64, BLACK);
				lcd_draw_string(0,0,"Direita->Chrome", WHITE, 1);
				lcd_draw_string(0,20,"Trás->SofaScore", WHITE, 1);
				lcd_draw_string(0,44,"Esquerda->Zoom ", WHITE, 1);
				display();

				recebido = prvUSART_GetChar();
				while (recebido != '0')
				{
//					vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY ); //1s --> se quisere por delay
					i2c_receive();
					xQueueReceive(xQueueI2C_Data,&I2C_Data,(TickType_t) portMAX_DELAY );
					sprintf (aux, "%d %d %d \r\n", I2C_Data.X,I2C_Data.Y,I2C_Data.Z);
					prvUSART_PutString( aux, strlen(aux));
					recebido = prvUSART_GetChar();
				}

				menu_display();
				break;
			case 3:
				lcd_draw_fillrect(0, 0, 127, 64, BLACK);
				lcd_draw_string(15, 3 ,"Grafico - Eixo X", 0xFFFF, 1 );
				lcd_draw_line(1,16, 1,62, WHITE); //Eixo X
				lcd_draw_line(1,39, 126,39, WHITE); //Eixo Y

				for (uint8_t i=0; i<=125;i++)
				{
					i2c_receive();
					xQueueReceive(xQueueI2C_Data,&I2C_Data,(TickType_t) portMAX_DELAY );
//					GPIO_WriteBit(GPIOA, GPIO_Pin_5, (1-GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)));
//					grafico_tempo(i,(I2C_Data.X));
					lcd_draw_pixel(i+1, (-0.0118*(I2C_Data.X)) + 39, WHITE);
					display();
				}

				joystick_flush();
				button = joystick_read();

				while (button != 'E')
				{
					button = joystick_read();
				}

				menu_display();
				break;
			case 4:
				lcd_draw_fillrect(0, 0, 127, 64, BLACK);
				lcd_draw_string(15, 3 ,"Grafico - Eixo Y", 0xFFFF, 1 );
				lcd_draw_line(1,16, 1,62, WHITE); //Eixo X
				lcd_draw_line(1,39, 126,39, WHITE); //Eixo Y

				for (uint8_t i=0; i<=125;i++)
				{
					i2c_receive();
					xQueueReceive(xQueueI2C_Data,&I2C_Data,(TickType_t) portMAX_DELAY );
//					GPIO_WriteBit(GPIOA, GPIO_Pin_5, (1-GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)));
//					grafico_tempo(i,(I2C_Data.X));
					lcd_draw_pixel(i+1, (-0.0118*(I2C_Data.Y)) + 39, WHITE);
					display();
				}

				joystick_flush();
				button = joystick_read();

				while (button != 'E')
				{
					button = joystick_read();
				}

				menu_display();
				break;
			case 5:
				lcd_draw_fillrect(0, 0, 127, 64, BLACK);
				lcd_draw_string(15, 3 ,"Grafico - Eixo Z", 0xFFFF, 1 );
				lcd_draw_line(1,16, 1,62, WHITE); //Eixo X
				lcd_draw_line(1,39, 126,39, WHITE); //Eixo Y

				for (uint8_t i=0; i<=125;i++)
				{
					i2c_receive();
					xQueueReceive(xQueueI2C_Data,&I2C_Data,(TickType_t) portMAX_DELAY );
//					GPIO_WriteBit(GPIOA, GPIO_Pin_5, (1-GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)));
//					grafico_tempo(i,(I2C_Data.X));
					lcd_draw_pixel(i+1, (-0.0118*(I2C_Data.Z)) + 39, WHITE);
					display();
				}

				joystick_flush();
				button = joystick_read();

				while (button != 'E')
				{
					button = joystick_read();
				}

				menu_display();
				break;
			default:
				break;
			}
			xSemaphoreGive( xSemaphore);
		}
		else
		{

		}


	}

}

void grafico_tempo(uint8_t posicao_x, int16_t posicao_y)
{

	lcd_draw_pixel(posicao_x+1, (-0.0118*(posicao_y)) + 39, WHITE);
	display();
}
/*-------------------------------------------------------------------------------------------------------------------------*/



/*-----------------------------------------------------> PERGUNTA 6 <-----------------------------------------------------*/
#define offset_opcao 12

static void prvPergunta6( void *pvParameters )
{
	menu_display();
	char button;
	uint8_t opcao=1;

	for(;;)
	{
		joystick_flush();
		button = joystick_read();

		if ( (button) == ('0') )
		{

		}
		else
		{
			switch (button)
			{
			case 'D':
				if (opcao == 1)
				{
					lcd_draw_fillrect(0,0,6,64, BLACK); //limpa coluna das opções --> "X"
					lcd_draw_string(0,9,"X", WHITE, 1);
					opcao ++;
				}
				else if (opcao == 6)
				{
					lcd_draw_fillrect(0,0,6,64, BLACK); //limpa coluna das opções --> "X"
					lcd_draw_string(0,0,"X", WHITE, 1);
					opcao = 1;
				}
				else
				{
					lcd_draw_fillrect(0,0,6,64, BLACK); //limpa coluna das opções --> "X"
					lcd_draw_string(0,( 9+(offset_opcao*(opcao-1)) ),"X", WHITE, 1);
					opcao ++;
				}
				break;

			case 'U':
				if (opcao == 1)
				{
					lcd_draw_fillrect(0,0,6,64, BLACK); //limpa coluna das opções --> "X"
					lcd_draw_string(0,57,"X", WHITE, 1);
					opcao = 6;
				}
				else if (opcao == 2)
				{
					lcd_draw_fillrect(0,0,6,64, BLACK); //limpa coluna das opções --> "X"
					lcd_draw_string(0,0,"X", WHITE, 1);
					opcao = 1;
				}
				else
				{
					lcd_draw_fillrect(0,0,6,64, BLACK); //limpa coluna das opções --> "X"
					lcd_draw_string(0,( (offset_opcao*(opcao-1)) - 15 ),"X", WHITE, 1);
					opcao --;
				}
				break;

			case 'E':
				xQueueReset(xOpcao_Menu);
				xQueueSendToBack( xOpcao_Menu, ( void * ) &opcao, ( TickType_t ) portMAX_DELAY );
				opcao = 1 ;
				xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY);
			} //switch
			display();
		}
	}
}
/*-------------------------------------------------------------------------------------------------------------------------*/
void menu_display ()
{
	lcd_draw_fillrect(0, 0, 128, 64, BLACK);
	lcd_draw_string(0,0,"X <- Amostra unica", WHITE, 1);
	lcd_draw_string(0,9,"  <- Executer", WHITE, 1);
	lcd_draw_string(0,21,"  <- Graf. Eixo x", WHITE, 1);
	lcd_draw_string(0,33,"  <- Graf. Eixo Y ", WHITE, 1);
	lcd_draw_string(0,45,"  <- Graf. Eixo Z ", WHITE, 1);
	lcd_draw_string(0,57,"  <- Menu", WHITE, 1);
	display();
}

static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);

        /* No division HCLK = SYSCLK = 72 MHz*/
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE = 12 MHz (12 MHz * 6 = 72 MHz) */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );

        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else
    {
    	/* HSE error? No further action */
        while(1);
    }
}

/*-------------------------------------------------------------------------------------------------------------------------*/

static void prvSetupGPIO( void )
{
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //GPIOB6 --> LED PLACA DE EXPANSÃO
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*-------------------------------------------------------------------------------------------------------------------------*/

void pvrI2C_init(uint16_t buffer_depth_i2c )
{
	//Enable GPIOB, AFIO e REMAP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

	//SCL - GPIOB8
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//SDA -> GPIOB9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	// ----------------- Configurações I2C ----------------- //

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	xQueueI2C_Data = xQueueCreate( buffer_depth_i2c, sizeof(struct structI2C)); //Criar Fila de Mensagens I2C_Data
}

void i2c_send(uint8_t addr, uint8_t data)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

	I2C_Send7bitAddress(I2C1, 0xA6, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, addr);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);

	I2C_SendData(I2C1, data);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);

	I2C_GenerateSTOP(I2C1, ENABLE);
}

void i2c_receive()
{
	struct structI2C I2C_Data;

	 uint8_t x0;			//guarda valor do registo menos (0x32) significativo
	 uint8_t x1;
	 uint8_t y0;			//guarda valor do registo menos (0x34) significativo
	 uint8_t y1;
	 uint8_t z0;			//guarda valor do registo menos (0x36) significativo
	 uint8_t z1;

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

	I2C_Send7bitAddress(I2C1, 0xA6, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, 0x32);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);

	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

	I2C_Send7bitAddress(I2C1, 0xA7, I2C_Direction_Receiver);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);


	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	x0 = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	x1 = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	y0= I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	y1= I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	z0 = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	z1 = I2C_ReceiveData(I2C1);

	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);

	I2C_Data.X = ((x1 << 8) + x0);				//conversao g --> m/s" | x100 por causa das casas decimais
	I2C_Data.X = ((I2C_Data.X)  * 0.0039 * 9.81) * 100;
	I2C_Data.Y = ((y1 << 8) + y0) ;
	I2C_Data.Y = ((I2C_Data.Y)  * 0.0039 * 9.81) * 100;
	I2C_Data.Z = ((z1 << 8) + z0);
	I2C_Data.Z = ((I2C_Data.Z)  * 0.0039 * 9.81) * 100;

	xQueueSendToBack( xQueueI2C_Data, ( void * ) &I2C_Data, ( TickType_t ) portMAX_DELAY );

}



/* ---------------------------------------------------------- */
/* ---------------------- DRIVER USART ---------------------- */
/* ---------------------------------------------------------- */

void prvUSART2_Init (uint32_t baudrate, uint16_t buffer_depth_send, uint16_t buffer_depth_receive)  //baudrate e tamanho para a fila de mensagens
{																									//de transmissão e receção

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

	/* USART Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	/* Configure the USART2 */
	USART_Init(USART2, &USART_InitStructure);
	/* Enable the USART2 */
	USART_Cmd(USART2, ENABLE);

	//-------------------- PRIORIDADES E CONFIG DA USART --------------------//

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Interrupção global do EXTI1 com prioridade 1 sub-prioridade 0 */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //prioridade com numero mais baixo --> maior prioridade
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //ATIVA INT DE RECEÇÃO DA USART

	//-------------------- CRIAÇÃO FILAS DE MENSAGENS --------------------//

	xUSART_Send = xQueueCreate( buffer_depth_send, sizeof( char ) ); //Criar Fila de Mensagens USART Send
	xUSART_Receive = xQueueCreate( buffer_depth_receive, sizeof( char ) ); //Criar Fila de Mensagens USART Receive
}

char prvUSART_GetChar()
{
	char c;
	xQueueReceive( xUSART_Receive, &c, ( TickType_t ) 20 / portTICK_PERIOD_MS  );
	return c;
}

void prvUSART_PutChar(char put_char)
{
	if( xUSART_Send != 0 )
	{
		xQueueSendToBack( xUSART_Send, ( void * ) &put_char, ( TickType_t ) portMAX_DELAY );
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	}
}

void prvUSART_PutString(char *put_string, uint8_t string_length)
{
	if( xUSART_Send != 0 )
	{
		for (uint8_t i = 0 ; i<string_length; i++ )
		{
			xQueueSendToBack( xUSART_Send, ( void * ) &put_string[i], ( TickType_t ) portMAX_DELAY );
		}
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	}
}

void USART_Flush()
{
   xQueueReset(xUSART_Send);
   xQueueReset(xUSART_Receive);
}

void USART_Close()
{
	USART_Cmd(USART2, DISABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    vQueueDelete(xUSART_Send);
    vQueueDelete(xUSART_Receive);
}




/* ---------------------------------------------------------- */
/* --------------------- DRIVER JOYSTICK -------------------- */
/* ---------------------------------------------------------- */

void joystick_setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;  // UP
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4; // RIGHT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0; //LEFT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2; // ENTER
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3; // DOWN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*------------------- PRIORIDADES NVCIS DE CADA BOTÃO -------------------*/

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitTypeDef NVIC_InitStructure;

	//EXTI0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //prioridade com numero mais baixo --> maior prioridade
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//EXTI1
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //prioridade com numero mais baixo --> maior prioridade
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//EXTI2
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //prioridade com numero mais baixo --> maior prioridade
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//EXTI3
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //prioridade com numero mais baixo --> maior prioridade
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//EXTI4
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //prioridade com numero mais baixo --> maior prioridade
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*------------------- CONFIG INT. EXTERNA DE CADA BOTÃO -------------------*/

	EXTI_InitTypeDef EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); /* Ativar o clock AFIO para o uso do EXTI */

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); // UP
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4); // RIGHT
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0); // LEFT
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2); // ENTER
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3); // DOWN
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//-------------------- CRIAÇÃO FILAS DE MENSAGENS --------------------//

	xButton_State = xQueueCreate( 15, sizeof( char ) );
}

char joystick_read()
{
	static char c;
	if( xButton_State != 0 )
	{
		if(xQueueReceive( xButton_State, &c, ( TickType_t ) 20 / portTICK_PERIOD_MS ) == pdTRUE)
		{
			return c;
		}
	}
	return '0';
}

void joystick_flush()
{
	xQueueReset(xButton_State);
}




