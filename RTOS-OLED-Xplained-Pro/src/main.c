/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <stdlib.h>
#include <string.h>
#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* IOS                                                                  */
/************************************************************************/

#define BTN_PIO PIOA
#define BTN_PIO_ID ID_PIOA
#define BTN_PIO_PIN 11
#define BTN_PIO_PIN_MASK (1 << BTN_PIO_PIN)

// Configuracao do buzzer
#define BUZZER_PIO			  PIOC
#define BUZZER_PIO_ID         ID_PIOC
#define BUZZER_PIO_IDX        13
#define BUZZER_PIO_IDX_MASK   (1 << BUZZER_PIO_IDX)

/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/

void btn_init(void);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void but_callback(void);
void tone(int frequency, int duration);

/************************************************************************/
/* rtos vars                                                            */
/************************************************************************/

SemaphoreHandle_t xBtnSemaphore;

QueueHandle_t xQueueCoins;

#define NOTE_B5  988
#define NOTE_E6  1319

volatile int flag_rtt;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_PLAY_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_PLAY_STACK_PRIORITY            (tskIDLE_PRIORITY + 1)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	xSemaphoreGiveFromISR(xBtnSemaphore, 0);
}

void tone(int frequency, int duration) {
	// tone
	if (frequency != 0){
		int tempo = 0;
		int duracao = (1000000 / (2*frequency));
		while (tempo < duration * 0.9 * 1000){
			pio_set(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
			delay_us(duracao);
			pio_clear(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
			delay_us(duracao);
			tempo += duracao;
		}
	}
	// delay
// 	int tempo = 0;
// 	while (tempo < duration * 0.1 * 1000){
// 		pio_clear(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
// 		delay_ms(1);
// 		tempo += 1000;
// 	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_coins(void *pvParameters)
{	
	for (;;)  {
		if (xSemaphoreTake(xBtnSemaphore, 1)) {
			if (flag_rtt == 0) {
				// COMECANDO SRAND	
				uint32_t tempo = rtt_read_timer_value(RTT);
				srand(tempo);
				printf("Seed: %d\n", tempo);
				flag_rtt = 1;
			}
			int coins = rand() % 3 + 1;
			printf("Coins: %d\n", coins);
			xQueueSend(xQueueCoins, (void *)&coins, 1);
		}
		vTaskDelay(1);
	}
}

static void task_play(void *pvParameters)
{
	for (;;)  {
		int coins;
		if (xQueueReceive(xQueueCoins, &coins, 1)) {
			for (int i = 0; i < coins; i++){
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);
			}
		}
		vTaskDelay(1);
	}
}

static void task_debug(void *pvParameters) {
	gfx_mono_ssd1306_init();

	for (;;) {
		gfx_mono_draw_filled_circle(10,10,4,1,GFX_WHOLE);
		vTaskDelay(150);
		gfx_mono_draw_filled_circle(10,10,4,0,GFX_WHOLE);
		vTaskDelay(150);

	}
}



/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void btn_init(void) {
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BTN_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BTN_PIO, PIO_INPUT, BTN_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN_PIO, BTN_PIO_PIN_MASK, 60);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BTN_PIO,
	BTN_PIO_ID,
	BTN_PIO_PIN_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BTN_PIO, BTN_PIO_PIN_MASK);
	pio_get_interrupt_status(BTN_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BTN_PIO_ID);
	NVIC_SetPriority(BTN_PIO_ID, 4); // Prioridade 4
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	pmc_enable_periph_clk(BUZZER_PIO_ID);
	pio_configure(BUZZER_PIO, PIO_OUTPUT_0, BUZZER_PIO_IDX_MASK, PIO_DEFAULT);
}


void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	
	// CRIANDO SEMAPHORE
	xBtnSemaphore = xSemaphoreCreateBinary();
	if (xBtnSemaphore == NULL)
	printf("falha em criar o semaforo btn \n");
	
	// CRIANDO QUEUE
	xQueueCoins = xQueueCreate(100, sizeof(int));
	if (xQueueCoins == NULL)
	printf("falha em criar a queue xQueueProc \n");
	
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	btn_init();

	/* Initialize the console uart */
	configure_console();
	
	if (xTaskCreate(task_debug, "debug", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create debug task\r\n");
	}
	
	// CRIANDO TASKS
	if (xTaskCreate(task_coins, "coins", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create coins task\r\n");
	}

	if (xTaskCreate(task_play, "play", TASK_PLAY_STACK_SIZE, NULL, TASK_PLAY_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create play task\r\n");
	}
	
	// INICIALIZANDO O RTT
	RTT_init(1000, 0, 0);
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
