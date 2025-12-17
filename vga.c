#include <unistd.h>
#include <stdio.h>

// if using CPUlator, you should copy+paste contents of the file below instead of using #include
/*******************************************************************************
 * This file provides address values that exist in the DE10-Lite Computer
 * This file also works for DE1-SoC, except change #define DE10LITE to 0
 ******************************************************************************/

#ifndef __SYSTEM_INFO__
#define __SYSTEM_INFO__

#define DE10LITE 1 // change to 0 for CPUlator or DE1-SoC, 1 for DE10-Lite

/* do not change anything after this line */

#if DE10LITE
 #define BOARD				"DE10-Lite"
 #define MAX_X		160
 #define MAX_Y		120
 #define YSHIFT		  8
#else
 #define MAX_X		320
 #define MAX_Y		240
 #define YSHIFT		  9
#endif

/* Memory */
#define SDRAM_BASE				0x00000000
#define SDRAM_END				0x03FFFFFF
#define FPGA_PIXEL_BUF_BASE		0x08000000
#define FPGA_PIXEL_BUF_END		0x0800FFFF
#define FPGA_CHAR_BASE			0x09000000
#define FPGA_CHAR_END			0x09001FFF

/* Devices */
#define LED_BASE				0xFF200000
#define LEDR_BASE				0xFF200000
#define HEX3_HEX0_BASE			0xFF200020
#define HEX5_HEX4_BASE			0xFF200030
#define SW_BASE					0xFF200040
#define KEY_BASE				0xFF200050
#define JP1_BASE				0xFF200060
#define ARDUINO_GPIO			0xFF200100
#define ARDUINO_RESET_N			0xFF200110
#define JTAG_UART_BASE			0xFF201000
#define TIMER_BASE				0xFF202000
#define TIMER_2_BASE			0xFF202020
#define MTIMER_BASE				0xFF202100
#define RGB_RESAMPLER_BASE    	0xFF203010
#define PIXEL_BUF_CTRL_BASE		0xFF203020
#define CHAR_BUF_CTRL_BASE		0xFF203030
#define ADC_BASE				0xFF204000
#define ACCELEROMETER_BASE		0xFF204020

/* Nios V memory-mapped registers */
#define MTIME_BASE             	0xFF202100

#endif

typedef uint16_t pixel_t;

volatile pixel_t *pVGA = (pixel_t *)FPGA_PIXEL_BUF_BASE;

const pixel_t blk = 0x0000;
const pixel_t wht = 0xffff;
const pixel_t red = 0xf800;
const pixel_t grn = 0x07e0;
const pixel_t blu = 0x001f;

void delay( int N )
{
	for( int i=0; i<N; i++ ) 
		*pVGA; // read volatile memory location to waste time
}

/* STARTER CODE BELOW. FEEL FREE TO DELETE IT AND START OVER */

int win;
int gameover;
int yPlayer, xPlayer;
int yRobot, xRobot;
int dirPlayer;
int dirRobot;
int scorePlayer = 0, scoreRobot = 0;

volatile int pending_turn_left = 0;
volatile int pending_turn_right = 0;

//const uint64_t PERIOD = (uint64_t)100000000; 
const uint64_t CLOCK_RATE  = (uint64_t)100000000;
	
volatile int *LEDR_ptr = (uint32_t *) LEDR_BASE;
volatile int *HEX30 = (uint32_t*)HEX3_HEX0_BASE;
volatile int *HEX54 = (uint32_t*)HEX5_HEX4_BASE;

volatile int *KEY_ptr = (int*)KEY_BASE;
//volatile uint32_t *KEY_data = (uint32_t *)(KEY_BASE + 0);
volatile int *KEY_edge = (uint32_t*)(KEY_BASE + 0xC);
volatile int *KEY_mask = (uint32_t*)(KEY_BASE + 0x8);

const int sevenseg[10] =
{
	0x3F, // 0
	0x06, // 1
	0x5B, // 2
	0x4F, // 3
	0x66, // 4
	0x6D, // 5
	0x7D, // 6
	0x07, // 7
	0x7F, // 8
	0x6F  // 9
};

static inline void set_led_pending(int left, int right)
{
	uint32_t v = 0;
	if (right) v |= (1<<0);
	if (left) v |= (1<<1);
	*LEDR_ptr = v;
}
	
void drawPixel( int y, int x, pixel_t colour )
{
	*(pVGA + (y<<YSHIFT) + x ) = colour;
}

pixel_t makePixel( uint8_t r8, uint8_t g8, uint8_t b8 )
{
	// inputs: 8b of each: red, green, blue
	const uint16_t r5 = (r8 & 0xf8)>>3; // keep 5b red
	const uint16_t g6 = (g8 & 0xfc)>>2; // keep 6b green
	const uint16_t b5 = (b8 & 0xf8)>>3; // keep 5b blue
	return (pixel_t)( (r5<<11) | (g6<<5) | b5 );
}

void rect( int y1, int y2, int x1, int x2, pixel_t c )
{
	for( int y=y1; y<y2; y++ )
		for( int x=x1; x<x2; x++ )
			drawPixel( y, x, c );
}

void displayPlayerScore(int scorePlayer)
{
	*HEX30 = sevenseg[scorePlayer];
}

void displayRobotScore(int scoreRobot)
{
	*HEX54 = sevenseg[scoreRobot];
}

void start()
{
	gameover = 0;
	
	xPlayer = MAX_X/3;
	yPlayer = MAX_Y/2;
	
	yRobot = MAX_Y/2;
	xRobot = MAX_X/2 + 50;
	
	dirPlayer = 1;
	dirRobot = 1;
	
	drawPixel(yPlayer,xPlayer,blk);
	drawPixel(yRobot,xRobot,blk);
	
	
	displayPlayerScore(scorePlayer);
	displayRobotScore(scoreRobot);
	
	// obstacle
	rect( MAX_Y/2 - 10, MAX_Y/2 + 10, MAX_X/2 - 10, MAX_X/2 + 10, wht);
}

void moveLogicPlayer() 
{
	switch (dirPlayer) 
	{
		case 1: yPlayer--; break;
		case 2: yPlayer++; break;
		case 3: xPlayer--; break;
		case 4: xPlayer++; break;
		default: break;
	}
}

void moveLogicRobot()
{
	switch (dirRobot) 
	{
		case 1: yRobot--; break;
		case 2: yRobot++; break;
		case 3: xRobot--; break;
		case 4: xRobot++; break;
		default: break;
	}
}

// Writing mtime
void set_mtimer(volatile uint32_t *time_ptr, uint64_t new_time64)
{
	*(time_ptr+0) = (uint32_t)0;
	// prevent hi from increasing to lo
	*(time_ptr+1) = (uint32_t)(new_time64>>32);
	// set hi part
	*(time_ptr+0) = (uint32_t)new_time64;
	// set lo part
}

// Reading mtime
uint64_t get_mtimer(volatile uint32_t *time_ptr)
{
	uint32_t mtime_h, mtime_l;
	// can only read 32b at a time
	// hi part may increment between reading hi and lo
	// if it increments, re-read lo and hi again
	do
	{
		mtime_h = *(time_ptr+1); // read mtime-hi
		mtime_l = *(time_ptr+0); // read mtime-lo
	}
	while(mtime_h != *(time_ptr+1)); 
	// if mtime-hi has changed, repeat
	
	//return 64b result
	return ((uint64_t)mtime_h << 32) | mtime_l;
}

// Set Up mtimecmp
volatile uint32_t *mtime_ptr = (uint32_t *) MTIMER_BASE;
const uint64_t PERIOD = (uint64_t)CLOCK_RATE; // 100M

void setup_mtimecmp()
{
	uint64_t mtime64 = get_mtimer(mtime_ptr);
	// read current mtime (counter)
	mtime64 = (mtime64/PERIOD+1) * PERIOD;
	// compute end of next time PERIOD
	set_mtimer(mtime_ptr+2, mtime64);
	// write first mtimecmp ("+2" == mtimecmp)
}

// mtime ISR
void mtimer_ISR(void)
{
	if (scorePlayer == 9)
	{
		rect( 0, MAX_Y, 0, MAX_X, red);
		return 0;
	}
	else if (scoreRobot == 9)
	{
		rect( 0, MAX_Y, 0, MAX_X, grn);
		return 0;
	}
	
	
	uint64_t mtimecmp64 = get_mtimer(mtime_ptr+2);
	// read mtimecmp
	//mtimecmp64 += PERIOD;
	// time of future irq = one period in future
	
	
	volatile uint32_t *SW_ptr = (uint32_t*)SW_BASE;
	uint32_t sw = *SW_ptr & 0x3FF;
	
	uint32_t divisor = (sw == 0 ? 1 : sw);
	mtimecmp64 += (CLOCK_RATE / divisor);
	
	set_mtimer(mtime_ptr+2, mtimecmp64);
	// write next mtimecmp
		
	// intended side effect
	if (pending_turn_left)
	{
		if (dirPlayer == 1) dirPlayer = 3;
		else if (dirPlayer == 2) dirPlayer = 4;
		else if (dirPlayer == 3) dirPlayer = 2;
		else if (dirPlayer == 4) dirPlayer = 1;
	}
	else if (pending_turn_right)
	{
		if (dirPlayer == 1) dirPlayer = 4;
		else if (dirPlayer == 2) dirPlayer = 3;
		else if (dirPlayer == 3) dirPlayer = 1;
		else if (dirPlayer == 4) dirPlayer = 2;
	}
	
	pending_turn_left = 0;
	pending_turn_right = 0;
	set_led_pending(0,0);
	
	moveLogicPlayer();
	handleRobotAI();
	moveLogicRobot();
	
	pixel_t pixelPlayer = *(pVGA + ((yPlayer<<YSHIFT) + xPlayer));
	pixel_t pixelRobot = *(pVGA + ((yRobot<<YSHIFT) + xRobot));
	
	// tie
	if ((pixelRobot != blk) && (pixelPlayer != blk))
	{	
		rect( 0, MAX_Y, 0, MAX_X, blk);
		rect( 0, MAX_Y, 0, 1, wht); // left vertical border
		rect( 0, MAX_Y, MAX_X-1, MAX_X, wht); // right vertical border
		rect( MAX_Y-1, MAX_Y, 0, MAX_X, wht); // bottom horizontal border
		rect( 0, 1, 0, MAX_X, wht); // top horizontal border
		start();
		return;
	}
	
	// gameover, player loses
	if (pixelPlayer != blk)
	{
		scoreRobot++;
		displayPlayerScore(scorePlayer);
		displayRobotScore(scoreRobot);
		rect( 0, MAX_Y, 0, MAX_X, blk);
		rect( 0, MAX_Y, 0, 1, wht); // left vertical border
		rect( 0, MAX_Y, MAX_X-1, MAX_X, wht); // right vertical border
		rect( MAX_Y-1, MAX_Y, 0, MAX_X, wht); // bottom horizontal border
		rect( 0, 1, 0, MAX_X, wht); // top horizontal border
		start();
		return;
	}
	
	// win, player wins
	if (pixelRobot != blk)
	{
		scorePlayer++;
		displayPlayerScore(scorePlayer);
		displayRobotScore(scoreRobot);
		rect( 0, MAX_Y, 0, MAX_X, blk);
		rect( 0, MAX_Y, 0, 1, wht); // left vertical border
		rect( 0, MAX_Y, MAX_X-1, MAX_X, wht); // right vertical border
		rect( MAX_Y-1, MAX_Y, 0, MAX_X, wht); // bottom horizontal border
		rect( 0, 1, 0, MAX_X, wht); // top horizontal border
		start();
		return;
	}
		
	drawPixel(yRobot,xRobot,grn);
	drawPixel(yPlayer,xPlayer,red);
}

// key ISR
void key_ISR(void)
{
	int edge = *KEY_edge;
	*KEY_edge = 0xF;
		
	//uint32_t keys = *KEY_ptr;
	
	if (edge & 0x1) // KEY 0
	{
		//if (pending_turn_right) pending_turn_right = 0;
		//else pending_turn_right = 1;
		pending_turn_right = 1;
		*LEDR_ptr |= 0x1;
		*LEDR_ptr &= ~0x2;
	}
	if (edge & 0x2) // KEY 1
	{
		//if (pending_turn_left) pending_turn_left = 0;
		//else pending_turn_left = 1;
		pending_turn_left = 1;
		*LEDR_ptr |= 0x2;
		*LEDR_ptr &= ~0x1;
	}
	if (!(edge & 0x8))
	{
		return;
	}
	
	set_led_pending(pending_turn_left, pending_turn_right);
	//*KEY_edge = edge & KEY_IRQ_MASK;
}

// Exception Handler
// this attribute tells compiler to use "mret" rather than "ret"
// at the end of handler() function; also declares its type
static void handler(void) __attribute__ ((interrupt ("machine")));

void handler(void)
{
	int mcause_value;
	
	// inline assembly, links register %0 to mcause_value
	__asm__ volatile( "csrr %0, mcause" : "=r"(mcause_value));
	
	
	if (mcause_value == 0x80000007) // machine timer
	{
		mtimer_ISR();
	}
	else if (mcause_value == 0x80000011 || mcause_value == 0x80000012 || mcause_value == 0x80000013)
	{
		key_ISR();
	}
	else
	{
		// nothing
	}
}

// Set Up CPU IRQs
void setup_cpu_irqs(uint32_t new_mie_value)
{
	uint32_t mstatus_value, mtvec_value, old_mie_value;
	
	mstatus_value = 0b1000;				 // interrupt bit mask
	mtvec_value   = (uint32_t) &handler; // set trap address
		
	__asm__ volatile("csrc mstatus, %0" :: "r"(mstatus_value));
	// master irq disable
	__asm__ volatile("csrw mtvec, %0" :: "r"(mtvec_value));
	// sets handler
	__asm__ volatile("csrr %0, mie" : "=r"(old_mie_value));
	__asm__ volatile("csrc mie, %0" :: "r"(old_mie_value));
	__asm__ volatile("csrs mie, %0" :: "r"(new_mie_value));
	// reads old irq mask, removes old irqs, sets new irq mask
	__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));
	// master irq enable
}

void handleRobotAI()
{
	int dy = 0, dx = 0;
	int left_dy = 0, left_dx = 0;
	int right_dy = 0, right_dx = 0;
	
	switch (dirRobot)
	{
		case 1: dy = -1; break;
		case 2: dy =  1; break;
		case 3: dx = -1; break;
		case 4: dx =  1; break;
	}
	
	int x1 = xRobot + dx;
	int y1 = yRobot + dy;
	
	int x2 = xRobot + 2*dx;
	int y2 = yRobot + 2*dy;
	
	pixel_t ahead1 = *(pVGA + ((y1<<YSHIFT) + x1));
	pixel_t ahead2 = *(pVGA + ((y2<<YSHIFT) + x2));
	
	if ((ahead1 != blk) || (ahead2 != blk))
	{
		if (dirRobot == 1) {left_dy =  0; left_dx = -1;}
		if (dirRobot == 2) {left_dy =  0; left_dx =  1;}
		if (dirRobot == 3) {left_dy =  1; left_dx =  0;}
		if (dirRobot == 4) {left_dy = -1; left_dx =  0;}
		
		pixel_t leftPixel = *(pVGA + (((yRobot + left_dy)<<YSHIFT) + (xRobot + left_dx)));
		
		if (leftPixel == blk)
		{
			if (dirRobot == 1) dirRobot = 3;
			else if (dirRobot == 2) dirRobot = 4;
			else if (dirRobot == 3) dirRobot = 2;
			else if (dirRobot == 4) dirRobot = 1;
			return;
		}
		else
		{
			if (dirRobot == 1) {right_dy =  0; right_dx =  1;}
			if (dirRobot == 2) {right_dy =  0; right_dx = -1;}
			if (dirRobot == 3) {right_dy = -1; right_dx =  0;}
			if (dirRobot == 4) {right_dy =  1; right_dx =  0;}
			
			pixel_t rightPixel = *(pVGA + (((yRobot + right_dy)<<YSHIFT) + (xRobot + right_dx)));
			
			if (rightPixel == blk)
			{
				if (dirRobot == 1) dirRobot = 4;
				else if (dirRobot == 2) dirRobot = 3;
				else if (dirRobot == 3) dirRobot = 1;
				else if (dirRobot == 4) dirRobot = 2;
				return;
			}
		}
	}
	else
	{
		return;
	}
}

int main()
{
	// initialisations
	rect( 0, MAX_Y, 0, MAX_X, blk); // black out the screen
	rect( 0, MAX_Y, 0, 1, wht); // left vertical border
	rect( 0, MAX_Y, MAX_X-1, MAX_X, wht); // right vertical border
	rect( MAX_Y-1, MAX_Y, 0, MAX_X, wht); // bottom horizontal border
	rect( 0, 1, 0, MAX_X, wht); // top horizontal border
	
	// obstacle
	rect( MAX_Y/2 - 10, MAX_Y/2 + 10, MAX_X/2 - 10, MAX_X/2 + 10, wht);
	
	start();
	
	setup_mtimecmp();
	//setup_cpu_irqs(0x80); // enable mtimer IRQs
	
	pending_turn_left = 0;
	pending_turn_right = 0;
	*KEY_mask = 0x3;
	*KEY_edge = 0xF;
	
	setup_cpu_irqs(0xE0080); // enable mtimer IRQs
	
	while (1)
	{
		//*LEDR_ptr = counter;
		__asm__ volatile("wfi"); // wait for interrupt
	}
}


