#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "hw_qei.h"
#include "qei.h"
#include "tm4c1294ncpdt.h"
#define SYSTICK_FREQUENCY		1000			//1000hz

#define	I2C_FLASHTIME				500				//500mS
#define GPIO_FLASHTIME			300				//300mS
#define SHINENUM            4
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06

volatile      uint16_t systick_10ms_couter,systick_100ms_couter,systick_10us_couter,systick_2ms_couter,systick_1ms_couter,systick_1s_couter,systick_10ps_couter;
volatile      uint8_t	systick_10ms_status,systick_100ms_status,systick_10us_status,systick_2ms_status,systick_1ms_status,systick_1s_status,systick_10ps_status;
uint32_t      ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t      ui32IntPrioritySystick,ui32IntPriorityUart0;
volatile      uint8_t result,cnt,key_value,gpio_status;
uint32_t      ui32SysClock;

void          Start(void);
void          Reset(void);
void          Show_In_Seg(uint32_t a1,uint32_t a2,uint32_t a3,uint32_t a4,uint32_t a5,uint32_t a6,uint32_t a7,uint32_t a8);
void          Translate(char *t);
void          Date();
void          Solarterms(void);
void          Timecount(void);
void          Timeset(void);
void          Timeshow(void);
void          Beep(void);
void          Countdown(void);
void 		      Delay(uint32_t value);
void 		      S800_GPIO_Init(void);
void          S800_QEI_Init(void);
uint8_t 	    I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	    I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		      S800_I2C0_Init(void);
void 		      S800_UART_Init(void);

uint32_t      year=2001,month=5,day=8; // initial my birthday
uint32_t      hour=0,minute=0,second=0; //clock
uint32_t      beep1_hour=0,beep1_minute=0,beep1_second=0; //beep1
uint32_t      beep2_hour=0,beep2_minute=0,beep2_second=0; //beep2
uint32_t      qei_value;
uint8_t       beep1 = 0x0,beep2 = 0x0,beepsign1 = 0x0,beepsign2 = 0x0; //beep1 2 unenable
uint32_t      bit1,bit2,bit3,bit4,bit5,bit6,bit7,bit8;
uint32_t      tbit1,tbit2,tbit3,tbit4,tbit5,tbit6,tbit7,tbit8;
uint32_t      dbit1,dbit2,dbit3,dbit4,dbit5,dbit6,dbit7,dbit8;
uint8_t       startbit=0x0,beginport=0x01;
uint32_t      startcounter=0x0,start_cnt=0x0;
uint16_t      translatesign;
uint16_t      startcountdown = 0,endcountdown = 0;
uint8_t       solartermsign;
uint8_t       lightlevel = 0xff,lightreverse = 0;
int endflag=0;	
uint8_t       seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
char	const   disp_tab[]			={'0','1','2','3','4','5',     //the could display char and its segment code   
															 '6','7','8','9','A','b','B',
															 'C','d','D','E','F','G',
															 'H','J','K','L','M','N',
															 'P','o','O','Q','R','T',
																'S','U','V','W','X','Y','Z',
															 '.','-','_',' '}; 
char const   disp_tab_7seg[]	={0x3F,0x06,0x5B,0x4F,0x66,0x6D,  
															0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x7c,
															0x39,0x5E,0x5e,0x79,0x71, 0x3d,
															0x76,0x0d,0x72,0x38,0x55,0x54,
															0x73,0x5c,0x5c,0x6f,0x50,0x78,
															0x6d,0x3e,0x1c,0x6a,0x1d,0x6e,0x49,
															0x80,0x40, 0x08,0x00}; 
uint8_t      uart_receive_char,uart_cnt;
char         uartdata[1000];															
uint8_t      key_pressed=0Xff;														
volatile     uint16_t	i2c_flash_cnt,gpio_flash_cnt;
uint8_t      flash_status=0x00;
uint8_t      timeset=0x0,timeselect;		
uint8_t      clockset1=0x0,clockselect1;			
uint8_t      clockset2=0x0,clockselect2;	
uint16_t     count;		
uint16_t     dark;
uint16_t     pw1,pw2,pw3,pw4,pw5,pw6,pw7,pw8;			
uint16_t     ipw1,ipw2,ipw3,ipw4,ipw5,ipw6,ipw7,ipw8;		
uint16_t     kpw1,kpw2,kpw3,kpw4,kpw5,kpw6,kpw7,kpw8;		
														
int main(void)
{
  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);
	SysTickEnable();
	SysTickIntEnable();
  
	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_QEI_Init();
	
	UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
	IntEnable(INT_UART0);
	IntPriorityGroupingSet(0x0); // set group ,not important
	IntPrioritySet(INT_UART0,0x00); // UART interrupt is the least 
	IntPrioritySet(FAULT_SYSTICK,0x20); // GPIO PJ0/PJ1 interrupt is the second least
	//IntPrioritySet(INT_TIMER0A,0x40); // INT_TIMEROA is the second highest
	//IntPrioritySet(INT_ADC0SS3,0x60);// INT_ADC is the highest
	Start();
	IntMasterEnable();		
	while(1){
		Reset();
		//*****************************************************
		//*********receive uart,uart is the most important*****
		//*****************************************************
			if (endflag){}
			/*
			if (UARTCharsAvail(UART0_BASE)){
				
				uart_receive_char = (uint8_t)UARTCharGet(UART0_BASE);
				UARTCharPutNonBlocking(UART0_BASE,uart_receive_char);
				uartdata[uart_cnt] = uart_receive_char;
				uart_cnt++;
				
					Translate(uartdata);
					if(uart_receive_char=='R') {
					uart_cnt=0;
				}
			}
			*/
				
		//timeset=0,no actions
			if(timeset==0x0)
				Timecount();
			
		//**********************************************
		//*********sw7**********************************
		//*******start beeps****************************
			if(key_pressed==0xbf){
				Delay(2000000);
				if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xbf){
					dark = 1;
					if (beep1==0xff)
						beep1_second=20;
					if (beep2==0xff)
						beep2_second=10;
					while(1){
					Timecount();
					result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0);
					result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
					if(hour==beep1_hour&&minute==beep1_minute&&second==beep1_second&&beep1==0xff){
						beepsign1 = 0x1;
						break;
					}
					if(hour==beep2_hour&&minute==beep2_minute&&second==beep2_second&&beep2==0xff){
						beepsign2 = 0x1;
						break;
					}
					if((key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0))!=0xff){
						dark = 0;
						break;
					}
					continue;
					}
				}
			}
		
		
		//timeset==1,qei matters
			if (timeset == 1)
			{
				Timeset();
				if((key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0))==0xfd){
					result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0);
					Delay(2000000);
					if((key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0))==0xfd){
						timeselect = (timeselect)%3+1;
					}
				}
	
			
		
			// sw4 quit timeset
			if((key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0))==0xfe){
				result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0);
				Delay(2000000);
				if((key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0))==0xfe){
					timeset = 0;
					timeselect = 0;
				}
			}
			continue;
		}
	 
		
		else{
			if (endflag){}
			key_pressed = I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
			
			if(systick_1ms_status==1){
				systick_1ms_status = 0;
				Beep();
			}
		//**********************************
		//*********sw5, enter coundown mode*
		//**********************************
			if(key_pressed==0xef){
			Delay(50);
				if(key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xef){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_PIN_0);
					Countdown();
				}
			}
			
			if(systick_2ms_status==1){
				systick_2ms_status = 0;
				i2c_flash_cnt = (i2c_flash_cnt+1)%100000;
				
				//**************************
				//*****sw1,date display*****
				//**************************
				if(key_pressed==0xfe){
					Delay(10);
					if(key_pressed==0xfe)
						Date();
				}
				//**************************
				//*****sw2,time display*****
				//**************************
				else if(key_pressed==0xfd){
					Delay(10);
					if(key_pressed==0xfd)
						Timeshow();
				}
				//**************************
				//*****sw3,sw4,beep set*****
				//**************************
				else if(key_pressed==0xfb){
					Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
					Delay(100);
					while(key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfb){
						Timecount();
						if(systick_2ms_status){
							Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
						}
					}
						beep1 = ~beep1;
					if ((beep1 == 0xff)&&(beep2 == 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfc);
					if ((beep1 == 0xff)&&(beep2 != 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfe);
					if ((beep1 != 0xff)&&(beep2 == 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfd);
					if ((beep1 != 0xff)&&(beep2 != 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
				}
				else if(key_pressed==0xf7){
					Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
					Delay(100);
					while(key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xf7){
						Timecount();
						if(systick_2ms_status){
							Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
						}
					}
					beep2 = ~beep2;
					if ((beep1 == 0xff)&&(beep2 == 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfc);
					if ((beep1 == 0xff)&&(beep2 != 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfe);
					if ((beep1 != 0xff)&&(beep2 == 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xfd);
					if ((beep1 != 0xff)&&(beep2 != 0xff))
						result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);	
				}
				//****************************
				//********sw6,solar term******
				//****************************
				else if(key_pressed==0xdf){
					Delay(50);
					if(key_pressed==0xdf){
						Solarterms();
					}
				}
				
				//*****************************
				//*********sw8,lightlevel******
				//*****************************
				else if(key_pressed==0x7f){
					Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
					Delay(100);
					while(key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0x7f){
						Timecount();
						if(systick_2ms_status){
							Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
						}
					}
					lightlevel = ~lightlevel;
				}
				else{
					Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);	
				}
			}
		}
	}
}
if (RxEndFlag){}



void UART0_Handler(void)
{
	uint8_t cnt=0;
	uint32_t ulStatus;
	ulStatus= UARTIntStatus(UART0_BASE,true);
	UARTIntClear(UART0_BASE,ulStatus);
    
    
    
    
    
    
    second = second +5;
    if (second >=60)
        second =second%60;
    minute++;
    minute=minute%60;
    
    minute =
    
    
    if((RxBuff[0]=='q')&&(RxBuff[1]=='5'))
    {
        second = second+5;
        minute+= minute/60;
        minute=minute%60;
        second=second%60;
        
        
    }
    
    
    
    
	
	while (UARTCharsAvail(UART0_BASE))
	{
			uart_receive_char = (uint8_t)UARTCharGet(UART0_BASE);
				UARTCharPutNonBlocking(UART0_BASE,uart_receive_char);
				uartdata[uart_cnt] = uart_receive_char;
				uart_cnt++;
        
        // if (uartdata[0]=='q'
		Translate(uartdata);
		if(uart_receive_char=='R') {
					uart_cnt=0;
				}
	}
    
    if (RxBuff[0]=='q')&&(R)
    {
        djhlsgjsdgkjd
    }
    if ()
    {
    }
    
    }
	&&RxBuff[2]=='\0'
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	 for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
}
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));	
	GPIO_PORTF_AHB_DIR_R = 0xf;
	GPIO_PORTF_AHB_DEN_R = 0xf;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK );						
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK ));	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE , GPIO_PIN_5);	
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}
void S800_QEI_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	// Enable the QEI0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	// Wait for the QEI0 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
	{}

	GPIOPinConfigure(GPIO_PL1_PHA0);
  GPIOPinConfigure(GPIO_PL2_PHB0);
	//software patch to force the PL3 to low voltage
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);			
	GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3,0);	
  
	GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1);
  GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_2);
	//
	// Configure the quadrature encoder to capture edges on both signals and
	// maintain an absolute position by resetting on index pulses. Using a
	// 1000 line encoder at four edges per line, there are 4000 pulses per
	// revolution; therefore set the maximum position to 3999 as the count
	// is zero based.
	//
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |	QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 100);
	// Enable the quadrature encoder.
	QEIEnable(QEI0_BASE);
}
void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value;

	while(I2CMasterBusy(I2C0_BASE)){};	//???
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false); //?????,?
	I2CMasterDataPut(I2C0_BASE, RegAddr); //?????
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND); //??????
	while(I2CMasterBusBusy(I2C0_BASE));
	if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
		return 0; //??
	Delay(100);

	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true); //?????,?
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE); //??????
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
		return 0; //??
	Delay(100);

	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	if (systick_100ms_couter	!= 0)
		systick_100ms_couter--;
	else
	{
		systick_100ms_couter	= SYSTICK_FREQUENCY/10;
		systick_100ms_status 	= 1;
	}
	if (systick_1s_couter	!= 0)
		systick_1s_couter--;
	else
	{
		systick_1s_couter	= SYSTICK_FREQUENCY;
		systick_1s_status 	= 1;
	}
	
	if (systick_10ms_couter	!= 0)
		systick_10ms_couter--;
	else
	{
		systick_10ms_couter		= SYSTICK_FREQUENCY/100;
		systick_10ms_status 	= 1;
	}
	if (systick_10us_couter	!= 0)
		systick_10us_couter--;
	else
	{
		systick_10us_couter		= SYSTICK_FREQUENCY/100000;
		systick_10us_status 	= 1;
	}
	if (systick_10ps_couter	!= 0)
		systick_10ps_couter--;
	else
	{
		systick_10ps_couter		= SYSTICK_FREQUENCY/100000000000;
		systick_10ps_status 	= 1;
	}
	if (systick_2ms_couter	!= 0)
		systick_2ms_couter--;
	else
	{
		systick_2ms_couter		= SYSTICK_FREQUENCY/500;
		systick_2ms_status 	= 1;
	}
	if (systick_1ms_couter	!= 0)
		systick_1ms_couter--;
	else
	{
		systick_1ms_couter		= SYSTICK_FREQUENCY/1000;
		systick_1ms_status 	= 1;
	}
	if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0)
	{
		systick_100ms_status	= systick_10ms_status = 0;
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,GPIO_PIN_0);		
	}
	else
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);		
}

void Start(void){
	while(startcounter<SHINENUM){
		// startcounter use to shine all segments once when started
		if (startcounter<2){
			while (1){
				if(systick_100ms_status){
						systick_100ms_status	= 0;
						if (++start_cnt	>= 10){
								start_cnt = 0;
								startbit = ~startbit;
								startcounter++;
								second = (second+1)%60;
								if(second==0)
									minute = (minute+1)%60;
								if(minute==0&&second==0)
									hour = (hour+1)%24;
								if(hour==0&&second==0&&minute==0)
									if(month==2)
										day = (day%28)+1;
									else if(month==1||month==3||month==5||month==7||month==8||month==10||month==12)
										day = (day%31)+1;
									else
										day = (day%30)+1;
								if(day==1&&hour==0&&second==0&&minute==0)
									month = (month)%12 +1;
								if(month==1&&day==1&&hour==0&&second==0&&minute==0)
									year++;
								if(startcounter==2) break;
							}
					}
					if(startbit==0x0){
					result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0xff);			
					result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,startbit);}	
					else
						Show_In_Seg(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
					result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~startbit);
				}
				result 	= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
			}
		// show my ID 
		else if(startcounter<4&&startcounter>=2){
				while (1){
					if(systick_100ms_status){
						systick_100ms_status	= 0;
						if (++start_cnt	>= 10){
							start_cnt = 0;
							startbit = ~startbit;
							startcounter++;
								if(second==0)
									minute = (minute+1)%60;
								if(minute==0&&second==0)
									hour = (hour+1)%24;
								if(hour==0&&second==0&&minute==0)
									if(month==2)
										day = (day%28)+1;
									else if(month==1||month==3||month==5||month==7||month==8||month==10||month==12)
										day = (day%31)+1;
									else
										day = (day%30)+1;
								if(day==1&&hour==0&&second==0&&minute==0)
									month = (month)%12 +1;
								if(month==1&&day==1&&hour==0&&second==0&&minute==0)
									year++;
							if(startcounter==4) break;
							}
						}
						if(startbit==0x0)
							result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0);
						else
							Show_In_Seg(seg7[2],seg7[1],seg7[9],seg7[1],seg7[1],seg7[2],seg7[4],seg7[8]);
					}
					result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0);
			}
	}
}

void Reset(void){
	// sw3 and sw4
			if(key_pressed==0xf3){
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0);
			Delay(200000);
			if((key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0))==0xf3){
				Start();
				beep1_hour = 0;
				beep1_minute = 0;
				beep1_second = 0;
				beep2_hour = 0;
				beep2_minute = 0;
				beep2_second = 0;
				startbit = 0;
				beginport = 0x1;
				startcounter = 0;
				start_cnt = 0;
				translatesign = 0;
				startcountdown = 0;
				endcountdown = 0;
				solartermsign = 0;
				key_pressed = 0xff;
				gpio_flash_cnt = 0;
				timeset = 0;
				timeselect = 0;
				count = 0;
				lightlevel = 0xff;
				lightreverse = 0;
				dbit1 = 0;
				dbit2 = 0;
				dbit3 = 0;
				dbit4 = 0;
				dbit5 = 0;
				dbit6 = 0;
				dbit7 = 0;
				dbit8 = 0;
			}
		}
}

void Date(){
	bit1 = year/1000;
	bit2 = (year%1000)/100;
	bit3 = (year/10)%10;
	bit4 = year%10;
	bit5 = month/10;
	bit6 = month%10;
	bit7 = day/10;
	bit8 = day%10;
	Show_In_Seg(seg7[bit1],seg7[bit2],seg7[bit3],seg7[bit4],seg7[bit5],seg7[bit6],seg7[bit7],seg7[bit8]);
}

void Timeshow(void){
	tbit1 = hour/10;
	tbit2 = hour%10;
	tbit3 = 0x40;
	tbit4 = minute/10;
	tbit5 = minute%10;
	tbit6 = 0x40;
	tbit7 = second/10;
	tbit8 = second%10;
	Show_In_Seg(seg7[tbit1],seg7[tbit2],tbit3,seg7[tbit4],seg7[tbit5],tbit6,seg7[tbit7],seg7[tbit8]);
}

void Translate(char *t){
	if(t[0]=='D'&&t[1]=='A'&&t[2]=='T')
		translatesign = 1;
	else if(t[0]=='T'&&t[1]=='I'&&t[2]=='M')
		translatesign = 2;
	else if(t[0]=='B'&&t[1]=='P'&&t[2]=='1')
		translatesign = 3;
	else if(t[0]=='B'&&t[1]=='P'&&t[2]=='2')
		translatesign = 4;
	else if(t[0]=='O'&&t[1]=='F'&&t[2]=='F')
		translatesign = 5;
	else if(t[0]=='S'&&t[1]=='T'&&t[2]=='A')
		translatesign = 6;
	else if(t[0]=='S'&&t[1]=='E'&&t[2]=='T')
		translatesign = 7;
	switch(translatesign){
		case(1):
			year = (t[3]-'0')*1000+(t[4]-'0')*100+(t[5]-'0')*10+(t[6]-'0');
			month = (t[7]-'0')*10+(t[8]-'0');
			day = (t[9]-'0')*10+(t[10]-'0');
			break;
		case(2):
			hour = (t[3]-'0')*10+(t[4]-'0');
			minute = (t[6]-'0')*10+(t[7]-'0');
			second = (t[9]-'0')*10+(t[10]-'0');
			break;
		case(3):
			beep1_hour = (t[3]-'0')*10+(t[4]-'0');
			beep1_minute = (t[6]-'0')*10+(t[7]-'0');
			beep1_second = (t[9]-'0')*10+(t[10]-'0');
			break;
		case(4):
			beep2_hour = (t[3]-'0')*10+(t[4]-'0');
			beep2_minute = (t[6]-'0')*10+(t[7]-'0');
			beep2_second = (t[9]-'0')*10+(t[10]-'0');
			break;
		case(5):
			dbit1 = t[3] - '0';
			dbit2 = t[4] - '0';
			dbit3 = t[5] - '0';
			dbit4 = t[6] - '0';
			dbit5 = t[7] - '0';
			dbit6 = 0x80;
			dbit7 = t[9] - '0';
			dbit8 = t[10] - '0';
			break;
		case(6):
			startcountdown = 1;
			break;
		case(7):
			timeselect = 1;
		  timeset = 1;
			break;
		default:break;
	}
}


void Show_In_Seg(uint32_t a1,uint32_t a2,uint32_t a3,uint32_t a4,uint32_t a5,uint32_t a6,uint32_t a7,uint32_t a8){
	if(lightlevel==0xff){
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a1);		
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x1);	
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a2);		
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x2);	
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a3);			
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x4);	
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a4);		
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x8);	
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a5);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x10);
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a6);			
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x20);	
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a7);			
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x40);	
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a8);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x80);
		Delay(10000);
		result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	}
		else if(lightlevel==0){
			lightreverse = (lightreverse+1)%3;	
// to set the light level of seg7, only to let the seg7 display some dark peirod in recent time, for example, as below, lightreverse can be the count of display , when enter 
// Show_In_Seg function with a value of lightlevel is 0, lighereverse add	1 each time, from 0 to 3, when lightreverse locate at 0 or 1, close displaying
// when locating at 2 or 3, enable to display
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x1);
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a1);
			Delay(3500);					
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x2);
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a2);
			Delay(3500);			
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x4);	
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a3);
			Delay(3500);					
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x8);	
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a4);
			Delay(3500);		
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x10);
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a5);
			Delay(3500);						
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x20);	
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a6);
			Delay(3500);						
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x40);	
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a7);
			Delay(3500);			
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(lightreverse <= 1&&lightreverse >= 0)?0x0:0x80);
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,a8);
			Delay(3500);	
			result 	= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	}
}


void Timeset(void){
	qei_value = QEIPositionGet(QEI0_BASE);
	if(systick_2ms_status){
		systick_2ms_status = 0;
		if(timeselect==1){
			hour = qei_value*0.24;
			Show_In_Seg(seg7[hour/10],seg7[hour%10],0x80,0x80,0x80,0x80,0x80,0x80);
		}
		else if(timeselect==2){
			minute = qei_value*0.59;
			Show_In_Seg(seg7[minute/10],seg7[minute%10],0x80,0x80,0x80,0x80,0x80,0x80);
		}
		else if(timeselect==3){
			second = qei_value*0.59;
			Show_In_Seg(seg7[second/10],seg7[second%10],0x80,0x80,0x80,0x80,0x80,0x80);
		}
	}		
}


void Beep(void){
	//beep1
	if(beep1==0xff){
		if((hour==beep1_hour&&minute==beep1_minute&&second==beep1_second))
			beepsign1 = 0x1;
		if(((hour-beep1_hour)*60+(minute-beep1_minute))>1)
			beepsign1 = 0x0;
		if(beepsign1==0x1){
			flash_status = ~flash_status;
			if(flash_status==0x00)
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,100);
			else if(beepsign2==0x0)
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,0);
	}
		else
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,0);}
	//beep2
	else if(beep2==0xff){
		if((hour==beep2_hour&&minute==beep2_minute&&second==beep2_second))
			beepsign2 = 0x1;
		if(((hour-beep2_hour)*60+(minute-beep2_minute))>1)
			beepsign2 = 0x0;
		if(beepsign2==0x1){
			flash_status = ~flash_status;
			if(flash_status==0x00)
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,100);
			else if(beepsign1==0x0)
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,0);
	}
			else
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,0);}
	else{
			beepsign2 = 0x0;
			beepsign1 = 0x0;
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5,0);
	}
}

void Timecount(void){
	if(systick_1s_status)
		{
			systick_1s_status	= 0;
			second = (second+1)%60;
			if(second==0)
				minute= (minute+1)%60;
			if(minute==0&&second==0)
				hour= (hour+1)%24;
			if(hour==0&&second==0&&minute==0)
				if(month==2)
					day= (day%28)+1;
				else if(month==1||month==3||month==5||month==7||month==8||month==10||month==12)
					day= (day%31)+1;
				else
					day= (day%30)+1;
			if(day==1&&hour==0&&second==0&&minute==0)
				month= (month)%12 +1;
			if(month==1&&day==1&&hour==0&&second==0&&minute==0)
				year++;			
		}
		// ensure not in dark mode
		int Mozi=0;
		Mozi= 1234*year+10*month+2*day;
		
	if(systick_1ms_status&&lightlevel==0xff&&!dark){
			systick_1ms_status = 0;
			gpio_flash_cnt++;
			if(gpio_flash_cnt>=1&&count==0){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 9;
				// 8-4 dianji rolling
				Delay(5000);
				GPIO_PORTF_AHB_DATA_R = 8;
				count++;
			}		
			else if(gpio_flash_cnt>=1&&count==1){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 12;
				Delay(5000);
				GPIO_PORTF_AHB_DATA_R = 4;
				count++;
			}	
			else if(gpio_flash_cnt>=2&&count==2){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 6;
				Delay(5000);
				GPIO_PORTF_AHB_DATA_R = 2;
				count++;
			}
			else if(gpio_flash_cnt>=1&&count==3){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 3;
				Delay(5000);
				GPIO_PORTF_AHB_DATA_R = 1;
				count=0;
			}	
		}
	else if(systick_1ms_status&&lightlevel==0&&!dark){
			systick_1ms_status = 0;
			gpio_flash_cnt++;
			if(gpio_flash_cnt>=4&&count==0){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 9;
				count++;
			}	
			else if(gpio_flash_cnt>=4&&count==1){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 8;
				count++;
			}	
			else if(gpio_flash_cnt>=4&&count==2){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 12;
				count++;
			}	
			else if(gpio_flash_cnt>=4&&count==3){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 4;
				count++;
			}	
			else if(gpio_flash_cnt>=4&&count==4){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 6;
				count++;
			}
			else if(gpio_flash_cnt>=4&&count==5){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 2;
				count++;
			}
			else if(gpio_flash_cnt>=4&&count==6){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 3;
				count++;
			}
			else if(gpio_flash_cnt>=3&&count==7){
				gpio_flash_cnt = 0;
				GPIO_PORTF_AHB_DATA_R = 1;
				count=0;
			}		
		}
}


void Countdown(void){
	while(1){
		Timecount();
				if (UARTCharsAvail(UART0_BASE)){
				
					uart_receive_char = (uint8_t)UARTCharGet(UART0_BASE);
					UARTCharPutNonBlocking(UART0_BASE,uart_receive_char);
					uartdata[uart_cnt] = uart_receive_char;
					uart_cnt++;
				
					Translate(uartdata);
					if(uart_receive_char=='R') {
					uart_cnt=0;
				}
			}
		if (I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfd)
			startcountdown=1;
		if(systick_10ms_status==1&&startcountdown){
				systick_10ms_status = 0;
				if(dbit8!=0)
					dbit8 = dbit8-1;
				else
					dbit8 = 9;
				if(dbit8==9)
				{
					if(dbit7!=0)
						dbit7 = dbit7-1;
					else
						dbit7 = 5;		
				}
				if(dbit8==9&&dbit7==5)
				{
					if(dbit5!=0)
						dbit5 = dbit5-1;
					else
						dbit5 = 9;		
				}
				if(dbit5==9&&dbit8==9&&dbit7==5)
				{
					if(dbit4!=0)
						dbit4 = dbit4-1;
					else
						dbit4 = 9;		
				}
				if(dbit4==9&&dbit5==9&&dbit8==9&&dbit7==5)
				{
					if(dbit3!=0)
						dbit3 = dbit3-1;
					else
						dbit3 = 9;		
				}
				if(dbit3==9&&dbit4==9&&dbit5==9&&dbit8==9&&dbit7==5)
				{
					if(dbit2!=0)
						dbit2 = dbit2-1;
					else
						dbit2 = 9;		
				}
				if(dbit2==9&&dbit3==9&&dbit4==9&&dbit5==9&&dbit8==9&&dbit7==5)
				{
					if(dbit1!=0)
						dbit1 = dbit7-1;
					else
						dbit1 = 9;		
				}
			}
		if(dbit1==0&&dbit2==0&&dbit3==0&&dbit4==0&&dbit5==0&&dbit7==0&&dbit8==0)
			startcountdown=0;
		key_pressed =I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
		if(key_pressed==0xfe){
			Show_In_Seg(seg7[dbit1],seg7[dbit2],seg7[dbit3],seg7[dbit4],seg7[dbit5],0x80,seg7[dbit7],seg7[dbit8]);
			Delay(100);
			while(key_pressed=I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0)==0xfe){
			dark=1;
			Timecount();
			dark=0;
			if(systick_2ms_status){
				systick_2ms_status = 0;
				Show_In_Seg(seg7[dbit1],seg7[dbit2],seg7[dbit3],seg7[dbit4],seg7[dbit5],0x80,seg7[dbit7],seg7[dbit8]);}
			}
			if (dbit5!=9)
			dbit5++;
			else if (dbit5 ==9){
				dbit5 =0;
				if (dbit4!=5)
					dbit4++;
				else if (dbit4==5)
					dbit4=0;
			}
		
		}
		if(systick_2ms_status){
			systick_2ms_status = 0;
			Show_In_Seg(seg7[dbit1],seg7[dbit2],seg7[dbit3],seg7[dbit4],seg7[dbit5],0x80,seg7[dbit7],seg7[dbit8]);}
	}
}


void Solarterms(){
	if(month==2&&(day>=3&&day<18)){
			solartermsign = 1;
	}
	else if((month==2&&day>=18)||(month==3&&day<5)){
			solartermsign = 2;
	}
	else if(month==3&&(day<20&&day>=5)){
			solartermsign = 3;
	}
	else if((month==3&&day>=20)||(month==4&&day<4)){
			solartermsign = 4;
	}
	else if(month==4&&(day<20&&day>=4)){
			solartermsign = 5;
	}
  else if((month==4&&day>=20)||(month==5&&day<5)){
			solartermsign = 6;
	}
	else if(month==5&&(day>=5&&day<21)){
			solartermsign = 11;
	}
	else if((month==5&&day>=21)||(month==6&&day<5)){
			solartermsign = 12;
	}
	else if(month==6&&(day<21&&day>=5)){
			solartermsign = 13;
	}
	else if((month==6&&day>=21)||(month==7&&day<7)){
			solartermsign = 14;
	}
	else if(month==7&&(day<22&&day>=7)){
			solartermsign = 15;
	}
  else if((month==7&&day>=22)||(month==8&&day<7)){
			solartermsign = 16;
	}
	else if(month==8&&(day>=7&&day<23)){
			solartermsign = 21;
	}
	else if((month==8&&day>=23)||(month==9&&day<7)){
			solartermsign = 22;
	}
	else if(month==9&&(day<23&&day>=7)){
			solartermsign = 23;
	}
	else if((month==9&&day>=23)||(month==10&&day<8)){
			solartermsign = 24;
	}
	else if(month==10&&(day<23&&day>=8)){
			solartermsign = 25;
	}
  else if((month==10&&day>=23)||(month==11&&day<7)){
			solartermsign = 26;
	}
  else if(month==11&&(day>=7&&day<22)){
			solartermsign = 31;
	}
	else if((month==11&&day>=22)||(month==12&&day<7)){
			solartermsign = 32;
	}
	else if(month==12&&(day<21&&day>=7)){
			solartermsign = 33;
	}
	else if((month==12&&day>=21)||(month==1&&day<5)){
			solartermsign = 34;
	}
	else if(month==1&&(day<20&&day>=5)){
			solartermsign = 35;
	}
  else if((month==1&&day>=20)||(month==2&&day<3)){
			solartermsign = 36;
	}
	switch(solartermsign){
		case(1):
		case(2):
		case(3):
		case(4):
		case(5):
		case(6):
			Show_In_Seg(0x6d,0x73,seg7[solartermsign],0x80,0x80,0x80,0x80,0x80);
			break;
		case(11):
		case(12):
		case(13):
		case(14):
		case(15):
		case(16):
			Show_In_Seg(0x6d,0x3e,seg7[solartermsign-10],0x80,0x80,0x80,0x80,0x80);
			break;
		case(21):
		case(22):
		case(23):
		case(24):
		case(25):
		case(26):
			Show_In_Seg(0x77,0x3e,seg7[solartermsign-20],0x80,0x80,0x80,0x80,0x80);
			break;
		case(31):
		case(32):
		case(33):
		case(34):
		case(35):
		case(36):
			Show_In_Seg(0x6a,0x54,seg7[solartermsign-30],0x80,0x80,0x80,0x80,0x80);
			break;
	}
}


