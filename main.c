//SCK: -->P1.6
//SDA: -->P1.7
#include <msp430g2553.h>
#include <UART.h>

//address command SHT10
#define STATUS_REG_W 0x06     //0000_0110 //write status register
#define STATUS_REG_R 0x07     //0000_0111 //read status register
#define MEASURE_TEMP 0x03     //0000_0011 //Temperature Measurement
#define MEASURE_HUMI 0x05     //0000_0101 //Humidity measurement
#define RESET        0x1e     //0001_1110 //reset

#define bitselect   0x01    //Select the low reading of temperature and humidity
#define noACK         0     //no ACK returned
#define ACK           1     //ACK returned
#define HUMIDITY      2
#define TEMPERATURE   1

#define SCK         BIT6    //P1.6
#define SDA         BIT7    //P1.7
#define SCK_H       P1OUT |= SCK     //Clock high
#define SCK_L       P1OUT &= ~SCK    //Clock low
#define SDA_H       P1OUT |= SDA     //Data high
#define SDA_L       P1OUT &= ~SDA    //Data low

typedef union
{
unsigned int i;
float f;
}value;

/**********************************************************************************************************
**Function Name:      S_Init
**Description:      initialization
**Input Parameters:   no
**Output Parameters:no
**********************************************************************************************************/
void S_Init()
{
	P1SEL &= ~(SCK+SDA);
	P1DIR |= SCK;   //p1.6 output
	P1DIR &= ~SDA;  //p1.7 input
}

/**********************************************************************************************************
**Function Name:    S_Transstart
**Description:      start timing
**                  generates a transmission start
**                        _____         ________
**                  DATA:      |_______|
**                            ___     ___
**                  SCK : ___|   |___|   |______
**********************************************************************************************************/
void S_Transstart()
{
	P1DIR |= SDA; //P1.6 is output
	SDA_H;
	SCK_L;
	__no_operation();
	SCK_H;
	__no_operation();
	SDA_L;
	__no_operation();
	SCK_L;
	__no_operation();
	__no_operation();
	__no_operation();
	SCK_H;
	__no_operation();
	SDA_H;
	__no_operation();
	SCK_L;
	P1DIR &= ~SDA; //p1.6 is input
	// 2 xung Clock de TransStart
}

/**********************************************************************************************************
**Function Name:    S_Connectionreset
**Description:      Connection reset funtion
**                  communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
**                        _____________________________________________________         ________
**                  DATA:                                                      |_______|
**                           _    _    _    _    _    _    _    _    _        ___     ___
**                  SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
**********************************************************************************************************/
void S_Connectionreset()
{
	unsigned char Clk;
	P1DIR |= SDA;       //P1.6 output
	SDA_H;
	SCK_L;   
	for(Clk=0; Clk<9; Clk++)      //9 SCK cycles
	{
		SCK_H;
		SCK_L;
	}
	S_Transstart();     //transmission start
	//9 Clock (Data=1) + 2 Clock TransStart
}

/**********************************************************************************************************
**Function Name:      S_WriteByte
**Description:      Write function
**********************************************************************************************************/
char S_WriteByte(unsigned char value)
{
// write 1 byte tren Sensibus và check ACK
	unsigned char i, error=0;
	P1DIR |= SDA;
	for(i=0x80; i>0; i/=2)   
	{
		if(i&value)
		{			//neu value[8]=1 -> data=1; nguoc lai data=0, tuong tu cho cac bit 7,6,...
			SDA_H;
		}
		else
		{			
			SDA_L;
		}
		SCK_H;             //clk for SENSI-BUS
		__no_operation();
		__no_operation();
		__no_operation();
		SCK_L;
	}
	SDA_H;             //release DATA-line
	P1DIR &= ~SDA;     //P1.6 input
	SCK_H;             //clk #9 for ack
	error = P1IN;      //read data(ACK) after send command line form MSP to SHT10 --> error=input-P1.6
	error &= SDA;      //kiem tra ACK la 0 or 1
	P1DIR |= SDA;      //P1.6 output
	SCK_L;
	if(error)
	{
		return 1;	
	}
	else
	{
		return 0;    
	}
}

/**********************************************************************************************************
**Function Name:    S_ReadByte
**Description:      read function
**Input Parameters: ack
**********************************************************************************************************/
char S_ReadByte(unsigned char ack)
{
	unsigned char i, val;
	val = 0;
	P1DIR |= SDA;        //P1.6 is output
	SDA_H;               //release DATA-line;
	P1DIR &= ~SDA;       //P1.6 is input
	for(i=0x80; i>0; i/=2) //neu value[8]=1 -> data=1; nguoc lai data=0, tuong tu cho cac bit 7,6,...
	{
		SCK_H;          //clk for SENSI-BUS
		if(P1IN & SDA)  //neu input!=0
			val = (val|i);  //read bit
		SCK_L;
	}
	P1DIR |= SDA;    
	if(ack)           //in case of "ack==1" pull down DATA-Line
		SDA_L;
	else
		SDA_H;
	SCK_H;           //clk #9 for ack
	__no_operation();
	__no_operation();
	__no_operation();
	SCK_L;
	SDA_H;           //release DATA-line
	P1DIR &= ~SDA;   //P1.6 is input
	return val;
}

/**********************************************************************************************************
**Function Name:      S_Mearsure
**Description:        makes a measurement (humidity/temperature) with checksum
**Input Parameters:   *p_value       ,*p_checknum       ,mode
**********************************************************************************************************/
unsigned char S_Measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
{
	unsigned error;
	unsigned int i;
	error=0;
	
	S_Transstart();      //transmission start
	switch(mode)
	{                                 //send command to sensor
		case TEMPERATURE: error += S_WriteByte(MEASURE_TEMP); break;
		case HUMIDITY:    error += S_WriteByte(MEASURE_HUMI); break;
	}
	P1DIR &= ~SDA;                      //P1.6 is Input
	for(i=0; i<65535; i++)
	{
		if((P1IN & SDA) == 0)
			break; //wait until sensor has finished the measurement
	}
	if(P1IN & SDA)
		error += 1;
	*(p_value)  = S_ReadByte(ACK);   //read the first byte (MSB) 8 bit cao
	*(p_value+1)= S_ReadByte(ACK);   //read the second byte (LSB) 8 bit thap
	*p_checksum = S_ReadByte(noACK); //read checksum
	return error;
}

/**********************************************************************************************************
**Function Name:     S_Calculate
**Description:      calculate
**Input Parameters:   humi (12 bit) default
**                    temp (14 bit) default
**Output Parameters: humi [%RH]
**                   temp [do C]
**********************************************************************************************************/
void S_Calculate(unsigned int *p_humidity, unsigned int *p_temperature)
{
	//12bit humi / 14bit temp Resolution
	const float C1 = -2.0468;          // for 12 Bit Humidity conversion parameters
	const float C2 =  0.0367;          // for 12 Bit Humidity conversion parameters
	const float C3 = -0.0000015955;    // for 12 Bit Humidity conversion parameters
	const float D1 = -39.6;            // for 14 Bit @ 3V temperature
	const float D2 =  0.01;            // for 14 Bit @ 3V temperature
	const float T1 =  0.01;            // for 12 bit Temperature and humidity compensation signal
	const float T2 =  0.00008;         // for 12 bit Temperature and humidity compensation signal
	
	
	float rh = *p_humidity;    // rh: Humidity 12 Bit
	float t = *p_temperature;  // t :Temperature 14 Bit
	float rh_lin;              // rh_lin: Humidity linear
	float rh_true;             // rh_true: Temperature compensated humidity
	float t_C;                 // t_C : Temperature [do C]
	
	t_C = t*D2+D1;                        
	rh_lin = C3*rh*rh + C2*rh + C1;       
	rh_true = (t_C-25)*(T1+T2*rh)+rh_lin; 
	if(rh_true > 100)
		rh_true = 100;         //gioi han gia tri la 100
	if(rh_true < 0.1)
		rh_true = 0.1;         //gioi han gia tri 0.1
	
	*p_temperature = (unsigned int)t_C;     //return temperature [do C]
	*p_humidity = (unsigned int)rh_true;    //return humidity[%RH]
}

/**********************************************************************************************************
**Function Name:      main
**********************************************************************************************************/

void main()
{
// 1. connection reset
// 2. measure humidity (12 bit) and temperature (14 bit)
// 3. calculate humidity [%RH] and temperature

	value humi_val, temp_val;
	unsigned char error, checksum;
	unsigned int temphigh, templow, humihigh, humilow;
	
	WDTCTL=WDTPW+WDTHOLD; //Stop watchdog timer
	
	S_Init(); //khoi tao
	S_Connectionreset(); //connection reset
	while(1)
	{
		unsigned int i;
		error=0;
		error += S_Measure((unsigned char*) &humi_val.i, &checksum, HUMIDITY);    //measure humidity
		error += S_Measure((unsigned char*) &temp_val.i, &checksum, TEMPERATURE); //measure temperature
		if(error!=0)
			S_Connectionreset();
		else
		{
	
			//-------------Modify Resolution:12bit humi / 14bit temp----------------//
			//12bit humidity MSB=0x0000，mmmm(4bits valid) LSB=0xmmmm,mmmm(valid)
			humihigh = ((humi_val.i&0x0f)<<8);
			humilow = ((humi_val.i&0xff00)>>8);
			humi_val.i = humihigh+humilow;//Humidity
		
			//14bit temperature MSB=0x00mm,mmmm( 6bit valid) LSB=0xmmmm,mmmm(valid)
			temphigh = ((temp_val.i&0x3f)<<8);
			templow = ((temp_val.i&0xff00)>>8);
			temp_val.i = temphigh+templow;//Temperature
		
			S_Calculate(&humi_val.i, &temp_val.i);         //calculate humidity, temperature
		}
			
			UART_Init();
			UART_Write_Char(10);  //Ky tu xuong dong
			UART_Write_String("Nhiet do: ");
			UART_Write_Float(temp_val.i,5);
			UART_Write_String("       ");
			UART_Write_String("Do Am: ");
			UART_Write_Float(humi_val.i,5);
			//---------------------------- delay 2s------------------------------
			_delay_cycles(2000000);
		//-----------------------------------------------------------------------------------
	}
}
