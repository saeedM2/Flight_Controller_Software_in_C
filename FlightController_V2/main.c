#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "driverlib/sysctl.h"

void Reset_microcontroller(void);
int readData(uint8_t Register_Address);
void write(uint8_t Register_Address, uint16_t data);
int ReadData_Auxillary_Magnetometer(uint8_t Register_Address);
void write_I2C_Auxillary_Magnetometer(uint8_t Register_Address, uint16_t data);
void i2c_initilization(void);
void printString(char * string);//Help with printing on putty
void Transmit(uint16_t raw_data);//This function transmitts to putty
void Caliberate_IMU_sensor(int16_t *GYRO_XOUT_OFFSET, int16_t *GYRO_YOUT_OFFSET, int16_t *GYRO_ZOUT_OFFSET);
void initilize_UART(void);
void configure_IMU(void);
void Euler_acceleromter_Angle(int16_t accelerometerx, int16_t accelerometery, int16_t accelerometerz, int16_t *ACCEL_XANGLE, int16_t *ACCEL_YANGLE);
void Convert_Gyro_Rates_to_degrees_per_second(int16_t Gyroscopex, int16_t Gyroscopey, int16_t Gyroscopez, int16_t *GYRO_XRATE_degrees_per_second, int16_t *GYRO_YRATE_degrees_per_second, int16_t *GYRO_ZRATE_degrees_per_second);
void ComplementaryFilter(int16_t GYRO_XRATE_degrees_per_second, int16_t GYRO_YRATE_degrees_per_second, int16_t  GYRO_ZRATE_degrees_per_second, float *pitch, float *roll, int16_t ACCEL_XANGLE, int16_t ACCEL_YANGLE,float dt );
void print(int16_t data);
void Recieve(uint16_t *raw_data);
void PWM_initilization(void);
void initilize_Timer(void);
void configure_IMU(void);
void rollPID(float roll_conversion, float *pid_roll, float *previous_integral_error_roll, float *previousError_roll,float sampleTime,int8_t desired_roll);
void pitchPID(float pitch_conversion, float *pid_pitch, float *previous_integral_error_pitch,float *previousError_pitch,float sampleTime,int8_t desired_pitch);
void PWM_adjustment(float pid_pitch, float pid_roll, float pid_yaw,uint16_t baseSpeed);
void configure_MagentoMeter(void);
void Tilt_compensation(float roll, float pitch,float RawMagx, float RawMagy, float RawMagz, float *yaw);
void Calculate_Yaw_Filtered(float yaw, int16_t GYRO_ZRATE_degrees_per_second,float *Yaw_filtered);
void yawPID(float current_yaw, float *pid_yaw, float *previous_integral_error_yaw,float *previousError_yaw,float sampleTime, int8_t desired_yaw);
void i2c_Recover_from_bus_hang(void);

//Flag for UART1 interrupt
#define FLAG_NONE 0x00000000
#define FLAG_Data_Recieved_UART1 0x00000001
#define FLAG_ENTER_PRESSED '\r'
volatile uint8_t flag=FLAG_NONE;

#define iMax 250
#define iMin -250

#define pitch_kp 7
#define pitch_ki 0
#define pitch_kd 0

#define roll_kp 0
#define roll_ki 0
#define roll_kd 0

#define yaw_kp 0
#define yaw_ki 0
#define yaw_kd 0

//#define dt	0.01 //10ms sample rate

void UART1_Handler(void)
{
	int8_t read=0;
	while ((UART1_FR_R & (1<<4)) !=0);
	read=UART1_DR_R;

		if(UART1_DR_R =='\r')
		{
			flag=FLAG_ENTER_PRESSED;
			//clears the interrupt flag for UART for aknowledgement
			UART1_ICR_R |=(1<<4);//Receive Interrupt Clear
		}
		else if(UART1_DR_R !='\r')
		{
			flag=FLAG_Data_Recieved_UART1;
			//clears the interrupt flag for UART for aknowledgement
			UART1_ICR_R |=(1<<4);//Receive Interrupt Clear
		}
}


int main(void)
{
	//Reset Entire Microcntroler
	Reset_microcontroller();
	//SYSCTL_RCC_R=SYSCTL_DC1_MINSYSDIV_66 | 0x0 | SYSCTL_RCC_XTAL_20MHZ |SYSCTL_RCC_OSCSRC_MAIN;
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|0x0|SYSCTL_RCC_XTAL_20MHZ|SYSCTL_RCC_OSCSRC_MAIN);

	//Declare and initilize variables here:
	int16_t GYRO_XOUT_OFFSET=0;
	int16_t GYRO_YOUT_OFFSET=0;
	int16_t GYRO_ZOUT_OFFSET=0;

	int16_t ACCEL_XANGLE=0;
	int16_t ACCEL_YANGLE=0;

	int16_t GYRO_XRATE_degrees_per_second=0;
	int16_t GYRO_YRATE_degrees_per_second=0;
	int16_t GYRO_ZRATE_degrees_per_second=0;

	uint16_t U=0;//used to intilize an important flag
	float pitch=0;
	float roll=0;
	float yaw=0;
	float Yaw_filtered=0;
	float RawMagx=0;
	float RawMagy=0;
	float RawMagz=0;
	float initialYaw=0;
	uint16_t Mx_offset=0;
	float Mz_offset=0;
	float My_offset=0;
	float Scalex=0;
	float Scaley=0;
	float Scalez=0;

	int16_t pitch_conversion=0;
	int16_t roll_conversion=0;
	int16_t yaw_conversion=0;

	float pid_pitch=0; //pitch pid controller
	float pid_roll=0;
	float pid_yaw=0;
	float previousError_pitch=0;
	float previousError_roll=0;
	float previousError_yaw=0;
	float previous_integral_error_pitch=0;
	float previous_integral_error_roll=0;
	float previous_integral_error_yaw=0;

	float sampleTime=0;
	float SampleTime_START=0;
	float SampleTime_END=0;
	float SampleTime_END_converted=0;
	uint16_t k= 0; //Must be 16 bits signed
	int16_t raw_data=0;

	int8_t array[4]={0};
	uint8_t i=0;
	uint8_t n=0;

	//Declare and initilize reading variables for high and low bytes of the readings
	uint8_t accelerometerx_low=0;
	uint8_t accelerometerx_High=0;
	uint8_t accelerometery_low=0;
	uint8_t accelerometery_High=0;
	uint8_t accelerometerz_low=0;
	uint8_t accelerometerz_High=0;
	uint8_t Gyroscopex_low=0;
	uint8_t Gyroscopex_High=0;
	uint8_t Gyroscopey_low=0;
	uint8_t Gyroscopey_High=0;
	uint8_t Gyroscopez_low=0;
	uint8_t Gyroscopez_High=0;
	uint8_t Magnetometerx_High=0;
	uint8_t Magnetometerx_Low=0;
	uint8_t Magnetometery_High=0;
	uint8_t Magnetometery_Low=0;
	uint8_t Magnetometerz_High=0;
	uint8_t Magnetometerz_Low=0;

	int16_t accelerometerx=0;
	int16_t accelerometery=0;
	int16_t accelerometerz=0;
	int16_t Gyroscopex=0;
	int16_t Gyroscopey=0;
	int16_t Gyroscopez=0;
	int16_t Magnetometerx=0;
	int16_t Magnetometery=0;
	int16_t Magnetometerz=0;

	//Ensure The I2C recovers incase it hangs due to slave pulling data line down
	i2c_Recover_from_bus_hang();
	//initilize the UART0 and initilize the UART1 to communicate with xbee
	initilize_UART();
	//initlizate the i2c0 and i2c3 peripheral for IMU and magnetometer
	i2c_initilization();
	//Configure MPU6050 IMU sensor (settings chosen within the functions)
	configure_IMU();
	//Configure HMC5983 sensor
	configure_MagentoMeter();
	//retrieve the caliberation Gyroscope offset values for next set of calculations (used for calculating Euler angles)
	Caliberate_IMU_sensor(&GYRO_XOUT_OFFSET, &GYRO_YOUT_OFFSET, &GYRO_ZOUT_OFFSET);
	//PWM initilizations
	PWM_initilization();
	//Initilize Timer peripheral for Timing the Loop frequency
	initilize_Timer();

	while(1)
	{
		//Re-initilize timer
		initilize_Timer();
		//Start Timer
		SampleTime_START=TIMER0_TAR_R;//time given in seconds

		//ACCEL_XOUT[15:8]
		accelerometerx_High=readData(0x3B);
		//ACCEL_XOUT[7:0]
		accelerometerx_low=readData(0x3C);
		//ACCEL_YOUT[15:8]
		accelerometery_High=readData(0x3D);
		//ACCEL_YOUT[7:0]
		accelerometery_low=readData(0x3E);
		//ACCEL_ZOUT[15:8]
		accelerometerz_High=readData(0x3F);
		//ACCEL_ZOUT[7:0]
		accelerometerz_low=readData(0x40);

		//GYRO_XOUT[15:8]
		Gyroscopex_High=readData(0x43);
		//GYRO_XOUT[7:0]
		Gyroscopex_low=readData(0x44);
		//GYRO_YOUT[15:8]
		Gyroscopey_High=readData(0x45);
		//GYRO_YOUT[7:0]
		Gyroscopey_low=readData(0x46);
		//GYRO_ZOUT[15:8]
		Gyroscopez_High=readData(0x47);
		//GYRO_ZOUT[7:0]
		Gyroscopez_low=readData(0x48);

		//Magnetometer_XOUT[15:8]
		Magnetometerx_High=ReadData_Auxillary_Magnetometer(0x03);
		//Magnetometer_XOUT[7:0]
		Magnetometerx_Low=ReadData_Auxillary_Magnetometer(0x04);
		//Magnetometer_YOUT[15:8]
		Magnetometery_High=ReadData_Auxillary_Magnetometer(0x07);
		//Magnetometer_YOUT[7:0]
		Magnetometery_Low=ReadData_Auxillary_Magnetometer(0x08);
		//Magnetometer_ZOUT[15:8]
		Magnetometerz_High=ReadData_Auxillary_Magnetometer(0x05);
		//Magnetometer_ZOUT[7:0]
		Magnetometerz_Low=ReadData_Auxillary_Magnetometer(0x06);

		//Reconcile High and Low bytes of the readings
		accelerometerx=accelerometerx_High<<8 | accelerometerx_low;
		accelerometery=accelerometery_High<<8 | accelerometery_low;
		accelerometerz=accelerometerz_High<<8 | accelerometerz_low;

		//Reconcile High and Low bytes of the readings and retrieve the true value by subtracting the offset value (sensor bias) from the measured values
		Gyroscopex=(Gyroscopex_High<<8 | Gyroscopex_low)-GYRO_XOUT_OFFSET;
		Gyroscopey=(Gyroscopey_High<<8 | Gyroscopey_low)-GYRO_YOUT_OFFSET;
		Gyroscopez=(Gyroscopez_High<<8 | Gyroscopez_low)-GYRO_ZOUT_OFFSET;

		//calculate the Final gyrscope angle rate by dividing by the sensitivity listed in the data sheet
		//Convert_Gyro_Rates_to_degrees_per_second(Gyroscopex, Gyroscopey, Gyroscopez, &GYRO_XRATE_degrees_per_second, &GYRO_YRATE_degrees_per_second, &GYRO_ZRATE_degrees_per_second);

		//Reconcile High and Low bytes of the readings
		Magnetometerx=Magnetometerx_High<<8 | Magnetometerx_Low;
		Magnetometery=Magnetometery_High<<8 | Magnetometery_Low;
		Magnetometerz=Magnetometerz_High<<8 | Magnetometerz_Low;

		//Calibrate Against Hard-Iron Distortion
		Mx_offset=143;
		My_offset=-482.50;
		Mz_offset=-482.50;
		//Calibrate Against Soft-Iron Distortion
		Scalex=0.9652;
		Scaley=1.0184;
		Scalez=0.9040;

		//Caliberate (Soft and Hard Distortion)
		Magnetometerx=Scalex*(Magnetometerx-Mx_offset);
		Magnetometery=Scaley*(Magnetometery-My_offset);
		Magnetometerz=Scalez*(Magnetometerz-Mz_offset);

		//Temporary Auxillary Variable
		RawMagx=(float)Magnetometerx;
		RawMagy=(float)Magnetometery;
		RawMagz=(float)Magnetometerz;

	/*	printString("\r\n");
		print(Magnetometerx);
		printString("\t");
		print(Magnetometery);
		printString("\t");
		print(Magnetometerz);*/

		//calculate the roll and pitch using accelerometer!
		Euler_acceleromter_Angle(accelerometerx, accelerometery, accelerometerz, &ACCEL_XANGLE, &ACCEL_YANGLE);

		//calculate the Final gyrscope angle rate by dividing by the sensitivity listed in the data sheet
		Convert_Gyro_Rates_to_degrees_per_second(Gyroscopex, Gyroscopey, Gyroscopez, &GYRO_XRATE_degrees_per_second, &GYRO_YRATE_degrees_per_second, &GYRO_ZRATE_degrees_per_second);

		//calculate the roll and pitch using sensor fusion (compplementary filter)
		ComplementaryFilter(GYRO_XRATE_degrees_per_second, GYRO_YRATE_degrees_per_second, GYRO_ZRATE_degrees_per_second, &pitch, &roll, ACCEL_XANGLE, ACCEL_YANGLE,SampleTime_END_converted );

		//convert data (for Printing on Terminal)
		pitch_conversion=(int16_t)pitch;
		roll_conversion=(int16_t)roll;

		//Calculate Yaw and Perfrom Tilt compnesation
		Tilt_compensation(roll, pitch, RawMagx, RawMagy, RawMagz, &yaw);

    	if(U==150)
		{
			initialYaw=yaw;
		}
		yaw=yaw-initialYaw;

		//Fuse Magnetometer data and gyroscope data to estimate Yaw with minimum drift// The output for this function is "Yaw_filtered"
		Calculate_Yaw_Filtered(yaw, GYRO_ZRATE_degrees_per_second,&Yaw_filtered);

		//convert data
		yaw_conversion=(int16_t)Yaw_filtered;


		if(U==0){
		printString("Roll");
		}
		printString("\t");
		if(U==0){
		printString("Pitch");
		}
		printString("\t");
		if(U==0)
		{
		printString("Yaw");
		}
		printString("\r\n");
		print(roll_conversion);
		printString("\t");
		print(pitch_conversion);
		printString("\t");
		print(yaw_conversion);
		printString("\t");

		if( U < 170)
		{
			U++;//once set we never print "Roll" and "Pitch" and "Yaw"
		}
		/*print(PWM0_2_CMPA_R);//m1
		printString("\t");
		print(PWM0_0_CMPA_R);//m2
		printString("\t");
		print(PWM0_1_CMPA_R);//m3
		printString("\t");
		print(PWM1_1_CMPA_R);//m4
		printString("\r\n");
		printString("\t");*/

		// read the duty cyle for motor
		if( (flag == FLAG_Data_Recieved_UART1))
		{
		  //while ((UART1_FR_R & (1<<4)) !=0);
		  raw_data=UART1_DR_R;
		  array[n]=raw_data-48;//convert from ASCII to raw numbers
		  n++;
		  flag=FLAG_NONE;
		}
		if(flag == FLAG_ENTER_PRESSED)
		{
		   k= 0;
		   while(i<n)
		   {
			   k = 10 * k + array[i];
			   i++;
		   }
		   raw_data=0;
		   array[0]=0;
		   array[1]=0;
		   array[2]=0;
		   array[3]=0;
		   i=0;
		   n=0;
		   flag=FLAG_NONE;
		  }
	//*****************************************************************************
		//motor shut off
		PWM0_2_CMPA_R= k-1;//(RED)//motor1
		PWM0_0_CMPA_R= k-1;//(Yellow)//motor2
		PWM0_1_CMPA_R= k-1;//(ORANGE)//motor3
		PWM1_1_CMPA_R =k-1;//(green)//motor4

		//End counting time and now subtract the difference to find the elapsed time
		SampleTime_END=((TIMER0_TAR_R)-(SampleTime_START));
		SampleTime_END_converted=(SampleTime_END/(80000000));
		sampleTime=SampleTime_END_converted;

	 //Take Corrective action ONLY is angles are reasonable
	 if(pitch_conversion >-90 && pitch_conversion <90 &&  roll_conversion >-90 && roll_conversion < 90)
	 {
		if(k>=38)
		{
			int16_t desired_pitch =  2;
			int16_t desired_roll  = 0;
			int16_t desired_yaw=0;

			//Calculate PID gains
			if((-4 > pitch) || (pitch > 4))
			{
				//PID control functions
				pitchPID(pitch,&pid_pitch,&previous_integral_error_pitch,&previousError_pitch,sampleTime, desired_pitch);
			}
			if((-3 > roll)  || (roll > 3))
			{
				//PID control functions
				rollPID(roll, &pid_roll,&previous_integral_error_roll,&previousError_roll,sampleTime, desired_roll);
			}
			if((-4 > Yaw_filtered ) || (Yaw_filtered > 4))
			{
				//PID control functions
				yawPID(Yaw_filtered, &pid_yaw, &previous_integral_error_yaw,&previousError_yaw,sampleTime,desired_yaw);
			}

			//Adjust speed
			PWM_adjustment(pid_pitch, pid_roll, pid_yaw,k);

			//Reset respective PID gains when conditions are met
			if((-4 < pitch) && (pitch < 4))
			{
				pid_pitch=0;//reset the variable
				previous_integral_error_pitch=0;
			}
			if((-4 < roll) && (roll < 4))
			{
				pid_roll=0;//reset the variable
				previous_integral_error_roll=0;
			}
			if((-4 < Yaw_filtered ) && (Yaw_filtered < 4))
			{
				pid_yaw=0;//reset the variable
				previous_integral_error_yaw=0;
			}
		}
	 }
		}
	return 0;
}
void Reset_microcontroller(void)
{
	//Reset Entire Microcontroller Upon Program Initiation
	NVIC_APINT_R = NVIC_APINT_SYSRESETREQ;
	//Shutdown VDD
/*
	//Hibernate (shutdown power)
	//1 WC interrupt
	HIB_IM_R = 0x00000010;
	//2 enable the oscillator input
	HIB_CTL_R = 0x40;
	//3 wait for wakeup flag is set
	//while((HIB_MIS_R & 0b1000 == 1)==0);
*/
}
void initilize_Timer(void)
{
	//11.4 enable the clock for timer
	SYSCTL_RCGCTIMER_R |= 0b00000001;
	//1)GPTM control- Timer0 Subtimer A is initially cleared. Can also select timer A or B or select what timer will do at each mode.
	TIMER0_CTL_R &= ~(1<<0);
	//2)GPTM configuratation register
	TIMER0_CFG_R |= 0x00000000;
	//3)Configure the Timer n mode register (one shot or periodic pg.722 of Data sheet). We selected 0x02 for periodic mode
	TIMER0_TAMR_R |= (0x1<<0);
	//4) The direction of counter (count up)
	TIMER0_TAMR_R |= (1<<4);
	//7)Enable the timer A
	TIMER0_CTL_R |= (1<<0);
}
//Recieves information from Port B, pin PB0. Note pin PB0 will connects to Dout pin of the Xbee
void Recieve(uint16_t *raw_data)
{
	while ((UART1_FR_R & (1<<4)) !=0){};
	*raw_data=UART0_DR_R;
}
void initilize_UART(void)
{
	//UART0
	SYSCTL_RCGCUART_R |= (1<<0);
	SYSCTL_RCGCGPIO_R |= (1<<0);
	GPIO_PORTA_AFSEL_R|= (1<<1)|(1<<0);
	GPIO_PORTA_PCTL_R |= (1<<0)|(1<<4);
	GPIO_PORTA_DEN_R  |= (1<<0)|(1<<1);
	//we want a baudrate of 115200
	//BRD = 16,000,000 / (16 * 115200) = 10.8507
	//UARTFBRD[DIVFRAC] = integer(0.8507 * 64 + 0.5) = 54
	UART0_CTL_R &= ~(1<<0);
	UART0_IBRD_R = 7;//128000 bud rate
	UART0_FBRD_R = 52;//128000 bud rate
	UART0_LCRH_R = (0x3<<5);
	UART0_CC_R = 0x5;
	UART0_CTL_R = (1<<0)|(1<<8)|(1<<9);

	//UART1
	//1.
	SYSCTL_RCGCUART_R |=0b00000010;//activated uart1 clock
	//2.
	SYSCTL_RCGCGPIO_R |=0b00000010;
	//3.
	GPIO_PORTB_AFSEL_R |=0b00000011; //set pin B0 and B1
	//4.// initialize empty FIFOs
	//5.
	GPIO_PORTB_PCTL_R |=(1<<0)|(1<<4);//same as 0x00000011 as shown on Table 23.5
	GPIO_PORTB_DEN_R |=0b00000011;
	//1. Disable the UART by clearing the UARTEN bit in the UARTCTL register.
	UART1_CTL_R &= ~(1<<0);
	//2. write the integer portion of the BRD to the UARTIBRD register.
	UART1_IBRD_R |= 104;
	//3. write the fractional portion of the BRD to the UARTFBRD register.
	UART1_FBRD_R |=11;
	//4. write the desired serial parameters to the UARTLCRH register (in this case, a valueof
	//0x0000.0060).
	UART1_LCRH_R &=0b00000000;//no parity
	UART1_LCRH_R &=~(1<<3);//one stop bit selected
	UART1_LCRH_R |= (UART_LCRH_WLEN_8);
	//UART1_LCRH_R |= (UART_LCRH_WLEN_8|UART_LCRH_FEN);//0x03 corresponds to 0b11 in Binary but we can shift bit 1 by 6 by and bit 0 by 7  (bit field 6:5 in data sheet) to give us 60 in hex (0b00000011)-->(0b01100000). FIFO enable
	//UART1_IFLS_R &= ~0x3F; ;  // clear TX and RX interrupt FIFO level fields// configure interrupt for RX FIFO >= 1/8 full
	//5. Configure the UART clock source by writing to the UARTCC register.
	UART1_CC_R=0X5;
	//Enable interrupt
	UART1_IM_R |= 0b00010000;//Recieve interrupt is enabled
	//7. Enable the UART by setting the UARTEN bit in the UARTCTL register.
	UART1_CTL_R |= 1<<0;//enable the uart
	UART1_CTL_R |=1<<8;//enable the transmit
	UART1_CTL_R |=1<<9;//enable the recieve
	//6.NVIC enable[0] activated because we need to access UART1 located at IRQ 6 of the Vtable;
	NVIC_EN0_R |=0b01000000;//Set bit 6 for Directing interrupt from IRQ 6 to CPU
}
void i2c_initilization(void)
{
	//Reset the i2c bus
	int16_t i=10000;
	while(i>=0)
	{
		SYSCTL_SRI2C_R=0b1111;
		i--;
	}
	i=10;
	while(i>=0)
	{
		SYSCTL_SRI2C_R=0b0000;
		i--;
	}
	//initilize i2c0
	//1.
	SYSCTL_RCGCI2C_R |= 0b00000001;//selected i2c module 0 (will connect to portB pin 2 and 3 with I2C0_SCL and I2C0_SDA respectively
	//2.
	SYSCTL_RCGCGPIO_R |= 0b00000010;//Clock to port B (pin 2 and 3 will have I2C0_SCL and I2C0_SDA respectively)
	//3.
	GPIO_PORTB_AFSEL_R |= 0b00001100;//pin 2 and 3 are assigned alternative function. Nanmely connecting to I2C0_SCL and I2C0_SDA respectively
	GPIO_PORTB_DEN_R |= 0b00001100;//set pin 2 and 3 to be digitcal pins
	//4.
	GPIO_PORTB_ODR_R |= (1<<3);//set SDA pin to open drain (PB3M, PIN4 is set to high)
	//5.
	GPIO_PORTB_PCTL_R |= 0x00003300;
	//6. Initialize the I2C Master by writing the I2CMCR register with a value of 0x0000.0010.
	I2C0_MCR_R |= (1<<4);
	//7.
	I2C0_MTPR_R |= 0x00000009;

	//Initilize I2c3
	//1.
	SYSCTL_RCGCI2C_R |=(1<<3);//selected i2c module 3 (D0 and D1)
	//2.
	SYSCTL_RCGCGPIO_R |=(1<<3);//Clock to port D
	//3.
	GPIO_PORTD_AFSEL_R |=0b00000011;//pin 0 and 1 are assigned alternative function.
	GPIO_PORTD_DEN_R |=0b00000011;//set pin 0 and 1 to be digitcal pins
	//4.
	GPIO_PORTD_ODR_R |=(1<<1);//set SDA pin to open drain (bit 0 (pin 1) is set to high)
	//5.
	GPIO_PORTD_PCTL_R |=0x00000033;
	//6. Initialize the I2C Master by writing the I2CMCR register with a value of 0x0000.0010.
	I2C3_MCR_R |= (1<<4);
	//7.
	I2C3_MTPR_R |=0x00000009;
}
void i2c_Recover_from_bus_hang(void)
{
	//MPU6050 Recovery
	SYSCTL_RCGCGPIO_R = 0b00000010;//Clock to port B (pin 2 and 3 will have I2C0_SCL and I2C0_SDA respectively)
	GPIO_PORTB_DIR_R = 0b00000100;
	GPIO_PORTB_AFSEL_R &= 0b00000000;//Disable Alternative function
	GPIO_PORTB_DEN_R |= 0b00001100;//set pin 2 and 3 to be digital pins
	int8_t i=10;
	while(i>=0)
	{
		GPIO_PORTB_DATA_R = 0b00000100;
		i--;
	}
	GPIO_PORTB_DATA_R &= 0b0000000;
	GPIO_PORTB_DATA_R |= 0b0001100;
	GPIO_PORTB_DATA_R &= 0b0000000;
	//HMC5983 Recovery
	SYSCTL_RCGCGPIO_R = (1<<3);//Clock to port B (pin 0 and 1 will have I2C0_SCL and I2C0_SDA respectively)
	GPIO_PORTD_DIR_R = 0b00000001;
	GPIO_PORTD_AFSEL_R &= 0b00000000;//Disable Alternative function
	GPIO_PORTD_DEN_R |= 0b00000011;//set pin 0 and 1 to be digital pins
	i=10;
	while(i>=0)
	{
		GPIO_PORTD_DATA_R = 0b00000001;
		i--;
	}
	GPIO_PORTD_DATA_R &= 0b0000000;
	GPIO_PORTD_DATA_R |= 0b0000011;
	GPIO_PORTD_DATA_R &= 0b0000000;
}
int readData(uint8_t Register_Address)
{
    //select the register to read from
	I2C0_MSA_R=(0b11010000);// select write mode
	I2C0_MDR_R=Register_Address;//write the register to read from
	I2C0_MCS_R=0x00000003;////run start
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent

	I2C0_MSA_R=(0b11010001);// select read mode
	I2C0_MCS_R=0x00000007;//run  start stop
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent

	return (I2C0_MDR_R);
}
void write(uint8_t Register_Address, uint16_t data)
{
	I2C0_MSA_R=(0b11010000);// select write mode
	I2C0_MDR_R= Register_Address;//select the register
	I2C0_MCS_R=0x00000003;//run start
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent

	I2C0_MDR_R=data;///Write your bit field setting (data) to the corresponding registe address
	I2C0_MCS_R= 0x00000005;//run  stop
	while(I2C0_MCS_R & 0b00000001 != 0);//wait until sent
}
int ReadData_Auxillary_Magnetometer(uint8_t Register_Address)
{
	//select the register to read from
	I2C3_MSA_R=(0x3C);// select write_I2C mode
	I2C3_MDR_R=Register_Address;//write_I2C the register to read from
	I2C3_MCS_R=0x00000003;////run start
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent

	I2C3_MSA_R=(0x3D);// select read mode
	I2C3_MCS_R=0x00000007;//run  start stop
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent

	return (I2C3_MDR_R);
}
void write_I2C_Auxillary_Magnetometer(uint8_t Register_Address, uint16_t data)
{
	I2C3_MSA_R=(0x3C);// select write_I2C mode
	I2C3_MDR_R= Register_Address;//select the register
	I2C3_MCS_R=0x00000003;//run start
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent

	I2C3_MDR_R=data;///write_I2C your bit field setting (data) to the corresponding registe address
	I2C3_MCS_R= 0x00000005;//run  stop
	while(I2C3_MCS_R & 0b00000001 != 0);//wait until sent
}
void printString(char * string)//Help with printing on putty
{
	while(*string)
	{
		Transmit(*(string)++);
	}
}
void Transmit(uint16_t raw_data)//This function transmitts to putty
{
	while(UART0_FR_R & (1<<5))//check the UART flag register to see if any previous transmission has ended bevause we need to check there is something to send
	{
		//keep looping until the transmission register is empty
	}
		UART0_DR_R =raw_data; //sends the value sent to putty via UART0 port A, pin A1
}
void Caliberate_IMU_sensor(int16_t *GYRO_XOUT_OFFSET, int16_t *GYRO_YOUT_OFFSET, int16_t *GYRO_ZOUT_OFFSET)
{

	uint8_t Gyroscopex_low=0;
	uint8_t Gyroscopex_High=0;
	uint8_t Gyroscopey_low=0;
	uint8_t Gyroscopey_High=0;
	uint8_t Gyroscopez_low=0;
	uint8_t Gyroscopez_High=0;

	int16_t Gyroscopex=0;
	int16_t Gyroscopey=0;
	int16_t Gyroscopez=0;

	int16_t GYRO_XOUT_OFFSET_1000SUM=0;
	int16_t GYRO_YOUT_OFFSET_1000SUM=0;
	int16_t GYRO_ZOUT_OFFSET_1000SUM=0;

	int x = 0;
	for(x = 0; x<1000; x++)
	{
		//GYRO_XOUT[15:8]
		Gyroscopex_High=readData(0x43);
		//GYRO_XOUT[7:0]
		Gyroscopex_low=readData(0x44);
		//GYRO_YOUT[15:8]
		Gyroscopey_High=readData(0x45);
		//GYRO_YOUT[7:0]
		Gyroscopey_low=readData(0x46);
		//GYRO_ZOUT[15:8]
		Gyroscopez_High=readData(0x47);
		//GYRO_ZOUT[7:0]
		Gyroscopez_low=readData(0x48);

		Gyroscopex=Gyroscopex_High<<8 | Gyroscopex_low;
		Gyroscopey=Gyroscopey_High<<8 | Gyroscopey_low;
		Gyroscopez=Gyroscopez_High<<8 | Gyroscopez_low;

		GYRO_XOUT_OFFSET_1000SUM=GYRO_XOUT_OFFSET_1000SUM+Gyroscopex;
		GYRO_YOUT_OFFSET_1000SUM=GYRO_YOUT_OFFSET_1000SUM+Gyroscopey;
		GYRO_ZOUT_OFFSET_1000SUM=GYRO_ZOUT_OFFSET_1000SUM+Gyroscopez;
	}
	*GYRO_XOUT_OFFSET=GYRO_XOUT_OFFSET_1000SUM/1000;
	*GYRO_YOUT_OFFSET=GYRO_YOUT_OFFSET_1000SUM/1000;
	*GYRO_ZOUT_OFFSET=GYRO_ZOUT_OFFSET_1000SUM/1000;
}
void configure_MagentoMeter(void)
{
	//Configuration A Register
	write_I2C_Auxillary_Magnetometer(0x00, 0b11011000);
	//Configutation B Register
	write_I2C_Auxillary_Magnetometer(0x01, 0b00000000);
	//Mode Register
	write_I2C_Auxillary_Magnetometer(0x02, 0b00000000);
}
void configure_IMU(void)
{
	//*********************************************Gyroscope and Accelerometer***********************************************
	//Reset registers
	write(0x6B, 0b10000000);
	write(0x19, 0x00);
	write(0x19, 0x00);
	write(0x19, 0x00);
	write(0x19, 0x00);
	write(0x19, 0x00);
	write(0x19, 0x00);
	write(0x1A, 0x00);
	write(0x1C, 0x00);
	write(0x1B, 0x00);
	write(0x6B, 0x00);

	//Register 107 – Power Management 1//reset device
	write(0x6B, 0b10000000);
	//(1) Register 25 – Sample Rate Divider// Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	write(0x19, 0x00);
	//(2) Register 25 – Sample Rate Divider// Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	write(0x19, 0x03);
	//(3) Register 25 – Sample Rate Divider// Set Gyro Full Scale Range
	write(0x19, 2<<3);
	//(4) Register 25 – Sample Rate Divider// Set Accelerometer Full Scale
	write(0x19, 3<<3);
	//(2) Register 25 – Sample Rate Divider// 41 Hz Acc filtering
	write(0x19, 0x03);
	//(5) Register 25 – Sample Rate Divider// SMPLRT_DIV = 0
	write(0x19, 0x00);
	//Register 26 – Configuration// disable input pin for data sync
	write(0x1A, 0x06);
	//Register 28 –  Configure Accelerometer (Full Scale = +/-2G)
	write(0x1C, 0x00<< 4);
	//Register 27 – Gyroscope Configuration ( FS_SEL: 0; Full Scale Range: ± 250 °/s; LSB Sensitivity: 131 LSB/°/s)
	write(0x1B, 0x00<< 4);
	//Register 107 – Power Management 1// turn sleep mode off
	write(0x6B, 0x00);
}
void Euler_acceleromter_Angle(int16_t accelerometerx, int16_t accelerometery, int16_t accelerometerz, int16_t *ACCEL_XANGLE, int16_t *ACCEL_YANGLE )
{
	*ACCEL_XANGLE = 57.295*atan((float)accelerometery/sqrt(pow((float)accelerometerz,2)+pow((float)accelerometerx,2)));
	*ACCEL_YANGLE = 57.295*atan((float)-accelerometerx/sqrt(pow((float)accelerometerz,2)+pow((float)accelerometery,2)));
}
void Convert_Gyro_Rates_to_degrees_per_second(int16_t Gyroscopex, int16_t Gyroscopey, int16_t Gyroscopez, int16_t *GYRO_XRATE_degrees_per_second, int16_t *GYRO_YRATE_degrees_per_second, int16_t *GYRO_ZRATE_degrees_per_second)
{
	//131 is the sensitivity constant from Datasheet
	*GYRO_XRATE_degrees_per_second=(float)Gyroscopex*(0.007634);
	*GYRO_YRATE_degrees_per_second=(float)Gyroscopey*(0.007634);
	*GYRO_ZRATE_degrees_per_second=(float)Gyroscopez*(0.007634);

	//Note we did implement the sensitivity division for the accelerometer like we did above (for gyroscope) because the atan2 function just relies on the ratio of the two for calulating the angles.
}
void Tilt_compensation(float roll, float pitch,float RawMagx, float RawMagy, float RawMagz, float *yaw)
{
	//Speedup Calculations by Storing Repetitive Calculations:
	float cosRoll = cos(roll*0.01745);
	float sinRoll = sin(roll*0.01745);
	float cosPitch =cos(pitch*0.01745);
	float sinPitch =sin(pitch*0.01745);

	//Calculate tilt Compensated Yaw
	float Y_h = (RawMagx*sinRoll*sinPitch) + (RawMagy*cosRoll) - (RawMagz*sinRoll*cosPitch);
	float X_h = (RawMagx*cosPitch) + (RawMagz*sinPitch);

	*yaw=57.295*atan2(Y_h, X_h);
	//*yaw=57.295*atan(Y_h / X_h);
/*	if (Heading < 0)
	Heading += 360;*/
}
void Calculate_Yaw_Filtered(float yaw, int16_t GYRO_ZRATE_degrees_per_second,float *Yaw_filtered)
{
	*Yaw_filtered= *Yaw_filtered+GYRO_ZRATE_degrees_per_second*0.0030;
	*Yaw_filtered=  (*Yaw_filtered)*(0.98) + (yaw)*(0.02);
}
void ComplementaryFilter(int16_t GYRO_XRATE_degrees_per_second, int16_t GYRO_YRATE_degrees_per_second, int16_t  GYRO_ZRATE_degrees_per_second, float *pitch, float *roll, int16_t ACCEL_XANGLE, int16_t ACCEL_YANGLE, float dt )
{
	//(1) calculate the roll and pitch without any compnesation from the accelerometer data
	*roll=*roll +(float)GYRO_XRATE_degrees_per_second*0.0030; // Angle around the X-axis
	*pitch=*pitch+(float)GYRO_YRATE_degrees_per_second*0.0030;// Angle around the Y-axis

	//(2) Next, calcualate the compensated (more accurate) data using the complementary filter (sensor fusion)
	*roll=*roll*0.98+ACCEL_XANGLE*0.02;
	*pitch=*pitch*0.98+ACCEL_YANGLE*0.02;
}
void print(int16_t data)
{
	int8_t array[5]={0};

	if(data < 0)
	{
	data=data*(-1);
	printString("-");
	}
	int digit=0;
	int i=0;

	while(data > 0)
	{
		 digit = (int16_t)data % 10;
		 array[i]=digit+48;//+48 because we need convert digits to a value that can be seen by putty
		 i++;
		 data /= 10;
	}
	if(data==0 & i==0)
	{
		while((UART0_FR_R & (1<<5)) != 0);
		Transmit(0+48);
	}
	int n=i;
	while(n>-1)
	{
		while((UART0_FR_R & (1<<5)) != 0);
		Transmit(array[n]);
		n--;
	}
}
void PWM_initilization(void)
{
	uint16_t period=400;
	//PWM INITILIZATION
	SYSCTL_RCGCPWM_R |= 0b00000011;;//unlock pwm
	SYSCTL_RCGC2_R|=0b00011111;//choose the port that recieves clock
	while((SYSCTL_PRGPIO_R & 0x00000008) == 0); // wait until port D is ready
	while((SYSCTL_PRGPIO_R & 0x00000010) == 0);
	while((SYSCTL_PRGPIO_R & 0x00000004) == 0);
	while((SYSCTL_PRGPIO_R & 0x00000002) == 0);
	while((SYSCTL_PRGPIO_R & 0x00000001) == 0);

	GPIO_PORTB_AFSEL_R |=0b01010000;//
	GPIO_PORTA_AFSEL_R |=0b01000000;//
	GPIO_PORTE_AFSEL_R |=0b00010000;//

	GPIO_PORTB_PCTL_R &= ~0xFF0F0000;
	GPIO_PORTB_PCTL_R |=0x04040000;//

	GPIO_PORTA_PCTL_R &= ~0x0F000000;
	GPIO_PORTA_PCTL_R |=  0x05000000;//

	GPIO_PORTE_PCTL_R &= ~0x000F0000;
	GPIO_PORTE_PCTL_R |=0x00040000;//

	GPIO_PORTB_AMSEL_R &=~(0b01010000); //
	GPIO_PORTB_DEN_R |=0b01010000;   //

	GPIO_PORTA_AMSEL_R &= ~(0b01000000); //
	GPIO_PORTA_DEN_R   |=   0b01000000;   //

	GPIO_PORTE_AMSEL_R &= ~(0b00010000);//
	GPIO_PORTE_DEN_R |= 0b00010000; //

	SYSCTL_RCC_R |=1<<20; //The PWM clock divider is the source for the PWM clock.
	SYSCTL_RCC_R |=SYSCTL_RCC_PWMDIV_M;//clear the control register
	SYSCTL_RCC_R |=SYSCTL_RCC_PWMDIV_8; //configure for /8 divider

	PWM0_0_CTL_R = 0b00000000;////B6
	PWM0_1_CTL_R = 0b00000000;//B4
	PWM0_2_CTL_R = 0b00000000;//E4
	PWM1_1_CTL_R = 0b00000000;//A6

	PWM0_0_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO; //
	PWM0_1_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO; //
	PWM0_2_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO;//
	PWM1_1_GENA_R |=PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO; // CMPA down, if counter=CMPA drive PWMA low, high if counter=LOAD
	//PWM0_3_GENB_R |= 0x0000080C; // CMPB down, if counter=CMPB drive PWMB low, high if counter=LOAD

	PWM0_0_CTL_R &= ~0x00000002; // count down mode (cleared second bit)
	PWM0_1_CTL_R &= ~0x00000002; // count down mode (cleared second bit)
	PWM0_2_CTL_R &= ~0x00000002; // count down mode (cleared second bit)
	PWM1_1_CTL_R &= ~0x00000002; // count down mode (cleared second bit)

	PWM0_0_LOAD_R |=period-1; //
	PWM0_1_LOAD_R |=period-1; //
	PWM0_2_LOAD_R |=period-1;//
	PWM1_1_LOAD_R |=period-1; //

	PWM0_0_CTL_R |= (PWM_0_CTL_ENABLE);//
	PWM0_1_CTL_R |= (PWM_1_CTL_ENABLE);//
	PWM0_2_CTL_R |= (PWM_2_CTL_ENABLE);//
	PWM1_1_CTL_R |= (PWM_1_CTL_ENABLE);//

	//Here you enable the PWM from Generator 0 1 2 and 3 into (each generator has two channels) to a specigic pin
	PWM0_ENABLE_R |=0b00010101;//
	PWM1_ENABLE_R |=0b00000100;//
}
void rollPID(float current_roll, float *pid_roll, float *previous_integral_error_roll, float *previousError_roll,float sampleTime, int8_t desired_roll)
{
	float previous_integral_error_print=0;
	float previousError_roll_print=0;

	previous_integral_error_print=*previous_integral_error_roll;
	previousError_roll_print=*previousError_roll;

	float	P_term,I_term=0, D_term;
	float	error_roll = desired_roll - current_roll;
	int16_t pid_roll_print=0;

	if(sampleTime>0.2)
	{
		sampleTime=0.0035;
	}
	if(sampleTime < 0 )
	{
		sampleTime=0.0035;
	}
	P_term = roll_kp*error_roll; //proportional
	I_term = previous_integral_error_print+roll_ki*(error_roll*sampleTime);
	D_term = roll_kd * ((error_roll-previousError_roll_print)/sampleTime);

	*previousError_roll = error_roll;
	*previous_integral_error_roll=I_term;

	if (I_term < iMin)
	{
		I_term=iMin;
		//*previous_integral_error_roll=0;
	}
	if (I_term > iMax)
	{
		I_term=iMin;
		//*previous_integral_error_roll=0;
	}
	if(D_term > 120)
	{
		D_term=120;
	}
	if(D_term < -120)
	{
		D_term=-120;
	}

	*pid_roll = P_term+I_term+D_term;
	pid_roll_print=*pid_roll;

	print(pid_roll_print);//roll
	printString("\t");
	//printString("\r\n");
	//change rotor1&3
}
void pitchPID(float current_pitch, float *pid_pitch, float *previous_integral_error_pitch,float *previousError_pitch,float sampleTime, int8_t desired_pitch)
{
	float previous_integral_error_print=0;
	float previousError_pitch_print=0;

	previous_integral_error_print=*previous_integral_error_pitch;
	previousError_pitch_print=*previousError_pitch;

	float	 P_term=0, I_term=0, D_term=0;
	float	 error_pitch = desired_pitch - current_pitch;
	int16_t pid_pitch_print=0;

	if(sampleTime>0.2)
	{
		sampleTime=0.0035;
	}
	if(sampleTime < 0 )
	{
		sampleTime=0.0035;
	}
	P_term = pitch_kp*error_pitch; //proportional
	I_term = previous_integral_error_print+pitch_ki*(error_pitch*sampleTime);
	D_term = pitch_kd * ((error_pitch-previousError_pitch_print )/sampleTime);

	*previousError_pitch = error_pitch;
	*previous_integral_error_pitch=I_term;

	if (I_term> iMax)
	{
		I_term= iMax;
		//*previous_integral_error_pitch=0;
	}
	if (I_term < iMin)
	{
		I_term = iMin;
		//*previous_integral_error_pitch=0;
	}
	if(D_term > 250)
	{
		D_term=250;
	}
	if(D_term < -250)
	{
		D_term=-250;
	}

	*pid_pitch = P_term+I_term+D_term;
	pid_pitch_print=*pid_pitch;

	print(pid_pitch_print);//pitch
	printString("\t");
	///print(D_term);
	//printString("\r\n");
	//change rotor1&2
}
void yawPID(float current_yaw, float *pid_yaw, float *previous_integral_error_yaw,float *previousError_yaw,float sampleTime, int8_t desired_yaw)
{
	float previous_integral_error_print=0;
	float previousError_yaw_print=0;

	previous_integral_error_print=*previous_integral_error_yaw;
	previousError_yaw_print=*previousError_yaw;

	float	P_term=0, I_term=0, D_term=0;
	float	error_yaw = desired_yaw - current_yaw;
	int16_t pid_yaw_print=0;

	if(sampleTime > 0.2 )
	{
		sampleTime=0.0035;
	}
	if(sampleTime < 0 )
	{
		sampleTime=0.0035;
	}
	P_term = yaw_kp*error_yaw; //proportional
	I_term = previous_integral_error_print+yaw_ki*(error_yaw*sampleTime);
	D_term = yaw_kd*((error_yaw-previousError_yaw_print )/sampleTime);

	*previousError_yaw = error_yaw;
	*previous_integral_error_yaw=I_term;

	//Check for Integral Windup and Filter Derivative term
	if (I_term> iMax)
	{
		I_term= iMax;
	}
	if (I_term < iMin)
	{
		I_term = iMin;
	}
	if (D_term > 150)
	{
		D_term = D_term/2;
	}
	if (D_term < -150)
	{
		D_term = D_term/2;
	}
	if(D_term > 250)
	{
		D_term=250;
	}
	if(D_term < -250)
	{
		D_term=-250;
	}

	*pid_yaw = P_term+I_term+D_term;
	pid_yaw_print=*pid_yaw;

	print(pid_yaw_print);//yaw
	printString("\t");
	//print(D_term);
}
void PWM_adjustment(float pid_pitch, float pid_roll, float pid_yaw,uint16_t baseSpeed)
{
	//initilize var
	int16_t sum1=0;
	int16_t sum2=0;
	int16_t sum3=0;
	int16_t sum4=0;

	//prior to feeding the PWM registers calculaate the overall sum and check if it breaks any speed bounds
	//Yaw adjustment Should be added later or else the Quadcopter will spin!!
	sum1 = baseSpeed - pid_pitch + pid_roll + pid_yaw;
	sum2 = baseSpeed + pid_pitch + pid_roll - pid_yaw;
	sum3 = baseSpeed - pid_pitch - pid_roll - pid_yaw;
	sum4 = baseSpeed + pid_pitch - pid_roll + pid_yaw;

	//Pitch Upper Bound
	if(sum2 > (250) && sum4 > (250))
	{
		sum2=250;
		sum4=250;
	}
	else if(sum1 > (250) && sum3 > (250))
	{
		sum1=250;
		sum3=250;
	}
	//Pitch Lower Bound
	if (sum2 < (34) && sum4 < (34))
	{
		sum2=35;
		sum4=35;
	}
	else if(sum1 < (34) && sum3 < (34))
	{
		sum1=35;
		sum3=35;
	}
	//Roll Upper Bound
	if(sum1 > (250) && sum2 > (250))
		{
			sum1=250;
			sum2=250;
		}
	else if(sum3 > (250) && sum4 > (250))
	{
		sum3=250;
		sum4=250;
	}
	//Roll Lower Bound
	if (sum1 < (34) && sum2 < (34))
	{
		sum1=35;
		sum2=35;
	}
	else if(sum3 < (34) && sum4 < (34))
	{
		sum3=35;
		sum4=35;
	}
	//YAW Upper Bound
	if(sum1 > (250) && sum4 > (250))
	{
		sum1=250;
		sum4=250;
	}
	else if(sum2 > (250) && sum3 > (250))
	{
		sum2=250;
		sum3=250;
	}
	//YAW Lower Bound
	if (sum1 < (34) && sum4 < (34))
	{
		sum1=35;
		sum4=35;
	}
	else if(sum2 < (34) && sum3 < (34))
	{
		sum2=35;
		sum3=35;
	}
	printString("\t");
	print(sum1);//pitch
	printString("\t");
	print(sum2);//pitch
	printString("\t");
	print(sum3);//pitch
	printString("\t");
	print(sum4);//pitch
	printString("\t");

	//Change the speeds here:
	PWM1_1_CMPA_R =sum4;//(green)//motor4   //ccw rotor
	PWM0_0_CMPA_R =sum2;//(Yellow)//motor2  //cw rotor
	PWM0_2_CMPA_R =sum1;//(RED)//motor1     //ccw rotor
	PWM0_1_CMPA_R =sum3;//(ORANGE)//motor3  //cw rotor
}
