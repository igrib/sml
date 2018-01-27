/*******************************MACROS*****************************************/
#define p(x) Serial.println(x)
#define pp(x, y)                                                               \
         Serial.print(x);                                                      \
         Serial.println(y)
			 
#define HALL_SENSOR_PIN 0   // A0
#define CURRENT_SENSOR_PIN 1 //A1
#define EM_PWM_PIN 11 //Arduino ~10; ATMEGA328 PB3-OC2A

/******************************************************************************/
void setup()
{

	//Set up pin directions
	pinMode(HALL_SENSOR_PIN, INPUT);
    pinMode(CURRENT_SENSOR_PIN, INPUT);
 	pinMode(EM_PWM_PIN, OUTPUT);
	analogReference(DEFAULT); // Use 5V as reference for A2D
	

	Serial.begin(115200);
	
	initTimerInterrupt();
	initPWM();
	setPWM(0); //Turn off current to EM
}

void loop() 
{
	parser(); 
}
void initPWM()
{
	/***********************************************
	*void initPWM()
	*Input: none
	*Output: none
	*Description: Set up PWM to use our pin and timer
	*			  
	***********************************************/
	p("Setting up PWM");
	pinMode(EM_PWM_PIN, OUTPUT); 
    TCCR2A = 0;
	TCCR2A = (1<<7)|(1<<6)|(1<<1)|(1<<0); ////Fast PWM; Set OC2A on compare match, Clear OC2A at BOTTOM; 

    TCCR2B = 0;
	TCCR2B = (1<<2); //Waveform generation - Fast PWM;
    OCR2A = 0;

	


}
void initTimerInterrupt()
{
	
	/***********************************************
	*void initTimerInterrupt()
	*Input: none
	*Output: none
	*Description: Set up timer to overflow every
	*			  2.5mS and trip interrupt
	*			  Clock Frequency = 16Mhz
	*             
	***********************************************/
	p("Setting up Timer Interrupts");
	cli();
	TCCR1A = 0x0; //Normal port operation, OC1A/OC1B disconnected
	TCCR1B = 0b00001001; //CTC mode; //No Prescaling
	TCCR1C = 0;
	OCR1A  = 40000;
	TIMSK1=0b00000010; //OCIEA: Output compare A Match Interrupt Enable

	sei();
}
ISR(TIMER1_COMPA_vect)
{
	/***********************************************
	*ISR(TIMER1_COMPA_vect)
	*Intput:none
	*Output:none
	*Description: Called every time TCNT1
	*			  matches ORCA1 in our
	*			  case it is every 10mS
	*             
	***********************************************/

	
}


void parser()
{
    p("Select Command");
	p("[d] - Set PWM Duty Cycle");
	p("[c] - Continuously read current sensor");
    char ch = Serial.read();

    while (ch == -1)
    {
    	ch = Serial.read();
    }
	
    pp("Read: ", ch);
    switch (ch)
    {
		case ('d'):
        {
			setDutyCycle();
            break;
        }
		case ('c'):
		{
			readCurrentSensor();
			break;
		}
		
        default:
        {
        	p("Bad command");
            break;
        }
    }
}
void readCurrentSensor()
{
	char ch;
    while (ch != 'e')
    {
		
		int currentValue = analogRead(CURRENT_SENSOR_PIN);
		pp("Current Value: ",currentValue);
        if (Serial.available())
        {
             ch = Serial.read();
        }
	 }
}
int readHallSensor()
{
/*
 * unsigned int readHallSensor()
 * Input: None
 * Output: unsigned int (16 bit 0-65535) value of the hall sensor
 * Reads the hall sensor A2D and averages a few values to clean up noise
 */
#define AVERAGE_SIZE 10
         double reading[AVERAGE_SIZE];
         double average = 0;

         for (uint8_t i = 0; i < AVERAGE_SIZE; i++)
         {
                  // ADC Conversion complete
                  reading[i] = analogRead(HALL_SENSOR_PIN);
                  average += reading[i];
                  // delay(1);
         }
         average = average / AVERAGE_SIZE;
         return (int)average;
}
void setDutyCycle()
{
	/***********************************************
	* void setDutyCucle()
	* Input: None
	* Output: None
	* Description: 
		* Called by the parse to take in user input
	    * and set the duty cycle on the PWM pin
	***********************************************/
	
	int val;
    p("Enter Duty Cycle [0-255]");
    while (!Serial.available())
    {
             ;
    }
    val = Serial.parseInt();

    pp("Read in:", val);
	setPWM(val);
}
void setPWM(int val)
{
	/***********************************************
	* void setPWM(int val)
	* Input: int val 0-255 which represents 0-100% duty cycle
	* Output: None
	* Description: 
		* We subtract 255 to flip what is 0% and what is 100%
		* This removes the glitch burst you get when pwm is set to 0.
		* This does add the glitch to when PWM is 100% but we don't mind as much.
	***********************************************/
	
	OCR2A=255-val;
}