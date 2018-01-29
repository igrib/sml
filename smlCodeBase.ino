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
	*			  case it is every 2.5mS
	*             
	***********************************************/
	//printSensors();	
}
void parser()
{
	/***********************************************
	*void parser()
	*Intput:none
	*Output:none
	*Description: User command parser
	*             
	***********************************************/
	
    p("Select Command");
	p("[d] - Set PWM Duty Cycle");
	p("[c] - Continuously read hall and current sensor");
	p("[q] - Characterize pwm vs hall vs current sensor");
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
			readSensors();
			break;
		}
		case ('q'):
		{
			characterizeSensors();
			break;
		}
        default:
        {
        	p("Bad command");
            break;
        }
    }
}
void characterizeSensors()
{
	/***********************************************
	*void characterizeSensors()
	*Intput:none
	*Output:none
	*Description: Step through PWM values from 0-255
	*			  read hall and  current sensors
	*			  output all 3 values
	* 			              
	***********************************************/

	p("Timestamp\tPWM\tCurrent\tHall\tNormalized Hall");
	for(int pwmValue=0;pwmValue<255;pwmValue++)
    {		
		long timeStamp=millis();
		setPWM(pwmValue);
		int currentValue = analogRead(CURRENT_SENSOR_PIN);
		int hallValue = analogRead(HALL_SENSOR_PIN);
		int normalizedHallValue=readHallSensorNormalized();
		Serial.print(timeStamp);
		Serial.print("\t");
		Serial.print(pwmValue);
		Serial.print("\t");
		Serial.print(currentValue);
		Serial.print("\t");
		Serial.print(hallValue);
		Serial.print("\t");
		Serial.println(normalizedHallValue);		
	}
	setPWM(0);
	
	//TODO Write characterize fucntion to calculate m and b to normalize hall sensor reading
}
void printSensors()
{
	/***********************************************
	*void printSensors()
	*Intput:none
	*Output:none
	*Description: Print sensor reading to screen
	* 			              
	***********************************************/
	long timeStamp=millis();
	int currentValue = analogRead(CURRENT_SENSOR_PIN);
	int hallValue = analogRead(HALL_SENSOR_PIN);
	int normalizedHallValue=readHallSensorNormalized();
	Serial.print(timeStamp);
	Serial.print("\t");
	Serial.print(currentValue);
	Serial.print("\t");
	Serial.println(hallValue);
	Serial.print("\t");
	Serial.println(normalizedHallValue);
}
void readSensors()
{
	char ch;
	
	p("Current\tHall\tHall Normalized");
    while (ch != 'e')
    {
			
		int currentValue = readCurrentSensor();
		int hallValue = readHallSensorRaw();
		int hallValueNormalized=readHallSensorNormalized();
		Serial.print(currentValue);
		Serial.print("\t");
		Serial.print(hallValue);
		Serial.print("\t");		
		Serial.println(hallValueNormalized);
        if (Serial.available())
        {
             ch = Serial.read();
        }
	 }
}
int readCurrentSensor()
{
	return analogRead(CURRENT_SENSOR_PIN);

}
int readHallSensorRaw()
{
	/*
	 * unsigned int readHallSensorRaw()
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
      average += analogRead(HALL_SENSOR_PIN);
    }
    average = average / AVERAGE_SIZE;
    return (int)average;
}
int readHallSensorNormalized()
{
   /*
    * unsigned int readHallSensorNormalized()
    * Input: None
    * Output: unsigned int (16 bit 0-65535) value of the hall sensor normalized
    * Takes the raw reading of the hall sensor and removes the effect
	* of the electromagnet. 
	* Calculate the impact of the electromagnet based on the current sensor
	* and subtract it out.
    */	
	
	
	//These aret he values for our lines
	float m=0.297; //Slope of the current vs hall
	float b=487; //Intercept of the current vs hall
	
	float currentValue=(float)readCurrentSensor();
	float rawHallValue=(float)readHallSensorRaw();
	float normalizedValue = ( m * currentValue+b); //Calculate what hall sensor should be based on current sensor
	
	normalizedValue=rawHallValue-normalizedValue; 
	
	// Serial.print("\t");
	// Serial.print(rawHallValue);
	// Serial.print("\t");
	// Serial.print(currentValue);
	// Serial.print("\t");
	// Serial.print(normalizedValue);
	
	
	
	//Check that we are in reasonable bounds and if not flatten
	if(normalizedValue<0)
		normalizedValue = 0;
	if(normalizedValue>1024)
		normalizedValue=1024;
	
	return (int)normalizedValue;
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