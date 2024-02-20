//Pin configuration on hardware
//PB0 - UART1 Rx - Bluetooth
//PB1 - UART1 Tx - Bluetooth
//

//PB6 - PWM Pin - Motor
//PB7 - pWM Pin - Motor
//PD2 - PWM Pin - Motor
//PD3 - PWM Pin - Motor
//PB3 - MD Pin Motor

//PE2 - ADC Pin - Sensor IR (Right)
//PE3 - ADC PIN -SENSOR IR (FRONT)

//PC6 - Reflectance sensor

//TIMER 1A - REFLECTANCE SENSOR
//TIMER 2A - PINGPONG TRANSMIT
//TIMER3A - PID

//WTIMER 1B - Flashing RED
//WTIMER 2B - Shutdown
//WTIMER 3B - Time collector

/*
 *  ======== empty_min.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/PWM.h"
#include "utils/uartstdio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"

// Define command strings for motor control
volatile uint32_t pwmMax = 0;
char myCommand[3];
uint8_t currentIndex = 0;

//Enable Semaphore handle
extern const ti_sysbios_knl_Semaphore_Handle PIDSemaphore;
extern const ti_sysbios_knl_Semaphore_Handle PingSemaphore;
extern const ti_sysbios_knl_Semaphore_Handle PongSemaphore;

//HEader Declaration
void SteerRigh(float steerValue);
void SteerLeft(float steerValue);

//Hardware settings
#define PERIOD 2000
#define PWM_DIVIDER 64
#define PWM_MAX (PWMClockGet(PWM0_BASE)/ 13000 - 1)
#define MAXSTEERING 99
#define MAXSPEED 100
#define desiredDistance 1600

#define COMMAND_run "RN"
#define COMMAND_stop "SP"
#define COMMAND_back "BK"
#define COMMAND_uturn "UT"
#define COMMAND_right "RT"

#define UART_BUFFER_SIZE 3
#define UART_BASE UART1_BASE

//Motor control state
enum MotorState
{
    RUN, STOP, BACKWARD, RIGHT, U_TURN
};
enum MotorState motorstate = STOP;

//Define Ping Pong buffers
#define DATA_BUFFER_SIZE 20
#define NUM_BUFFERS 2
#define BAUD_RATE 2400
#define FRAME 7

uint16_t *currentBuffer;
uint32_t bufferCount = 0;
uint16_t PING[DATA_BUFFER_SIZE];
uint16_t PONG[DATA_BUFFER_SIZE];

//Define ADC channels
volatile uint32_t ADCValues[4];
#define ADC_FRONT_SENSOR 0
#define ADC_RIGHT_SENSOR 1
#define DESIRED_ADC_VALUE 2048

//PID control variables
volatile float integralPrev = 0.0;
volatile float prevError = 0.0;

//PWM signal IDS
#define PWM_LEFT_MOTOR PWM_OUT_0
#define PWM_RIGHT_MOTOR PWM_OUT_1

//Motor speed variables
float leftSpeed = 0.0;
float rightSpeed = 0.0;

//Initialize time running counter
uint64_t start = 0;
uint64_t end = 0;

//Initializations
void SteerRight(float steerValue)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (int) (pwmMax * MAXSPEED * 0.01)); //Left wheel
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                     (int) (pwmMax * (MAXSPEED - steerValue) * 0.01)); //Right wheel
}

void SteerLeft(float steerValue)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                     (int) (pwmMax * (MAXSPEED - steerValue) * 0.01)); //Left wheel
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (int) (pwmMax * MAXSPEED * 0.01)); //Right wheel
}

//Initizalize UART0
void UART0Init()
{
    // Enable the UART1 and GPIOA peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)
            || SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }
    // Configure GPIO pins for UART0 operation
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART1 module (baud rate: 115200, 8N1)
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}
// Function to send a string via Bluetooth (UART1)
void UART1_SendString(const char *str)
{
    while (*str)
    {
        UARTCharPut(UART1_BASE, *str);
        str++;
    }
}

void UART1_GetString(char *buffer, uint32_t length)
{
    uint32_t i = 0;
    char receivedChar;

    while (i < length - 1)
    {
        // Wait for a character to be received
        while (!UARTCharsAvail(UART1_BASE))
            ;  // Use UART1_BASE here

        // Read the character
        receivedChar = UARTCharGet(UART1_BASE);  // Use UART1_BASE here

        // Store the character in the buffer
        buffer[i] = receivedChar;
        i++;

        // Terminate the string if a newline or carriage return is received
        if (receivedChar == '\n' || receivedChar == '\r')
        {
            buffer[i] = '\0';
            break;
        }
    }
}

//Initalize PWM Function
void PWM_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);

    // PB6 Left wheel pwm
    // PB7 Right wheel pwm
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    pwmMax = SysCtlClockGet() / 64 / 13000; //pwmMax = (1/100)*(40000000 /64) pwmMax = (SystemClock/PWMDivisior)/DesiredFrequency

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmMax);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (uint32_t) ((float) pwmMax * .5));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (uint32_t) ((float) pwmMax * .5));

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

// Function to initialize the ADC module
// Sensors
void ADC_Init(void)
{
    // Enable the clock to PortD and ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Configure the GPIO for analog input (e.g., AIN0)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); //RIGHT SENSOR
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // FRONT SENSOR

    // Configure the ADC0 module
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1,
    ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
}

// Read the analog value from the sensor
uint32_t ADC_Read()
{

    ADCProcessorTrigger(ADC0_BASE, 2); // Trigger the conversion
    while (!ADCIntStatus(ADC0_BASE, 2, false))
    {
    }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t*) ADCValues);

    return ADCValues[0];

}

/*
 *  ======== main ========
 */
int main()
{

    memset(myCommand, 0, 3);
    IntMasterEnable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    // Set the system clock to 40MHz (assuming a 16MHz external crystal)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable the UART1 and GPIOB peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Enable the GPIOF peripheral for the LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Initialize PWM
    PWM_Init();

    // Configure the UART1 pins (PB0 as U1RX and PB1 as U1TX)
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART1 module
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStdioConfig(1, 115200, SysCtlClockGet());

    // Enable UART1
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    UARTFIFOEnable(UART1_BASE);
    UARTIntEnable(UART1_BASE, UART_INT_RX);
    IntEnable(INT_UART1);

    UARTEnable(UART1_BASE);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

//***************************************************************//
    //Enable timer 3A peripheral(PID)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    //Configure the timer to 1Hz operation
    TimerConfigure(TIMER3_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / 1000 - 1);

    TimerPrescaleSet(TIMER3_BASE, TIMER_A, 50 - 1);
    IntEnable(INT_TIMER3A);

    //Enable timer interrupt
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    //Enable the timer
    TimerEnable(TIMER3_BASE, TIMER_A);

//***************************************************************//
    //TIMER 1A FOR LIGHT SENSOR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //Enable timer peripheral(timer 1A)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //Configure the timer to 1Hz operation
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 1000 - 1);

    TimerPrescaleSet(TIMER1_BASE, TIMER_A, 1 - 1);
    IntEnable(INT_TIMER1A);

    //Enable timer interrupt
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //Enable the timer
    TimerEnable(TIMER1_BASE, TIMER_A);

//***************************************************************//
    //Enable timer peripheral(TIMER 2A) for PING PONG
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    TimerConfigure(TIMER2_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() / 1000 - 1); // For a 100ms interval

    TimerPrescaleSet(TIMER2_BASE, TIMER_A, 100 - 1);

    IntEnable(INT_TIMER2A);

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //TimerEnable(TIMER2_BASE, TIMER_A);

//******************************************************************//
    //TIMER 1B FOR FLASHING RED

    //Enable timer peripheral(timer 1B)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

    //Configure the timer to 1Hz operation
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    TimerLoadSet(WTIMER1_BASE, TIMER_B, SysCtlClockGet() / 1000 * 200 - 1);

    IntEnable(INT_TIMER1B);

    //Enable timer interrupt
    //TimerIntEnable(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);

    //Enable the timer
    //TimerEnable(TIMER1_BASE, TIMER_B);

//******************************************************************//
    //Enable timer peripheral(TIMER 2B) for shutdown
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);

    TimerConfigure(WTIMER2_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    TimerLoadSet(WTIMER2_BASE, TIMER_B, SysCtlClockGet() * 60 - 1); // For a 100ms interval

    IntEnable(INT_TIMER2B);

    TimerIntEnable(WTIMER2_BASE, TIMER_TIMB_TIMEOUT);

//******************************************************************//
    //Enable timer 3B peripheral(RUNNING TIME COLLECTOR)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);

    //Configure the timer to 1Hz operation
    TimerConfigure(WTIMER3_BASE, TIMER_CFG_B_PERIODIC);

    TimerLoadSet64(WTIMER3_BASE, 18446744073709551615);

    //Enable the timer
    TimerEnable(WTIMER3_BASE, TIMER_BOTH);

    TimerValueGet64(WTIMER3_BASE);
//******************************************************************//
    ADC_Init();

    BIOS_start();

    return (0);
}

void UART1_ISR()
{
    //clear ISR flag
    uint32_t ui32Status = UARTIntStatus(UART1_BASE, true); // Get the interrupt status.

    //Clear the interrupt
    UARTIntClear(UART1_BASE, ui32Status); // Clear the asserted interrupts.
    //if(UARTCharsAvail(UART1_BASE)){
    myCommand[0] = UARTCharGet(UART1_BASE);
    myCommand[1] = UARTCharGet(UART1_BASE);

    if (strncmp(myCommand, "SP", 3) == 0)
    {
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
        motorstate = STOP;
    }
    else if (strncmp(myCommand, "RN", 3) == 0)
    {
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
        int LeftDutyCycle = 100;
        int RightDutyCycle = 100;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, LeftDutyCycle * pwmMax);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);
        motorstate = RUN;
        start = TimerValueGet64(WTIMER3_BASE);
    }
    else if (strncmp(myCommand, "BK", 3) == 0)
    {
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
        int LeftDutyCycle = -50;
        int RightDutyCycle = -50;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, LeftDutyCycle * pwmMax);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3,
        GPIO_PIN_2 | GPIO_PIN_3);
        motorstate = BACKWARD;
    }

    currentIndex = (currentIndex + 1) % 2;
}

//PID TIMER
void Timer3A_ISR()
{
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(PIDSemaphore);
}

bool blackFlag = false;
uint32_t timeCount = 0;
int tapeFound = 0;

//REFLECTANCE SENSOR
void Timer1A_ISR()
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Disable the timer
    TimerDisable(TIMER1_BASE, TIMER_A);

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);

    // Delay for 10 microseconds
    SysCtlDelay(SysCtlClockGet() / 3 / 1000000 * 10);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);
    uint32_t count = 0;

    // Wait for the sensor pin to go low
    while (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6))
    {
        // Wait for the sensor to discharge
        count++;

    }
    //UARTprintf("%i\n", count);

    if (count >= 2800 && blackFlag == false)
    {
        blackFlag = true;
    }
    else if (count >= 2800 && blackFlag == true)
    {
        timeCount++;
    }
    else if (count < 2800 && blackFlag == true)
    {
        UARTprintf("%i\n", timeCount);
        if (timeCount >= 3)
        {
            if (timeCount < 20)
            {
                tapeFound++;
                if (tapeFound == 1)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // GREEN LED
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 , GPIO_PIN_2 );
                    //begin PID collection
                    TimerEnable(TIMER2_BASE, TIMER_A);
                }
                else if (tapeFound == 2)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0 );

                    GPIOPinWrite(GPIO_PORTF_BASE,
                                 GPIO_PIN_1, GPIO_PIN_1); // Turn off other LEDs
                    // Stop PID Collection
                    TimerDisable(TIMER2_BASE, TIMER_A);
                    // Transmit remaining data
                    Semaphore_post(PongSemaphore);

                }
            }

            //FINISH LINE
            else if (timeCount >= 20)
            {
                end = TimerValueGet64(WTIMER3_BASE);

                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0); // Turn off other LEDs
                // Stop Data Collection
                TimerDisable(TIMER2_BASE, TIMER_A);
                // Turn on Flashing LED
                TimerEnable(WTIMER1_BASE, TIMER_B);
                // Timer for 60 seconds
                TimerEnable(WTIMER2_BASE, TIMER_B);
                // Stop reflectance
                TimerDisable(TIMER1_BASE, TIMER_A);
                // Disable PID
                TimerDisable(TIMER3_BASE, TIMER_A);


                // Stop the robot
                PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
                motorstate = STOP;
                uint32_t timeToRun = (float) (start - end) / SysCtlClockGet()* 1000;
                UARTprintf("\n%i\n", timeToRun);
                int i;
                for(i = 0; i < 60; i++){
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); //Turn off red
                    SysCtlDelay((SysCtlClockGet() / 6));
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); //Turn on red
                    SysCtlDelay((SysCtlClockGet() / 6));

                }
                return;
            }
        }
        timeCount = 0;
        blackFlag = false;
    }

    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 1000);
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, 1 - 1);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

//PING PONGG
void Timer2A_ISR() //DATA COLLECTION
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(PingSemaphore);
}

uint32_t currentDistance = 0;
uint32_t currentFrontDistance = 0;
void PID_Task_Function()
{
    while (1)
    {
        Semaphore_pend(PIDSemaphore, BIOS_WAIT_FOREVER);

        //Read the sensor value
        ADC_Read();
        currentDistance = ADCValues[0];
        currentFrontDistance = ADCValues[1];

        float Kp = 2.0; //Proportional gain
        float Ki = 0.1; //Integral gain
        float Kd = 0.08; //Derivative gain
        float integral = 0.0;
        float deltaTime = 0.05; // 50ms time interval between control loops
        float error = (desiredDistance - (float) currentDistance)
                / desiredDistance;

        //Calculate Kp, Ki and Kd
        float proportional = Kp * error;
        integral = Ki * (integral + error * deltaTime);
        float derivative = Kd * (error - prevError) / deltaTime;

        //Calculate the control output (steering value)

        float steerValue = proportional;

        if (steerValue > .9)
        {
            steerValue = .9;
        }
        else if (steerValue < -.9)
        {
            steerValue = -.9;
        }
        //UTURN
        if (currentFrontDistance >= 2300)
        {
            //Disable reflectance sensor
            TimerDisable(TIMER1_BASE, TIMER_A);
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // left wheel backwards
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (int) (pwmMax)); //Left wheel max speed
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (int) (pwmMax)); //Right wheel max speed

            while (currentFrontDistance > 1400)
            {
                // delay for 10ms
                SysCtlDelay((SysCtlClockGet() / 3 / 1000) * 10); //SysCtlClockGet()/3 = 1s
                // update front sensor value
                ADC_Read();
                currentFrontDistance = ADCValues[1];
            }
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);
            continue;
        }

        if (steerValue < 0)
        {                //steer left
            float absSteering = 1 - fabs(steerValue);

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                             (int) (pwmMax * absSteering)); //Left wheel
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (int) (pwmMax)); //Right wheel
        }
        else
        {
            float absSteering = 1 - fabs(steerValue);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                             (int) (pwmMax * absSteering)); //Right wheel
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (int) (pwmMax)); //Left wheel
        }

        prevError = error;

        TimerEnable(TIMER1_BASE, TIMER_A); //Enable reflectance sensor again
    }
}

void Ping_Task_Function()  //DATA COLLECTION BUFFER
{
    while (1)
    {
        Semaphore_pend(PingSemaphore, BIOS_WAIT_FOREVER);
        ADC_Read();
        currentDistance = ADCValues[0];
        currentBuffer[bufferCount] = abs(desiredDistance - currentDistance);
        bufferCount++;

        if (bufferCount >= 20)
        {
            Semaphore_post(PongSemaphore);

        }
    }
}

void Pong_Task_Function()
{

    while (1)
    {
        Semaphore_pend(PongSemaphore, BIOS_WAIT_FOREVER);
        uint16_t *localBuffer = currentBuffer;
        uint16_t localIndex = bufferCount;
        // Reset and swap data buffers
        currentBuffer = (currentBuffer == PING) ? PONG : PING;
        if (tapeFound > 1)
        {
            // Turn off all LEDs
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                         0);
        }
        else if (currentBuffer == PING)
        {
            // Turn on blue
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            // Turn off red
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        }
        else
        {
            // Turn off blue
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            // Turn on red
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        }
        bufferCount = 0;
        UARTprintf(":5");
        int i;
        for (i = 0; i < localIndex; i++)
        {
            char upper = localBuffer[i] >> 8;
            char lower = localBuffer[i] & 0x0F;
            UARTCharPut(UART1_BASE, upper);
            UARTCharPut(UART1_BASE, lower);
        }
        UARTprintf("r\n");
    }
}

void WTimer1B_ISR()
{  //Flash the RED LED for a minute
// Clear the timer interrupt
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);

    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);  // Turn off the red LED
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // Turn on the red LED
    }
}

void WTimer2B_ISR()
{ //Stop the robot if RED flash for a minute
// Clear the timer interrupt
    TimerIntClear(WTIMER2_BASE, TIMER_TIMB_TIMEOUT);

    // Disable Timer 2B
    TimerDisable(TIMER2_BASE, TIMER_B);
}

void WTimer3B_ISR()
{ //Calculate the time running
// Clear the timer interrupt
    TimerIntClear(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

    //Disable Timer 3B
    TimerDisable(TIMER3_BASE, TIMER_B);

}
