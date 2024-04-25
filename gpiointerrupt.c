#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>
#include "ti_drivers_config.h"

// Global variables to track button presses
int tempDown = 0;
int tempUp = 0;


void gpioButtonFxn0(uint_least8_t index)   // Copied From <Thermostat Lab Guide>
{
    tempDown = 1;
}

void gpioButtonFxn1(uint_least8_t index)    // Copied From <Thermostat Lab Guide>
{
    tempUp = 1;
}

// UART Global Variables
char output[64];
UART_Handle uart;

#define DISPLAY(x) UART_write(uart, &output, x); // Copied From <Thermostat Lab Guide>

//Method (initUART) Copied From <Thermostat Lab Guide>
void initUART(void) {
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

//Copied From <Thermostat Lab Guide>
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

//Copied From <Thermostat Lab Guide>
sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];

I2C_Transaction i2cTransaction;

I2C_Handle i2c;
UART_Handle uart;
Timer_Handle timer0;


// Function prototypes Copied From <Thermostat Lab Guide>
void initI2C(void);
void initUART(void);
void initTimer(void);
void timerCallback(Timer_Handle myHandle, int_fast16_t status);
int16_t readTemp(void);

//Method Copied From <Thermostat Lab Guide>
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    };

    if (found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,"
                                     "contact professor\n\r"));
    }
}

//Method Copied From <Thermostat Lab Guide>
int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r"
                                     ,i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging "
                                     "USB and plugging back in.\n\r"));
    }

    return temperature;
}


//Copied From <Thermostat Lab Guide>
char TimerFlag = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

//Method Copied From <Thermostat Lab Guide>
void initTimer(void)
{
 //Timer_Handle timer0;
 Timer_Params params;
 Timer_init();

 // Configure the driver
 Timer_Params_init(&params);
 params.period = 1000000;
 params.periodUnits = Timer_PERIOD_US;
 params.timerMode = Timer_CONTINUOUS_CALLBACK;
 params.timerCallback = timerCallback;

 // Open the driver
 timer0 = Timer_open(CONFIG_TIMER_0, &params);

 if (timer0 == NULL) {
     /* Failed to initialized timer */
     while (1) {}
  }

  if (Timer_start(timer0) == Timer_STATUS_ERROR) {
          /* Failed to start timer */
      while (1) {}
 }
}

void *mainThread(void *arg0){

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART();
    initI2C();
    initTimer();

    /*Incremented based on a timer. These states control the execution of the button pressed.
    Transitions between these states occur based on time*/

    // Define timing constants
    int checkTemp = 500;
    int displayCheck = 1000;
    int time = 200;

    // Initialize state variables
    int tempCheckTime = 0;
    int displayCheckTime = 0;

    // for output statement seen in the <Thermostat Lab Guide>
    int temperature = 0;
    int setpoint = 20;
    int heat = 0;
    int seconds = 0;


#define BUTTON_UP 1
#define BUTTON_DOWN 2
#define BUTTON_NONE 0

int buttonState = BUTTON_NONE;


while (1)
{
    temperature = readTemp();

    // Check if button is pressed
    if (tempDown)
    {
        buttonState = BUTTON_DOWN;
        tempDown = 0;
    }
    else if (tempUp)
    {
        buttonState = BUTTON_UP;
        tempUp = 0;
    }
    else
    {
        buttonState = BUTTON_NONE;
    }

    switch (buttonState)
    {
        case BUTTON_UP:
            setpoint++;
            DISPLAY(printf("<Temp Increased>\n\r"));
            break;
        case BUTTON_DOWN:
            setpoint--;
            DISPLAY(printf("<Temp Decreased>\n\r"));
            break;
        default:
            break;
    }

    if (tempCheckTime >= checkTemp)
    {
        if (temperature < setpoint)
        {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);  // Turn on heating
            heat = 1;
        }
        else
        {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);   // Turn off heating
            heat = 0;
        }
    }

    // Update display
    if (displayCheckTime >= displayCheck)
    {
        DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds));  //Copied From <Thermostat Lab Guide>
        seconds++;
    }

    while (!TimerFlag){}
    TimerFlag = 0;
    int buttonCheckTime = 0;
    int buttonFlags = 200;

    buttonCheckTime = tempCheckTime = displayCheckTime += time;
    }
}
