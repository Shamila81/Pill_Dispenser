#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

#define SW_1 8
#define SW_2 7
#define LED_1 20
#define LED_2 21
#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define OPTO_FORK 28
#define PIEZO_SENSOR 27
#define STEP_DELAY_MS 2

// EEPROM Constants
#define EEPROM_I2C_ADDRESS 0x50
#define EEPROM_MAX_ADDRESS 0x7FFF
#define EEPROM_CALIBRATION_STATE_ADDR 0x0004


// UART Configurations
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_TIMEOUT 5000
#define BUFFER_SIZE 128

// Global Variables
#define UART_ID uart1
#define BAUD_RATE 9600
#define MAX_RETRY_ATTEMPTS 5
void readLoRaResponse();
void run_motor_to_position(int target_position);
void calibrate_motor();
void activate_motor_step();

// I2C Configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17
#define I2C_SPEED 100000

bool sensor_triggered = false;
char lora_response[BUFFER_SIZE];

// Variables to store state from EEPROM
int motor_position = 0;
int motor_state = 0;  // 0 = idle, 1 = turning
bool power_loss_detected = false;

void initI2C() {
    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}
void eeprom_write(uint16_t address, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = (address >> 8) & 0xFF;  // High byte of the address
    buffer[1] = address & 0xFF;         // Low byte of the address
    buffer[2] = data;                   // Data to be written

    i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDRESS, buffer, 3, false);
    sleep_ms(5);  // EEPROM write cycle time
}

uint8_t eeprom_read(uint16_t address) {
    uint8_t buffer[2];
    buffer[0] = (address >> 8) & 0xFF;  // High byte of the address
    buffer[1] = address & 0xFF;         // Low byte of the address

    i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDRESS, buffer, 2, true);
    uint8_t data;
    i2c_read_blocking(I2C_PORT, EEPROM_I2C_ADDRESS, &data, 1, false);
    return data;
}

void store_motor_state() {
    eeprom_write(EEPROM_CALIBRATION_STATE_ADDR, motor_position); // Store motor position
    eeprom_write(EEPROM_CALIBRATION_STATE_ADDR + 1, motor_state); // Store motor state
    printf("Motor state stored: Position = %d, State = %d\n", motor_position, motor_state);
}

void restore_motor_state() {
    motor_position = eeprom_read(EEPROM_CALIBRATION_STATE_ADDR);  // Load motor position
    motor_state = eeprom_read(EEPROM_CALIBRATION_STATE_ADDR + 1); // Load motor state
    printf("Motor state restored: Position = %d, State = %d\n", motor_position, motor_state);

    // If the motor was turning before the power loss, resume the turn
    if (motor_state == 1) {
        printf("Power loss detected, resuming turn from position %d...\n", motor_position);

        run_motor_to_position(motor_position);
    } else {
        printf("Motor is idle. Start with calibration...\n");
    }
}


void run_motor_to_position(int target_position) {
    // Logic to move the motor to the saved position (if the motor was turning)
    while (motor_position != target_position) {
        activate_motor_step();  // Move the motor by one step
        if (motor_position < target_position) {
            motor_position++;
        } else {
            motor_position--;
        }
        sleep_ms(STEP_DELAY_MS);
    }
    printf("Motor successfully moved to position %d\n", target_position);
}


void initLoRa() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);

    uart_puts(UART_ID, "AT+MODE=LWOTAA\r\n");
    sleep_ms(500);
    readLoRaResponse();


    uart_puts(UART_ID, "AT+KEY=APPKEY,\"8a123a6dcbf0ad49daddee1b5139370e\"\r\n");
    sleep_ms(500);
    readLoRaResponse();

    uart_puts(UART_ID, "AT+CLASS=A\n");
    sleep_ms(500);
    readLoRaResponse();

    uart_puts(UART_ID, "AT+PORT=8\n");
    sleep_ms(500);
    readLoRaResponse();

}

// Join LoRa Network
bool joinLoRa() {
    uart_puts(UART_ID, "AT+JOIN\n");
    printf("Joining LoRa network...\n");

    // Define a timeout (e.g., 30 seconds)
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = 30000; // 30 seconds timeout
    bool result = false;

    while (strstr(lora_response, "+JOIN: Done") == NULL) {
        readLoRaResponse();

        if (strstr(lora_response, "+JOIN: NetID") == NULL) {
            result = true;
        }

        // Check if timeout has occurred
        if (to_ms_since_boot(get_absolute_time()) - start_time > timeout_ms) {
            printf("Timeout while waiting for LoRa join response.\n");
            return false;
        }
    }

    // Check if the join was successful
    if (result) {
        printf("Successfully joined LoRa network.\n");
        return true;
    } else {
        printf("Failed to join LoRa network. Response: %s\n", lora_response);
        return false;
    }
}

// Read UART Line with Timeout
int uart_read_line_with_timeout(uart_inst_t *uart, char *buffer, size_t len, uint32_t timeout_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    size_t index = 0;

    while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
        if (uart_is_readable(uart)) {
            char c = uart_getc(uart);
            if (c == '\n' || index >= len - 1) {
                buffer[index] = '\0';
                return index;
            }
            buffer[index++] = c;
        }
    }

    buffer[0] = '\0';
    return -1; // Timeout
}

//// Read LoRa Response
void readLoRaResponse() {
    if (uart_is_readable_within_us(UART_ID, UART_TIMEOUT * 1000)) {
        int len = uart_read_line_with_timeout(UART_ID, lora_response, BUFFER_SIZE, UART_TIMEOUT);
        if (len > 0) {
            printf("LoRa Response: %s\n", lora_response);
        } else {
            printf("No response from LoRa module.\n");
        }
    }
}

// Function to send a LoRa message
void sendLoRaMessage(const char* message) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = 30000; // 30 seconds timeout
    bool result = false;
    bool timeout = false;
    char command[BUFFER_SIZE];
    snprintf(command, BUFFER_SIZE, "AT+MSG=\"%s\"\r\n", message);

    for (int i = 0; i < 3 && !result; ++i) {
        timeout = false;
        uart_puts(UART_ID, command);
        //sleep_ms(500);
        while (strstr(lora_response, "+MSG: Done") == NULL && !timeout) {
            readLoRaResponse();

            if (strstr(lora_response, "+JOIN: FPENDING") == NULL) {
                result = true;
            }

            // Check if timeout has occurred
            if (to_ms_since_boot(get_absolute_time()) - start_time > timeout_ms) {
                printf("Timeout while waiting for LoRa join response.\n");
                //return false;
                timeout = true;
            }
        }
    }
}



void SetMotor() {
    gpio_init(IN1);
    gpio_init(IN2);
    gpio_init(IN3);
    gpio_init(IN4);

    gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_set_dir(IN4, GPIO_OUT);
}

void SetOptoFork() {
    gpio_init(OPTO_FORK);
    gpio_set_dir(OPTO_FORK, GPIO_IN);
    gpio_pull_up(OPTO_FORK);
}

void SetPiezoSensor() {
    gpio_init(PIEZO_SENSOR);
    gpio_set_dir(PIEZO_SENSOR, GPIO_IN);
    gpio_pull_up(PIEZO_SENSOR);
}

void piezo_interrupt(uint gpio, uint32_t events) {
    sensor_triggered = true;
   // printf("Sensor triggered: %d\n", sensor_triggered);
}

const int half_step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

void activate_motor_step() {
    static int step_index = 0;
    step_index = (step_index + 1) % 8;

    gpio_put(IN1, half_step_sequence[step_index][0]);
    gpio_put(IN2, half_step_sequence[step_index][1]);
    gpio_put(IN3, half_step_sequence[step_index][2]);
    gpio_put(IN4, half_step_sequence[step_index][3]);
}

void calibrate_motor() {
    int count = 0;

    while (gpio_get(OPTO_FORK)) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
        count++;
    }

    while (!gpio_get(OPTO_FORK)) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
        count++;
    }

    while (gpio_get(OPTO_FORK)) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
        count++;
    }

    printf("Calibration complete: %d \n", count);

}

void align_motor() {

    int alignment_steps = 150;
    for (int i = 0; i < alignment_steps; i++) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
    }
    //printf("Motor aligned to the drop tube.\n");
}

void run_motor(int N) {
    for (int i = 0; i < N; i++) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
    }
}

void IdleMode() {
    gpio_put(LED_1, 1);
    sleep_ms(200);
    gpio_put(LED_1, 0);
    sleep_ms(200);
}

void DispensePills() {
    int step_and_stop = 0;
    int pills_dispensed = 0;

    while (step_and_stop < 7) {
        sensor_triggered = false;
        run_motor(512);
        absolute_time_t time = make_timeout_time_ms(500);
        while (!sensor_triggered && !time_reached(time)) {
            tight_loop_contents();
        }

        //printf("Sensor triggered before check: %d\n", sensor_triggered);

        if (sensor_triggered) {
            pills_dispensed++;
            printf("Pill dispensed %d.\n", step_and_stop + 1);
            sensor_triggered = false;
            sendLoRaMessage("Pill dispensed!");
        } else {
            printf("No pill dispensed %d.\n", step_and_stop + 1);
            sendLoRaMessage("No pill dispensed!");
            // Blink LED 5 times to indicate error
            for (int blink = 0; blink < 5; blink++) {
                gpio_put(LED_1, 1);
                sleep_ms(200);
                gpio_put(LED_1, 0);
                sleep_ms(200);
            }
        }

        sleep_ms(2000); // Wait for next cycle
        step_and_stop++;
    }
    char final_message[50];
    snprintf(final_message, sizeof(final_message), "Total pills dispensed: %d", pills_dispensed);
    sendLoRaMessage(final_message);
}


int main() {
    stdio_init_all();
    SetMotor();
    SetOptoFork();
    SetPiezoSensor();

    gpio_init(SW_1);
    gpio_set_dir(SW_1, GPIO_IN);
    gpio_pull_up(SW_1);
    gpio_init(LED_1);
    gpio_set_dir(LED_1, GPIO_OUT);
    gpio_put(LED_1, false);
    gpio_init(SW_2);
    gpio_set_dir(SW_2, GPIO_IN);
    gpio_pull_up(SW_2);
    gpio_init(LED_2);
    gpio_set_dir(LED_2, GPIO_OUT);
    gpio_put(LED_2, false);




    gpio_set_irq_enabled_with_callback(PIEZO_SENSOR, GPIO_IRQ_EDGE_FALL, true, piezo_interrupt);

    initLoRa();
    if (!joinLoRa()) {
        printf("Failed to join LoRa network. Exiting.\n");
    }


    initI2C();
    restore_motor_state();
    bool system_calibrated = false;
    while (1) {
        // Wait for calibration button press
        IdleMode();
        if (gpio_get(SW_1) == 0 && !system_calibrated) {
            printf("Calibrating...\n");
            calibrate_motor();
            align_motor();
            system_calibrated = true;
            gpio_put(LED_1, 1);
            store_motor_state();
        }

        // Wait for pill dispensing button press
        if (gpio_get(SW_2) == 0 && system_calibrated) {
            printf("Dispensing pills...\n");
            DispensePills();
            align_motor();
            system_calibrated = false;
            gpio_put(LED_2, 0);
        }
    }

    return 0;
}