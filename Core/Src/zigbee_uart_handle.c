#include "zigbee_uart_handle.h"
#include <stdbool.h>
#include <string.h> // Required for string comparison functions like strncmp
#include <stdlib.h> // Required for atoi
#include <ctype.h>  // Required for isxdigit, tolower

#define RX_BUFFER_SIZE 512
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_data;
volatile uint16_t rx_index = 0;
volatile uint8_t data_ready = 0;
volatile uint8_t rejoin_detect = 0;

volatile uint32_t state_enter_tick = 0;
#define ZIGBEE_RESPONSE_TIMEOUT 5000 // 5 seconds
#define ZIGBEE_INTERVAL_RESPONSE 10  // 60 ms
#define ZIGBEE_MAX_NETWORK_RETRY 12
/* --------------------------- State Machine Definitions -------------------------- */

// Define the states for our Zigbee startup flow
typedef enum {
    ZB_STARTUP_BEGIN,              // Start the process
    ZB_STARTUP_SEND_NWK_CHECK,     // Send "AT+NWK?"
    ZB_STARTUP_WAIT_NWK_STATUS,    // Wait for the "NWK=..." response
    ZB_STARTUP_SEND_JOIN,          // Not in a network, so send "AT+JOIN"
    ZB_STARTUP_WAIT_JOIN_OK,       // Wait for "OK" after sending AT+JOIN
    ZB_STARTUP_WAIT_JOIN_COMPLETE, // Wait for the async "NWK JOINED" message
    ZB_STARTUP_DONE,               // Process finished successfully
    ZB_STARTUP_ERROR,              // An error occurred
    ZB_STARTUP_EXIT_AT,            // Exit AT mode
    ZB_STARTUP_WAIT_EXIT_OK,       // Wait for "OK" after exiting AT mode
    ZB_STARTUP_GET_ADDR,
    ZB_STARTUP_WAIT_ADDR_OK,
    ZB_STARTUP_SET_DSTADDR,
    ZB_STARTUP_WAIT_DSTADDR_OK,
    ZB_STARTUP_SET_DSTEP,
    ZB_STARTUP_WAIT_DSTEP_OK,
    ZB_STARTUP_SET_CHANNEL,
    ZB_STARTUP_WAIT_CHANNEL_OK,
    ZB_STARTUP_DEV_CHECK,
    ZB_STARTUP_WAIT_DEV_OK
} ZigbeeStartupState_t;

typedef enum {
    ZB_INIT_INFO_GET_ID,
    ZB_INIT_INFO_WAIT_ID_OK,
    ZB_INIT_INFO_GET_ID_DONE
} ZigbeeInitState_t;

typedef struct {
    uint8_t zigbee_addr[16];
    uint8_t zigbee_id[16];
    uint8_t zigbee_id_uart_data[16];
} ZigbeeInfo_t;

// Create a global variable to hold the current state
volatile ZigbeeStartupState_t zigbee_startup_state = ZB_STARTUP_BEGIN;
volatile ZigbeeInitState_t zigbee_init_info_state = ZB_INIT_INFO_GET_ID;
volatile ZigbeeInfo_t zigbee_info;

void zigbee_network_init_manager(void);
void zigbee_get_id_manager(void);
/* -------------------------- Private function prototypes ------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/**
 * @brief Converts a single hexadecimal character to its integer value.
 * @param c The character ('0'-'9', 'a'-'f', 'A'-'F').
 * @return The integer value (0-15), or -1 on error.
 */
static int hex_char_to_int(char c)
{
    c = tolower(c);
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

/**
 * @brief Decodes a hex string into a binary bitmap.
 * @param hex_str The null-terminated hex string (e.g., "a813").
 * @param bitmap  The output byte array to store the binary bitmap.
 * @param max_bitmap_size The size of the output bitmap buffer.
 * @return The number of bytes written to the bitmap.
 */
static int decode_hex_to_bitmap(const char* hex_str, uint8_t* bitmap, int max_bitmap_size)
{
    int hex_len = strlen(hex_str);
    int byte_count = 0;
    memset(bitmap, 0, max_bitmap_size); // Clear the bitmap first

    // Process two hex characters at a time to form one byte
    for (int i = 0; i < hex_len && (i + 1) < hex_len; i += 2) {
        if (byte_count >= max_bitmap_size) break; // Prevent buffer overflow

        int high_nibble = hex_char_to_int(hex_str[i]);
        int low_nibble = hex_char_to_int(hex_str[i + 1]);

        if (high_nibble != -1 && low_nibble != -1) {
            bitmap[byte_count++] = (high_nibble << 4) | low_nibble;
        }
    }
    return byte_count;
}

/**
 * @brief Checks if a device should respond and calculates its time slot.
 *
 * @param bitmap The decoded bitmap from the master.
 * @param max_bit The highest possible ID number represented in the bitmap.
 * @param self_id The ID of this device.
 * @return The time slot (0 for first, 1 for second, etc.), or -1 if the device should not respond.
 */
static int get_response_slot(const uint8_t* bitmap, int max_bit, int self_id)
{
    if (self_id <= 0 || self_id > max_bit) {
        return -1;
    }

    // 1. First, check if our own ID is present in the bitmap.
    int self_byte_index = (self_id - 1) / 8;
    int self_bit_index = (self_id - 1) % 8;
    if (!((bitmap[self_byte_index] >> self_bit_index) & 1)) {
        return -1; // Our bit is not set, we should not respond.
    }

    // 2. Our bit is set. Now, count how many devices with a LOWER ID are also set.
    // This count determines our unique time slot for responding.
    int preceding_slaves = 0;
    for (int i = 1; i < self_id; i++) {
        int byte_index = (i - 1) / 8;
        int bit_index = (i - 1) % 8;
        if ((bitmap[byte_index] >> bit_index) & 1) {
            preceding_slaves++;
        }
    }

    return preceding_slaves;
}
void start_timer(void)
{
    state_enter_tick = HAL_GetTick();
}

static bool check_timer_timeout(uint32_t start_tick, uint32_t timeout_ms)
{
    return (HAL_GetTick() - start_tick > timeout_ms);
}

void zigbee_uart_data_send(char *data)
{
    U1_printf(data);
}

void clear_buffer_reable_interrupt(void)
{
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    data_ready = 0;
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}
void zigbee_init(void)
{
    // set low PB9
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    zigbee_startup_state = ZB_STARTUP_BEGIN;
    zigbee_init_info_state = ZB_INIT_INFO_GET_ID;
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    // wait 2s for zigbee reset
    HAL_Delay(2000);
    zigbee_uart_data_send("+AT");
    start_timer(); // MODIFIED: Start timer for the initial command
    U2_printf("Starting...\r\n");
}

void zigbee_transmit_data_handle()
{
    if (data_ready) {
        U2_printf("rx_buffer: %s\r\n", rx_buffer);

        // Check for the new hex bitmap prefix "MBMP:"
        if (strncmp((char *)rx_buffer, "MBMP:", 5) == 0) {
            const char *hex_payload = (const char *)rx_buffer + 5;
            
            // Create a buffer to hold the decoded binary bitmap.
            // Size 64 supports up to 512 slave IDs, adjust if needed.
            uint8_t decoded_bitmap[64]; 
            int bitmap_byte_count = decode_hex_to_bitmap(hex_payload, decoded_bitmap, sizeof(decoded_bitmap));

            if (bitmap_byte_count > 0) {
                // Convert our own ID from string to integer
                int self_id = atoi((char*)zigbee_info.zigbee_id);

                // Get our time slot (e.g., 0, 1, 2...)
                int slot = get_response_slot(decoded_bitmap, bitmap_byte_count * 8, self_id);

                // If slot is not -1, it means we must respond
                if (slot != -1) {
                    U2_printf("ID %d is present. Responding in slot %d.\r\n", self_id, slot);
                    
                    // Wait for our designated time slot to avoid collisions
                    start_timer();
                    while (!check_timer_timeout(state_enter_tick, ZIGBEE_INTERVAL_RESPONSE * slot)) {
                        ; // Busy wait for the correct delay
                    }

                    // Send our ID back to the master
                    zigbee_uart_data_send((char *)zigbee_info.zigbee_id_uart_data);
                }
            }
        }
        clear_buffer_reable_interrupt();
    }
}


void zigbee_run(void)
{
    if (zigbee_startup_state != ZB_STARTUP_DONE) {
        zigbee_network_init_manager();
    } else if (zigbee_init_info_state != ZB_INIT_INFO_GET_ID_DONE) {
        zigbee_get_id_manager();
    } else {
        // start serial receive and send logic
        zigbee_transmit_data_handle();
    }
}

void zigbee_get_id_manager(void)
{
    if (zigbee_init_info_state == ZB_INIT_INFO_GET_ID_DONE) {
        return;
    }

    switch (zigbee_init_info_state) {
    case ZB_INIT_INFO_GET_ID:
        zigbee_uart_data_send(zigbee_info.zigbee_addr);
        U2_printf("Get ID: %s\r\n", zigbee_info.zigbee_addr);
        zigbee_init_info_state = ZB_INIT_INFO_WAIT_ID_OK;
        start_timer();
        break;

    case ZB_INIT_INFO_WAIT_ID_OK:
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Get ID timeout, retrying\r\n");
            zigbee_init_info_state = ZB_INIT_INFO_GET_ID;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, (char *)(zigbee_info.zigbee_addr + 6), 6) == 0) {
                // 0x4653:03
                U2_printf("Get ID OK: %s\r\n", rx_buffer);
                memcpy(zigbee_info.zigbee_id, rx_buffer + 7, 2);
                zigbee_info.zigbee_id[2] = '\0';
                strcpy((char *)zigbee_info.zigbee_id_uart_data, (char *)zigbee_info.zigbee_id);
                zigbee_info.zigbee_id_uart_data[2] = '\n';
                U2_printf("ID: %s\r\n", zigbee_info.zigbee_id);
                zigbee_init_info_state = ZB_INIT_INFO_GET_ID_DONE;
            } else {
                U2_printf("Get ID fail: %s\r\n", rx_buffer);
            }
            clear_buffer_reable_interrupt();
        }
        break;
    }
}

void zigbee_network_init_manager(void)
{
    // If the process is finished or has failed, do nothing.
    if (zigbee_startup_state == ZB_STARTUP_DONE || zigbee_startup_state == ZB_STARTUP_ERROR) {
        return;
    }

    // --- Main state machine logic ---
    switch (zigbee_startup_state) {

    case ZB_STARTUP_BEGIN:
        // MODIFIED: Check for a timeout while waiting for the initial "AT_MODE" response.
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout waiting for AT_MODE, retrying...\r\n");
            zigbee_uart_data_send("+AT");
            start_timer();
        }
        if (data_ready) {
            U2_printf("Starting Zigbee network check...\r\n");
            U2_printf("rx_buffer: %s\r\n", rx_buffer);
            if (strncmp((char *)rx_buffer, "AT_MODE", 7) == 0) {
                zigbee_startup_state = ZB_STARTUP_DEV_CHECK;
            } else {
                zigbee_uart_data_send("+AT");
                start_timer(); // MODIFIED: Restart timer on retry
            }
            clear_buffer_reable_interrupt();
        }
        break;

    case ZB_STARTUP_DEV_CHECK:
        zigbee_uart_data_send("AT+DEV?"); // Send the device status query
        start_timer();                   // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_DEV_OK;
        break;
    case ZB_STARTUP_WAIT_DEV_OK:
        // MODIFIED: Add timeout check for AT+DEV? response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout waiting for DEV status, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_DEV_CHECK;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "DEV=", 4) == 0) {
                U2_printf("Device type detect OK: %s\r\n", rx_buffer);
                zigbee_startup_state = ZB_STARTUP_SEND_NWK_CHECK;
            } else {
                U2_printf("Device type detect not OK, retrying...\r\n");
                zigbee_startup_state = ZB_STARTUP_DEV_CHECK;
            }
            clear_buffer_reable_interrupt();
        }
        break;
    
    case ZB_STARTUP_SEND_NWK_CHECK:
        zigbee_uart_data_send("AT+NWK?"); // Send the network status query
        start_timer();                   // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_NWK_STATUS;
        break;

    case ZB_STARTUP_WAIT_NWK_STATUS:
        // MODIFIED: Add timeout check for AT+NWK? response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout waiting for NWK status, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_SEND_NWK_CHECK;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "NWK=1", 5) == 0) {
                U2_printf("Network status OK. Startup complete.\r\n");
                zigbee_startup_state = ZB_STARTUP_GET_ADDR;
            } else if (strncmp((char *)rx_buffer, "NWK=0", 5) == 0) {
                U2_printf("Not in a network. Attempting to join...\r\n");
                zigbee_startup_state = ZB_STARTUP_SET_CHANNEL;
            } else if (strncmp((char *)rx_buffer, "NWK=2", 5) == 0) {
                U2_printf("Network offline, redetect\r\n");
                HAL_Delay(5000);
                rejoin_detect++;
							  zigbee_startup_state = ZB_STARTUP_SET_CHANNEL;
                if (rejoin_detect > ZIGBEE_MAX_NETWORK_RETRY) {
                   zigbee_uart_data_send("AT+LEAVE");
                   U2_printf("Leave network for rejoin\r\n");
                   zigbee_init();
                   HAL_Delay(1000);
                }
                
            } else {
							
                U2_printf("Error: Unexpected response to AT+NWK?: %s\r\n", rx_buffer);
                zigbee_startup_state = ZB_STARTUP_SEND_NWK_CHECK;
            }
            clear_buffer_reable_interrupt();
        }
        break;

    case ZB_STARTUP_SEND_JOIN:
        zigbee_uart_data_send("AT+JOIN"); // Send the join command
        start_timer();                    // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_JOIN_OK;
        break;

    case ZB_STARTUP_WAIT_JOIN_OK:
        // MODIFIED: Add timeout check for AT+JOIN response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout waiting for JOIN OK, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_SEND_JOIN;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
                U2_printf("Join command accepted. Waiting for network connection...\r\n");
                zigbee_startup_state = ZB_STARTUP_SEND_NWK_CHECK;
            } else {
                U2_printf("Error: AT+JOIN command failed.\r\n");
                zigbee_startup_state = ZB_STARTUP_SEND_NWK_CHECK;
            }
            clear_buffer_reable_interrupt();
        }
        break;

    case ZB_STARTUP_DONE:
        U2_printf("Zigbee network init complete.\r\n");
        break;

    case ZB_STARTUP_EXIT_AT:
        zigbee_uart_data_send("AT+EXIT");
        start_timer(); // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_EXIT_OK;
        break;

    case ZB_STARTUP_WAIT_EXIT_OK:
        // MODIFIED: Add timeout check for AT+EXIT response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout waiting for EXIT OK, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_EXIT_AT;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
                U2_printf("AT+EXIT finish.\r\n");
                zigbee_startup_state = ZB_STARTUP_DONE;
            } else {
                U2_printf("Error: AT+EXIT command failed.\r\n");
                zigbee_startup_state = ZB_STARTUP_EXIT_AT;
            }
            clear_buffer_reable_interrupt();
        }
        break;
    case ZB_STARTUP_GET_ADDR:
        zigbee_uart_data_send("AT+ADDR?");
        start_timer(); // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_ADDR_OK;
        break;

    case ZB_STARTUP_WAIT_ADDR_OK:
        // MODIFIED: Add timeout check for AT+ADDR? response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout waiting for ADDR, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_GET_ADDR;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "ADDR=", 5) == 0) {
                sprintf((char *)zigbee_info.zigbee_addr, "GETID:%s\r\n", (char *)(rx_buffer + 5));
                U2_printf("%s\r\n", rx_buffer);
                zigbee_startup_state = ZB_STARTUP_SET_DSTADDR;
                U2_printf("ADDR: %s\r\n", zigbee_info.zigbee_addr);
            } else {
                U2_printf("Error: AT+ADDR command failed., rx_buffer: %s\r\n", rx_buffer);
                zigbee_startup_state = ZB_STARTUP_GET_ADDR;
            }
            clear_buffer_reable_interrupt();
        }
        break;

    case ZB_STARTUP_SET_DSTADDR:
        zigbee_uart_data_send("AT+DSTADDR=0x0000");
        start_timer(); // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_DSTADDR_OK;
        break;

    case ZB_STARTUP_WAIT_DSTADDR_OK:
        // MODIFIED: Add timeout check for AT+DSTADDR response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout setting DSTADDR, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_SET_DSTADDR;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "DSTADDR=0x0000", 14) == 0) {
                U2_printf("AT+DSTADDR command accepted.\r\n");
                zigbee_startup_state = ZB_STARTUP_SET_DSTEP;
            } else {
                U2_printf("Error: AT+DSTADDR command failed.\r\n");
                zigbee_startup_state = ZB_STARTUP_SET_DSTADDR;
            }
            clear_buffer_reable_interrupt();
        }
        break;

    case ZB_STARTUP_SET_DSTEP:
        zigbee_uart_data_send("AT+DSTEP=0x01");
        start_timer(); // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_DSTEP_OK;
        break;

    case ZB_STARTUP_WAIT_DSTEP_OK:
        // MODIFIED: Add timeout check for AT+DSTEP response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout setting DSTEP, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_SET_DSTEP;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "DSTEP=0x01", 10) == 0) {
                U2_printf("AT+DSTEP command accepted.\r\n");
                zigbee_startup_state = ZB_STARTUP_EXIT_AT;
            } else {
                U2_printf("Error: AT+DSTEP command failed.\r\n");
                zigbee_startup_state = ZB_STARTUP_SET_DSTEP;
            }
            clear_buffer_reable_interrupt();
        }
        break;

    case ZB_STARTUP_SET_CHANNEL:
        zigbee_uart_data_send("AT+CH=11");
        start_timer(); // MODIFIED: Start timer
        zigbee_startup_state = ZB_STARTUP_WAIT_CHANNEL_OK;
        break;
    case ZB_STARTUP_WAIT_CHANNEL_OK:
        // MODIFIED: Add timeout check for AT+CH response
        if (check_timer_timeout(state_enter_tick, ZIGBEE_RESPONSE_TIMEOUT)) {
            U2_printf("Timeout setting CH, retrying...\r\n");
            zigbee_startup_state = ZB_STARTUP_SET_CHANNEL;
        }
        if (data_ready) {
            if (strncmp((char *)rx_buffer, "CH=11", 5) == 0) {
                U2_printf("AT+CH command accepted.\r\n");
                zigbee_startup_state = ZB_STARTUP_SEND_JOIN;
            } else {
                U2_printf("rx_buffer: %s\r\n", rx_buffer);
                U2_printf("Error: AT+CH command failed.\r\n");
                zigbee_startup_state = ZB_STARTUP_SET_CHANNEL;
            }
            clear_buffer_reable_interrupt();
        }
        break;
    default:
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // Check if the interrupt is from the correct UART
    {
        // Ensure we don't overflow the buffer, leave one byte for the null terminator
        if (rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = rx_data; // Store the received byte

            // Check if the received character is a newline ('\n')
            if (rx_data == '\n') {
                rx_buffer[rx_index] = '\0'; // Null-terminate the string
                data_ready = 1;             // Set flag for the main loop to process
            } else {
                // If not the end of the line, re-arm the interrupt to get the next byte
                HAL_UART_Receive_IT(&huart1, &rx_data, 1);
            }
        } else {
            // Buffer is full, reset to prevent errors
            rx_index = 0;
            HAL_UART_Receive_IT(&huart1, &rx_data, 1);
        }
    }
}