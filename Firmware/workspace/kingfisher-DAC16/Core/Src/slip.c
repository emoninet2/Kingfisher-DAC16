#include "slip.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Sends a packet over SLIP protocol, handling special characters.
 * @param  slip: Pointer to SLIP handle that provides the send_char function.
 * @param  p: Pointer to the data buffer to send.
 * @param  len: Length of the data to send.
 * @retval None
 */
void slip_send_packet(SLIP_HandleTypeDef *slip, uint8_t *p, uint32_t len) {
    // Send the initial SLIP_END to start the packet
    slip->send_char(SLIP_END);

    // Iterate over each byte in the packet
    for (uint32_t i = 0; i < len; i++) {
        switch (p[i]) {
            // If the byte is SLIP_END, it needs to be escaped
            case SLIP_END:
                slip->send_char(SLIP_ESC);        // Escape the SLIP_END
                slip->send_char(SLIP_ESC_END);   // Send the escaped SLIP_END
                break;

            // If the byte is SLIP_ESC, it needs to be escaped
            case SLIP_ESC:
                slip->send_char(SLIP_ESC);        // Escape the SLIP_ESC
                slip->send_char(SLIP_ESC_ESC);   // Send the escaped SLIP_ESC
                break;

            // For normal data bytes, send them as is
            default:
                slip->send_char(p[i]);
                break;
        }
    }

    // Send the final SLIP_END to end the packet
    slip->send_char(SLIP_END);
}

/**
 * @brief  Receives a SLIP packet and stores it in the provided buffer.
 *         Handles SLIP_END and SLIP_ESC special characters.
 * @param  slip: Pointer to SLIP handle that provides the recv_char function.
 * @param  p: Pointer to the buffer to store the received data.
 * @param  len: Maximum length of the buffer to store the received data.
 * @retval The number of bytes received and stored in the buffer (as uint8_t).
 */
uint32_t slip_recv_packet(SLIP_HandleTypeDef *slip, uint8_t *p, uint32_t len) {
    uint32_t received = 0;
    bool escaping = false;
    uint8_t c;

    // Receive characters until a SLIP_END is encountered (end of packet)
    while (received < len) {
        c = slip->recv_char();  // Receive one byte

        if (escaping) {
            // Handle escape sequence
            if (c == SLIP_ESC_END) {
                p[received++] = SLIP_END;  // Escaped SLIP_END
            } else if (c == SLIP_ESC_ESC) {
                p[received++] = SLIP_ESC;  // Escaped SLIP_ESC
            }
            escaping = false;  // Clear escaping state after handling escape
        } else {
            // Regular data byte or special SLIP_END byte
            if (c == SLIP_ESC) {
                escaping = true;  // Start escaping sequence
            } else if (c == SLIP_END) {
                if (received > 0) {
                    // End of packet and data has been received
                    break;
                }
            } else {
                // Regular byte, add to buffer
                p[received++] = c;
            }
        }
    }

    return received;  // Return number of bytes received
}

