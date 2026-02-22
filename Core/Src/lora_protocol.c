/**
  ******************************************************************************
  * @file           : lora_protocol.c
  * @brief          : LoRa framed protocol implementation (compatible with Arduino)
  ******************************************************************************
  */

#include "lora_protocol.h"
#include <string.h>

/**
  * @brief  Initialize protocol handler
  * @param  proto: Protocol handle
  * @param  lora: LoRa handle
  */
void LoRa_Protocol_Init(LoRa_Protocol_t *proto, SX127x_t *lora)
{
    proto->lora = lora;
    proto->seq_counter = 0;
    proto->data_received_callback = NULL;
    proto->ack_received_callback = NULL;
}

/**
  * @brief  Send a data packet
  * @param  proto: Protocol handle
  * @param  dst: Destination address
  * @param  msg: Message string (null-terminated)
  * @retval true if successful
  */
bool LoRa_Protocol_SendData(LoRa_Protocol_t *proto, uint8_t dst, const char *msg)
{
    if (!proto || !proto->lora || !msg) {
        return false;
    }
    
    uint8_t len = strlen(msg);
    if (len > MAX_PAYLOAD_SIZE) {
        len = MAX_PAYLOAD_SIZE;
    }
    
    if (len == 0) {
        return false; // Don't send empty messages
    }
    
    // Build framed packet
    uint8_t packet[5 + len];
    packet[0] = dst;                    // Destination
    packet[1] = ADDR_STM32;             // Source (STM32)
    packet[2] = PKT_TYPE_DATA;          // Type
    packet[3] = proto->seq_counter++;   // Sequence number
    packet[4] = len;                    // Payload length
    memcpy(&packet[5], msg, len);       // Payload
    
    // Send via LoRa
    SX127x_BeginPacket(proto->lora);
    SX127x_Write(proto->lora, packet, 5 + len);
    SX127x_EndPacket(proto->lora);
    
    // Put back in RX mode
    SX127x_Receive(proto->lora);
    
    return true;
}

/**
  * @brief  Send an ACK packet
  * @param  proto: Protocol handle
  * @param  dst: Destination address
  * @param  seq: Sequence number to acknowledge
  * @retval true if successful
  */
bool LoRa_Protocol_SendAck(LoRa_Protocol_t *proto, uint8_t dst, uint8_t seq)
{
    if (!proto || !proto->lora) {
        return false;
    }
    
    // Build ACK packet (header only, no payload)
    uint8_t ack[5];
    ack[0] = dst;              // Destination (original sender)
    ack[1] = ADDR_STM32;       // Source (STM32)
    ack[2] = PKT_TYPE_ACK;     // Type
    ack[3] = seq;              // Sequence number
    ack[4] = 0;                // Payload length = 0
    
    // Send via LoRa
    SX127x_BeginPacket(proto->lora);
    SX127x_Write(proto->lora, ack, 5);
    SX127x_EndPacket(proto->lora);
    
    // Put back in RX mode
    SX127x_Receive(proto->lora);
    
    return true;
}

/**
  * @brief  Process incoming packets
  * @param  proto: Protocol handle
  */
void LoRa_Protocol_ProcessIncoming(LoRa_Protocol_t *proto)
{
    if (!proto || !proto->lora) {
        return;
    }
    
    // Check for incoming packet
    int packetSize = SX127x_ParsePacket(proto->lora);
    
    if (packetSize > 0) {
        // Read entire packet into buffer first
        uint8_t buffer[256];
        int bytesRead = 0;
        
        // Read all available bytes
        while (SX127x_Available(proto->lora) && bytesRead < sizeof(buffer)) {
            int b = SX127x_Read(proto->lora);
            if (b != -1) {
                buffer[bytesRead++] = (uint8_t)b;
            } else {
                break;
            }
        }
        
        // Check if we got at least a valid header (5 bytes)
        if (bytesRead >= 5) {
            uint8_t dst = buffer[0];
            uint8_t src = buffer[1];
            uint8_t type = buffer[2];
            uint8_t seq = buffer[3];
            uint8_t len = buffer[4];
            
            // Check if packet is for us
            if (dst == ADDR_STM32) {
                
                if (type == PKT_TYPE_DATA) {
                    // Data packet - extract payload
                    if (bytesRead >= 5 + len) {
                        uint8_t payload[MAX_PAYLOAD_SIZE + 1];
                        int payloadLen = (len < MAX_PAYLOAD_SIZE) ? len : MAX_PAYLOAD_SIZE;
                        memcpy(payload, &buffer[5], payloadLen);
                        payload[payloadLen] = '\0'; // Null-terminate
                        
                        // Send ACK back to sender
                        LoRa_Protocol_SendAck(proto, src, seq);
                        
                        // Call callback if registered
                        if (proto->data_received_callback) {
                            proto->data_received_callback((char*)payload, payloadLen);
                        }
                    }
                    
                } else if (type == PKT_TYPE_ACK) {
                    // ACK packet - no payload
                    if (proto->ack_received_callback) {
                        proto->ack_received_callback(seq);
                    }
                }
            }
        }
    }
}
