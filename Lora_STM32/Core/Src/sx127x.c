/**
  ******************************************************************************
  * @file           : sx127x.c
  * @brief          : SX127x LoRa driver implementation
  ******************************************************************************
  */

#include "sx127x.h"
#include <string.h>

/* Private variables */
static uint8_t _packetIndex = 0;
static uint8_t _packetLength = 0;  // salvato al momento della ricezione
static uint8_t _implicitHeaderMode = 0;

/* Private function prototypes */
static void SX127x_SetLdoFlag(SX127x_t *lora);

/**
  * @brief  Read a register from SX127x
  * @param  lora: LoRa handle
  * @param  reg: Register address
  * @retval Register value
  */
uint8_t SX127x_ReadRegister(SX127x_t *lora, uint8_t reg)
{
    uint8_t txBuf[2] = {reg & 0x7F, 0x00};
    uint8_t rxBuf[2] = {0};
    
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(lora->hspi, txBuf, rxBuf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_SET);
    
    return rxBuf[1];
}

/**
  * @brief  Write a register to SX127x
  * @param  lora: LoRa handle
  * @param  reg: Register address
  * @param  value: Value to write
  */
void SX127x_WriteRegister(SX127x_t *lora, uint8_t reg, uint8_t value)
{
    uint8_t txBuf[2] = {reg | 0x80, value};
    
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(lora->hspi, txBuf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(lora->nss_port, lora->nss_pin, GPIO_PIN_SET);
}

/**
  * @brief  Reset SX127x module
  * @param  lora: LoRa handle
  */
void SX127x_Reset(SX127x_t *lora)
{
    HAL_GPIO_WritePin(lora->rst_port, lora->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(20);   // datasheet: tRESET min 100µs, usiamo 20ms per sicurezza
    HAL_GPIO_WritePin(lora->rst_port, lora->rst_pin, GPIO_PIN_SET);
    HAL_Delay(150);  // datasheet: tREADY max 10ms dopo il reset, usiamo 150ms
}

/**
  * @brief  Put SX127x in sleep mode
  * @param  lora: LoRa handle
  */
void SX127x_Sleep(SX127x_t *lora)
{
    SX127x_WriteRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
  * @brief  Put SX127x in idle/standby mode
  * @param  lora: LoRa handle
  */
void SX127x_Idle(SX127x_t *lora)
{
    SX127x_WriteRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
  * @brief  Check if SX127x is transmitting
  * @param  lora: LoRa handle
  * @retval true if transmitting
  */
bool SX127x_IsTransmitting(SX127x_t *lora)
{
    uint8_t mode = SX127x_ReadRegister(lora, REG_OP_MODE);
    return (mode & MODE_TX) == MODE_TX;
}

/**
  * @brief  Set frequency
  * @param  lora: LoRa handle
  * @param  frequency: Frequency in Hz
  */
void SX127x_SetFrequency(SX127x_t *lora, uint32_t frequency)
{
    lora->frequency = frequency;
    
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    
    SX127x_WriteRegister(lora, REG_FRF_MSB, (uint8_t)(frf >> 16));
    SX127x_WriteRegister(lora, REG_FRF_MID, (uint8_t)(frf >> 8));
    SX127x_WriteRegister(lora, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
  * @brief  Set TX power
  * @param  lora: LoRa handle
  * @param  power: Power level (2-17 dBm)
  */
void SX127x_SetTxPower(SX127x_t *lora, uint8_t power)
{
    lora->tx_power = power;
    
    if (power > 17) {
        power = 17;
    } else if (power < 2) {
        power = 2;
    }
    
    SX127x_WriteRegister(lora, REG_PA_CONFIG, PA_BOOST | (power - 2));
}

/**
  * @brief  Set spreading factor
  * @param  lora: LoRa handle
  * @param  sf: Spreading factor (6-12)
  */
void SX127x_SetSpreadingFactor(SX127x_t *lora, uint8_t sf)
{
    if (sf < 6) sf = 6;
    if (sf > 12) sf = 12;
    
    lora->spreading_factor = sf;
    
    if (sf == 6) {
        SX127x_WriteRegister(lora, REG_DETECTION_OPTIMIZE, 0xC5);
        SX127x_WriteRegister(lora, REG_DETECTION_THRESHOLD, 0x0C);
    } else {
        SX127x_WriteRegister(lora, REG_DETECTION_OPTIMIZE, 0xC3);
        SX127x_WriteRegister(lora, REG_DETECTION_THRESHOLD, 0x0A);
    }
    
    uint8_t config2 = SX127x_ReadRegister(lora, REG_MODEM_CONFIG_2);
    config2 = (config2 & 0x0F) | ((sf << 4) & 0xF0);
    SX127x_WriteRegister(lora, REG_MODEM_CONFIG_2, config2);
    
    SX127x_SetLdoFlag(lora);
}

/**
  * @brief  Set bandwidth
  * @param  lora: LoRa handle
  * @param  bw: Bandwidth in Hz
  */
void SX127x_SetBandwidth(SX127x_t *lora, uint32_t bw)
{
    lora->bandwidth = bw;
    
    uint8_t bw_val;
    if (bw <= 7800) bw_val = 0;
    else if (bw <= 10400) bw_val = 1;
    else if (bw <= 15600) bw_val = 2;
    else if (bw <= 20800) bw_val = 3;
    else if (bw <= 31250) bw_val = 4;
    else if (bw <= 41700) bw_val = 5;
    else if (bw <= 62500) bw_val = 6;
    else if (bw <= 125000) bw_val = 7;
    else if (bw <= 250000) bw_val = 8;
    else bw_val = 9; // 500kHz
    
    uint8_t config1 = SX127x_ReadRegister(lora, REG_MODEM_CONFIG_1);
    config1 = (config1 & 0x0F) | (bw_val << 4);
    SX127x_WriteRegister(lora, REG_MODEM_CONFIG_1, config1);
    
    SX127x_SetLdoFlag(lora);
}

/**
  * @brief  Set coding rate
  * @param  lora: LoRa handle
  * @param  cr: Coding rate (5-8, denominator)
  */
void SX127x_SetCodingRate(SX127x_t *lora, uint8_t cr)
{
    if (cr < 5) cr = 5;
    if (cr > 8) cr = 8;
    
    lora->coding_rate = cr;
    cr = cr - 4;
    
    uint8_t config1 = SX127x_ReadRegister(lora, REG_MODEM_CONFIG_1);
    config1 = (config1 & 0xF1) | (cr << 1);
    SX127x_WriteRegister(lora, REG_MODEM_CONFIG_1, config1);
}

/**
  * @brief  Set sync word
  * @param  lora: LoRa handle
  * @param  sw: Sync word (0x12 for private, 0x34 for public)
  */
void SX127x_SetSyncWord(SX127x_t *lora, uint8_t sw)
{
    lora->sync_word = sw;
    SX127x_WriteRegister(lora, REG_SYNC_WORD, sw);
}

/**
  * @brief  Set LDO flag based on symbol duration
  * @param  lora: LoRa handle
  */
static void SX127x_SetLdoFlag(SX127x_t *lora)
{
    // Symbol duration in ms
    long symbolDuration = 1000 / (lora->bandwidth / (1L << lora->spreading_factor));
    
    uint8_t config3 = SX127x_ReadRegister(lora, REG_MODEM_CONFIG_3);
    if (symbolDuration > 16) {
        config3 |= 0x08; // LowDataRateOptimize
    } else {
        config3 &= ~0x08;
    }
    SX127x_WriteRegister(lora, REG_MODEM_CONFIG_3, config3);
}

/**
  * @brief  Initialize LoRa module
  * @param  lora: LoRa handle
  * @retval true if successful
  */
bool SX127x_Init(SX127x_t *lora)
{
    // Reset module
    SX127x_Reset(lora);
    
    // Check version
    uint8_t version = SX127x_ReadRegister(lora, REG_VERSION);
    if (version != 0x12) {
        return false; // Not SX1276/77/78/79
    }
    
    // Put in sleep mode
    SX127x_Sleep(lora);
    
    // Set LoRa mode
    SX127x_WriteRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    
    // Set base addresses
    SX127x_WriteRegister(lora, REG_FIFO_TX_BASE_ADDR, 0);
    SX127x_WriteRegister(lora, REG_FIFO_RX_BASE_ADDR, 0);
    
    // Set LNA boost
    SX127x_WriteRegister(lora, REG_LNA, SX127x_ReadRegister(lora, REG_LNA) | 0x03);
    
    // Set auto AGC
    SX127x_WriteRegister(lora, REG_MODEM_CONFIG_3, 0x04);
    
    // Put in standby mode
    SX127x_Idle(lora);
    
    return true;
}

/**
  * @brief  Begin LoRa with specified frequency
  * @param  lora: LoRa handle
  * @param  frequency: Frequency in Hz
  * @retval true if successful
  */
bool SX127x_Begin(SX127x_t *lora, uint32_t frequency)
{
    if (!SX127x_Init(lora)) {
        return false;
    }
    
    // Set default parameters
    SX127x_SetFrequency(lora, frequency);
    SX127x_SetTxPower(lora, 17);
    SX127x_SetSpreadingFactor(lora, 7);
    SX127x_SetBandwidth(lora, 125000);
    SX127x_SetCodingRate(lora, 5);
    SX127x_SetSyncWord(lora, 0x12);
    
    return true;
}

/**
  * @brief  Begin packet transmission
  * @param  lora: LoRa handle
  */
void SX127x_BeginPacket(SX127x_t *lora)
{
    // Put in standby mode
    SX127x_Idle(lora);
    
    // Set explicit header mode
    _implicitHeaderMode = 0;
    uint8_t config1 = SX127x_ReadRegister(lora, REG_MODEM_CONFIG_1);
    config1 &= 0xFE; // Explicit header
    SX127x_WriteRegister(lora, REG_MODEM_CONFIG_1, config1);
    
    // Reset FIFO address and payload length
    SX127x_WriteRegister(lora, REG_FIFO_ADDR_PTR, 0);
    SX127x_WriteRegister(lora, REG_PAYLOAD_LENGTH, 0);
    
    _packetIndex = 0;
}

/**
  * @brief  End packet transmission
  * @param  lora: LoRa handle
  */
void SX127x_EndPacket(SX127x_t *lora)
{
    // Put in TX mode
    SX127x_WriteRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    // Wait for TX done
    while ((SX127x_ReadRegister(lora, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        // Wait
    }
    
    // Clear IRQ flags
    SX127x_WriteRegister(lora, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
  * @brief  Write data to packet
  * @param  lora: LoRa handle
  * @param  buf: Data buffer
  * @param  size: Data size
  */
void SX127x_Write(SX127x_t *lora, uint8_t *buf, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        SX127x_WriteByte(lora, buf[i]);
    }
}

/**
  * @brief  Write one byte to packet
  * @param  lora: LoRa handle
  * @param  byte: Byte to write
  */
void SX127x_WriteByte(SX127x_t *lora, uint8_t byte)
{
    SX127x_WriteRegister(lora, REG_FIFO, byte);
    
    // Update length
    uint8_t currentLength = SX127x_ReadRegister(lora, REG_PAYLOAD_LENGTH);
    SX127x_WriteRegister(lora, REG_PAYLOAD_LENGTH, currentLength + 1);
}

/**
  * @brief  Check for received packet
  * @param  lora: LoRa handle
  * @retval Packet size or 0 if no packet
  */
int SX127x_ParsePacket(SX127x_t *lora)
{
    uint8_t irqFlags = SX127x_ReadRegister(lora, REG_IRQ_FLAGS);

    // Clear all IRQ flags immediately
    SX127x_WriteRegister(lora, REG_IRQ_FLAGS, 0xFF);

    if ((irqFlags & IRQ_RX_DONE_MASK) && !(irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
        _packetIndex = 0;

        // Save the length NOW, before changing operating mode
        _packetLength = SX127x_ReadRegister(lora, REG_RX_NB_BYTES);

        // Point FIFO_ADDR_PTR to the beginning of the received packet
        SX127x_WriteRegister(lora, REG_FIFO_ADDR_PTR,
                             SX127x_ReadRegister(lora, REG_FIFO_RX_CURRENT_ADDR));

        // DO NOT call Idle/Sleep here: the FIFO is accessible in RX continuous
        // Changing mode resets REG_RX_NB_BYTES and corrupts the read

        return _packetLength;

    } else if ((SX127x_ReadRegister(lora, REG_OP_MODE) & 0x07) != MODE_RX_CONTINUOUS) {
        // Not in RX: restore
        SX127x_WriteRegister(lora, REG_FIFO_RX_BASE_ADDR, 0);
        SX127x_WriteRegister(lora, REG_FIFO_ADDR_PTR, 0);
        SX127x_WriteRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    }

    return 0;
}

/**
  * @brief  Get number of bytes available to read
  * @param  lora: LoRa handle
  * @retval Number of bytes available
  */
int SX127x_Available(SX127x_t *lora)
{
    // Usa _packetLength salvato al momento della ricezione
    // NON rileggere REG_RX_NB_BYTES: in standby vale 0 o un valore casuale
    return (_packetLength - _packetIndex);
}

/**
  * @brief  Read one byte from packet
  * @param  lora: LoRa handle
  * @retval Byte read or -1 if no data
  */
int SX127x_Read(SX127x_t *lora)
{
    if (!SX127x_Available(lora)) {
        return -1;
    }
    
    _packetIndex++;
    return SX127x_ReadRegister(lora, REG_FIFO);
}

/**
  * @brief  Read multiple bytes from packet
  * @param  lora: LoRa handle
  * @param  buf: Buffer to store data
  * @param  size: Number of bytes to read
  * @retval Number of bytes read
  */
int SX127x_ReadBytes(SX127x_t *lora, uint8_t *buf, size_t size)
{
    int count = 0;
    while (count < size && SX127x_Available(lora)) {
        int b = SX127x_Read(lora);
        if (b == -1) break;
        buf[count++] = (uint8_t)b;
    }
    return count;
}

/**
  * @brief  Get RSSI of last packet
  * @param  lora: LoRa handle
  * @retval RSSI in dBm
  */
int SX127x_PacketRssi(SX127x_t *lora)
{
    return (SX127x_ReadRegister(lora, REG_PKT_RSSI_VALUE) - 157);
}

/**
  * @brief  Get SNR of last packet
  * @param  lora: LoRa handle
  * @retval SNR in dB
  */
float SX127x_PacketSnr(SX127x_t *lora)
{
    return ((int8_t)SX127x_ReadRegister(lora, REG_PKT_SNR_VALUE)) * 0.25;
}

/**
  * @brief  Put module in continuous receive mode
  * @param  lora: LoRa handle
  */
void SX127x_Receive(SX127x_t *lora)
{
    // Reset base address and pointer in RX continuous mode, for safety
    // (after a TX the pointer is dirty and the next packet would be
    //  written in a wrong position in the 256-byte FIFO)
    SX127x_WriteRegister(lora, REG_FIFO_RX_BASE_ADDR, 0);
    SX127x_WriteRegister(lora, REG_FIFO_ADDR_PTR, 0);
    SX127x_WriteRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
