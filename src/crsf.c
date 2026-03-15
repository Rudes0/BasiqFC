/*
Filip Michalak 2025

CRSF driver protocol for Drone project 

This protocol uses LSB and little endian structure

*/
#include "crsf.h"

// ---------------------------------------
// Public API
// ---------------------------------------
void CRSF_Init(crsf_data* CRSF) // CRSF basic values 
{
    CRSF_UartInit(CRSF);
    CRSF_CRC8LutInit(CRSF); // creating look up table 
    CRSF->state = state_wait; // initial state of state machine 
    CRSF->byteID = 0;
    CRSF->packetLen = 0;
}

void CRSF_StateMachine(crsf_data* CRSF) // state machine that allows for correct data transmission 
{
    uint8_t c = 0;
    uint8_t sync = 0;

    uint8_t crc8_test = 0; // CRC8 test value that we calculate 
    uint8_t crc8_true = 0; // CRC8 value that comes with a packet of data 
    while(uart_is_readable(CRSF->CRSFUartCRSFPort))
    {
        c = uart_getc(CRSF->CRSFUartCRSFPort);
        switch (CRSF->state)
        {
        case state_wait:
            sync = c;
            if(sync == CRSF_SYNC) // if first byte is correct byte for synchronization 
            {
                CRSF->frame[CRSF->byteID++] = sync;
                CRSF->state = state_check_lenght; // change state to next one 
            }
            break;
        case state_check_lenght:
            CRSF->packetLen = c;
            if(CRSF->packetLen > 3 && CRSF->packetLen <= sizeof(CRSF->frame) - 2)  // if second byte is lenght of correct size 
            {
                CRSF->frame[CRSF->byteID++] = CRSF->packetLen;
                CRSF->state = state_read_payload;
            } 
            else // if not go back to first state and reset values 
            {
                CRSF->state = state_wait;
                CRSF->frame[0] = 0;
                CRSF->byteID = 0;
            }
            break;
        case state_read_payload:
            CRSF->frame[CRSF->byteID++] = c;
            if(CRSF->byteID == CRSF->packetLen + 2) // if collected all data then continue 
            {
                crc8_true = CRSF->frame[CRSF->byteID-1]; 
                crc8_test = CRSF_CRC8Check(CRSF);
                if(crc8_test == crc8_true && CRSF->frame[2] == CRSF_TYPE_RC && CRSF->packetLen >= 24) // if everything is correct then unpack data
                {
                    CRSF_UnpackPayloadData(CRSF);
                }
                CRSF->state = state_wait;
                CRSF->byteID = 0;
                CRSF_ChannelMaping(CRSF);
            }
            break;
        default:
                CRSF->state = state_wait;
                CRSF->byteID = 0;
            break;
        }
    }
    
}

// ---------------------------------------
// Internal functions
// ---------------------------------------
void CRSF_UartInit(crsf_data* CRSF) 
{
    uart_init(CRSF->CRSFUartCRSFPort,CRSF_BAUD_RATE);
    gpio_set_function(CRSF->CRSFUartTxPin,GPIO_FUNC_UART);
    gpio_set_function(CRSF->CRSFUartRxPin,GPIO_FUNC_UART);
    uart_set_baudrate(CRSF->CRSFUartCRSFPort, CRSF_BAUD_RATE);
    uart_set_format(CRSF->CRSFUartCRSFPort, 8, 1, UART_PARITY_NONE);
}

void CRSF_UnpackPayloadData(crsf_data* CRSF) // unpacking data to corect channels 
{
    const uint8_t numOfChannels = 16;
    const uint8_t srcBits = 11; 
    const uint16_t inputChannelMask = (1 << srcBits) - 1;

    uint8_t mergedBits = 0;
    uint8_t byteIndex = 3;
    uint32_t readValue = 0;
    uint8_t readByte = 0;

    for(int i = 0; i < numOfChannels; i++)
    {
        while(mergedBits < srcBits)
        {
            readByte = CRSF->frame[byteIndex++];
            readValue |= ((uint32_t)readByte) << mergedBits;
            mergedBits += 8;
        }
        CRSF->rawData[i] = (readValue & inputChannelMask);
        readValue >>= srcBits;
        mergedBits -= srcBits;
    }
}

void CRSF_CRC8LutInit(crsf_data* CRSF) // look up table that is used for faster CRC8 check  
{
    for(int val = 0; val < 256; val++) // table has 256 posible values 
    {
        uint8_t crc = val;
        for(int i = 0; i < 8; i++)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRC_POLY : 0); // moveing left bit and if it is a 1 then we divide with polynomial 
        }
        CRSF->lut[val] = crc; // after 8 moves and some divisions we are left with a reminder that is what we are looking for
    }
} 

uint8_t CRSF_CRC8Check(crsf_data* CRSF)
{
    uint8_t crc = 0;
    uint8_t len = CRSF->frame[1] - 1;
    for(int i = 0; i < len; i++)
    {
        crc = CRSF->lut[crc ^ CRSF->frame[i + 2]]; 
        // we calculate our own reminder(by using the table for fast division/bit shifting) 
        // and then we can compare the two with each other to check if message is correct
    }
    return crc;
}

void CRSF_ChannelMaping(crsf_data* CRSF) // mapping data from CRSF format to PWM 1000-2000us format
{
    for(uint8_t i = 0; i < sizeof(CRSF->rawData) / sizeof(CRSF->rawData[0]); i++)
    {
       CRSF->pwmData[i] = 1500 + ((5 * (CRSF->rawData[i] - 992)) / 8); 
       if(CRSF->pwmData[i] > 2000) CRSF->pwmData[i] = 2000; // limiting value to 2000 if above (rare but could happend)
       if(CRSF->pwmData[i] < 1000) CRSF->pwmData[i] = 1000; // limiting value to 1000 if below (rare but could happend)
    }
}