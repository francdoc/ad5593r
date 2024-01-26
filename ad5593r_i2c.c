#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define _ADAC_NULL 0b00000000
#define _ADAC_ADC_SEQUENCE 0b00000010
#define _ADAC_GP_CONTROL 0b00000011
#define _ADAC_ADC_CONFIG 0b00000100
#define _ADAC_DAC_CONFIG 0b00000101
#define _ADAC_PULL_DOWN 0b00000110
#define _ADAC_LDAC_MODE 0b00000111
#define _ADAC_GPIO_WR_CONFIG 0b00001000
#define _ADAC_GPIO_WR_DATA 0b00001001
#define _ADAC_GPIO_RD_CONFIG 0b00001010
#define _ADAC_POWER_REF_CTRL 0b00001011
#define _ADAC_OPEN_DRAIN_CFG 0b00001100
#define _ADAC_THREE_STATE 0b00001101
#define _ADAC_RESERVED 0b00001110
#define _ADAC_SOFT_RESET 0b00001111

#define _ADAC_VREF_ON 0b00000010
#define _ADAC_SEQUENCE_ON 0b00000010

#define _ADAC_DAC_WRITE 0b00010000
#define _ADAC_ADC_READ 0b01000000
#define _ADAC_DAC_READ 0b01010000
#define _ADAC_GPIO_READ 0b01110000
#define _ADAC_REG_READ 0b01100000

#define _num_of_channels 8

struct configuration
{
    bool ADCs[8]; // ADC pins
    bool DACs[8]; // DAC pins
    bool GPIs[8]; // input pins
    bool GPOs[8]; // output pins
};

// This structure contains arrays of
struct Read_write_values
{
    float ADCs[8];
    float DACs[8];
    bool GPI_reads[8];
    bool GPO_writes[8];
};

struct AD5593R
{
    uint8_t _a0;
    uint8_t _GPRC_msbs;
    uint8_t _GPRC_lsbs;
    uint8_t _PCR_msbs;
    uint8_t _PCR_lsbs;
    uint8_t _i2c_address;
    uint8_t _DAC_config;
    uint8_t _ADC_config;
    uint8_t _GPI_config;
    uint8_t _GPO_config;

    struct configuration config;
    struct Read_write_values values;

    float _Vref;

    float _ADC_max;
    float _DAC_max;

    bool _ADC_2x_mode;
    bool _DAC_2x_mode;
};

#define SDA 10
#define SCL 11
#define i2c i2c1_inst
static int addr = 0x10;

void AD5593R_init(struct AD5593R *device, int a0)
{
    // Initialize the device structure
    device->_a0 = a0;
    device->_GPRC_msbs = 0x00;
    device->_GPRC_lsbs = 0x00;
    device->_PCR_msbs = 0x00;
    device->_PCR_lsbs = 0x00;
    device->_i2c_address = 0x10;
    device->_Vref = -1;
    device->_ADC_2x_mode = 0;
    device->_DAC_2x_mode = 0;
    device->_ADC_max = -1;
    device->_DAC_max = -1;

    // Initialize the configuration arrays
    for (int i = 0; i < _num_of_channels; i++)
    {
        device->config.ADCs[i] = 0;
        device->config.DACs[i] = 0;
    }

    // Initialize the read/write arrays
    for (int i = 0; i < _num_of_channels; i++)
    {
        device->values.ADCs[i] = -1;
        device->values.DACs[i] = -1;
    }

    // Initialize the GPIO configuration arrays
    if (device->_a0 > -1)
    {
        gpio_init(device->_a0);
        gpio_set_dir(device->_a0, GPIO_OUT);
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico) -> hw pins 6 and 7.
    i2c_init(&i2c, 100*1000);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
    
    // Make the I2C pins available to picotool
    // bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C));
}

void AD5593R_enable_internal_Vref(struct AD5593R *device)
{   
    // Enable selected device for writing
    device->_Vref = 2.5;
    device->_ADC_max = device->_Vref;
    device->_DAC_max = device->_Vref;

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    // Check if the on bit is already flipped on
    if ((device->_PCR_msbs & 0x02) != 0x02)
    {
        device->_PCR_msbs = device->_PCR_msbs ^ 0x02;
    }

    uint8_t buf[] = {_ADAC_POWER_REF_CTRL, device->_PCR_msbs, device->_PCR_lsbs};

    i2c_write_blocking(&i2c, addr, buf, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
}

void AD5593R_disable_internal_Vref(struct AD5593R *device)
{
    device->_Vref = -1;
    device->_ADC_max = device->_Vref;
    device->_DAC_max = device->_Vref;
    // Enable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    // Check if the on bit is already flipped off
    if ((device->_PCR_msbs & 0x02) == 0x02)
    {
        device->_PCR_msbs = device->_PCR_msbs ^ 0x02;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_POWER_REF_CTRL;
    buffer[1] = device->_PCR_msbs;
    buffer[2] = device->_PCR_lsbs;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
}

void AD5593R_set_ADC_max_2x_Vref(struct AD5593R *device)
{
    device->_ADC_max = 2 * device->_Vref;
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    // check if 2x bit is on in the general purpose register
    if ((device->_GPRC_lsbs & 0x20) != 0x20)
    {
        device->_GPRC_lsbs = device->_GPRC_lsbs ^ 0x20;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_GP_CONTROL;
    buffer[1] = device->_GPRC_msbs;
    buffer[2] = device->_GPRC_lsbs;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
    device->_ADC_2x_mode = 1;
}

void AD5593R_set_ADC_max_1x_Vref(struct AD5593R *device)
{
    device->_ADC_max = device->_Vref;
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    // check if 2x bit is on in the general purpose register
    if ((device->_GPRC_lsbs & 0x20) == 0x20)
    {
        device->_GPRC_lsbs = device->_GPRC_lsbs ^ 0x20;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_GP_CONTROL;
    buffer[1] = device->_GPRC_msbs;
    buffer[2] = device->_GPRC_lsbs;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
    device->_ADC_2x_mode = 0;
}

void AD5593R_set_DAC_max_2x_Vref(struct AD5593R *device)
{
    device->_DAC_max = 2 * device->_Vref;
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    // check if 2x bit is on in the general purpose register
    if ((device->_GPRC_lsbs & 0x10) != 0x10)
    {
        device->_GPRC_lsbs = device->_GPRC_lsbs ^ 0x10;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_GP_CONTROL;
    buffer[1] = device->_GPRC_msbs;
    buffer[2] = device->_GPRC_lsbs;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
    device->_DAC_2x_mode = 1;
}

void AD5593R_set_DAC_max_1x_Vref(struct AD5593R *device)
{
    device->_DAC_max = device->_Vref;
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    // check if 2x bit is on in the general purpose register
    if ((device->_GPRC_lsbs & 0x10) == 0x10)
    {
        device->_GPRC_lsbs = device->_GPRC_lsbs ^ 0x10;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_GP_CONTROL;
    buffer[1] = device->_GPRC_msbs;
    buffer[2] = device->_GPRC_lsbs;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
    device->_DAC_2x_mode = 0;
}

void AD5593R_set_Vref(struct AD5593R *device, float Vref)
{
    device->_Vref = Vref;
    if (device->_ADC_2x_mode == 0)
    {
        device->_ADC_max = Vref;
    }
    else
    {
        device->_ADC_max = 2 * Vref;
    }

    if (device->_DAC_2x_mode == 0)
    {
        device->_DAC_max = Vref;
    }
    else
    {
        device->_DAC_max = 2 * Vref;
    }
}

void AD5593R_configure_DAC(struct AD5593R *device, int channel)
{
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }
    device->config.DACs[channel] = 1;
    uint8_t channel_byte = 1 << channel;

    // check to see if the channel is a DAC already
    if ((device->_DAC_config & channel_byte) != channel_byte)
    {
        device->_DAC_config = device->_DAC_config ^ channel_byte;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_DAC_CONFIG;
    buffer[1] = 0x00;
    buffer[2] = device->_DAC_config;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
}

void AD5593R_write_DAC(struct AD5593R *device, int channel, float voltage)
{
    // error checking
    if (device->config.DACs[channel] == 0)
    {
        printf("Channel %d is not configured as a DAC\n", channel);
        return;
    }
    if (device->_DAC_max == -1)
    {
        printf("Vref or DAC max has not been set\n");
        return;
    }
    if (voltage > device->_DAC_max)
    {
        printf("Voltage is greater than DAC max\n");
        return;
    }

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    unsigned int data_bits = (voltage / device->_DAC_max) * 4095;

    // extract the 4 most signifigant bits, and move them down to the bottom
    uint8_t data_msbs = (data_bits & 0xF00) >> 8;
    // extract the 8 least signifigant bits
    uint8_t data_lsbs = data_bits & 0xFF;

    // place the channel data in the most signifigant bits
    data_msbs = (0b00110000 | (channel << 4)) | data_msbs; // 0b00110000 is the DAC write command

    uint8_t buffer[3];
    buffer[0] = _ADAC_DAC_WRITE | channel;
    buffer[1] = data_msbs;
    buffer[2] = data_lsbs;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
}

float AD5593R_configure_ADC(struct AD5593R *device, int channel)
{
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }
    device->config.ADCs[channel] = 1;
    uint8_t channel_byte = 1 << channel;

    // check to see if the channel is an ADC already
    if ((device->_ADC_config & channel_byte) != channel_byte)
    {
        device->_ADC_config = device->_ADC_config ^ channel_byte;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_ADC_CONFIG;
    buffer[1] = 0x00;
    buffer[2] = device->_ADC_config;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }

    printf("Channel %d is now configured as an ADC\n", channel);
}

void AD5593R_configure_ADCs(struct AD5593R *device, int *channels, int num_channels)
{
    for (size_t i = 0; i < num_channels; i++)
    {
        AD5593R_configure_ADC(device, channels[i]);
    }
}

float AD5593R_read_ADC(struct AD5593R *device, uint8_t channel)
{
    if (device->config.ADCs[channel] == 0)
    {
        printf("ERROR! Channel %d is not an ADC\n", channel);
        return -1.0f;
    }

    if (device->_ADC_max == -1)
    {
        printf("Vref, or ADC_max is not defined\n");
        return -2.0f;
    }

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    uint8_t data_channel = (1 << channel);
    uint8_t adc_sequence_cmd[] = {_ADAC_ADC_SEQUENCE, 0x02, data_channel};
    i2c_write_blocking(&i2c, device->_i2c_address, adc_sequence_cmd, sizeof(adc_sequence_cmd), false);

    uint8_t adc_read_cmd[] = {_ADAC_ADC_READ};
    i2c_write_blocking(&i2c, device->_i2c_address, adc_read_cmd, sizeof(adc_read_cmd), false);

    uint16_t data_bits = 0;

    uint8_t data[2];
    i2c_read_blocking(&i2c, device->_i2c_address, data, sizeof(data), false);
    data_bits = (data[0] & 0x0F) << 8 | data[1]; // 0x0F is to mask the 4 most signifigant bits

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }

    float data_volts = (device->_ADC_max * data_bits / 4095);

    printf("Channel %d reads %.4f Volts\n", channel, data_volts);
    return data_volts;
}

// Function to read ADC values for all configured channels and return the ADC values array
float *AD5593R_read_ADCs(struct AD5593R *device)
{
    static float adc_values[_num_of_channels];

    for (size_t i = 0; i < _num_of_channels; i++)
    {
        if (device->config.ADCs[i] == 1)
        {
            device->values.ADCs[i] = AD5593R_read_ADC(device, i);
        }
        else
        {
            // Set non-configured channels to a specific value (e.g., -1) as an indication
            device->values.ADCs[i] = -1.0f;
        }

        // Assign ADC values to the provided array
        adc_values[i] = device->values.ADCs[i];
    }

    return adc_values;
}

void ADD593R_configure_GPI(struct AD5593R *device, int channel)
{
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }
    device->config.DACs[channel] = 1;
    uint8_t channel_byte = 1 << channel;

    // check to see if the channel is a gpi already
    if ((device->_GPI_config & channel_byte) != channel_byte)
    {
        device->_GPI_config = device->_GPI_config ^ channel_byte;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_GPIO_RD_CONFIG;
    buffer[1] = 0x00;
    buffer[2] = device->_GPI_config;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }

    printf("Channel %d is now configured as a GPI\n", channel);
}

void ADD593R_configure_GPIs(struct AD5593R *device, int *channels, int num_channels)
{
    for (size_t i = 0; i < num_channels; i++)
    {
        ADD593R_configure_GPI(device, channels[i]);
    }
}

void ADD593R_configure_GPO(struct AD5593R *device, int channel)
{
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }
    device->config.DACs[channel] = 1;
    uint8_t channel_byte = 1 << channel;

    // check to see if the channel is a gpo already
    if ((device->_GPO_config & channel_byte) != channel_byte)
    {
        device->_GPO_config = device->_GPO_config ^ channel_byte;
    }

    uint8_t buffer[3];
    buffer[0] = _ADAC_GPIO_WR_CONFIG;
    buffer[1] = 0x00;
    buffer[2] = device->_GPO_config;

    i2c_write_blocking(&i2c, device->_i2c_address, buffer, 3, false);

    // Disable selected device for writing
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }

    printf("Channel %d is now configured as a GPO\n", channel);
}

void ADD593R_configure_GPOs(struct AD5593R *device, int *channels, int num_channels)
{
    for (size_t i = 0; i < num_channels; i++)
    {
        ADD593R_configure_GPO(device, channels[i]);
    }
}

bool *ADD593R_read_GPIs(struct AD5593R *device)
{
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    uint8_t gpi_read_cmd[] = {_ADAC_GPIO_READ};

    i2c_write_blocking(&i2c, device->_i2c_address, gpi_read_cmd, sizeof(gpi_read_cmd), false);

    uint8_t data[2];
    i2c_read_blocking(&i2c, device->_i2c_address, data, sizeof(data), false);

    // mask bits, build the word
    uint16_t data_bits = (data[0] & 0x0F) << 8;
    data_bits = data[1] | data_bits;

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }

    for (size_t i = 0; i < _num_of_channels; i++)
    {
        if (device->config.GPIs[i] == 1)
        {
            device->values.GPI_reads[i] = (data_bits >> i) & 0x01;
        }
        else
        {
            // Set non-configured channels to a specific value (e.g., -1) as an indication
            device->values.GPI_reads[i] = -1;
        }

        return device->values.GPI_reads;
    }
}

void ADD593R_write_GPOs(struct AD5593R *device, bool *values)
{
    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    uint8_t gpo_write_cmd[] = {_ADAC_GPIO_WR_DATA, 0x00, 0x00};

    for (size_t i = 0; i < _num_of_channels; i++)
    {
        if (device->config.GPOs[i] == 1)
        {
            gpo_write_cmd[1] = gpo_write_cmd[1] | (values[i] << i);
        }
    }

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 0); // Set the GPIO pin low
    }

    i2c_write_blocking(&i2c, device->_i2c_address, _ADAC_GPIO_WR_DATA, sizeof(_ADAC_GPIO_WR_DATA), false);
    i2c_write_blocking(&i2c, device->_i2c_address, 0x00, sizeof(0x00), false);

    if (device->_a0 > -1)
    {
        gpio_put(device->_a0, 1); // Set the GPIO pin high
    }
}

// main to perform a basic test compiling for wifi chip
int main()
{
    bool my_DACs[8] = {1, 1, 1, 1, 0, 0, 0, 0};
    bool my_ADCs[8] = {0, 0, 0, 0, 1, 1, 1, 1};

    struct AD5593R device;

    stdio_init_all();

    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed");
        return -1;
    }

    AD5593R_init(&device, -1);
        
    AD5593R_enable_internal_Vref(&device);
    
    while (true)
    {
        printf("led on");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        printf("led off");
        sleep_ms(250);
    }
    return 0;
}