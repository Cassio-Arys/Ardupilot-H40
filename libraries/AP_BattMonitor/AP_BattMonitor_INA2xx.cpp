#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA2XX_ENABLED

/*
  supports INA226, INA228 and INA238 I2C battery monitors
 */

#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_INA2xx.h"

extern const AP_HAL::HAL& hal;


// INA226 specific registers
#define REG_226_CONFIG        0x00
#define  REG_226_CONFIG_DEFAULT 0x4127
#define  REG_226_CONFIG_RESET   0x8000
#define REG_226_BUS_VOLTAGE   0x02
#define REG_226_CURRENT       0x04
#define REG_226_CALIBRATION   0x05
#define REG_226_MANUFACT_ID   0xfe

// INA228 specific registers
#define REG_228_CONFIG        0x00
#define  REG_228_CONFIG_RESET   0x8000
#define REG_228_ADC_CONFIG    0x01
#define REG_228_SHUNT_CAL     0x02
#define REG_228_VBUS          0x05
#define REG_228_CURRENT       0x07
#define REG_228_MANUFACT_ID   0x3e
#define REG_228_DEVICE_ID     0x3f

// INA238 specific registers
#define REG_238_CONFIG        0x00
#define  REG_238_CONFIG_RESET   0x8000
#define REG_238_ADC_CONFIG    0x01
#define REG_238_SHUNT_CAL     0x02
#define REG_238_VBUS          0x05
#define REG_238_CURRENT       0x07
#define REG_238_MANUFACT_ID   0x3e
#define REG_238_DEVICE_ID     0x3f

#ifndef DEFAULT_BATTMON_INA2XX_MAX_AMPS
#define DEFAULT_BATTMON_INA2XX_MAX_AMPS 90.0
#endif

#ifndef DEFAULT_BATTMON_INA2XX_SHUNT
#define DEFAULT_BATTMON_INA2XX_SHUNT 0.0005
#endif

#ifndef HAL_BATTMON_INA2XX_BUS
#define HAL_BATTMON_INA2XX_BUS 0
#endif
#ifndef HAL_BATTMON_INA2XX_ADDR
#define HAL_BATTMON_INA2XX_ADDR 0
#endif

#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin

#define ADS1115_I2C_ADDR            ADS1115_ADDRESS_ADDR_GND
#define ADS1115_I2C_BUS             1

#define ADS1115_RA_CONVERSION       0x00
#define ADS1115_RA_CONFIG           0x01
#define ADS1115_RA_LO_THRESH        0x02
#define ADS1115_RA_HI_THRESH        0x03

#define ADS1115_OS_SHIFT            15
#define ADS1115_OS_INACTIVE         0x00 << ADS1115_OS_SHIFT
#define ADS1115_OS_ACTIVE           0x01 << ADS1115_OS_SHIFT

#define ADS1115_MUX_SHIFT           12
#define ADS1115_MUX_P0_N1           0x00 << ADS1115_MUX_SHIFT /* default */
#define ADS1115_MUX_P0_N3           0x01 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_N3           0x02 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_N3           0x03 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P0_NG           0x04 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_NG           0x05 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_NG           0x06 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P3_NG           0x07 << ADS1115_MUX_SHIFT

#define ADS1115_PGA_SHIFT           9
#define ADS1115_PGA_6P144           0x00 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_4P096           0x01 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_2P048           0x02 << ADS1115_PGA_SHIFT // default
#define ADS1115_PGA_1P024           0x03 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P512           0x04 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256           0x05 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256B          0x06 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256C          0x07 << ADS1115_PGA_SHIFT

#define ADS1115_MV_6P144            0.187500f
#define ADS1115_MV_4P096            0.125000f
#define ADS1115_MV_2P048            0.062500f // default
#define ADS1115_MV_1P024            0.031250f
#define ADS1115_MV_0P512            0.015625f
#define ADS1115_MV_0P256            0.007813f
#define ADS1115_MV_0P256B           0.007813f
#define ADS1115_MV_0P256C           0.007813f

#define ADS1115_MODE_SHIFT          8
#define ADS1115_MODE_CONTINUOUS     0x00 << ADS1115_MODE_SHIFT
#define ADS1115_MODE_SINGLESHOT     0x01 << ADS1115_MODE_SHIFT // default

#define ADS1115_RATE_SHIFT          5
#define ADS1115_RATE_8              0x00 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_16             0x01 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_32             0x02 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_64             0x03 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_128            0x04 << ADS1115_RATE_SHIFT // default
#define ADS1115_RATE_250            0x05 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_475            0x06 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_860            0x07 << ADS1115_RATE_SHIFT

#define ADS1115_COMP_MODE_SHIFT         4
#define ADS1115_COMP_MODE_HYSTERESIS    0x00 << ADS1115_COMP_MODE_SHIFT        // default
#define ADS1115_COMP_MODE_WINDOW        0x01 << ADS1115_COMP_MODE_SHIFT

#define ADS1115_COMP_POL_SHIFT          3
#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 << ADS1115_COMP_POL_SHIFT     // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01 << ADS1115_COMP_POL_SHIFT

#define ADS1115_COMP_LAT_SHIFT          2
#define ADS1115_COMP_LAT_NON_LATCHING   0x00 << ADS1115_COMP_LAT_SHIFT    // default
#define ADS1115_COMP_LAT_LATCHING       0x01 << ADS1115_COMP_LAT_SHIFT

#define ADS1115_COMP_QUE_SHIFT      0
#define ADS1115_COMP_QUE_ASSERT1    0x00 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT2    0x01 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT4    0x02 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_DISABLE    0x03 // default


// list of addresses to probe if I2C_ADDR is zero
const uint8_t AP_BattMonitor_INA2XX::i2c_probe_addresses[] { 0x41, 0x44, 0x45, 0x48};

const AP_Param::GroupInfo AP_BattMonitor_INA2XX::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 25, AP_BattMonitor_INA2XX, i2c_bus, HAL_BATTMON_INA2XX_BUS),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address. If this is zero then probe list of supported addresses
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 26, AP_BattMonitor_INA2XX, i2c_address, HAL_BATTMON_INA2XX_ADDR),

    // @Param: MAX_AMPS
    // @DisplayName: Battery monitor max current
    // @Description: This controls the maximum current the INS2XX sensor will work with.
    // @Range: 1 400
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("MAX_AMPS", 27, AP_BattMonitor_INA2XX, max_amps, DEFAULT_BATTMON_INA2XX_MAX_AMPS),

    // @Param: SHUNT
    // @DisplayName: Battery monitor shunt resistor
    // @Description: This sets the shunt resistor used in the device
    // @Range: 0.0001 0.01
    // @Units: Ohm
    // @User: Advanced
    AP_GROUPINFO("SHUNT", 28, AP_BattMonitor_INA2XX, rShunt, DEFAULT_BATTMON_INA2XX_SHUNT),
    
    AP_GROUPEND
};

AP_BattMonitor_INA2XX::AP_BattMonitor_INA2XX(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_INA2XX::init(void)
{
    dev = hal.i2c_mgr->get_device(0, 0x48, 100000, false, 20);
    if (!dev) {
        return;
    }
    // register now and configure in the timer callbacks
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA2XX::timer, void));
}

bool AP_BattMonitor_INA2XX::configure(DevType dtype)
{
    switch (dtype) {
    case DevType::UNKNOWN:
        return false;

    case DevType::INA226: {
        break;
    }

    case DevType::INA228: {
        break;
    }

    case DevType::INA238: {
        break;
    }

    case DevType::ADS1115: {
        // configure for MAX_AMPS
        dev_type = dtype;
        return true;
        break;
    }
        
    }
    return false;
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_INA2XX::read(void)
{
    WITH_SEMAPHORE(accumulate.sem);
    _state.healthy = accumulate.count > 0;
    if (!_state.healthy) {
        return;
    }

    _state.voltage = accumulate.volt_sum / accumulate.count;
    _state.current_amps = accumulate.current_sum / accumulate.count;
    accumulate.volt_sum = 0;
    accumulate.current_sum = 0;
    accumulate.count = 0;

    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;
    
    // update total current drawn since startup
    update_consumed(_state, dt_us);

    _state.last_time_micros = tnow;
}

/*
  read 16 bit word from register
  returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::read_word16(const uint8_t reg, int16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = int16_t(be16toh(uint16_t(data)));

    return true;
}

/*
  read 24 bit signed value from register
  returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::read_word24(const uint8_t reg, int32_t& data) const
{
    // read the appropriate register from the device
    uint8_t d[3];
    if (!dev->read_registers(reg, d, sizeof(d))) {
        return false;
    }
    // 24 bit 2s complement data. Shift into upper 24 bits of int32_t then divide by 256
    // to cope with negative numbers properly
    data = d[0]<<24 | d[1]<<16 | d[2] << 8;
    data = data / 256;

    return true;
}

/*
  write word to a register, byte swapped
  returns true if write was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { reg, uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

/*
  detect device type. This may happen well after power on if battery is
  not plugged in yet
*/
bool AP_BattMonitor_INA2XX::detect_device(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_detect_ms < 200) {
        // don't flood the bus
        return false;
    }
    last_detect_ms = now;

    WITH_SEMAPHORE(dev->get_semaphore());

    
    return configure(DevType::ADS1115);

}

float AP_BattMonitor_INA2XX::_convert_register_data_to_mv(int16_t word) const
{
    float pga;

    switch (_gain) {
    case ADS1115_PGA_6P144:
        pga = ADS1115_MV_6P144;
        break;
    case ADS1115_PGA_4P096:
        pga = ADS1115_MV_4P096;
        break;
    case ADS1115_PGA_2P048:
        pga = ADS1115_MV_2P048;
        break;
    case ADS1115_PGA_1P024:
        pga = ADS1115_MV_1P024;
        break;
    case ADS1115_PGA_0P512:
        pga = ADS1115_MV_0P512;
        break;
    case ADS1115_PGA_0P256:
        pga = ADS1115_MV_0P256;
        break;
    case ADS1115_PGA_0P256B:
        pga = ADS1115_MV_0P256B;
        break;
    case ADS1115_PGA_0P256C:
        pga = ADS1115_MV_0P256C;
        break;
    default:
        pga = 0.0f;
        DEV_PRINTF("Wrong gain");
        AP_HAL::panic("ADS1115: wrong gain selected");
        break;
    }

    return (float) word * pga;
}

#define ADS1115_CHANNELS_COUNT           6

const uint8_t AP_BattMonitor_INA2XX::_channels_number = ADS1115_CHANNELS_COUNT;

/* Only two differential channels used */
static const uint16_t mux_table[ADS1115_CHANNELS_COUNT] = {
    ADS1115_MUX_P1_N3,
    ADS1115_MUX_P2_N3,
    ADS1115_MUX_P0_NG,
    ADS1115_MUX_P1_NG,
    ADS1115_MUX_P2_NG,
    ADS1115_MUX_P3_NG
};

bool AP_BattMonitor_INA2XX::_start_conversion(uint8_t channel)
{
    struct PACKED {
        uint8_t reg;
        be16_t val;
    } config;

    config.reg = ADS1115_RA_CONFIG;
    config.val = htobe16(ADS1115_OS_ACTIVE | _gain | mux_table[channel] |
                         ADS1115_MODE_SINGLESHOT | ADS1115_COMP_QUE_DISABLE |
                         ADS1115_RATE_250);

    return dev->transfer((uint8_t *)&config, sizeof(config), nullptr, 0);
}


void AP_BattMonitor_INA2XX::timer(void)
{
    if (dev_type == DevType::UNKNOWN) {
        if (!detect_device()) {
            return;
        }
    }

    float voltage = 0, current = 0;

    switch (dev_type) {
    case DevType::UNKNOWN:
        return;

    case DevType::INA226: {
        int16_t bus_voltage16, current16;
        if (!read_word16(REG_226_BUS_VOLTAGE, bus_voltage16) ||
            !read_word16(REG_226_CURRENT, current16)) {
            failed_reads++;
            if (failed_reads > 10) {
                // device has disconnected, we need to reconfigure it
                dev_type = DevType::UNKNOWN;
            }
            return;
        }
        voltage = bus_voltage16 * voltage_LSB;
        current = current16 * current_LSB;
        break;
    }

    case DevType::INA228: {
        int32_t bus_voltage24, current24;
        if (!read_word24(REG_228_VBUS, bus_voltage24) ||
            !read_word24(REG_228_CURRENT, current24)) {
            failed_reads++;
            if (failed_reads > 10) {
                // device has disconnected, we need to reconfigure it
                dev_type = DevType::UNKNOWN;
            }
            return;
        }
        voltage = (bus_voltage24>>4) * voltage_LSB;
        current = (current24>>4) * current_LSB;
        break;
    }

    case DevType::INA238: {
        int16_t bus_voltage16, current16;
        if (!read_word16(REG_238_VBUS, bus_voltage16) ||
            !read_word16(REG_238_CURRENT, current16)) {
            failed_reads++;
            if (failed_reads > 10) {
                // device has disconnected, we need to reconfigure it
                dev_type = DevType::UNKNOWN;
            }
            return;
        }
        voltage = bus_voltage16 * voltage_LSB;
        current = current16 * current_LSB;
        break;
    }

    case DevType::ADS1115: {

    uint8_t config[2];
    be16_t val;

    if (!dev->read_registers(ADS1115_RA_CONFIG, config, sizeof(config))) {
        return;
    }

    /* check rdy bit */
    if ((config[1] & 0x80) != 0x80 ) {
        return;
    }

    if (!dev->read_registers(ADS1115_RA_CONVERSION, (uint8_t *)&val,  sizeof(val))) {
        return;
    }

    float sample = _convert_register_data_to_mv(be16toh(val));

    _samples[0].data = sample;
    _samples[0].id = 0;

    voltage = (sample * (100000 + 3900)/3900)/1000;
    //voltage = (sample)/1000;

    current = 0;

    /* select next channel */
    _channel_to_read = 2;
    _start_conversion(2);
}

    }

    failed_reads = 0;

    WITH_SEMAPHORE(accumulate.sem);
    accumulate.volt_sum += voltage;
    accumulate.current_sum += current;
    accumulate.count++;
}

#endif // AP_BATTERY_INA2XX_ENABLED
