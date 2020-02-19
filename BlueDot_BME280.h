

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"
#include "SPI.h"

#define BME280_CHIP_ID 0xD0
#define BME280_CTRL_HUM 0xF2
#define BME280_CTRL_MEAS 0xF4
#define BME280_CONFIG 0xF5
#define BME280_PRESSURE_MSB 0xF7
#define BME280_PRESSURE_LSB 0xF8
#define BME280_PRESSURE_XLSB 0xF9
#define BME280_TEMPERATURE_MSB 0xFA
#define BME280_TEMPERATURE_LSB 0xFB
#define BME280_TEMPERATURE_XLSB 0xFC
#define BME280_HUMIDITY_MSB 0xFD
#define BME280_HUMIDITY_LSB 0xFE

enum Coefficients
{
	BME280_DIG_T1_LSB = 0x88,
	BME280_DIG_T1_MSB = 0x89,
	BME280_DIG_T2_LSB = 0x8A,
	BME280_DIG_T2_MSB = 0x8B,
	BME280_DIG_T3_LSB = 0x8C,
	BME280_DIG_T3_MSB = 0x8D,

	BME280_DIG_P1_LSB = 0x8E,
	BME280_DIG_P1_MSB = 0x8F,
	BME280_DIG_P2_LSB = 0x90,
	BME280_DIG_P2_MSB = 0x91,
	BME280_DIG_P3_LSB = 0x92,
	BME280_DIG_P3_MSB = 0x93,
	BME280_DIG_P4_LSB = 0x94,
	BME280_DIG_P4_MSB = 0x95,
	BME280_DIG_P5_LSB = 0x96,
	BME280_DIG_P5_MSB = 0x97,
	BME280_DIG_P6_LSB = 0x98,
	BME280_DIG_P6_MSB = 0x99,
	BME280_DIG_P7_LSB = 0x9A,
	BME280_DIG_P7_MSB = 0x9B,
	BME280_DIG_P8_LSB = 0x9C,
	BME280_DIG_P8_MSB = 0x9D,
	BME280_DIG_P9_LSB = 0x9E,
	BME280_DIG_P9_MSB = 0x9F,

	BME280_DIG_H1 = 0xA1,
	BME280_DIG_H2_LSB = 0xE1,
	BME280_DIG_H2_MSB = 0xE2,
	BME280_DIG_H3 = 0xE3,
	BME280_DIG_H4_MSB = 0xE4,
	BME280_DIG_H4_LSB = 0xE5,
	BME280_DIG_H5_MSB = 0xE6,
	BME280_DIG_H6 = 0xE7,
};

struct BME280_Coefficients
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
};

// Recomended settings
// weather monitoring:
//   T,P,H Filter::F1, no IIRFilter
//   Forced (1/min)
// Indoor navigation
//   P F16, T F2, H F1, IIR 16x
//   Normal mode with t_Standby 0.5ms

struct DeviceParameter
{
	enum class CommunicationMode : uint8_t
	{
		I2C,
		SPI_SOFT,
		SPI_HARD
	};
	enum class Filter : uint8_t
	{
		Off = 0b000, //:      factor 0 (Disable humidity measurement)
		F1 = 0b001, //:      factor 1
		F2 = 0b010, //:      factor 2
		F4 = 0b011, //:      factor 4
		F8 = 0b100, //:      factor 8
		F16 = 0b101 //:      factor 16 (default value)
	};
	enum class IIRFilter : uint8_t
	{
		Off = 0b000, //:      factor 0 (filter off)
		F2 = 0b001, //:      factor 2
		F4 = 0b010, //:      factor 4
		F8 = 0b011, //:      factor 8
		F16 = 0b100 //:      factor 16 (default value)
	};
	enum class SensorMode : uint8_t
	{
		Sleep = 0b00, //:     In sleep mode no measurements are performed, but power consumption is at a minimum
		Forced = 0b01, //:     In forced mode a single measured is performed and the device returns automatically to sleep mode
		Normal = 0b11 //:     In normal mode the sensor measures continually (default value)
	};

	CommunicationMode communication = CommunicationMode::I2C; // i2c
	int8_t SPI_cs;
	int8_t SPI_sck;
	int8_t SPI_mosi;
	int8_t SPI_miso;
	uint8_t I2CAddress = 0x76;
	// NodeMCU board; SDA = GPIO0 = D3; SCL = GPIO2 = D4
	// MakerHawk ESP32 SDA 4, SCL 15
	int8_t I2C_SDA = -1;
	int8_t I2C_SCL = -1;

	SensorMode sensorMode = SensorMode::Normal; //normal mode
	IIRFilter IIRfilter = IIRFilter::F16;
	Filter tempOversampling = Filter::F16;
	Filter pressOversampling = Filter::F16;
	Filter humidOversampling = Filter::F16;
	uint16_t pressureSeaLevel = 1013.25;
	int16_t tempOutsideCelsius = 15;
	int16_t tempOutsideFahrenheit = 999;

	/// Compute maximum delay (ms) after a forced read
	uint32_t forcedReadDelay()
	{
		const uint8_t filterFactor[] = {0, 1, 2, 4, 8, 16};
		const uint8_t temp_osr = filterFactor[static_cast<uint8_t>(tempOversampling)];
		const uint8_t pres_osr = filterFactor[static_cast<uint8_t>(pressOversampling)];
		const uint8_t hum_osr = filterFactor[static_cast<uint8_t>(humidOversampling)];
		// section 9.1 datasheet bme280
		const auto MeasOffset = 1250;
		const auto MeasDuration = 2300;
		const auto PressureHumMeasOffset = 575;
		const auto ScalingFactor = 1000;
		const uint32_t delay_ms =
			(MeasOffset + (MeasDuration * (temp_osr + pres_osr + hum_osr)) +
			 +PressureHumMeasOffset + PressureHumMeasOffset) /
			ScalingFactor;

		return delay_ms;
	}
};

class BlueDot_BME280
{
public:
	DeviceParameter parameter;

	BlueDot_BME280();

	void forceRead(); /// triggers a forced read and sleep so that a following read is valid

	uint8_t init();
	uint8_t checkID();
	uint8_t readByte(byte reg);
	uint8_t spiTransfer(uint8_t data);
	void writeByte(byte reg, byte value);
	void writeIIRFilter();
	void readCoefficients();
	void writeCTRLMeas();
	float readPressure();
	float readTempC();
	float readTempF();
	float readHumidity();
	float readAltitudeFeet();
	float readAltitudeMeter();
	float convertTempKelvin();

private:
	int32_t t_fine;
	BME280_Coefficients bme280_coefficients;
	void setSensorMode(DeviceParameter::SensorMode mode);
};
