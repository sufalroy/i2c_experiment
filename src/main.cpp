#include <functional>
#include <iostream>
#include <array>

#include <spdlog/spdlog.h>

#include <linux/i2c-dev.h>

extern "C" {
#include <i2c/smbus.h>
}

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

struct Linux_i2c
{
	using handle_type = int;

	void close(int handle)
	{
		::close(handle);
	}

	int open_i2c_device(const int adapter_nr, const int deviceid)
	{
		const auto filehandle = open(fmt::format("/dev/i2c-{}", adapter_nr).c_str(), O_RDWR);
		if(filehandle < 0)
		{
			spdlog::error("Unable to open i2c-{} device for R/W", adapter_nr);
			exit(1);
		}

		if(ioctl(filehandle, I2C_SLAVE, deviceid) < 0)
		{
			spdlog::error("Unable to set I2C_SLAVE addr to {}", deviceid);
			exit(1);
		}

		return filehandle;
	}

	template<typename Iter>
	void smbus_read_block(int device, std::uint8_t address, Iter buffer_begin, Iter buffer_end)
	{
		spdlog::warn("Read block data is not implemented, falling back to 1 byte at a time");
		read_block_1_byte_at_a_time(device, address, buffer_begin, buffer_end);
	}

	auto read_byte(int device, std::uint8_t reg) const ->  std::uint8_t 
	{
		const auto res = i2c_smbus_read_byte_data(device, reg);
		if(res < 0)
		{
			spdlog::error("Error reading byte of data from {} (err result: {})", reg, res);
		}
		return static_cast<std::uint8_t>(res);
	}

	auto read_byte(int device) -> std::uint8_t
	{
		const auto res = i2c_smbus_read_byte(device);
		if(res < 0)
		{
			spdlog::error("Error reading byte of data from (err result: {})", res);
		}
		return static_cast<std::uint8_t>(res);
	}

	template<typename Iter>
	void i2c_read_block(int device, std::uint8_t address, Iter buffer_begin, Iter buffer_end)
	{
		const auto bytes_to_read = std::distance(buffer_begin, buffer_end);
		if(bytes_to_read > I2C_SMBUS_BLOCK_MAX)
		{
			spdlog::error("Error, cannot even try to read more than 255 bytes");
		}
		auto result = i2c_smbus_read_i2c_block_data(device, address, static_cast<std::uint8_t>(std::min(bytes_to_read, I2C_SMBUS_BLOCK_MAX)), &(*buffer_begin));
		if(result < 0)
		{
			spdlog::error("Error reading block of data from {} (err result: {})", address, result);
			spdlog::error("Error description: '{}'", strerror(errno));
		}
	}

	template<typename Iter>
	void read_block_1_byte_at_a_time(int device, std::uint8_t starting_address, Iter buffer_begin, Iter buffer_end)
	{
		spdlog::trace("Reading {} bytes from {:02x} starting at {:02x}", std::distance(buffer_begin, buffer_end), device, starting_address);
		for(auto address = starting_address; buffer_begin != buffer_end; ++address, ++buffer_begin)
		{
			*buffer_begin = read_byte(device, address);
		}
	}

	void write_byte(int device, std::uint8_t reg, std::uint8_t value)
	{
		const auto res = i2c_smbus_write_byte_data(device, reg, value);
		if(res < 0)
		{
			spdlog::error("Error writing byte of data reg {} (err result: {})", reg, res);
		}
	}

	template<typename Iter>
	void write_block_1_byte_at_a_time(int device, std::uint8_t starting_address, Iter buffer_begin, Iter buffer_end)
	{
		spdlog::trace("Writing {} bytes from {:02x} starting at {:02x}", std::distance(buffer_begin, buffer_end), device, starting_address );
		for(auto address = starting_address; buffer_begin != buffer_end; ++address, ++buffer_begin)
		{
			write_byte(device, address, *buffer_begin);
		}
	}

	template<typename Iter>
	void i2c_write_block(int device, std::uint8_t starting_address, Iter buffer_begin, Iter buffer_end)
	{
		const auto bytes_to_write = std::distance(buffer_begin, buffer_end);

		if(bytes_to_write > I2C_SMBUS_BLOCK_MAX)
		{
			spdlog::error("Error, cannot even try to write more than {} bytes, attempted {}", I2C_SMBUS_BLOCK_MAX, bytes_to_write);
		}

		spdlog::trace("Writing {} bytes from {:02x} starting at {:02x}", bytes_to_write, device, starting_address);
		i2c_smbus_write_i2c_block_data(device, starting_address, static_cast<std::uint8_t>(std::min(bytes_to_write, I2C_SMBUS_BLOCK_MAX)), &(*buffer_begin));
	}

	template<typename Iter>
	void smbus_write_block(int device, std::uint8_t starting_address, Iter buffer_begin, Iter buffer_end)
	{
		spdlog::warn("smbus write block not implemented, falling back to 1 byte at a time");
		write_block_1_byte_at_a_time(device, starting_address, buffer_begin, buffer_end);
	}

	void send_command(int device, std::uint8_t value)
	{
		const auto res = i2c_smbus_write_byte_data(device, 0, value);
		if(res < 0)
		{
			spdlog::error("Error writing command data '{}' (err result: {})", value, res);
		}
	}

};

enum struct Block_Mode
{
	one_byte_at_a_time,
	smbus_block_protocol,
	i2c_multi_byte
};

template<typename Driver, Block_Mode read_mode>
struct i2c_device
{
	Driver driver;

	typename Driver::handle_type handle{};

	i2c_device(const int adapter_nr, const int deviceid)
		: handle(driver.open_i2c_device(adapter_nr, deviceid))
	{
	}

	~i2c_device()
	{
		driver.close(handle);
	}
	
	i2c_device &operator=(const i2c_device &) = delete;
	i2c_device &operator=(i2c_device &&) = delete;
	i2c_device(const i2c_device &) = delete;
	i2c_device(i2c_device &&) = delete;

	[[nodiscard]] auto read_byte(std::uint8_t reg) const -> std::uint8_t
	{
		return driver.read_byte(handle, reg);
	}

	[[nodiscard]] auto read_block(std::uint8_t reg) const
	{
		return driver.read_block(handle, reg);
	}

	void read_block(const std::uint8_t starting_address, std::uint8_t* begin_iterator, std::uint8_t* end_iterator)
	{
		switch(read_mode)
		{
			case Block_Mode::one_byte_at_a_time:
				driver.read_block_1_byte_at_a_time(handle, starting_address, begin_iterator, end_iterator);
				return;
			case Block_Mode::smbus_block_protocol:
				driver.smbus_read_block(handle, starting_address, begin_iterator, end_iterator);
				return;
			case Block_Mode::i2c_multi_byte:
				driver.i2c_read_block(handle, starting_address, begin_iterator, end_iterator);
				return;
		}
	}

	void write_block(const std::uint8_t starting_address, std::uint8_t* begin_iterator, std::uint8_t* end_iterator)
	{
		switch(read_mode)
		{
			case Block_Mode::one_byte_at_a_time:
				driver.write_block_1_byte_at_a_time(handle, starting_address, begin_iterator, end_iterator);
				return;
			case Block_Mode::smbus_block_protocol:
				driver.smbus_write_block(handle, starting_address, begin_iterator, end_iterator);
				return;
			case Block_Mode::i2c_multi_byte:
				driver.i2c_write_block(handle, starting_address, begin_iterator, end_iterator);
				return;
		}
	}

	void write_byte(std::uint8_t reg, std::uint8_t value)
	{
		driver.write_byte(handle, reg, value);
	}

	void send_command(std::uint8_t value)
	{
		driver.send_command(handle, value);
	}
};

constexpr auto MPU6050_I2C_ADDRESS_AD0 = 0x68; // MPU6050 default i2c address w/ AD0 low

constexpr auto MPU6050_SELF_TEST_X = 0x0D;  // Self test factory calibrated values register
constexpr auto MPU6050_SELF_TEST_Y = 0x0E;  // Self test factory calibrated values register
constexpr auto MPU6050_SELF_TEST_Z = 0x0F;  // Self test factory calibrated values register
constexpr auto MPU6050_SELF_TEST_A = 0x10;  // Self test factory calibrated values register

constexpr auto MPU6050_SMPLRT_DIV     = 0x19; // Sample Rate Divisor register 
constexpr auto MPU6050_CONFIG         = 0x1A; // General config register
constexpr auto MPU6050_GYRO_CONFIG    = 0x1B; // Gyroscope specific config register
constexpr auto MPU6050_ACCEL_CONFIG   = 0x1C; // Accelerometer specific config register
constexpr auto MPU6050_INT_PIN_CONFIG = 0x37; // Interrupt pin configuration register
constexpr auto MPU6050_ACCEL_OUT      = 0x3B; // Base address for accelaration data reads
constexpr auto MPU6050_TEMP_OUT       = 0x41; // Temperature data high byte register
constexpr auto MPU6050_GYRO_OUT       = 0x43; // Base address for gyro data reads
constexpr auto MPU6050_SIG_PATH_RESET = 0x68; // Register to reset sensor signal paths
constexpr auto MPU6050_USER_CTRL      = 0x6A; // FIFO and I2C Master control register
constexpr auto MPU6050_PWR_MGMT_1     = 0x6B; // Primary power/sleep control register
constexpr auto MPU6050_PWR_MGMT_2     = 0x6C; // Secondary power/sleep control register
constexpr auto MPU6050_WHO_AM_I       = 0x75; // Divice ID register

typedef enum fsync_out
{
  MPU6050_FSYNC_OUT_DISABLED,
  MPU6050_FSYNC_OUT_TEMP,
  MPU6050_FSYNC_OUT_GYROX,
  MPU6050_FSYNC_OUT_GYROY,
  MPU6050_FSYNC_OUT_GYROZ,
  MPU6050_FSYNC_OUT_ACCELX,
  MPU6050_FSYNC_OUT_ACCELY,
  MPU6050_FSYNC_OUT_ACCEL_Z,
} mpu6050_fsync_out_t;


typedef enum clock_select
{
  MPU6050_INTR_8MHz,
  MPU6050_PLL_GYROX,
  MPU6050_PLL_GYROY,
  MPU6050_PLL_GYROZ,
  MPU6050_PLL_EXT_32K,
  MPU6050_PLL_EXT_19MHz,
  MPU6050_STOP = 7,
} mpu6050_clock_select_t;

// Accelerometer Range.
typedef enum
{
	MPU6050_RANGE_2_G  = 0b00,  //< +/- 2g (default value)
	MPU6050_RANGE_4_G  = 0b01,  //< +/- 4g
	MPU6050_RANGE_8_G  = 0b10,  //< +/- 8g
	MPU6050_RANGE_16_G = 0b11,  //< +/- 16g
} mpu6050_accel_range_t;

// Gyroscope Range
typedef enum
{
	MPU6050_RANGE_250_DEG,  //< +/- 250 deg/s (default value)
	MPU6050_RANGE_500_DEG,  //< +/- 500 deg/s
	MPU6050_RANGE_1000_DEG, //< +/- 1000 deg/s
	MPU6050_RANGE_2000_DEG, //< +/- 2000 deg/s
} mpu6050_gyro_range_t;

// LPF Bandwidth
typedef enum 
{
	MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
	MPU6050_BAND_184_HZ, ///< 184 Hz
	MPU6050_BAND_94_HZ,  ///< 94 Hz
	MPU6050_BAND_44_HZ,  ///< 44 Hz
	MPU6050_BAND_21_HZ,  ///< 21 Hz
	MPU6050_BAND_10_HZ,  ///< 10 Hz
	MPU6050_BAND_5_HZ,   ///< 5 Hz
} mpu6050_bandwidth_t;

// Periodic Measurement Options
typedef enum 
{
	MPU6050_CYCLE_1_25_HZ, //< 1.25 Hz
	MPU6050_CYCLE_5_HZ,    //< 5 Hz
	MPU6050_CYCLE_20_HZ,   //< 20 Hz
	MPU6050_CYCLE_40_HZ,   //< 40 Hz
} mpu6050_cycle_rate_t;

std::uint16_t merge_bytes(std::uint8_t LSB, std::uint8_t MSB) 
{
	return static_cast<std::uint16_t>((( LSB & 0xFF) << 8) | MSB);
}

std::int16_t two_complement_to_int(std::uint8_t LSB, std::uint8_t MSB) 
{	
	std::int16_t signed_int = 0;
	std::uint16_t word;

	word = merge_bytes(LSB, MSB);

	if((word & 0x8000) == 0x8000) { // negative number
		signed_int = static_cast<std::int16_t>(-(~word));
	} else {
		signed_int = static_cast<std::int16_t>(word & 0x7fff);
	}

	return signed_int;
}

template<typename Driver>
struct mpu6050_imu : i2c_device<Driver, Block_Mode::i2c_multi_byte>
{
	static constexpr int mpu6050_imu_address = MPU6050_I2C_ADDRESS_AD0;

	mpu6050_imu(const int adapter)
		: i2c_device<Driver, Block_Mode::i2c_multi_byte>{adapter, mpu6050_imu_address}
	{
		this->write_byte(MPU6050_SMPLRT_DIV, 0x07);
		this->write_byte(MPU6050_CONFIG, 0x00);
		this->write_byte(MPU6050_PWR_MGMT_1, 0b00000010);
		this->write_byte(MPU6050_PWR_MGMT_2, 0x00);	
	}

	std::array<std::uint8_t, 8> buffer{};

	void sync_buffer()
	{
		this->read_block(0, begin(buffer), end(buffer));
	}

	[[nodiscard]] constexpr std::uint8_t get_bufferd_byte(std::size_t offset) const noexcept
	{
		return buffer[offset];
	}

	float temperature() const
	{
		auto hi = this->read_byte(MPU6050_TEMP_OUT);
		auto lo = this->read_byte(MPU6050_TEMP_OUT + 1);

		auto temp = two_complement_to_int(hi, lo);
		
		return  static_cast<float>(temp/340 + 36.53);
	}

	float accel_x() const
	{
		auto hi = this->read_byte(MPU6050_ACCEL_OUT);
		auto lo = this->read_byte(MPU6050_ACCEL_OUT + 1);
		
		auto x_accel = two_complement_to_int(hi, lo);

		return static_cast<float>(x_accel) / 16384;
	}

	float accel_y() const
	{
		auto hi = this->read_byte(MPU6050_ACCEL_OUT + 2);
		auto lo = this->read_byte(MPU6050_ACCEL_OUT + 3);
		
		auto y_accel = two_complement_to_int(hi, lo);

		return static_cast<float>(y_accel) / 16384;
	}

	float accel_z() const
	{
		auto hi = this->read_byte(MPU6050_ACCEL_OUT + 4);
		auto lo = this->read_byte(MPU6050_ACCEL_OUT + 5);
		
		auto z_accel = two_complement_to_int(hi, lo);

		return static_cast<float>(z_accel) / 16384;
	}

	float gyro_x() const
	{
		auto hi = this->read_byte(MPU6050_GYRO_OUT);
		auto lo = this->read_byte(MPU6050_GYRO_OUT + 1);
		
		auto x_gyro = two_complement_to_int(hi, lo);

		return static_cast<float>(x_gyro) / 131;
	}

	float gyro_y() const
	{
		auto hi = this->read_byte(MPU6050_GYRO_OUT + 2);
		auto lo = this->read_byte(MPU6050_GYRO_OUT + 3);
		
		auto y_gyro = two_complement_to_int(hi, lo);

		return static_cast<float>(y_gyro) / 131;
	}

	float gyro_z() const
	{
		auto hi = this->read_byte(MPU6050_GYRO_OUT + 4);
		auto lo = this->read_byte(MPU6050_GYRO_OUT + 5);
		
		auto z_gyro = two_complement_to_int(hi, lo);

		return static_cast<float>(z_gyro) / 131;
	}
};

int main()
{
	spdlog::info("Starting i2c Experiments");
	spdlog::set_level(spdlog::level::debug);
	mpu6050_imu<Linux_i2c> imu(1);
	
	while(true)
	{
		spdlog::info("Gx : {}°s, Gy : {}°s, Gz : {}°s, Ax : {}g, Ay : {}g, Az : {}g", imu.gyro_x(), imu.gyro_y(), imu.gyro_z(), imu.accel_x(), imu.accel_y(), imu.accel_z());

		usleep(1000000);
	}
}
