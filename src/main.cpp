#include <functional>
#include <iostream>
#include <array>

#include <spdlog/spdlog.h>

#include <linux/i2c-dev.h>

extern "C" {
#include <i2c/smbus.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
}

#include <sys/socket.h>
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
	void smbus_read_block(int device, std::uint8_t address, Iter buffer_begin, Iter buffer_end) const
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

	auto read_byte(int device) const -> std::uint8_t
	{
		const auto res = i2c_smbus_read_byte(device);
		if(res < 0)
		{
			spdlog::error("Error reading byte of data from (err result: {})", res);
		}
		return static_cast<std::uint8_t>(res);
	}

	template<typename Iter>
	void i2c_read_block(int device, std::uint8_t address, Iter buffer_begin, Iter buffer_end) const
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
	void read_block_1_byte_at_a_time(int device, std::uint8_t starting_address, Iter buffer_begin, Iter buffer_end) const
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

	void read_block(const std::uint8_t starting_address, std::uint8_t* begin_iterator, std::uint8_t* end_iterator) const
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
constexpr auto MPU6050_ACCEL_OUT      = 0x3B; // Base address for accelerometer data reads
constexpr auto MPU6050_TEMP_OUT       = 0x41; // Temperature data high byte register
constexpr auto MPU6050_GYRO_OUT       = 0x43; // Base address for gyro data reads
constexpr auto MPU6050_SIG_PATH_RESET = 0x68; // Register to reset sensor signal paths
constexpr auto MPU6050_USER_CTRL      = 0x6A; // FIFO and I2C Master control register
constexpr auto MPU6050_PWR_MGMT_1     = 0x6B; // Primary power/sleep control register
constexpr auto MPU6050_PWR_MGMT_2     = 0x6C; // Secondary power/sleep control register
constexpr auto MPU6050_WHO_AM_I       = 0x75; // Divice ID register


static std::uint16_t merge_bytes(std::uint8_t LSB, std::uint8_t MSB) 
{
	return static_cast<std::uint16_t>((( LSB & 0xFF) << 8) | MSB);
}

std::int16_t two_complement_to_int(std::uint8_t LSB, std::uint8_t MSB) 
{	
	std::int16_t signed_int = 0;
	std::uint16_t word = merge_bytes(LSB, MSB);

	if((word & 0x8000) == 0x8000) { 
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
		 this->write_byte(MPU6050_PWR_MGMT_1, 0x80); 
        usleep(100000);
        
        this->write_byte(MPU6050_PWR_MGMT_1, 0x00);
        usleep(100000);
    
        this->write_byte(MPU6050_PWR_MGMT_1, 0x01); 
        usleep(100000);

        this->write_byte(MPU6050_SMPLRT_DIV, 0x00);  
        usleep(100000);

        this->write_byte(MPU6050_CONFIG, 0x00); 
        usleep(100000);

        this->write_byte(MPU6050_GYRO_CONFIG, 0b00000); 
        usleep(100000);

        this->write_byte(MPU6050_ACCEL_CONFIG, 0b00000); 
        usleep(100000);

        this->write_byte(MPU6050_INT_PIN_CONFIG, 0x22); 
        usleep(100000);	
	}

	void reset()
	{
		this->write_byte(MPU6050_PWR_MGMT_1, 0x80);
        usleep(100000);

        this->write_byte(MPU6050_PWR_MGMT_1, 0x00);
        usleep(100000);
	}

	[[nodiscard]] double temperature() const
    {
        auto hi = this->read_byte(MPU6050_TEMP_OUT);
		auto lo = this->read_byte(MPU6050_TEMP_OUT + 1);
        
        auto temp_data = two_complement_to_int(hi, lo);
        return (temp_data/340 + 36.53);
    }

	[[nodiscard]] std::array<double, 3> acceleration() const
    {
        std::array<std::uint8_t, 6> buffer{};
        std::array<double, 3> res = {0.0, 0.0, 0.0};
        this->read_block(MPU6050_ACCEL_OUT, begin(buffer), end(buffer));        

        for(std::size_t i = 0, j = 0; j  < buffer.size() - 1; i++, j += 2)
        {
            res[i] = (two_complement_to_int(buffer[j], buffer[j + 1]) / std::pow(2, 15)) * 2;
        }
        return res;
    }

    [[nodiscard]] std::array<double, 3> angular_velocity() const
    {
        std::array<std::uint8_t, 6> buffer{};
        std::array<double, 3> res = {0.0, 0.0, 0.0};
        this->read_block(MPU6050_GYRO_OUT, begin(buffer), end(buffer));

        for(std::size_t i = 0, j = 0; j  < buffer.size() - 1 ; i++, j += 2)
        {
            res[i] = (two_complement_to_int(buffer[j], buffer[j + 1]) / std::pow(2, 15)) * 250;
        }
        return res;
	}

};

int main(/*int argc, char** argv*/)
{
	spdlog::info("Starting Experiments");
	spdlog::set_level(spdlog::level::debug);

	/*
	int len = 8, flags = IREQ_CACHE_FLUSH, max_rsp = 255;
	char addr[19] = {0}, name[248] = {0};

	int dev_id = hci_get_route(NULL);
	int sock = hci_open_dev(dev_id);

	if (dev_id < 0 || sock < 0)
	{
		spdlog::error("opening socket");
		exit(1);
	}

	inquiry_info *ii = new inquiry_info;

	int num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
	if(num_rsp < 0)
	{
		spdlog::error("hci_inquiry");
	}

	for(int i = 0; i < num_rsp; i++)
	{
		ba2str(&(ii+i)->bdaddr, addr);
		memset(name, 0, sizeof(name));
		if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), name, 0) < 0)
		{
			strcpy(name, "[unknown]");
		}
		
		spdlog::info("Address: {} Name: {}", addr, name);
	}

	delete ii;
	close(sock);
	*/

	mpu6050_imu<Linux_i2c> imu(1);
	
	while (true) 
	{
		auto accel = imu.acceleration();
		// auto omega = imu.angular_velocity();
		spdlog::info("ax = {} g, ay = {} g, az = {} g", accel[0], accel[1], accel[2]);
		// spdlog::info("wx = {} °/s, wy = {} °/s, wz = {} °/s", omega[0], omega[1], omega[2]);
		usleep(100000);
    }
}
