#pragma once

#include <stdio.h>
#include <string.h>
#include <vector>
#include <driver/spi_master.h>
#include <driver/gpio.h>

namespace mcp3008 {

/**
 * \brief The MCP3008 driver.
 *
 * This class is not thread-safe, you have to make sure the methods are called
 * from one thread at a time only.
 * The install() method has to be called before you can use any other methods.
 */
class MCPDriver {
public:
    static constexpr int CHANNELS = 8; //!< Amount of channels on the chip
    static constexpr uint16_t MAX_VAL = 1023; //!< Maximum value returned by from the chip (10bits).

    /**
     * \brief The MCPDriver SPI configuration.
     */
    struct Config {
        Config(gpio_num_t pin_cs = GPIO_NUM_22, gpio_num_t pin_mosi = GPIO_NUM_23,
            gpio_num_t pin_miso = GPIO_NUM_19, gpio_num_t pin_sck = GPIO_NUM_18,
            uint8_t channels_mask = 0xFF, int freq = 1350000, spi_host_device_t spi_dev = VSPI_HOST) {
            this->freq = freq;
            this->spi_dev = spi_dev;
            this->channels_mask = channels_mask;

            this->pin_cs = pin_cs;
            this->pin_mosi = pin_mosi;
            this->pin_miso = pin_miso;
            this->pin_sck = pin_sck;
        }

        int freq; //!< SPI communication frequency
        spi_host_device_t spi_dev; //!< Which ESP32 SPI device to use.
        uint8_t channels_mask; //!< Which channels to use, bit mask:
                               //!< (1 << 0) | (1 << 2) == channels 0 and 2 only.

        gpio_num_t pin_cs;
        gpio_num_t pin_mosi;
        gpio_num_t pin_miso;
        gpio_num_t pin_sck;
    };

    MCPDriver();
    virtual ~MCPDriver(); //!< The uninstall() method is called from the destructor.

    /**
     * \brief Initialize the SPI bus. Must be called before any other methods,
     *        otherwise they will return ESP_FAIL.
     *
     * \param cfg the SPI bus configuration.
     * \return ESP_OK or any error code encountered during the inialization.
     *         Will return ESP_FAIL if called when already installed.
     */
    esp_err_t install(const Config& cfg = Config());

    /**
     * \brief Free the SPI bus. Must be called last, or other methods return ESP_FAIL.
     *
     * \return ESP_OK or any error code encountered during the freeing.
     *         Will return ESP_FAIL if called when not installed.
     */
    esp_err_t uninstall();

    uint8_t getChannelsMask() const { return m_channels_mask; } //!< Get the channel mask, specified in Config::channels_mask

    /**
     * \brief Read values from the chip. Returns values in range <0; MCPDriver::MAX_VAL>.
     *
     * \param results the results will be APPENDED to this vector.
     *        It will be unchanged unless the ESP_OK result is returned (except possibly its capacity).
     *        Between 0 and MCPDriver::CHANNELS values are appended, depending on Config::channels_mask.
     * \param differential return differential readings, as specified in the MCP3008 datasheet.
     * \return ESP_OK or any error code encountered during reading.
     *         Will return ESP_FAIL if called when not installed.
     */
    esp_err_t read(std::vector<uint16_t>& results, bool differential = false) const;

    /**
     * \brief See the other read(std::vector<uint16_t>&, bool) const method.
     *
     * \param dest array MUST be big enough to accomodate all the channels specified by Config::channels_mask!
     */
    esp_err_t read(uint16_t *dest, bool differential = false) const;

    /**
     * \brief Read a single channel from the chip. Returns value is in range <0; MCPDriver::MAX_VAL>.
     *
     * \param differential return differential readings, as specified in the MCP3008 datasheet.
     * \param result result code will be written here, may be null.
     * \return measured value or 0xFFFF on error.
     */
    uint16_t readChannel(uint8_t channel, bool differential = false, esp_err_t *result = nullptr) const;

protected:
    int requestToChannel(int request) const;

private:
    MCPDriver(const MCPDriver&) = delete;

    spi_device_handle_t m_spi;
    spi_host_device_t m_spi_dev;
    bool m_installed;
    uint8_t m_channels_mask;
};

}; // namespace mcp3008
