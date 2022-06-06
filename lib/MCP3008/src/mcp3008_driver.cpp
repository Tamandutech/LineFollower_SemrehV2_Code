#include <cmath>
#include <esp_log.h>

#include "mcp3008_driver.h"

#define TAG "Mcp3008Driver"

namespace mcp3008
{

MCPDriver::MCPDriver() : m_spi(NULL), m_spi_dev(HSPI_HOST), m_installed(false), m_channels_mask(0xFF)
{
}

MCPDriver::~MCPDriver()
{
    uninstall();
}

esp_err_t MCPDriver::install(const MCPDriver::Config &cfg)
{
    if (m_installed)
        return ESP_OK;

    esp_err_t ret;
    spi_bus_config_t buscfg /* = {.max_transfer_sz = 0, .flags = 0, .intr_flags = 0} */;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = cfg.pin_miso;
    buscfg.mosi_io_num = cfg.pin_mosi;
    buscfg.sclk_io_num = cfg.pin_sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_device_interface_config_t devcfg /* = {0} */;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = cfg.freq;
    devcfg.mode = 0;
    devcfg.spics_io_num = cfg.pin_cs;
    devcfg.queue_size = CHANNELS;

    ret = spi_bus_initialize(cfg.spi_dev, &buscfg, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = spi_bus_add_device(cfg.spi_dev, &devcfg, &m_spi);
    if (ret != ESP_OK)
    {
        spi_bus_free(cfg.spi_dev);
        return ret;
    }

    m_spi_dev = cfg.spi_dev;
    m_channels_mask = cfg.channels_mask;
    m_installed = true;
    return ESP_OK;
}

esp_err_t MCPDriver::uninstall()
{
    if (!m_installed)
        return ESP_OK;

    esp_err_t res = spi_bus_remove_device(m_spi);
    if (res != ESP_OK)
        return res;

    res = spi_bus_free(m_spi_dev);
    if (res != ESP_OK)
        return res;

    m_installed = false;
    return ESP_OK;
}

int MCPDriver::requestToChannel(int request) const
{
    if (m_channels_mask == 0xFF)
        return request;

    int requested = 0;
    for (int i = 0; i < CHANNELS; ++i)
    {
        if (((1 << i) & m_channels_mask) != 0)
        {
            if (requested++ == request)
                return i;
        }
    }

    ESP_LOGE(TAG, "Invalid requestToChannel call %d", request);
    return 0;
}

esp_err_t MCPDriver::read(std::vector<uint16_t> &results, bool differential) const
{
    int requested = 0;
    for (int i = 0; i < CHANNELS; ++i)
    {
        if (((1 << i) & m_channels_mask) != 0)
            ++requested;
    }

    const size_t orig_size = results.size();
    results.resize(orig_size + requested);

    esp_err_t res = this->read(results.data() + orig_size, differential);
    if (res != ESP_OK)
    {
        results.resize(orig_size);
        return res;
    }
    return ESP_OK;
}

esp_err_t MCPDriver::read(uint16_t *dest, bool differential) const
{
    if (!m_installed)
        return ESP_FAIL;

    int requested = 0;
    spi_transaction_t transactions[CHANNELS] /* = {0} */;
    memset(&transactions, 0, sizeof(transactions));
    for (int i = 0; i < CHANNELS; ++i)
    {
        if (((1 << i) & m_channels_mask) == 0)
            continue;

        auto &t = transactions[i];
        t.user = (void *)requested;
        t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
        t.length = 3 * 8;
        t.tx_data[0] = 1;
        t.tx_data[1] = (!differential << 7) | ((i & 0x07) << 4);

        esp_err_t res = spi_device_queue_trans(m_spi, &transactions[i], 100);
        if (res != ESP_OK)
            return res;
        ++requested;
    }

    if (requested == 0)
        return ESP_OK;

    spi_transaction_t *trans = NULL;
    for (int i = 0; i < requested; ++i)
    {
        esp_err_t res = spi_device_get_trans_result(m_spi, &trans, portMAX_DELAY);
        if (res != ESP_OK)
        {
            return res;
        }

        const int idx = (int)trans->user;
        dest[idx] = ((trans->rx_data[1] & 0x03) << 8) | trans->rx_data[2];
    }
    return ESP_OK;
}

uint16_t MCPDriver::readChannel(uint8_t channel, bool differential, esp_err_t *result) const
{
    if (!m_installed || channel >= CHANNELS)
    {
        if (result)
            *result = ESP_FAIL;
        return 0xFFFF;
    }

    spi_transaction_t trans /* = {0} */;
    memset(&trans, 0, sizeof(trans));
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.length = 3 * 8;
    trans.tx_data[0] = 1;
    trans.tx_data[1] = (!differential << 7) | ((channel & 0x07) << 4);

    esp_err_t res = spi_device_transmit(m_spi, &trans);
    if (res != ESP_OK)
    {
        if (result)
            *result = res;
        return 0xFFFF;
    }

    if (result)
        *result = ESP_OK;
    return ((trans.rx_data[1] & 0x03) << 8) | trans.rx_data[2];
}

}; // namespace mcp3008
