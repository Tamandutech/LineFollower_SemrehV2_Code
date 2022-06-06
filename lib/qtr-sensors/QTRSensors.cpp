#include "QTRSensors.h"

void QTRSensors::setTypeRC()
{
  _type = QTRType::RC;
  _maxValue = _timeout;
}

void QTRSensors::setTypeAnalog()
{
  _type = QTRType::Analog;
  _maxValue = 1024; // Arduino analogRead() returns a 10-bit value by default
}

void QTRSensors::setTypeAnalogESP()
{
  _type = QTRType::AnalogESP;
  _maxValue = 4095; // Arduino analogRead() returns a 10-bit value by default

  adc1_config_width(ADC_WIDTH_BIT_12);
}

void QTRSensors::setTypeMCP3008()
{
  _type = QTRType::MCP3008;
  _maxValue = 1023; // MCP3008 returns a 10-bit value by default
}

void QTRSensors::setSensorPins(const uint8_t *pins, uint8_t sensorCount,
                               gpio_num_t miso, gpio_num_t mosi, gpio_num_t sck,
                               gpio_num_t cs, int freq,
                               spi_host_device_t spi_dev)
{

  if (_type == QTRType::MCP3008)
  {
    mcp_cfg =
        mcp3008::MCPDriver::Config(cs, mosi, miso, sck, 0xFF, freq, spi_dev);

    ls.install(mcp_cfg);
  }

  setSensorPins(pins, sensorCount);
}

void QTRSensors::setSensorPins(const adc1_channel_t *pins, uint8_t sensorCount)
{
  if (sensorCount > QTRMaxSensors)
  {
    sensorCount = QTRMaxSensors;
  }

  adc1_channel_t *oldSensorPins = _sensorPinsESP;

  _sensorPinsESP = (adc1_channel_t *)heap_caps_realloc(
      _sensorPinsESP, sizeof(adc1_channel_t) * sensorCount, MALLOC_CAP_8BIT);

  if (_sensorPinsESP == nullptr)
  {
    heap_caps_free(oldSensorPins);
    return;
  }

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    _sensorPinsESP[i] = pins[i];
    adc1_config_channel_atten(_sensorPinsESP[i], ADC_ATTEN_DB_11);
  }

  _sensorCount = sensorCount;

  calibrationOn.initialized = false;
  calibrationOff.initialized = false;
}

void QTRSensors::setSensorPins(const uint8_t *pins, uint8_t sensorCount)
{
  if (sensorCount > QTRMaxSensors)
  {
    sensorCount = QTRMaxSensors;
  }

  uint8_t *oldSensorPins = _sensorPins;

  _sensorPins = (uint8_t *)heap_caps_realloc(
      _sensorPins, sizeof(uint8_t) * sensorCount, MALLOC_CAP_8BIT);

  if (_sensorPins == nullptr)
  {
    heap_caps_free(oldSensorPins);
    return;
  }

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    _sensorPins[i] = pins[i];
  }

  _sensorCount = sensorCount;

  calibrationOn.initialized = false;
  calibrationOff.initialized = false;
}

void QTRSensors::setTimeout(uint16_t timeout)
{
  if (timeout > 32767)
  {
    timeout = 32767;
  }
  _timeout = timeout;
  if (_type == QTRType::RC)
  {
    _maxValue = timeout;
  }
}

void QTRSensors::setSamplesPerSensor(uint8_t samples)
{
  if (samples > 64)
  {
    samples = 64;
  }
  _samplesPerSensor = samples;
}

void QTRSensors::setEmitterPin(uint8_t emitterPin)
{
  releaseEmitterPins();

  _oddEmitterPin = emitterPin;
  gpio_set_direction((gpio_num_t)_oddEmitterPin, GPIO_MODE_INPUT_OUTPUT);

  _emitterPinCount = 1;
}

void QTRSensors::setEmitterPins(uint8_t oddEmitterPin, uint8_t evenEmitterPin)
{
  releaseEmitterPins();

  _oddEmitterPin = oddEmitterPin;
  _evenEmitterPin = evenEmitterPin;
  gpio_set_direction((gpio_num_t)_oddEmitterPin, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction((gpio_num_t)_evenEmitterPin, GPIO_MODE_INPUT_OUTPUT);

  _emitterPinCount = 2;
}

void QTRSensors::releaseEmitterPins()
{
  if (_oddEmitterPin != QTRNoEmitterPin)
  {
    gpio_set_direction((gpio_num_t)_oddEmitterPin, GPIO_MODE_INPUT_OUTPUT);
    _oddEmitterPin = QTRNoEmitterPin;
  }

  if (_evenEmitterPin != QTRNoEmitterPin)
  {
    gpio_set_direction((gpio_num_t)_evenEmitterPin, GPIO_MODE_INPUT_OUTPUT);
    _evenEmitterPin = QTRNoEmitterPin;
  }

  _emitterPinCount = 0;
}

void QTRSensors::setDimmingLevel(uint8_t dimmingLevel)
{
  if (dimmingLevel > 31)
  {
    dimmingLevel = 31;
  }
  _dimmingLevel = dimmingLevel;
}

// emitters defaults to QTREmitters::All; wait defaults to true
void QTRSensors::emittersOff(QTREmitters emitters, bool wait)
{
  bool pinChanged = false;

  // Use odd emitter pin in these cases:
  // - 1 emitter pin, emitters = all
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = odd
  if (emitters == QTREmitters::All ||
      (_emitterPinCount == 2 && emitters == QTREmitters::Odd))
  {
    // Check if pin is defined and only turn off if not already off
    if ((_oddEmitterPin != QTRNoEmitterPin) &&
        gpio_get_level((gpio_num_t)_oddEmitterPin) == 1)
    {
      gpio_set_level((gpio_num_t)_oddEmitterPin, 0);
      pinChanged = true;
    }
  }

  // Use even emitter pin in these cases:
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = even
  if (_emitterPinCount == 2 &&
      (emitters == QTREmitters::All || emitters == QTREmitters::Even))
  {
    // Check if pin is defined and only turn off if not already off
    if ((_evenEmitterPin != QTRNoEmitterPin) &&
        gpio_get_level((gpio_num_t)_evenEmitterPin) == 1)
    {
      gpio_set_level((gpio_num_t)_evenEmitterPin, 0);
      pinChanged = true;
    }
  }

  if (wait && pinChanged)
  {
    if (_dimmable)
    {
      // driver min is 1 ms
      vTaskDelay(1.2 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(0.2 / portTICK_PERIOD_MS);
    }
  }
}

void QTRSensors::emittersOn(QTREmitters emitters, bool wait)
{
  bool pinChanged = false;
  uint16_t emittersOnStart = 0;

  // Use odd emitter pin in these cases:
  // - 1 emitter pin, emitters = all
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = odd
  if (emitters == QTREmitters::All ||
      (_emitterPinCount == 2 && emitters == QTREmitters::Odd))
  {
    // Check if pin is defined, and only turn on non-dimmable sensors if not
    // already on, but always turn dimmable sensors off and back on because
    // we might be changing the dimming level (emittersOnWithPin() should take
    // care of this)
    if ((_oddEmitterPin != QTRNoEmitterPin) &&
        (_dimmable || gpio_get_level((gpio_num_t)_oddEmitterPin) == 0))
    {
      emittersOnStart = emittersOnWithPin(_oddEmitterPin);
      pinChanged = true;
    }
  }

  // Use even emitter pin in these cases:
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = even
  if (_emitterPinCount == 2 &&
      (emitters == QTREmitters::All || emitters == QTREmitters::Even))
  {
    // Check if pin is defined, and only turn on non-dimmable sensors if not
    // already on, but always turn dimmable sensors off and back on because
    // we might be changing the dimming level (emittersOnWithPin() should take
    // care of this)
    if ((_evenEmitterPin != QTRNoEmitterPin) &&
        (_dimmable || (gpio_get_level((gpio_num_t)_evenEmitterPin) == 0)))
    {
      emittersOnStart = emittersOnWithPin(_evenEmitterPin);
      pinChanged = true;
    }
  }

  if (wait && pinChanged)
  {
    if (_dimmable)
    {
      // Make sure it's been at least 300 us since the emitter pin was first
      // set high before returning. (Driver min is 250 us.) Some time might
      // have already passed while we set the dimming level.
      while ((uint16_t)(esp_timer_get_time() - emittersOnStart) < 300)
      {
        vTaskDelay(0.001 / portTICK_PERIOD_MS);
      }
    }
    else
    {
      vTaskDelay(0.02 / portTICK_PERIOD_MS);
    }
  }
}

// assumes pin is valid (not QTRNoEmitterPin)
// returns time when pin was first set high (used by emittersSelect())
uint16_t QTRSensors::emittersOnWithPin(uint8_t pin)
{
  if (_dimmable && (gpio_get_level((gpio_num_t)pin) == 1))
  {
    // We are turning on dimmable emitters that are already on. To avoid
    // messing up the dimming level, we have to turn the emitters off and back
    // on. This means the turn-off delay will happen even if wait = false was
    // passed to emittersOn(). (Driver min is 1 ms.)
    gpio_set_level((gpio_num_t)pin, 0);
    vTaskDelay(1.2 / portTICK_PERIOD_MS);
  }

  gpio_set_level((gpio_num_t)pin, 1);
  uint16_t emittersOnStart = esp_timer_get_time();

  if (_dimmable && (_dimmingLevel > 0))
  {
    for (uint8_t i = 0; i < _dimmingLevel; i++)
    {
      vTaskDelay(0.001 / portTICK_PERIOD_MS);
      gpio_set_level((gpio_num_t)pin, 0);
      vTaskDelay(0.001 / portTICK_PERIOD_MS);
      gpio_set_level((gpio_num_t)pin, 1);
    }
  }

  return emittersOnStart;
}

void QTRSensors::emittersSelect(QTREmitters emitters)
{
  QTREmitters offEmitters;

  switch (emitters)
  {
  case QTREmitters::Odd:
    offEmitters = QTREmitters::Even;
    break;

  case QTREmitters::Even:
    offEmitters = QTREmitters::Odd;
    break;

  case QTREmitters::All:
    emittersOn();
    return;

  case QTREmitters::None:
    emittersOff();
    return;

  default: // invalid
    return;
  }

  // Turn off the off-emitters; don't wait before proceeding, but record the
  // time.
  emittersOff(offEmitters, false);
  uint16_t turnOffStart = esp_timer_get_time();

  // Turn on the on-emitters and wait.
  emittersOn(emitters);

  if (_dimmable)
  {
    // Finish waiting for the off-emitters emitters to turn off: make sure
    // it's been at least 1200 us since the off-emitters was turned off before
    // returning. (Driver min is 1 ms.) Some time has already passed while we
    // waited for the on-emitters to turn on.
    while ((uint16_t)(esp_timer_get_time() - turnOffStart) < 1200)
    {
      vTaskDelay(0.001 / portTICK_PERIOD_MS);
    }
  }
}

void QTRSensors::resetCalibration()
{
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    if (calibrationOn.maximum)
    {
      calibrationOn.maximum[i] = 0;
    }
    if (calibrationOff.maximum)
    {
      calibrationOff.maximum[i] = 0;
    }
    if (calibrationOn.minimum)
    {
      calibrationOn.minimum[i] = _maxValue;
    }
    if (calibrationOff.minimum)
    {
      calibrationOff.minimum[i] = _maxValue;
    }
  }
}

void QTRSensors::setCalibrationOn(uint16_t maxChannel[], uint16_t minChannel[])
{
  // (Re)allocate and initialize the arrays if necessary.
  if (!calibrationOn.initialized)
  {
    uint16_t *oldMaximum = calibrationOn.maximum;
    calibrationOn.maximum = (uint16_t *)realloc(calibrationOn.maximum,
                                                sizeof(uint16_t) * _sensorCount);
    if (calibrationOn.maximum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMaximum); // deallocate any memory used by old array
      return;
    }

    uint16_t *oldMinimum = calibrationOn.minimum;
    calibrationOn.minimum = (uint16_t *)realloc(calibrationOn.minimum,
                                                sizeof(uint16_t) * _sensorCount);
    if (calibrationOn.minimum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMinimum); // deallocate any memory used by old array
      return;
    }

    // Initialize the max and min calibrated values to values that
    // will cause the first reading to update them.
    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      calibrationOn.maximum[i] = 0;
      calibrationOn.minimum[i] = _maxValue;
    }

    calibrationOn.initialized = true;
  }

  // record the min and max calibration values
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    calibrationOn.maximum[i] = maxChannel[i];
    calibrationOn.minimum[i] = minChannel[i];
  }
}

void QTRSensors::calibrate(QTRReadMode mode)
{
  calibrateOnOrOff(calibrationOn, QTRReadMode::On);
}

void QTRSensors::calibrateOnOrOff(CalibrationData &calibration,
                                  QTRReadMode mode)
{
  uint16_t sensorValues[QTRMaxSensors];
  uint16_t maxSensorValues[QTRMaxSensors];
  uint16_t minSensorValues[QTRMaxSensors];

  // (Re)allocate and initialize the arrays if necessary.
  if (!calibration.initialized)
  {
    uint16_t *oldMaximum = calibration.maximum;
    calibration.maximum = (uint16_t *)realloc(calibration.maximum,
                                              sizeof(uint16_t) * _sensorCount);
    if (calibration.maximum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMaximum); // deallocate any memory used by old array
      return;
    }

    uint16_t *oldMinimum = calibration.minimum;
    calibration.minimum = (uint16_t *)realloc(calibration.minimum,
                                              sizeof(uint16_t) * _sensorCount);
    if (calibration.minimum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMinimum); // deallocate any memory used by old array
      return;
    }

    // Initialize the max and min calibrated values to values that
    // will cause the first reading to update them.
    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      calibration.maximum[i] = 0;
      calibration.minimum[i] = _maxValue;
    }

    calibration.initialized = true;
  }

  for (uint8_t j = 0; j < 10; j++)
  {
    read(sensorValues, mode);

    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      // set the max we found THIS time
      if ((j == 0) || (sensorValues[i] > maxSensorValues[i]))
      {
        maxSensorValues[i] = sensorValues[i];
      }

      // set the min we found THIS time
      if ((j == 0) || (sensorValues[i] < minSensorValues[i]))
      {
        minSensorValues[i] = sensorValues[i];
      }
    }
  }

  // record the min and max calibration values
  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    // Update maximum only if the min of 10 readings was still higher than it
    // (we got 10 readings in a row higher than the existing maximum).
    if (minSensorValues[i] > calibration.maximum[i])
    {
      calibration.maximum[i] = minSensorValues[i];
    }

    // Update minimum only if the max of 10 readings was still lower than it
    // (we got 10 readings in a row lower than the existing minimum).
    if (maxSensorValues[i] < calibration.minimum[i])
    {
      calibration.minimum[i] = maxSensorValues[i];
    }
  }
}

void QTRSensors::read(uint16_t *sensorValues, QTRReadMode mode)
{
  readPrivate(sensorValues);
}

void QTRSensors::readCalibrated(uint16_t *sensorValues, QTRReadMode mode)
{
  if (!calibrationOn.initialized)
    return;

  // read the needed values
  read(sensorValues, mode);

  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    uint16_t calmin, calmax;

    calmax = calibrationOn.maximum[i];
    calmin = calibrationOn.minimum[i];

    uint16_t denominator = calmax - calmin;
    int16_t value = 0;

    if (denominator != 0)
      value = (((uint16_t)sensorValues[i]) - calmin) * 1000 / denominator;

    if (value < 0)
      value = 0;
    else if (value > 1000)
      value = 1000;

    sensorValues[i] = value;
  }
  ESP_LOGD("QTRSensors", "%d | %d | %d | %d | %d | %d | %d | %d\n", sensorValues[0], sensorValues[1],
          sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5],
          sensorValues[6], sensorValues[7]);
}

// Reads the first of every [step] sensors, starting with [start] (0-indexed,
// so start = 0 means start with the first sensor). For example, step = 2,
// start = 1 means read the *even-numbered* sensors. start defaults to 0, step
// defaults to 1
void QTRSensors::readPrivate(uint16_t *sensorValues, uint8_t start,
                             uint8_t step)
{
  if (_sensorPins == nullptr && _sensorPinsESP == nullptr)
  {
    return;
  }

  switch (_type)
  {
  case QTRType::RC:
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] = _maxValue;
      // make sensor line an output (drives low briefly, but doesn't matter)
      gpio_set_direction((gpio_num_t)_sensorPins[i], GPIO_MODE_INPUT_OUTPUT);
      // drive sensor line high
      gpio_set_level((gpio_num_t)_sensorPins[i], 1);
    }

    vTaskDelay(0.001 / portTICK_PERIOD_MS);

    {
      // disable interrupts so we can switch all the pins as close to the same
      // time as possible

      // record start time before the first sensor is switched to input
      // (similarly, time is checked before the first sensor is read in the
      // loop below)
      uint32_t startTime = esp_timer_get_time();
      uint16_t time = 0;

      for (uint8_t i = start; i < _sensorCount; i += step)
      {
        // make sensor line an input (should also ensure pull-up is disabled)
        gpio_set_direction((gpio_num_t)_sensorPins[i], GPIO_MODE_INPUT_OUTPUT);
      }

      while (time < _maxValue)
      {
        // disable interrupts so we can read all the pins as close to the same
        // time as possible

        time = esp_timer_get_time() - startTime;
        for (uint8_t i = start; i < _sensorCount; i += step)
        {
          if ((gpio_get_level((gpio_num_t)_sensorPins[i]) == 0) &&
              (time < sensorValues[i]))
          {
            // record the first time the line reads low
            sensorValues[i] = time;
          }
        }
      }
    }
    return;

  case QTRType::Analog:
    // reset the values
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] = 0;
    }

    for (uint8_t j = 0; j < _samplesPerSensor; j++)
    {
      for (uint8_t i = start; i < _sensorCount; i += step)
      {
        // add the conversion result
        sensorValues[i] += adc1_get_raw((adc1_channel_t)_sensorPins[i]);
      }
    }

    // get the rounded average of the readings for each sensor
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] =
          (sensorValues[i] + (_samplesPerSensor >> 1)) / _samplesPerSensor;
    }
    return;

  case QTRType::AnalogESP:
    // reset the values
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] = 0;
    }

    for (uint8_t j = 0; j < _samplesPerSensor; j++)
    {
      for (uint8_t i = start; i < _sensorCount; i += step)
      {
        // add the conversion result
        sensorValues[i] += adc1_get_raw(_sensorPinsESP[i]);
      }
    }

    // get the rounded average of the readings for each sensor
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] =
          (sensorValues[i] + (_samplesPerSensor >> 1)) / _samplesPerSensor;
    }
    return;

  case QTRType::MCP3008:
    // reset the values
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] = 0;
    }

    for (uint8_t j = 0; j < _samplesPerSensor; j++)
    {
      for (uint8_t i = start; i < _sensorCount; i += step)
      {
        // add the conversion result
        sensorValues[i] += ls.readChannel(_sensorPins[i]);
      }
    }

    // get the rounded average of the readings for each sensor
    for (uint8_t i = start; i < _sensorCount; i += step)
    {
      sensorValues[i] =
          (sensorValues[i] + (_samplesPerSensor >> 1)) / _samplesPerSensor;
    }
    return;

  default: // QTRType::Undefined or invalid - do nothing
    return;
  }
}

uint16_t QTRSensors::readLinePrivate(uint16_t *sensorValues, QTRReadMode mode,
                                     bool invertReadings)
{
  bool onLine = false;
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000

  readCalibrated(sensorValues, mode);

  for (uint8_t i = 0; i < _sensorCount; i++)
  {
    uint16_t value = sensorValues[i];
    if (invertReadings)
    {
      value = 1000 - value;
    }

    // keep track of whether we see the line at all
    if (value > 200)
    {
      onLine = true;
    }

    // only average in values that are above a noise threshold
    if (value > 50)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (_lastPosition < (_sensorCount - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (_sensorCount - 1) * 1000;
    }
  }

  _lastPosition = avg / sum;
  return _lastPosition;
}

// the destructor frees up allocated memory
QTRSensors::~QTRSensors()
{
  releaseEmitterPins();

  if (_sensorPins)
  {
    heap_caps_free(_sensorPins);
  }
  if (_sensorPinsESP)
  {
    heap_caps_free(_sensorPinsESP);
  }
  if (calibrationOn.maximum)
  {
    heap_caps_free(calibrationOn.maximum);
  }
  if (calibrationOff.maximum)
  {
    heap_caps_free(calibrationOff.maximum);
  }
  if (calibrationOn.minimum)
  {
    heap_caps_free(calibrationOn.minimum);
  }
  if (calibrationOff.minimum)
  {
    heap_caps_free(calibrationOff.minimum);
  }
}