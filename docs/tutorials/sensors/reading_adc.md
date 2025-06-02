# Using the ADS131M0x Analog to Digital Converter

This tutorial demonstrates how to use the ADS131M0x analog to digital converter with the Open Source Leg platform. This example shows how to:

- Initialize and configure an ADS131M0x ADC
- Read voltage data

## Hardware Setup

1. Connect the ADS131M0x ADC to your Raspberry Pi's SPI pins
2. Make sure the ADC is powered on and the SPI pins are connected correctly

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/reading_adc.py) is organized as follows:

### 1. Initialization

```python
--8<-- "tutorials/sensors/reading_adc.py:1:26"
```

This section:

1. Creates a data logger for recording measurements
2. Sets up a real-time loop for consistent timing
3. Initializes the ADS131M0x ADC with specified parameters
4. Gets the latest ADC data in millivolts

### 2. Main Loop

```python
--8<-- "tutorials/sensors/reading_adc.py:28:33"
```

The main loop:

1. Updates the ADC to get the latest reading
2. Logs the time and current voltage reading

## ADC Parameters

When initializing the ADS131M0x ADC, several parameters can be configured:

```python
--8<-- "tutorials/sensors/reading_adc.py:14:24"
```

### Parameter Details

1. **tag** (str):
      - Unique identifier for the ADC instance
      - Useful when using multiple ADCs
      - Default is "ADS131M0x"

2. **spi_bus** (int):
      - SPI bus number
      - Default bus on Raspberry Pi is typically `0`

3. **spi_cs** (int):
      - SPI chip select line
      - Default is `0`

4. **data_rate** (int):
      - Sampling rate in Hz
      - Default is 500 Hz

5. **clock_freq** (int):
      - SPI clock frequency in Hz
      - Default is 8192000 Hz.

6. **num_channels** (int):
      - Number of ADC channels
      - Default is 6

7. **gains** (List[int]):
      - Programmable gain values for each channel
      - Default is [1] * num_channels.

8. **voltage_reference** (float):
      - ADC reference voltage in volts
      - Default reference for ADS131M0x ADC is 1.2 V and should not be changed if using this ADC

9. **gain_error** (List[int]):
      - Gain error correction values for each channel
      - Default is None.

10. **offline** (bool):
      - Enables and disables offline mode for ADS131M0x ADC
      - If `True`, the ADC operates in offline mode, meaning that the ADC hardware does not have to be connected in order for your script to run
      - Default is `False`

## Available Properties

The ADS131M0x ADC provides the following properties:

1. **is_streaming**
      - Checks if ADC is streaming data
      - Returns `True` if streaming, `False` otherwise

2. **gain**:
      - Gets the programmable gain values for each channel
      - Returns array of gain values (np.ndarray)

3. **data**
      - Gets the latest ADC data in millivolts
      - Returns array of voltage readings for each channel (np.ndarray)


4. **data_counts**
      - Gets the latest ADC data in raw counts
      - Returns array of raw ADC counts for each channel (np.ndarray)

5. **num_channels**
      - Get the number of ADC channels.
      - Returns # of channels (int)

## Running the Example

1. Navigate to the tutorial directory:
   ```bash
   cd tutorials/sensors
   ```

2. Run the script:
   ```bash
   python reading_adc.py
   ```

3. Expected behavior:
      - ADC begins reading voltage values in mV continuously at sampling rate (default is 500 Hz)
      - Data is logged to `./logs/reading_adc_data.csv`

## Common Issues


If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
