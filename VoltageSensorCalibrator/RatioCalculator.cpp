#include <iostream>
#include <cmath>

float CalculateVoltagePrimaryResistor(const float pin_voltage, const float sensor_output_ratio, const int32_t primary_resistance, const int32_t burden_resistance) {
    
    return pin_voltage * primary_resistance / (burden_resistance * sensor_output_ratio);
}

int main() {
    constexpr int primary_resistance = 4700; // Chosen resistance for primary resistor of voltage sensor, dimensioned to 10mA at nominal measurement voltage.
    constexpr int primary_coil_resistance = 250; // Internal resistance of primary coil of voltage sensor.
    constexpr float primary_resistance_ratio = float(primary_coil_resistance) / primary_resistance; // Ratio between primary coil resistance and primary resistor.
    constexpr int burden_resistance = 33; // Sets the gain of the voltage sensor. Lower resistances are preferred for higher accuracy.
    constexpr float pin_voltage = 0.930688f; // voltage in Volts measured directly at ADS1115 pin input.
    constexpr float measured_voltage_primary_resistor = 51.162f; // voltage drop in Volts measured across resistor connected to primary coil of voltage sensor.
    float sensor_output_ratio = 2.50f; // Datasheet reference value for initial iteration
    float calculated_voltage = CalculateVoltagePrimaryResistor(pin_voltage, sensor_output_ratio, primary_resistance, burden_resistance);

    // Both the burden resistance and sensor output ratio have tolerances that affect the output, but, by assuming the burden resistance is accurate, we can find vary 
    // the sensor output ratio until the calculated voltage matches the measured voltage across the primary resistor. This is a simple algorithm 
    // that will converge on the correct sensor output ratio, such that the calculated voltage matches the voltage drop across the primary resistor.
    while (std::abs(measured_voltage_primary_resistor - calculated_voltage) > 0.001f) {
        if (calculated_voltage > measured_voltage_primary_resistor) {
            sensor_output_ratio += 0.0001f; // Increasing the ratio will decrease the next calculated voltage
        } else {
            sensor_output_ratio -= 0.0001f; // Decreasing the ratio will increase the next calculated voltage
        }
        calculated_voltage = CalculateVoltagePrimaryResistor(pin_voltage, sensor_output_ratio, primary_resistance, burden_resistance);
    }
    std::cout << "Voltage: " << calculated_voltage << "\nSensor ratio:" << sensor_output_ratio << std::endl;
}