#include <Arduino.h>
#include "DisplaySetup.hpp"
#include "Utilities.hpp"

/* Set all widgets related to motor RPM measure. */
void lv_rpm_set(int rpm, lv_obj_t *label, lv_obj_t *arc, lv_obj_t *haste_img){
    char rpm_text[5];
    itoa(rpm, rpm_text, 10);
    lv_label_set_text(label, rpm_text);
    lv_arc_set_value(arc, rpm);
    int32_t haste_angle_l = lv_map(rpm, 0, 5500, -600, 600);
    lv_img_set_angle(haste_img, haste_angle_l);
}

/* Set all widgets related to charts */
void lv_chart_set(float value, lv_chart_series_t *serie, lv_obj_t *chart, lv_obj_t *label){
    char label_text[8];
    sprintf(label_text, "%.2f", value);
    lv_label_set_text(label, label_text);
    lv_chart_set_next_value(chart, serie, value);
    lv_chart_refresh(chart);
}

/* Task to update display */
void CockpitDisplayTask(void* parameter) {
    // Sin wave test variables
    constexpr float rpm_full_scale = 5500.0;
    constexpr float rpm_zero_scale = 0.0;
    constexpr float battery_volts_full_scale = 60.0;
    constexpr float battery_volts_zero_scale = 40.0;
    constexpr float battery_amps_full_scale = 60.0;
    constexpr float battery_amps_zero_scale = -25.0;
    constexpr float motor_amps_full_scale = 60.0;
    constexpr float motor_amps_zero_scale = 0.0;
    constexpr float mppt_amps_full_scale = 40.0;
    constexpr float mppt_amps_zero_scale = 0.0;
    constexpr float widget_length = 239.0f;
    constexpr float widget_height = 126.0f;

    // Initializing variables for chart widget
    lv_chart_series_t *serieRPM_L = lv_chart_add_series(ui_RPM_Current_Chart, lv_color_hex(0x081947), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *serieRPM_R = lv_chart_add_series(ui_RPM_Current_Chart, lv_color_hex(0xCA4D0F), LV_CHART_AXIS_SECONDARY_Y);
    lv_chart_series_t *serieBatA = lv_chart_add_series(ui_Battery_Chart, lv_color_hex(0x30DD3D), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *serieBatV = lv_chart_add_series(ui_Battery_Chart, lv_color_hex(0xDB3939), LV_CHART_AXIS_SECONDARY_Y);
    lv_chart_series_t *serieMppt = lv_chart_add_series(ui_Solar_Chart, lv_color_hex(0xE1C027), LV_CHART_AXIS_PRIMARY_Y);
  
    while (true) {
        constexpr int loop_period = 500; 
        static uint32_t update_time = 0;  

        auto test_sine_wave = [&]() {
            static uint32_t test_update_time = 0;
            static float angle = 0.0f;
            constexpr float radians_to_degrees = 3.14159265358979323846 / 180.0;

            if (millis() - test_update_time > loop_period) {
                test_update_time = millis();
                angle += 4; if (angle > 360) angle = 0;

                // Create a Sine wave for testing, value is in range 0 - 100
                float value = 50.0 + 50.0 * sin(angle * radians_to_degrees);

                auto mapValue = [](float ip, float ipmin, float ipmax, float tomin, float tomax) {
                    return (ip - ipmin) * (tomax - tomin) / (ipmax - ipmin) + tomin;
                };

                float rpm;
                rpm = mapValue(value, (float)0.0, (float)100.0, rpm_zero_scale, rpm_full_scale);
                lv_rpm_set(rpm, ui_RPM_L_text, ui_RPM_L_ARC, ui_Haste_RPM_L);
                lv_rpm_set(rpm, ui_RPM_R_text, ui_RPM_R_ARC, ui_Haste_RPM_R);
                
                // float battery_current = mapValue(value, (float)0.0, (float)100.0, battery_amps_zero_scale, battery_amps_full_scale);
                // widget_battery_current.updateNeedle(battery_current, 0);

                float battery_voltage;
                battery_voltage = mapValue(value, (float)0.0, (float)100.0, battery_volts_zero_scale, battery_volts_full_scale);
                lv_chart_set(battery_voltage, serieBatV, ui_Battery_Chart, ui_Battery_Volts_text);

                float motor_current;
                motor_current = mapValue(value, (float)0.0, (float)100.0, motor_amps_zero_scale, motor_amps_full_scale);
                lv_chart_set(motor_current, serieRPM_L, ui_RPM_Current_Chart, ui_RPM_L_Current_text);
                lv_chart_set(motor_current, serieRPM_R, ui_RPM_Current_Chart, ui_RPM_R_Current_text);

                float mppt_current;
                mppt_current = mapValue(value, (float)0.0, (float)100.0, mppt_amps_zero_scale, mppt_amps_full_scale);
                lv_chart_set(mppt_current, serieMppt, ui_Solar_Chart, ui_MPPT_Current_text);
            }
        };
        

        // Use data from SystemData static class to update the display

        auto update_display = [&]() {
            if (millis() - update_time > loop_period) {
                update_time = millis();

                /* Motors */
                // L
                lv_rpm_set(SystemData::getInstance().all_info.rpm_left, ui_RPM_L_text, ui_RPM_L_ARC, ui_Haste_RPM_L);

                // R
                lv_rpm_set(SystemData::getInstance().all_info.rpm_right, ui_RPM_R_text, ui_RPM_R_ARC, ui_Haste_RPM_R);

                // Motor Current
                // L
                lv_chart_set(SystemData::getInstance().all_info.motor_current_left, serieRPM_L, ui_RPM_Current_Chart, ui_RPM_L_Current_text);

                // R
                lv_chart_set(SystemData::getInstance().all_info.motor_current_right, serieRPM_R, ui_RPM_Current_Chart, ui_RPM_R_Current_text);

                /* Battery */
                // Voltage
                lv_chart_set(SystemData::getInstance().all_info.battery_voltage, serieBatV, ui_Battery_Chart, ui_Battery_Volts_text);

                // Current ??? - Is it not measured?
                // char bat_current_text[8];
                // sprintf(bat_current_text, "%.2f A", batA);
                // lv_label_set_text(ui_Battery_Current_text, bat_current_text);
                // lv_chart_set_next_value(ui_Battery_Chart, serieBatA, batA);

                /* MPPT */
                lv_chart_set(SystemData::getInstance().all_info.mppt_current, serieMppt, ui_Solar_Chart, ui_MPPT_Current_text);
            }
        };

        update_display();
        // test_sine_wave();
        vTaskDelay(pdMS_TO_TICKS(loop_period));
    }
}