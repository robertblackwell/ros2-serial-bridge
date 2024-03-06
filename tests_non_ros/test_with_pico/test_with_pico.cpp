
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <format>
#include <cstdint>
#include <libserial/SerialPort.h>
#include <unittest.h>
#include "jsoncons/json.hpp"
#include "jsoncons_ext/jsonpath/jsonpath.hpp"
#include <bridge_lib/serial_settings.h>
#include <fbcontrol/custom.h>


struct SideStatus {
    double          tick_count;
    uint64_t        sample_elapsedtime_usecs;
    double          wheel_speed_mm_secs;
    double          motor_rpm_estimate;
};

struct EncoderStatusRecord {
    double left_pwm;
    double right_pwm;
    SideStatus left;
    SideStatus right;
};

struct TargetRecord {
    double target_rpm;
    std::vector<EncoderStatusRecord> status_list;
};

std::vector<TargetRecord> run_record;

void print_run_record()
{
    for(auto r: run_record) {
        printf("{\n");
        printf("    \"target\":%f,\n", r.target_rpm);
        for(auto s: r.status_list) {
            printf("    \"left_pwm\" :%f,\n", s.left_pwm);
            printf("    \"right_pwm\":%f,\n", s.right_pwm);
            printf("    [\n");

            printf("\"left\": {");
            printf("\"ticks\": %f, ", s.left.tick_count);
            printf("\"elapsedus\": %ld, ", s.left.sample_elapsedtime_usecs);
            printf("\"rpm\": %f, ", s.left.motor_rpm_estimate);
            printf("\"wspeed\": %f,", s.left.wheel_speed_mm_secs);
            printf("},\n");
            printf("\"right\":{");
            printf("\"ticks\": %f, ", s.right.tick_count);
            printf("\"elapsedus\": %ld, ", s.right.sample_elapsedtime_usecs);
            printf("\"rpm\": %f, ", s.right.motor_rpm_estimate);
            printf("\"wspeed\": %f,", s.right.wheel_speed_mm_secs);
            printf("}\n},");
        }
    }
}

void send_pwm(LibSerial::SerialPort& serial_port, double left, double right)
{
//    printf("    \"left\":%f,\n    \"right\":%f,\n", left, right);
    serial_port.Write(std::format("pwm {} {}\n", left, right));
}
void read_encoder_status(LibSerial::SerialPort& serial_port, EncoderStatusRecord& status)
{
    std::string line{};
    serial_port.Write(std::format("e\n"));
    while(1) {
        serial_port.ReadLine(line);
        if(line.substr(0,2) == "1J") {
//            printf("%s", line.c_str());
            std::string json_string = line.substr(2, std::string::npos);
            jsoncons::json j = jsoncons::json::parse(json_string);
            status.left.tick_count = j[0]["ss"].as<int64_t>();
            status.left.sample_elapsedtime_usecs = j[0]["ts"].as<int64_t>();
            status.left.motor_rpm_estimate = j[0]["mr"].as<float>();
            status.left.wheel_speed_mm_secs = j[0]["ws"].as<int8_t>();

            status.right.tick_count = j[1]["ss"].as<int64_t>();
            status.right.sample_elapsedtime_usecs = j[1]["ts"].as<int64_t>();
            status.right.motor_rpm_estimate = j[1]["mr"].as<float>();
            status.right.wheel_speed_mm_secs = j[1]["ws"].as<int8_t>();
            return;
        } else if(line.substr(0, 5) == "Heart") {
            ;
        } else {
//            printf("%s", line.c_str());
        }
    }
}
int main() {
    auto devices = list_serial_devices();
    LibSerial::SerialPort serial_port{
            devices[0],
            LibSerial::BaudRate::BAUD_1152000,
            LibSerial::CharacterSize::CHAR_SIZE_8,
            LibSerial::FlowControl::FLOW_CONTROL_NONE,
            LibSerial::Parity::PARITY_NONE,
            LibSerial::StopBits::STOP_BITS_1};
//    while (1) {
//        EncoderStatusRecord status;
//        read_encoder_status(serial_port, status);
//        printf("Stop here\n");
//    }
    serial_port.FlushIOBuffers();
    CustomController left_controller(CustomController::Algorithm::fixed_step, MotorSide::left);
    CustomController right_controller(CustomController::Algorithm::fixed_step, MotorSide::right);

    for(int i = 3500; i <= 7500; i+= 500) {
        TargetRecord  target_record{};
        EncoderStatusRecord status_record{};

        double saved_left_pwm;
        double saved_right_pwm;
        double target = (double)i;

        target_record.target_rpm = target;

        double previous_left_rpm = 0.0;
        double previous_right_rpm = 0.0;
        auto left_new_pwm = left_controller.set_target(ValueWithDirection{target});
        auto right_new_pwm = right_controller.set_target(ValueWithDirection{target});

        saved_left_pwm = left_new_pwm.signed_value();
        saved_right_pwm= right_new_pwm.signed_value();

//        printf("{\n");
//        printf("    \"target\":%f,\n", target);
//        printf("    \"left\" :%f,\n", left_new_pwm.signed_value());
//        printf("    \"right\":%f,\n", right_new_pwm.signed_value());
//        printf("    [\n");

        send_pwm(serial_port, left_new_pwm.signed_value(), right_new_pwm.signed_value());
        for(int count = 0; count < 50; count++) {
            sleep(1.0);
            EncoderStatusRecord status{};
            read_encoder_status(serial_port, status);
            status_record.left_pwm = left_new_pwm.signed_value();
            status_record.right_pwm = right_new_pwm.signed_value();

//            printf("{\n");
//            printf("\"left\": {");
//            printf("\"ticks\": %f, ", status.left.tick_count);
//            printf("\"elapsedus\": %ld, ", status.left.sample_elapsedtime_usecs);
//            printf("\"rpm\": %f, ", status.left.motor_rpm_estimate);
//            printf("\"wspeed\": %f,", status.left.wheel_speed_mm_secs);
//            printf("},\n");
//            printf("\"right\":{");
//            printf("\"ticks\": %f, ", status.right.tick_count);
//            printf("\"elapsedus\": %ld, ", status.right.sample_elapsedtime_usecs);
//            printf("\"rpm\": %f, ", status.right.motor_rpm_estimate);
//            printf("\"wspeed\": %f,", status.right.wheel_speed_mm_secs);
//            printf("}\n},");
#if 1
            double left_rpm = status.left.motor_rpm_estimate;
            double right_rpm = status.right.motor_rpm_estimate;
            auto new_left_pwm = left_controller.update(left_rpm);
            auto new_right_pwm = right_controller.update(right_rpm);
            bool left_same = (fabs(fabs(saved_left_pwm) - fabs(new_left_pwm.signed_value())) < 0.1);
            bool right_same = (fabs(fabs(saved_right_pwm) - fabs(new_right_pwm.signed_value())) < 0.1);
            saved_left_pwm = new_left_pwm.signed_value();
            saved_right_pwm = new_right_pwm.signed_value();
            printf("controller output: left_pwm: %f  right_pwm: %f\n", new_left_pwm.signed_value(), new_right_pwm.signed_value());
            if(!(left_same && right_same)) {
                send_pwm(serial_port, new_left_pwm.signed_value(), new_right_pwm.signed_value());
            } else {
                printf("did not send pwm  %f %f \n", new_left_pwm.signed_value(), new_right_pwm.signed_value());
            }
//            report(status);
#endif
        }


    }
    return 0;
}