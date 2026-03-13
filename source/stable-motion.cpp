#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <linux/input.h>
#include <linux/uinput.h>
#include <vector>
#include <fftw3.h>
#include <string>
#include "constants.h"

namespace SMC = StableMotionConstants;

class STFTStableMotion {
private:
    std::vector<int> buffer;
    fftw_complex *in, *out;
    fftw_plan p_forward, p_backward;

    double previous_output = 0.0;
    double alpha;
    double fractional_remainder = 0.0;

public:
    STFTStableMotion(double smoothness_level) {
        alpha = smoothness_level;

        in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SMC::WINDOW_SIZE);
        out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SMC::WINDOW_SIZE);

        p_forward = fftw_plan_dft_1d(SMC::WINDOW_SIZE, in, out, FFTW_FORWARD, FFTW_MEASURE);
        p_backward = fftw_plan_dft_1d(SMC::WINDOW_SIZE, out, in, FFTW_BACKWARD, FFTW_MEASURE);
    }

    ~STFTStableMotion() {
        fftw_destroy_plan(p_forward);
        fftw_destroy_plan(p_backward);
        fftw_free(in);
        fftw_free(out);
    }

    int apply(int new_val) {
        buffer.push_back(new_val);
        if (buffer.size() > SMC::WINDOW_SIZE) buffer.erase(buffer.begin());
        if (buffer.size() < SMC::WINDOW_SIZE) return new_val;

        for (int i = 0; i < SMC::WINDOW_SIZE; i++) {
            in[i][0] = static_cast<double>(buffer[i]);
            in[i][1] = 0.0;
        }

        fftw_execute(p_forward);

        double df = SMC::SAMPLE_RATE / SMC::WINDOW_SIZE;
        for (int i = 0; i < SMC::WINDOW_SIZE; i++) {
            double freq = (i <= SMC::WINDOW_SIZE / 2) ? (i * df) : ((SMC::WINDOW_SIZE - i) * df);
            if (freq >= 4.0 && freq <= 6.0) {
                out[i][0] = 0.0; 
                out[i][1] = 0.0;
            }
        }

        fftw_execute(p_backward);

        double raw_stft_output = in[SMC::WINDOW_SIZE - 1][0] / SMC::WINDOW_SIZE;
        double smoothed_output = (alpha * raw_stft_output) + ((1.0 - alpha) * previous_output);
        previous_output = smoothed_output;

        smoothed_output += fractional_remainder;
        int final_out = static_cast<int>(smoothed_output);
        fractional_remainder = smoothed_output - final_out;

        return final_out;
    }
};

void emit(int fd, int type, int code, int val) {
    struct input_event ie;
    ie.type = type;
    ie.code = code;
    ie.value = val;
    ie.time.tv_sec = 0.0;
    ie.time.tv_usec = 0.0;
    write(fd, &ie, sizeof(ie));
}

void emit_interpolated(int fd, int dx, int dy, int steps) {
    if (dx == 0 && dy == 0) return; // No movement to upscale

    double step_x = static_cast<double>(dx) / steps;
    double step_y = static_cast<double>(dy) / steps;

    double accum_x = 0.0;
    double accum_y = 0.0;

    for (int i = 0; i < steps; i++) {
        accum_x += step_x;
        accum_y += step_y;

        int send_x = static_cast<int>(accum_x);
        int send_y = static_cast<int>(accum_y);

        accum_x -= send_x;
        accum_y -= send_y;

        if (send_x != 0) emit(fd, EV_REL, REL_X, send_x);
        if (send_y != 0) emit(fd, EV_REL, REL_Y, send_y);
        
        emit(fd, EV_SYN, SYN_REPORT, 0); 
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: sudo ./stable_motion /dev/input/eventX [glide_factor]\n";
        return 1;
    }

    double smoothness_level = (argc >= 3) ? std::stod(argv[2]) : 0.05;

    int physical_fd = open(argv[1], O_RDONLY);
    if (physical_fd < 0) {
        std::cerr << "Root execution required! (sudo)\n";
        return 1;
    }
    ioctl(physical_fd, EVIOCGRAB, 1); // EXCLUSIVE GRAB

    int virtual_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    
    ioctl(virtual_fd, UI_SET_EVBIT, EV_KEY);
    ioctl(virtual_fd, UI_SET_KEYBIT, BTN_LEFT);
    ioctl(virtual_fd, UI_SET_KEYBIT, BTN_RIGHT);
    ioctl(virtual_fd, UI_SET_KEYBIT, BTN_MIDDLE);
    ioctl(virtual_fd, UI_SET_KEYBIT, BTN_SIDE);
    ioctl(virtual_fd, UI_SET_KEYBIT, BTN_EXTRA);

    ioctl(virtual_fd, UI_SET_EVBIT, EV_REL);
    ioctl(virtual_fd, UI_SET_RELBIT, REL_X);
    ioctl(virtual_fd, UI_SET_RELBIT, REL_Y);
    ioctl(virtual_fd, UI_SET_RELBIT, REL_WHEEL);  
    ioctl(virtual_fd, UI_SET_RELBIT, REL_HWHEEL);

    ioctl(virtual_fd, UI_SET_PROPBIT, INPUT_PROP_POINTER);

    struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Stable Motion");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1234;
    uidev.id.product = 0x5678;
    uidev.id.version = 1;

    write(virtual_fd, &uidev, sizeof(uidev));
    ioctl(virtual_fd, UI_DEV_CREATE);

    std::cout << "Stable Motion is live with glide value: " << smoothness_level << " \n";
    std::cout << "Press CTRL+C to exit.\n";

    STFTStableMotion filterX(smoothness_level);
    STFTStableMotion filterY(smoothness_level);

    struct input_event ev;
    int current_dx = 0;
    int current_dy = 0;
    bool motion_event = false;

    while (read(physical_fd, &ev, sizeof(ev)) > 0) {
        if (ev.type == EV_REL) {
            if (ev.code == REL_X) {
                current_dx += ev.value;
                motion_event = true;
            } else if (ev.code == REL_Y) {
                current_dy += ev.value;
                motion_event = true;
            } else if (ev.code == REL_WHEEL || ev.code == REL_HWHEEL) {

                emit(virtual_fd, EV_REL, ev.code, ev.value);
                emit(virtual_fd, EV_SYN, SYN_REPORT, 0); 
            }
        }
        else if (ev.type == EV_KEY) {
            
            emit(virtual_fd, EV_KEY, ev.code, ev.value);
            emit(virtual_fd, EV_SYN, SYN_REPORT, 0);
            continue;
        }
        else if (ev.type == EV_SYN) {
            if (motion_event) {
                // Filter the entire frame's movement
                int smooth_x = filterX.apply(current_dx);
                int smooth_y = filterY.apply(current_dy);
                
                if (smooth_x != 0 || smooth_y != 0) {
                    emit_interpolated(virtual_fd, smooth_x, smooth_y, 8);
                }
                
                // 3. Reset hardware tick
                current_dx = 0;
                current_dy = 0;
                motion_event = false;
            }
        }
    }

    ioctl(physical_fd, EVIOCGRAB, 0);
    ioctl(virtual_fd, UI_DEV_DESTROY);
    close(physical_fd);
    close(virtual_fd);

    return 0;
}
