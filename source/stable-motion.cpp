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
};