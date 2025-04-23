#include "training.h"

int main(int argc, char** argv) {
    size_t seed = 0;
    if (argc > 1) {
        seed = std::stoul(argv[1]);
    }
    run(seed);
    return 0;
}
