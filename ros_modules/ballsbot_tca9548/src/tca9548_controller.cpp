#include "tca9548.h"
#include <stdint.h>
#include <cstdlib>
#include <iostream>
#include <bitset>
#include <string>

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "USAGE: tca9548_controller <bus_number> <device_address> <new_state_bits>\n";
        exit(1);
    }

    uint8_t bus_number = atoi(argv[1]);
    uint8_t address = atoi(argv[2]);
    std::string new_state_bits = argv[3];
    std::bitset<8> new_state(new_state_bits);

    TCA9548 controller(bus_number, address);
    controller.Start();

    controller.SetState(new_state.to_ullong());

    auto state = controller.GetState();
    std::bitset<8> state_bits(state);
    std::cout << "new state: " << state_bits << std::endl;

    controller.Finish();
}
