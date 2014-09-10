#include <iostream>
#include <atlaas/atlaas.hpp>

int main(int argc, char * argv[]) {
    atlaas::atlaas obj;
    obj.init(120.0, 120.0, 0.1, 0, 0, 0, 31, true);
    obj.process();
    obj.save_currents();
    return 0;
}
