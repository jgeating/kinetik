#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Swerve Robot Simulation Test" << std::endl;
    std::cout << "  GCC Version: " << __VERSION__ << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "Starting simulation test..." << std::endl;
    
    for (int i = 0; i < 10; i++) {
        std::cout << "Loop " << i << ": Robot running..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Test completed successfully!" << std::endl;
    return 0;
}
