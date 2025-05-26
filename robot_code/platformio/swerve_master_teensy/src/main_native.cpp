/**
 * @file main_native.cpp
 * @brief Native C++ entry point for desktop simulation
 * 
 * This file provides the main() function for running the swerve robot code
 * on a desktop computer for simulation and testing purposes.
 */

#include "hal/HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

#include "hal/HALFactory.h"
#include "hal/sim/SimulationInterface.h"
#include <iostream>
#include <signal.h>
#include <chrono>
#include <thread>

// Forward declarations of main robot functions
void robotSetup();
void robotLoop();

// Global flag for clean shutdown
std::atomic<bool> g_running{true};

/**
 * @brief Signal handler for clean shutdown
 */
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

/**
 * @brief Print startup banner
 */
void printBanner() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Swerve Robot Simulation Mode" << std::endl;
    std::cout << "  HAL Implementation: " << HALFactory::getImplementationType() << std::endl;
    std::cout << "========================================" << std::endl;
}

/**
 * @brief Print usage information
 */
void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --server-url URL    WebSocket server URL (default: ws://localhost:8080)" << std::endl;
    std::cout << "  --help             Show this help message" << std::endl;
}

/**
 * @brief Main entry point for native simulation
 */
int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string serverUrl = "ws://localhost:8080";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--server-url" && i + 1 < argc) {
            serverUrl = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
    // Set up signal handlers for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    printBanner();
    
    try {
        // Initialize simulation interface
        std::cout << "Connecting to simulation server at " << serverUrl << "..." << std::endl;
        SimulationManager simManager(serverUrl);
        
        if (!simManager.isReady()) {
            std::cerr << "Failed to connect to simulation server!" << std::endl;
            std::cerr << "Make sure the simulation server is running at " << serverUrl << std::endl;
            return 1;
        }
        
        std::cout << "Connected to simulation server successfully!" << std::endl;
        
        // Initialize HAL
        std::cout << "Initializing HAL..." << std::endl;
        HAL::initialize();
        
        // Call robot setup (equivalent to Arduino setup())
        std::cout << "Running robot setup..." << std::endl;
        robotSetup();
        
        std::cout << "Starting main loop..." << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
        // Main loop (equivalent to Arduino loop())
        auto lastLoopTime = std::chrono::steady_clock::now();
        const auto targetLoopTime = std::chrono::microseconds(4500); // 4.5ms loop time
        
        while (g_running) {
            auto loopStart = std::chrono::steady_clock::now();
            
            // Update simulation interface
            simManager.getInterface().update();
            
            // Run robot loop
            robotLoop();
            
            // Maintain consistent loop timing
            auto loopEnd = std::chrono::steady_clock::now();
            auto loopDuration = loopEnd - loopStart;
            
            if (loopDuration < targetLoopTime) {
                std::this_thread::sleep_for(targetLoopTime - loopDuration);
            }
            
            // Print timing info occasionally
            static int loopCount = 0;
            if (++loopCount % 1000 == 0) {
                auto actualLoopTime = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - lastLoopTime).count();
                std::cout << "Loop time: " << actualLoopTime << " µs" << std::endl;
                lastLoopTime = std::chrono::steady_clock::now();
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    // Clean shutdown
    std::cout << "Shutting down..." << std::endl;
    HAL::shutdown();
    std::cout << "Shutdown complete." << std::endl;
    
    return 0;
}

#endif // HAL_IMPLEMENTATION == HAL_SIM
