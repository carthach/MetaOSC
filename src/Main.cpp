#include <JuceHeader.h>
#include "MetaMotionController.h"
#include <csignal>
#include <atomic>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Global flag for shutdown signal
std::atomic<bool> g_shutdown_requested{false};

class MetaOSCThread : public juce::Thread {
    BleInterface bleInterface;
    OwnedArray<MetaMotionController> controllers;
    std::vector<SimpleBLE::Peripheral> peripherals;
    OwnedArray<juce::OSCSender> oscSenders;
        
public:
    MetaOSCThread(const json& config) : juce::Thread("MetaOSC Thread")
    {
            bleInterface.setup();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            peripherals = bleInterface.getMetaMotionPeripherals();
        
            auto macs = config["macs"].get<std::vector<std::string>>();
            
            // Filter peripherals to only include those in the macs vector
        
            if(!macs.empty())
            {
                std::vector<SimpleBLE::Peripheral> filteredPeripherals;
                for (const auto& mac : macs)
                {
                    for (auto& p : peripherals)
                    {
                        if (p.identifier() == mac || p.address() == mac)
                        {
                            filteredPeripherals.push_back(p);
                            break;
                        }
                    }
                }
                
                // Replace peripherals with the filtered and sorted version
                peripherals = filteredPeripherals;
            }
                                
            for(auto & p : peripherals)
            {
                p.connect();
                
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                auto controller = new MetaMotionController(p);
                controller->setup();
                controllers.add(controller);
            }
                            
            for(auto & server : config["servers"])
            {
                
                oscSenders.add(new OSCSender());
                                
                oscSenders.getLast()->connect(server["host"].get<std::string>(), server["port"].get<int>());
            }
                                        
            if(controllers.isEmpty()) {
                juce::Logger::writeToLog("No MetaMotion controllers found!");
            }
    }
    
    void run() override
    {
        while (!threadShouldExit() && !g_shutdown_requested.load())
        {
            for(int i=0; i<controllers.size();++i)
            {
                auto controller = controllers[i];
                if (controller == nullptr) continue;
                
                controller->update();
                                        
                {
                    auto address = String::formatted("/euler/%d", i);
                    juce::OSCMessage oscMessage(address, controller->outputEuler[0], controller->outputEuler[1], controller->outputEuler[2], controller->outputEuler[3]);
                    for(auto & oscSender : oscSenders)
                        oscSender->send(oscMessage);
                    
                    juce::Logger::writeToLog(String::formatted("/euler/%d %f %f %f %f", i, controller->outputEuler[0], controller->outputEuler[1], controller->outputEuler[2], controller->outputEuler[3]));
                }
                
                {
                    auto address = String::formatted("/acc/%d", i);
                    juce::OSCMessage oscMessage(address, controller->outputAcceleration[0], controller->outputAcceleration[1], controller->outputAcceleration[2]);
                    for(auto & oscSender : oscSenders)
                        oscSender->send(oscMessage);
                }
                
                {
                    auto address = String::formatted("/mag/%d", i);
                    juce::OSCMessage oscMessage(address, controller->outputMag[0], controller->outputMag[1], controller->outputGyro[2]);
                    for(auto & oscSender : oscSenders)
                        oscSender->send(oscMessage);
                }
                
                {
                    auto address = String::formatted("/gyro/%d", i);
                    juce::OSCMessage oscMessage(address, controller->outputGyro[0], controller->outputGyro[1], controller->outputGyro[2]);
                    for(auto & oscSender : oscSenders)
                        oscSender->send(oscMessage);
                }
            }
                
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void shutdown()
    {
        juce::Logger::writeToLog("Shutting down MetaOSC...");
        
        try {
            // Disconnect OSC sender
            for(auto & oscSender : oscSenders)
                oscSender->disconnect();
            
            // Disconnect all MetaMotion controllers
            for (int i = 0; i < controllers.size(); ++i)
            {
                auto controller = controllers[i];
                if (controller != nullptr && controller->isConnected)
                {
                    controller->disconnectDevice(controller->board);
                }
            }
            
            // Clear controllers
            controllers.clear();
            
            // Stop BLE scanning - use the first adapter if available
            if (!bleInterface.adapters.empty()) {
                bleInterface.exit(bleInterface.adapters[0]);
            }
            
            juce::Logger::writeToLog("MetaOSC shutdown complete.");
        } catch (const std::exception& e) {
            juce::Logger::writeToLog("Error during shutdown: " + String(e.what()));
        }
    }
    
};

// Signal handler for SIGINT
void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        juce::Logger::writeToLog("Received SIGINT, initiating shutdown...");
        g_shutdown_requested.store(true);
    }
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler);
    
    juce::Logger::writeToLog("Starting MetaOSC application...");
    
    // Default configuration, no macs connects to all addresses
    json config = json::parse(R"(
      {
        "macs": [
        ],
        "servers": [
            {
                "host": "127.0.0.1",
                "port": 8000
            },
            {
                "host": "127.0.0.1",
                "port": 8001
            }
        ]
      }
    )");
    
    // Parse command line arguments
    if (argc > 1)
    {
        juce::String configPath;
        
        // Look for --config or -c flag
        for (int i = 1; i < argc; ++i)
        {
            juce::String arg(argv[i]);
            if ((arg == "--config" || arg == "-c") && i + 1 < argc)
            {
                configPath = argv[i + 1];
                break;
            }
        }
        
        // Load config file if path was provided
        if (configPath.isNotEmpty())
        {
            try
            {
                std::ifstream configFile(configPath.toStdString());
                if (configFile.is_open())
                {
                    config = json::parse(configFile);
                    juce::Logger::writeToLog("Loaded configuration from: " + configPath);
                }
                else
                {
                    juce::Logger::writeToLog("Warning: Could not open config file: " + configPath);
                    juce::Logger::writeToLog("Using default configuration.");
                }
            }
            catch (const std::exception& e)
            {
                juce::Logger::writeToLog("Error parsing config file: " + juce::String(e.what()));
                juce::Logger::writeToLog("Using default configuration.");
            }
        }
    }
    
    MetaOSCThread metaOSCThread(config);
    
    if (!metaOSCThread.startThread()) {
        juce::Logger::writeToLog("Failed to start MetaOSC thread!");
        return 1;
    }
    
    // Wait for shutdown signal instead of infinite loop
    while (!g_shutdown_requested.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Perform clean shutdown
    metaOSCThread.shutdown();
    
    if (!metaOSCThread.stopThread(5000)) { // Wait up to 5 seconds for thread to stop
        juce::Logger::writeToLog("Warning: Thread did not stop gracefully, forcing stop...");
        metaOSCThread.stopThread(1000); // Force stop with shorter timeout
    }
    
    juce::Logger::writeToLog("Application terminated gracefully.");
    return 0;
}
