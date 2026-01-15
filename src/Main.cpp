#include <JuceHeader.h>
#include "MetaMotionController.h"
#include <csignal>
#include <atomic>

// Global flag for shutdown signal
std::atomic<bool> g_shutdown_requested{false};

class MetaOSCThread : public juce::Thread {
    BleInterface bleInterface;
    OwnedArray<MetaMotionController> controllers;
    std::vector<SimpleBLE::Peripheral> peripherals;
    
    std::array<int, 2> ports = {8000, 8001};
    
    OwnedArray<juce::OSCSender> oscSenders;
        
public:
    MetaOSCThread() : juce::Thread("MetaOSC Thread")
    {
            bleInterface.setup();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            peripherals = bleInterface.getMetaMotionPeripherals();
            
            for(auto & p : peripherals)
            {
                p.connect();
                
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                auto controller = new MetaMotionController(p);
                controller->setup();
                controllers.add(controller);
            }
                            
            for(auto & port : ports)
            {
                
                oscSenders.add(new OSCSender());
                oscSenders.getLast()->connect("127.0.0.1", port);
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
    
    MetaOSCThread metaOSCThread;
    
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
    juce::ignoreUnused(argc, argv);
    return 0;
}
