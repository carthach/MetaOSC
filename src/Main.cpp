#include <JuceHeader.h>
#include "MetaMotionController.h"
#include <csignal>
#include <atomic>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Set to true by the SIGINT handler to trigger a clean shutdown.
std::atomic<bool> g_shutdown_requested{false};

// ---------------------------------------------------------------------------
// MetaOSCThread
//
// Owns the BLE interface, MetaWear controllers, and OSC senders.
// Constructor blocks while scanning and connecting; run() streams sensor
// data at ~10 Hz to all configured OSC servers.
// ---------------------------------------------------------------------------

class MetaOSCThread : public juce::Thread {
    BleInterface                   bleInterface;
    OwnedArray<MetaMotionController> controllers;
    std::vector<SimpleBLE::Peripheral> peripherals;
    OwnedArray<juce::OSCSender>    oscSenders;
    bool verboseLogging;

public:
    MetaOSCThread(const json& config, bool verbose = true)
        : juce::Thread("MetaOSC Thread"), verboseLogging(verbose)
    {
        // --- BLE scan ---
        bleInterface.setup();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        peripherals = bleInterface.getMetaMotionPeripherals();

        // Filter to only the MAC addresses listed in the config (if any).
        auto macs = config["macs"].get<std::vector<std::string>>();
        if (!macs.empty()) {
            std::vector<SimpleBLE::Peripheral> filtered;
            for (const auto& mac : macs) {
                for (auto& p : peripherals) {
                    if (p.identifier() == mac || p.address() == mac) {
                        filtered.push_back(p);
                        break;
                    }
                }
            }
            peripherals = filtered;
        }

        // --- Connect and initialise each sensor ---
        for (auto& p : peripherals) {
            p.connect();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            auto* controller = new MetaMotionController(p);
            controller->setup();
            controllers.add(controller);
        }

        if (controllers.isEmpty())
            juce::Logger::writeToLog("No MetaMotion controllers found!");

        // --- Open OSC connections ---
        for (const auto& server : config["servers"]) {
            auto* sender = new juce::OSCSender();
            sender->connect(server["host"].get<std::string>(),
                            server["port"].get<int>());
            oscSenders.add(sender);
        }
    }

    // Main loop: poll each controller and broadcast sensor data over OSC.
    void run() override {
        // Helper: build an OSC message and send it to every configured server.
        auto sendOSC = [&](const juce::String& address, std::initializer_list<float> values) {
            juce::OSCMessage msg(address);
            for (float v : values) msg.addFloat32(v);
            for (auto& sender : oscSenders)
                sender->send(msg);
        };

        while (!threadShouldExit() && !g_shutdown_requested.load()) {
            for (int i = 0; i < controllers.size(); ++i) {
                auto* c = controllers[i];
                if (!c) continue;

                c->update();

                const float* e = c->outputEuler;        // heading/yaw, pitch, roll, yaw/heading
                const float* a = c->outputAcceleration;
                const float* g = c->outputGyro;
                const float* m = c->outputMag;

                sendOSC(juce::String::formatted("/euler/%d", i), {e[0], e[1], e[2], e[3]});
                sendOSC(juce::String::formatted("/acc/%d",   i), {a[0], a[1], a[2]});
                sendOSC(juce::String::formatted("/gyro/%d",  i), {g[0], g[1], g[2]});
                sendOSC(juce::String::formatted("/mag/%d",   i), {m[0], m[1], m[2]});

                if (verboseLogging)
                    juce::Logger::writeToLog(juce::String::formatted(
                        "/euler/%d %f %f %f %f", i, e[0], e[1], e[2], e[3]));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void shutdown() {
        juce::Logger::writeToLog("Shutting down MetaOSC...");
        try {
            for (auto& sender : oscSenders)
                sender->disconnect();

            for (int i = 0; i < controllers.size(); ++i) {
                auto* c = controllers[i];
                if (c && c->isConnected)
                    c->disconnectDevice(c->board);
            }

            controllers.clear();

            if (!bleInterface.adapters.empty())
                bleInterface.exit(bleInterface.adapters[0]);

            juce::Logger::writeToLog("MetaOSC shutdown complete.");
        } catch (const std::exception& e) {
            juce::Logger::writeToLog("Error during shutdown: " + juce::String(e.what()));
        }
    }
};

// ---------------------------------------------------------------------------
// Signal handler / main
// ---------------------------------------------------------------------------

void signalHandler(int signal) {
    if (signal == SIGINT) {
        juce::Logger::writeToLog("Received SIGINT, initiating shutdown...");
        g_shutdown_requested.store(true);
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);

    juce::Logger::writeToLog("Starting MetaOSC application...");

    bool verboseLogging = true;

    // Default config: connect to all MetaWear devices, stream to two local OSC ports.
    json config = json::parse(R"({
        "macs": [],
        "servers": [
            { "host": "127.0.0.1", "port": 8000 },
            { "host": "127.0.0.1", "port": 8001 }
        ]
    })");

    // Parse command-line arguments: -c/--config <path>, -q/--quiet
    juce::String configPath;
    for (int i = 1; i < argc; ++i) {
        juce::String arg(argv[i]);
        if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--quiet" || arg == "-q") {
            verboseLogging = false;
        }
    }

    if (configPath.isNotEmpty()) {
        try {
            std::ifstream file(configPath.toStdString());
            if (file.is_open()) {
                config = json::parse(file);
                juce::Logger::writeToLog("Loaded configuration from: " + configPath);
            } else {
                juce::Logger::writeToLog("Warning: Could not open config file: " + configPath + ". Using defaults.");
            }
        } catch (const std::exception& e) {
            juce::Logger::writeToLog("Error parsing config file: " + juce::String(e.what()) + ". Using defaults.");
        }
    }

    MetaOSCThread metaOSCThread(config, verboseLogging);

    if (!metaOSCThread.startThread()) {
        juce::Logger::writeToLog("Failed to start MetaOSC thread!");
        return 1;
    }

    // Block until Ctrl-C.
    while (!g_shutdown_requested.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    metaOSCThread.shutdown();

    if (!metaOSCThread.stopThread(5000)) {
        juce::Logger::writeToLog("Warning: Thread did not stop gracefully, forcing stop...");
        metaOSCThread.stopThread(1000);
    }

    juce::Logger::writeToLog("Application terminated gracefully.");
    return 0;
}
