// Undefine LINUX macro that may be defined by JUCE to avoid conflict with SimpleBLE enum
#ifdef LINUX
#undef LINUX
#endif

#include "simpleble/SimpleBLE.h"

#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <thread>

#define SCAN_TIMEOUT_MS 10000

#define NORDIC_UART_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NORDIC_UART_CHAR_RX      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NORDIC_UART_CHAR_TX      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define METAMOTION_READ_SERVICE_UUID    "0000180a-0000-1000-8000-00805f9b34fb"
#define METAMOTION_READ_UUID            "00002a26-0000-1000-8000-00805f9b34fb"
#define METAMOTION_NOTIFY_SERVICE_UUID  "326a9000-85cb-9195-d9dd-464cfbbae75a"
#define METAMOTION_NOTIFY_UUID          "326a9006-85cb-9195-d9dd-464cfbbae75a"
#define METAMOTION_WRITE_SERVICE_UUID   "326a9000-85cb-9195-d9dd-464cfbbae75a"
#define METAMOTION_WRITE_UUID           "326a9001-85cb-9195-d9dd-464cfbbae75a"

class BleInterface {
public:
    std::vector<SimpleBLE::Peripheral> peripherals;
    SimpleBLE::Adapter ble;
    std::vector<SimpleBLE::Adapter> adapters;

    void setup() {
        // Setup callback functions
        try{
            adapters = SimpleBLE::Adapter::get_adapters();
        } catch (...) {
            throw;
        }
        
        if (adapters.size() == 0) {
            std::cout << "No adapter was found." << std::endl;
            return;
        }

        scanDevices();
    }
    
    void exit(SimpleBLE::Adapter ble) {
        ble.scan_stop();
    }
    
    void scanDevices(){
        // Setup callback functions
        try {
            adapters = SimpleBLE::Adapter::get_adapters();
        } catch (...) {
            throw;
        }
        
        if (adapters.size() == 0) {
            std::cout << "No adapter was found." << std::endl;
            return;
        }
        
        try {
            SimpleBLE::Adapter adapter = adapters[0];

            adapter.set_callback_on_scan_start([]() { std::cout << "Scan started." << std::endl; });
            adapter.set_callback_on_scan_stop([]() { std::cout << "Scan stopped." << std::endl; });
            adapter.set_callback_on_scan_found([this](SimpleBLE::Peripheral peripheral) {
                std::cout << "Found device: " << peripheral.identifier() << " [" << peripheral.address() << "] " << peripheral.rssi() << " dBm" << std::endl;
                peripherals.push_back(peripheral);
            });
            adapter.scan_for(SCAN_TIMEOUT_MS);
        } catch (...) {
            throw;
        }
    }
    
    void listDevices(){
        std::cout << "The following devices were found:" << std::endl;
        for (int i = 0; i < peripherals.size(); i++) {
            std::cout << "  " << i << ": " << peripherals[i].identifier() << " (" << peripherals[i].address() << ")" << std::endl;
        }
    }
    
    std::vector<SimpleBLE::Peripheral> getMetaMotionPeripherals(){
        std::vector<SimpleBLE::Peripheral> metaMotionPeripherals;
        for (auto & p : peripherals) {
            if (p.identifier().find("MetaWear") != std::string::npos) {
                std::cout << "Auto found MetaMotion: " << p.address() << '\n';
                metaMotionPeripherals.push_back(p);
            }
        }
        return metaMotionPeripherals;
    }        
};
