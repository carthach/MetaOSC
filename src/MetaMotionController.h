#pragma once

#include <array>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <stdio.h>

#ifndef WIN32
#include <unistd.h>
#endif

#include "BleInterface.h"

// MetaWear SDK headers
#include "metawear/core/metawearboard.h"
#include "metawear/core/module.h"
#include "metawear/core/settings.h"
#include "metawear/core/status.h"
#include "metawear/core/debug.h"
#include "metawear/core/logging.h"
#include "metawear/core/macro.h"
#include "metawear/peripheral/led.h"
#include "metawear/core/data.h"
#include "metawear/core/datasignal.h"
#include "metawear/core/types.h"
#include "metawear/sensor/sensor_common.h"
#include "metawear/sensor/sensor_fusion.h"

// Bridges a SimpleBLE peripheral to the MetaWear C SDK.
// Configures sensor fusion and exposes Euler, acceleration, gyro, and
// magnetometer data as plain float arrays ready for OSC transmission.
class MetaMotionController {
public:
    MetaMotionController(SimpleBLE::Peripheral& peripheralIn);
    ~MetaMotionController();

    // Called each update tick to copy sensor fusion angles into `angle[]`.
    void update();

    // --- Connection ---
    bool setup();   // Initialise the MetaWear board over BLE. Returns true if started.
    void disconnectDevice(MblMwMetaWearBoard* board);
    bool isConnected = false;

    // --- Sensor output (updated asynchronously by MetaWear callbacks) ---
    float outputEuler[4];        // [heading/yaw, pitch, roll, yaw/heading]
    float outputAcceleration[3]; // corrected acceleration (x, y, z)
    float outputMag[3];          // corrected magnetometer (x, y, z)
    float outputGyro[3];         // corrected gyroscope (x, y, z)

    // If true, outputEuler[0] uses the magnetometer-corrected heading;
    // otherwise it uses the gyro-integrated yaw.
    bool bUseMagnoHeading = true;

    // --- Orientation helpers ---
    float angle[3];              // Current Euler angles (yaw/heading, pitch, roll)
    float angle_shift[3];        // Offset applied by recenter()
    std::array<float, 3> getAngle();  // Returns angle[] + angle_shift[]
    void resetOrientation();     // Zeroes angle_shift
    void recenter();             // Sets angle_shift so current angle reads as zero

    // --- Device info ---
    int battery_level = 0;
    const char* module_name = nullptr;

    // --- BLE peripheral and MetaWear board handle ---
    SimpleBLE::Peripheral& peripheral;
    MblMwMetaWearBoard* board = nullptr;

    // --- Board configuration helpers ---
    void get_current_power_status(MblMwMetaWearBoard* board);
    void get_battery_percentage(MblMwMetaWearBoard* board);
    void configure_sensor_fusion(MblMwMetaWearBoard* board);
    void enable_fusion_sampling(MblMwMetaWearBoard* board);
    void disable_fusion_sampling(MblMwMetaWearBoard* board);
    void enable_led(MblMwMetaWearBoard* board);
    void disable_led(MblMwMetaWearBoard* board);
    void set_ad_name(MblMwMetaWearBoard* board);
    void get_ad_name(MblMwMetaWearBoard* board);

    // --- MetaWear GATT bridge callbacks (called by the MetaWear C SDK) ---
    static void read_gatt_char(void* context, const void* caller,
                               const MblMwGattChar* characteristic,
                               MblMwFnIntVoidPtrArray handler);

    static void write_gatt_char(void* context, const void* caller,
                                MblMwGattCharWriteType writeType,
                                const MblMwGattChar* characteristic,
                                const uint8_t* value, uint8_t length);

    static void enable_char_notify(void* context, const void* caller,
                                   const MblMwGattChar* characteristic,
                                   MblMwFnIntVoidPtrArray handler,
                                   MblMwFnVoidVoidPtrInt ready);

    static void on_disconnect(void* context, const void* caller,
                              MblMwFnVoidVoidPtrInt handler);
};
