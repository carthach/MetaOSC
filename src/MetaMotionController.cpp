//
//  MetaMotionController.cpp
//  Created by Mach1 on 01/28/21.
//
//  MetaWear C SDK reference: https://mbientlab.com/cppdocs/latest/index.html
//

#include "MetaMotionController.h"

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MetaMotionController::MetaMotionController(SimpleBLE::Peripheral& peripheralIn)
    : peripheral(peripheralIn)
{
    resetOrientation();
}

MetaMotionController::~MetaMotionController() {
    if (isConnected)
        disconnectDevice(board);
}

// ---------------------------------------------------------------------------
// Setup – connects the MetaWear C SDK to our BLE peripheral via callbacks
// ---------------------------------------------------------------------------

bool MetaMotionController::setup() {
    if (!peripheral.is_connected())
        isConnected = false;

    // Wire the MetaWear SDK's GATT operations to our SimpleBLE peripheral.
    MblMwBtleConnection btleConnection;
    btleConnection.context              = this;
    btleConnection.write_gatt_char      = write_gatt_char;
    btleConnection.read_gatt_char       = read_gatt_char;
    btleConnection.enable_notifications = enable_char_notify;
    btleConnection.on_disconnect        = on_disconnect;
    board = mbl_mw_metawearboard_create(&btleConnection);

    // Asynchronously initialise the board; the lambda is called when done.
    mbl_mw_metawearboard_initialize(board, this, [](void* context, MblMwMetaWearBoard* board, int32_t status) {
        // MetaWear SDK: status == MBL_MW_STATUS_OK (0) means success.
        if (status != MBL_MW_STATUS_OK) {
            printf("Error initializing board: %d\n", status);
            return;
        }
        printf("Board initialized\n");

        // Spin-wait for the async init to complete (SDK requirement).
        while (!mbl_mw_metawearboard_is_initialized(board)) {}

        auto dev_info = mbl_mw_metawearboard_get_device_information(board);
        printf("Firmware: %s  Model: %s (%s)\n",
               dev_info->firmware_revision,
               dev_info->model_number,
               mbl_mw_metawearboard_get_model_name(board));

        auto* self = static_cast<MetaMotionController*>(context);
        self->enable_fusion_sampling(board);
        self->get_current_power_status(board);
        self->get_battery_percentage(board);
        self->get_ad_name(board);
        self->isConnected = true;
    });

    return true;
}

// ---------------------------------------------------------------------------
// Per-frame update
// ---------------------------------------------------------------------------

void MetaMotionController::update() {
    if (!peripheral.is_connected())
        return;

    // Select heading source based on whether magnetometer correction is used.
    angle[0] = bUseMagnoHeading ? outputEuler[0] : outputEuler[3];
    angle[1] = outputEuler[1];
    angle[2] = outputEuler[2];
}

// ---------------------------------------------------------------------------
// Disconnect
// ---------------------------------------------------------------------------

void MetaMotionController::disconnectDevice(MblMwMetaWearBoard* board) {
    if (isConnected) {
        disable_led(board);
        mbl_mw_metawearboard_free(board);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    isConnected = false;
}

// ---------------------------------------------------------------------------
// Sensor fusion configuration
// ---------------------------------------------------------------------------

void MetaMotionController::configure_sensor_fusion(MblMwMetaWearBoard* board) {
    // NDOF = 9-DoF fusion using accelerometer, gyro, and magnetometer.
    mbl_mw_sensor_fusion_set_mode(board, MBL_MW_SENSOR_FUSION_MODE_NDOF);
    mbl_mw_sensor_fusion_set_acc_range(board, MBL_MW_SENSOR_FUSION_ACC_RANGE_4G);
    mbl_mw_sensor_fusion_set_gyro_range(board, MBL_MW_SENSOR_FUSION_GYRO_RANGE_2000DPS);
    mbl_mw_sensor_fusion_write_config(board);

    // MetaMotion S supports a higher TX power level.
    bool isMMSModel = (mbl_mw_metawearboard_get_model(board) == MBL_MW_MODEL_METAMOTION_S);
    mbl_mw_settings_set_tx_power(board, isMMSModel ? 8 : 4);
}

void MetaMotionController::enable_fusion_sampling(MblMwMetaWearBoard* board) {
    if (!board) return;

    configure_sensor_fusion(board);

    // Subscribe to Euler angles (heading, pitch, roll, yaw).
    auto euler_signal = mbl_mw_sensor_fusion_get_data_signal(board, MBL_MW_SENSOR_FUSION_DATA_EULER_ANGLE);
    mbl_mw_datasignal_subscribe(euler_signal, this, [](void* context, const MblMwData* data) {
        auto* self  = static_cast<MetaMotionController*>(context);
        auto* euler = static_cast<MblMwEulerAngles*>(data->value);
        // Store heading and yaw in positions 0/3 depending on the magno flag.
        if (self->bUseMagnoHeading) {
            self->outputEuler[0] = euler->heading;
            self->outputEuler[3] = euler->yaw;
        } else {
            self->outputEuler[0] = euler->yaw;
            self->outputEuler[3] = euler->heading;
        }
        self->outputEuler[1] = euler->pitch;
        self->outputEuler[2] = euler->roll;
    });

    // Subscribe to corrected acceleration.
    auto acc_signal = mbl_mw_sensor_fusion_get_data_signal(board, MBL_MW_SENSOR_FUSION_DATA_CORRECTED_ACC);
    mbl_mw_datasignal_subscribe(acc_signal, this, [](void* context, const MblMwData* data) {
        auto* self = static_cast<MetaMotionController*>(context);
        auto* acc  = static_cast<MblMwCorrectedCartesianFloat*>(data->value);
        self->outputAcceleration[0] = acc->x;
        self->outputAcceleration[1] = acc->y;
        self->outputAcceleration[2] = acc->z;
    });

    // Subscribe to corrected gyroscope.
    auto gyro_signal = mbl_mw_sensor_fusion_get_data_signal(board, MBL_MW_SENSOR_FUSION_DATA_CORRECTED_GYRO);
    mbl_mw_datasignal_subscribe(gyro_signal, this, [](void* context, const MblMwData* data) {
        auto* self = static_cast<MetaMotionController*>(context);
        auto* gyro = static_cast<MblMwCorrectedCartesianFloat*>(data->value);
        self->outputGyro[0] = gyro->x;
        self->outputGyro[1] = gyro->y;
        self->outputGyro[2] = gyro->z;
    });

    // Subscribe to corrected magnetometer.
    auto mag_signal = mbl_mw_sensor_fusion_get_data_signal(board, MBL_MW_SENSOR_FUSION_DATA_CORRECTED_MAG);
    mbl_mw_datasignal_subscribe(mag_signal, this, [](void* context, const MblMwData* data) {
        auto* self = static_cast<MetaMotionController*>(context);
        auto* mag  = static_cast<MblMwCorrectedCartesianFloat*>(data->value);
        self->outputMag[0] = mag->x;
        self->outputMag[1] = mag->y;
        self->outputMag[2] = mag->z;
    });

    // Enable all data streams and start fusion.
    mbl_mw_sensor_fusion_enable_data(board, MBL_MW_SENSOR_FUSION_DATA_EULER_ANGLE);
    mbl_mw_sensor_fusion_enable_data(board, MBL_MW_SENSOR_FUSION_DATA_CORRECTED_ACC);
    mbl_mw_sensor_fusion_enable_data(board, MBL_MW_SENSOR_FUSION_DATA_CORRECTED_GYRO);
    mbl_mw_sensor_fusion_enable_data(board, MBL_MW_SENSOR_FUSION_DATA_CORRECTED_MAG);
    mbl_mw_sensor_fusion_start(board);

    enable_led(board);
}

void MetaMotionController::disable_fusion_sampling(MblMwMetaWearBoard* board) {
    if (!board) return;
    auto fusion_signal = mbl_mw_sensor_fusion_get_data_signal(board, MBL_MW_SENSOR_FUSION_DATA_EULER_ANGLE);
    mbl_mw_datasignal_unsubscribe(fusion_signal);
    mbl_mw_sensor_fusion_stop(board);
}

// ---------------------------------------------------------------------------
// Power / battery status
// ---------------------------------------------------------------------------

void MetaMotionController::get_current_power_status(MblMwMetaWearBoard* board) {
    if (!board) return;

    auto power_signal = mbl_mw_settings_get_power_status_data_signal(board);
    mbl_mw_datasignal_subscribe(power_signal, this, [](void*, const MblMwData* data) {
        std::cout << "Power Status: " << data << std::endl;
    });

    auto charge_signal = mbl_mw_settings_get_charge_status_data_signal(board);
    mbl_mw_datasignal_subscribe(charge_signal, this, [](void*, const MblMwData* data) {
        std::cout << "Charge Status: " << data << std::endl;
    });
}

void MetaMotionController::get_battery_percentage(MblMwMetaWearBoard* board) {
    if (!board) return;

    auto battery_signal = mbl_mw_settings_get_battery_state_data_signal(board);
    mbl_mw_datasignal_subscribe(battery_signal, this, [](void* context, const MblMwData* data) {
        auto* self  = static_cast<MetaMotionController*>(context);
        auto* state = static_cast<MblMwBatteryState*>(data->value);
        self->battery_level = state->charge;
    });
    mbl_mw_datasignal_read(battery_signal);
}

// ---------------------------------------------------------------------------
// Device name / advertising name
// ---------------------------------------------------------------------------

void MetaMotionController::set_ad_name(MblMwMetaWearBoard* board) {
    if (!board) return;

    const char* name = "MetaWear"; // default
    switch (mbl_mw_metawearboard_get_model(board)) {
        case MBL_MW_MODEL_METAMOTION_S:  name = "Mach1-MMS";  break;
        case MBL_MW_MODEL_METAMOTION_RL: name = "Mach1-MMRL"; break;
        case MBL_MW_MODEL_METAWEAR_R:    name = "Mach1-MMR";  break;
        default: break;
    }

    mbl_mw_settings_set_device_name(board,
                                    reinterpret_cast<const uint8_t*>(name),
                                    strlen(name));
}

void MetaMotionController::get_ad_name(MblMwMetaWearBoard* board) {
    if (!board) return;
    module_name = mbl_mw_metawearboard_get_model_name(board);
}

// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------

void MetaMotionController::enable_led(MblMwMetaWearBoard* board) {
    if (!board) return;
    MblMwLedPattern pattern = { 8, 0, 250, 250, 250, 5000, 0, 0 };
    mbl_mw_led_write_pattern(board, &pattern, MBL_MW_LED_COLOR_RED);
    mbl_mw_led_play(board);
}

void MetaMotionController::disable_led(MblMwMetaWearBoard* board) {
    if (!board) return;
    mbl_mw_led_stop_and_clear(board);
}

// ---------------------------------------------------------------------------
// Orientation helpers
// ---------------------------------------------------------------------------

void MetaMotionController::resetOrientation() {
    std::fill(std::begin(angle_shift), std::end(angle_shift), 0.0f);
}

void MetaMotionController::recenter() {
    // Make the current angle the new zero by negating it into angle_shift.
    for (int i = 0; i < 3; ++i)
        angle_shift[i] = -angle[i];
}

// Returns angle[] with angle_shift[] applied.
std::array<float, 3> MetaMotionController::getAngle() {
    return { angle[0] + angle_shift[0],
             angle[1] + angle_shift[1],
             angle[2] + angle_shift[2] };
}

// ---------------------------------------------------------------------------
// GATT bridge helpers
//
// The MetaWear C SDK communicates with the sensor entirely via four callbacks:
// read, write, notify, and disconnect. These static functions translate those
// calls into SimpleBLE operations on our peripheral.
//
// UUIDs are stored as two 64-bit halves (high/low) in MblMwGattChar and must
// be converted to the "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx" string format
// expected by SimpleBLE. Note: SimpleBLE lowercases UUIDs internally, so we
// must use lowercase hex (%02x) here to match.
// ---------------------------------------------------------------------------

static std::string HighLow2Uuid(uint64_t high, uint64_t low) {
    const uint8_t* h = reinterpret_cast<const uint8_t*>(&high);
    const uint8_t* l = reinterpret_cast<const uint8_t*>(&low);
    char uuid[37];
    std::sprintf(uuid,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        h[7], h[6], h[5], h[4], h[3], h[2], h[1], h[0],
        l[7], l[6], l[5], l[4], l[3], l[2], l[1], l[0]);
    return uuid;
}

void MetaMotionController::read_gatt_char(void* context, const void* caller,
                                          const MblMwGattChar* characteristic,
                                          MblMwFnIntVoidPtrArray handler) {
    auto* self = static_cast<MetaMotionController*>(context);
    auto data = self->peripheral.read(
        HighLow2Uuid(characteristic->service_uuid_high, characteristic->service_uuid_low),
        HighLow2Uuid(characteristic->uuid_high, characteristic->uuid_low));
    handler(caller, reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

void MetaMotionController::write_gatt_char(void* context, const void* caller,
                                           MblMwGattCharWriteType /*writeType*/,
                                           const MblMwGattChar* characteristic,
                                           const uint8_t* value, uint8_t length) {
    auto* self = static_cast<MetaMotionController*>(context);
    self->peripheral.write_command(
        HighLow2Uuid(characteristic->service_uuid_high, characteristic->service_uuid_low),
        HighLow2Uuid(characteristic->uuid_high, characteristic->uuid_low),
        SimpleBLE::ByteArray(value, static_cast<size_t>(length)));
}

void MetaMotionController::enable_char_notify(void* context, const void* caller,
                                              const MblMwGattChar* characteristic,
                                              MblMwFnIntVoidPtrArray handler,
                                              MblMwFnVoidVoidPtrInt ready) {
    auto* self = static_cast<MetaMotionController*>(context);
    self->peripheral.notify(
        HighLow2Uuid(characteristic->service_uuid_high, characteristic->service_uuid_low),
        HighLow2Uuid(characteristic->uuid_high, characteristic->uuid_low),
        [handler, caller](SimpleBLE::ByteArray payload) {
            handler(caller, reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
        });
    ready(caller, MBL_MW_STATUS_OK);
}

void MetaMotionController::on_disconnect(void* /*context*/, const void* caller,
                                         MblMwFnVoidVoidPtrInt handler) {
    if (handler)
        handler(caller, MBL_MW_STATUS_OK);
}
