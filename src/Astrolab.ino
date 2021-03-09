
////  !!!! This sketch is not finished yet
///   It just shows how to add/integrate garage door charachteristic with apple
///   garage door functionality should be reviewed and properly implemented
#include <Arduino.h>
// Analog Read for ESP32
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <SPIFFS.h>
#include <WiFiManager.h>        //https://github.com/tzapu/WiFiManager


const char* HOSTNAME = "Astrolab";

const int identity_led = 2;

extern "C"{
#include "homeintegration.h"
}
#include <hapfilestorage/hapfilestorage.hpp>


homekit_service_t* svc_astrtolab = NULL;
homekit_service_t* svc_astrobattery = NULL;

const int RPWM = 16;
const int RPWM_channel_0 = 0;       // channel
const int RPWM_resolution = 8;      // use 12 bit precission
const int RPWM_base_freq = 5000;    // use 5000 Hz as a LEDC base frequency

const int LPWM = 2;
const int LPWM_channel_1 = 1;       // channel
const int LPWM_resolution = 8;      // use 12 bit precission
const int LPWM_base_freq = 5000;    // use 5000 Hz as a LEDC base frequency

// Batterie Level
const int BL_pin = 34;
// Stall Detector
const int LR_IS = 33;
esp_adc_cal_characteristics_t *adc_chars;

uint8_t current_table_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED;
uint8_t target_door_state = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN;


void startwifimanager() {
    WiFiManager wifiManager;
    if (!wifiManager.autoConnect(HOSTNAME, NULL)) {
        ESP.restart();
        delay(1000);
    }
}


void setup() {
    Serial.begin(9600);
    delay(500);

    // We start by connecting to a WiFi network
    if (!SPIFFS.begin(true)) {
        Serial.print("SPIFFS Mount failed");
    }

    //////////////////////////////////////////////////////
    // Analog
    //////////////////////////////////////////////////////
    #define DEFAULT_VREF    1100
    adc1_config_width(ADC_WIDTH_BIT_12);   //Range 0-4096
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF,adc_chars);
    // LR_IS
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
    // BatteryLevel
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V

    //////////////////////////////////////////////////////
    // configure PWM functionalities
    //////////////////////////////////////////////////////
    // RPWM
    ledcSetup(RPWM_channel_0, RPWM_base_freq, RPWM_resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(RPWM, RPWM_channel_0);
    pinMode(RPWM, OUTPUT);
    // LPWM
    ledcSetup(LPWM_channel_1, LPWM_base_freq, LPWM_resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LPWM, LPWM_channel_1);
    pinMode(LPWM, OUTPUT);

    //////////////////////////////////////////////////////
    // WIFI
    //////////////////////////////////////////////////////
    startwifimanager();

    //////////////////////////////////////////////////////
    // HOMEKIT
    //////////////////////////////////////////////////////
    // this is for custom storage usage
    // using \pair.dat   file in our spiffs system
    Serial.print("Free heap: ");
    Serial.println(system_get_free_heap_size());

    // Remove /pair.dat so we create a new one, if already paired
    // SPIFFS.remove("/pair.dat");

    init_hap_storage("/pair.dat");

    // Our accessory type is garage door, apple interface will proper show that
    hap_setbase_accessorytype(homekit_accessory_category_garage);

    // init base properties
    hap_initbase_accessory_service("ES", "Rabourdin Inc", "3-1415", "Astrolab", "1.0");

    // for base accessory registering door
    svc_astrtolab = hap_add_garagedoor_service("garagedoor", hap_callback_process, 0);
    svc_astrobattery = hap_add_battery_service("batterry", hap_callback_battery, 0);

    // set a default (initial) value to be informed Apple about initial state
    homekit_characteristic_t * ch_astrolab_state = homekit_service_characteristic_by_type(svc_astrtolab, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE);
    homekit_characteristic_t * ch_astrolab_obstruction = homekit_service_characteristic_by_type(svc_astrtolab, HOMEKIT_CHARACTERISTIC_OBSTRUCTION_DETECTED);
    hap_set_initial_characteristic_int_value(ch_astrolab_state, current_table_state);       // start with a close table
    hap_set_initial_characteristic_bool_value(ch_astrolab_obstruction, false);              // no obstruction

    // Informational Start
    int voltage = getPinVoltage(ADC1_CHANNEL_5);
    Serial.printf("@@@ Setup: voltageFromIS %d\n mV\n", voltage);
    
    hap_init_homekit_server();
}


void loop() {
    batteryManager();
    delay(1000*60*10); // 10 minutes
    // delay(1000*2); // 2 seconds (during tests)
}

//////////////////////////////////////////////////////
// helper to get the pin voltage (mV)
//////////////////////////////////////////////////////
int getPinVoltage(adc1_channel_t adc_channel) {
    uint32_t pinVoltage = 0;
    for (size_t i = 0; i < 20; i++)
    {
        uint32_t reading =  adc1_get_raw(adc_channel);
        pinVoltage += esp_adc_cal_raw_to_voltage(reading, adc_chars);
        delay(5);
    }
    pinVoltage /= 20;
    return pinVoltage;
}

//////////////////////////////////////////////////////
// Battery
//////////////////////////////////////////////////////
void hap_callback_battery(homekit_characteristic_t *ch, homekit_value_t value, void *context) {}

void batteryManager() {
    // 12.8V 
    const float q_0 = 1.562475653120889E+00;
    const float a = -1.327041316170583E+03;
    const float b = 5.803108563000592E-01;

    uint32_t pinVoltage = getPinVoltage(ADC1_CHANNEL_6);

    // Hyperbolic Decline
    float batteryVoltage = q_0*pow((1 + b*pinVoltage/a),(-1/b));

    Serial.printf("@@@ batteryManager: Pin Voltage %d mV\n", pinVoltage);
    Serial.printf("@@@ batteryManager: Battery Voltage %f V\n", batteryVoltage);
    
    setBatteryLevel(batteryVoltage);
}   


void setBatteryLevel(float batteryVoltage) {
    int level = 0;
    if(batteryVoltage > 12) {
        level = 100;
    }
    else if(batteryVoltage > 11.5) {
        level = 90;
    }
    else if(batteryVoltage > 11) {
        level = 70;
    }
    else if(batteryVoltage > 10.5) {
        level = 50;
    }
    else if(batteryVoltage > 10) {
        level = 30;
    }
    else if(batteryVoltage > 9.5) {
        level = 10;
    }
    else {
        level = 0;
    }
    homekit_characteristic_t * ch_astrobattery_state = homekit_service_characteristic_by_type(svc_astrobattery, HOMEKIT_CHARACTERISTIC_BATTERY_LEVEL);
    hap_set_initial_characteristic_int_value(ch_astrobattery_state, level);
}


//////////////////////////////////////////////////////
// Motor
//////////////////////////////////////////////////////
const int STOP = 0;
const int CONTINUE = 1;

void current_state_set(uint8_t new_state) {
    homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(svc_astrtolab, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE);
    
    Serial.printf("@@@ current_state_set >> new_state: %d\n", new_state);
    Serial.printf("@@@ current_state_set >> current_table_state: %d\n", current_table_state);

    if (ch_currentstate && current_table_state != new_state) {
        Serial.printf("@@@ current_state_set >> IN\n");
        current_table_state = new_state;
        //HAP_NOTIFY_CHANGES(int, ch_currentstate, new_state, 0);
        ch_currentstate->value.int_value = new_state;
        homekit_characteristic_notify(ch_currentstate, ch_currentstate->value);
    }
    else if(ch_currentstate && new_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING || new_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
        Serial.printf("@@@ current_state_set >> FORCE\n");
        //HAP_NOTIFY_CHANGES(int, ch_currentstate, new_state, 0);
        ch_currentstate->value.int_value = new_state;
        homekit_characteristic_notify(ch_currentstate, ch_currentstate->value);
    }
}


void hap_callback_process(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    if(!svc_astrtolab){
        Serial.println("@@@ hap_callback >> service not defined");
        return;
    }

    homekit_characteristic_t * ch_currentstate = homekit_service_characteristic_by_type(svc_astrtolab, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE);
    homekit_characteristic_t * ch_targetstate = homekit_service_characteristic_by_type(svc_astrtolab, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE);
    // homekit_characteristic_t * ch_obstruction = homekit_service_characteristic_by_type(svc_astrtolab, HOMEKIT_CHARACTERISTIC_OBSTRUCTION_DETECTED);

    if(ch == ch_targetstate) {
        Serial.println("@@@ hap_callback >> ch_targetstate");
        int target_table_state = value.int_value;

        if(
            (current_table_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING && target_table_state == HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED)||
            (current_table_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING && target_table_state == HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN)
        ) {
            hardStop(target_table_state);
        }

        if(target_table_state == current_table_state) {
            Serial.println("target_state_set ignored: table is already in that state, nothing to do!");
            return;
        }

        if(smoothStart(target_table_state, 150, 170) == CONTINUE) {
            getCurrentSensingFromRun(target_table_state);
        }

        hardStop(target_table_state);
    }
    else if(ch == ch_currentstate) {
        Serial.printf("@@@ hap_callback >> current state see as: %d\n", ch_currentstate->value.int_value);
        Serial.printf("@@@ hap_callback >> ch_currentstate: %d\n", current_table_state);
        if( current_table_state == ch_currentstate->value.int_value ) {
            return;
        }

        // HAP_NOTIFY_CHANGES(int, ch_currentstate, current_table_state, 0);
        ch_currentstate->value.int_value = current_table_state;
        homekit_characteristic_notify(ch_currentstate, ch_currentstate->value);
    }
    else {
        Serial.println("@@@ hap_callback >> UNKNOWN");
    }
}


int messageStart(int state) {
    switch (state)
    {
    case HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN:
        Serial.println("@@@ messageStart >> Opening table...");
        current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
       break;
    case HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED:
        Serial.println("@@@ messageStart >> Closing table...");
        current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
        break;
    default:
        return STOP;
    }
    return CONTINUE;
}


int channelFromState(int state) {
    if(state == HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN) {
        return RPWM_channel_0;
    }
    else if(state == HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED) {
        return LPWM_channel_1;
    }
    return -1;
}


int smoothStart(int state, int min, int max) {
    Serial.printf("@@@ smoothStart >> %d\n", state);
    if( messageStart(state) == STOP ) {
        return STOP;
    }

    int channel = channelFromState(state);

    for(size_t i = min; i < max; i++)
    {
        ledcWrite(channel, i);
        if(actionFromIS() == STOP) {
            return STOP;
        }
    }
    return CONTINUE;
}


void hardStart(int state, int speed) {
    Serial.println("@@@ hardStart");
    if( !messageStart(state) ) {
        return;
    }

    int channel = channelFromState(state);
    ledcWrite(channel, speed);
}


void hardStop(int state) {    
    Serial.println("@@@ hardStop");
    int channel = channelFromState(state);
    
    ledcWrite(channel, 0);

    current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED);
    current_state_set(state);
}

/////////////////////////////////////////////
// Sensing from IS
/////////////////////////////////////////////
int actionFromIS() {
    int voltage = getPinVoltage(ADC1_CHANNEL_5);
    
    Serial.printf("@@@ actionFromIS >> voltageFromIS %d mV\n", voltage);

    if(voltage >= 3000 || voltage < 900) {
        return STOP;
    }
    return CONTINUE;
}

void getCurrentSensingFromRun(int state) {
    bool keepRunning = true;
    while(keepRunning)
    {
        if(actionFromIS() == STOP) {
            keepRunning = false;
        }
    }
}