#ifndef LOCAL_DATATYPES_H_
#define LOCAL_DATATYPES_H_

// Added by AC to store measured values
// struct bldcMeasure {
// 	//7 Values int16_t not read(14 byte)
// 	float avgMotorCurrent;
// 	float avgInputCurrent;
// 	float dutyCycleNow;
// 	long rpm;
// 	float inpVoltage;
// 	float ampHours;
// 	float ampHoursCharged;
// 	//2 values int32_t not read (8 byte)
// 	long tachometer;
// 	long tachometerAbs;
// };

typedef struct{
    double temp_mos;
    double temp_motor;
    double current_motor;
    double current_in;
    double id;
    double iq;
    double rpm;
    double v_in;
    double duty_now;
    double amp_hours;
    double amp_hours_charged;
    double watt_hours;
    double watt_hours_charged;
    int tachometer;
    int tachometer_abs;
    double position;
    mc_fault_code fault_code;
} bldcMeasure;


//Define remote Package

struct remotePackage {

	int		valXJoy;
	int		valYJoy;
	boolean	valUpperButton;
	boolean	valLowerButton;

};

#endif