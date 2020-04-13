//Data of IMU "CSM-MG100"
using std::string;

int getBit(int n, uint8_t b){
	assert(0 <= n && n < 8);
  return ((0x1 << n) & b) ? 1 : 0;
}

int byte2int(uint8_t a, uint8_t b){
	return (b << 8) | a;
}

float byte2long(uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3){

	//ans = b[index] + b[index+1]*0x100 + b[index+2]*0x10000 + b[index+3]*0x1000000;

	return (a3 << 24) | (a2 << 16) | (a1 << 8) | a0;
}

double byte2double(uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7){
	return (a7 << 56) | (a6 << 48) | (a5 << 40) | (a4 << 32) |
	       (a3 << 24) | (a2 << 16) | (a1 <<  8) |  a0;
}

struct Output_Setting{
	bool output_imu_data, output_sensor_data, output_GPS_data;
	int data_type;	//IMUデータ値種別
	int frequency;	//出力周波数
};

struct Data_Status{
	string imu, ahrs, ins, GPS;
	bool using_imu, using_geomagnetic, using_temp, using_pressure, using_GPS;
};

struct CSM_Status{
  uint16_t product_id, serial_number, version;
	uint16_t year, month, day, hour, minute,seconds;
	Output_Setting output_setting;
	string calc_mode;
	bool need_proofreading;

	Data_Status sensor;
	string alignment_status;
	string GPS_status;
	string acc_status;
	string malfunction;
	string mounting_correction;
};

struct CSM_Data{
	float P_rate, Q_rate, R_rate;
	float X_acc, Y_acc, Z_acc;
	float roll, pitch, yaw;
	float latitude, longitude, altitude;
	float NS_speed, EW_speed, DU_speed;
};

class CSM{
private:

public:
	CSM_Status status;
	CSM_Data data;

	void set_status(uint8_t b[48]){
		status.sensor.GPS = b[0];
		switch ( b[0] & 0b11 ) {
			case 0b00: status.sensor.GPS = "Failure Warning";  break;
			case 0b01: status.sensor.GPS = "No Computed Data"; break;
			case 0b11: status.sensor.GPS = "Normal Operation"; break;
		}

		switch (getBit(0,b[28])) {
			case 0: status.output_setting.frequency = 100; break;
			case 1: status.output_setting.frequency = 500; break;
		}

		status.output_setting.data_type          = getBit(2,b[28]);
		status.output_setting.output_GPS_data    = getBit(1,b[29]);
		status.output_setting.output_sensor_data = getBit(2,b[29]);
		status.output_setting.output_imu_data    = getBit(4,b[29]);

		switch (byte2int(b[30],b[31])) {
			case 0x301: status.calc_mode = "normal_mode"; break;
			case 0x302: status.calc_mode = "room_mode";   break;
			default   : status.calc_mode = "error";       break;
		}

		status.sensor.using_GPS         = getBit(0,b[32]);
		status.sensor.using_pressure    = getBit(1,b[32]);
		status.sensor.using_temp        = getBit(2,b[32]);
		status.sensor.using_geomagnetic = getBit(3,b[32]);
		status.sensor.using_imu         = getBit(4,b[32]);

		status.need_proofreading = getBit(0,b[33]);

		switch (b[36]) {
			case 0x0: status.alignment_status = "none";      break;
			case 0x1: status.alignment_status = "trying";    break;
			case 0x2: status.alignment_status = "failed";    break;
			case 0x3: status.alignment_status = "successed"; break;
			default : status.alignment_status = "error";     break;
		}

		switch (b[37]) {
			case 0x0: status.GPS_status = "invalid"; break;
			case 0x1: status.GPS_status = "failed";  break;
			case 0x4: status.GPS_status = "valid";   break;
			default : status.GPS_status = "error";   break;
		}

		switch (b[38]) {
			case 0x0: status.acc_status = "none";      break;
			case 0x1: status.acc_status = "trying";    break;
			case 0x3: status.acc_status = "successed"; break;
			default : status.acc_status = "error";     break;
		}

		status.malfunction = "";
		if (getBit(4,b[41]) == 1) status.malfunction += "RS422,";
		if (getBit(5,b[41]) == 1) status.malfunction += "GPS,";
		if (getBit(6,b[41]) == 1) status.malfunction += "Sensor,";
		if (getBit(7,b[41]) == 1) status.malfunction += "Memory,";

		switch (b[44]) {
			case 0  : status.mounting_correction = "valid";   break;
			case 1  : status.mounting_correction = "invalid"; break;
			default : status.mounting_correction = "error";   break;
		}

		return;

	}

	void set_data(uint8_t b[72]){
		switch ( b[0] & 0b00000011 ) {
			case 0b00: status.sensor.ins = "Failure Warning or No GPS";  break;
			case 0b01: status.sensor.ins = "No Computed Data";           break;
			case 0b11: status.sensor.ins = "Normal Operation";           break;
		}

		switch ( b[0] & 0b00001100 ) {
			case 0b00: status.sensor.ahrs = "Failure Warning or No GPS";  break;
			case 0b01: status.sensor.ahrs = "No Computed Data";           break;
			case 0b11: status.sensor.ahrs = "Normal Operation";           break;
		}

		switch ( b[0] & 0b00110000 ) {
			case 0b00: status.sensor.imu = "Failure Warning or No GPS";  break;
			case 0b01: status.sensor.imu = "No Computed Data";           break;
			case 0b11: status.sensor.imu = "Normal Operation";           break;
		}

		data.P_rate = byte2long(b[4],b[5],b[6],b[7])     * 0.00001;  //[rad/s]
		data.Q_rate = byte2long(b[8],b[9],b[10],b[11])   * 0.00001;  //[rad/s]
		data.R_rate = -byte2long(b[12],b[13],b[14],b[15]) * 0.00001;  //[rad/s]

		data.X_acc = byte2long(b[16],b[17],b[18],b[19]) * 0.001;  //[m/s/s]
		data.Y_acc = byte2long(b[20],b[21],b[22],b[23]) * 0.001;  //[m/s/s]
		data.Z_acc = byte2long(b[24],b[25],b[26],b[27]) * 0.001;  //[m/s/s]

		data.roll  = byte2long(b[28],b[29],b[30],b[31]) * 0.0001;  //[rad]
		data.pitch = byte2long(b[32],b[33],b[34],b[35]) * 0.0001;  //[rad]
		data.yaw   = byte2long(b[36],b[37],b[38],b[39]) * 0.0001;  //[rad]

		data.latitude = byte2double(b[40],b[41],b[42],b[43],b[44],b[45],b[46],b[47]) * 0.0000000001;  //[rad] +:North -:South
		data.longitude = byte2double(b[48],b[49],b[50],b[51],b[52],b[53],b[54],b[55]) * 0.0000000001;  //[rad] +:East -:West
		data.altitude = byte2long(b[56],b[57],b[58],b[59]) * 0.0001;  //[m]

		data.NS_speed = byte2long(b[60],b[61],b[62],b[63]) * 0.01;  //[m/s]
		data.EW_speed = byte2long(b[64],b[65],b[66],b[67]) * 0.01;  //[m/s]
		data.DU_speed = byte2long(b[68],b[69],b[70],b[71]) * 0.01;  //[m/s]

		return;
	}

};
