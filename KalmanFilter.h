#pragma once

class KalmanFilter {
private:
	double gain;
	int memory;
	double* values;
  int currentAmount = 0;

	double* ShiftArray(double arr[]);

public:
	KalmanFilter();
	KalmanFilter(double gain, int memory);
	double Filter(double value);

};
