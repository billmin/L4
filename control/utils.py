import numpy as np
import time
import matplotlib.pyplot as plt
import scipy.interpolate, scipy.optimize


def polyfit_solver(x, y, n):
	f = np.polyfit(x, y, n)
	p = np.poly1d(f)

	return p


def intercept_solver(x, y1, y2):
	assert len(x)==len(y1)==len(y2), "dimension conflict!"

	interp1 = scipy.interpolate.InterpolatedUnivariateSpline(x, y1)
	interp2 = scipy.interpolate.InterpolatedUnivariateSpline(x, y2)

	new_x = np.linspace(x.min(), x.max(), 100)
	new_y1 = interp1(new_x)
	new_y2 = interp2(new_x)

	def __difference(x):
		return np.abs(interp1(x) - interp2(x))

	x_at_crossing = scipy.optimize.fsolve(__difference, x0=3.0)

	return x_at_crossing, interp1, interp2

if __name__ == '__main__':
	x  = np.linspace(1, 4, 10)
	#x  = np.array([1.0, 1.2, 1.5, 4.0])
	y1 = np.sin(x)
	y2 = 0.05*x
	y3 = 2.0*np.ones(x.shape[0])
	#print(y2)

	t0 = time.time()
	x_at_crossing, interp1, interp2 = intercept_solver(x, y1, y2)
	t1 = time.time()
	#print("time: ", t1-t0)
	x_at_crossing_, interp1_, interp2_ = intercept_solver(x, y1, y3)
	print("x_at_crossing: ", x_at_crossing[0], interp1(x_at_crossing)[0])
	#print("x_at_crossing_: ", x_at_crossing_[0], interp1_(x_at_crossing)[0])
	#print('--------------------')
	
	t2 = time.time()
	p1 = polyfit_solver(x, y1, 3)
	t3 = time.time()
	print("time: ", t3-t2)
	p2 = polyfit_solver(x, y2, 3)
	p3 = polyfit_solver(x, y3, 3)
	#print(p2(x))
	#print(p3(x))
	
	x_at_crossing, interp1, interp2 = intercept_solver(x, p1(x), p2(x))
	x_at_crossing_, interp1_, interp2_ = intercept_solver(x, p1(x), p3(x))
	#print("x_at_crossing: ", x_at_crossing[0], interp1(x_at_crossing)[0])
	#print("x_at_crossing_: ", x_at_crossing_[0], interp1_(x_at_crossing_)[0])
	
