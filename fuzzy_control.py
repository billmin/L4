import os, sys
import numpy as np


class FuzzySteer:
	def __init__(self):
		#steer_domain = [-400.0, -200.0, -50.0, -10.0, 0.0, 10.0, 50.0, 200.0, 400.0]
		self._curv_fuzzy_set = ['CURV_NB+', 'CURV_NB', 'CURV_NS', 'CURV_NS-', 'CURV_ZO', 'CURV_PS-', 'CURV_PS', 'CURV_PB', 'CURV_PB+']
		self._curv_domain = [-400.0, -200.0, -50.0, -10.0, 0.0, 10.0, 50.0, 200.0, 400.0]
		self._steer_fuzzy_set = ['STEER_NB+', 'STEER_NB', 'STEER_NS', 'STEER_NS-', 'STEER_ZO', 'STEER_PS-', 'STEER_PS', 'STEER_PB', 'STEER_PB+']
		self._steer_domain = [-4, -3, -2, -1, 0, 1, 2, 3, 4]
		self._fuzzy_rules = dict()
		for idx, cf in enumerate(self._curv_fuzzy_set):
			self._fuzzy_rules[cf] = self._steer_fuzzy_set[idx]
		
		self._curv_membership = dict()
		self._steer_membership = dict()
		# curve's membership
		self._curv_membership['CURV_NB+'] = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._curv_membership['CURV_NB'] = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._curv_membership['CURV_NS'] = np.array([0.0, 0.224, 1.0, 0.379, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._curv_membership['CURV_NS-'] = np.array([0.0, 0.0, 0.71, 1.0, 0.192, 0.0, 0.0, 0.0, 0.0])
		self._curv_membership['CURV_ZO'] = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
		self._curv_membership['CURV_PS-'] = np.array([0.0, 0.0, 0.0, 0.0, 0.192, 1.0, 0.71, 0.0, 0.0])
		self._curv_membership['CURV_PS'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.379, 1.0, 0.224, 0.0])
		self._curv_membership['CURV_PB'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
		self._curv_membership['CURV_PB+'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
		# steer's membership
		self._steer_membership['STEER_NB+'] = np.array([1.0, 0.333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._steer_membership['STEER_NB'] = np.array([0.333, 1.0, 0.25, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._steer_membership['STEER_NS'] = np.array([0.0, 0.0, 1.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._steer_membership['STEER_NS-'] = np.array([0.0, 0.0, 0.556, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self._steer_membership['STEER_ZO'] = np.array([0.0, 0.0, 0.0, 0.333, 1.0, 0.333, 0.0, 0.0, 0.0])
		self._steer_membership['STEER_PS-'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.556, 0.0, 0.0])
		self._steer_membership['STEER_PS'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.0])
		self._steer_membership['STEER_PB'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.25, 1.0, 0.333])
		self._steer_membership['STEER_PB+'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.333, 1.0])
	
	@property
	def curv_fuzzy_set(self):
		return self._curv_fuzzy_set

	@property
	def curv_domain(self):
		return self._curv_domain

	@property
	def steer_fuzzy_set(self):
		return self._steer_fuzzy_set
	
	@property
	def steer_domain(self):
		return self._steer_domain

	@property
	def fuzzy_rules(self):
		return self._fuzzy_rules


	def getDefuzzification(self):
		pass

		


#class FuzzyThrottle:



#class FuzzyBrake:


if __name__ == '__main__':
	fs = FuzzySteer()
	print(fs.curv_fuzzy_set)
	print(fs.steer_fuzzy_set)
	print(fs.curv_domain)
	print(fs.steer_domain)
