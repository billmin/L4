import os, sys


class FuzzySteer:
		'''
		steer_range_domain = [-400.0, -200.0, -50.0, -10.0, 0.0, 10.0, 50.0, 200.0, 400.0]
		'''
	def __init__(self, curv_domain, steer_domain):
		self._curv_fuzzy_set = ['CURV_NB+', 'CURV_NB', 'CURV_NS', 'CURV_NS-', 'CURV_ZO', 'CURV_PS-', 'CURV_PS', 'CURV_PB', 'CURV_PB+']
		self._curv_domain = curv_domain
		self._steer_fuzzy_set = ['STEER_NB+', 'STEER_NB', 'STEER_NS', 'STEER_NS-', 'STEER_ZO', 'STEER_PS-', 'STEER_PS', 'STEER_PB', 'STEER_PB+']
		self._steer_domain = steer_domain
		self._fuzzy_rules = dict()
		for idx, cf in enumerate(self._curv_fuzzy_set):
			self._fuzzy_rules[cf] = self._steer_fuzzy_set[idx]
		self._curvz
	
	@property
	def curv_fuzzy_set(self):
		return self._curv_fuzzy_set

	@property
	def curv_domain(self):
		return self._curv_domain

	@property
	def steer_fuzzy_set(self):
		return self._steer_range_fuzzy_set
	
	@property
	def steer_domain(self):
		return self._steer_range_domain

	@property
	def fuzzy_rules(self):
		return self._fuzzy_rules

	def setCurvMmembership(self, fuzzy):
		
	
	def setSteerMembership(self, fuzzy):
		
		
	def calcRelationMatrix(self):



	def getRelationMatrix(self):



	def getDefuzzification(self):


		


class FuzzyThrottle:



class FuzzyBrake:


if __name__ == '__main__':
	fs = FuzzySteer()

	print(fs.curvature_fuzzy_set)
	print(fs.steer_range_fuzzy_set)
	print(fs.curvature_domain)
	print(fs.steer_range_domain)
