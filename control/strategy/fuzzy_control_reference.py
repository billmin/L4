import os, sys
import numpy as np


class FuzzySteer:
	def __init__(self):
		self._curv_fuzzy_set = ['CURV_NB+', 'CURV_NB', 'CURV_NS', 'CURV_NS-', 'CURV_ZO', 'CURV_PS-', 'CURV_PS', 'CURV_PB', 'CURV_PB+']
		self._curv_domain = [-0.13, -0.067, -0.022, -0.013, 0.0, 0.013, 0.022, 0.067, 0.13]
		self._steer_fuzzy_set = ['STEER_NB+', 'STEER_NB', 'STEER_NS', 'STEER_NS-', 'STEER_ZO', 'STEER_PS-', 'STEER_PS', 'STEER_PB', 'STEER_PB+']
		self._steer_domain = [-400.0, -200.0, -50.0, -10.0, 0.0, 10.0, 50.0, 200.0, 400.0]
		self._fuzzy_rules = dict()
		for idx, cf in enumerate(self._curv_fuzzy_set):
			self._fuzzy_rules[cf] = self._steer_fuzzy_set[idx]

		# membership function definition
		self._curv_membership_func_def = dict()
		self._steer_membership_func_def = dict()
		# curv membership
		self._curv_membership_func_def['CURV_NB+'] = {'func_type':'zmf', 'anchors':[-0.13, -0.1]}
		self._curv_membership_func_def['CURV_NB'] = {'func_type':'trimf', 'anchors':[-0.13, -0.067, -0.022]}
		self._curv_membership_func_def['CURV_NS'] = {'func_type':'trimf', 'anchors':[-0.08, -0.022, -0.0075]}
		self._curv_membership_func_def['CURV_NS-'] = {'func_type':'trimf', 'anchors':[-0.044, -0.013, 0.0]}
		self._curv_membership_func_def['CURV_ZO'] = {'func_type':'trapmf', 'anchors':[-0.013, -0.0025, 0.0025, 0.013]}
		self._curv_membership_func_def['CURV_PS-'] = {'func_type':'trimf', 'anchors':[0.0, 0.013, 0.044]}
		self._curv_membership_func_def['CURV_PS'] = {'func_type':'trimf', 'anchors':[0.0075, 0.022, 0.08]}
		self._curv_membership_func_def['CURV_PB'] = {'func_type':'trimf', 'anchors':[0.022, 0.067, 0.13]}
		self._curv_membership_func_def['CURV_PB+'] = {'func_type':'zmf', 'anchors':[0.1, 0.13]}
		# steer membership
		self._steer_membership_func_def['STEER_NB+'] = {}
		self._steer_membership_func_def['STEER_NB'] = {}
		self._steer_membership_func_def['STEER_NS'] = {}
		self._steer_membership_func_def['STEER_NS-'] = {}
		self._steer_membership_func_def['STEER_ZO'] = {}
		self._steer_membership_func_def['STEER_PS-'] = {}
		self._steer_membership_func_def['STEER_PS'] = {}
		self._steer_membership_func_def['STEER_PB'] = {}
		self._steer_membership_func_def['STEER_PB+'] = {}
		
		# membership
		self._curv_membership = dict()
		self._steer_membership = dict()
		# curve's membership
		self._curv_membership['CURV_NB+'] = np.array([1.0, 0.0,   0.0,  0.0,   0.0,   0.0,   0.0,  0.0,   0.0])
		self._curv_membership['CURV_NB'] = np.array([ 0.0, 1.0,   0.0,  0.0,   0.0,   0.0,   0.0,  0.0,   0.0])
		self._curv_membership['CURV_NS'] =  np.array([0.0, 0.224, 1.0,  0.379, 0.0,   0.0,   0.0,  0.0,   0.0])
		self._curv_membership['CURV_NS-'] = np.array([0.0, 0.0,   0.71, 1.0,   0.192, 0.0,   0.0,  0.0,   0.0])
		self._curv_membership['CURV_ZO'] = np.array([ 0.0, 0.0,   0.0,  0.0,   1.0,   0.0,   0.0,  0.0,   0.0])
		self._curv_membership['CURV_PS-'] = np.array([0.0, 0.0,   0.0,  0.0,   0.192, 1.0,   0.71, 0.0,   0.0])
		self._curv_membership['CURV_PS'] = np.array([ 0.0, 0.0,   0.0,  0.0,   0.0,   0.379, 1.0,  0.224, 0.0])
		self._curv_membership['CURV_PB'] = np.array([ 0.0, 0.0,   0.0,  0.0,   0.0,   0.0,   0.0,  1.0,   0.0])
		self._curv_membership['CURV_PB+'] = np.array([0.0, 0.0,   0.0,  0.0,   0.0,   0.0,   0.0,  0.0,   1.0])
		# steer's membership
		self._steer_membership['STEER_NB+'] = np.array([1.0,   0.333, 0.0,   0.0,   0.0, 0.0,   0.0,   0.0,   0.0])
		self._steer_membership['STEER_NB'] = np.array([ 0.333, 1.0,   0.25,  0.05,  0.0, 0.0,   0.0,   0.0,   0.0])
		self._steer_membership['STEER_NS'] = np.array([ 0.0,   0.0,   1.0,   0.2,   0.0, 0.0,   0.0,   0.0,   0.0])
		self._steer_membership['STEER_NS-'] = np.array([0.0,   0.0,   0.556, 1.0,   0.0, 0.0,   0.0,   0.0,   0.0])
		self._steer_membership['STEER_ZO'] = np.array([ 0.0,   0.0,   0.0,   0.333, 1.0, 0.333, 0.0,   0.0,   0.0])
		self._steer_membership['STEER_PS-'] = np.array([0.0,   0.0,   0.0,   0.0,   0.0, 1.0,   0.556, 0.0,   0.0])
		self._steer_membership['STEER_PS'] = np.array([ 0.0,   0.0,   0.0,   0.0,   0.0, 0.2,   1.0,   0.0,   0.0])
		self._steer_membership['STEER_PB'] = np.array([ 0.0,   0.0,   0.0,   0.0,   0.0, 0.05,  0.25,  1.0,   0.333])
		self._steer_membership['STEER_PB+'] = np.array([0.0,   0.0,   0.0,   0.0,   0.0, 0.0,   0.0,   0.333, 1.0])
		# get relationship matrix
		n_rank, n_col = len(self._curv_domain), len(self._steer_domain)
		self._relationship_matrix = np.zeros([n_rank, n_col])
		for i in range(n_rank):
			for j in range(n_col):
				buf = []
				for k, v in self._fuzzy_rules.items():
					buf.append(np.min([self._curv_membership[k][i], self._steer_membership[v][j]]))
				self._relationship_matrix[i][j] = np.max(buf)
	
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

	@property
	def relationship_matrix(self):
		return self._relationship_matrix

	def getDefuzzificationByDomain(self, in_curv):
		n = len(self._curv_fuzzy_set)
		in_curv_membership = np.zeros([n])

		for i, cf in enumerate(self._curv_fuzzy_set):
			func_type = self._curv_membership_func_def[cf]['func_type']
			anchors = self._curv_membership_func_def[cf]['anchors']
			##
			if func_type == 'zmf':
				if anchors[0] < 0.0:
					if in_curv <= anchors[0]:
						in_curv_membership[i] = 1.0
					elif in_curv >= anchors[-1]:
						in_curv_membership[i] = 0.0
					else:
						in_curv_membership[i] = (anchors[-1]-in_curv)/(anchors[-1]-anchors[0])
				else:
					if in_curv <= anchors[0]:
						in_curv_membership[i] = 0.0
					elif in_curv >= anchors[-1]:
						in_curv_membership[i] = 1.0
					else:
						in_curv_membership[i] = (in_curv-anchors[0])/(anchors[-1]-anchors[0])
			elif func_type == 'trimf':
				if in_curv <= anchors[0]:
					in_curv_membership[i] = 0.0
				elif anchors[0] < in_curv <= anchors[1]:
					in_curv_membership[i] = (in_curv-anchors[0])/(anchors[1]-anchors[0])
				elif anchors[1] < in_curv <= anchors[-1]:
					in_curv_membership[i] = (anchors[-1]-in_curv)/(anchors[-1]-anchors[1])
				else:
					in_curv_membership[i] = 0.0
			elif func_type == 'trapmf':
				if in_curv <= anchors[0]:
					in_curv_membership[i] = 0.0
				elif anchors[0] < in_curv <= anchors[1]:
					in_curv_membership[i] = (in_curv-anchors[0])/(anchors[1]-anchors[0])
				elif anchors[1] < in_curv <= anchors[2]:
					in_curv_membership[i] = 1.0
				elif anchors[2] < in_curv <= anchors[-1]:
					in_curv_membership[i] = (anchors[-1]-in_curv)/(anchors[-1]-anchors[2])
				else:
					in_curv_membership[i] = 0.0
		##
		idx_belong_to_membership = 0
		max_v = np.max(in_curv_membership)
		for i in range(n):
			if in_curv_membership[i] == max_v:
				idx_belong_to_membership = i
				break

		## fuzzy member and membership
		in_curv_fuzzy = self._curv_fuzzy_set[idx_belong_to_membership]
		in_curv_fuzzified_membership = self._curv_membership[in_curv_fuzzy]

		# infer fuzzy membership of steer
		out_fuzzy = np.zeros([len(self._steer_domain)])
		for col in range(self._relationship_matrix.shape[1]):
			buf = np.zeros([len(self._curv_domain)])
			for i in range(len(self._curv_domain)):
				buf[i] = np.min([self._relationship_matrix[:,col][i], in_curv_fuzzified_membership[i]])
			##
			out_fuzzy[col] = np.max(buf)

		# using centroid
		out_defuzzification = sum(out_fuzzy*np.array(self._steer_domain))/sum(out_fuzzy)

		return out_defuzzification

	def getDefuzzificationByFuzzy(self, in_curv_fuzzy):
		## fuzzy member and membership
		in_curv_fuzzified_membership = self._curv_membership[in_curv_fuzzy]

		# infer fuzzy membership of steer
		out_fuzzy = np.zeros([len(self._steer_domain)])
		for col in range(self._relationship_matrix.shape[1]):
			buf = np.zeros([len(self._curv_domain)])
			for i in range(len(self._curv_domain)):
				buf[i] = np.min([self._relationship_matrix[:,col][i], in_curv_fuzzified_membership[i]])
			##
			out_fuzzy[col] = np.max(buf)

		# using centroid
		out_defuzzification = sum(out_fuzzy*np.array(self._steer_domain))/sum(out_fuzzy)

		return out_defuzzification


class FuzzyThrottle:
	pass


class FuzzyBrake:
	pass


if __name__ == '__main__':
	fs = FuzzySteer()
	print(fs.curv_fuzzy_set)
	print(fs.steer_fuzzy_set)
	print(fs.curv_domain)
	print(fs.steer_domain)
	print(fs.fuzzy_rules)
	print(fs.relationship_matrix)
	for i, d in enumerate(fs.curv_domain):
		print(fs.steer_domain[i], fs.getDefuzzificationByDomain(d))
	print('------------------------------')
	for s in fs.curv_fuzzy_set:
		print(fs.getDefuzzificationByFuzzy(s))
