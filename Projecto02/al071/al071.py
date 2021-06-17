# -*- coding: utf-8 -*-
"""
Grupo al071
Bernardo Rosa id #88077
Joao Jorge id #88079
"""
import numpy as np
import math
import random


def equal_classify(examples):
	classify = examples[0][1]
	equal = True
	for ex in examples:
		if classify != ex[1]:
			equal = False
			break
	return equal
		

def plurality_value(examples):
   
	classify_freq = [0, 0]
	
	for ex in examples:
		if ex[1]:
			classify_freq[1] += 1
		else:
			classify_freq[0] += 1
	
	return classify_freq.index(max(classify_freq))

def entropy(p,n):
	if p == 0 or n == 0:
		return 0

	entropy = -p/(n+p) * math.log(p/(n+p),2) - n/(n+p) * math.log(n/(n+p),2)
	return entropy		

def importance(a, examples):

	

	p = 0
	n = 0
	for i in examples:
		if i[1] == 1:
			p+=1
		else:
			n+=1
	entropy_Y = entropy(p,n)


	#feature == 1, Y ==1
	p_1 = 0
	#feature == 0, Y == 1
	n_1 = 0
	#feature == 1, Y == 0
	p_0 = 0
	#feature == 0, Y == 0
	n_0 = 0

	for i in examples:

		#se exemplo for 1
		if i[0][a] == 1:
			if i[1] == 1:
				#feature positiva e classificacao positiva
				p_1+=1
			else:
				#feature positiva e classificacao negativa
				n_1+=1
		else:
			if i[1] == 1:
				#feature negativa e classificacao positiva
				p_0+=1
			else:
				#feature negativa e classificacao negativa
				n_0+=1

	#calcula ganho de entropia			
	entr_1 = entropy(p_1,n_1);
	entr_2 = entropy(p_0,n_0);

	entropy_win = entropy_Y - (p_1+n_1)/(p+n) *entr_1 - (p_0+n_0)/(p+n) *entr_2

	return entropy_win


	

def decision_tree_learning(examples, attributes, parent_examples):
	
	if not len(examples):
		return plurality_value(parent_examples)
	elif equal_classify(examples):
		if examples[0][1]:
			return 1
		else:
			return 0


	elif not len(attributes):
		return plurality_value(examples)
		
	else:
		
		importance_max = 0
		argmaxlist = []
		argmax = 0
		for i in range(len(attributes)):
			a_importance = importance(attributes[i], examples)
			if a_importance >= importance_max:
				importance_max = a_importance
				argmax = attributes[i]
				argmaxlist.append(argmax)
		
		A = random.choice(argmaxlist)
		


		tree = [A]

		attributes.remove(A)
		
		for vk in range(2):
			exs = []
			examples_len = len(examples)
			for i in range(examples_len):
				if examples[i][0][A] == vk:
					exs.append(examples[i])
			if vk:
				subtree = decision_tree_learning(exs, attributes, examples)
			else:
				subtree = decision_tree_learning(exs, attributes.copy(), examples)

			tree.append(subtree)

		return tree


def createdecisiontree(D,Y, noise = False):
	examples = []
	D_len = len(D)
	for i in range(D_len):
		examples.append([D[i].tolist(), Y[i]])
	arguments = []
	for i in range(D[0].size):
		arguments.append(i)
	
	result = decision_tree_learning(examples, arguments, examples)

	if result==0 or result==1:
		return [0, result, result]
	else:
		return result
