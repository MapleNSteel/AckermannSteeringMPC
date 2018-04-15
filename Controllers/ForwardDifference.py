import numpy as np

def forwardDifference(N, l):
	D=np.eye(N)
	
	for i in range(l, N):
		D[i, i-l]=-1

	return D

def main():
	D=forwardDifference(10,2)
	print(D)

if __name__=="__main__":
	main()
