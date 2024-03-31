#pragma once

#include <iostream>
#include <vector>



using std::vector;
// �����������㷨����
class HungarianAlgorithm
{
	
public:

	HungarianAlgorithm();
	~HungarianAlgorithm();
	// ��⺯��
	double Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
private:
	// ���ŷ��亯��
	void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nOfRows, int nOfColumns);
	// �������������ɺ���
	void BuildAssignmentVector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns);
	// ������ۼ��㺯��
	void ComputeAssignmentCost(int* assignment, double* cost, double* distMatrix, int nOfRows);
	// ���ǵ�������
	void CoverAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
	// ��ָ�ɺ���
	void TryAssign(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
	// �����������
	void MatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
	// ��Ǿ�����º���
	void StarMatrixUpdate(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
	// ���۾����������
	void CostMatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
};

