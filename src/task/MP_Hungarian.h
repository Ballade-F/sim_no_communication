#pragma once

#include <iostream>
#include <vector>



using std::vector;
// 匈牙利分配算法工具
class HungarianAlgorithm
{
	
public:

	HungarianAlgorithm();
	~HungarianAlgorithm();
	// 求解函数
	double Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
private:
	// 最优分配函数
	void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nOfRows, int nOfColumns);
	// 分配结果向量生成函数
	void BuildAssignmentVector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns);
	// 分配代价计算函数
	void ComputeAssignmentCost(int* assignment, double* cost, double* distMatrix, int nOfRows);
	// 覆盖调整函数
	void CoverAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
	// 试指派函数
	void TryAssign(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
	// 矩阵调整函数
	void MatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
	// 标记矩阵更新函数
	void StarMatrixUpdate(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
	// 代价矩阵调整函数
	void CostMatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
};

