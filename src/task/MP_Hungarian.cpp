#include <stdlib.h>
#include <cfloat>
#include <cmath>
#include "MP_Hungarian.h"


HungarianAlgorithm::HungarianAlgorithm() {}
HungarianAlgorithm::~HungarianAlgorithm() {}


/**
 * @brief 匈牙利算法求解函数
 * @param DistMatrix	代价矩阵
 * @param Assignment	输出分配结果
 * @return cost			分配代价值
 */
double HungarianAlgorithm::Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment)
{
    unsigned int nRows = DistMatrix.size();
    unsigned int nCols = DistMatrix[0].size();

    // 代价矩阵转化为列展开一维数组
    double* distMatrixIn = new double[nRows * nCols];
    int* assignment = new int[nRows];
    double cost = 0.0;
    for (unsigned int i = 0; i < nRows; i++)
    {
        for (unsigned int j = 0; j < nCols; j++)
        {
            distMatrixIn[i + nRows * j] = DistMatrix[i][j];
        }
    }

    // 调用分配最优求解函数
    assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

    Assignment.clear();
    for (unsigned int r = 0; r < nRows; r++)
        Assignment.push_back(assignment[r]);

    delete[] distMatrixIn;
    delete[] assignment;
    return cost;
}

/**
 * @brief 分配问题最优求解函数
 * @param assignment	输出分配结果
 * @param cost			代价值
 * @param distMatrixIn	代价矩阵-列展开一维形式
 * @param nOfRows		行数
 * @param nOfColumns	列数
 * @return None
 */
void HungarianAlgorithm::assignmentoptimal(int* assignment, double* cost, double* distMatrixIn, int nOfRows, int nOfColumns)
{
    double* distMatrix, * distMatrixTemp, * distMatrixEnd, * columnEnd, value, minValue;
    bool* coveredColumns, * coveredRows, * starMatrix, * newStarMatrix, * primeMatrix;
    int nOfElements, minDim, row, col;

    // 结果初始化
    *cost = 0;
    for (row = 0; row < nOfRows; row++)
        assignment[row] = -1;

    // 生成代价矩阵并作输入检验，判断所有元素是非负数
    nOfElements = nOfRows * nOfColumns;
    distMatrix = (double*)malloc(nOfElements * sizeof(double));
    distMatrixEnd = distMatrix + nOfElements;

    for (row = 0; row < nOfElements; row++)
    {
        value = distMatrixIn[row];
        distMatrix[row] = value;
    }


    // 行、列覆盖状态数组
    coveredColumns = (bool*)calloc(nOfColumns, sizeof(bool));
    coveredRows = (bool*)calloc(nOfRows, sizeof(bool));
    // 标记状态矩阵(数组形式)
    starMatrix = (bool*)calloc(nOfElements, sizeof(bool));
    primeMatrix = (bool*)calloc(nOfElements, sizeof(bool));
    newStarMatrix = (bool*)calloc(nOfElements, sizeof(bool));

    // 初始化步骤
    if (nOfRows <= nOfColumns)
    {
        minDim = nOfRows;
        // 行规约
        for (row = 0; row < nOfRows; row++)
        {
            // 寻找该行所有元素的最小值 minValue
            distMatrixTemp = distMatrix + row;
            minValue = *distMatrixTemp;
            distMatrixTemp += nOfRows;
            while (distMatrixTemp < distMatrixEnd)
            {
                value = *distMatrixTemp;
                if (value < minValue)
                    minValue = value;
                distMatrixTemp += nOfRows;
            }
            // 该行各列元素减该行最小值
            distMatrixTemp = distMatrix + row;
            while (distMatrixTemp < distMatrixEnd)
            {
                *distMatrixTemp -= minValue;
                distMatrixTemp += nOfRows;
            }
        }

        // 标记行规约后矩阵中的独立零元素 对应列设为已覆盖
        for (row = 0; row < nOfRows; row++)
        {
            for (col = 0; col < nOfColumns; col++)
            {
                if (fabs(distMatrix[row + nOfRows * col]) < DBL_EPSILON)
                {
                    if (!coveredColumns[col])
                    {
                        starMatrix[row + nOfRows * col] = true;
                        coveredColumns[col] = true;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        minDim = nOfColumns;
        // 列规约
        for (col = 0; col < nOfColumns; col++)
        {
            // 寻找该列所有元素的最小值 minValue
            distMatrixTemp = distMatrix + nOfRows * col;
            columnEnd = distMatrixTemp + nOfRows;
            minValue = *distMatrixTemp++;
            while (distMatrixTemp < columnEnd)
            {
                value = *distMatrixTemp++;
                if (value < minValue)
                    minValue = value;
            }
            // 该列各行元素减该列最小值
            distMatrixTemp = distMatrix + nOfRows * col;
            while (distMatrixTemp < columnEnd)
                *distMatrixTemp++ -= minValue;
        }

        // 标记列规约后矩阵中的独立零元素 对应行列设为已覆盖
        for (col = 0; col < nOfColumns; col++)
        {
            for (row = 0; row < nOfRows; row++)
            {
                if (fabs(distMatrix[row + nOfRows * col]) < DBL_EPSILON)
                {
                    if (!coveredRows[row])
                    {
                        starMatrix[row + nOfRows * col] = true;
                        coveredColumns[col] = true;
                        coveredRows[row] = true;
                        break;
                    }
                }
            }
        }
        for (row = 0; row < nOfRows; row++)
        {
            coveredRows[row] = false;
        }

    }
    // 试指派
    TryAssign(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

    ComputeAssignmentCost(assignment, cost, distMatrixIn, nOfRows);

    free(distMatrix);
    free(coveredColumns);
    free(coveredRows);
    free(starMatrix);
    free(primeMatrix);
    free(newStarMatrix);

    return;
}


/**
 * @brief 分配向量生成函数
 * @param assignment	输出分配结果
 * @param starMatrix	标记矩阵-列展开一维形式
 * @param nOfRows		行数
 * @param nOfColumns	列数
 * @return None
 */
void HungarianAlgorithm::BuildAssignmentVector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns)
{
    int row, col;
    for (row = 0; row < nOfRows; row++)
    {
        for (col = 0; col < nOfColumns; col++)
        {
            if (starMatrix[row + nOfRows * col])
            {
                assignment[row] = col;
                break;
            }
        }
    }
}

/**
 * @brief 分配代价计算函数
 * @param assignment	输出分配结果
 * @param cost			代价值输出
 * @param distMatrix	代价矩阵-列展开一维形式
 * @param nOfRows		行数
 * @return None
 */
void HungarianAlgorithm::ComputeAssignmentCost(int* assignment, double* cost, double* distMatrix, int nOfRows)
{
    int row, col;
    for (row = 0; row < nOfRows; row++)
    {
        col = assignment[row];
        if (col >= 0)
            *cost += distMatrix[row + nOfRows * col];
    }
}

/**
 * @brief 覆盖状态调整函数
 * @param assignment		输出分配结果
 * @param distMatrix		代价矩阵-列展开一维形式
 * @param starMatrix		标记矩阵-列展开一维形式
 * @param newStarMatrix		新标记矩阵-列展开一维形式
 * @param primeMatrix		标记矩阵-列展开一维形式
 * @param coveredColumns	列覆盖状态数组
 * @param coveredRows		行覆盖状态数组
 * @param nOfRows			行数
 * @param nOfColumns		列数
 * @param minDim			最小维数
 * @return None
 */
void HungarianAlgorithm::CoverAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool* starMatrixTemp, * columnEnd;
    int col;
    // 各列遍历 有标记元素的列设为已覆盖
    for (col = 0; col < nOfColumns; col++)
    {
        starMatrixTemp = starMatrix + nOfRows * col;
        columnEnd = starMatrixTemp + nOfRows;
        while (starMatrixTemp < columnEnd)
        {
            if (*starMatrixTemp++)
            {
                coveredColumns[col] = true;
                break;
            }
        }
    }
    // 重新试指派
    TryAssign(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/**
 * @brief 试指派函数
 * @param assignment		输出分配结果
 * @param distMatrix		代价矩阵-列展开一维形式
 * @param starMatrix		标记矩阵-列展开一维形式
 * @param newStarMatrix		新标记矩阵-列展开一维形式
 * @param primeMatrix		标记矩阵-列展开一维形式
 * @param coveredColumns	列覆盖状态数组
 * @param coveredRows		行覆盖状态数组
 * @param nOfRows			行数
 * @param nOfColumns		列数
 * @param minDim			最小维数
 * @return None
 */
void HungarianAlgorithm::TryAssign(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    int col, nOfCoveredColumns;

    nOfCoveredColumns = 0;
    for (col = 0; col < nOfColumns; col++)
        if (coveredColumns[col])
            nOfCoveredColumns++;

    if (nOfCoveredColumns == minDim)
    {
        BuildAssignmentVector(assignment, starMatrix, nOfRows, nOfColumns);
    }
    else
    {
        MatrixAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }

}

/**
 * @brief 矩阵调整函数
 * @param assignment		输出分配结果
 * @param distMatrix		代价矩阵-列展开一维形式
 * @param starMatrix		标记矩阵-列展开一维形式
 * @param newStarMatrix		新标记矩阵-列展开一维形式
 * @param primeMatrix		标记矩阵-列展开一维形式
 * @param coveredColumns	列覆盖状态数组
 * @param coveredRows		行覆盖状态数组
 * @param nOfRows			行数
 * @param nOfColumns		列数
 * @param minDim			最小维数
 * @return None
 */
void HungarianAlgorithm::MatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool zerosFound;
    int row, col, starCol;

    zerosFound = true;
    while (zerosFound)
    {
        zerosFound = false;
        for (col = 0; col < nOfColumns; col++)
        {
            if (!coveredColumns[col])
            {
                for (row = 0; row < nOfRows; row++)
                {
                    if ((!coveredRows[row]) && (fabs(distMatrix[row + nOfRows * col]) < DBL_EPSILON))
                    {
                        // 寻找未覆盖且未标记的0元素
                        primeMatrix[row + nOfRows * col] = true;

                        for (starCol = 0; starCol < nOfColumns; starCol++)
                            if (starMatrix[row + nOfRows * starCol])
                                break;

                        if (starCol == nOfColumns)
                        {
                            StarMatrixUpdate(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
                            return;
                        }
                        else
                        {
                            coveredRows[row] = true;
                            coveredColumns[starCol] = false;
                            zerosFound = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    // 代价矩阵调整
    CostMatrixAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/**
 * @brief 标记矩阵更新函数
 * @param assignment		输出分配结果
 * @param distMatrix		代价矩阵-列展开一维形式
 * @param starMatrix		标记矩阵-列展开一维形式
 * @param newStarMatrix		新标记矩阵-列展开一维形式
 * @param primeMatrix		标记矩阵-列展开一维形式
 * @param coveredColumns	列覆盖状态数组
 * @param coveredRows		行覆盖状态数组
 * @param nOfRows			行数
 * @param nOfColumns		列数
 * @param minDim			最小维数
 * @return None
 */
void HungarianAlgorithm::StarMatrixUpdate(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
    int n, starRow, starCol, primeRow, primeCol;
    int nOfElements = nOfRows * nOfColumns;

    // 更新标记矩阵数据赋值
    for (n = 0; n < nOfElements; n++)
        newStarMatrix[n] = starMatrix[n];

    newStarMatrix[row + nOfRows * col] = true;

    starCol = col;
    for (starRow = 0; starRow < nOfRows; starRow++)
        if (starMatrix[starRow + nOfRows * starCol])
            break;

    while (starRow < nOfRows)
    {
        newStarMatrix[starRow + nOfRows * starCol] = false;
        primeRow = starRow;
        for (primeCol = 0; primeCol < nOfColumns; primeCol++)
            if (primeMatrix[primeRow + nOfRows * primeCol])
                break;
        newStarMatrix[primeRow + nOfRows * primeCol] = true;
        starCol = primeCol;
        for (starRow = 0; starRow < nOfRows; starRow++)
            if (starMatrix[starRow + nOfRows * starCol])
                break;
    }

    for (n = 0; n < nOfElements; n++)
    {
        primeMatrix[n] = false;
        starMatrix[n] = newStarMatrix[n];
    }
    for (n = 0; n < nOfRows; n++)
        coveredRows[n] = false;
    // 覆盖状态调整
    CoverAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


/**
 * @brief 矩阵调整函数
 * @param assignment		输出分配结果
 * @param distMatrix		代价矩阵-列展开一维形式
 * @param starMatrix		标记矩阵-列展开一维形式
 * @param newStarMatrix		新标记矩阵-列展开一维形式
 * @param primeMatrix		标记矩阵-列展开一维形式
 * @param coveredColumns	列覆盖状态数组
 * @param coveredRows		行覆盖状态数组
 * @param nOfRows			行数
 * @param nOfColumns		列数
 * @param minDim			最小维数
 * @return None
 */
void HungarianAlgorithm::CostMatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    double h, value;
    int row, col;

    // 计算未覆盖元素的最小值
    h = DBL_MAX;
    for (row = 0; row < nOfRows; row++)
    {
        if (!coveredRows[row])
        {
            for (col = 0; col < nOfColumns; col++)
            {
                if (!coveredColumns[col])
                {
                    value = distMatrix[row + nOfRows * col];
                    if (value < h)
                        h = value;
                }
            }
        }
    }

    // 覆盖各行的所有元素加该值
    for (row = 0; row < nOfRows; row++)
        if (coveredRows[row])
            for (col = 0; col < nOfColumns; col++)
                distMatrix[row + nOfRows * col] += h;
    // 未覆盖各列的所有元素减该值
    for (col = 0; col < nOfColumns; col++)
        if (!coveredColumns[col])
            for (row = 0; row < nOfRows; row++)
                distMatrix[row + nOfRows * col] -= h;
    // 矩阵调整
    MatrixAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


