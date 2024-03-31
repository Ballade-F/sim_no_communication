#include <stdlib.h>
#include <cfloat>
#include <cmath>
#include "MP_Hungarian.h"


HungarianAlgorithm::HungarianAlgorithm() {}
HungarianAlgorithm::~HungarianAlgorithm() {}


/**
 * @brief �������㷨��⺯��
 * @param DistMatrix	���۾���
 * @param Assignment	���������
 * @return cost			�������ֵ
 */
double HungarianAlgorithm::Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment)
{
    unsigned int nRows = DistMatrix.size();
    unsigned int nCols = DistMatrix[0].size();

    // ���۾���ת��Ϊ��չ��һά����
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

    // ���÷���������⺯��
    assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

    Assignment.clear();
    for (unsigned int r = 0; r < nRows; r++)
        Assignment.push_back(assignment[r]);

    delete[] distMatrixIn;
    delete[] assignment;
    return cost;
}

/**
 * @brief ��������������⺯��
 * @param assignment	���������
 * @param cost			����ֵ
 * @param distMatrixIn	���۾���-��չ��һά��ʽ
 * @param nOfRows		����
 * @param nOfColumns	����
 * @return None
 */
void HungarianAlgorithm::assignmentoptimal(int* assignment, double* cost, double* distMatrixIn, int nOfRows, int nOfColumns)
{
    double* distMatrix, * distMatrixTemp, * distMatrixEnd, * columnEnd, value, minValue;
    bool* coveredColumns, * coveredRows, * starMatrix, * newStarMatrix, * primeMatrix;
    int nOfElements, minDim, row, col;

    // �����ʼ��
    *cost = 0;
    for (row = 0; row < nOfRows; row++)
        assignment[row] = -1;

    // ���ɴ��۾�����������飬�ж�����Ԫ���ǷǸ���
    nOfElements = nOfRows * nOfColumns;
    distMatrix = (double*)malloc(nOfElements * sizeof(double));
    distMatrixEnd = distMatrix + nOfElements;

    for (row = 0; row < nOfElements; row++)
    {
        value = distMatrixIn[row];
        distMatrix[row] = value;
    }


    // �С��и���״̬����
    coveredColumns = (bool*)calloc(nOfColumns, sizeof(bool));
    coveredRows = (bool*)calloc(nOfRows, sizeof(bool));
    // ���״̬����(������ʽ)
    starMatrix = (bool*)calloc(nOfElements, sizeof(bool));
    primeMatrix = (bool*)calloc(nOfElements, sizeof(bool));
    newStarMatrix = (bool*)calloc(nOfElements, sizeof(bool));

    // ��ʼ������
    if (nOfRows <= nOfColumns)
    {
        minDim = nOfRows;
        // �й�Լ
        for (row = 0; row < nOfRows; row++)
        {
            // Ѱ�Ҹ�������Ԫ�ص���Сֵ minValue
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
            // ���и���Ԫ�ؼ�������Сֵ
            distMatrixTemp = distMatrix + row;
            while (distMatrixTemp < distMatrixEnd)
            {
                *distMatrixTemp -= minValue;
                distMatrixTemp += nOfRows;
            }
        }

        // ����й�Լ������еĶ�����Ԫ�� ��Ӧ����Ϊ�Ѹ���
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
        // �й�Լ
        for (col = 0; col < nOfColumns; col++)
        {
            // Ѱ�Ҹ�������Ԫ�ص���Сֵ minValue
            distMatrixTemp = distMatrix + nOfRows * col;
            columnEnd = distMatrixTemp + nOfRows;
            minValue = *distMatrixTemp++;
            while (distMatrixTemp < columnEnd)
            {
                value = *distMatrixTemp++;
                if (value < minValue)
                    minValue = value;
            }
            // ���и���Ԫ�ؼ�������Сֵ
            distMatrixTemp = distMatrix + nOfRows * col;
            while (distMatrixTemp < columnEnd)
                *distMatrixTemp++ -= minValue;
        }

        // ����й�Լ������еĶ�����Ԫ�� ��Ӧ������Ϊ�Ѹ���
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
    // ��ָ��
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
 * @brief �����������ɺ���
 * @param assignment	���������
 * @param starMatrix	��Ǿ���-��չ��һά��ʽ
 * @param nOfRows		����
 * @param nOfColumns	����
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
 * @brief ������ۼ��㺯��
 * @param assignment	���������
 * @param cost			����ֵ���
 * @param distMatrix	���۾���-��չ��һά��ʽ
 * @param nOfRows		����
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
 * @brief ����״̬��������
 * @param assignment		���������
 * @param distMatrix		���۾���-��չ��һά��ʽ
 * @param starMatrix		��Ǿ���-��չ��һά��ʽ
 * @param newStarMatrix		�±�Ǿ���-��չ��һά��ʽ
 * @param primeMatrix		��Ǿ���-��չ��һά��ʽ
 * @param coveredColumns	�и���״̬����
 * @param coveredRows		�и���״̬����
 * @param nOfRows			����
 * @param nOfColumns		����
 * @param minDim			��Сά��
 * @return None
 */
void HungarianAlgorithm::CoverAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    bool* starMatrixTemp, * columnEnd;
    int col;
    // ���б��� �б��Ԫ�ص�����Ϊ�Ѹ���
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
    // ������ָ��
    TryAssign(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/**
 * @brief ��ָ�ɺ���
 * @param assignment		���������
 * @param distMatrix		���۾���-��չ��һά��ʽ
 * @param starMatrix		��Ǿ���-��չ��һά��ʽ
 * @param newStarMatrix		�±�Ǿ���-��չ��һά��ʽ
 * @param primeMatrix		��Ǿ���-��չ��һά��ʽ
 * @param coveredColumns	�и���״̬����
 * @param coveredRows		�и���״̬����
 * @param nOfRows			����
 * @param nOfColumns		����
 * @param minDim			��Сά��
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
 * @brief �����������
 * @param assignment		���������
 * @param distMatrix		���۾���-��չ��һά��ʽ
 * @param starMatrix		��Ǿ���-��չ��һά��ʽ
 * @param newStarMatrix		�±�Ǿ���-��չ��һά��ʽ
 * @param primeMatrix		��Ǿ���-��չ��һά��ʽ
 * @param coveredColumns	�и���״̬����
 * @param coveredRows		�и���״̬����
 * @param nOfRows			����
 * @param nOfColumns		����
 * @param minDim			��Сά��
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
                        // Ѱ��δ������δ��ǵ�0Ԫ��
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
    // ���۾������
    CostMatrixAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/**
 * @brief ��Ǿ�����º���
 * @param assignment		���������
 * @param distMatrix		���۾���-��չ��һά��ʽ
 * @param starMatrix		��Ǿ���-��չ��һά��ʽ
 * @param newStarMatrix		�±�Ǿ���-��չ��һά��ʽ
 * @param primeMatrix		��Ǿ���-��չ��һά��ʽ
 * @param coveredColumns	�и���״̬����
 * @param coveredRows		�и���״̬����
 * @param nOfRows			����
 * @param nOfColumns		����
 * @param minDim			��Сά��
 * @return None
 */
void HungarianAlgorithm::StarMatrixUpdate(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
    int n, starRow, starCol, primeRow, primeCol;
    int nOfElements = nOfRows * nOfColumns;

    // ���±�Ǿ������ݸ�ֵ
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
    // ����״̬����
    CoverAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


/**
 * @brief �����������
 * @param assignment		���������
 * @param distMatrix		���۾���-��չ��һά��ʽ
 * @param starMatrix		��Ǿ���-��չ��һά��ʽ
 * @param newStarMatrix		�±�Ǿ���-��չ��һά��ʽ
 * @param primeMatrix		��Ǿ���-��չ��һά��ʽ
 * @param coveredColumns	�и���״̬����
 * @param coveredRows		�и���״̬����
 * @param nOfRows			����
 * @param nOfColumns		����
 * @param minDim			��Сά��
 * @return None
 */
void HungarianAlgorithm::CostMatrixAdjust(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    double h, value;
    int row, col;

    // ����δ����Ԫ�ص���Сֵ
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

    // ���Ǹ��е�����Ԫ�ؼӸ�ֵ
    for (row = 0; row < nOfRows; row++)
        if (coveredRows[row])
            for (col = 0; col < nOfColumns; col++)
                distMatrix[row + nOfRows * col] += h;
    // δ���Ǹ��е�����Ԫ�ؼ���ֵ
    for (col = 0; col < nOfColumns; col++)
        if (!coveredColumns[col])
            for (row = 0; row < nOfRows; row++)
                distMatrix[row + nOfRows * col] -= h;
    // �������
    MatrixAdjust(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


