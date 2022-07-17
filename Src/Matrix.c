#include "Matrix.h"
#include <stdlib.h>
#include <stdio.h>

//创建一个矩阵对象
Matrix creat_matrix(float* e, int line, int cols)
{
    Matrix m;
    m.line = line;
    m.cols = cols;
    m.matrix = (float**)malloc(sizeof(float*) * line);
    for(int i = 0; i < line; ++i)
    {
        m.matrix[i] = (float*)malloc(sizeof(float) * cols);
    }
    for(int i = 0; i < line; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            m.matrix[i][j] = e[i * cols + j];
        }
    }
    return m;
}

void print_matrix(Matrix m)
{
    for(int i = 0; i < m.line; ++i)
    {
        printf("[ ");
        for(int j = 0; j < m.cols; ++j)
        {
            printf("%f ", m.matrix[i][j]);
        }
        printf("]\n");
    }
    printf("\n");
}

void delete_matrix(Matrix m)
{
    for(int i = 0; i < m.line; ++i)
    {
        free(m.matrix[i]);
    }
    free(m.matrix);
    m.matrix = NULL;
}

//矩阵乘法
Matrix multiply_matrix(Matrix m1, Matrix m2)
{
    float* e = (float*)malloc(sizeof(float) * (m1.line * m2.cols));
    for(int i = 0; i < m1.line; ++i)
    {
        for(int j = 0; j < m2.cols; ++j)
        {
            e[m2.cols * i + j] = 0;
            for(int k = 0; k < m1.cols; ++k)
            {
                e[m2.cols * i + j] += m1.matrix[i][k] * m2.matrix[k][j];
            }
        }
    }
    Matrix temp = creat_matrix(e, m1.line, m2.cols);
    free(e);
    return temp;
}

//求转置矩阵
Matrix matrix_T(Matrix m)
{
    Matrix temp;
    temp.line = m.cols;
    temp.cols = m.line;
    temp.matrix = (float**)malloc(sizeof(float*) * temp.line);
    for(int i = 0; i < temp.line; ++i)
        temp.matrix[i] = (float*)malloc(sizeof(float) * temp.cols);
    for(int i = 0; i < temp.line; ++i)
    {
        for(int j = 0; j < temp.cols; ++j)
        {
            temp.matrix[i][j] = m.matrix[j][i];
        }
    }
    return temp;
}

//求2*2矩阵的逆矩阵
Matrix matrix_inv2_2(Matrix m)
{
    float delta = m.matrix[0][0] * m.matrix[1][1] - m.matrix[0][1] * m.matrix[1][0];
    float e[4] = {m.matrix[1][1] / delta, -m.matrix[0][1] / delta, -m.matrix[1][0] / delta, m.matrix[0][0] / delta};
    return creat_matrix(e, 2, 2);
}

//求3*3矩阵的逆矩阵
Matrix matrix_inv3_3(Matrix m)
{
    float delta = m.matrix[0][0] * m.matrix[1][1] * m.matrix[2][2] + m.matrix[1][0] * m.matrix[2][1] * m.matrix[0][2] + m.matrix[2][0] * m.matrix[0][1] * m.matrix[1][2] - m.matrix[0][2] * m.matrix[1][1] * m.matrix[2][0] - m.matrix[0][1] * m.matrix[1][0] * m.matrix[2][2] - m.matrix[0][0] * m.matrix[1][2] * m.matrix[2][1];
    //printf("delta:%f\n", delta);
    float e[9] = {(m.matrix[1][1] * m.matrix[2][2] - m.matrix[1][2] * m.matrix[2][1]) / delta , - (m.matrix[1][0] * m.matrix[2][2] - m.matrix[1][2] * m.matrix[2][0]) / delta , (m.matrix[1][0] * m.matrix[2][1] - m.matrix[1][1] * m.matrix[2][0]) / delta ,
                     - (m.matrix[0][1] * m.matrix[2][2] - m.matrix[0][2] * m.matrix[2][1]) / delta , (m.matrix[0][0] * m.matrix[2][2] - m.matrix[0][2] * m.matrix[2][0]) / delta , - (m.matrix[0][0] * m.matrix[2][1] - m.matrix[0][1] * m.matrix[2][0]) / delta ,
                     (m.matrix[0][1] * m.matrix[1][2] - m.matrix[0][2] * m.matrix[1][1]) / delta , - (m.matrix[0][0] * m.matrix[1][2] - m.matrix[0][2] * m.matrix[1][0]) / delta , (m.matrix[0][0] * m.matrix[1][1] - m.matrix[0][1] * m.matrix[1][0]) / delta };
    Matrix temp1 = creat_matrix(e, 3, 3);
    Matrix temp2 = matrix_T(temp1);
    delete_matrix(temp1);
    return temp2;
}

//根据四边距离计算飞行器当前坐标
//参数说明：
//pamar1: 存放四个距离的数组
//pamar2,pamar3: 计算出的飞行器的当前坐标
void calculate_location(float d[], float location[])
{
    float e1[3] = {
        STATION1_X * STATION1_X - STATION4_X * STATION4_X + STATION1_Y * STATION1_Y - STATION4_Y * STATION4_Y + STATION1_Z * STATION1_Z - STATION4_Z * STATION4_Z - d[0] * d[0] + d[3] * d[3],
        STATION2_X * STATION2_X - STATION4_X * STATION4_X + STATION2_Y * STATION2_Y - STATION4_Y * STATION4_Y + STATION2_Z * STATION2_Z - STATION4_Z * STATION4_Z - d[1] * d[1] + d[3] * d[3],
        STATION3_X * STATION3_X - STATION4_X * STATION4_X + STATION3_Y * STATION3_Y - STATION4_Y * STATION4_Y + STATION3_Z * STATION3_Z - STATION4_Z * STATION4_Z - d[2] * d[2] + d[3] * d[3]
    };
    Matrix b = creat_matrix(e1, 3, 1);
    //print_matrix(b);
    float e2[9] = {
        2 * (STATION1_X - STATION4_X), 2 * (STATION1_Y - STATION4_Y), 2 * (STATION1_Z - STATION4_Z),
        2 * (STATION2_X - STATION4_X), 2 * (STATION2_Y - STATION4_Y), 2 * (STATION2_Z - STATION4_Z),
        2 * (STATION3_X - STATION4_X), 2 * (STATION3_Y - STATION4_Y), 2 * (STATION3_Z - STATION4_Z)
    };
    Matrix A = creat_matrix(e2, 3, 3);
    //print_matrix(A);
    Matrix A_inv = matrix_inv3_3(A);
    //print_matrix(A_inv);
    Matrix result = multiply_matrix(A_inv, b);
    //print_matrix(result);
    // Matrix A_T = matrix_T(A);
    // Matrix m1 = multiply_matrix(A_T, A);
    // Matrix m2 = matrix_inv(m1);
    // Matrix m3 = multiply_matrix(m2, A_T);
    // Matrix result = multiply_matrix(m3, b);
    location[0] = result.matrix[0][0];
    location[1] = result.matrix[1][0];
    location[2] = result.matrix[2][0];
    // print_matrix(b);print_matrix(A);print_matrix(A_T);print_matrix(m1);print_matrix(m2);print_matrix(m3);
    // delete_matrix(b);delete_matrix(A);delete_matrix(A_T);delete_matrix(m1);delete_matrix(m2);delete_matrix(m3);delete_matrix(result);
    printf("x:%f y:%f z:%f", location[0], location[1], location[2]);
    delete_matrix(A);delete_matrix(b);delete_matrix(A_inv);delete_matrix(result);
}

/*
根据飞行器的位置计算光流
*/
void calculate_cxof(float location[], short d_location[])
{
    static float pre_location[2] = {0, 0};
    d_location[0] = (short)(location[0] - pre_location[0]);
    d_location[1] = (short)(location[1] - pre_location[1]);

    pre_location[0] = location[0];
    pre_location[1] = location[1]; 
}
