#include "Matrix.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

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

//初始化A矩阵
/*
A.shape  : 3x2, b.shape = 3x1
[x,y].T  : (A_T * A)^(-1) * A_T * b，利用伪逆算子求解矩阵方程
A_resinv : (A_T * A)^(-1)
A_res    : (A_T * A)^(-1) * A_T
*/
void init_A_matrix(void)
{
    Matrix A, A_resinv, A_T, A_temp;
    float e2[9] = {
        2 * (STATION1_X - STATION4_X), 2 * (STATION1_Y - STATION4_Y),
        2 * (STATION2_X - STATION4_X), 2 * (STATION2_Y - STATION4_Y),
        2 * (STATION3_X - STATION4_X), 2 * (STATION3_Y - STATION4_Y)
    };
    A = creat_matrix(e2, 3, 2);
    A_T = matrix_T(A);
    A_temp = multiply_matrix(A_T, A);
    A_resinv = matrix_inv2_2(A_temp);
    A_res = multiply_matrix(A_resinv, A_T);
    delete_matrix(A);delete_matrix(A_resinv);delete_matrix(A_T);delete_matrix(A_temp);
}

//根据四边距离计算飞行器当前坐标
//参数说明：
//pamar1: 存放四个距离的数组
//pamar2,pamar3: 计算出的飞行器的当前坐标
void calculate_location(float d[], float location[], float height)
{
    // 先将三维距离转化为xy平面投影上的距离
    // d_xy = sqrt(d**2 - dh**2)
    float d1 = (float)sqrt(d[0]*d[0] - (STATION1_Z - height - 0.15)*(STATION1_Z - height - 0.15));
    float d2 = (float)sqrt(d[1]*d[1] - (STATION2_Z - height - 0.15)*(STATION2_Z - height - 0.15));
    float d3 = (float)sqrt(d[2]*d[2] - (STATION3_Z - height - 0.15)*(STATION3_Z - height - 0.15));
    float d4 = (float)sqrt(d[3]*d[3] - (STATION4_Z - height - 0.15)*(STATION4_Z - height - 0.15));
    float e1[3] = {
        STATION1_X * STATION1_X - STATION4_X * STATION4_X + STATION1_Y * STATION1_Y - STATION4_Y * STATION4_Y - d1 * d1 + d4 * d4,
        STATION2_X * STATION2_X - STATION4_X * STATION4_X + STATION2_Y * STATION2_Y - STATION4_Y * STATION4_Y - d2 * d2 + d4 * d4,
        STATION3_X * STATION3_X - STATION4_X * STATION4_X + STATION3_Y * STATION3_Y - STATION4_Y * STATION4_Y - d3 * d3 + d4 * d4
    };
    Matrix b = creat_matrix(e1, 3, 1);
    //print_matrix(b);
    
    //print_matrix(A);
    //print_matrix(A_inv);
    Matrix result = multiply_matrix(A_res, b);
    //print_matrix(result);
    // Matrix A_T = matrix_T(A);
    // Matrix m1 = multiply_matrix(A_T, A);
    // Matrix m2 = matrix_inv(m1);
    // Matrix m3 = multiply_matrix(m2, A_T);
    // Matrix result = multiply_matrix(m3, b);
    location[0] = result.matrix[0][0];
    location[1] = result.matrix[1][0];
    location[2] = height;
    // print_matrix(b);print_matrix(A);print_matrix(A_T);print_matrix(m1);print_matrix(m2);print_matrix(m3);
    // delete_matrix(b);delete_matrix(A);delete_matrix(A_T);delete_matrix(m1);delete_matrix(m2);delete_matrix(m3);delete_matrix(result);
    // printf("x:%f y:%f z:%f\r\n", location[0], location[1], location[2]);
    delete_matrix(b);delete_matrix(result);
    //重置测距数组
    for(int i = 0; i < 4; ++i)
        d[i] = 0;
}

/*
根据飞行器的位置计算光流
*/
void calculate_cxof(float location[], short d_location[], float *speed, uint32_t Dtime)
{
    static float pre_location[2] = {2.45, 2.05};
    //将dx，dy转化为mm为单位，方便求速度
    d_location[0] = (short)((location[0] - pre_location[0]) * 1000);
    d_location[1] = (short)((location[1] - pre_location[1]) * 1000);

    //求得速度单位为m/s
    speed[0] = ((location[0] - pre_location[0]) * 1000) / Dtime;
    speed[1] = ((location[1] - pre_location[1]) * 1000) / Dtime;
    //printf("vx:%f, vy:%f\r\n", speed[0], speed[1]);

//    pre_location[0] = location[0];
//    pre_location[1] = location[1];
}

void mid_filter(float raw_data, float* location_esm, float* array)
{
    float temp[7];
    for(int i = 0; i < 6; ++i)
    {
        array[i] = array[i + 1];
        temp[i] = array[i];
    }
    array[6] = raw_data;
    temp[6] = array[6];
    BubbleSort(temp, 7);
    *location_esm = temp[3];
}

void limit_filter(float data, float* location_esm)
{
    if(fabs(data - *location_esm) > 40)
        *location_esm += (data - *location_esm) * 0.1;
    else
        *location_esm = data;
}

void BubbleSort(float* array, int len)
{
    float temp = 0;
    bool flag = 0;
    for(int i = 0; i < len - 1; ++i)
    {
        if(array[i] > array[i + 1])
        {
            temp = array[i + 1];
            array[i + 1] = array[i];
            array[i] = temp;
            flag = 1;
        }
        if(flag) break;
    }
}