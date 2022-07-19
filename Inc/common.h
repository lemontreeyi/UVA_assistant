/*******************************************************************************
 *     File Name :                        common.h
 *     Library/Module Name :              MatrixComputation
 *     Author :                           Marc Pony(marc_pony@163.com)
 *     Create Date :                      2021/6/28
 *     Abstract Description :             矩阵运算库公用头文件
 *******************************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

/*******************************************************************************
 * (1)Debug Switch Section
 *******************************************************************************/

/*******************************************************************************
 * (2)Include File Section
 *******************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*******************************************************************************
 * (3)Macro Define Section
 *******************************************************************************/
#define _IN
#define _OUT
#define _IN_OUT
#define MAX(x, y) (x) > (y) ? (x) : (y)
#define MIN(x, y) (x) < (y) ? (x) : (y)

#define _CRT_SECURE_NO_WARNINGS
#define PI 3.14159265358979323846
#define POSITIVE_INFINITY 999999999
#define NEGATIVE_INFINITY -999999999
#define _ERROR_NO_ERROR 0X00000000                                     //无错误
#define _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY 0X00000001               //分配堆内存失败
#define _ERROR_SVD_EXCEED_MAX_ITERATIONS 0X00000002                    // svd超过最大迭代次数
#define _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL 0X00000003             //矩阵行数或列数不相等
#define _ERROR_MATRIX_MULTIPLICATION 0X00000004                        //矩阵乘法错误(第一个矩阵的列数不等于第二个矩阵行数)
#define _ERROR_MATRIX_MUST_BE_SQUARE 0X00000005                        //矩阵必须为方阵
#define _ERROR_MATRIX_NORM_TYPE_INVALID 0X00000006                     //矩阵模类型无效
#define _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS 0X00000007             //矩阵方程无解
#define _ERROR_MATRIX_EQUATION_HAS_INFINITY_MANNY_SOLUTIONS 0X00000008 //矩阵方程有无穷多解
#define _ERROR_QR_DECOMPOSITION_FAILED 0X00000009                      // QR分解失败
#define _ERROR_CHOLESKY_DECOMPOSITION_FAILED 0X0000000a                // cholesky分解失败
#define _ERROR_IMPROVED_CHOLESKY_DECOMPOSITION_FAILED 0X0000000b       // improved cholesky分解失败
#define _ERROR_LU_DECOMPOSITION_FAILED 0X0000000c                      // LU分解失败
#define _ERROR_CREATE_MATRIX_FAILED 0X0000000d                         //创建矩阵失败
#define _ERROR_MATRIX_TRANSPOSE_FAILED 0X0000000e                      //矩阵转置失败
#define _ERROR_CREATE_VECTOR_FAILED 0X0000000f                         //创建向量失败
#define _ERROR_VECTOR_DIMENSION_NOT_EQUAL 0X00000010                   //向量维数不相同
#define _ERROR_VECTOR_NORM_TYPE_INVALID 0X00000011                     //向量模类型无效
#define _ERROR_VECTOR_CROSS_FAILED 0X00000012                          //向量叉乘失败
#define _ERROR_INPUT_PARAMETERS_ERROR 0X00010000                       //输入参数错误

/*******************************************************************************
 * (4)Struct(Data Types) Define Section
 *******************************************************************************/
typedef unsigned int ERROR_ID;
typedef int INDEX;
typedef short FLAG;
typedef int INTEGER;
typedef double REAL;
typedef char *STRING;
typedef void VOID;

typedef struct matrix
{
    INTEGER rows;
    INTEGER columns;
    REAL *p;
} MATRIX;

typedef struct matrix_node
{
    MATRIX *ptr;
    struct matrix_node *next;
} MATRIX_NODE;

typedef struct matrix_element_node
{
    REAL *ptr;
    struct matrix_element_node *next;
} MATRIX_ELEMENT_NODE;

typedef struct stacks
{
    MATRIX_NODE *matrixNode;
    MATRIX_ELEMENT_NODE *matrixElementNode;

    // ...
    // 添加其他对象的指针
} STACKS;

/*******************************************************************************
 * (5)Prototype Declare Section
 *******************************************************************************/
/**********************************************************************************************
Function: init_stack
Description: 初始化栈
Input: 无
Output: 无
Input_Output: 栈指针
Return: 无
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
VOID init_stack(_IN_OUT STACKS *S);

/**********************************************************************************************
Function: free_stack
Description: 释放栈
Input: 栈指针
Output: 无
Input_Output: 无
Return: 无
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
VOID free_stack(_IN STACKS *S);

#endif