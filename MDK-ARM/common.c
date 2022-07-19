/*******************************************************************************
 *     File Name :                        common.c
 *     Library/Module Name :              MatrixComputation
 *     Author :                           Marc Pony(marc_pony@163.com)
 *     Create Date :                      2021/7/16
 *     Abstract Description :            矩阵运算库公用源文件
 *******************************************************************************/

/*******************************************************************************
 * (1)Debug Switch Section
 *******************************************************************************/

/*******************************************************************************
 * (2)Include File Section
 *******************************************************************************/
#include "common.h"

/*******************************************************************************
 * (3)Macro Define Section
 *******************************************************************************/

/*******************************************************************************
 * (4)Struct(Data Types) Define Section
 *******************************************************************************/

/*******************************************************************************
 * (5)Prototype Declare Section
 *******************************************************************************/

/*******************************************************************************
 * (6)Global Variable Declare Section
 *******************************************************************************/

/*******************************************************************************
 * (7)File Static Variable Define Section
 *******************************************************************************/

/*******************************************************************************
 * (8)Function Define Section
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
VOID init_stack(_IN_OUT STACKS *S)
{
    if (S == NULL)
    {
        return;
    }

    memset(S, 0, sizeof(STACKS));
}

/**********************************************************************************************
Function: free_stack
Description: 释放栈
Input: 栈指针
Output: 无
Input_Output: 无
Return: 无
Author: Marc Pony(marc_pony@163.com)
***********************************************************************************************/
VOID free_stack(_IN STACKS *S)
{
    MATRIX_NODE *matrixNode = NULL;
    MATRIX_ELEMENT_NODE *matrixElementNode = NULL;

    if (S == NULL)
    {
        return;
    }

    while (S->matrixNode != NULL)
    {
        matrixNode = S->matrixNode;
        S->matrixNode = matrixNode->next;

        free(matrixNode->ptr);
        matrixNode->ptr = NULL;
        free(matrixNode);
        matrixNode = NULL;
    }

    while (S->matrixElementNode != NULL)
    {
        matrixElementNode = S->matrixElementNode;
        S->matrixElementNode = matrixElementNode->next;

        free(matrixElementNode->ptr);
        matrixElementNode->ptr = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;
    }

    // ...
    // 释放其他指针
}