#include <stdio.h>
#include <string.h>
typedef struct stuInfo
{
    char name[21];
    int sum;
} StuInfo;
int compare(StuInfo stu1, StuInfo stu2);
void order(StuInfo *index, int n);
void output(const StuInfo *index, int n);
int main()
{
    int i, n;
    StuInfo index[100];
    scanf("%d", &n);
    for (i = 0; i < n; i++)
    {
        scanf("%s %d", (index + i)->name, &(index + i)->sum);
    }
    order(index, n);
    output(index, n);
    return 0;
}
void output(const StuInfo *index, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        printf("Name:%s\n", (index + i)->name);
        printf("total:%d\n\n", (index + i)->sum);
    }
}
int compare(StuInfo stu1, StuInfo stu2)
{
    // 输出为1，表示stu1在排序时放在stu2前面
    if (stu1.sum > stu2.sum)
        return 1;
    else if (stu1.sum < stu2.sum)
        return -1;
    else
    {
        if (strcmp(stu1.name, stu2.name) < 0)
            return 1;
        else if (strcmp(stu1.name, stu2.name) > 0)
            return -1;
        else
            return 0;
    }
}
void order(StuInfo *index, int n)
{
    StuInfo temp; // 同时用于储存最前值
    // 选择排序
    int i, j, k;
    for (i = 0; i < n; i++)
    {
        temp = index[i];
        for (j = i, k = i; j < n; j++)
        {
            if (compare(index[j], temp) == 1)
            {
                // 结构本身可以作为参数传递
                temp = index[j];
                k = j;
            }
        }
        if (k != i)
        {
            index[k] = index[i];
            index[i] = temp;
        }
    }
}