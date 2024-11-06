#include "dichotomy_find.h"

int dichotomy_find(void * pValue, int MinIndex, int MaxIndex,COMPARE_CALLBACK CompareFunc,void *CallbackPara)
{
    int min = MinIndex;
    int max = MaxIndex;
    
    if (min > max)
        return -1;
    
    while(min <= max)
    {
        int mid = (max + min)/2;
        int rst = CompareFunc(pValue, mid, CallbackPara);

        if (rst < 0)
            max = mid - 1;
        else if (rst > 0)
            min = mid + 1;
        else
           return mid;
    }
    return -1;
}


