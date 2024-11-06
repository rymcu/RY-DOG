#ifndef __DICHOTOMY_FIND_H__
#define __DICHOTOMY_FIND_H__



#ifdef __cplusplus
extern "C" {
#endif


//二分法查找的回调函数类型
//pValue:       指向要查找的值的指针
//index:        指定用于比较的项的索引号
//CallbackPara: 回调参数,可选,用于传递参数给回调函数,可以用于把数组传递给回调函数
typedef int (*COMPARE_CALLBACK)(void *pValue, int index, void *CallbackPara);


//二分法查找
//pValue:       指向要查找的值的指针
//MinIndex:     查找范围的下限
//MaxIndex:     查找范围的上限
//CompareFunc:  回调函数,用于比较查找值与指定索引的值
//CallbackPara: 回调参数,可选,用于传递参数给回调函数,可以用于把数组传递给回调函数
//返回值:       大于等于0，返回的是查找到的索引号。小于0，查找失败，未找到。
int dichotomy_find(void *pValue, int MinIndex, int MaxIndex,COMPARE_CALLBACK CompareFunc,void *CallbackPara);



#ifdef __cplusplus
}
#endif

#endif

