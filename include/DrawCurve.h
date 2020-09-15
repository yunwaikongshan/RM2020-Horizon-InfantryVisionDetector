#ifndef DRAWCURVE_H
#define DRAWCURVE_H
#include<include/Variables.h>

class DrawCurve{
public:
    void ClearSaveData();
    void InsertData(float Data);
    void InsertData(float Data1,float Data2,string s1,string s2);
};

#endif // DRAWCURVE_H
