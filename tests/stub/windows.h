#pragma once
#include <cstdint>
#include <cstddef>
typedef unsigned long DWORD; typedef unsigned char BYTE;
typedef const wchar_t* LPCWSTR; typedef const char* LPCSTR; typedef wchar_t* LPWSTR;
typedef void* HWND; typedef void* HINSTANCE; typedef void* HMODULE;
typedef int BOOL; struct RECT{long left,top,right,bottom;}; struct POINT{long x,y;};
typedef unsigned int UINT; typedef unsigned long long WPARAM; typedef long long LPARAM;
#define CP_UTF8 65001
#define EXTERN_C extern "C"
#define __declspec(x)
int MultiByteToWideChar(unsigned,unsigned long,const char*,int,wchar_t*,int);
void OutputDebugStringW(const wchar_t*);
