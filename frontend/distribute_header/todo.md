use lua+regex match to automatically generate the precompiled header file, steps:

1. get precompiled header file from the project
2. clear all '''#include <macros.h>'''
3. replace '''__declspec(dllexport)''' with '''__declspec(dllimport)'''