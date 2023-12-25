#ifndef EXPORT_H_
#define EXPORT_H_

#ifdef EXPORT
#define BAL_CTRL_API __declspec(dllexport)
#else
#define BAL_CTRL_API __declspec(dllimport)
#endif

#endif