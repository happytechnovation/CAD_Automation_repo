#ifdef  _WINDOWS_SOURCE
#ifdef  __TPACommonAdding
#define ExportedByTPACommonAdding     __declspec(dllexport)
#else
#define ExportedByTPACommonAdding     __declspec(dllimport)
#endif
#else
#define ExportedByTPACommonAdding
#endif
