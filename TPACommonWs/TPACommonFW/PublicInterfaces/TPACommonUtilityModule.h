#ifdef  _WINDOWS_SOURCE
#ifdef  __TPACommonUtilityModule
#define ExportedByTPACommonUtilityModule     __declspec(dllexport)
#else
#define ExportedByTPACommonUtilityModule     __declspec(dllimport)
#endif
#else
#define ExportedByTPACommonUtilityModule
#endif
