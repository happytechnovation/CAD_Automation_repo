#ifdef  _WINDOWS_SOURCE
#ifdef  __TPADVSDialogModule
#define ExportedByTPADVSDialogModule     __declspec(dllexport)
#else
#define ExportedByTPADVSDialogModule     __declspec(dllimport)
#endif
#else
#define ExportedByTPADVSDialogModule
#endif
