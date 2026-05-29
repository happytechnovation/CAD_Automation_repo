#ifdef  _WINDOWS_SOURCE
#ifdef  __TPADVSAuto
#define ExportedByTPADVSAuto     __declspec(dllexport)
#else
#define ExportedByTPADVSAuto     __declspec(dllimport)
#endif
#else
#define ExportedByTPADVSAuto
#endif
