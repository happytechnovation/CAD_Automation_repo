#ifdef  _WINDOWS_SOURCE
#ifdef  __TPAAutoDrawingModule
#define ExportedByTPAAutoDrawingModule     __declspec(dllexport)
#else
#define ExportedByTPAAutoDrawingModule     __declspec(dllimport)
#endif
#else
#define ExportedByTPAAutoDrawingModule
#endif
