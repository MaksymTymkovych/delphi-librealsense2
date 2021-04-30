{$DEFINE LIBREALSENSE_RS2_H}

{$Z4} // Set minimum enum size equal to 4 bytes

{$IFDEF MSWINDOWS}
  {$DEFINE WINDOWS}
{$ENDIF}
{$IFDEF WINDOWS}
  {$IF DEFINED(WIN32) or DEFINED(WIN64)}
    {$DEFINE WINDESKTOP}
  {$ELSE}
    {$DEFINE WINMOBILE}

  {$IFEND}
  //{$DEFINE STDCALL}
  {$DEFINE CDECL}
{$ENDIF}
{$IFDEF LINUX}
  {$DEFINE CDECL}
{$ENDIF}
{$IFDEF DARWIN}
  {$IF DEFINED(iPHONESIM) or (DEFINED(DARWIN) and DEFINED(CPUARM))}
    {$DEFINE iOS}
  {$ELSE}
    {$DEFINE MACOSX}
  {$IFEND}
  {$DEFINE CDECL}
{$ENDIF}


{$IFNDEF FPC}
  {$IFDEF VER110}//Builder 3
    {$DEFINE DEFINE_8087CW_NOT_IMPLEMENTED}
    {$DEFINE DEFINE_UINT64_EQU_INT64}
    {$DEFINE DEFINE_REGION_NOT_IMPLEMENTED}
  {$ENDIF}
  {$IFDEF VER100}//Delphi3
    {$DEFINE DEFINE_8087CW_NOT_IMPLEMENTED}
    {$DEFINE DEFINE_UINT64_EQU_INT64}
    {$DEFINE DEFINE_REGION_NOT_IMPLEMENTED}
  {$ENDIF}
  {$IFDEF VER120}//Delphi 4
    {$DEFINE DEFINE_8087CW_NOT_IMPLEMENTED}
    {$DEFINE DEFINE_UINT64_EQU_INT64}
    {$DEFINE DEFINE_REGION_NOT_IMPLEMENTED}
  {$ENDIF}
  {$IFDEF VER130}//Delphi 5
    {$DEFINE DEFINE_UINT64_EQU_INT64}
    {$DEFINE DEFINE_UINT64_EQU_INT64}
  {$ENDIF}
  {$IFDEF VER140}//Delphi 6
    {$DEFINE DEFINE_UINT64_EQU_INT64}
    {$DEFINE DEFINE_REGION_NOT_IMPLEMENTED}
  {$ENDIF}
  {$IFDEF VER150}//Delphi 7
    {$DEFINE DEFINE_REGION_NOT_IMPLEMENTED}
  {$ENDIF}
{$ENDIF}