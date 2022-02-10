#ifndef DATASPEED_FORD_INTERFACE__VISIBILITY_CONTROL_HPP_
#define DATASPEED_FORD_INTERFACE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(DATASPEED_FORD_INTERFACE_BUILDING_DLL) || defined(DATASPEED_FORD_INTERFACE_EXPORTS)
    #define DATASPEED_FORD_INTERFACE_PUBLIC __declspec(dllexport)
    #define DATASPEED_FORD_INTERFACE_LOCAL
  #else  // defined(DATASPEED_FORD_INTERFACE_BUILDING_DLL) || defined(DATASPEED_FORD_INTERFACE_EXPORTS)
    #define DATASPEED_FORD_INTERFACE_PUBLIC __declspec(dllimport)
    #define DATASPEED_FORD_INTERFACE_LOCAL
  #endif  // defined(DATASPEED_FORD_INTERFACE_BUILDING_DLL) || defined(DATASPEED_FORD_INTERFACE_EXPORTS)
#elif defined(__linux__)
  #define DATASPEED_FORD_INTERFACE_PUBLIC __attribute__((visibility("default")))
  #define DATASPEED_FORD_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define DATASPEED_FORD_INTERFACE_PUBLIC __attribute__((visibility("default")))
  #define DATASPEED_FORD_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // DATASPEED_FORD_INTERFACE__VISIBILITY_CONTROL_HPP_
