#ifndef PIPETTE_CLIENT__VISIBILITY_CONTROL_H_
#define PIPETTE_CLIENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PIPETTE_CLIENT_EXPORT __attribute__ ((dllexport))
    #define PIPETTE_CLIENT_IMPORT __attribute__ ((dllimport))
  #else
    #define PIPETTE_CLIENT_EXPORT __declspec(dllexport)
    #define PIPETTE_CLIENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef PIPETTE_CLIENT_BUILDING_LIBRARY
    #define PIPETTE_CLIENT_PUBLIC PIPETTE_CLIENT_EXPORT
  #else
    #define PIPETTE_CLIENT_PUBLIC PIPETTE_CLIENT_IMPORT
  #endif
  #define PIPETTE_CLIENT_PUBLIC_TYPE PIPETTE_CLIENT_PUBLIC
  #define PIPETTE_CLIENT_LOCAL
#else
  #define PIPETTE_CLIENT_EXPORT __attribute__ ((visibility("default")))
  #define PIPETTE_CLIENT_IMPORT
  #if __GNUC__ >= 4
    #define PIPETTE_CLIENT_PUBLIC __attribute__ ((visibility("default")))
    #define PIPETTE_CLIENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PIPETTE_CLIENT_PUBLIC
    #define PIPETTE_CLIENT_LOCAL
  #endif
  #define PIPETTE_CLIENT_PUBLIC_TYPE
#endif

#endif  // PIPETTE_CLIENT__VISIBILITY_CONTROL_H_
