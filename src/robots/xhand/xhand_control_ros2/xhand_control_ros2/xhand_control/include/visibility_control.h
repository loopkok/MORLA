#ifndef XHAND_CONTROL__VISIBILITY_CONTROL_H_
#define XHAND_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define XHAND_CONTROL_EXPORT __attribute__((dllexport))
#define XHAND_CONTROL_IMPORT __attribute__((dllimport))
#else
#define XHAND_CONTROL_EXPORT __declspec(dllexport)
#define XHAND_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef XHAND_CONTROL_BUILDING_LIBRARY
#define XHAND_CONTROL_PUBLIC XHAND_CONTROL_EXPORT
#else
#define XHAND_CONTROL_PUBLIC XHAND_CONTROL_IMPORT
#endif
#define XHAND_CONTROL_PUBLIC_TYPE XHAND_CONTROL_PUBLIC
#define XHAND_CONTROL_LOCAL
#else
#define XHAND_CONTROL_EXPORT __attribute__((visibility("default")))
#define XHAND_CONTROL_IMPORT
#if __GNUC__ >= 4
#define XHAND_CONTROL_PUBLIC __attribute__((visibility("default")))
#define XHAND_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define XHAND_CONTROL_PUBLIC
#define XHAND_CONTROL_LOCAL
#endif
#define XHAND_CONTROL_PUBLIC_TYPE
#endif

#endif  // XHAND_CONTROL__VISIBILITY_CONTROL_H_