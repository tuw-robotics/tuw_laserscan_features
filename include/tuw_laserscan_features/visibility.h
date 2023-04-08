#ifndef TUW_LASERSCAN_FEATURES__VISIBILITY_H_
#define TUW_LASERSCAN_FEATURES__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define TUW_LASERSCAN_FEATURES_PACKAGE_EXPORT __attribute__ ((dllexport))
    #define TUW_LASERSCAN_FEATURES_PACKAGE_IMPORT __attribute__ ((dllimport))
  #else
    #define TUW_LASERSCAN_FEATURES_PACKAGE_EXPORT __declspec(dllexport)
    #define TUW_LASERSCAN_FEATURES_PACKAGE_IMPORT __declspec(dllimport)
  #endif

  #ifdef TUW_LASERSCAN_FEATURES_PACKAGE_DLL
    #define TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC TUW_LASERSCAN_FEATURES_PACKAGE_EXPORT
  #else
    #define TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC TUW_LASERSCAN_FEATURES_PACKAGE_IMPORT
  #endif

  #define TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC_TYPE TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC

  #define TUW_LASERSCAN_FEATURES_PACKAGE_LOCAL

#else

  #define TUW_LASERSCAN_FEATURES_PACKAGE_EXPORT __attribute__ ((visibility("default")))
  #define TUW_LASERSCAN_FEATURES_PACKAGE_IMPORT

  #if __GNUC__ >= 4
    #define TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC __attribute__ ((visibility("default")))
    #define TUW_LASERSCAN_FEATURES_PACKAGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC
    #define TUW_LASERSCAN_FEATURES_PACKAGE_LOCAL
  #endif

  #define TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TUW_LASERSCAN_FEATURES__VISIBILITY_H_
