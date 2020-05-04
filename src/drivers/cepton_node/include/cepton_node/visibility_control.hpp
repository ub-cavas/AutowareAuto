// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#ifndef CEPTON_NODE__VISIBILITY_CONTROL_HPP_
#define CEPTON_NODE__VISIBILITY_CONTROL_HPP_

#include <apexutils/apexdef.h>

////////////////////////////////////////////////////////////////////////////////
#if defined(APEX_WINDOWS)
  #if defined(CEPTON_NODE_BUILDING_DLL) || defined(CEPTON_NODE_EXPORTS)
    #define CEPTON_NODE_PUBLIC __declspec(dllexport)
    #define CEPTON_NODE_LOCAL
  #else  // defined(CEPTON_NODE_BUILDING_DLL) || defined(CEPTON_NODE_EXPORTS)
    #define CEPTON_NODE_PUBLIC __declspec(dllimport)
    #define CEPTON_NODE_LOCAL
  #endif  // defined(CEPTON_NODE_BUILDING_DLL) || defined(CEPTON_NODE_EXPORTS)
#elif defined(APEX_LINUX)
  #define CEPTON_NODE_PUBLIC __attribute__((visibility("default")))
  #define CEPTON_NODE_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_OSX)
  #define CEPTON_NODE_PUBLIC __attribute__((visibility("default")))
  #define CEPTON_NODE_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_QNX)
  #define CEPTON_NODE_PUBLIC __attribute__((visibility("default")))
  #define CEPTON_NODE_LOCAL __attribute__((visibility("hidden")))
#else  // defined(APEX_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(APEX_WINDOWS)

#endif  // CEPTON_NODE__VISIBILITY_CONTROL_HPP_
