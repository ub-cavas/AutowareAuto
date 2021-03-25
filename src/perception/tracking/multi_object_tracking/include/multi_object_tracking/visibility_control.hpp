// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MULTI_OBJECT_TRACKING__VISIBILITY_CONTROL_HPP_
#define MULTI_OBJECT_TRACKING__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(MULTI_OBJECT_TRACKING_BUILDING_DLL) || defined(MULTI_OBJECT_TRACKING_EXPORTS)
    #define MULTI_OBJECT_TRACKING_PUBLIC __declspec(dllexport)
    #define MULTI_OBJECT_TRACKING_LOCAL
  #else  // defined(MULTI_OBJECT_TRACKING_BUILDING_DLL) || defined(MULTI_OBJECT_TRACKING_EXPORTS)
    #define MULTI_OBJECT_TRACKING_PUBLIC __declspec(dllimport)
    #define MULTI_OBJECT_TRACKING_LOCAL
  #endif  // defined(MULTI_OBJECT_TRACKING_BUILDING_DLL) || defined(MULTI_OBJECT_TRACKING_EXPORTS)
#elif defined(__linux__)
  #define MULTI_OBJECT_TRACKING_PUBLIC __attribute__((visibility("default")))
  #define MULTI_OBJECT_TRACKING_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MULTI_OBJECT_TRACKING_PUBLIC __attribute__((visibility("default")))
  #define MULTI_OBJECT_TRACKING_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // MULTI_OBJECT_TRACKING__VISIBILITY_CONTROL_HPP_
