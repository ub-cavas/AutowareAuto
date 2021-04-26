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

#ifndef SYNCHRONIZED_SUBSCRIPTION__VISIBILITY_CONTROL_HPP_
#define SYNCHRONIZED_SUBSCRIPTION__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SYNCHRONIZED_SUBSCRIPTION_BUILDING_DLL) || defined(SYNCHRONIZED_SUBSCRIPTION_EXPORTS)
    #define SYNCHRONIZED_SUBSCRIPTION_PUBLIC __declspec(dllexport)
    #define SYNCHRONIZED_SUBSCRIPTION_LOCAL
  #else
// defined(SYNCHRONIZED_SUBSCRIPTION_BUILDING_DLL) || defined(SYNCHRONIZED_SUBSCRIPTION_EXPORTS)
    #define SYNCHRONIZED_SUBSCRIPTION_PUBLIC __declspec(dllimport)
    #define SYNCHRONIZED_SUBSCRIPTION_LOCAL
  #endif
// defined(SYNCHRONIZED_SUBSCRIPTION_BUILDING_DLL) || defined(SYNCHRONIZED_SUBSCRIPTION_EXPORTS)
#elif defined(__linux__)
  #define SYNCHRONIZED_SUBSCRIPTION_PUBLIC __attribute__((visibility("default")))
  #define SYNCHRONIZED_SUBSCRIPTION_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SYNCHRONIZED_SUBSCRIPTION_PUBLIC __attribute__((visibility("default")))
  #define SYNCHRONIZED_SUBSCRIPTION_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // SYNCHRONIZED_SUBSCRIPTION__VISIBILITY_CONTROL_HPP_
