
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LED_CONTROLLER__VISIBILITY_H_
#define LED_CONTROLLER__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define LED_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define LED_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define LED_CONTROLLER_EXPORT __declspec(dllexport)
    #define LED_CONTROLLER_IMPORT __declspec(dllimport)
  #endif

  #ifdef LED_CONTROLLER_DLL
    #define LED_CONTROLLER_PUBLIC LED_CONTROLLER_EXPORT
  #else
    #define LED_CONTROLLER_PUBLIC LED_CONTROLLER_IMPORT
  #endif

  #define LED_CONTROLLER_PUBLIC_TYPE LED_CONTROLLER_PUBLIC

  #define LED_CONTROLLER_LOCAL

#else

  #define LED_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define LED_CONTROLLER_IMPORT

  #if __GNUC__ >= 4
    #define LED_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define LED_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LED_CONTROLLER_PUBLIC
    #define LED_CONTROLLER_LOCAL
  #endif

  #define LED_CONTROLLER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LED_CONTROLLER__VISIBILITY_H_