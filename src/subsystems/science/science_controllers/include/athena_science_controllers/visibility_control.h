// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_CONTROL_H_
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_EXPORT __attribute__((dllexport))
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_EXPORT __declspec(dllexport)
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_BUILDING_DLL
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_EXPORT
#else
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_IMPORT
#endif
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC_TYPE ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_LOCAL
#else
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_LOCAL
#endif
#define ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // ATHENA_SCIENCE_CONTROLLERS__VISIBILITY_CONTROL_H_
