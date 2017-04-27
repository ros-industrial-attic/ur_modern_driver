/*
 * do_output.h
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UR_DO_OUTPUT_H_
#define UR_DO_OUTPUT_H_

#ifdef ROS_BUILD
#include <ros/ros.h>
#endif
#include <string>

void print_debug(const char* inp);
void print_info(const char* inp);
void print_warning(const char* inp);
void print_error(const char* inp);
void print_fatal(const char* inp);

inline void print_debug(const std::string& inp)   { print_debug(inp.c_str()); }
inline void print_info(const std::string& inp)    { print_info(inp.c_str()); }
inline void print_warning(const std::string& inp) { print_warning(inp.c_str()); }
inline void print_error(const std::string& inp)   { print_error(inp.c_str()); }
inline void print_fatal(const std::string& inp)   { print_fatal(inp.c_str()); }


#endif /* UR_DO_OUTPUT_H_ */
