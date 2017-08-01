#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <utility>
#include <boost/any.hpp>

#include "core/core_errors.h"
#include "ros/ros.h"

typedef std::pair <std::string, std::vector<std::string>> ArgWithValues;
typedef std::vector < std::vector<ArgWithValues> > ParamList;

// REDO TYPEDEFS
typedef std::vector< std::pair<std::string, std::vector<boost::any>> > TaskList;
typedef std::pair <std::string, std::string> TaskAddress;
//typedef std::vector<std::string, std::vector<std::string>> TaskToArgBook;

#endif
