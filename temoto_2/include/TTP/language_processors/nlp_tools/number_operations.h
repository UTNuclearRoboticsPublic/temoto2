#ifndef NUMBER_OPERATIONS_H
#define NUMBER_OPERATIONS_H

// credits go to https://codereview.stackexchange.com/a/30675

#include <vector>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace TTP
{

typedef std::unordered_map<std::string, int> nummap;

const std::vector<std::string> first14 = {"zero", "one", "two", "three", "four", "five", "six", "seven",
                                          "eight", "nine", "ten", "eleven", "twelve", "thirteen",
                                          "fourteen" };
const std::vector<std::string> prefixes = {"twen", "thir", "for", "fif", "six", "seven", "eigh", "nine"};

/**
 * @brief inttostr
 * @param number
 * @return
 */
std::string inttostr( const unsigned int number );

/**
 * @brief Function that generates an str->int map
 * @param max_nr
 * @return
 */
nummap generateStrToNrMap(int max_nr);

}

#endif
