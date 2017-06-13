#include "boost/filesystem.hpp"
#include <string.h>

class FileFinder
{
public:

    /**
     * @brief find
     * @param basePath
     * @param searchWord
     * @param depth
     * @return
     */
    std::string find (std::string basePath, std::string searchWord, int depth);

};
