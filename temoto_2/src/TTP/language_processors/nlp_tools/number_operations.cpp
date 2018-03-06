#include "TTP/language_processors/nlp_tools/number_operations.h"

namespace TTP
{

std::string inttostr( const unsigned int number )
{
  if( number <= 14 )
  {
    return first14.at(number);
  }
  if( number < 20 )
  {
    return prefixes.at(number-12) + "teen";
  }
  if( number < 100 )
  {
    unsigned int remainder = number - (static_cast<int>(number/10)*10);
    return prefixes.at(number/10-2) + (0 != remainder ? "ty " + inttostr(remainder) : "ty");
  }
  if( number < 1000 )
  {
    unsigned int remainder = number - (static_cast<int>(number/100)*100);
    return first14.at(number/100) + (0 != remainder ? " hundred " + inttostr(remainder) : " hundred");
  }
  if( number < 1000000 )
  {
    unsigned int thousands = static_cast<int>(number/1000);
    unsigned int remainder = number - (thousands*1000);
    return inttostr(thousands) + (0 != remainder ? " thousand " + inttostr(remainder) : " thousand");
  }
  if( number < 1000000000 )
  {
    unsigned int millions = static_cast<int>(number/1000000);
    unsigned int remainder = number - (millions*1000000);
    return inttostr(millions) + (0 != remainder ? " million " + inttostr(remainder) : " million");
  }

  throw std::out_of_range("inttostr() value too large");
}

nummap generateStrToNrMap(int max_nr)
{
  nummap str_int_map;

  // Fill the map
  for (int i=0; i<=max_nr; i++)
  {
    str_int_map[inttostr(i)] = i;
  }

  return str_int_map;
}

}
