#include "pure_pursite.hpp"

namespace get_pure_pursite
{
   PP::PP()
    {
    }
double PP::calc(double alpha,double ld,double axs) 
{
    return atan((2*sin(alpha)*axs) / ld);
}
}
