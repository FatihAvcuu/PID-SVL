#include "stanley.hpp"


namespace get_stanley
{
   SC::SC()
    {

    }


double SC::calc(double k,double vel,double e_error) 
{
    return atan(k*e_error/vel);
}


}
