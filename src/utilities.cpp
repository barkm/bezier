#include <stdexcept>
#include "bezier/utilities.h"

namespace bezier {

    int factorial(int n){
        if (n < 0){
            throw std::domain_error("Factorial defined for n >= 0.");
        }
        if(n == 0){
            return 1;
        }
        return n * factorial(n - 1);
    }
}
