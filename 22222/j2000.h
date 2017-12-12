#ifndef J2000_H
#define J2000_H
#include "emath.h"

class j2000
{
public:
    j2000();
    static void IS_GS(int    NG,          /* I: year */
               int    NM,          /* I: month */
               int    N,           /* I: day */
               int JT,          /* I: hour */
               int JM,          /* I: minute */
               int JC,          /* I: second */
               matrixd *M_SNP);
};

#endif // J2000_H
