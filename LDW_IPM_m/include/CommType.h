#ifndef _COMMLIB_COMMTYPE_H_
#define _COMMLIB_COMMTYPE_H_

namespace AUTONOMOUS
{
    namespace COMMLIB
    {
        enum COMMTYPE
        {
            TCP = 0,
            UDP = 1,
            PIPE = 2 // do not use this type, only for CFW
        };
    }
}
#endif