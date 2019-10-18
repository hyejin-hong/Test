#ifndef __COMMLIB_HEADER_H_
#define __COMMLIB_HEADER_H_
#include <cstring>
#include <vector>
#include <stdint.h>
#include <algorithm>
namespace AUTONOMOUS
{
    namespace COMMLIB
    {
        struct Header
        {
            // system heder part
            uint8_t message_type;
            uint8_t message_ID;  // required
            uint8_t sourUnit;
            uint8_t destUnit;
            uint8_t SN;
            uint8_t QoS;
            uint16_t length_dummy;

            // autonomous header part
            uint16_t sourComp;
            uint16_t destComp;
            uint32_t length; // required

            void setFrame(uint8_t* buff);
            void getFrame(std::vector<uint8_t>& buff);

            private:
            virtual void setFrameData(uint8_t* buff)=0;// {};
            virtual void getFrameData(std::vector<uint8_t>& buff)=0;

            protected:
            template<typename T> void serializeData(std::vector<uint8_t>& buff, T data)
            {
                uint8_t tmp[sizeof(T)];
                memcpy(tmp, &data, sizeof(T));               
                std::for_each(tmp, tmp+sizeof(T),
                [&buff](uint8_t d){
                    buff.push_back(d);
                });
            };

            template<typename T> uint8_t* deserializeData(uint8_t* buff, T& data)
            {
                memcpy(&data, buff, sizeof(T));
                return buff + sizeof(T);
            }
        };
    }
}





#endif