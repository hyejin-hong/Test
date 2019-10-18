#ifndef _COMMLIB_CFW_H_
#define _COMMLIB_CFW_H_
#include <functional>
#include <memory>
#include <map>
#include <vector>
#include "Header.h"
#include "CommType.h"

namespace AUTONOMOUS
{
    namespace COMMLIB
    {
      
        class CommObject;
        class RcvWrapper
        {
            public:
            virtual void rcv(uint8_t* buff) = 0;
        };

        template <typename M>
        class RcvWrapperT : public RcvWrapper
        {
            typedef typename std::remove_reference<typename std::remove_const<M>::type>::type MSGTYPE;
            public:
            std::function<void (M ) > FNC;

            void rcv(uint8_t* buff)
            {
                MSGTYPE msg;
                msg.setFrame(buff);
                FNC(msg);
            }

        };

        class CFW
        {
            public:
            static CFW& GetInstance();
            
            template <typename T, typename M> void RegisterSub(const uint32_t id, void (T::*fn)(M ), T* obj)
            {
                std::shared_ptr<RcvWrapperT<M> > rw = std::make_shared<RcvWrapperT<M>>();
                rw->FNC = std::bind(fn, obj, std::placeholders::_1);
                m_subMsgFncs[id] = rw;
            }
            
            template <typename M> void RegisterSub(const uint32_t id, void (*fn)(M ))
            {
                std::shared_ptr<RcvWrapperT<M> > rw = std::make_shared<RcvWrapperT<M>>();
                rw->FNC = std::bind(fn, std::placeholders::_1);
                m_subMsgFncs[id] = rw;
            }

            void Send(Header& msg, std::string ip_tag="IP1");
            void Initialize();
            void StartCFW();

            private:
            CFW() {}
            void openTCPListenPort();
            void openUDPListenPort();
            void openPIPEListenPort();
            void startAcceptReceive(int svrfd);
            void innerSend(Header& msg, const uint16_t compid, COMMTYPE type, std::string& ipTag);
            void innerCallBack(uint8_t* buff);
            void readConfig();
            inline bool isInSameNode(std::string _peer_ipport, std::string _peer_iptag);

            struct nodeInfo
            {
                std::string name;
                uint16_t id;
                std::map<std::string, std::string > IPs;
            };
            nodeInfo m_myNodeInfo;
            std::vector<nodeInfo> m_peerNodeInfo;

            struct pubMsgInfo
            {
                uint16_t msgID;
                COMMTYPE type;
                // key: compId, value: IPtag
                std::map<uint16_t, std::string > subscribers;
            };

            std::vector<pubMsgInfo> m_pubMsgInfo;

            std::map<uint32_t, std::shared_ptr<RcvWrapper > > m_subMsgFncs;
            
            std::map<uint16_t, // comp id
                     std::vector<std::shared_ptr<CommObject> > > m_commObjects;
        };
    }
}

#endif